"""Shared types and helpers for chassis-motion calibration trials.

A chassis-motion trial drives the robot (real or sim) at a fixed wheel
command for N seconds and logs per-frame pose samples. This module holds
the on-disk schema, the I/O helpers, and the linear-fit functions used to
turn a per-frame log into a single velocity number.

Both the collection scripts (``scripts/collect_chassis_motion_*.py``) and
the analysis script (``scripts/analyze_chassis_motion.py``) import from
here. The module is pure — no I/O at import time, no global state.

Trial directory layout:

    <trial_dir>/
    ├── poses.jsonl       one record per pose sample
    └── trial_meta.json   per-trial metadata
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Tuple

import numpy as np


# ─────────────────────────────────────────────────────────────────────────
# On-disk schema
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class TrialMeta:
    """Metadata for one chassis-motion trial. One per trial directory."""

    trial_kind: Literal["straight", "turn", "arc"]
    source: Literal["real", "sim"]
    left_cmd: float
    right_cmd: float
    cmd_units: str                       # "pwm" (real) or "fraction" (sim)
    duration_s: float
    started_at_utc_iso: str
    n_samples: int
    # Source-specific (exactly one of these is populated)
    real_meta: Optional[Dict[str, Any]] = None
    sim_meta: Optional[Dict[str, Any]] = None


@dataclass
class PoseSample:
    """One pose observation. t_s is seconds since the trial t0."""

    t_s: float
    x_cm: float
    y_cm: float
    theta_deg: float


# ─────────────────────────────────────────────────────────────────────────
# Trial directory naming
# ─────────────────────────────────────────────────────────────────────────


def trial_name(
    trial_kind: Literal["straight", "turn", "arc"],
    cmd_magnitude: float,
    direction: Optional[str] = None,
) -> str:
    """Canonical directory name for a trial.

    Encodes the command magnitude as ``cmd0p20`` (= 0.20), avoiding decimal
    points in filesystem paths. For "turn" trials, append a direction
    suffix (``_ccw`` or ``_cw``) so opposite-rotation runs don't collide.

    Examples:
        trial_name("straight", 0.20)               → "straight_cmd0p20"
        trial_name("turn", 0.20, direction="ccw")  → "turn_cmd0p20_ccw"
        trial_name("arc", 0.15)                    → "arc_cmd0p15"
    """
    mag_str = f"cmd{cmd_magnitude:0.2f}".replace(".", "p")
    base = f"{trial_kind}_{mag_str}"
    if direction:
        return f"{base}_{direction}"
    return base


def infer_trial_kind(left_cmd: float, right_cmd: float) -> str:
    """Classify a wheel-command pair as straight / turn / arc.

    - "straight": both wheels same direction, same magnitude (within 1e-6).
    - "turn":     opposite directions, equal magnitude (in-place rotation).
    - "arc":      anything else.
    """
    if abs(left_cmd - right_cmd) < 1e-6:
        return "straight"
    if abs(left_cmd + right_cmd) < 1e-6 and abs(left_cmd) > 1e-6:
        return "turn"
    return "arc"


def turn_direction(left_cmd: float, right_cmd: float) -> Optional[str]:
    """For a turn trial, return 'ccw' if the robot spins counter-clockwise
    (right wheel forward, left wheel backward), 'cw' for the opposite.
    Returns None if the input isn't a turn.
    """
    if abs(left_cmd + right_cmd) > 1e-6:
        return None
    if right_cmd > left_cmd:
        return "ccw"
    if right_cmd < left_cmd:
        return "cw"
    return None


# ─────────────────────────────────────────────────────────────────────────
# Atomic file I/O
# ─────────────────────────────────────────────────────────────────────────


def write_trial(
    out_dir: Path,
    meta: TrialMeta,
    poses: List[PoseSample],
) -> None:
    """Write ``poses.jsonl`` and ``trial_meta.json`` atomically.

    Writes to ``.tmp_<name>`` first, then ``os.replace`` to the final name.
    If the process is killed mid-write, no half-written file is observable
    to the analyzer.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    # Write poses.jsonl
    poses_final = out_dir / "poses.jsonl"
    poses_tmp = out_dir / ".tmp_poses.jsonl"
    with open(poses_tmp, "w") as f:
        for p in poses:
            f.write(json.dumps(asdict(p)) + "\n")
    os.replace(poses_tmp, poses_final)

    # Write trial_meta.json
    meta_final = out_dir / "trial_meta.json"
    meta_tmp = out_dir / ".tmp_trial_meta.json"
    with open(meta_tmp, "w") as f:
        json.dump(asdict(meta), f, indent=2)
    os.replace(meta_tmp, meta_final)


def read_trial(trial_dir: Path) -> Tuple[TrialMeta, List[PoseSample]]:
    """Read ``poses.jsonl`` + ``trial_meta.json`` from a trial directory.

    Raises FileNotFoundError if either file is missing.
    """
    meta_path = trial_dir / "trial_meta.json"
    poses_path = trial_dir / "poses.jsonl"
    meta_dict = json.loads(meta_path.read_text())
    meta = TrialMeta(**meta_dict)
    poses: List[PoseSample] = []
    with open(poses_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            poses.append(PoseSample(**json.loads(line)))
    return meta, poses


# ─────────────────────────────────────────────────────────────────────────
# Linear fits — pure, no I/O
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class StraightFit:
    v_linear_cm_s: float                 # magnitude of (vx, vy)
    vx_cm_s: float
    vy_cm_s: float
    n_steady_samples: int
    steady_window_s: float
    fit_residual_rms_cm: float
    se_v_cm_s: float                     # propagated standard error of v_linear


@dataclass
class TurnFit:
    omega_deg_s: float
    n_steady_samples: int
    steady_window_s: float
    fit_residual_rms_deg: float
    se_omega_deg_s: float


def _steady_slice(
    poses: List[PoseSample], settle_s: float
) -> List[PoseSample]:
    """Return poses with t_s relative to the first sample exceeding settle_s.

    Skips the acceleration phase. Caller checks that the result is non-empty.
    """
    if not poses:
        return []
    t0 = poses[0].t_s
    return [p for p in poses if (p.t_s - t0) > settle_s]


def fit_straight(
    poses: List[PoseSample], settle_s: float = 2.0, min_samples: int = 5
) -> StraightFit:
    """Linear fit of x(t) and y(t) over the steady-state window.

    Skips the first ``settle_s`` of motion to exclude the acceleration
    phase. Returns velocity magnitude in cm/s plus standard-error estimates
    derived from the residuals.

    Raises ValueError if fewer than ``min_samples`` lie in the window.
    """
    steady = _steady_slice(poses, settle_s)
    if len(steady) < min_samples:
        raise ValueError(
            f"fit_straight: only {len(steady)} samples in the steady-state "
            f"window (settle_s={settle_s}); need >= {min_samples}."
        )
    ts = np.array([p.t_s for p in steady], dtype=float)
    xs = np.array([p.x_cm for p in steady], dtype=float)
    ys = np.array([p.y_cm for p in steady], dtype=float)

    vx, _, se_vx, rms_x = _linfit_slope(ts, xs)
    vy, _, se_vy, rms_y = _linfit_slope(ts, ys)

    v_lin = float(math.hypot(vx, vy))
    # Propagate SE of vx, vy to SE of magnitude:
    #   v = sqrt(vx² + vy²)
    #   ∂v/∂vx = vx/v;  ∂v/∂vy = vy/v
    #   var(v) ≈ (vx/v)²·var(vx) + (vy/v)²·var(vy)
    if v_lin > 1e-9:
        se_v = math.hypot((vx / v_lin) * se_vx, (vy / v_lin) * se_vy)
    else:
        se_v = math.hypot(se_vx, se_vy)
    rms = math.hypot(rms_x, rms_y)

    return StraightFit(
        v_linear_cm_s=v_lin,
        vx_cm_s=float(vx),
        vy_cm_s=float(vy),
        n_steady_samples=len(steady),
        steady_window_s=float(ts[-1] - ts[0]),
        fit_residual_rms_cm=float(rms),
        se_v_cm_s=float(se_v),
    )


def fit_turn(
    poses: List[PoseSample], settle_s: float = 2.0, min_samples: int = 5
) -> TurnFit:
    """Linear fit of theta(t) over the steady-state window.

    Unwraps theta first so a 359° → 1° wrap doesn't look like -358° of
    motion. Returns omega in deg/s.

    Raises ValueError if fewer than ``min_samples`` lie in the window.
    """
    steady = _steady_slice(poses, settle_s)
    if len(steady) < min_samples:
        raise ValueError(
            f"fit_turn: only {len(steady)} samples in the steady-state "
            f"window (settle_s={settle_s}); need >= {min_samples}."
        )
    ts = np.array([p.t_s for p in steady], dtype=float)
    theta_deg = np.array([p.theta_deg for p in steady], dtype=float)
    # Unwrap in degrees: convert to rad, np.unwrap, convert back.
    theta_unwrapped = np.degrees(np.unwrap(np.radians(theta_deg)))

    omega, _, se_omega, rms = _linfit_slope(ts, theta_unwrapped)

    return TurnFit(
        omega_deg_s=float(omega),
        n_steady_samples=len(steady),
        steady_window_s=float(ts[-1] - ts[0]),
        fit_residual_rms_deg=float(rms),
        se_omega_deg_s=float(se_omega),
    )


def _linfit_slope(
    t: np.ndarray, y: np.ndarray
) -> Tuple[float, float, float, float]:
    """Least-squares linear fit y = slope * t + intercept.

    Returns (slope, intercept, se_slope, residual_rms). se_slope is the
    standard error of the slope estimate from the residuals (the classic
    formula  s² = Σr² / (n - 2); se(slope)² = s² / Σ(t - t̄)²).
    """
    n = len(t)
    if n < 2:
        raise ValueError(f"_linfit_slope needs >= 2 points, got {n}")
    slope, intercept = np.polyfit(t, y, 1)
    residuals = y - (slope * t + intercept)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    if n > 2:
        s2 = float(np.sum(residuals ** 2) / (n - 2))
        t_var = float(np.sum((t - t.mean()) ** 2))
        se_slope = math.sqrt(s2 / t_var) if t_var > 0 else float("inf")
    else:
        se_slope = float("inf")
    return float(slope), float(intercept), float(se_slope), rms


# ─────────────────────────────────────────────────────────────────────────
# Per-frame helpers — used by the collectors
# ─────────────────────────────────────────────────────────────────────────


def pose_dedupe(poses: List[PoseSample], min_dt_s: float = 1e-4) -> List[PoseSample]:
    """Drop samples whose timestamp is within ``min_dt_s`` of the previous.

    The real collector polls the camera service's cache; if it polls
    faster than the publish rate it sees the same observation twice. We
    dedupe by inter-sample Δt so the regression isn't biased by repeated
    points.
    """
    if not poses:
        return []
    kept = [poses[0]]
    for p in poses[1:]:
        if (p.t_s - kept[-1].t_s) >= min_dt_s:
            kept.append(p)
    return kept


# ─────────────────────────────────────────────────────────────────────────
# Pure-pursuit trial helpers — used by the PP calibration scripts
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class CommandSample:
    """One commanded action per controller tick."""

    t_s: float
    left_cmd: float
    right_cmd: float
    mode: str = ""  # FollowPathController metadata.mode (TRACK / ALIGN / ACQUIRE)


@dataclass
class PpCruiseResult:
    """Per-trial PP metrics computed from the pose log."""

    forward_cruise_cm_s: float          # primary metric
    n_cruise_samples: int
    cruise_window_s: float
    arc_length_cm: float                # total forward progress reached
    goal_reached: bool
    time_to_goal_s: Optional[float]


def cruise_forward_velocity(
    poses: List[PoseSample],
    path_start_cm: Tuple[float, float],
    path_unit_vec: Tuple[float, float],
    window_frac: Tuple[float, float] = (0.2, 0.8),
) -> PpCruiseResult:
    """Compute forward cruise velocity along the path direction.

    Forward progress at time t = ((pos − start) · û). Lateral motion does
    NOT count — only progress along the path direction. This makes the
    metric insensitive to PP wiggle corrections.

    Cruise window = the time during which forward progress is in the
    middle window_frac of total progress. Skips accel + arrival.
    """
    if not poses:
        raise ValueError("cruise_forward_velocity: no poses")
    sx, sy = path_start_cm
    ux, uy = path_unit_vec
    # Defensive: sort by t_s so any out-of-order frames (rare but seen
    # when the camera service buffers stale frames) don't produce
    # negative dt in the slope downstream. Stable sort is fine because
    # pose_dedupe (called by the collector) already dropped exact dups.
    poses_sorted = sorted(poses, key=lambda p: p.t_s)
    ts = [float(p.t_s) for p in poses_sorted]
    s = [(p.x_cm - sx) * ux + (p.y_cm - sy) * uy for p in poses_sorted]
    s_end = s[-1]
    if s_end <= 0:
        raise ValueError(
            f"cruise_forward_velocity: no forward progress (s_end={s_end:.2f} cm)"
        )
    s_lo = window_frac[0] * s_end
    s_hi = window_frac[1] * s_end
    cruise_idx = [i for i, sv in enumerate(s) if s_lo <= sv <= s_hi]
    if len(cruise_idx) < 2:
        raise ValueError(
            f"cruise_forward_velocity: <2 samples in cruise window "
            f"([{s_lo:.1f}, {s_hi:.1f}] cm; s_end={s_end:.1f}; n_samples={len(poses)})"
        )
    i0, i1 = cruise_idx[0], cruise_idx[-1]
    dt = ts[i1] - ts[i0]
    ds = s[i1] - s[i0]
    if dt <= 0:
        raise ValueError("cruise_forward_velocity: cruise window has zero duration")
    return PpCruiseResult(
        forward_cruise_cm_s=ds / dt,
        n_cruise_samples=len(cruise_idx),
        cruise_window_s=dt,
        arc_length_cm=s_end,
        goal_reached=False,             # caller fills
        time_to_goal_s=None,            # caller fills
    )


def median_with_mad_se(values: List[float]) -> Tuple[float, float]:
    """Return (median, standard_error_of_median) using MAD-based estimate.

    SE(median) ≈ 1.2533 · σ̂ / √N where σ̂ ≈ 1.4826 · MAD (normal-distribution
    consistent estimator). For N=5 this comes out to roughly 0.83 · MAD.
    """
    if not values:
        raise ValueError("median_with_mad_se: empty input")
    a = np.array(values, dtype=float)
    n = len(a)
    med = float(np.median(a))
    mad = float(np.median(np.abs(a - med)))
    sigma_hat = 1.4826 * mad
    se = 1.2533 * sigma_hat / math.sqrt(n) if n > 0 else 0.0
    return med, float(se)


def write_command_log(out_path: Path, commands: List[CommandSample]) -> None:
    """Write a list of CommandSample to a jsonl file, atomic via .tmp."""
    tmp = out_path.with_name(".tmp_" + out_path.name)
    with open(tmp, "w") as f:
        for c in commands:
            f.write(json.dumps(asdict(c)) + "\n")
    os.replace(tmp, out_path)


def quat_z_to_theta_deg(qw: float, qz: float) -> float:
    """Convert MuJoCo quaternion (Z-axis rotation) to theta in degrees.

    Assumes qx ≈ qy ≈ 0 (chassis stays flat on the floor). Returns a
    value in (-180, 180].
    """
    return math.degrees(2.0 * math.atan2(qz, qw))
