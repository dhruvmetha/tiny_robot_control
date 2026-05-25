"""Compare a push-calibration tree's real and sim records leaf-by-leaf.

Tree layout (what ``execute_real_push.py`` + ``execute_sim_push.py``
produce when invoked per (edge, depth) leaf):

    <root>/                              e.g. push_calibration/obj_1/
    ├── edge<E>/
    │   └── depth<D>/
    │       ├── spec.yaml
    │       ├── trial1/
    │       │   ├── pushes.jsonl         one push record
    │       │   └── (mid_obs.jsonl, wheel_commands.jsonl, etc.)
    │       ├── trial2/
    │       │   └── pushes.jsonl
    │       ├── trial3/
    │       │   └── pushes.jsonl
    │       └── sim/
    │           └── pushes.jsonl         deterministic, single trial
    └── ...

Per leaf, the real side has up to N repetitions (trial1..trialN) and
the sim side has exactly one push (deterministic). The diff computes
the per-trial gap real-vs-sim, averages across the real trials within
each leaf, and rolls up to a headline loss across leaves.

This script reads ``pushes.jsonl`` directly. The derived
``tier2_push_trials/`` bundle is NOT required (it's a convenience
layer for chassis-motion windowing, redundant here — every field a
pose-Δ diff needs is in ``pushes.jsonl``).

Loss (object pose only — robot pose is reported but does not drive
tuning):

    gap_object_xy_cm     = ‖Δobject_pos_real − Δobject_pos_sim‖    (Euclidean cm)
    gap_object_theta_deg = |Δobject_theta_real − Δobject_theta_sim| (deg, wrapped ±180)

    L_leaf  = mean over the leaf's real trials of (gap_object_xy + w · gap_object_theta)
    L_total = mean over leaves of L_leaf

with ``w = 0.5 cm/deg`` by default. Why object-only: NAMO cares about
whether the obstacle ended up where the planner expected. Robot pose
gap is computed and reported for diagnostic value but doesn't enter
the loss.

Outputs (under --out-dir, default ``<root>/diff/``):
    _per_trial.csv      one row per (leaf × real trial), all four gaps
    _per_leaf.csv       one row per leaf, gaps averaged across real trials
    _aggregate.json     per-component stats + headline L + leaves list
    _unmatched.json     leaves missing real or sim data
    comparison.md       human-readable headline + worst-leaves table
    _plots/             optional, matplotlib-gated

Usage:
    python scripts/diff_real_vs_sim.py \\
        --root push_calibration/obj_1 \\
        [--out-dir <dir>] [--theta-weight 0.5]
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ─────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────

DEFAULT_THETA_WEIGHT_CM_PER_DEG = 0.5

# Controller-setting fields that must be byte-identical real-vs-sim. Any
# non-zero gap is a config-drift bug, not a calibration result. Reported
# but excluded from the loss.
CONTROLLER_SETTINGS = (
    "push_controller_max_speed",
    "push_lookahead_distance_cm",
    "push_dynamic_direction",
    "push_path_length_cm",
)

# Leaf path regex: edge<E>/depth<D>
_LEAF_PATH_RE = re.compile(r"edge(\d+)/depth(\d+)$")


# ─────────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class Trial:
    """One push record parsed from a single pushes.jsonl line."""
    source_path: str  # the directory the push came from (trialN/ or sim/)
    source_kind: str  # "real" or "sim"
    trial_label: str  # "trial1" / "trial2" / "trial3" / "sim"
    edge_idx: int
    push_steps: int
    object_id: str
    push_start_ts: float
    delta_object_pos_cm: Tuple[float, float]
    delta_object_theta_deg: float
    delta_robot_pos_cm: Tuple[float, float]
    delta_robot_heading_deg: float
    controller_settings: Dict[str, Any]


@dataclass
class LeafData:
    edge_idx: int
    depth: int
    real_trials: List[Trial] = field(default_factory=list)
    sim_trial: Optional[Trial] = None


@dataclass
class TrialGap:
    """Per-real-trial gap vs the leaf's sim trial."""
    edge_idx: int
    depth: int
    trial_label: str
    gap_object_xy_cm: float
    gap_object_theta_deg: float
    gap_robot_xy_cm: float
    gap_robot_theta_deg: float
    controller_setting_diffs: Dict[str, Optional[float]]


@dataclass
class LeafGap:
    """Per-leaf aggregate gaps (mean across the leaf's real trials)."""
    edge_idx: int
    depth: int
    n_real_trials: int
    gap_object_xy_cm: float
    gap_object_theta_deg: float
    gap_robot_xy_cm: float
    gap_robot_theta_deg: float


@dataclass
class FieldStats:
    n: int
    mean: float
    median: float
    p90: float
    max: float


# ─────────────────────────────────────────────────────────────────────────
# Parsing
# ─────────────────────────────────────────────────────────────────────────


def _wrap_to_180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def _require_field(rec: Dict[str, Any], key: str, where: str) -> Any:
    v = rec.get(key)
    if v is None:
        raise ValueError(
            f"[diff_real_vs_sim] {where}: missing/null field {key!r}. "
            f"Schema mismatch — check the writer (execute_real_push.py / "
            f"execute_sim_push.py / push.py)."
        )
    return v


def _load_first_push_record(jsonl_path: Path) -> Dict[str, Any]:
    """Read the first non-empty JSON line of pushes.jsonl. Per-leaf invariant:
    exactly one push per file (one push per invocation)."""
    with open(jsonl_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            return json.loads(line)
    raise ValueError(f"[diff_real_vs_sim] empty pushes.jsonl: {jsonl_path}")


def _trial_from_push_record(
    rec: Dict[str, Any],
    source_path: Path,
    source_kind: str,
    trial_label: str,
) -> Trial:
    where = f"{source_kind} push at {source_path}"

    edge_idx = int(_require_field(rec, "expected_edge", where))
    push_steps = int(_require_field(rec, "expected_push_steps", where))
    object_id = str(_require_field(rec, "object_id", where))

    delta_pos = _require_field(rec, "delta_pos_cm", where)
    delta_object_pos = (float(delta_pos[0]), float(delta_pos[1]))
    delta_object_theta = float(_require_field(rec, "delta_theta_deg", where))

    # Robot Δ = after − before. Both fields are in the push record (added
    # to push.py in commit e4bee13). Heading delta gets wrapped to ±180.
    rob_before = _require_field(rec, "robot_pose_before_cm_deg", where)
    rob_after = _require_field(rec, "robot_pose_after_cm_deg", where)
    drob_x = float(rob_after[0]) - float(rob_before[0])
    drob_y = float(rob_after[1]) - float(rob_before[1])
    drob_heading = _wrap_to_180(float(rob_after[2]) - float(rob_before[2]))

    push_start_ts = float(_require_field(rec, "push_start_obs_timestamp", where))

    controller_settings = {k: rec.get(k) for k in CONTROLLER_SETTINGS}

    return Trial(
        source_path=str(source_path),
        source_kind=source_kind,
        trial_label=trial_label,
        edge_idx=edge_idx,
        push_steps=push_steps,
        object_id=object_id,
        push_start_ts=push_start_ts,
        delta_object_pos_cm=delta_object_pos,
        delta_object_theta_deg=delta_object_theta,
        delta_robot_pos_cm=(drob_x, drob_y),
        delta_robot_heading_deg=drob_heading,
        controller_settings=controller_settings,
    )


# ─────────────────────────────────────────────────────────────────────────
# Discovery
# ─────────────────────────────────────────────────────────────────────────


def _discover_leaves(root: Path) -> Dict[Tuple[int, int], LeafData]:
    """Walk <root>/edge<E>/depth<D>/ and bin trials and sim runs into LeafData."""
    if not root.exists():
        raise ValueError(f"[diff_real_vs_sim] root not found: {root}")

    leaves: Dict[Tuple[int, int], LeafData] = {}

    for edge_dir in sorted(root.iterdir()):
        if not edge_dir.is_dir():
            continue
        m_edge = re.fullmatch(r"edge(\d+)", edge_dir.name)
        if not m_edge:
            continue
        edge_idx = int(m_edge.group(1))

        for depth_dir in sorted(edge_dir.iterdir()):
            if not depth_dir.is_dir():
                continue
            m_depth = re.fullmatch(r"depth(\d+)", depth_dir.name)
            if not m_depth:
                continue
            depth = int(m_depth.group(1))

            leaf = leaves.setdefault((edge_idx, depth), LeafData(edge_idx, depth))

            for child in sorted(depth_dir.iterdir()):
                if not child.is_dir():
                    continue
                pushes_jsonl = child / "pushes.jsonl"
                if not pushes_jsonl.exists():
                    continue
                # Identify by directory name: trialN → real, sim → sim.
                if re.fullmatch(r"trial\d+", child.name):
                    rec = _load_first_push_record(pushes_jsonl)
                    leaf.real_trials.append(
                        _trial_from_push_record(rec, child, "real", child.name)
                    )
                elif child.name == "sim":
                    rec = _load_first_push_record(pushes_jsonl)
                    if leaf.sim_trial is not None:
                        # Shouldn't happen — sim is deterministic, only one allowed.
                        raise ValueError(
                            f"[diff_real_vs_sim] multiple sim/ dirs under "
                            f"{depth_dir}? second at {child}"
                        )
                    leaf.sim_trial = _trial_from_push_record(rec, child, "sim", "sim")

            leaf.real_trials.sort(key=lambda t: t.trial_label)

    return leaves


# ─────────────────────────────────────────────────────────────────────────
# Gap computation
# ─────────────────────────────────────────────────────────────────────────


def _euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _setting_diff(real_v: Any, sim_v: Any) -> Optional[float]:
    if real_v is None or sim_v is None:
        return None
    try:
        return float(sim_v) - float(real_v)
    except (TypeError, ValueError):
        return None


def _trial_gap(real: Trial, sim: Trial) -> TrialGap:
    return TrialGap(
        edge_idx=real.edge_idx,
        depth=real.push_steps - 1,
        trial_label=real.trial_label,
        gap_object_xy_cm=_euclidean(real.delta_object_pos_cm, sim.delta_object_pos_cm),
        gap_object_theta_deg=abs(
            _wrap_to_180(real.delta_object_theta_deg - sim.delta_object_theta_deg)
        ),
        gap_robot_xy_cm=_euclidean(real.delta_robot_pos_cm, sim.delta_robot_pos_cm),
        gap_robot_theta_deg=abs(
            _wrap_to_180(real.delta_robot_heading_deg - sim.delta_robot_heading_deg)
        ),
        controller_setting_diffs={
            k: _setting_diff(real.controller_settings[k], sim.controller_settings[k])
            for k in CONTROLLER_SETTINGS
        },
    )


def _leaf_gap(leaf: LeafData, trial_gaps: List[TrialGap]) -> LeafGap:
    """Average per-trial gaps within a leaf."""
    n = len(trial_gaps)
    return LeafGap(
        edge_idx=leaf.edge_idx,
        depth=leaf.depth,
        n_real_trials=n,
        gap_object_xy_cm=sum(g.gap_object_xy_cm for g in trial_gaps) / n,
        gap_object_theta_deg=sum(g.gap_object_theta_deg for g in trial_gaps) / n,
        gap_robot_xy_cm=sum(g.gap_robot_xy_cm for g in trial_gaps) / n,
        gap_robot_theta_deg=sum(g.gap_robot_theta_deg for g in trial_gaps) / n,
    )


# ─────────────────────────────────────────────────────────────────────────
# Aggregation
# ─────────────────────────────────────────────────────────────────────────


def _stats(values: List[float]) -> FieldStats:
    if not values:
        return FieldStats(n=0, mean=0.0, median=0.0, p90=0.0, max=0.0)
    sv = sorted(values)
    n = len(sv)
    mean = sum(sv) / n
    median = sv[n // 2] if n % 2 == 1 else 0.5 * (sv[n // 2 - 1] + sv[n // 2])
    p90 = sv[max(0, min(n - 1, int(round(0.9 * (n - 1)))))]
    return FieldStats(n=n, mean=mean, median=median, p90=p90, max=sv[-1])


def _aggregate(leaf_gaps: List[LeafGap], theta_weight: float) -> Dict[str, Any]:
    stats_map = {
        "gap_object_xy_cm": _stats([g.gap_object_xy_cm for g in leaf_gaps]),
        "gap_object_theta_deg": _stats([g.gap_object_theta_deg for g in leaf_gaps]),
        "gap_robot_xy_cm": _stats([g.gap_robot_xy_cm for g in leaf_gaps]),
        "gap_robot_theta_deg": _stats([g.gap_robot_theta_deg for g in leaf_gaps]),
    }
    L_components = {
        "object_xy_cm": stats_map["gap_object_xy_cm"].mean,
        "object_theta_weighted_cm": theta_weight * stats_map["gap_object_theta_deg"].mean,
    }
    L = sum(L_components.values())
    return {
        "n_leaves_with_both_sides": len(leaf_gaps),
        "theta_weight_cm_per_deg": theta_weight,
        "headline_loss_cm": L,
        "loss_components_cm": L_components,
        "field_stats": {
            k: {"n": v.n, "mean": v.mean, "median": v.median,
                "p90": v.p90, "max": v.max}
            for k, v in stats_map.items()
        },
        "leaves": [
            {"edge_idx": g.edge_idx, "depth": g.depth,
             "n_real_trials": g.n_real_trials,
             "gap_object_xy_cm": g.gap_object_xy_cm,
             "gap_object_theta_deg": g.gap_object_theta_deg,
             "gap_robot_xy_cm": g.gap_robot_xy_cm,
             "gap_robot_theta_deg": g.gap_robot_theta_deg}
            for g in leaf_gaps
        ],
    }


# ─────────────────────────────────────────────────────────────────────────
# Writers
# ─────────────────────────────────────────────────────────────────────────


def write_per_trial_csv(path: Path, trial_gaps: List[TrialGap]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "edge_idx", "depth", "trial_label",
            "gap_object_xy_cm", "gap_object_theta_deg",
            "gap_robot_xy_cm", "gap_robot_theta_deg",
            *[f"controller_setting_diff_{k}" for k in CONTROLLER_SETTINGS],
        ])
        for g in trial_gaps:
            w.writerow([
                g.edge_idx, g.depth, g.trial_label,
                f"{g.gap_object_xy_cm:.4f}", f"{g.gap_object_theta_deg:.4f}",
                f"{g.gap_robot_xy_cm:.4f}", f"{g.gap_robot_theta_deg:.4f}",
                *[g.controller_setting_diffs.get(k) for k in CONTROLLER_SETTINGS],
            ])


def write_per_leaf_csv(path: Path, leaf_gaps: List[LeafGap]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "edge_idx", "depth", "n_real_trials",
            "gap_object_xy_cm", "gap_object_theta_deg",
            "gap_robot_xy_cm", "gap_robot_theta_deg",
        ])
        for g in leaf_gaps:
            w.writerow([
                g.edge_idx, g.depth, g.n_real_trials,
                f"{g.gap_object_xy_cm:.4f}", f"{g.gap_object_theta_deg:.4f}",
                f"{g.gap_robot_xy_cm:.4f}", f"{g.gap_robot_theta_deg:.4f}",
            ])


def write_aggregate_json(path: Path, agg: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(agg, f, indent=2)


def write_unmatched_json(
    path: Path,
    leaves: Dict[Tuple[int, int], LeafData],
) -> None:
    real_only = []
    sim_only = []
    empty = []
    for (e, d), leaf in sorted(leaves.items()):
        has_real = bool(leaf.real_trials)
        has_sim = leaf.sim_trial is not None
        if has_real and not has_sim:
            real_only.append({"edge_idx": e, "depth": d,
                              "n_real_trials": len(leaf.real_trials)})
        elif has_sim and not has_real:
            sim_only.append({"edge_idx": e, "depth": d})
        elif not has_real and not has_sim:
            empty.append({"edge_idx": e, "depth": d})
    payload = {"real_only": real_only, "sim_only": sim_only, "empty": empty}
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


def write_comparison_md(
    path: Path,
    agg: Dict[str, Any],
    leaf_gaps: List[LeafGap],
    trial_gaps: List[TrialGap],
    leaves: Dict[Tuple[int, int], LeafData],
) -> None:
    n = agg["n_leaves_with_both_sides"]
    L = agg["headline_loss_cm"]
    w = agg["theta_weight_cm_per_deg"]
    c = agg["loss_components_cm"]
    s = agg["field_stats"]

    lines: List[str] = []
    lines.append("# Real vs Sim — push calibration diff (tree walk)\n")
    lines.append(
        f"**Headline loss `L = {L:.3f} cm`**  "
        f"(w_theta = {w} cm/deg, n_leaves = {n})\n"
    )
    lines.append("## Loss breakdown\n")
    lines.append("Loss is object pose only. Robot pose gap is reported "
                 "below for diagnostic value but does not enter the headline.\n")
    lines.append("| Component | cm |")
    lines.append("|---|---:|")
    lines.append(f"| object xy           | {c['object_xy_cm']:.3f} |")
    lines.append(f"| object θ (weighted) | {c['object_theta_weighted_cm']:.3f} |")
    lines.append("")

    lines.append("## Per-field statistics (over leaves)\n")
    lines.append("| Field | n | mean | median | p90 | max |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for k in (
        "gap_object_xy_cm",
        "gap_object_theta_deg",
        "gap_robot_xy_cm",
        "gap_robot_theta_deg",
    ):
        v = s[k]
        lines.append(
            f"| {k} | {v['n']} | {v['mean']:.3f} | {v['median']:.3f} | "
            f"{v['p90']:.3f} | {v['max']:.3f} |"
        )
    lines.append("")

    if leaf_gaps:
        lines.append("## Per-leaf gaps (real trials averaged)\n")
        lines.append("| edge | depth | n_real | obj xy (cm) | obj θ (°) | "
                     "rob xy (cm) | rob θ (°) |")
        lines.append("|---:|---:|---:|---:|---:|---:|---:|")
        for g in sorted(leaf_gaps, key=lambda g: (g.edge_idx, g.depth)):
            lines.append(
                f"| {g.edge_idx} | {g.depth} | {g.n_real_trials} | "
                f"{g.gap_object_xy_cm:.3f} | {g.gap_object_theta_deg:.2f} | "
                f"{g.gap_robot_xy_cm:.3f} | {g.gap_robot_theta_deg:.2f} |"
            )
        lines.append("")

        lines.append("## Worst-5 leaves by object xy + θ gap\n")
        worst = sorted(
            leaf_gaps,
            key=lambda g: g.gap_object_xy_cm + w * g.gap_object_theta_deg,
            reverse=True,
        )[:5]
        lines.append("| edge | depth | obj xy (cm) | obj θ (°) | "
                     "rob xy (cm) | rob θ (°) |")
        lines.append("|---:|---:|---:|---:|---:|---:|")
        for g in worst:
            lines.append(
                f"| {g.edge_idx} | {g.depth} | "
                f"{g.gap_object_xy_cm:.3f} | {g.gap_object_theta_deg:.2f} | "
                f"{g.gap_robot_xy_cm:.3f} | {g.gap_robot_theta_deg:.2f} |"
            )
        lines.append("")

    # Controller-setting sanity
    nonzero = []
    for g in trial_gaps:
        for k, v in g.controller_setting_diffs.items():
            if v is not None and abs(v) > 1e-6:
                nonzero.append((g.edge_idx, g.depth, g.trial_label, k, v))
    if nonzero:
        lines.append("## ⚠ Controller-setting drift (expected zero)\n")
        lines.append("Non-zero values mean real and sim ran with different "
                     "controller settings. Fix the config drift before trusting "
                     "the calibration numbers above.\n")
        lines.append("| edge | depth | trial | setting | sim − real |")
        lines.append("|---:|---:|---|---|---:|")
        for e, d, t, k, v in nonzero[:20]:
            lines.append(f"| {e} | {d} | {t} | {k} | {v:+.4f} |")
        lines.append("")
    elif trial_gaps:
        lines.append("## Controller-setting sanity\n")
        lines.append("All controller settings match across real and sim. ✓\n")

    # Unmatched leaves
    unmatched_real = sorted([
        (e, d) for (e, d), leaf in leaves.items()
        if leaf.real_trials and leaf.sim_trial is None
    ])
    unmatched_sim = sorted([
        (e, d) for (e, d), leaf in leaves.items()
        if leaf.sim_trial is not None and not leaf.real_trials
    ])
    if unmatched_real or unmatched_sim:
        lines.append("## Unmatched leaves\n")
        if unmatched_real:
            lines.append(f"- **{len(unmatched_real)} real-only** "
                         f"(have real trials, no sim): " +
                         ", ".join(f"edge{e}/depth{d}" for e, d in unmatched_real))
        if unmatched_sim:
            lines.append(f"- **{len(unmatched_sim)} sim-only** "
                         f"(have sim, no real trials yet): " +
                         ", ".join(f"edge{e}/depth{d}" for e, d in unmatched_sim))
        lines.append("\nDetail in `_unmatched.json`.\n")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def write_plots(plot_dir: Path, trial_gaps: List[TrialGap], leaves: Dict[Tuple[int, int], LeafData]) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    # Per-real-trial scatter: real Δ vs sim Δ for each gap component
    real_vals = {"obj_dx": [], "obj_dy": [], "obj_dth": [],
                 "rob_dx": [], "rob_dy": [], "rob_dh": []}
    sim_vals = dict((k, []) for k in real_vals)
    for (_e, _d), leaf in leaves.items():
        if not leaf.real_trials or leaf.sim_trial is None:
            continue
        s = leaf.sim_trial
        for r in leaf.real_trials:
            real_vals["obj_dx"].append(r.delta_object_pos_cm[0])
            sim_vals["obj_dx"].append(s.delta_object_pos_cm[0])
            real_vals["obj_dy"].append(r.delta_object_pos_cm[1])
            sim_vals["obj_dy"].append(s.delta_object_pos_cm[1])
            real_vals["obj_dth"].append(r.delta_object_theta_deg)
            sim_vals["obj_dth"].append(s.delta_object_theta_deg)
            real_vals["rob_dx"].append(r.delta_robot_pos_cm[0])
            sim_vals["rob_dx"].append(s.delta_robot_pos_cm[0])
            real_vals["rob_dy"].append(r.delta_robot_pos_cm[1])
            sim_vals["rob_dy"].append(s.delta_robot_pos_cm[1])
            real_vals["rob_dh"].append(r.delta_robot_heading_deg)
            sim_vals["rob_dh"].append(s.delta_robot_heading_deg)

    if not real_vals["obj_dx"]:
        return False

    def _scatter(ax, rs, ss, title, units):
        ax.scatter(rs, ss, alpha=0.6)
        lo = min(min(rs), min(ss))
        hi = max(max(rs), max(ss))
        if hi == lo:
            hi = lo + 1e-3
        ax.plot([lo, hi], [lo, hi], "k--", alpha=0.4, label="y=x")
        ax.set_xlabel(f"real ({units})")
        ax.set_ylabel(f"sim ({units})")
        ax.set_title(title)
        ax.legend()
        ax.set_aspect("equal", adjustable="datalim")

    plot_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    _scatter(axes[0], real_vals["obj_dx"], sim_vals["obj_dx"], "object Δx", "cm")
    _scatter(axes[1], real_vals["obj_dy"], sim_vals["obj_dy"], "object Δy", "cm")
    _scatter(axes[2], real_vals["obj_dth"], sim_vals["obj_dth"], "object Δθ", "deg")
    fig.tight_layout()
    fig.savefig(plot_dir / "object_pose.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    _scatter(axes[0], real_vals["rob_dx"], sim_vals["rob_dx"], "robot Δx", "cm")
    _scatter(axes[1], real_vals["rob_dy"], sim_vals["rob_dy"], "robot Δy", "cm")
    _scatter(axes[2], real_vals["rob_dh"], sim_vals["rob_dh"], "robot Δheading", "deg")
    fig.tight_layout()
    fig.savefig(plot_dir / "robot_pose.png", dpi=150)
    plt.close(fig)
    return True


# ─────────────────────────────────────────────────────────────────────────
# Orchestration
# ─────────────────────────────────────────────────────────────────────────


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", required=True,
                        help="push-calibration object root, e.g. push_calibration/obj_1")
    parser.add_argument("--out-dir", default=None,
                        help="output dir (default: <root>/diff/)")
    parser.add_argument("--theta-weight", type=float,
                        default=DEFAULT_THETA_WEIGHT_CM_PER_DEG,
                        help=f"cm-per-deg weight for combining xy + theta gaps "
                             f"(default {DEFAULT_THETA_WEIGHT_CM_PER_DEG})")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else root / "diff"

    print(f"[diff_real_vs_sim] root: {root}")
    print(f"[diff_real_vs_sim]  out: {out_dir}")

    leaves = _discover_leaves(root)
    print(f"[diff_real_vs_sim] discovered {len(leaves)} leaves total")

    # Compute per-trial and per-leaf gaps over leaves with both real + sim.
    trial_gaps: List[TrialGap] = []
    leaf_gaps: List[LeafGap] = []
    for (e, d), leaf in sorted(leaves.items()):
        if not leaf.real_trials or leaf.sim_trial is None:
            continue
        this_leaf_trial_gaps = [_trial_gap(r, leaf.sim_trial) for r in leaf.real_trials]
        trial_gaps.extend(this_leaf_trial_gaps)
        leaf_gaps.append(_leaf_gap(leaf, this_leaf_trial_gaps))

    paired_leaves = len(leaf_gaps)
    print(f"[diff_real_vs_sim] paired {paired_leaves} leaves "
          f"({len(trial_gaps)} real trials across them)")
    if paired_leaves == 0:
        print("[diff_real_vs_sim] no leaves have both real + sim — nothing to diff",
              file=sys.stderr)
        # Still write the unmatched report so the user can see why.
        write_unmatched_json(out_dir / "_unmatched.json", leaves)
        return 1

    agg = _aggregate(leaf_gaps, args.theta_weight)
    print(f"[diff_real_vs_sim] headline L = {agg['headline_loss_cm']:.3f} cm")

    write_per_trial_csv(out_dir / "_per_trial.csv", trial_gaps)
    write_per_leaf_csv(out_dir / "_per_leaf.csv", leaf_gaps)
    write_aggregate_json(out_dir / "_aggregate.json", agg)
    write_unmatched_json(out_dir / "_unmatched.json", leaves)
    write_comparison_md(out_dir / "comparison.md", agg, leaf_gaps, trial_gaps, leaves)
    plotted = write_plots(out_dir / "_plots", trial_gaps, leaves)
    if not plotted:
        print("[diff_real_vs_sim] matplotlib unavailable or no data — skipped plots")

    print(f"[diff_real_vs_sim] wrote outputs under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
