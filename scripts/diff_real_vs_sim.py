"""Compare an execute_real_push session against matched execute_sim_push runs.

Reads one real run dir and one parent dir of sim runs (one sim run per push).
Joins trials on ``(object_id, expected_edge, expected_push_steps)``, with
``push_start_obs_timestamp`` order as the tiebreaker for duplicate triples.
For each matched pair, computes four pose-gap components:

    gap_object_xy_cm     = ‖Δobject_pos_real − Δobject_pos_sim‖   (Euclidean cm)
    gap_object_theta_deg = |Δobject_theta_real − Δobject_theta_sim| (deg, wrapped)
    gap_robot_xy_cm      = ‖Δrobot_pos_real − Δrobot_pos_sim‖
    gap_robot_theta_deg  = |Δrobot_heading_real − Δrobot_heading_sim|

Headline loss (what a Tier 2 tuner minimizes over the chosen knob):

    L = mean(gap_object_xy) + w · mean(gap_object_theta)
      + mean(gap_robot_xy)  + w · mean(gap_robot_theta)

with ``w = 0.5 cm/deg`` by default.

Outputs (under --out-dir, default ``<real_dir>/diff_vs_<sim_basename>``):
    _per_trial.csv      one row per matched trial
    _aggregate.json     per-field stats + L breakdown + run metadata
    _unmatched.json     trials present on only one side
    comparison.md       human-readable headline + tables
    _plots/             optional, matplotlib-gated

Usage:
    python scripts/diff_real_vs_sim.py \\
        --real /path/to/real_run_dir \\
        --sim  /path/to/sim_parent_dir \\
        [--out-dir <dir>] [--theta-weight 0.5]
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ─────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────

DEFAULT_THETA_WEIGHT_CM_PER_DEG = 0.5

# Controller-setting fields that must be identical real-vs-sim. Any non-zero
# gap here is a config-drift bug, not a calibration result. Reported but
# excluded from the loss.
CONTROLLER_SETTINGS = (
    "push_controller_max_speed",
    "push_lookahead_distance_cm",
    "push_dynamic_direction",
    "push_path_length_cm",
)


# ─────────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class Trial:
    tag: str
    source: str  # "real" or "sim"
    object_id: str
    edge_idx: int
    push_steps: int
    push_start_ts: float
    delta_object_pos_cm: Tuple[float, float]
    delta_object_theta_deg: float
    delta_robot_pos_cm: Tuple[float, float]
    delta_robot_heading_deg: float
    controller_settings: Dict[str, Any]


@dataclass
class TrialPair:
    real: Trial
    sim: Trial


@dataclass
class TrialGap:
    tag_real: str
    tag_sim: str
    object_id: str
    edge_idx: int
    push_steps: int
    gap_object_xy_cm: float
    gap_object_theta_deg: float
    gap_robot_xy_cm: float
    gap_robot_theta_deg: float
    controller_setting_diffs: Dict[str, Optional[float]]


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


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    out: List[Dict[str, Any]] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def _load_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except json.JSONDecodeError:
        return None


def _wrap_to_180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def _require_field(rec: Dict[str, Any], key: str, source: str, tag: str) -> Any:
    """Schema-strict field accessor. Bails on null mismatch rather than
    silently producing a misleading diff."""
    v = rec.get(key)
    if v is None:
        raise ValueError(
            f"[diff_real_vs_sim] {source} trial {tag!r}: missing or null field "
            f"{key!r}. Schema mismatch — check the writer for {source} side."
        )
    return v


def _load_trial(
    trial_dir: Path,
    pushes_by_subgoal: Dict[int, Dict[str, Any]],
    source: str,
) -> Trial:
    tag = trial_dir.name
    metrics = _load_json(trial_dir / "metrics.json")
    if metrics is None:
        raise ValueError(
            f"[diff_real_vs_sim] {source} trial {tag!r}: missing metrics.json"
        )

    subgoal_id = int(_require_field(metrics, "subgoal_id", source, tag))
    push_rec = pushes_by_subgoal.get(subgoal_id)
    if push_rec is None:
        raise ValueError(
            f"[diff_real_vs_sim] {source} trial {tag!r}: subgoal_id "
            f"{subgoal_id} not found in parent pushes.jsonl"
        )

    object_id = str(_require_field(metrics, "object_id", source, tag))
    edge_idx = int(_require_field(metrics, "expected_edge", source, tag))
    push_steps = int(_require_field(metrics, "expected_push_steps", source, tag))

    # Object delta — from pushes.jsonl
    delta_pos = _require_field(push_rec, "delta_pos_cm", source, tag)
    delta_obj_pos = (float(delta_pos[0]), float(delta_pos[1]))
    delta_obj_theta = float(_require_field(push_rec, "delta_theta_deg", source, tag))

    # Robot delta — from metrics.json
    robot_net = _require_field(metrics, "robot_net_delta_cm", source, tag)
    delta_rob_pos = (float(robot_net[0]), float(robot_net[1]))
    delta_rob_heading = float(_require_field(metrics, "robot_heading_delta_deg", source, tag))

    push_start_ts = float(
        _require_field(push_rec, "push_start_obs_timestamp", source, tag)
    )

    controller_settings = {k: metrics.get(k) for k in CONTROLLER_SETTINGS}

    return Trial(
        tag=tag,
        source=source,
        object_id=object_id,
        edge_idx=edge_idx,
        push_steps=push_steps,
        push_start_ts=push_start_ts,
        delta_object_pos_cm=delta_obj_pos,
        delta_object_theta_deg=delta_obj_theta,
        delta_robot_pos_cm=delta_rob_pos,
        delta_robot_heading_deg=delta_rob_heading,
        controller_settings=controller_settings,
    )


def _discover_trials(run_dir: Path, source: str) -> List[Trial]:
    """All trials in one run dir (real or one sim run)."""
    tier2_root = run_dir / "tier2_push_trials"
    if not tier2_root.exists():
        raise ValueError(
            f"[diff_real_vs_sim] {source} run {run_dir!r}: no tier2_push_trials/ "
            f"subdir. Was this produced by execute_real_push / execute_sim_push?"
        )
    pushes = _load_jsonl(run_dir / "pushes.jsonl")
    pushes_by_subgoal = {
        int(r["subgoal_id"]): r for r in pushes if r.get("subgoal_id") is not None
    }
    trials: List[Trial] = []
    for trial_dir in sorted(tier2_root.iterdir()):
        if not trial_dir.is_dir():
            continue
        trials.append(_load_trial(trial_dir, pushes_by_subgoal, source))
    return trials


def _discover_sim_trials(sim_parent: Path) -> List[Trial]:
    """Discover sim trials. Accepts either a parent dir of N sim runs OR a
    single sim run dir."""
    if not sim_parent.exists():
        raise ValueError(
            f"[diff_real_vs_sim] sim parent dir not found: {sim_parent!r}"
        )
    if (sim_parent / "tier2_push_trials").exists():
        return _discover_trials(sim_parent, "sim")
    all_trials: List[Trial] = []
    for child in sorted(sim_parent.iterdir()):
        if not child.is_dir():
            continue
        if not (child / "tier2_push_trials").exists():
            continue
        all_trials.extend(_discover_trials(child, "sim"))
    if not all_trials:
        raise ValueError(
            f"[diff_real_vs_sim] sim parent {sim_parent!r}: no tier2_push_trials "
            f"found in any subdir (or directly). Did execute_sim_push run?"
        )
    return all_trials


# ─────────────────────────────────────────────────────────────────────────
# Pairing
# ─────────────────────────────────────────────────────────────────────────


def _pair_trials(
    real: List[Trial], sim: List[Trial]
) -> Tuple[List[TrialPair], List[Trial], List[Trial]]:
    """Join on (object_id, edge_idx, push_steps); within duplicates, sort
    both sides by push_start_ts and zip."""

    def _bucket(trials: List[Trial]) -> Dict[Tuple, List[Trial]]:
        out: Dict[Tuple, List[Trial]] = {}
        for t in trials:
            out.setdefault((t.object_id, t.edge_idx, t.push_steps), []).append(t)
        for ts in out.values():
            ts.sort(key=lambda t: t.push_start_ts)
        return out

    real_b = _bucket(real)
    sim_b = _bucket(sim)
    paired: List[TrialPair] = []
    real_only: List[Trial] = []
    sim_only: List[Trial] = []

    for key, real_ts in real_b.items():
        sim_ts = sim_b.get(key, [])
        n = min(len(real_ts), len(sim_ts))
        for i in range(n):
            paired.append(TrialPair(real=real_ts[i], sim=sim_ts[i]))
        real_only.extend(real_ts[n:])
    for key, sim_ts in sim_b.items():
        n_real = len(real_b.get(key, []))
        sim_only.extend(sim_ts[n_real:])
    return paired, real_only, sim_only


# ─────────────────────────────────────────────────────────────────────────
# Gap computation
# ─────────────────────────────────────────────────────────────────────────


def _euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _setting_diff(real_v: Any, sim_v: Any) -> Optional[float]:
    """Numeric diff for controller settings; None if either is non-numeric/null."""
    if real_v is None or sim_v is None:
        return None
    try:
        return float(sim_v) - float(real_v)
    except (TypeError, ValueError):
        return None


def _compute_gap(pair: TrialPair) -> TrialGap:
    r, s = pair.real, pair.sim
    return TrialGap(
        tag_real=r.tag,
        tag_sim=s.tag,
        object_id=r.object_id,
        edge_idx=r.edge_idx,
        push_steps=r.push_steps,
        gap_object_xy_cm=_euclidean(r.delta_object_pos_cm, s.delta_object_pos_cm),
        gap_object_theta_deg=abs(
            _wrap_to_180(r.delta_object_theta_deg - s.delta_object_theta_deg)
        ),
        gap_robot_xy_cm=_euclidean(r.delta_robot_pos_cm, s.delta_robot_pos_cm),
        gap_robot_theta_deg=abs(
            _wrap_to_180(r.delta_robot_heading_deg - s.delta_robot_heading_deg)
        ),
        controller_setting_diffs={
            k: _setting_diff(r.controller_settings[k], s.controller_settings[k])
            for k in CONTROLLER_SETTINGS
        },
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


def _aggregate(gaps: List[TrialGap], theta_weight: float) -> Dict[str, Any]:
    stats_map = {
        "gap_object_xy_cm": _stats([g.gap_object_xy_cm for g in gaps]),
        "gap_object_theta_deg": _stats([g.gap_object_theta_deg for g in gaps]),
        "gap_robot_xy_cm": _stats([g.gap_robot_xy_cm for g in gaps]),
        "gap_robot_theta_deg": _stats([g.gap_robot_theta_deg for g in gaps]),
    }
    L_components = {
        "object_xy_cm": stats_map["gap_object_xy_cm"].mean,
        "object_theta_weighted_cm": theta_weight * stats_map["gap_object_theta_deg"].mean,
        "robot_xy_cm": stats_map["gap_robot_xy_cm"].mean,
        "robot_theta_weighted_cm": theta_weight * stats_map["gap_robot_theta_deg"].mean,
    }
    L = sum(L_components.values())
    return {
        "n_paired_trials": len(gaps),
        "theta_weight_cm_per_deg": theta_weight,
        "headline_loss_cm": L,
        "loss_components_cm": L_components,
        "field_stats": {
            k: {
                "n": v.n,
                "mean": v.mean,
                "median": v.median,
                "p90": v.p90,
                "max": v.max,
            }
            for k, v in stats_map.items()
        },
    }


# ─────────────────────────────────────────────────────────────────────────
# Writers
# ─────────────────────────────────────────────────────────────────────────


def write_per_trial_csv(path: Path, gaps: List[TrialGap]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "tag_real",
                "tag_sim",
                "object_id",
                "edge_idx",
                "push_steps",
                "gap_object_xy_cm",
                "gap_object_theta_deg",
                "gap_robot_xy_cm",
                "gap_robot_theta_deg",
                *[f"controller_setting_diff_{k}" for k in CONTROLLER_SETTINGS],
            ]
        )
        for g in gaps:
            w.writerow(
                [
                    g.tag_real,
                    g.tag_sim,
                    g.object_id,
                    g.edge_idx,
                    g.push_steps,
                    f"{g.gap_object_xy_cm:.4f}",
                    f"{g.gap_object_theta_deg:.4f}",
                    f"{g.gap_robot_xy_cm:.4f}",
                    f"{g.gap_robot_theta_deg:.4f}",
                    *[g.controller_setting_diffs.get(k) for k in CONTROLLER_SETTINGS],
                ]
            )


def write_aggregate_json(path: Path, agg: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(agg, f, indent=2)


def write_unmatched_json(
    path: Path, real_only: List[Trial], sim_only: List[Trial]
) -> None:
    def _trial_brief(t: Trial) -> Dict[str, Any]:
        return {
            "tag": t.tag,
            "object_id": t.object_id,
            "edge_idx": t.edge_idx,
            "push_steps": t.push_steps,
            "push_start_ts": t.push_start_ts,
        }

    payload = {
        "real_only": [_trial_brief(t) for t in real_only],
        "sim_only": [_trial_brief(t) for t in sim_only],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


def write_comparison_md(
    path: Path,
    agg: Dict[str, Any],
    gaps: List[TrialGap],
    real_only: List[Trial],
    sim_only: List[Trial],
) -> None:
    n = agg["n_paired_trials"]
    L = agg["headline_loss_cm"]
    w = agg["theta_weight_cm_per_deg"]
    c = agg["loss_components_cm"]
    s = agg["field_stats"]

    lines: List[str] = []
    lines.append("# Real vs Sim — Tier 2 calibration diff\n")
    lines.append(
        f"**Headline loss `L = {L:.3f} cm`**  "
        f"(w_theta = {w} cm/deg, n = {n} matched trials)\n"
    )
    lines.append("## Loss breakdown\n")
    lines.append("| Component | cm |")
    lines.append("|---|---:|")
    lines.append(f"| object xy           | {c['object_xy_cm']:.3f} |")
    lines.append(f"| object θ (weighted) | {c['object_theta_weighted_cm']:.3f} |")
    lines.append(f"| robot xy            | {c['robot_xy_cm']:.3f} |")
    lines.append(f"| robot θ (weighted)  | {c['robot_theta_weighted_cm']:.3f} |")
    lines.append("")

    lines.append("## Per-field statistics\n")
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

    if gaps:
        lines.append("## Worst-5 trials by object xy + θ gap\n")
        worst = sorted(
            gaps,
            key=lambda g: g.gap_object_xy_cm + w * g.gap_object_theta_deg,
            reverse=True,
        )[:5]
        lines.append(
            "| trial | obj | edge | depth | object xy (cm) | object θ (°) | "
            "robot xy (cm) | robot θ (°) |"
        )
        lines.append("|---|---|---:|---:|---:|---:|---:|---:|")
        for g in worst:
            lines.append(
                f"| {g.tag_real} | {g.object_id} | {g.edge_idx} | "
                f"{g.push_steps} | {g.gap_object_xy_cm:.3f} | "
                f"{g.gap_object_theta_deg:.2f} | {g.gap_robot_xy_cm:.3f} | "
                f"{g.gap_robot_theta_deg:.2f} |"
            )
        lines.append("")

    # Controller-setting sanity bucket
    nonzero = []
    for g in gaps:
        for k, v in g.controller_setting_diffs.items():
            if v is not None and abs(v) > 1e-6:
                nonzero.append((g.tag_real, k, v))
    if nonzero:
        lines.append("## ⚠ Controller-setting drift (expected zero)\n")
        lines.append(
            "Non-zero values mean real and sim ran with different controller "
            "settings. Fix the config drift before trusting the calibration "
            "numbers above.\n"
        )
        lines.append("| trial | setting | sim − real |")
        lines.append("|---|---|---:|")
        for tag, k, v in nonzero[:20]:
            lines.append(f"| {tag} | {k} | {v:+.4f} |")
        lines.append("")
    elif gaps:
        lines.append("## Controller-setting sanity\n")
        lines.append("All controller settings match across real and sim. ✓\n")

    if real_only or sim_only:
        lines.append("## Unmatched trials\n")
        lines.append(
            f"- {len(real_only)} real-only, {len(sim_only)} sim-only. "
            f"Detail in `_unmatched.json`.\n"
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def write_plots(
    plot_dir: Path,
    real: List[Trial],
    sim: List[Trial],
) -> bool:
    """Return True if plots were written; False if matplotlib unavailable."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    real_by_key = {(t.object_id, t.edge_idx, t.push_steps): t for t in real}
    sim_by_key = {(t.object_id, t.edge_idx, t.push_steps): t for t in sim}
    keys = sorted(set(real_by_key) & set(sim_by_key))
    if not keys:
        return False

    def _vals(side: Dict[Tuple, Trial], attr: str, idx: Optional[int] = None) -> List[float]:
        out = []
        for k in keys:
            v = getattr(side[k], attr)
            out.append(v[idx] if idx is not None else v)
        return out

    def _scatter(ax, real_vals, sim_vals, title, units):
        ax.scatter(real_vals, sim_vals, alpha=0.6)
        lo = min(min(real_vals), min(sim_vals))
        hi = max(max(real_vals), max(sim_vals))
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
    _scatter(axes[0], _vals(real_by_key, "delta_object_pos_cm", 0),
             _vals(sim_by_key, "delta_object_pos_cm", 0), "object Δx", "cm")
    _scatter(axes[1], _vals(real_by_key, "delta_object_pos_cm", 1),
             _vals(sim_by_key, "delta_object_pos_cm", 1), "object Δy", "cm")
    _scatter(axes[2], _vals(real_by_key, "delta_object_theta_deg"),
             _vals(sim_by_key, "delta_object_theta_deg"), "object Δθ", "deg")
    fig.tight_layout()
    fig.savefig(plot_dir / "object_pose.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    _scatter(axes[0], _vals(real_by_key, "delta_robot_pos_cm", 0),
             _vals(sim_by_key, "delta_robot_pos_cm", 0), "robot Δx", "cm")
    _scatter(axes[1], _vals(real_by_key, "delta_robot_pos_cm", 1),
             _vals(sim_by_key, "delta_robot_pos_cm", 1), "robot Δy", "cm")
    _scatter(axes[2], _vals(real_by_key, "delta_robot_heading_deg"),
             _vals(sim_by_key, "delta_robot_heading_deg"), "robot Δheading", "deg")
    fig.tight_layout()
    fig.savefig(plot_dir / "robot_pose.png", dpi=150)
    plt.close(fig)

    return True


# ─────────────────────────────────────────────────────────────────────────
# Orchestration
# ─────────────────────────────────────────────────────────────────────────


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real", required=True, help="execute_real_push run dir")
    parser.add_argument(
        "--sim",
        required=True,
        help="execute_sim_push parent dir (or a single sim run dir)",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="output dir (default: <real>/diff_vs_<sim_basename>)",
    )
    parser.add_argument(
        "--theta-weight",
        type=float,
        default=DEFAULT_THETA_WEIGHT_CM_PER_DEG,
        help=f"cm-per-deg weight for combining xy + theta gaps "
        f"(default {DEFAULT_THETA_WEIGHT_CM_PER_DEG})",
    )
    args = parser.parse_args()

    real_dir = Path(args.real).resolve()
    sim_dir = Path(args.sim).resolve()
    out_dir = (
        Path(args.out_dir).resolve()
        if args.out_dir
        else real_dir / f"diff_vs_{sim_dir.name}"
    )

    print(f"[diff_real_vs_sim] real: {real_dir}")
    print(f"[diff_real_vs_sim]  sim: {sim_dir}")
    print(f"[diff_real_vs_sim]  out: {out_dir}")

    real_trials = _discover_trials(real_dir, "real")
    sim_trials = _discover_sim_trials(sim_dir)
    print(
        f"[diff_real_vs_sim] discovered {len(real_trials)} real, "
        f"{len(sim_trials)} sim trials"
    )

    paired, real_only, sim_only = _pair_trials(real_trials, sim_trials)
    print(
        f"[diff_real_vs_sim] paired {len(paired)} "
        f"(real-only: {len(real_only)}, sim-only: {len(sim_only)})"
    )
    if not paired:
        print(
            "[diff_real_vs_sim] no paired trials — nothing to diff",
            file=sys.stderr,
        )
        return 1

    gaps = [_compute_gap(p) for p in paired]
    agg = _aggregate(gaps, args.theta_weight)
    print(f"[diff_real_vs_sim] headline L = {agg['headline_loss_cm']:.3f} cm")

    write_per_trial_csv(out_dir / "_per_trial.csv", gaps)
    write_aggregate_json(out_dir / "_aggregate.json", agg)
    write_unmatched_json(out_dir / "_unmatched.json", real_only, sim_only)
    write_comparison_md(out_dir / "comparison.md", agg, gaps, real_only, sim_only)
    plotted = write_plots(out_dir / "_plots", real_trials, sim_trials)
    if not plotted:
        print("[diff_real_vs_sim] matplotlib unavailable — skipped plots")

    print(f"[diff_real_vs_sim] wrote outputs under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
