"""Fit chassis velocities from per-frame pose logs and compare real vs sim.

Two subcommands:

  trial    Fit one trial directory, print the velocity number.
  session  Walk a session directory (with real/ and sim/ subtrees),
           fit every trial, pair them by trial-name, and write
           velocities.csv + comparison.md.

Usage:
    # Single trial — quick diagnostic.
    python scripts/analyze_chassis_motion.py trial \\
        --trial-dir chassis_calibration/<session>/real/straight_cmd0p20

    # Full session — produces analysis/velocities.csv and comparison.md.
    python scripts/analyze_chassis_motion.py session \\
        --session-dir chassis_calibration/<session>
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Make in-tree imports work whether or not the package is pip-install -e'd.
HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_control.controller.config import load_controller_configs
from robot_control.diagnostics.chassis_motion import (
    StraightFit,
    TrialMeta,
    TurnFit,
    fit_straight,
    fit_turn,
    read_trial,
)


# ─────────────────────────────────────────────────────────────────────────
# Single-trial analysis
# ─────────────────────────────────────────────────────────────────────────


def analyze_trial(
    trial_dir: Path, settle_s: float
) -> Tuple[TrialMeta, Optional[StraightFit], Optional[TurnFit], Optional[str]]:
    """Read one trial; return (meta, straight_fit?, turn_fit?, error?).

    Exactly one of straight_fit / turn_fit is populated, depending on
    trial_kind. If fitting fails (e.g., too few samples), the error
    string is populated and both fits are None.
    """
    meta, poses = read_trial(trial_dir)
    try:
        if meta.trial_kind == "straight":
            return meta, fit_straight(poses, settle_s=settle_s), None, None
        elif meta.trial_kind == "turn":
            return meta, None, fit_turn(poses, settle_s=settle_s), None
        else:  # "arc"
            # Both linear and angular velocity are meaningful for arcs.
            try:
                sfit = fit_straight(poses, settle_s=settle_s)
            except ValueError:
                sfit = None
            try:
                tfit = fit_turn(poses, settle_s=settle_s)
            except ValueError:
                tfit = None
            return meta, sfit, tfit, None
    except ValueError as exc:
        return meta, None, None, str(exc)


def print_trial_summary(
    trial_dir: Path,
    meta: TrialMeta,
    sfit: Optional[StraightFit],
    tfit: Optional[TurnFit],
    error: Optional[str],
) -> None:
    """Human-readable single-trial report."""
    print(f"=== {trial_dir.name} ({meta.source}, {meta.trial_kind}) ===")
    print(
        f"  cmd: left={meta.left_cmd:+.3f}, right={meta.right_cmd:+.3f} "
        f"({meta.cmd_units}); duration={meta.duration_s:.1f}s; "
        f"n_samples={meta.n_samples}"
    )
    if error:
        print(f"  FIT FAILED: {error}")
        return
    if sfit is not None:
        ci95 = 1.96 * sfit.se_v_cm_s
        print(
            f"  v_linear = {sfit.v_linear_cm_s:.3f} ± {ci95:.3f} cm/s "
            f"(95% CI)  [vx={sfit.vx_cm_s:+.3f}, vy={sfit.vy_cm_s:+.3f}]"
        )
        print(
            f"  n_steady={sfit.n_steady_samples}, "
            f"window={sfit.steady_window_s:.2f}s, "
            f"residual_rms={sfit.fit_residual_rms_cm:.3f} cm"
        )
    if tfit is not None:
        ci95 = 1.96 * tfit.se_omega_deg_s
        print(
            f"  omega = {tfit.omega_deg_s:+.3f} ± {ci95:.3f} deg/s (95% CI)"
        )
        print(
            f"  n_steady={tfit.n_steady_samples}, "
            f"window={tfit.steady_window_s:.2f}s, "
            f"residual_rms={tfit.fit_residual_rms_deg:.3f} deg"
        )


# ─────────────────────────────────────────────────────────────────────────
# Session analysis
# ─────────────────────────────────────────────────────────────────────────


def fit_all_in(
    parent: Path, settle_s: float
) -> Dict[str, Tuple[TrialMeta, Optional[StraightFit], Optional[TurnFit], Optional[str]]]:
    """Fit every trial dir under ``parent``. Key = trial dir name."""
    results: Dict[str, Tuple] = {}
    if not parent.is_dir():
        return results
    for child in sorted(parent.iterdir()):
        if not child.is_dir():
            continue
        if not (child / "trial_meta.json").exists():
            continue
        results[child.name] = analyze_trial(child, settle_s=settle_s)
    return results


def write_velocities_csv(
    real: Dict[str, Tuple], sim: Dict[str, Tuple], out: Path
) -> None:
    """One row per trial across both sides."""
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "source", "trial_kind", "trial_name",
            "left_cmd", "right_cmd", "cmd_units",
            "v_linear_cm_s", "omega_deg_s",
            "n_steady_samples", "se_v_cm_s", "se_omega_deg_s",
            "residual_rms_cm", "residual_rms_deg",
            "fit_error",
        ])
        for source, results in (("real", real), ("sim", sim)):
            for name, (meta, sfit, tfit, err) in results.items():
                w.writerow([
                    source, meta.trial_kind, name,
                    meta.left_cmd, meta.right_cmd, meta.cmd_units,
                    f"{sfit.v_linear_cm_s:.4f}" if sfit else "",
                    f"{tfit.omega_deg_s:.4f}" if tfit else "",
                    (sfit.n_steady_samples if sfit else (tfit.n_steady_samples if tfit else "")),
                    f"{sfit.se_v_cm_s:.5f}" if sfit else "",
                    f"{tfit.se_omega_deg_s:.5f}" if tfit else "",
                    f"{sfit.fit_residual_rms_cm:.4f}" if sfit else "",
                    f"{tfit.fit_residual_rms_deg:.4f}" if tfit else "",
                    err or "",
                ])


def write_comparison_md(
    real: Dict[str, Tuple], sim: Dict[str, Tuple], out: Path
) -> None:
    """Side-by-side comparison grouped by trial kind, plus a slope ratio.

    The slope ratio (real-velocity-per-cmd / sim-velocity-per-cmd) is the
    Tier 1 calibration factor: it tells you how to adjust
    ``skill.push_tracker_max_speed`` (YAML, no recompile) so sim's
    production chassis velocity matches real's.

    Specifically, if real production drives wheels at PWM
    ``push.max_speed`` (from controller.yaml; currently 0.2) and the
    slope ratio k_real/k_sim is r, then setting
    ``push_tracker_max_speed = push.max_speed × r`` makes sim's chassis
    velocity at production saturation match real's chassis velocity at
    its production PWM. The compiled constant ``kCarWheelMaxSpeedMs`` is
    NOT touched — that's Tier 2, only reach for it if Tier 1 can't
    close the gap.
    """
    out.parent.mkdir(parents=True, exist_ok=True)
    paired = sorted(set(real.keys()) & set(sim.keys()))
    real_only = sorted(set(real.keys()) - set(sim.keys()))
    sim_only = sorted(set(sim.keys()) - set(real.keys()))

    lines: List[str] = []
    lines.append("# Chassis-motion calibration — sim vs real\n")

    # Straight trials
    straight_pairs = [n for n in paired if real[n][0].trial_kind == "straight"]
    if straight_pairs:
        lines.append("## Straight-line trials\n")
        lines.append("| trial | cmd | v_real (cm/s) | v_sim (cm/s) | gap (cm/s) | gap % | real/sim |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|")
        slope_data: List[Tuple[float, float, float]] = []  # (cmd, v_real, v_sim)
        for name in straight_pairs:
            r_meta, r_sfit, _, _ = real[name]
            s_meta, s_sfit, _, _ = sim[name]
            if r_sfit is None or s_sfit is None:
                lines.append(
                    f"| {name} | {r_meta.left_cmd:+.3f} | "
                    f"{'?' if r_sfit is None else f'{r_sfit.v_linear_cm_s:.3f}'} | "
                    f"{'?' if s_sfit is None else f'{s_sfit.v_linear_cm_s:.3f}'} | "
                    "— | — | — |"
                )
                continue
            gap = r_sfit.v_linear_cm_s - s_sfit.v_linear_cm_s
            gap_pct = 100.0 * gap / s_sfit.v_linear_cm_s if s_sfit.v_linear_cm_s != 0 else float("inf")
            ratio = r_sfit.v_linear_cm_s / s_sfit.v_linear_cm_s if s_sfit.v_linear_cm_s != 0 else float("inf")
            lines.append(
                f"| {name} | {r_meta.left_cmd:+.3f} | "
                f"{r_sfit.v_linear_cm_s:.3f} | "
                f"{s_sfit.v_linear_cm_s:.3f} | "
                f"{gap:+.3f} | "
                f"{gap_pct:+.2f}% | "
                f"{ratio:.4f} |"
            )
            slope_data.append((r_meta.left_cmd, r_sfit.v_linear_cm_s, s_sfit.v_linear_cm_s))

        # Slope through the origin: best fit of v vs cmd via least squares
        # constrained to pass through (0, 0).
        if slope_data:
            cmds = [d[0] for d in slope_data]
            vreal = [d[1] for d in slope_data]
            vsim = [d[2] for d in slope_data]
            slope_real = sum(c * v for c, v in zip(cmds, vreal)) / sum(c * c for c in cmds)
            slope_sim = sum(c * v for c, v in zip(cmds, vsim)) / sum(c * c for c in cmds)
            lines.append("")
            lines.append("**Linear regression through the origin, v = k · cmd:**")
            lines.append(f"- real: k_real = {slope_real:.4f} cm/s per unit cmd")
            lines.append(f"- sim:  k_sim  = {slope_sim:.4f} cm/s per unit cmd")
            if slope_sim != 0:
                cal_ratio = slope_real / slope_sim
                lines.append(f"- ratio (real/sim): **{cal_ratio:.4f}**\n")

                # Tier 1 recommendation: set push_tracker_max_speed in the
                # car YAML to match real production at its push.max_speed.
                real_push_pwm = load_controller_configs().push.max_speed
                recommended_ptm = real_push_pwm * cal_ratio
                lines.append("### Tier 1 calibration recommendation\n")
                lines.append(
                    f"Real production drives wheels at "
                    f"`controller.yaml: push.max_speed = {real_push_pwm:.3f}` (PWM)."
                )
                lines.append(
                    "Sim production uses `skill.push_tracker_max_speed` as the "
                    "follower-output cap (defaulted to 0.3 by ConfigManager if "
                    "absent from `namo_config_complete_skill15_car_1x.yaml`)."
                )
                lines.append("")
                lines.append(
                    f"To make sim's chassis velocity at the production cap "
                    f"match real's at `push.max_speed = {real_push_pwm:.3f}`, set "
                    f"`skill.push_tracker_max_speed = "
                    f"{real_push_pwm:.3f} × {cal_ratio:.4f} = "
                    f"**{recommended_ptm:.4f}**` in "
                    f"`namo_cpp/config/namo_config_complete_skill15_car_1x.yaml`."
                )
                lines.append("")
                lines.append(
                    "This is the Tier 1 fix — YAML edit only, no recompile. "
                    "`kCarWheelMaxSpeedMs` (compiled constant in "
                    "`namo_push_controller.cpp:22`) and Tier 2 knobs (`kv`, "
                    "`forcerange` on the wheel actuators) are not touched."
                )
                lines.append("")

    # Turn trials
    turn_pairs = [n for n in paired if real[n][0].trial_kind == "turn"]
    if turn_pairs:
        lines.append("## Turn-in-place trials\n")
        lines.append("| trial | cmd | omega_real (deg/s) | omega_sim (deg/s) | gap | real/sim |")
        lines.append("|---|---:|---:|---:|---:|---:|")
        for name in turn_pairs:
            r_meta, _, r_tfit, _ = real[name]
            s_meta, _, s_tfit, _ = sim[name]
            if r_tfit is None or s_tfit is None:
                lines.append(
                    f"| {name} | {r_meta.left_cmd:+.3f}/{r_meta.right_cmd:+.3f} | "
                    f"{'?' if r_tfit is None else f'{r_tfit.omega_deg_s:+.3f}'} | "
                    f"{'?' if s_tfit is None else f'{s_tfit.omega_deg_s:+.3f}'} | "
                    "— | — |"
                )
                continue
            gap = r_tfit.omega_deg_s - s_tfit.omega_deg_s
            ratio = (
                r_tfit.omega_deg_s / s_tfit.omega_deg_s
                if s_tfit.omega_deg_s != 0 else float("inf")
            )
            lines.append(
                f"| {name} | {r_meta.left_cmd:+.3f}/{r_meta.right_cmd:+.3f} | "
                f"{r_tfit.omega_deg_s:+.3f} | "
                f"{s_tfit.omega_deg_s:+.3f} | "
                f"{gap:+.3f} | "
                f"{ratio:.4f} |"
            )

    # Unpaired trials
    if real_only or sim_only:
        lines.append("\n## Unpaired trials\n")
        if real_only:
            lines.append(f"**Real only** ({len(real_only)}): {', '.join(real_only)}")
        if sim_only:
            lines.append(f"**Sim only** ({len(sim_only)}): {', '.join(sim_only)}")

    out.write_text("\n".join(lines) + "\n")


def analyze_session(session_dir: Path, settle_s: float) -> None:
    real_results = fit_all_in(session_dir / "real", settle_s)
    sim_results = fit_all_in(session_dir / "sim", settle_s)

    analysis_dir = session_dir / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)

    csv_out = analysis_dir / "velocities.csv"
    md_out = analysis_dir / "comparison.md"

    write_velocities_csv(real_results, sim_results, csv_out)
    write_comparison_md(real_results, sim_results, md_out)

    print(f"[session] {len(real_results)} real trials, {len(sim_results)} sim trials")
    print(f"[session] wrote {csv_out}")
    print(f"[session] wrote {md_out}")


# ─────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────


def main() -> int:
    args = parse_args()
    if args.subcommand == "trial":
        trial_dir = Path(args.trial_dir).resolve()
        meta, sfit, tfit, err = analyze_trial(trial_dir, settle_s=args.settle_s)
        print_trial_summary(trial_dir, meta, sfit, tfit, err)
        return 0
    elif args.subcommand == "session":
        analyze_session(Path(args.session_dir).resolve(), settle_s=args.settle_s)
        return 0
    return 1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="subcommand", required=True)

    p_trial = sub.add_parser("trial", help="Fit a single trial directory.")
    p_trial.add_argument(
        "--trial-dir", required=True, help="Trial directory to analyze."
    )
    p_trial.add_argument(
        "--settle-s",
        type=float,
        default=2.0,
        help="Seconds to skip from the start of each trial when fitting "
        "(default: 2.0, skips the acceleration phase).",
    )

    p_session = sub.add_parser(
        "session", help="Walk a session directory, fit all trials, emit CSV + MD."
    )
    p_session.add_argument(
        "--session-dir",
        required=True,
        help="Session directory containing real/ and sim/ subtrees.",
    )
    p_session.add_argument(
        "--settle-s", type=float, default=2.0, help="Skip this many seconds; default 2."
    )
    return p.parse_args()


if __name__ == "__main__":
    raise SystemExit(main())
