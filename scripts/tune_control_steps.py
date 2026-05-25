"""Binary-search ``control_steps_per_push`` to minimize sim-vs-real push gap.

Same algorithm shape as ``find_sim_fraction.py`` (Tier 1) and
``tune_forcerange.py``, applied to the *correct* Tier 2 parameter:
``control_steps_per_push`` in the namo_cpp YAML config. This sets how
many MuJoCo physics ticks one NAMO push step lasts in sim. At default
482 ticks × 0.002 s = 0.964 s per NAMO unit. Reducing it shortens each
push → less sim displacement.

Forcerange was previously tried as the Tier 2 knob and found to have no
effect across [0.1, 1.5] Nm — the wheel actuator never saturates for
these pushes, so it can't shrink the sim/real gap. The 10 cm signed
"sim overshoots real" gap lives in *push duration*, not torque.

Algorithm:

  * Search variable: ``control_steps_per_push`` (int). Patched into the
    YAML config via regex (preserves the trailing comment).

  * Convergence signal (signed): per leaf, project sim's Δobject and
    the mean of real trials' Δobject onto the leaf's push direction.
    Take the difference (sim − real_mean). Average across leaves.
    Positive ⇒ sim overshoots real ⇒ control_steps_per_push too high.
    Monotone in the parameter (more ticks → more displacement, linearly
    up to wheel saturation).

  * Convergence threshold ε is data-driven (same as forcerange tuner):
        ε = eps_multiplier × mean(SE_leaf across leaves)

  * Reported loss (unsigned): diff_real_vs_sim's headline_loss_cm
    tracked per iter; recommendation falls back to best-by-L if signed
    signal doesn't cleanly converge.

Per iteration:
  1. Patch ``<yaml>`` to set control_steps_per_push: X.
  2. For each leaf with real trials: subprocess execute_sim_push.py
     --no-video --from-real-run <leaf>/trial1 --push-index 0
     --diag-path <leaf> --run-name sim --allow-overwrite
     (overwrites <leaf>/sim/, deterministic).
  3. Subprocess diff_real_vs_sim.py --root <root>.
  4. Read headline_loss_cm; reload tree; compute signed signal.
  5. Update bracket: signed > 0 ⇒ U = X (too high); else L = X.

YAML state saved at entry, restored at exit (try/finally + SIGINT/
SIGTERM handlers). Also restored after the final-video pass.

Usage:
    python scripts/tune_control_steps.py \\
        --root push_calibration/obj_1 \\
        --yaml namo_cpp/config/namo_config_complete_skill15_car_1x.yaml \\
        [--bracket-low 30] [--bracket-high 1000] \\
        [--eps-multiplier 2.0 | --epsilon X] \\
        [--max-iters 10] [--out-dir <dir>] [--skip-final-video]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from diff_real_vs_sim import (  # type: ignore  # noqa: E402
    LeafData,
    Trial,
    _discover_leaves,
)


# ─────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────

DEFAULT_EPS_MULTIPLIER = 2.0
DEFAULT_MAX_ITERS = 10
DEFAULT_BRACKET_LOW = 30          # ticks; below ~30, push is too short to mean anything
DEFAULT_BRACKET_HIGH = 1000       # ticks; ~2 s/NAMO unit ceiling
MIN_BRACKET = 5                   # stop if (U − L) drops below 5 ticks


# ─────────────────────────────────────────────────────────────────────────
# YAML patching
# ─────────────────────────────────────────────────────────────────────────

# Matches `<indent>control_steps_per_push: <int><optional comment/whitespace>`.
# Captures the prefix (indent + key + colon + space) and the trailing tail
# (comment / whitespace), so the value can be swapped without disturbing
# either.
_CONTROL_STEPS_RE = re.compile(
    r'(?m)^(\s*control_steps_per_push:\s*)(\d+)(\s*.*)$'
)


def _patch_control_steps(yaml_path: Path, ticks: int) -> int:
    """Set control_steps_per_push: <ticks> in the YAML. Returns the count
    of replacements made. Raises if zero — wrong file."""
    src = yaml_path.read_text()

    def _replace(m: re.Match) -> str:
        return f"{m.group(1)}{ticks}{m.group(3)}"

    new_src, count = _CONTROL_STEPS_RE.subn(_replace, src)
    if count == 0:
        raise ValueError(
            f"[tune_control_steps] no `control_steps_per_push:` line in "
            f"{yaml_path}. Is this the correct car config YAML?"
        )
    yaml_path.write_text(new_src)
    return count


# ─────────────────────────────────────────────────────────────────────────
# Subprocess runners (identical to tune_forcerange's)
# ─────────────────────────────────────────────────────────────────────────


def _run_sim_for_leaf(
    *,
    leaf: LeafData,
    leaf_dir: Path,
    no_video: bool,
    robot_control_root: Path,
) -> bool:
    if not leaf.real_trials:
        return False
    trial1_path = Path(leaf.real_trials[0].source_path)

    cmd = [
        sys.executable,
        "scripts/execute_sim_push.py",
        "--from-real-run", str(trial1_path),
        "--push-index", "0",
        "--diag-path", str(leaf_dir),
        "--run-name", "sim",
        "--allow-overwrite",
    ]
    if no_video:
        cmd.append("--no-video")

    proc = subprocess.run(
        cmd, cwd=str(robot_control_root),
        capture_output=True, text=True,
    )
    if proc.returncode != 0:
        tail = "\n      ".join(proc.stderr.strip().splitlines()[-5:])
        print(f"      ✗ sim FAILED for edge{leaf.edge_idx}/depth{leaf.depth} "
              f"(rc={proc.returncode}):\n      {tail}", flush=True)
        return False
    return True


def _run_diff(*, root: Path, robot_control_root: Path) -> Dict[str, Any]:
    cmd = [
        sys.executable,
        "scripts/diff_real_vs_sim.py",
        "--root", str(root),
    ]
    proc = subprocess.run(
        cmd, cwd=str(robot_control_root),
        capture_output=True, text=True,
    )
    if proc.returncode != 0:
        tail = "\n  ".join(proc.stderr.strip().splitlines()[-10:])
        raise RuntimeError(
            f"[tune_control_steps] diff_real_vs_sim failed (rc={proc.returncode}). "
            f"stderr tail:\n  {tail}"
        )
    return json.loads((root / "diff" / "_aggregate.json").read_text())


# ─────────────────────────────────────────────────────────────────────────
# Direction signal + standard error (identical to tune_forcerange)
# ─────────────────────────────────────────────────────────────────────────


def _leaf_push_direction(real_trials: List[Trial]) -> Optional[Tuple[float, float]]:
    sx = sum(t.delta_object_pos_cm[0] for t in real_trials)
    sy = sum(t.delta_object_pos_cm[1] for t in real_trials)
    mag = math.hypot(sx, sy)
    if mag < 1e-9:
        return None
    return (sx / mag, sy / mag)


def _project(vec: Tuple[float, float], unit: Tuple[float, float]) -> float:
    return vec[0] * unit[0] + vec[1] * unit[1]


def _sample_std(values: List[float]) -> float:
    n = len(values)
    if n < 2:
        return 0.0
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    return math.sqrt(var)


@dataclass
class LeafSignal:
    edge_idx: int
    depth: int
    n_real_trials: int
    real_mean_proj_cm: float
    real_se_proj_cm: float
    sim_proj_cm: float
    signed_gap_cm: float
    push_dir: Tuple[float, float]


def _compute_leaf_signals(
    leaves: Dict[Tuple[int, int], LeafData],
    exclude_edges: Optional[set] = None,
) -> List[LeafSignal]:
    """Returns one LeafSignal per leaf that has both real + sim data and is
    NOT in the exclude_edges set. Excluded leaves don't contribute to the
    bisection signal even if their data exists on disk."""
    out: List[LeafSignal] = []
    for (e, d), leaf in sorted(leaves.items()):
        if exclude_edges is not None and e in exclude_edges:
            continue
        if not leaf.real_trials or leaf.sim_trial is None:
            continue
        push_dir = _leaf_push_direction(leaf.real_trials)
        if push_dir is None:
            print(f"  ⚠ edge{e}/depth{d}: zero net real displacement, skipping",
                  flush=True)
            continue
        real_projs = [_project(t.delta_object_pos_cm, push_dir)
                      for t in leaf.real_trials]
        sim_proj = _project(leaf.sim_trial.delta_object_pos_cm, push_dir)
        real_mean = sum(real_projs) / len(real_projs)
        real_std = _sample_std(real_projs)
        real_se = real_std / math.sqrt(len(real_projs)) if real_projs else 0.0
        out.append(LeafSignal(
            edge_idx=e, depth=d,
            n_real_trials=len(leaf.real_trials),
            real_mean_proj_cm=real_mean,
            real_se_proj_cm=real_se,
            sim_proj_cm=sim_proj,
            signed_gap_cm=sim_proj - real_mean,
            push_dir=push_dir,
        ))
    return out


def _compute_epsilon(
    leaves: Dict[Tuple[int, int], LeafData],
    eps_multiplier: float,
    exclude_edges: Optional[set] = None,
) -> Tuple[float, float, int]:
    """Epsilon from the SE of real trials at non-excluded leaves only.
    Excluded edges (e.g. edge59 corner-of-face outliers) are removed from
    the SE pool so the convergence threshold matches the signal we're
    actually bisecting on."""
    per_leaf_se: List[float] = []
    for (e, _d), leaf in leaves.items():
        if exclude_edges is not None and e in exclude_edges:
            continue
        if len(leaf.real_trials) < 2:
            continue
        push_dir = _leaf_push_direction(leaf.real_trials)
        if push_dir is None:
            continue
        projs = [_project(t.delta_object_pos_cm, push_dir)
                 for t in leaf.real_trials]
        se = _sample_std(projs) / math.sqrt(len(projs))
        per_leaf_se.append(se)
    if not per_leaf_se:
        raise ValueError(
            "[tune_control_steps] no leaves have ≥2 real trials with non-zero "
            "displacement; cannot derive epsilon from data. Pass --epsilon."
        )
    mean_se = sum(per_leaf_se) / len(per_leaf_se)
    return eps_multiplier * mean_se, mean_se, len(per_leaf_se)


# ─────────────────────────────────────────────────────────────────────────
# Iteration record + best-iter pick
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class Iteration:
    idx: int
    X: int                              # control_steps_per_push (ticks)
    L_before: int
    U_before: int
    n_sim_succeeded: int
    n_sim_failed: int
    headline_loss_cm: float
    mean_signed_gap_cm: float
    n_leaves_paired: int
    leaf_signals: List[Dict[str, Any]] = field(default_factory=list)
    wall_s: float = 0.0


def _best_iteration(iters: List[Iteration]) -> Optional[Iteration]:
    valid = [it for it in iters if not math.isnan(it.headline_loss_cm)]
    if not valid:
        return None
    return min(valid, key=lambda it: it.headline_loss_cm)


# ─────────────────────────────────────────────────────────────────────────
# Binary search (integer X)
# ─────────────────────────────────────────────────────────────────────────


def binary_search_control_steps(
    *,
    root: Path,
    yaml_path: Path,
    bracket_low: int,
    bracket_high: int,
    epsilon: float,
    max_iters: int,
    robot_control_root: Path,
    exclude_edges: Optional[set] = None,
) -> Tuple[List[Iteration], bool, Optional[int], int, int]:
    L, U = int(bracket_low), int(bracket_high)
    iterations: List[Iteration] = []
    converged = False
    final_X: Optional[int] = None

    print(f"[search] bracket=[{L}, {U}] ticks, ε={epsilon:.3f} cm, "
          f"max_iters={max_iters}")

    for i in range(1, max_iters + 1):
        X = (L + U) // 2  # int midpoint
        t0 = time.time()
        print(f"\n[iter {i:02d}] X={X} ticks "
              f"(= {X * 0.002:.3f} s per NAMO push step)  "
              f"(L={L}, U={U})")

        _patch_control_steps(yaml_path, X)

        leaves_pre = _discover_leaves(root)
        leaves_with_real = [((e, d), leaf) for (e, d), leaf in sorted(leaves_pre.items())
                            if leaf.real_trials]
        print(f"  running sim for {len(leaves_with_real)} leaves…")
        n_ok = n_fail = 0
        for (e, d), leaf in leaves_with_real:
            leaf_dir = root / f"edge{e}" / f"depth{d}"
            ok = _run_sim_for_leaf(
                leaf=leaf, leaf_dir=leaf_dir,
                no_video=True,
                robot_control_root=robot_control_root,
            )
            if ok:
                n_ok += 1
            else:
                n_fail += 1
        print(f"  sim: {n_ok} ok, {n_fail} failed")

        try:
            agg = _run_diff(root=root, robot_control_root=robot_control_root)
        except Exception as exc:
            print(f"  ✗ diff failed: {exc!r}")
            iterations.append(Iteration(
                idx=i, X=X, L_before=L, U_before=U,
                n_sim_succeeded=n_ok, n_sim_failed=n_fail,
                headline_loss_cm=float("nan"),
                mean_signed_gap_cm=float("nan"),
                n_leaves_paired=0, wall_s=time.time() - t0,
            ))
            L = X
            continue

        headline_L = float(agg.get("headline_loss_cm", float("nan")))
        leaves_post = _discover_leaves(root)
        signals = _compute_leaf_signals(leaves_post, exclude_edges=exclude_edges)
        if not signals:
            print(f"  ✗ no paired leaves with non-zero displacement; bias L=X")
            iterations.append(Iteration(
                idx=i, X=X, L_before=L, U_before=U,
                n_sim_succeeded=n_ok, n_sim_failed=n_fail,
                headline_loss_cm=headline_L,
                mean_signed_gap_cm=float("nan"),
                n_leaves_paired=0, wall_s=time.time() - t0,
            ))
            L = X
            continue

        mean_signed = sum(s.signed_gap_cm for s in signals) / len(signals)
        wall_s = time.time() - t0
        within = abs(mean_signed) <= epsilon
        print(f"  L_headline={headline_L:.3f} cm  "
              f"mean_signed_gap={mean_signed:+.3f} cm  "
              f"({'✓ within ε' if within else 'continue'})  "
              f"wall={wall_s:.1f}s")

        iterations.append(Iteration(
            idx=i, X=X, L_before=L, U_before=U,
            n_sim_succeeded=n_ok, n_sim_failed=n_fail,
            headline_loss_cm=headline_L,
            mean_signed_gap_cm=mean_signed,
            n_leaves_paired=len(signals),
            leaf_signals=[{
                "edge_idx": s.edge_idx, "depth": s.depth,
                "n_real_trials": s.n_real_trials,
                "real_mean_proj_cm": s.real_mean_proj_cm,
                "real_se_proj_cm": s.real_se_proj_cm,
                "sim_proj_cm": s.sim_proj_cm,
                "signed_gap_cm": s.signed_gap_cm,
            } for s in signals],
            wall_s=wall_s,
        ))

        if within:
            converged = True
            final_X = X
            print(f"[search] ✓ converged at iter {i}: X={X} ticks")
            break

        # Bracket update: signed > 0 ⇒ sim overshoots ⇒ X too high.
        if mean_signed > 0:
            U = X
        else:
            L = X

        if (U - L) < MIN_BRACKET:
            best = _best_iteration(iterations)
            if best is not None:
                final_X = best.X
                print(f"[search] bracket below {MIN_BRACKET} ticks ({U - L}) — "
                      f"stopping; reporting best-tested X={final_X} "
                      f"(L={best.headline_loss_cm:.3f} cm)")
            else:
                final_X = (L + U) // 2
                print(f"[search] bracket too narrow, no good iterations; "
                      f"reporting midpoint {final_X}")
            break

    if final_X is None:
        best = _best_iteration(iterations)
        if best is not None:
            final_X = best.X
            print(f"[search] max_iters ({max_iters}) hit without convergence; "
                  f"reporting best-tested X={final_X} "
                  f"(L={best.headline_loss_cm:.3f} cm)")
        else:
            final_X = (L + U) // 2
            print(f"[search] max_iters hit, no good iterations; "
                  f"reporting midpoint {final_X}")

    return iterations, converged, final_X, L, U


# ─────────────────────────────────────────────────────────────────────────
# Final pass with video
# ─────────────────────────────────────────────────────────────────────────


def final_pass_with_video(
    *,
    root: Path,
    yaml_path: Path,
    recommended_X: int,
    robot_control_root: Path,
) -> Optional[Dict[str, Any]]:
    print(f"\n[final-pass] X={recommended_X} ticks "
          f"(= {recommended_X * 0.002:.3f} s/NAMO step) — with video")
    _patch_control_steps(yaml_path, recommended_X)
    leaves = _discover_leaves(root)
    leaves_with_real = [((e, d), leaf) for (e, d), leaf in sorted(leaves.items())
                        if leaf.real_trials]
    n_ok = n_fail = 0
    for (e, d), leaf in leaves_with_real:
        leaf_dir = root / f"edge{e}" / f"depth{d}"
        ok = _run_sim_for_leaf(
            leaf=leaf, leaf_dir=leaf_dir,
            no_video=False,
            robot_control_root=robot_control_root,
        )
        if ok:
            n_ok += 1
        else:
            n_fail += 1
    print(f"[final-pass] sim: {n_ok} ok, {n_fail} failed; running final diff…")
    try:
        return _run_diff(root=root, robot_control_root=robot_control_root)
    except Exception as exc:
        print(f"[final-pass] ✗ diff failed: {exc!r}")
        return None


# ─────────────────────────────────────────────────────────────────────────
# CLI / orchestration
# ─────────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--root", required=True,
                   help="push-calibration root (e.g. push_calibration/obj_1)")
    p.add_argument("--yaml", required=True,
                   help="path to namo_config_complete_skill15_car_1x.yaml "
                        "(edited per iteration, restored on exit)")
    p.add_argument("--bracket-low", type=int, default=DEFAULT_BRACKET_LOW,
                   help=f"lower bracket bound in ticks (default {DEFAULT_BRACKET_LOW})")
    p.add_argument("--bracket-high", type=int, default=DEFAULT_BRACKET_HIGH,
                   help=f"upper bracket bound in ticks (default {DEFAULT_BRACKET_HIGH})")
    p.add_argument("--eps-multiplier", type=float, default=DEFAULT_EPS_MULTIPLIER,
                   help=f"epsilon = mult × mean(SE_leaf) "
                        f"(default {DEFAULT_EPS_MULTIPLIER})")
    p.add_argument("--epsilon", type=float, default=None,
                   help="override epsilon directly (cm)")
    p.add_argument("--max-iters", type=int, default=DEFAULT_MAX_ITERS,
                   help=f"max bisection iterations (default {DEFAULT_MAX_ITERS})")
    p.add_argument("--out-dir", default=None,
                   help="output dir (default: <root>/tune_control_steps/)")
    p.add_argument("--skip-final-video", action="store_true",
                   help="skip the post-convergence with-video pass")
    p.add_argument("--exclude-edges", default="",
                   help="comma-separated edge_idx values to exclude from the "
                        "bisection signal (e.g. '59' to skip corner-of-face "
                        "outliers). Sim still runs for these leaves so the "
                        "diff report has complete data; only the bisection "
                        "steering changes.")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    root = Path(args.root).resolve()
    yaml_path = Path(args.yaml).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else root / "tune_control_steps"
    out_dir.mkdir(parents=True, exist_ok=True)

    if not root.exists():
        print(f"[tune_control_steps] --root not found: {root}", file=sys.stderr)
        return 2
    if not yaml_path.exists():
        print(f"[tune_control_steps] --yaml not found: {yaml_path}", file=sys.stderr)
        return 2

    robot_control_root = HERE.parent

    # Parse --exclude-edges into a set of ints.
    exclude_edges: set = set()
    if args.exclude_edges:
        try:
            exclude_edges = {int(x.strip()) for x in args.exclude_edges.split(",")
                             if x.strip()}
        except ValueError as exc:
            print(f"[tune_control_steps] --exclude-edges parse error: {exc}",
                  file=sys.stderr)
            return 2
    if exclude_edges:
        print(f"[tune_control_steps] excluding edges from bisection signal: "
              f"{sorted(exclude_edges)}")

    leaves_init = _discover_leaves(root)
    if args.epsilon is not None:
        epsilon = float(args.epsilon)
        mean_se = 0.0
        n_leaves_for_se = 0
        print(f"[tune_control_steps] using explicit --epsilon {epsilon} cm")
    else:
        epsilon, mean_se, n_leaves_for_se = _compute_epsilon(
            leaves_init, args.eps_multiplier, exclude_edges=exclude_edges
        )
        print(f"[tune_control_steps] epsilon = {args.eps_multiplier} × "
              f"{mean_se:.3f} cm (mean SE over {n_leaves_for_se} non-excluded "
              f"leaves) = {epsilon:.3f} cm")

    backup_path = out_dir / f"{yaml_path.name}.tune_backup"
    shutil.copy2(yaml_path, backup_path)
    print(f"[tune_control_steps] backed up {yaml_path.name} → {backup_path}")

    def _restore_yaml_signal(signum, _frame):
        print(f"[tune_control_steps] caught signal {signum}, restoring YAML…",
              file=sys.stderr)
        shutil.copy2(backup_path, yaml_path)
        sys.exit(130)

    signal.signal(signal.SIGINT, _restore_yaml_signal)
    signal.signal(signal.SIGTERM, _restore_yaml_signal)

    iterations: List[Iteration] = []
    converged = False
    final_X: Optional[int] = None
    final_diff_agg: Optional[Dict[str, Any]] = None
    final_L = args.bracket_low
    final_U = args.bracket_high

    try:
        iterations, converged, final_X, final_L, final_U = binary_search_control_steps(
            root=root,
            yaml_path=yaml_path,
            bracket_low=args.bracket_low,
            bracket_high=args.bracket_high,
            epsilon=epsilon,
            max_iters=args.max_iters,
            robot_control_root=robot_control_root,
            exclude_edges=exclude_edges if exclude_edges else None,
        )
        if final_X is not None and not args.skip_final_video:
            final_diff_agg = final_pass_with_video(
                root=root,
                yaml_path=yaml_path,
                recommended_X=final_X,
                robot_control_root=robot_control_root,
            )
    finally:
        shutil.copy2(backup_path, yaml_path)
        print(f"[tune_control_steps] restored {yaml_path.name} from backup")

    summary = {
        "root": str(root),
        "yaml": str(yaml_path),
        "excluded_edges": sorted(exclude_edges),
        "bracket_initial": [int(args.bracket_low), int(args.bracket_high)],
        "bracket_final": [int(final_L), int(final_U)],
        "epsilon_cm": float(epsilon),
        "epsilon_source": "explicit" if args.epsilon is not None else "auto",
        "mean_se_cm": float(mean_se),
        "n_leaves_used_for_se": int(n_leaves_for_se),
        "eps_multiplier": float(args.eps_multiplier),
        "max_iters": int(args.max_iters),
        "converged": bool(converged),
        "recommended_control_steps_per_push": (
            None if final_X is None else int(final_X)
        ),
        "n_iterations": len(iterations),
        "final_diff_headline_loss_cm": (
            None if final_diff_agg is None
            else float(final_diff_agg.get("headline_loss_cm", float("nan")))
        ),
        "iterations": [
            {
                "idx": it.idx, "X": it.X,
                "X_sec_per_step": it.X * 0.002,
                "L_before": it.L_before, "U_before": it.U_before,
                "n_sim_succeeded": it.n_sim_succeeded,
                "n_sim_failed": it.n_sim_failed,
                "headline_loss_cm": it.headline_loss_cm,
                "mean_signed_gap_cm": it.mean_signed_gap_cm,
                "n_leaves_paired": it.n_leaves_paired,
                "leaf_signals": it.leaf_signals,
                "wall_s": it.wall_s,
            }
            for it in iterations
        ],
    }
    summary_path = out_dir / "tune_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"\n[tune_control_steps] summary → {summary_path}")
    if final_X is not None:
        print(f"[tune_control_steps] recommended control_steps_per_push = "
              f"{final_X} ticks  (= {final_X * 0.002:.3f} s/NAMO push step, "
              f"converged={converged})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
