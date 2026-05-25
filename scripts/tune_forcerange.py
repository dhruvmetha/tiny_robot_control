"""Binary-search the wheel forcerange to minimize the sim-vs-real push gap.

Operates on the push-calibration tree layout:

    <root>/                               e.g. push_calibration/obj_1/
    ├── edge<E>/depth<D>/
    │   ├── spec.yaml
    │   ├── trial1/                       real push 1
    │   ├── trial2/
    │   ├── trial3/
    │   └── sim/                          ← overwritten each iteration
    └── ...

Algorithm — mirrors find_sim_fraction.py's Tier 1 bisection:

  Search variable: wheel actuator forcerange (Nm, symmetric ±X) on the two
  <velocity> actuators in little_car.xml.

  Convergence signal (signed): for each leaf, project sim's Δobject and the
  mean of real trials' Δobject onto the leaf's push direction. Take the
  difference (sim − real_mean). Average across leaves. Positive ⇒ sim
  overshoots real along the intended push direction ⇒ forcerange too high.
  Monotone in forcerange below saturation.

  Convergence threshold ε is data-driven:
      SE_leaf      = std(projected real Δobject across the leaf's trials) / √n
      mean_SE      = mean over leaves of SE_leaf
      ε            = eps_multiplier · mean_SE        (default eps_multiplier=2)
  So we stop when sim is within "noise" of real's mean. Same shape as Tier 1.

  Reported loss (unsigned): the diff's headline_loss_cm is tracked per
  iteration; if the signed signal converges at a non-minimum, we recommend
  the best-tested X by L instead.

Per iteration:
  1. Patch the XML wheel actuators to forcerange="-X X".
  2. For each leaf (with at least 1 real trial): subprocess
     execute_sim_push.py --no-video --from-real-run <leaf>/trial1
                          --push-index 0 --diag-path <leaf> --run-name sim
     This OVERWRITES <leaf>/sim/.
  3. Subprocess diff_real_vs_sim.py --root <root>.
  4. Read <root>/diff/_aggregate.json's headline_loss_cm.
  5. Compute mean signed gap directly from the discovered leaves.
  6. Update bracket, check convergence, record in tune_summary.json.

After convergence:
  1. Patch XML to the recommended X.
  2. Re-run execute_sim_push WITHOUT --no-video for each leaf → MP4s.
  3. Final canonical diff_real_vs_sim.

XML state is saved at entry and restored at exit (try/finally + SIGINT/
SIGTERM handler).

Usage:
    python scripts/tune_forcerange.py \\
        --root push_calibration/obj_1 \\
        --xml namo_cpp/test_xml/little-car-modeling-package/assets/mjcf/little_car.xml \\
        [--bracket-low 0.10] [--bracket-high 1.50] \\
        [--eps-multiplier 2.0] [--max-iters 10] \\
        [--out-dir <dir>]
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

# Reuse the new tree-walking discovery from diff_real_vs_sim.
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
DEFAULT_BRACKET_LOW_N = 0.10
DEFAULT_BRACKET_HIGH_N = 1.50
MIN_BRACKET_N = 0.02           # stop if (U − L) drops below this


# ─────────────────────────────────────────────────────────────────────────
# XML patching
# ─────────────────────────────────────────────────────────────────────────

_FORCERANGE_RE = re.compile(
    r'(<velocity\s+[^>]*?forcerange=")(-?\d*\.?\d+\s+-?\d*\.?\d+)(")'
)


def _patch_forcerange(xml_path: Path, half_range_n: float) -> int:
    """Replace forcerange='...' on every <velocity> actuator with
    forcerange='-X X'. Returns the count of replacements. Raises if zero."""
    src = xml_path.read_text()
    new_attr = f"-{half_range_n:.6f} {half_range_n:.6f}"

    def _replace(m: re.Match) -> str:
        return f"{m.group(1)}{new_attr}{m.group(3)}"

    new_src, count = _FORCERANGE_RE.subn(_replace, src)
    if count == 0:
        raise ValueError(
            f"[tune_forcerange] no <velocity ... forcerange='...'> match in "
            f"{xml_path}. Is this the correct car XML?"
        )
    xml_path.write_text(new_src)
    return count


# ─────────────────────────────────────────────────────────────────────────
# Subprocess runners
# ─────────────────────────────────────────────────────────────────────────


def _run_sim_for_leaf(
    *,
    leaf: LeafData,
    leaf_dir: Path,
    no_video: bool,
    robot_control_root: Path,
) -> bool:
    """Run execute_sim_push for one leaf. Returns True if subprocess
    succeeded (rc 0). Source real trial is trial1 by convention."""
    # Find trial1 path. _discover_leaves sorts trials by label, so [0] is
    # trial1 IF the leaf has at least one real trial.
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
        cmd,
        cwd=str(robot_control_root),
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        tail = "\n      ".join(proc.stderr.strip().splitlines()[-5:])
        print(
            f"      ✗ sim FAILED for edge{leaf.edge_idx}/depth{leaf.depth} "
            f"(rc={proc.returncode}):\n      {tail}",
            flush=True,
        )
        return False
    return True


def _run_diff(
    *,
    root: Path,
    robot_control_root: Path,
) -> Dict[str, Any]:
    """Run diff_real_vs_sim.py against the tree; return the aggregate dict."""
    cmd = [
        sys.executable,
        "scripts/diff_real_vs_sim.py",
        "--root", str(root),
    ]
    proc = subprocess.run(
        cmd,
        cwd=str(robot_control_root),
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        tail = "\n  ".join(proc.stderr.strip().splitlines()[-10:])
        raise RuntimeError(
            f"[tune_forcerange] diff_real_vs_sim failed (rc={proc.returncode}). "
            f"stderr tail:\n  {tail}"
        )
    return json.loads((root / "diff" / "_aggregate.json").read_text())


# ─────────────────────────────────────────────────────────────────────────
# Direction signal + standard error
# ─────────────────────────────────────────────────────────────────────────


def _leaf_push_direction(real_trials: List[Trial]) -> Optional[Tuple[float, float]]:
    """Mean push direction across the leaf's real trials. Equivalent to the
    sum of Δobject vectors, renormalized. Returns None if magnitude is zero."""
    sx = sum(t.delta_object_pos_cm[0] for t in real_trials)
    sy = sum(t.delta_object_pos_cm[1] for t in real_trials)
    mag = math.hypot(sx, sy)
    if mag < 1e-9:
        return None
    return (sx / mag, sy / mag)


def _project(vec: Tuple[float, float], unit: Tuple[float, float]) -> float:
    return vec[0] * unit[0] + vec[1] * unit[1]


def _sample_std(values: List[float]) -> float:
    """Sample standard deviation (n-1). Returns 0 for n<2."""
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
    real_mean_proj_cm: float       # mean of projected real Δobject
    real_se_proj_cm: float         # std / sqrt(n) of projected real Δobject
    sim_proj_cm: float             # projected sim Δobject
    signed_gap_cm: float           # sim − real_mean
    push_dir: Tuple[float, float]  # for logging / debugging


def _compute_leaf_signals(leaves: Dict[Tuple[int, int], LeafData]) -> List[LeafSignal]:
    """One signed-gap reading per leaf that has both real trials AND sim."""
    out: List[LeafSignal] = []
    for (e, d), leaf in sorted(leaves.items()):
        if not leaf.real_trials or leaf.sim_trial is None:
            continue
        push_dir = _leaf_push_direction(leaf.real_trials)
        if push_dir is None:
            print(f"  ⚠ edge{e}/depth{d}: zero net real displacement, skipping",
                  flush=True)
            continue
        real_projs = [_project(t.delta_object_pos_cm, push_dir) for t in leaf.real_trials]
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


def _compute_epsilon(leaves: Dict[Tuple[int, int], LeafData],
                     eps_multiplier: float) -> Tuple[float, float, int]:
    """ε = eps_multiplier · mean(SE_leaf across leaves with ≥2 real trials).
    Returns (epsilon, mean_SE, n_leaves_used). Computed once from real data
    before the search starts — it doesn't depend on sim."""
    per_leaf_se: List[float] = []
    for (_e, _d), leaf in leaves.items():
        if len(leaf.real_trials) < 2:
            continue
        push_dir = _leaf_push_direction(leaf.real_trials)
        if push_dir is None:
            continue
        projs = [_project(t.delta_object_pos_cm, push_dir) for t in leaf.real_trials]
        se = _sample_std(projs) / math.sqrt(len(projs))
        per_leaf_se.append(se)
    if not per_leaf_se:
        raise ValueError(
            "[tune_forcerange] no leaves have ≥2 real trials with non-zero "
            "displacement; cannot derive epsilon from data. Collect more "
            "real trials or pass an explicit --epsilon."
        )
    mean_se = sum(per_leaf_se) / len(per_leaf_se)
    return eps_multiplier * mean_se, mean_se, len(per_leaf_se)


# ─────────────────────────────────────────────────────────────────────────
# Iteration record
# ─────────────────────────────────────────────────────────────────────────


@dataclass
class Iteration:
    idx: int
    X: float
    L_before: float
    U_before: float
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
# Binary search
# ─────────────────────────────────────────────────────────────────────────


def binary_search_forcerange(
    *,
    root: Path,
    xml_path: Path,
    bracket_low: float,
    bracket_high: float,
    epsilon: float,
    max_iters: int,
    robot_control_root: Path,
) -> Tuple[List[Iteration], bool, Optional[float], float, float]:
    """Bisect [L, U] until |mean_signed_gap| ≤ epsilon or other stop condition.

    Returns (iterations, converged, final_X, final_L, final_U)."""
    L, U = float(bracket_low), float(bracket_high)
    iterations: List[Iteration] = []
    converged = False
    final_X: Optional[float] = None

    print(f"[search] bracket=[{L:.4f}, {U:.4f}] Nm, ε={epsilon:.3f} cm, "
          f"max_iters={max_iters}")

    for i in range(1, max_iters + 1):
        X = (L + U) / 2.0
        t0 = time.time()
        print(f"\n[iter {i:02d}] X={X:.4f} Nm  (L={L:.4f}, U={U:.4f})")

        _patch_forcerange(xml_path, X)

        # Discover the tree fresh so we can iterate by leaf, but only use it
        # for the (edge, depth, trial1_path) info — we don't need sim
        # results loaded yet.
        leaves_pre = _discover_leaves(root)
        leaves_with_real = [
            ((e, d), leaf) for (e, d), leaf in sorted(leaves_pre.items())
            if leaf.real_trials
        ]
        print(f"  running sim for {len(leaves_with_real)} leaves…")
        n_ok = 0
        n_fail = 0
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
                n_leaves_paired=0,
                wall_s=time.time() - t0,
            ))
            # Can't decide bracket direction without a signal; bias upward.
            L = X
            continue

        headline_L = float(agg.get("headline_loss_cm", float("nan")))

        # Reload to pick up sim trials and compute the signed signal.
        leaves_post = _discover_leaves(root)
        signals = _compute_leaf_signals(leaves_post)
        if not signals:
            print(f"  ✗ no paired leaves with non-zero displacement; bias L=X")
            iterations.append(Iteration(
                idx=i, X=X, L_before=L, U_before=U,
                n_sim_succeeded=n_ok, n_sim_failed=n_fail,
                headline_loss_cm=headline_L,
                mean_signed_gap_cm=float("nan"),
                n_leaves_paired=0,
                wall_s=time.time() - t0,
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
            print(f"[search] ✓ converged at iter {i}: X={X:.4f} Nm")
            break

        # Bracket update: mean_signed > 0 ⇒ sim overshoots ⇒ X too high.
        if mean_signed > 0:
            U = X
        else:
            L = X

        if (U - L) < MIN_BRACKET_N:
            best = _best_iteration(iterations)
            if best is not None:
                final_X = best.X
                print(
                    f"[search] bracket below {MIN_BRACKET_N} Nm ({U - L:.4f}) — "
                    f"stopping; reporting best-tested X={final_X:.4f} Nm "
                    f"(L={best.headline_loss_cm:.3f} cm)"
                )
            else:
                final_X = (L + U) / 2.0
                print(f"[search] bracket too narrow, no good iterations; "
                      f"reporting midpoint {final_X:.4f}")
            break

    if final_X is None:
        best = _best_iteration(iterations)
        if best is not None:
            final_X = best.X
            print(f"[search] max_iters ({max_iters}) hit without convergence; "
                  f"reporting best-tested X={final_X:.4f} Nm "
                  f"(L={best.headline_loss_cm:.3f} cm)")
        else:
            final_X = (L + U) / 2.0
            print(f"[search] max_iters hit, no good iterations; "
                  f"reporting midpoint {final_X:.4f}")

    return iterations, converged, final_X, L, U


# ─────────────────────────────────────────────────────────────────────────
# Final pass with video
# ─────────────────────────────────────────────────────────────────────────


def final_pass_with_video(
    *,
    root: Path,
    xml_path: Path,
    recommended_X: float,
    robot_control_root: Path,
) -> Optional[Dict[str, Any]]:
    """Re-patch XML to recommended X, re-run sim WITH video for every leaf,
    re-run diff for the canonical final report."""
    print(f"\n[final-pass] X={recommended_X:.4f} Nm — with video")
    _patch_forcerange(xml_path, recommended_X)
    leaves = _discover_leaves(root)
    leaves_with_real = [
        ((e, d), leaf) for (e, d), leaf in sorted(leaves.items())
        if leaf.real_trials
    ]
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
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", required=True,
                   help="push-calibration root (e.g. push_calibration/obj_1)")
    p.add_argument("--xml", required=True,
                   help="path to little_car.xml (edited per iteration, restored on exit)")
    p.add_argument("--bracket-low", type=float, default=DEFAULT_BRACKET_LOW_N,
                   help=f"lower bracket bound in Nm (default {DEFAULT_BRACKET_LOW_N})")
    p.add_argument("--bracket-high", type=float, default=DEFAULT_BRACKET_HIGH_N,
                   help=f"upper bracket bound in Nm (default {DEFAULT_BRACKET_HIGH_N})")
    p.add_argument("--eps-multiplier", type=float, default=DEFAULT_EPS_MULTIPLIER,
                   help=f"epsilon = mult × mean(SE_leaf) "
                        f"(default {DEFAULT_EPS_MULTIPLIER})")
    p.add_argument("--epsilon", type=float, default=None,
                   help="override epsilon directly (cm); takes precedence over "
                        "--eps-multiplier")
    p.add_argument("--max-iters", type=int, default=DEFAULT_MAX_ITERS,
                   help=f"max bisection iterations (default {DEFAULT_MAX_ITERS})")
    p.add_argument("--out-dir", default=None,
                   help="output dir (default: <root>/tune_forcerange/)")
    p.add_argument("--skip-final-video", action="store_true",
                   help="skip the post-convergence with-video pass")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    root = Path(args.root).resolve()
    xml_path = Path(args.xml).resolve()
    out_dir = Path(args.out_dir).resolve() if args.out_dir else root / "tune_forcerange"
    out_dir.mkdir(parents=True, exist_ok=True)

    if not root.exists():
        print(f"[tune_forcerange] --root not found: {root}", file=sys.stderr)
        return 2
    if not xml_path.exists():
        print(f"[tune_forcerange] --xml not found: {xml_path}", file=sys.stderr)
        return 2

    robot_control_root = HERE.parent

    # Compute epsilon up front (depends only on real data, doesn't change
    # across iterations).
    leaves_init = _discover_leaves(root)
    if args.epsilon is not None:
        epsilon = float(args.epsilon)
        mean_se = 0.0
        n_leaves_for_se = 0
        print(f"[tune_forcerange] using explicit --epsilon {epsilon} cm")
    else:
        epsilon, mean_se, n_leaves_for_se = _compute_epsilon(
            leaves_init, args.eps_multiplier
        )
        print(f"[tune_forcerange] epsilon = {args.eps_multiplier} × "
              f"{mean_se:.3f} cm (mean SE over {n_leaves_for_se} leaves) "
              f"= {epsilon:.3f} cm")

    # Backup XML for restoration.
    backup_path = out_dir / f"{xml_path.name}.tune_backup"
    shutil.copy2(xml_path, backup_path)
    print(f"[tune_forcerange] backed up {xml_path.name} → {backup_path}")

    def _restore_xml_signal(signum, _frame):
        print(f"[tune_forcerange] caught signal {signum}, restoring XML…",
              file=sys.stderr)
        shutil.copy2(backup_path, xml_path)
        sys.exit(130)

    signal.signal(signal.SIGINT, _restore_xml_signal)
    signal.signal(signal.SIGTERM, _restore_xml_signal)

    iterations: List[Iteration] = []
    converged = False
    final_X: Optional[float] = None
    final_diff_agg: Optional[Dict[str, Any]] = None
    final_L = args.bracket_low
    final_U = args.bracket_high

    try:
        iterations, converged, final_X, final_L, final_U = binary_search_forcerange(
            root=root,
            xml_path=xml_path,
            bracket_low=args.bracket_low,
            bracket_high=args.bracket_high,
            epsilon=epsilon,
            max_iters=args.max_iters,
            robot_control_root=robot_control_root,
        )
        if final_X is not None and not args.skip_final_video:
            final_diff_agg = final_pass_with_video(
                root=root,
                xml_path=xml_path,
                recommended_X=final_X,
                robot_control_root=robot_control_root,
            )
    finally:
        shutil.copy2(backup_path, xml_path)
        print(f"[tune_forcerange] restored {xml_path.name} from backup")

    summary = {
        "root": str(root),
        "xml": str(xml_path),
        "bracket_initial_n": [float(args.bracket_low), float(args.bracket_high)],
        "bracket_final_n": [float(final_L), float(final_U)],
        "epsilon_cm": float(epsilon),
        "epsilon_source": "explicit" if args.epsilon is not None else "auto",
        "mean_se_cm": float(mean_se),
        "n_leaves_used_for_se": int(n_leaves_for_se),
        "eps_multiplier": float(args.eps_multiplier),
        "max_iters": int(args.max_iters),
        "converged": bool(converged),
        "recommended_forcerange_n": (
            None if final_X is None else float(final_X)
        ),
        "n_iterations": len(iterations),
        "final_diff_headline_loss_cm": (
            None if final_diff_agg is None
            else float(final_diff_agg.get("headline_loss_cm", float("nan")))
        ),
        "iterations": [
            {
                "idx": it.idx, "X": it.X,
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
    print(f"\n[tune_forcerange] summary → {summary_path}")
    if final_X is not None:
        print(f"[tune_forcerange] recommended forcerange = "
              f"±{final_X:.4f} Nm  (converged={converged})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
