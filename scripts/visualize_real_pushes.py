"""Generate trajectory-style visualizations of real-robot push outcomes.

Walks ``robot_control/calibration/<object_id>/**/real/pushes.jsonl`` and
plots one PNG per object using the same 5-panel layout as
``namo_cpp/tools/visualize_primitives.py`` (the sim primitive plotter), so
real-robot push outcomes can be visually compared side-by-side with the
calibrated sim primitive DB.

Each push record stores ``delta_pos_cm`` and ``delta_theta_deg`` in the
WORLD frame, but sim primitives are recorded in the OBJECT-LOCAL frame
(object placed at canonical pose for each primitive). We rotate each
real delta into object-local by -theta_before so the two visualizations
share a coordinate convention.

Pushes flagged ``stuck: true`` are excluded by default — they record near-
zero motion (object didn't slide) and would smear the 2D distribution
plot. Pass --include-stuck to keep them in.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


# Reuse the sim plotter's plot_primitives() so the layout is identical.
NAMO_TOOLS = Path("/home/dhruv/projects_dhruv/namo/namo_cpp/tools")
sys.path.insert(0, str(NAMO_TOOLS))
from visualize_primitives import plot_primitives  # noqa: E402


def load_real_pushes(calibration_root: Path, object_id: str,
                     include_stuck: bool = False) -> list[dict]:
    """Walk one object's calibration trees and collect pushes.

    Returns a list of dicts with the keys plot_primitives expects:
        delta_x, delta_y      (meters, object-local frame)
        delta_theta           (radians, signed)
        edge_idx, push_steps  (ints)
    """
    base = calibration_root / object_id
    if not base.exists():
        print(f"[load] {object_id}: dir not found at {base}")
        return []

    pushes: list[dict] = []
    stuck_dropped = 0
    for jsonl in base.rglob("real/pushes.jsonl"):
        with open(jsonl) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if not include_stuck and rec.get("stuck", False):
                    stuck_dropped += 1
                    continue

                # World-frame delta (cm) → object-local delta (m).
                # Object orientation at push-start: theta_before (degrees).
                # R(-theta) maps world → object-local.
                theta_before_deg = float(rec["object_pose_before"][2])
                theta_rad = math.radians(theta_before_deg)
                dx_w = float(rec["delta_pos_cm"][0])
                dy_w = float(rec["delta_pos_cm"][1])
                dx_local_cm = math.cos(theta_rad) * dx_w + math.sin(theta_rad) * dy_w
                dy_local_cm = -math.sin(theta_rad) * dx_w + math.cos(theta_rad) * dy_w

                pushes.append({
                    "delta_x": dx_local_cm / 100.0,                # meters
                    "delta_y": dy_local_cm / 100.0,                # meters
                    "delta_theta": math.radians(float(rec["delta_theta_deg"])),
                    "edge_idx": int(rec["expected_edge"]),
                    "push_steps": int(rec["expected_push_steps"]),
                })

    print(f"[load] {object_id}: {len(pushes)} pushes "
          f"(+ {stuck_dropped} stuck dropped)")
    return pushes


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--calibration-root",
        default="/home/dhruv/projects_dhruv/namo/robot_control/calibration",
        help="Directory containing obj_<id>/ subtrees.",
    )
    p.add_argument(
        "--out-dir", default="/tmp/primitive_videos/viz",
        help="Output directory for PNGs (default: /tmp/primitive_videos/viz).",
    )
    p.add_argument(
        "--objects", nargs="+", default=["obj_1", "obj_4"],
        help="Object IDs to plot (default: obj_1 obj_4).",
    )
    p.add_argument(
        "--include-stuck", action="store_true",
        help="Include pushes flagged stuck=true (default: drop them).",
    )
    args = p.parse_args()

    calibration_root = Path(args.calibration_root).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    for obj in args.objects:
        pushes = load_real_pushes(
            calibration_root, obj, include_stuck=args.include_stuck
        )
        if not pushes:
            print(f"[plot] {obj}: skipping — no pushes")
            continue
        png = out_dir / f"real_pushes_{obj}.png"
        stats = out_dir / f"real_pushes_{obj}_stats.txt"
        plot_primitives(pushes, output_file=str(png), stats_file=str(stats))
        print(f"[plot] {obj}: wrote {png}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
