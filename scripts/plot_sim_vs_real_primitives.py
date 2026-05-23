"""Render sim + real push primitives with a SHARED axis scale so they
can be compared visually side-by-side.

Loads:
  - Sim primitive .dat files (square, wide, tall) — 600 primitives each.
  - Real-robot push jsonl trees under robot_control/calibration/obj_<id>/
    — N pushes per object, deltas converted to object-local frame.

Computes a single (xlim, ylim, mag_lim, rot_lim) bound across ALL 5
datasets so each panel uses the same axis range — translation magnitudes,
rotation magnitudes, and 2D-displacement axes all share one scale.

Writes 5 PNGs to --out-dir (default /tmp/primitive_videos/viz_shared/):
  shared_sim_{square,wide,tall}.png
  shared_real_{obj_1,obj_4}.png
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from pathlib import Path
from typing import List, Dict

import matplotlib.pyplot as plt
import numpy as np


NAMO_CPP = Path("/home/dhruv/projects_dhruv/namo/namo_cpp")
ROBOT_CTRL = Path("/home/dhruv/projects_dhruv/namo/robot_control")

SIM_DATS = {
    "square": NAMO_CPP / "data" / "motion_primitives_1x_car_square.dat",
    "wide":   NAMO_CPP / "data" / "motion_primitives_1x_car_wide.dat",
    "tall":   NAMO_CPP / "data" / "motion_primitives_1x_car_tall.dat",
}

REAL_OBJECTS = ["obj_1", "obj_4"]


# ─────────────────────────────────────────────────────────────────────────
# Loaders → list of {delta_x_m, delta_y_m, delta_theta_rad, edge_idx, push_steps}
# ─────────────────────────────────────────────────────────────────────────


def load_sim_dat(path: Path) -> List[Dict]:
    """Read packed binary primitives (header uint32 count, then 14B records)."""
    out: List[Dict] = []
    with open(path, "rb") as f:
        count_bytes = f.read(4)
        count = struct.unpack("I", count_bytes)[0]
        for _ in range(count):
            data = f.read(14)
            if len(data) < 14:
                break
            dx, dy, dth, edge, steps = struct.unpack("fffBB", data)
            out.append({
                "delta_x": float(dx),
                "delta_y": float(dy),
                "delta_theta": float(dth),
                "edge_idx": int(edge),
                "push_steps": int(steps),
            })
    return out


def load_real_pushes(calibration_root: Path, object_id: str,
                     include_stuck: bool = False) -> List[Dict]:
    """Walk calibration/<obj>/**/real/pushes.jsonl, convert to object-local frame."""
    base = calibration_root / object_id
    out: List[Dict] = []
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
                    continue
                theta_rad = math.radians(float(rec["object_pose_before"][2]))
                dx_w = float(rec["delta_pos_cm"][0])
                dy_w = float(rec["delta_pos_cm"][1])
                # World → object-local (rotate by -theta).
                dx_l = math.cos(theta_rad) * dx_w + math.sin(theta_rad) * dy_w
                dy_l = -math.sin(theta_rad) * dx_w + math.cos(theta_rad) * dy_w
                out.append({
                    "delta_x": dx_l / 100.0,
                    "delta_y": dy_l / 100.0,
                    "delta_theta": math.radians(float(rec["delta_theta_deg"])),
                    "edge_idx": int(rec["expected_edge"]),
                    "push_steps": int(rec["expected_push_steps"]),
                })
    return out


# ─────────────────────────────────────────────────────────────────────────
# Plotting with explicit shared axis limits
# ─────────────────────────────────────────────────────────────────────────


def compute_shared_limits(datasets: Dict[str, List[Dict]]) -> Dict[str, tuple]:
    """Return (xy_lim, mag_lim, rot_lim, push_steps_range) across all datasets.

    xy_lim is a single symmetric (-L, +L) bound covering all delta_x and
    delta_y so both axes can share one scale; mag_lim and rot_lim are the
    max translation magnitude and |rotation| respectively.
    """
    all_dx, all_dy, all_dth = [], [], []
    all_push_steps = []
    for name, recs in datasets.items():
        if not recs:
            continue
        all_dx.extend(r["delta_x"] for r in recs)
        all_dy.extend(r["delta_y"] for r in recs)
        all_dth.extend(r["delta_theta"] for r in recs)
        all_push_steps.extend(r["push_steps"] for r in recs)

    L = max(max(abs(x) for x in all_dx), max(abs(y) for y in all_dy)) * 1.1
    mag_lim = math.sqrt(max(x * x + y * y for x, y in zip(all_dx, all_dy))) * 1.1
    rot_lim = max(abs(t) for t in all_dth) * 1.1
    ps_lo = min(all_push_steps)
    ps_hi = max(all_push_steps)
    return {
        "xy_lim": (-L, L),
        "mag_lim": (0.0, mag_lim),
        "rot_lim": (0.0, rot_lim),
        "push_steps_lim": (ps_lo, ps_hi),
    }


def plot_with_shared_limits(
    title: str,
    primitives: List[Dict],
    lims: Dict[str, tuple],
    output_file: Path,
) -> None:
    """Same 5-panel layout as visualize_primitives.plot_primitives, but
    with explicit axis limits so multiple PNGs share a coordinate scale."""
    data = np.array([(p["delta_x"], p["delta_y"], p["delta_theta"],
                      p["edge_idx"], p["push_steps"]) for p in primitives])
    dx, dy, dth = data[:, 0], data[:, 1], data[:, 2]
    edge_idx = data[:, 3].astype(int)
    push_steps = data[:, 4].astype(int)
    mag = np.sqrt(dx * dx + dy * dy)

    xy_lim = lims["xy_lim"]
    mag_lim = lims["mag_lim"]
    rot_lim = lims["rot_lim"]
    ps_lim = lims["push_steps_lim"]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f"{title}  ({len(primitives)} pushes)", fontsize=16)

    # 1. Displacement vectors by edge
    ax = axes[0, 0]
    sc = ax.scatter(dx, dy, c=edge_idx, cmap="tab20", alpha=0.7, s=30)
    ax.set_xlim(*xy_lim); ax.set_ylim(*xy_lim)
    ax.set_aspect("equal")
    ax.set_xlabel("Delta X (m, object-local)")
    ax.set_ylabel("Delta Y (m, object-local)")
    ax.set_title("Displacement Vectors by Edge Index")
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label="Edge Index")

    # 2. Displacement vectors by push steps
    ax = axes[0, 1]
    sc = ax.scatter(dx, dy, c=push_steps, cmap="viridis", alpha=0.7, s=30,
                    vmin=ps_lim[0], vmax=ps_lim[1])
    ax.set_xlim(*xy_lim); ax.set_ylim(*xy_lim)
    ax.set_aspect("equal")
    ax.set_xlabel("Delta X (m, object-local)")
    ax.set_ylabel("Delta Y (m, object-local)")
    ax.set_title("Displacement Vectors by Push Steps")
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label="Push Steps")

    # 3. Translation magnitude vs |rotation|
    ax = axes[0, 2]
    sc = ax.scatter(mag, np.abs(dth), c=edge_idx, cmap="tab20", alpha=0.7, s=30)
    ax.set_xlim(*mag_lim); ax.set_ylim(*rot_lim)
    ax.set_xlabel("Translation Magnitude (m)")
    ax.set_ylabel("|Rotation| (rad)")
    ax.set_title("Translation vs Rotation")
    ax.grid(True, alpha=0.3)
    plt.colorbar(sc, ax=ax, label="Edge Index")

    # 4. Magnitude vs push steps, per edge (cap at 12 for readability)
    ax = axes[1, 0]
    unique_edges = sorted(set(edge_idx.tolist()))
    for edge in unique_edges[:12]:
        m = edge_idx == edge
        if np.any(m):
            ax.plot(push_steps[m], mag[m], "o-", label=f"Edge {edge}", alpha=0.7)
    ax.set_xlim(ps_lim[0] - 0.5, ps_lim[1] + 0.5)
    ax.set_ylim(*mag_lim)
    ax.set_xlabel("Push Steps")
    ax.set_ylabel("Translation Magnitude (m)")
    ax.set_title("Translation Magnitude vs Push Steps")
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", fontsize=8)

    # 5. 2D displacement histogram
    ax = axes[1, 1]
    # Fixed bin edges so all PNGs use identical bin layout
    bins = np.linspace(xy_lim[0], xy_lim[1], 30)
    ax.hist2d(dx, dy, bins=[bins, bins], cmap="Blues", alpha=0.85)
    ax.set_xlim(*xy_lim); ax.set_ylim(*xy_lim)
    ax.set_aspect("equal")
    ax.set_xlabel("Delta X (m, object-local)")
    ax.set_ylabel("Delta Y (m, object-local)")
    ax.set_title("Displacement Distribution (2D Histogram)")

    axes[1, 2].axis("off")

    output_file.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plot] wrote {output_file} (xy ±{xy_lim[1]:.3f} m, mag ≤{mag_lim[1]:.3f} m, |rot| ≤{rot_lim[1]:.3f} rad)")


# ─────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--out-dir", default="/tmp/primitive_videos/viz_shared",
        help="Output directory for PNGs.",
    )
    p.add_argument(
        "--calibration-root",
        default=str(ROBOT_CTRL / "calibration"),
        help="Real-robot calibration root.",
    )
    p.add_argument(
        "--include-stuck", action="store_true",
        help="Include real pushes flagged stuck=true.",
    )
    args = p.parse_args()

    out_dir = Path(args.out_dir).resolve()
    cal_root = Path(args.calibration_root).resolve()

    # Load all 5 datasets
    datasets: Dict[str, List[Dict]] = {}
    for scene, dat in SIM_DATS.items():
        if dat.exists():
            datasets[f"sim_{scene}"] = load_sim_dat(dat)
            print(f"[load] sim_{scene}: {len(datasets[f'sim_{scene}'])} primitives")
        else:
            print(f"[load] sim_{scene}: missing {dat}")
    for obj in REAL_OBJECTS:
        datasets[f"real_{obj}"] = load_real_pushes(cal_root, obj,
                                                   include_stuck=args.include_stuck)
        print(f"[load] real_{obj}: {len(datasets[f'real_{obj}'])} pushes")

    # Compute shared limits across everything
    lims = compute_shared_limits(datasets)
    print(f"[lims] xy={lims['xy_lim']}  mag={lims['mag_lim']}  "
          f"rot={lims['rot_lim']}  push_steps={lims['push_steps_lim']}")

    # Render full datasets
    for name, recs in datasets.items():
        if not recs:
            continue
        title = name.replace("_", " ").upper()
        png = out_dir / f"shared_{name}.png"
        plot_with_shared_limits(title, recs, lims, png)

    # Filtered sim plots: keep only primitives whose (edge, push_steps) pair
    # was actually tried in real. Lets the user compare "what sim predicted
    # for the exact conditions reality tested" against "what real produced".
    real_pairs = set()
    for obj in REAL_OBJECTS:
        for r in datasets.get(f"real_{obj}", []):
            real_pairs.add((int(r["edge_idx"]), int(r["push_steps"])))
    if real_pairs:
        for scene in SIM_DATS:
            recs = datasets.get(f"sim_{scene}", [])
            if not recs:
                continue
            filtered = [
                r for r in recs
                if (int(r["edge_idx"]), int(r["push_steps"])) in real_pairs
            ]
            print(f"[filter] sim_{scene}: {len(recs)} → {len(filtered)} "
                  f"matching {len(real_pairs)} real (edge, push_steps) pairs")
            if not filtered:
                continue
            png = out_dir / f"shared_sim_{scene}_realtried.png"
            title = f"SIM {scene.upper()} (real-tried subset)"
            plot_with_shared_limits(title, filtered, lims, png)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
