#!/usr/bin/env python3
"""Turn a build-sheet CSV into one printable card per scene, and check the rows.

Reading a table of centres and bearings and then placing bars by hand is where
90-degree errors come from. A drawing of the finished layout is faster to work
from and much harder to misread. This makes one PNG per build_id and refuses to
draw a scene whose rows do not describe something buildable.

CSV columns, as agreed with the scene generator:
    build_id, item, marker_hint, centre_x_cm, centre_y_cm,
    long_axis_bearing_deg, long_cm, short_cm, height_cm, n_bricks,
    robot_start_x_cm, robot_start_y_cm, goal_x_cm, goal_y_cm,
    solve_rate, tried, valid_1push, valid_first_push

``long_axis_bearing_deg`` is the bearing of the item's LONG side in world,
counter-clockwise from +X. It is the only angle to place by. Local-frame yaw is
deliberately absent, because shipping both invites placing by the wrong one.

Usage:
    python scripts/make_build_cards.py sheet.csv --out-dir cards/
    python scripts/make_build_cards.py sheet.csv --check-only
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict, deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
from matplotlib.patches import Circle, Rectangle

import numpy as np

from robot_control.controller.edge_points import get_edge_point
from robot_control.core.types import ObjectPose
from robot_control.planner.reachability_filter import (
    EDGE_COUNT,
    GRID_RESOLUTION_M,
    POINTS_PER_FACE,
    PUSH_OFFSET_MARGIN_CM,
    robot_inflation_radius_cm,
)
from robot_control.utils.wavefront import WavefrontConfig, WavefrontPlanner
from robot_control.utils.wavefront_inflation_config import get_wavefront_inflation_config

# ─── Named constants ────────────────────────────────────────────────────

# Interior of the taped workspace, matching config/real.yaml `workspace`.
WORKSPACE_W_CM = 49.0
WORKSPACE_H_CM = 77.5

# Robot footprint from config/real.yaml `robot`.
ROBOT_W_CM = 7.0
ROBOT_H_CM = 7.0

# Clearance the start pose must have from every item. The sheet gives a start
# POSITION and no heading, so a person may set the car down at any yaw and the
# check has to hold for all of them. That is the circumscribed radius, not the
# inflation radius: the wavefront inflates by max(hx, hy) = 3.5 because it
# models the car as a disc for PLANNING, while a 7x7 square's corners reach
# hypot(3.5, 3.5). Checking a square at yaw 0 found 1 unplaceable scene in 600;
# checking the circumscribed radius found 10. Planning inflation and placement
# clearance are different questions about the same robot. Measured 2026-08-22.
ROBOT_CIRCUMSCRIBED_R_CM = math.hypot(ROBOT_W_CM / 2.0, ROBOT_H_CM / 2.0)

# How far outside the taped interior an item may sit before it is refused. A
# bar touching the tape is fine; one hanging over the edge is not buildable.
EDGE_TOLERANCE_CM = 0.5

# Two placed items may share this much overlap before it counts as a clash.
# Non-zero because centres and bearings are rounded to 0.1 in the sheet, so a
# scene can graze by a few tenths without meaning anything. Applied to the
# robot's start footprint too: checking that one at exactly zero refused a
# scene overlapping by 0.05 cm, which is smaller than the sheet's precision.
OVERLAP_TOLERANCE_CM = 0.3

# How many contacts may change class before a checksum mismatch stops being a
# rasterisation tie. Both sides walk the same rule over their own grid, so a
# pose sitting on the reachable/cut-off boundary can land either way. A real
# placement error moves far more: one bearing 90 degrees out moved 30 of
# easy_000's 60 contacts. Measured 2026-08-22 over 600 scenes, where the only
# mismatch anywhere moved 2. Such a scene is still refused, since its own tier
# label is noise-sensitive, but it is named a tie so it can be dropped rather
# than debugged.
CHECKSUM_TIE_TOLERANCE_CONTACTS = 3

# Radius drawn around the robot start and the goal marker on the card.
GOAL_MARKER_RADIUS_CM = 2.0

# The three checksum columns. Present means the sheet claims a contact
# breakdown for the blocker, which this script recomputes and compares. A
# 90-degree bearing slip produces a layout that is geometrically valid and
# simply describes a different scene, so no other check here can see it. This
# one can: rotating one bar in easy_000 moved the counts from 18/15/27 to
# 48/0/12. Measured 2026-08-22.
CHECKSUM_COLUMNS = ("n_contacts_reachable", "n_contacts_cutoff", "n_contacts_collision")

# Columns this script reads by name. Checked once at load so a rename or a
# dropped column fails with the missing name rather than a KeyError forty
# scenes in. Column ORDER is deliberately not checked: csv.DictReader is
# name-based, so the generator inserting robot_start_bearing_deg at position 15
# and shifting six columns after it cost nothing here. A positional parser
# would have read goal_x_cm as a bearing and drawn 600 wrong cards in silence.
REQUIRED_COLUMNS = (
    "build_id", "item", "marker_hint", "centre_x_cm", "centre_y_cm",
    "long_axis_bearing_deg", "long_cm", "short_cm", "n_bricks",
    "robot_start_x_cm", "robot_start_y_cm", "robot_start_bearing_deg",
    "goal_x_cm", "goal_y_cm", "push_kind",
)

# push_kind values, and what each means to a person watching the robot.
# On a needs_2_chain scene the first correct push leaves the region SHUT. Watch
# it without knowing that and you call a correct move a failure, and stop the
# run. The generator states this per scene rather than leaving it to be read
# out of valid_1push, and the two agree on all 600 scenes (checked 2026-08-22).
PUSH_KIND_NOTE = {
    "one_push": "ONE PUSH OPENS IT\nthe region should be through after push 1",
    "needs_2_chain": "NEEDS A 2-CHAIN\npush 1 is a SETUP and leaves the region"
                     " shut. Not a failure, let it run",
}

# Contacts on the blocker, over four faces. The counts must add up to this.
CONTACTS_PER_OBJECT = EDGE_COUNT

# Neighbours the reachability flood fill steps through. EIGHT, matching the
# planner's own BFS at namo_cpp `src/wavefront/wavefront_planner.cpp:563`. The
# car drives diagonally, so a 4-connected fill seals gaps it can actually pass.
# This was 4-connected first and it cost one scene in the hard tier: hard_065
# came out 22/6/30 against a sheet value of 24/6/30, and two contacts behind a
# diagonal opening read as cut off. Across the tier, 4-connected reproduced the
# sheet on 99 of 100 scenes and 8-connected on 100 of 100. Measured 2026-08-22.
FLOOD_FILL_NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1),
                         (1, 1), (1, -1), (-1, 1), (-1, -1))

# The checksum only compares between two sides that rasterise the same way.
# Holding the inflation fixed and recomputing all 600 scenes at 2 mm instead of
# 5 mm shifts 293 of them by at least one contact. So exact agreement with the
# scene generator reflects a SHARED CONVENTION, not an invariant of the
# geometry, and a resolution change would start refusing roughly half the pool
# while looking like corrupted data. Pinned so the change is loud.
#
# Resolution is not the only such convention. Connectivity is the other, and it
# was the one that actually differed: see FLOOD_FILL_NEIGHBOURS. A finer grid
# converges to the 8-connected answer, which is why a resolution sweep looked
# like it explained a disagreement that connectivity actually caused. CHECK
# CONNECTIVITY BEFORE RESOLUTION.
#
# Note what this 5 mm is NOT. The deployed planner runs its reachability grid at
# 1 cm (`high_level_resolution`, namo_config_complete_skill15_car_1x.yaml:28),
# so the checksum reproduces neither the planner's rasterisation nor anything
# the robot sees. It only proves that the rows on this sheet describe the same
# layout the generator measured. That is its whole job. Do not read it as
# ground truth about what the planner considers reachable.
CHECKSUM_GRID_RESOLUTION_M = 0.005
if GRID_RESOLUTION_M != CHECKSUM_GRID_RESOLUTION_M:
    raise RuntimeError(
        f"reachability_filter rasterises at {GRID_RESOLUTION_M} m but the build "
        f"sheets' checksums were computed at {CHECKSUM_GRID_RESOLUTION_M} m. "
        f"Roughly half the pool would now be refused for no real reason. "
        f"Regenerate the sheets at the new resolution, or pin this back."
    )


def _corners(cx, cy, long_cm, short_cm, bearing_deg):
    """The four corners of an item, long side along ``bearing_deg``."""
    a = math.radians(bearing_deg)
    ux, uy = math.cos(a), math.sin(a)            # along the long side
    vx, vy = -uy, ux                             # along the short side
    hl, hs = long_cm / 2.0, short_cm / 2.0
    return [(cx + sl * hl * ux + ss * hs * vx, cy + sl * hl * uy + ss * hs * vy)
            for sl, ss in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _overlap_depth(a, b):
    """Penetration depth between two convex polygons, 0 if they are apart.

    Separating-axis test. Returns the smallest overlap across all candidate
    axes, which is 0 or less exactly when a separating axis exists.
    """
    best = float("inf")
    for poly in (a, b):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = -(y2 - y1), (x2 - x1)
            norm = math.hypot(nx, ny)
            if norm == 0:
                continue
            nx, ny = nx / norm, ny / norm
            pa = [px * nx + py * ny for px, py in a]
            pb = [px * nx + py * ny for px, py in b]
            gap = min(max(pa) - min(pb), max(pb) - min(pa))
            if gap <= 0:
                return 0.0
            best = min(best, gap)
    return best


def scene_contact_counts(rows):
    """Recompute the blocker's contact breakdown from the placed geometry.

    COUNTS ONLY, never indices. This rebuilds each item with its long side on
    local X, while the scene generator carries movables with the long side on
    local Y. Same four physical faces and the same 60 approach poses, different
    numbering. The counts port across that difference; the indices do not.

    Returns (reachable, cut_off, in_collision), summing to CONTACTS_PER_OBJECT.
    """
    head = rows[0]
    radius_cm = robot_inflation_radius_cm(ROBOT_W_CM, ROBOT_H_CM)
    standoff_cm = radius_cm + PUSH_OFFSET_MARGIN_CM
    margin_m = get_wavefront_inflation_config().tier1_base_inflation_margin_m

    poses = {
        r["marker_hint"]: ObjectPose(
            x=float(r["centre_x_cm"]), y=float(r["centre_y_cm"]),
            theta=float(r["long_axis_bearing_deg"]),
            depth=float(r["long_cm"]), width=float(r["short_cm"]),
            is_static=(r["item"] == "brick"),
        )
        for r in rows
    }
    planner = WavefrontPlanner(WavefrontConfig(
        resolution=GRID_RESOLUTION_M, robot_radius=radius_cm / 100.0,
        inflation_margin=margin_m))
    planner.build_grid(
        (0.0, WORKSPACE_W_CM / 100.0, 0.0, WORKSPACE_H_CM / 100.0),
        {n: (p.x / 100.0, p.y / 100.0, p.depth / 200.0, p.width / 200.0, p.theta)
         for n, p in poses.items()},
    )
    robot_m = (float(head["robot_start_x_cm"]) / 100.0,
               float(head["robot_start_y_cm"]) / 100.0)
    planner.apply_trapped_start_recovery(robot_m)
    grid = planner.get_grid()
    if grid is None:
        raise RuntimeError("wavefront grid unavailable, cannot recompute the checksum")

    free = grid == planner.FREE
    height, width = free.shape
    si, sj = planner._world_to_grid(*robot_m)
    reachable_cells = np.zeros_like(free)
    if free[sj, si]:
        reachable_cells[sj, si] = True
        queue = deque([(sj, si)])
        while queue:
            j, i = queue.popleft()
            for dj, di in FLOOD_FILL_NEIGHBOURS:
                nj, ni = j + dj, i + di
                if (0 <= nj < height and 0 <= ni < width
                        and free[nj, ni] and not reachable_cells[nj, ni]):
                    reachable_cells[nj, ni] = True
                    queue.append((nj, ni))

    counts = {"reachable": 0, "cutoff": 0, "collision": 0}
    for pose in poses.values():
        if pose.is_static:
            continue
        for edge_idx in range(EDGE_COUNT):
            point = get_edge_point(pose, edge_idx=edge_idx, standoff=standoff_cm,
                                   points_per_face=POINTS_PER_FACE)
            i, j = planner._world_to_grid(point.position[0] / 100.0,
                                          point.position[1] / 100.0)
            if not (0 <= i < width and 0 <= j < height) or not free[j, i]:
                counts["collision"] += 1
            elif reachable_cells[j, i]:
                counts["reachable"] += 1
            else:
                counts["cutoff"] += 1
    return counts["reachable"], counts["cutoff"], counts["collision"]


def _point_to_polygon_distance(px, py, poly):
    """Shortest distance from a point to a convex polygon. 0 if inside."""
    inside = True
    best = float("inf")
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        ex, ey = x2 - x1, y2 - y1
        cross = ex * (py - y1) - ey * (px - x1)
        if cross < 0:
            inside = False
        length_sq = ex * ex + ey * ey
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / length_sq))
        best = min(best, math.hypot(px - (x1 + t * ex), py - (y1 + t * ey)))
    return 0.0 if inside else best


def check_scene(build_id, rows):
    """Reasons to refuse this scene, and reasons to warn about it.

    Returns (problems, warnings). A problem means the rows do not describe a
    buildable scene, or describe a different one than the labels came from, so
    the card is not drawn. A warning means the card is worth having but comes
    with a caveat stamped on it.
    """
    problems, warnings = [], []
    head = rows[0]
    declared = int(head["n_bricks"])
    actual = sum(1 for r in rows if r["item"] == "brick")
    if declared != actual:
        problems.append(f"n_bricks says {declared} but {actual} brick rows present")

    placed = []
    for r in rows:
        poly = _corners(float(r["centre_x_cm"]), float(r["centre_y_cm"]),
                        float(r["long_cm"]), float(r["short_cm"]),
                        float(r["long_axis_bearing_deg"]))
        placed.append((r["marker_hint"], poly))
        for x, y in poly:
            if not (-EDGE_TOLERANCE_CM <= x <= WORKSPACE_W_CM + EDGE_TOLERANCE_CM and
                    -EDGE_TOLERANCE_CM <= y <= WORKSPACE_H_CM + EDGE_TOLERANCE_CM):
                problems.append(
                    f"{r['marker_hint']} corner ({x:.1f}, {y:.1f}) is outside the workspace")
                break

    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            depth = _overlap_depth(placed[i][1], placed[j][1])
            if depth > OVERLAP_TOLERANCE_CM:
                problems.append(
                    f"{placed[i][0]} and {placed[j][0]} overlap by {depth:.1f} cm")

    # The disc of radius ROBOT_CIRCUMSCRIBED_R_CM around the start point covers
    # the car at every yaw. Boxing that disc into a square was the first attempt
    # and over-rejected by up to 2 cm at the corners, because the square's own
    # half-diagonal is 7.0 against the disc's 4.95.
    rx, ry = float(head["robot_start_x_cm"]), float(head["robot_start_y_cm"])
    for name, poly in placed:
        clearance = _point_to_polygon_distance(rx, ry, poly)
        shortfall = ROBOT_CIRCUMSCRIBED_R_CM - clearance
        if shortfall > OVERLAP_TOLERANCE_CM:
            problems.append(
                f"robot start is {clearance:.2f} cm from {name}, needs "
                f"{ROBOT_CIRCUMSCRIBED_R_CM:.2f} to be placeable at every yaw")
    goal = (float(head["goal_x_cm"]), float(head["goal_y_cm"]))
    for name, poly in placed:
        if _overlap_depth(_corners(goal[0], goal[1], 0.2, 0.2, 0.0), poly) > 0:
            problems.append(f"goal {goal} sits inside {name}")

    if all(c in head and head[c] != "" for c in CHECKSUM_COLUMNS):
        claimed = tuple(int(head[c]) for c in CHECKSUM_COLUMNS)
        if sum(claimed) != CONTACTS_PER_OBJECT:
            problems.append(
                f"checksum columns sum to {sum(claimed)}, expected {CONTACTS_PER_OBJECT}")
        else:
            actual = scene_contact_counts(rows)
            if actual != claimed:
                # The three counts always sum to 60, so any disagreement is a
                # reallocation and the L1 distance double-counts each contact
                # that moved. Halve it to get contacts moved.
                moved = sum(abs(a - c) for a, c in zip(actual, claimed)) // 2
                counts = (f"sheet says {claimed[0]}/{claimed[1]}/{claimed[2]}, "
                          f"this layout gives {actual[0]}/{actual[1]}/{actual[2]} "
                          f"(reachable/cut-off/collision)")
                if moved <= CHECKSUM_TIE_TOLERANCE_CONTACTS:
                    problems.append(
                        f"contact checksum tie, {moved} contact(s) changed class: "
                        f"{counts}. Too close to call, so this scene's own tier "
                        f"label is noise-sensitive. Drop it rather than debug it.")
                else:
                    problems.append(
                        f"contact checksum disagrees, {moved} contacts moved: "
                        f"{counts}. The rows do not describe the scene the labels "
                        f"came from.")
    else:
        # Not a refusal. A card without the checksum is still worth building
        # from; it just carries less assurance, and the card says so rather
        # than looking identical to a verified one.
        warnings.append(
            "no contact checksum on this sheet, so a bearing 90 degrees out "
            "would not be caught")
    return problems, warnings


def draw_scene(build_id, rows, out_path, warnings=()):
    head = rows[0]
    fig, ax = plt.subplots(figsize=(7.5, 10.5))
    ax.add_patch(Rectangle((0, 0), WORKSPACE_W_CM, WORKSPACE_H_CM,
                           fill=False, ec="k", lw=2.5))
    for r in rows:
        cx, cy = float(r["centre_x_cm"]), float(r["centre_y_cm"])
        L, S = float(r["long_cm"]), float(r["short_cm"])
        bearing = float(r["long_axis_bearing_deg"])
        is_brick = r["item"] == "brick"
        patch = Rectangle((-L / 2, -S / 2), L, S, angle=bearing,
                          rotation_point="center",
                          fc="0.55" if is_brick else "gold", ec="k", lw=1.5)
        patch.set_transform(mtransforms.Affine2D().translate(cx, cy) + ax.transData)
        ax.add_patch(patch)
        ax.annotate(f"{r['marker_hint']}\n({cx:.1f}, {cy:.1f})\n{bearing:.1f}°  "
                    f"{L:.1f}x{S:.1f}", (cx, cy), textcoords="offset points",
                    xytext=(0, 0), ha="center", va="center", fontsize=8,
                    weight="bold", bbox=dict(fc="w", alpha=0.85, ec="0.4", pad=1.5))
    rx, ry = float(head["robot_start_x_cm"]), float(head["robot_start_y_cm"])
    bearing = float(head["robot_start_bearing_deg"])
    gx, gy = float(head["goal_x_cm"]), float(head["goal_y_cm"])
    car = Rectangle((-ROBOT_H_CM / 2, -ROBOT_W_CM / 2), ROBOT_H_CM, ROBOT_W_CM,
                    angle=bearing, rotation_point="center",
                    fc="tab:blue", alpha=0.6, ec="k")
    car.set_transform(mtransforms.Affine2D().translate(rx, ry) + ax.transData)
    ax.add_patch(car)
    # The heading is part of the labelled state, not decoration. The solve rate
    # on this card was measured with the car pointing this way.
    nose = math.radians(bearing)
    ax.arrow(rx, ry, ROBOT_H_CM * 0.75 * math.cos(nose),
             ROBOT_H_CM * 0.75 * math.sin(nose), width=0.35, head_width=1.6,
             fc="navy", ec="navy", length_includes_head=True, zorder=5)
    ax.annotate(f"robot start\n({rx:.1f}, {ry:.1f})  facing {bearing:.0f}°",
                (rx, ry), textcoords="offset points", xytext=(0, -14), ha="center",
                va="top", fontsize=8, color="tab:blue", weight="bold")
    ax.add_patch(Circle((gx, gy), GOAL_MARKER_RADIUS_CM, fc="tab:green", alpha=0.5, ec="k"))
    ax.annotate(f"goal\n({gx:.1f}, {gy:.1f})", (gx, gy), textcoords="offset points",
                xytext=(0, 12), ha="center", fontsize=8, color="darkgreen", weight="bold")

    ax.set_xlim(-4, WORKSPACE_W_CM + 4); ax.set_ylim(-4, WORKSPACE_H_CM + 4)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xticks(range(0, int(WORKSPACE_W_CM) + 1, 5))
    ax.set_yticks(range(0, int(WORKSPACE_H_CM) + 1, 5))
    ax.set_xlabel("x (cm) from interior bottom-left"); ax.set_ylabel("y (cm)")
    kind = head["push_kind"]
    ax.set_title(
        f"{build_id}   ({head['n_bricks']} bricks)\n"
        f"solve_rate {float(head['solve_rate']):.4f}   tried {head['tried']}   "
        f"valid_1push {head['valid_1push']}   valid_first_push {head['valid_first_push']}\n"
        "angles are LONG-side bearings, CCW from +X",
        fontsize=10)
    fig.tight_layout(rect=(0, 0.03, 1, 0.935))
    # What the watcher needs before the run starts, so it reads before the plot.
    # solve_rate says which pushes open a region, never how far anything moves:
    # mass and friction in the sim that produced it are unmeasured.
    fig.text(0.5, 0.967, PUSH_KIND_NOTE.get(kind, f"push_kind: {kind}"),
             ha="center", va="center", fontsize=9.5, weight="bold", linespacing=1.4,
             color="darkgreen" if kind == "one_push" else "saddlebrown",
             bbox=dict(fc="honeydew" if kind == "one_push" else "cornsilk",
                       ec="0.4", pad=4))
    if warnings:
        fig.text(0.5, 0.012, "UNVERIFIED: " + "; ".join(warnings), ha="center",
                 va="center", fontsize=8, color="darkred", weight="bold",
                 bbox=dict(fc="mistyrose", ec="darkred", pad=3))
    fig.savefig(out_path, dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path", type=Path)
    ap.add_argument("--out-dir", type=Path, default=Path("build_cards"))
    ap.add_argument("--check-only", action="store_true",
                    help="validate the rows and print the report, draw nothing")
    args = ap.parse_args()

    scenes = defaultdict(list)
    with args.csv_path.open() as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or ())]
        if missing:
            print(f"{args.csv_path} is missing required columns: {', '.join(missing)}",
                  file=sys.stderr)
            return 2
        for row in reader:
            scenes[row["build_id"]].append(row)

    failed = warned = 0
    verdicts = {}
    for build_id in sorted(scenes):
        problems, warnings = check_scene(build_id, scenes[build_id])
        verdicts[build_id] = (problems, warnings)
        if problems:
            failed += 1
            print(f"  REFUSED {build_id}")
            for p in problems:
                print(f"      {p}")
        elif warnings:
            warned += 1
            print(f"  warn    {build_id}")
            for w in warnings:
                print(f"      {w}")
        else:
            print(f"  ok      {build_id}")

    if not args.check_only:
        args.out_dir.mkdir(parents=True, exist_ok=True)
        for build_id in sorted(scenes):
            problems, warnings = verdicts[build_id]
            if not problems:
                draw_scene(build_id, scenes[build_id],
                           args.out_dir / f"{build_id}.png", warnings)
        print(f"\ncards written to {args.out_dir}")

    print(f"\n{len(scenes) - failed} of {len(scenes)} scenes buildable"
          + (f", {warned} unverified" if warned else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
