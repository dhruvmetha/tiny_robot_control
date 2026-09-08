#!/usr/bin/env python3
"""Generate NAMO scenes a human can physically build on the real 49.0 x 77.5 cm table.

Every existing car pool (`datasets/car_envs/v3/.../real_template_*`) copies the real table's SIZE
but nothing else: its inner walls have arbitrary lengths (13.8 cm, 24.4 cm, 3.0 cm) and arbitrary
yaw, and its blocks have arbitrary half-extents. None of it can be built. This generator samples
from the ACTUAL inventory instead, so each surviving scene is a build sheet.

Inventory is transcribed from the hardware repo `robot_control/config/{real,objects}.yaml`
(reported verbatim 2026-08-22, see the `reference-real-robot-bridge` memory):
  arena interior 49.0 x 77.5 cm, origin bottom-left, +x right, +y up
  bricks   19.5 x 5.5 x 10.0 cm, 3 confirmed on the table, a 4th declared (wall_9, orientation suspect)
  movables obj_1 15.0x7.0x4.0 and obj_4 12.0x7.5x5.0 confirmed; obj_2/3/5 declared but not sighted
The yaml states blocks as `width` x `depth` where DEPTH IS THE X EXTENT and WIDTH IS THE Y EXTENT.
MuJoCo half-extents are therefore (depth/2, width/2, height/2). Getting this backwards is silent;
the hardware side did it twice in one day. `real_test_envs/1push/1hop/env1` pins it: obj_1 appears
there as size="0.035 0.075 0.02", which is exactly (7.0/2, 15.0/2, 4.0/2) cm.

Geometry gate, applied before any physics runs. The wavefront inflates every obstacle by
`compute_wavefront_inflation_radius_m` (`include/wavefront/goal_tolerance_utils.hpp:34`) = 4.0 cm,
so a corridor needs 8.0 cm of clear width. READ THAT FUNCTION BEFORE CHANGING THIS NUMBER: it calls
`compute_rotation_safe_robot_radius_m`, whose name says diagonal but whose body returns
max(hx, hy) = 3.5 cm, the diagonal having been dropped because 4.95 cm "crowded out push
approaches". Believing the name gives 5.45 cm inflation and a 10.9 cm corridor rule, which rejects
buildable scenes: on the first pilot it disagreed with the C++ probe on 29 of 60, and the measured
disconnect threshold across that pilot was 4.04-4.25 cm, not 5.45. A scene is kept only if
  (a) with the blocker present, the goal is UNREACHABLE from the robot start
  (b) with the blocker deleted, the goal IS reachable
  (c) (b) still holds at `--margin-cm` of clearance instead of 8.0
(a)+(b) are the counterfactual certificate `probe_static_topology.py` documents. (c) exists because
8.0 cm is a hard threshold and ArUco pose noise is a few mm, so a scene built at the threshold is a
coin flip on hardware rather than an experiment.

This gate is NECESSARY, NOT SUFFICIENT: it proves the blocker is what separates the two regions, not
that any push can move it clear. Only the simulator decides that. SINGLE HOP is what this generator
enforces (one blocker between robot and goal); the push count is a property of the search, so label
at CHAIN DEPTH 2 and read both answers off one pass. `build_2push_validset.py` splits the trial log
by `chain_depth`, giving F (`valid_1push`) and F1' (`valid_first_push`) together, so a depth-2 run
says which scenes are 1-push solvable AND which of the rest are 2-push solvable.

Use `region_opening_exhaustive_2push_multihop_car.yaml` UNCHANGED. It already carries the four
settings that make a run exhaustive, and three of them fail silently when wrong:
`region_exhaustive_mode: true` is the only thing that populates `primitive_trial_log` at all
(region_opening.py:1703 writes None without it, and the answer key then has nothing to read);
`region_sample_k: 0` stops the planner drawing a random k-subset of the reachable candidates
(region_opening.py:2702, and the rung1 config DOES subsample, at k=25); and the two
`*_solutions_per_neighbor: 9999` caps stop it halting at the first hit, which would leave a
denominator that means nothing.

  PYTHONPATH=build_python:python python -m namo.data_collection.modular_parallel_collection \
      --config-yaml python/namo/data_collection/region_opening_exhaustive_2push_multihop_car.yaml \
      --manifest <out>/manifest.txt --output-dir <pkls> --workers 48

then tier with `probe_static_topology.py --expect-hop 1` + `label_keyhole1_difficulty.py`, exactly
as for any other pool. Leave `--target-goal-region` OFF: these scenes are single-hop by
construction, so letting the collection enumerate every neighbour of the robot region costs little
and lets the probe's hop count CONTRADICT this generator's numpy gate rather than hide a
disagreement between it and the C++ wavefront.

  python scripts/pipeline/gen_real_buildable_scenes.py --out-dir /path/scenes --num 2000 --seed 0
"""
import argparse
import json
import math
import os
import random
import numpy as np
from scipy import ndimage

#: full 3x3 = 8-connectivity, the C++ wavefront's neighbourhood
_S8 = np.ones((3, 3), dtype=int)

# ---------------------------------------------------------------------------
# Real inventory. All lengths in METRES. See module docstring for provenance.
# ---------------------------------------------------------------------------
ARENA_W = 0.490          # x extent of the interior
ARENA_H = 0.775          # y extent of the interior
BORDER_HALF = 0.010      # border wall half-thickness, inner faces at 0 and ARENA_*

BRICK_HALF = (0.0975, 0.0275, 0.050)     # 19.5 x 5.5 x 10.0 cm

# name -> (half_x, half_y, half_z), from objects.yaml (depth/2, width/2, height/2)
MOVABLES = {
    "obj_1": (0.0350, 0.0750, 0.020),    # 15.0 x 7.0 x 4.0, marker 5, ON TABLE
    "obj_4": (0.0375, 0.0600, 0.025),    # 12.0 x 7.5 x 5.0, marker 8, ON TABLE
    "obj_2": (0.0250, 0.0625, 0.025),    # 12.5 x 5.0 x 5.0, marker 6, declared only
    "obj_3": (0.0250, 0.0625, 0.025),    # 12.5 x 5.0 x 5.0, marker 7, declared only
    "obj_5": (0.0200, 0.0825, 0.025),    # 16.5 x 4.0 x 5.0, marker 2, declared only
}
ON_TABLE = ("obj_1", "obj_4")

ROBOT_HALF_X = 0.035                      # 7 x 7 cm chassis, so hx == hy
ROBOT_HALF_Y = 0.035
TIER1_MARGIN = 0.005                      # config/wavefront_inflation.yaml tier1
#: max(hx, hy), NOT hypot. See the docstring and goal_tolerance_utils.hpp:12.
WAVEFRONT_ROBOT_R = max(ROBOT_HALF_X, ROBOT_HALF_Y)
INFLATE_R = WAVEFRONT_ROBOT_R + TIER1_MARGIN          # 0.040 m -> 8.0 cm corridors

#: The car's CORNER reach. The wavefront inflates by max(hx, hy), which models the robot as a 3.5 cm
#: disc, and that is the planner's business. But a person places a physical 7x7 SQUARE, whose corners
#: reach hypot(3.5, 3.5) = 4.95 cm, so a start pose that clears the planner by 4.10 cm can still
#: intersect a block and be impossible to set down. That happened on 10 of the first 600 delivered
#: scenes. Placement is checked against this radius, planning against INFLATE_R; they are different
#: questions and the same max-vs-diagonal distinction that has bitten this pipeline twice already.
ROBOT_CIRCUMSCRIBED_R = math.hypot(ROBOT_HALF_X, ROBOT_HALF_Y)

#: The car's heading at t=0, in every generated scene. `CAR_BODY` writes `<body name="car" pos=...>`
#: with no euler or quat, so MuJoCo uses identity and the car faces +X. Every label in this pipeline
#: was measured with the car at this heading. The build sheet has to SAY so: a person handed only a
#: start position places the car whichever way it happens to be pointing, which is a different
#: initial state from the one that was simulated. It is also why placement is checked against the
#: circumscribed disc rather than the real 7x7 footprint, since a sheet silent about heading has to
#: hold for every yaw. Stating the heading is what makes the tighter check legitimate.
ROBOT_START_BEARING_DEG = 0.0

GRID_RES = 0.005                          # 5 mm rasterisation for the reachability gate


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
class Rect:
    """Axis-aligned-in-its-own-frame box, placed at (cx, cy) with yaw."""

    __slots__ = ("cx", "cy", "hx", "hy", "yaw", "name", "kind")

    def __init__(self, cx, cy, hx, hy, yaw, name, kind):
        self.cx, self.cy, self.hx, self.hy, self.yaw = cx, cy, hx, hy, yaw
        self.name, self.kind = name, kind

    def corners(self, pad=0.0):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.hx + pad, self.hy + pad
        return [(self.cx + c * sx * hx - s * sy * hy, self.cy + s * sx * hx + c * sy * hy)
                for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]

    def contains(self, px, py, pad=0.0):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.cx, py - self.cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return abs(lx) <= self.hx + pad and abs(ly) <= self.hy + pad

    def aabb(self, pad=0.0):
        xs = [p[0] for p in self.corners(pad)]
        ys = [p[1] for p in self.corners(pad)]
        return min(xs), min(ys), max(xs), max(ys)


def _axes(r):
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    return ((c, s), (-s, c))


def overlaps(a, b, pad=0.0):
    """Separating-axis test between two rotated rectangles, `pad` inflates both."""
    ca, cb = a.corners(pad / 2.0), b.corners(pad / 2.0)
    for ax, ay in _axes(a) + _axes(b):
        pa = [x * ax + y * ay for x, y in ca]
        pb = [x * ax + y * ay for x, y in cb]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def inside_arena(r, pad=0.0):
    x0, y0, x1, y1 = r.aabb(pad)
    return x0 >= 0.0 and y0 >= 0.0 and x1 <= ARENA_W and y1 <= ARENA_H


# ---------------------------------------------------------------------------
# Reachability gate
# ---------------------------------------------------------------------------
def _blocked_mask(rects, inflate):
    """Rasterise the arena, marking a cell blocked if its centre lies inside any inflated rect.

    The border is handled by inflating the arena edge inward by the same radius, which is what the
    wavefront does with the four boundary walls.
    """
    nx = int(round(ARENA_W / GRID_RES))
    ny = int(round(ARENA_H / GRID_RES))
    xs = (np.arange(nx) + 0.5) * GRID_RES
    ys = (np.arange(ny) + 0.5) * GRID_RES
    gx, gy = np.meshgrid(xs, ys, indexing="ij")

    blocked = (gx < inflate) | (gx > ARENA_W - inflate) | (gy < inflate) | (gy > ARENA_H - inflate)
    for r in rects:
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        dx, dy = gx - r.cx, gy - r.cy
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        blocked |= (np.abs(lx) <= r.hx + inflate) & (np.abs(ly) <= r.hy + inflate)
    return blocked


def _connected(blocked, start, goal):
    """8-connected free-space connectivity, MATCHING `WavefrontPlanner::DIRECTIONS`.

    `ndimage.label` with a full 3x3 structure is the same 8-neighbourhood as the C++ BFS, and it
    runs in C. The generator calls this tens of thousands of times per scene once `open_frac` is on,
    so a python deque here dominates the whole runtime.
    """
    lab, _ = ndimage.label(~blocked, structure=_S8)
    a, b = lab[start], lab[goal]
    return bool(a) and a == b


def _cell(p):
    return (int(p[0] / GRID_RES), int(p[1] / GRID_RES))


def start_is_placeable(statics, blocker, start):
    """Can a human actually set the 7x7 car down here, at any yaw?

    Distinct from "is the start cell free in the wavefront grid". See ROBOT_CIRCUMSCRIBED_R.
    """
    return all(_surface_gap(start, r) >= ROBOT_CIRCUMSCRIBED_R for r in statics + [blocker])


def _surface_gap(pt, r):
    """Distance from a point to a rotated rectangle's surface, 0 if inside."""
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    dx, dy = pt[0] - r.cx, pt[1] - r.cy
    lx, ly = abs(c * dx + s * dy), abs(-s * dx + c * dy)
    return math.hypot(max(lx - r.hx, 0.0), max(ly - r.hy, 0.0))


def gate(statics, blockers, start, goal, margin_r):
    """Return (passed, reason). See module docstring for the three conditions.

    `blockers` is the list of movables that together separate the two regions -- length 1 for the
    default single-object scene, length 2 under `--n-movables 2`. The certificate reads ALL of them
    present (must be unreachable) vs ALL of them removed (must be reachable): with two movables in
    one doorway, that is the honest generalisation, since it is the domino of BOTH objects moving
    that is meant to open the goal, not either alone. `start_is_placeable` still runs once per
    blocker, since the robot must not be able to spawn inside any of them.
    """
    if not all(start_is_placeable(statics, blk, start) for blk in blockers):
        return False, "start_not_placeable"
    with_block = _blocked_mask(statics + list(blockers), INFLATE_R)
    si, sj = _cell(start)
    gi, gj = _cell(goal)
    nx, ny = with_block.shape
    if not (0 <= si < nx and 0 <= sj < ny and 0 <= gi < nx and 0 <= gj < ny):
        return False, "out_of_grid"
    if with_block[si, sj]:
        return False, "start_in_collision"
    if with_block[gi, gj]:
        return False, "goal_in_collision"
    if _connected(with_block, (si, sj), (gi, gj)):
        return False, "already_open"

    without = _blocked_mask(statics, INFLATE_R)
    if without[si, sj] or without[gi, gj] or not _connected(without, (si, sj), (gi, gj)):
        return False, "walls_alone_block_it"

    wide = _blocked_mask(statics, margin_r)
    if wide[si, sj] or wide[gi, gj] or not _connected(wide, (si, sj), (gi, gj)):
        return False, "no_margin"
    return True, "ok"


#: push standoff, mirroring namo_push_controller.cpp:162-163 = max(hx,hy) + push_offset_margin.
#: The margin is `planning.wavefront_edge_offset_margin` = 0.01 in the car config.
PUSH_STANDOFF = WAVEFRONT_ROBOT_R + 0.010
POINTS_PER_FACE = 15


def contact_points(blk):
    """The 60 push contact poses, mirroring namo_push_controller.cpp:176-189 exactly.

    15 per face. Top and bottom sample along the object's local x at y = +-(hy + standoff); left and
    right sample along local y at x = +-(hx + standoff). Then rotate into world by the object yaw.
    """
    pts = []
    for u in np.linspace(-blk.hx, blk.hx, POINTS_PER_FACE):
        pts.append((u, blk.hy + PUSH_STANDOFF))
        pts.append((u, -blk.hy - PUSH_STANDOFF))
    for v in np.linspace(-blk.hy, blk.hy, POINTS_PER_FACE):
        pts.append((blk.hx + PUSH_STANDOFF, v))
        pts.append((-blk.hx - PUSH_STANDOFF, v))
    c, s = math.cos(blk.yaw), math.sin(blk.yaw)
    return [(blk.cx + c * px - s * py, blk.cy + s * px + c * py) for px, py in pts]


def n_reachable_contacts(statics, blocker, start):
    """How many of the 60 contact poses sit in the robot's own free region at t=0.

    This is the SEARCH DENOMINATOR, and it decides the tier more than anything else does. Measured
    against 981 exhaustively-labelled scenes it tracks the simulator's `tried_1push` at Spearman
    0.961, for no physics at all. Note the scales differ: `tried_1push` counts (edge, depth) PAIRS,
    so it runs about 5x this count over the 5 push depths. Spearman is rank-based, so the agreement
    statement survives that, but do not compare the two numbers directly.

    ⚠ THAT 0.961 IS PARTLY CIRCULAR AND PROVES LESS THAN IT LOOKS. `tried_1push` is itself geometric,
    decided by the same wavefront rule this function reimplements, so the correlation shows the
    reimplementation is faithful, NOT that either matches the real robot. A shared wrong assumption
    stays invisible to it: the max(hx,hy)-vs-diagonal radius bug would have held a high Spearman
    while shifting every count. For STEERING that is fine, because the tier is defined as valid over
    the planner's own `tried`, so predicting the planner is the entire job. For TRANSFER to hardware
    it is not fine, and only driving the real robot at a contact sim calls reachable settles it.

    The direction is the opposite of the obvious guess. Wedging a block into a brick pocket does not
    make a scene HARD, it makes it UNSOLVABLE: those scenes average 10.1 reachable contacts, and
    with a denominator that small any push that works at all pushes the solve rate straight into the
    easy band. Hard scenes are the ones where the block is wide open (mean 25.6 contacts) and almost
    none of the many available pushes clear the corridor. Conditioning on >= 24 contacts with 2
    bricks yields 17.6% hard on the hmax=2 axis against a 3.7% base rate.

    ⛔ THIS IS A TIER PREDICTOR, NOT A PLANNER BLACKLIST. Do not feed it to
    `external_edge_blacklist`. That map seeds `{edge_idx: 0}` and the test is
    `depth >= edge_min_stuck_depth[edge_idx]` (region_opening.py:2210-2216), so depth 0 means SKIP
    AT EVERY DEPTH. Region membership is a t=0 property and a first push changes it, so banning on
    it deletes whole solution classes: on one real two-object scene every one of an object's 60
    contacts was in collision or cut off at t=0, and banning them all would erase every chain of
    the form "push the other object clear, then push this one". Collision-freedom survives a first
    push and is the right test for an all-depths ban. Region membership is the right test HERE,
    where nothing happens after t=0 by construction. Scenes from this generator carry exactly one
    movable and so cannot show that failure, which is why the warning has to be written down.
    """
    m = _blocked_mask(statics + [blocker], INFLATE_R)
    si = _cell(start)
    if m[si]:
        return 0
    lab, _ = ndimage.label(~m, structure=_S8)
    home, (nx, ny) = lab[si], m.shape
    k = 0
    for p in contact_points(blocker):
        i, j = _cell(p)
        if 0 <= i < nx and 0 <= j < ny and lab[i, j] == home:
            k += 1
    return k


def contact_breakdown(statics, blocker, start):
    """(reachable, cut_off, in_collision) over the blocker's 60 contact poses at t=0. Sums to 60.

    Shipped on every build sheet as a CHECKSUM, because the one error class geometric validation
    cannot catch is a scene that is perfectly buildable and simply not the scene the labels describe.
    Rotate a brick 90 degrees and it still sits inside the workspace, still overlaps nothing, still
    matches n_bricks. Nothing about it looks wrong. That is why this error has bitten this pipeline
    five times.

    These three numbers move sharply under exactly those faults. The hardware side measured it on
    easy_000: as specced 18/15/27, with wall_10 rotated 90 degrees 48/0/12. A wrong centre, a wrong
    bearing, a swapped bar or a mistyped dimension all shift them too.

    Recomputable from the CSV alone, since (centre, long_axis_bearing, long_cm, short_cm) fixes the
    world footprint. Only the COUNTS are comparable across implementations, not contact indices: a
    consumer that builds the rect with its long side on local X samples the same four physical faces
    as this one, which puts its long side on local Y for blocks, so the point SET matches while the
    numbering does not. Both sides independently produced 16/16/28 on rb_00003 with poses agreeing
    to 0.07 cm, so the counts are a real cross-check and not a shared assumption.
    """
    m = _blocked_mask(statics + [blocker], INFLATE_R)
    si = _cell(start)
    if m[si]:
        return 0, 0, len(contact_points(blocker))
    lab, _ = ndimage.label(~m, structure=_S8)
    home, (nx, ny) = lab[si], m.shape
    reach = cut = coll = 0
    for p in contact_points(blocker):
        i, j = _cell(p)
        if not (0 <= i < nx and 0 <= j < ny) or m[i, j]:
            coll += 1
        elif lab[i, j] == home:
            reach += 1
        else:
            cut += 1
    return reach, cut, coll


def open_frac(statics, blocker, start, goal, margin_r, span=0.08, n=9, nrot=4):
    """Fraction of feasible DISPLACED blocker poses that open the corridor. A physics-free proxy
    for how many of the 60x5 pushes will solve the scene, so difficulty can be targeted before
    spending any simulator time.

    Validated against 51 exhaustively-labelled pilot scenes: Spearman 0.758 against the true
    `solve_rate_1push`, with tier means easy 0.244, med 0.054, unsolvable 0.024. It is a STEERING
    signal, not a label. The bands only bias what gets generated; the exhaustive collection still
    decides every scene's actual tier.

    ⛔ DO NOT DROP THIS FILTER WHEN STEERING ON CONTACT COUNT. It looks marginally useless on the
    hmax=2 axis (median 0.03 for easy, med and hard alike across 981 scenes) and it is not: it is
    doing the work CONDITIONALLY. A pool generated with `--contacts 24,60 --max-bricks 2` but
    `--open-frac 0,1` returned 1 hard scene in 768, against 6.0% and 3.9% in two pools that had no
    contact filter at all but did have an open_frac band. Median open_frac was 0.503 in the wide
    pool against 0.027 in the banded ones. The 17.6%-hard figure that motivated the contact rule was
    measured INSIDE an already open_frac-filtered population, so it was a conditional rate read as a
    marginal one. Steer on both together or neither.
    """
    si, gi = _cell(start), _cell(goal)
    feasible = opened = 0
    for dx in np.linspace(-span, span, n):
        for dy in np.linspace(-span, span, n):
            for k in range(nrot):
                cand = Rect(blocker.cx + dx, blocker.cy + dy, blocker.hx, blocker.hy,
                            blocker.yaw + k * math.pi / nrot, blocker.name, blocker.kind)
                m = _blocked_mask(statics + [cand], margin_r)
                if m[si] or m[gi]:
                    continue
                feasible += 1
                opened += _connected(m, si, gi)
    return opened / feasible if feasible else 0.0


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------
def _brick(cx, cy, yaw_deg, idx):
    return Rect(cx, cy, BRICK_HALF[0], BRICK_HALF[1], math.radians(yaw_deg),
                f"wall_inner_{idx}", "brick")


def _x_span(yaw_deg):
    """How much of the arena's 49 cm width a brick eats at this yaw."""
    c, s = abs(math.cos(math.radians(yaw_deg))), abs(math.sin(math.radians(yaw_deg)))
    return 2 * (BRICK_HALF[0] * c + BRICK_HALF[1] * s)


def _layout(rng, name, n_bricks):
    """Return (statics, y_lo, y_hi, passage_x) or None. `passage_x` is where the blocker belongs.

    `y_lo`/`y_hi` are the same single divider height for all four layouts here; they only differ
    under `--bands 2` (see `_layout_bands2`), which is why the caller always unpacks two values
    instead of one -- so the single-divider and two-band paths share one interface with no branch
    in `sample_scene` for where the robot start / goal y-bounds come from.

    The arena is 49.0 cm wide and a brick is 19.5 cm, so the layouts differ mainly in how much of
    that width they eat and where they leave the hole. Two bricks flush to both side walls leave
    exactly 10.0 cm, which clears the corrected 8.0 cm corridor with 2 cm to spare: passable, tight,
    and the regime where few pushes solve. Under the old wrong 10.9 cm rule every such scene was
    unsolvable and got thrown away, which is most of why the first pool came out 59% easy.
    """
    y = rng.uniform(0.30, 0.48)
    st = []
    if name == "side_gap":
        # one brick flush to a side wall; the rest of the width is the passage
        left = rng.random() < 0.5
        yaw = rng.uniform(-12, 12)
        half = _x_span(yaw) / 2
        cx = half if left else ARENA_W - half
        st.append(_brick(cx, y, yaw, 1))
        passage_x = rng.uniform(half * 2 + 0.05, ARENA_W - 0.05) if left else rng.uniform(0.05, ARENA_W - half * 2 - 0.05)

    elif name == "center_door":
        # bricks in from both side walls, doorway in the middle; yaw sets the width
        yl, yr = rng.uniform(-25, 25), rng.uniform(-25, 25)
        hl, hr = _x_span(yl) / 2, _x_span(yr) / 2
        inset_l = rng.uniform(0.0, 0.03)
        inset_r = rng.uniform(0.0, 0.03)
        st.append(_brick(hl + inset_l, y + rng.uniform(-0.02, 0.02), yl, 1))
        st.append(_brick(ARENA_W - hr - inset_r, y + rng.uniform(-0.02, 0.02), yr, 2))
        lo, hi = 2 * hl + inset_l, ARENA_W - 2 * hr - inset_r
        if hi - lo < 0.06:
            return None
        passage_x = (lo + hi) / 2

    elif name == "stagger":
        # bricks at different y so the robot weaves round rather than through
        n = max(2, min(n_bricks, 3))
        xs = sorted(rng.uniform(0.10, ARENA_W - 0.10) for _ in range(n))
        for k, cx in enumerate(xs):
            st.append(_brick(cx, y + rng.uniform(-0.07, 0.07), rng.uniform(-55, 55), k + 1))
        passage_x = rng.uniform(0.08, ARENA_W - 0.08)

    elif name == "pocket":
        # a divider plus a perpendicular stub, so the blocker sits in a recess and most of its
        # contact faces are unreachable. This is the layout that should starve the push grid.
        yaw = rng.uniform(-12, 12)
        half = _x_span(yaw) / 2
        left = rng.random() < 0.5
        cx = half if left else ARENA_W - half
        st.append(_brick(cx, y, yaw, 1))
        passage_x = (2 * half + 0.06) if left else (ARENA_W - 2 * half - 0.06)
        stub_x = passage_x + rng.uniform(0.05, 0.10) * (1 if left else -1)
        st.append(_brick(stub_x, y + rng.uniform(0.06, 0.13), rng.uniform(70, 110), 2))
        if n_bricks >= 4 and rng.random() < 0.6:
            st.append(_brick(stub_x, y - rng.uniform(0.06, 0.13), rng.uniform(70, 110), 3))
    else:
        return None

    if len(st) > n_bricks:
        return None
    for a in range(len(st)):
        if not inside_arena(st[a]):
            return None
        for b in range(a + 1, len(st)):
            if overlaps(st[a], st[b], 0.004):
                return None
    return st, y, y, min(max(passage_x, 0.03), ARENA_W - 0.03)


def _layout_bands2(rng, n_bricks):
    """Two side_gap dividers at two y-levels, gaps on opposite sides, so the corridor bends.

    This is the aug9 car v3 turn primitive: a stub flush to one wall leaving a gap on the far side,
    repeated at a second y with the gap on the OPPOSITE side. Robot start goes below the lower
    band, goal above the upper band (see `sample_scene`, which reads `y_lo`/`y_hi` for exactly
    that). The blocker always occupies the LOWER band's gap; the upper band's gap is left clear,
    so it only bends the path, it is never itself an obstacle. Each band gets 1 or 2 bricks flush
    to its own wall -- a second brick end-to-end on the same wall narrows that band's own gap --
    within the `n_bricks` budget (3..6 total per the real inventory, split across the two bands).
    """
    y_lo = rng.uniform(0.20, 0.30)
    y_hi = y_lo + rng.uniform(0.18, 0.28)
    if y_hi > 0.58:
        return None
    left_lo = rng.random() < 0.5
    left_hi = not left_lo

    budget_lo = max(1, n_bricks // 2)
    budget_hi = max(1, n_bricks - budget_lo)

    def band(y, left, idx0, budget):
        yaw = rng.uniform(-12, 12)
        span = _x_span(yaw)
        n = 2 if (budget >= 2 and rng.random() < 0.4) else 1
        bricks = []
        for k in range(n):
            half = span * (k + 0.5)
            cx = half if left else ARENA_W - half
            bricks.append(_brick(cx, y + rng.uniform(-0.02, 0.02), yaw, idx0 + k))
        edge = span * n
        passage_x = (rng.uniform(edge + 0.05, ARENA_W - 0.05) if left
                     else rng.uniform(0.05, ARENA_W - edge - 0.05))
        return bricks, passage_x

    b_lo, px_lo = band(y_lo, left_lo, 1, budget_lo)
    b_hi, _px_hi = band(y_hi, left_hi, 1 + len(b_lo), budget_hi)

    st = b_lo + b_hi
    if len(st) > n_bricks:
        return None
    for a in range(len(st)):
        if not inside_arena(st[a]):
            return None
        for b in range(a + 1, len(st)):
            if overlaps(st[a], st[b], 0.004):
                return None
    return st, y_lo, y_hi, min(max(px_lo, 0.03), ARENA_W - 0.03)


LAYOUTS = ("side_gap", "center_door", "stagger", "pocket")


def _place_second_blocker(rng, statics, blocker1, name2, tries=30):
    """Place a second movable touching one of blocker1's 4 local faces (picked at random), so a
    push that drives blocker1 toward that face shoves blocker2 too -- the FEATURE 1 domino.

    Uses the rotated box's support function to put the two footprints exactly tangent along the
    chosen face plus a small gap, then re-checks with the ordinary `overlaps(..., 0.004)`
    convention like every other placement in this file (that convention wants >=4 mm of true
    clearance to certify no-overlap, so the gap is sampled comfortably above that).
    """
    hx2, hy2, _hz2 = MOVABLES[name2]
    c1, s1 = math.cos(blocker1.yaw), math.sin(blocker1.yaw)
    faces = [(1, 0, blocker1.hx), (-1, 0, blocker1.hx), (0, 1, blocker1.hy), (0, -1, blocker1.hy)]
    rng.shuffle(faces)
    for lux, luy, edge1 in faces:
        wx, wy = c1 * lux - s1 * luy, s1 * lux + c1 * luy
        for _try in range(tries):
            gap = rng.uniform(0.005, 0.015)
            yaw2 = math.radians(rng.uniform(0.0, 180.0))
            c2, s2 = math.cos(yaw2), math.sin(yaw2)
            support = abs(wx * c2 + wy * s2) * hx2 + abs(-wx * s2 + wy * c2) * hy2
            dist = edge1 + support + gap
            cand = Rect(blocker1.cx + wx * dist, blocker1.cy + wy * dist, hx2, hy2, yaw2,
                        "obstacle_1_movable", name2)
            if not inside_arena(cand):
                continue
            if any(overlaps(cand, o, 0.004) for o in statics):
                continue
            if overlaps(cand, blocker1, 0.004):
                continue
            return cand
    return None


def sample_scene(rng, movable_names, max_bricks, margin_r, band, layouts, contacts=(0, 60),
                 tries=600, n_movables=1, bands=1):
    """Sample one scene, gate it geometrically, then keep it only if `open_frac` lands in `band`.

    `band` is a (lo, hi) window on the physics-free difficulty proxy. Steering here is what makes
    100 hard scenes affordable: in the unsteered pilot, hard came in at 1 of 51, so filling that
    tier blind would have meant exhaustively labelling several thousand scenes.

    `n_movables` (default 1): with 2, a second movable is placed touching the first, inside the
    passage (`_place_second_blocker`), so one push on object 0 can shove object 1 too. Both
    movables count as the removable obstacle set for the geometry gate, and object 1 counts as an
    extra obstacle (like a static) for the `n_reachable_contacts`/`open_frac` proxies computed on
    object 0 -- see the FEATURE 1 report for why.

    `bands` (default 1): with 2, the walls are two side_gap dividers at different y, gaps on
    opposite sides (`_layout_bands2`), so the corridor bends. The blocker always sits in the
    lower band's gap; the upper band's gap is left clear.
    """
    lo, hi = band
    for _ in range(tries):
        if bands == 2:
            got = _layout_bands2(rng, max_bricks)
        else:
            got = _layout(rng, layouts[rng.randrange(len(layouts))], max_bricks)
        if got is None:
            continue
        statics, y_lo, y_hi, passage_x = got

        name = movable_names[rng.randrange(len(movable_names))]
        hx, hy, _hz = MOVABLES[name]
        blocker = None
        for _try in range(50):
            bx = passage_x + rng.uniform(-0.05, 0.05)
            by = y_lo + rng.uniform(-0.05, 0.05)
            cand = Rect(min(max(bx, max(hx, hy)), ARENA_W - max(hx, hy)), by, hx, hy,
                        math.radians(rng.uniform(0.0, 180.0)), "obstacle_0_movable", name)
            if inside_arena(cand) and not any(overlaps(cand, o, 0.004) for o in statics):
                blocker = cand
                break
        if blocker is None:
            continue

        blocker2, name2 = None, None
        if n_movables >= 2:
            remaining = [m for m in movable_names if m != name] or [name]
            name2 = remaining[rng.randrange(len(remaining))]
            blocker2 = _place_second_blocker(rng, statics, blocker, name2)
            if blocker2 is None:
                continue

        start = (rng.uniform(0.06, ARENA_W - 0.06), rng.uniform(0.06, y_lo - 0.11))
        goal = (rng.uniform(0.06, ARENA_W - 0.06), rng.uniform(y_hi + 0.13, ARENA_H - 0.06))
        if goal[1] <= start[1]:
            continue

        blockers = [blocker] + ([blocker2] if blocker2 is not None else [])
        passed, reason = gate(statics, blockers, start, goal, margin_r)
        if not passed:
            continue

        extra = [blocker2] if blocker2 is not None else []
        nc = n_reachable_contacts(statics + extra, blocker, start)
        if not (contacts[0] <= nc <= contacts[1]):
            continue
        of = open_frac(statics + extra, blocker, start, goal, margin_r)
        if not (lo <= of <= hi):
            continue

        blockers_out = [(blocker, name)] + ([(blocker2, name2)] if blocker2 is not None else [])
        return {"statics": statics, "blocker": blocker, "blocker_name": name,
                "blockers": blockers_out, "start": start, "goal": goal, "y_div": y_lo,
                "open_frac": of, "n_contacts": nc}, "ok"
    return None, "exhausted"


# ---------------------------------------------------------------------------
# Emit
# ---------------------------------------------------------------------------
CAR_BODY = """    <body name="car" pos="{sx} {sy} 0.01">
      <freejoint name="car_freejoint" />
      <inertial pos="0 0 0.0375" mass="0.35" diaginertia="0.000266 0.000266 0.000286" />
      <geom name="front_chassis_collision" type="box" pos="0.0175 0 0.0375" size="0.0175 0.035 0.0325" rgba="0.3 0.3 0.7 1" />
      <geom name="rear_chassis_collision" type="box" pos="-0.0175 0 0.0375" size="0.0175 0.035 0.0325" rgba="0.25 0.25 0.6 1" />
      <geom name="front_marker" type="box" pos="0.034 0 0.0505" size="0.002 0.015 0.01" rgba="1 0.2 0.2 1" contype="0" conaffinity="0" />
      <body name="rear_support_body" pos="-0.03 0 0.0025">
        <joint name="rear_caster_joint" type="ball" damping="0.0001" />
        <inertial pos="0 0 0" mass="0.025" diaginertia="0.000001 0.000001 0.000001" />
        <geom name="rear_support" type="sphere" pos="0 0 0" size="0.0025" friction="0 0 0" rgba="0.7 0.1 0.1 1" />
      </body>
      <body name="front_support_body" pos="0.03 0 0.0025">
        <joint name="front_caster_joint" type="ball" damping="0.0001" />
        <inertial pos="0 0 0" mass="0.025" diaginertia="0.000001 0.000001 0.000001" />
        <geom name="front_support" type="sphere" pos="0 0 0" size="0.0025" friction="0 0 0" rgba="0.1 0.7 0.1 1" />
      </body>
      <body name="left_wheel" pos="0 0.0375 0.015">
        <inertial pos="0 0 0" mass="0.05" diaginertia="0.000003 0.000006 0.000003" />
        <joint name="left_wheel_joint" type="hinge" axis="0 1 0" damping="0.01" armature="0.0001" />
        <geom name="left_wheel_collision" type="cylinder" size="0.015 0.0005" euler="90 0 0" rgba="0.1 0.1 0.1 1" />
      </body>
      <body name="right_wheel" pos="0 -0.0375 0.015">
        <inertial pos="0 0 0" mass="0.05" diaginertia="0.000003 0.000006 0.000003" />
        <joint name="right_wheel_joint" type="hinge" axis="0 1 0" damping="0.01" armature="0.0001" />
        <geom name="right_wheel_collision" type="cylinder" size="0.015 0.0005" euler="90 0 0" rgba="0.1 0.1 0.1 1" />
      </body>
    </body>"""

HEADER = """<?xml version='1.0' encoding='utf-8'?>
<mujoco model="real_buildable">
  <compiler angle="degree" />
  <option timestep="0.002" integrator="implicitfast" iterations="100" cone="elliptic" />
  <default>
    <geom density="1" />
  </default>
  <asset>
    <texture builtin="gradient" height="3072" rgb1="0.3 0.5 0.7" rgb2="0 0 0" type="skybox" width="512" />
    <texture builtin="checker" height="300" mark="edge" markrgb="0.8 0.8 0.8" name="groundplane" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" type="2d" width="300" />
    <material name="groundplane" reflectance="0.2" texrepeat="5 5" texture="groundplane" texuniform="true" />
  </asset>
  <worldbody>
    <light dir="0 0 -1" directional="true" pos="0 0 1.5" />
    <geom name="floor" type="plane" condim="4" friction="0.5 0.005 0.001" material="groundplane" size="0 0 0.05" />
    <body name="walls">"""

FOOTER = """  </worldbody>
  <actuator>
    <velocity name="left_wheel_drive" joint="left_wheel_joint" ctrlrange="-25 25" kv="0.75" forcerange="-0.5 0.5" />
    <velocity name="right_wheel_drive" joint="right_wheel_joint" ctrlrange="-25 25" kv="0.75" forcerange="-0.5 0.5" />
  </actuator>
</mujoco>
"""

WALL_GEOM = ('      <geom name="{name}" type="box" condim="4" friction="1.0 0.005 0.0001" '
             'rgba="0.8 0.8 0.8 1" pos="{x:.6f} {y:.6f} {z:.6f}" euler="0 0 {yaw:.6f}" '
             'size="{hx:.6f} {hy:.6f} {hz:.6f}" />')


def to_xml(scene):
    lines = [HEADER]
    lines.append(WALL_GEOM.format(name="wall_boundary_left", x=-BORDER_HALF, y=ARENA_H / 2,
                                  z=0.05, yaw=0.0, hx=BORDER_HALF, hy=ARENA_H / 2 + BORDER_HALF,
                                  hz=0.05))
    lines.append(WALL_GEOM.format(name="wall_boundary_right", x=ARENA_W + BORDER_HALF, y=ARENA_H / 2,
                                  z=0.05, yaw=0.0, hx=BORDER_HALF, hy=ARENA_H / 2 + BORDER_HALF,
                                  hz=0.05))
    lines.append(WALL_GEOM.format(name="wall_boundary_bottom", x=ARENA_W / 2, y=-BORDER_HALF,
                                  z=0.05, yaw=0.0, hx=ARENA_W / 2, hy=BORDER_HALF, hz=0.05))
    lines.append(WALL_GEOM.format(name="wall_boundary_top", x=ARENA_W / 2, y=ARENA_H + BORDER_HALF,
                                  z=0.05, yaw=0.0, hx=ARENA_W / 2, hy=BORDER_HALF, hz=0.05))
    for r in scene["statics"]:
        lines.append(WALL_GEOM.format(name=r.name, x=r.cx, y=r.cy, z=BRICK_HALF[2],
                                      yaw=math.degrees(r.yaw), hx=r.hx, hy=r.hy, hz=BRICK_HALF[2]))
    lines.append("    </body>")
    lines.append(CAR_BODY.format(sx=scene["start"][0], sy=scene["start"][1]))
    blockers = scene.get("blockers") or [(scene["blocker"], scene["blocker_name"])]
    for i, (b, bname) in enumerate(blockers):
        hx, hy, hz = MOVABLES[bname]
        body_name = f"obstacle_{i}_movable"
        lines.append(f'    <body name="{body_name}">')
        lines.append(f'      <geom name="{body_name}" condim="4" '
                     f'pos="{b.cx:.6f} {b.cy:.6f} {hz:.6f}" euler="0 0 {math.degrees(b.yaw):.6f}" '
                     f'friction="1 0.005 0.0001" rgba="1 1 0 1" '
                     f'size="{hx:.6f} {hy:.6f} {hz:.6f}" type="box" mass="0.1" />')
        lines.append('      <joint type="free" />')
        lines.append("    </body>")
    lines.append(f'    <site name="goal" type="sphere" pos="{scene["goal"][0]:.6f} '
                 f'{scene["goal"][1]:.6f} 0.0" size="0.02" rgba="1 0 0 0.5" />')
    lines.append(FOOTER)
    return "\n".join(lines)


def long_axis_bearing_deg(hx, hy, yaw_rad):
    """World bearing of the item's LONG axis, CCW from +X, in [0, 180).

    The ONE number a human needs to place a bar or a block, and the only one that is unambiguous
    without also knowing whose local frame you meant. Three separate 90-degree errors turned up in
    one day from writing an angle down without naming its frame: wall_9's tag is mounted a quarter
    turn off from wall_10/wall_11, this generator numbered every scene's first brick as wall_9, and
    the hardware repo's objects.yaml puts a brick's long side on local Y while this generator's XML
    puts it on local X.

    That last one bites INSIDE this file too: BRICK_HALF is (9.75, 2.75) so a brick's long axis is
    local X, while every entry in MOVABLES is taller than it is wide, so a block's long axis is
    local Y. A raw `yaw` therefore means different things on different rows of the same sheet.
    """
    bearing = math.degrees(yaw_rad) + (0.0 if hx >= hy else 90.0)
    return round(bearing % 180.0, 1)


def _blocker_row(cm, name, b):
    hx, hy, hz = MOVABLES[name]
    return {"object": name,
            "center_cm": [cm(b.cx), cm(b.cy)],
            "long_axis_bearing_deg": long_axis_bearing_deg(b.hx, b.hy, b.yaw),
            "yaw_deg": round(math.degrees(b.yaw) % 180.0, 1),
            "long_cm": round(max(hx, hy) * 200, 1),
            "short_cm": round(min(hx, hy) * 200, 1),
            "height_cm": round(hz * 200, 1),
            "size_cm": [round(hx * 200, 1), round(hy * 200, 1)]}


def to_build_sheet(scene, scene_id):
    """Human-readable placement, centres in cm from the bottom-left interior corner.

    Under `--n-movables 2` this also carries a `"blockers"` list, one row per movable in the same
    shape as the single `"blocker"` entry, so the hardware side can place both. That key is added
    ONLY when there is more than one movable -- the default single-object sheet is untouched, which
    is what keeps it byte-identical to the pre-feature output (see the HARD REQUIREMENT in the
    caller's report).
    """
    cm = lambda v: round(v * 100.0, 1)
    hx, hy, hz = MOVABLES[scene["blocker_name"]]
    b = scene["blocker"]
    sheet = {
        "scene_id": scene_id,
        "arena_cm": [ARENA_W * 100, ARENA_H * 100],
        "angle_convention": ("long_axis_bearing_deg = bearing of the item's LONG side in world, "
                             "counter-clockwise from +X, in [0,180). Place by this, not by yaw. "
                             "yaw_deg is the MuJoCo local-frame rotation and its meaning differs "
                             "between bricks (long side on local X) and blocks (long side on "
                             "local Y)."),
        "tag_convention": ("every bar uses the wall_10/wall_11 ArUco mounting; wall_9 is excluded "
                           "(its tag is rotated 90 deg and it measures 19.0 cm, not 19.5)"),
        "bricks": [{"marker_hint": f"wall_{10 + i}",
                    "center_cm": [cm(r.cx), cm(r.cy)],
                    "long_axis_bearing_deg": long_axis_bearing_deg(r.hx, r.hy, r.yaw),
                    "yaw_deg": round(math.degrees(r.yaw) % 180.0, 1),
                    "long_cm": 19.5, "short_cm": 5.5, "height_cm": 10.0}
                   for i, r in enumerate(scene["statics"])],
        "blocker": {"object": scene["blocker_name"],
                    "center_cm": [cm(b.cx), cm(b.cy)],
                    "long_axis_bearing_deg": long_axis_bearing_deg(b.hx, b.hy, b.yaw),
                    "yaw_deg": round(math.degrees(b.yaw) % 180.0, 1),
                    "long_cm": round(max(hx, hy) * 200, 1),
                    "short_cm": round(min(hx, hy) * 200, 1),
                    "height_cm": round(hz * 200, 1),
                    "size_cm": [round(hx * 200, 1), round(hy * 200, 1)]},
        "robot_start_cm": [cm(scene["start"][0]), cm(scene["start"][1])],
        "robot_start_bearing_deg": ROBOT_START_BEARING_DEG,
        "goal_cm": [cm(scene["goal"][0]), cm(scene["goal"][1])],
        "run_namo_goal_flag": f"--goal {cm(scene['goal'][0]):.0f} {cm(scene['goal'][1]):.0f}",
        "n_bricks": len(scene["statics"]),
        "open_frac": round(scene["open_frac"], 4),
        "n_contacts": scene["n_contacts"],
    }
    blockers = scene.get("blockers") or [(b, scene["blocker_name"])]
    if len(blockers) > 1:
        sheet["blockers"] = [_blocker_row(cm, nm, obj) for obj, nm in blockers]
    return sheet


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--num", type=int, default=500, help="scenes to emit")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-bricks", type=int, default=4,
                    help="bricks the build sheet may call for. 3 are confirmed on the table and a "
                         "4th is declared; the user can add more, and the pocket/center_door "
                         "layouts need them to make anything hard")
    ap.add_argument("--open-frac", default="0,1",
                    help="lo,hi window on the physics-free difficulty proxy. Roughly: easy 0.15+, "
                         "med 0.02-0.12, hard 0.005-0.04. Below ~0.01 scenes tip to unsolvable")
    ap.add_argument("--contacts", default="0,60",
                    help="lo,hi on reachable contact count, the strongest tier signal there is. "
                         "hmax=2 hard wants >=24 with 2 bricks (17.6%% hard vs 3.7%% base); <12 "
                         "means the block is wedged and the scene tends to be UNSOLVABLE, not hard")
    ap.add_argument("--layouts", default=",".join(LAYOUTS),
                    help=f"comma-separated wall layouts; all: {','.join(LAYOUTS)}")
    ap.add_argument("--movables", default=",".join(ON_TABLE),
                    help=f"comma-separated; all known: {','.join(MOVABLES)}")
    ap.add_argument("--margin-cm", type=float, default=10.0,
                    help="post-push corridor width the scene must still admit. 8.0 is the hard "
                         "threshold; ArUco noise is a few mm, so build with headroom")
    ap.add_argument("--n-movables", type=int, default=1, choices=(1, 2),
                    help="1 (default): today's single blocker, unchanged. 2: a second movable is "
                         "placed touching the first inside the passage, so one push can shove both "
                         "-- see FEATURE 1 in the implementation report")
    ap.add_argument("--bands", type=int, default=1, choices=(1, 2),
                    help="1 (default): today's single wall divider, unchanged. 2: two side_gap "
                         "dividers at different y with gaps on opposite sides, so the corridor "
                         "bends (needs --max-bricks >= 2) -- see FEATURE 2 in the report")
    args = ap.parse_args()

    movable_names = [m.strip() for m in args.movables.split(",") if m.strip()]
    unknown = [m for m in movable_names if m not in MOVABLES]
    if unknown:
        ap.error(f"unknown movables: {unknown}")
    if args.n_movables >= 2 and len(movable_names) < 2:
        ap.error("--n-movables 2 needs at least 2 --movables to draw a distinct second one from")
    lo, hi = (float(v) for v in args.open_frac.split(","))
    clo, chi = (int(v) for v in args.contacts.split(","))
    layouts = [l.strip() for l in args.layouts.split(",") if l.strip()]
    bad = [l for l in layouts if l not in LAYOUTS]
    if bad:
        ap.error(f"unknown layouts: {bad}")
    if args.bands >= 2 and args.max_bricks < 2:
        ap.error("--bands 2 needs --max-bricks >= 2 (one brick per band, minimum)")
    margin_r = args.margin_cm / 200.0 + TIER1_MARGIN

    os.makedirs(args.out_dir, exist_ok=True)
    rng = random.Random(args.seed)
    manifest, sheets, rejects = [], [], {}
    attempts = 0
    while len(manifest) < args.num and attempts < args.num * 60:
        attempts += 1
        scene, reason = sample_scene(rng, movable_names, args.max_bricks, margin_r,
                                     (lo, hi), layouts, (clo, chi),
                                     n_movables=args.n_movables, bands=args.bands)
        if scene is None:
            rejects[reason] = rejects.get(reason, 0) + 1
            continue
        sid = f"rb_{len(manifest):05d}"
        d = os.path.join(args.out_dir, sid)
        os.makedirs(d, exist_ok=True)
        xml_path = os.path.join(d, "env.xml")
        with open(xml_path, "w") as f:
            f.write(to_xml(scene))
        manifest.append(os.path.realpath(xml_path))
        sheets.append(to_build_sheet(scene, sid))

    with open(os.path.join(args.out_dir, "manifest.txt"), "w") as f:
        f.write("\n".join(manifest) + "\n")
    with open(os.path.join(args.out_dir, "build_sheets.json"), "w") as f:
        json.dump(sheets, f, indent=2)

    print(f"emitted {len(manifest)} scenes in {attempts} attempts -> {args.out_dir}")
    print(f"inventory: <={args.max_bricks} bricks, movables={movable_names}, "
          f"margin>={args.margin_cm} cm, open_frac in [{lo},{hi}], layouts={layouts}, "
          f"n_movables={args.n_movables}, bands={args.bands}")
    if sheets:
        ofs = sorted(s["open_frac"] for s in sheets)
        print(f"open_frac: min={ofs[0]:.3f} med={ofs[len(ofs)//2]:.3f} max={ofs[-1]:.3f}")
        ncs = sorted(s["n_contacts"] for s in sheets)
        print(f"contacts:  min={ncs[0]} med={ncs[len(ncs)//2]} max={ncs[-1]}")
        import collections as _c
        print("bricks used:", dict(sorted(_c.Counter(s["n_bricks"] for s in sheets).items())))
    if rejects:
        print("rejects:", dict(sorted(rejects.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
