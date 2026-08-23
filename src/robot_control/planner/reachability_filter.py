"""Contact points the planner may offer but the robot cannot stand on.

The planner and the executor each decide occupancy from their own grid. Both
apply the same rule, inflate obstacles by the rotation-safe robot radius plus
the tier-1 margin, but they rasterise it at different resolutions, so near a
wall their answers differ by a cell or two. The numbers, since the asymmetry is
the whole reason this module exists:

    namo_cpp planner    1 cm    `high_level_resolution`,
                                config/namo_config_complete_skill15_car_1x.yaml:28
    this navigator      5 mm    WavefrontPathPlanner's default, not overridden
                                anywhere (runtime.py:556 passes no resolution)

This module matches the NAVIGATOR, because it exists to say what the car can
actually drive to. Do not "align" it to the planner's 1 cm; disagreeing with
the planner is the point. This computes the executor's answer up
front so it can go to the planner as a blacklist before the search starts. It
only ever removes candidates the executor would reject anyway.

WHAT THIS TESTS, AND WHAT IT DOES NOT. Each approach pose is checked with
``WavefrontPlanner.is_free``, which asks whether that cell is clear of inflated
obstacles. It does NOT ask whether the robot can drive there. A pose sitting in
open floor on the far side of a wall passes this filter.

That gap is deliberate, and reversing it would break chained plans. Measured
2026-08-22 on the last real scene: of obstacle_1's 60 approach poses, 26 were
in collision, 34 were clear, and 0 were in the robot's connected region. A
reachability test would blacklist all 60. The blacklist reaches the planner as
``external_edge_blacklist``, which region_opening applies at every chain depth
(see ``region_opening.py:2208``), so that would delete every 2-chain that
opens a path to obstacle_1 and then pushes it. Collision-freedom survives a
first push far better than region membership does.

So this is the right test for a planner blacklist and the wrong test for
predicting a scene's difficulty from one frame. For the latter, flood-fill from
the robot and count what it can actually get to.

Recompute it every plan. Objects move, and an edge blocked in one state is
often free in the next.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Set, Tuple

from robot_control.controller.edge_points import get_edge_point
from robot_control.core.types import ObjectPose, Observation
from robot_control.utils.wavefront import WavefrontConfig, WavefrontPlanner
from robot_control.utils.robot_geometry import effective_robot_radius_cm
from robot_control.utils.wavefront_inflation_config import (
    get_wavefront_inflation_config,
)

# ─── Named constants ────────────────────────────────────────────────────

# Contact points per object face. namo_cpp's production value, giving edge
# indices 0..59 (4 faces x 15).
POINTS_PER_FACE = 15
EDGE_COUNT = 4 * POINTS_PER_FACE

# Standoff from the object face to the approach pose is
# effective_robot_radius_cm + this, the same sum PushController builds
# (controller/push.py:177). The margin mirrors namo_cpp's
# planning.wavefront_edge_offset_margin = 0.01 m and is set in
# config/controller.yaml as push.edge_offset_margin_cm.
PUSH_OFFSET_MARGIN_CM = 1.0

# Occupancy grid resolution the navigator uses, metres per cell.
GRID_RESOLUTION_M = 0.005

# Cell value marking free space in WavefrontPlanner's grid.
_FREE = WavefrontPlanner.FREE


def robot_inflation_radius_cm(width_cm: float, height_cm: float) -> float:
    """The radius both wavefronts inflate by. Delegates, never recomputes.

    This used to return the half-diagonal, hypot(3.5, 3.5) = 4.95 cm for the
    7x7 car, on the reasoning that a robot turning in place sweeps its
    diagonal. That is geometrically true and operationally wrong here. namo_cpp
    settled on max(hx, hy) = 3.5 cm
    (wavefront/goal_tolerance_utils.hpp:12, whose comment records that the
    diagonal "crowded out push approaches"), and robot_geometry.py already
    carried the matching formula with the same history. Inflating by 4.95
    against a planner using 3.5 made this filter reject contacts the planner
    could reach: 1.45 cm of phantom inflation, plus a standoff 2.45 cm too far
    out. Measured 2026-08-22 by dm1487's session across 60 scenes: the C++
    region snapshot disconnects at 4.0 cm total inflation, not 5.45.
    """
    return effective_robot_radius_cm(width_cm, height_cm)


def unreachable_contact_points(
    observation: Observation,
    workspace_width_cm: float,
    workspace_height_cm: float,
    robot_width_cm: float,
    robot_height_cm: float,
    inflation_margin_m: Optional[float] = None,
) -> Set[Tuple[str, int]]:
    """Every (object_id, edge_idx) whose approach pose lands in an obstacle.

    Collision-freedom only, not reachability. See the module docstring for why
    that distinction is load-bearing.

    Shaped to drop straight into ``NAMOPlanBridge.plan(failed_pushes=...)``,
    which maps real object ids to sim names and merges them into
    ``external_edge_blacklist``.

    Raises if the grid cannot be built. The caller decides whether to degrade
    to "filter nothing", and logs it when it does. Swallowing it here would
    make a failed filter and an empty result the same value.
    """
    if inflation_margin_m is None:
        inflation_margin_m = get_wavefront_inflation_config().tier1_base_inflation_margin_m

    radius_cm = robot_inflation_radius_cm(robot_width_cm, robot_height_cm)
    standoff_cm = radius_cm + PUSH_OFFSET_MARGIN_CM

    planner = WavefrontPlanner(
        WavefrontConfig(
            resolution=GRID_RESOLUTION_M,
            robot_radius=radius_cm / 100.0,
            inflation_margin=inflation_margin_m,
        )
    )
    grid_objects: Dict[str, Tuple[float, float, float, float, float]] = {
        name: (
            pose.x / 100.0,
            pose.y / 100.0,
            pose.depth / 200.0,   # half-extent along local X, metres
            pose.width / 200.0,   # half-extent along local Y, metres
            pose.theta,
        )
        for name, pose in observation.objects.items()
    }
    planner.build_grid(
        (0.0, workspace_width_cm / 100.0, 0.0, workspace_height_cm / 100.0),
        grid_objects,
    )

    robot_m = (observation.robot_x / 100.0, observation.robot_y / 100.0)
    # The robot often sits inside its own inflation halo; without this every
    # cell reads blocked and the whole set comes back unreachable.
    planner.apply_trapped_start_recovery(robot_m)

    if planner.get_grid() is None:
        # Returning an empty set here would be indistinguishable from "nothing
        # is blocked", so a grid that failed to build would read as a filter
        # that ran and found nothing. Raise instead and let the caller log the
        # skip. A no-op must never look like a result.
        raise RuntimeError(
            "wavefront grid unavailable, cannot decide contact reachability"
        )

    blocked: Set[Tuple[str, int]] = set()
    for name, pose in observation.objects.items():
        if pose.is_static:
            continue
        for edge_idx in range(EDGE_COUNT):
            point = get_edge_point(
                pose, edge_idx=edge_idx, standoff=standoff_cm,
                points_per_face=POINTS_PER_FACE,
            )
            if not planner.is_free(point.position[0] / 100.0, point.position[1] / 100.0):
                blocked.add((name, edge_idx))
    return blocked
