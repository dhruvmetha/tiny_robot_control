"""Where to send the robot when the goal POINT is covered but the goal
REGION is still open.

A pushed object routinely ends up sitting on top of the exact goal
coordinate even though the surrounding floor is free -- the push only had to
clear a corridor, not that one cell. Left alone, the planner either:

  - dispatches a NavigateSubgoal straight at the covered point, which the
    navigator's own wavefront can never path to (bug: the runtime then
    treats the failed navigate as an unrecoverable IDLE and wedges), or
  - decides the goal is BLOCKED and runs a full NAMO push search whose only
    job is to nudge the object a few centimetres off the goal point -- a
    search-sized hammer for a problem the navigator could have sidestepped.

This module answers a narrower question first: is there free, robot-
reachable floor within a small radius of the goal point? If so, retarget the
navigation there instead of running NAMO planning at all.

Reuses the same WavefrontPlanner class, grid resolution, and inflation
convention as ``reachability_filter.py``, because this has to agree with
what the navigator (WavefrontPathPlanner) can actually drive to -- not with
the C++ planner's coarser region-reachability check. See
``reachability_filter.py``'s module docstring for why those two grids
disagree and which one each caller should use.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, Optional, Tuple

import numpy as np

from robot_control.core.types import Observation
from robot_control.planner.reachability_filter import (
    GRID_RESOLUTION_M,
    robot_inflation_radius_cm,
)
from robot_control.utils.wavefront import WavefrontConfig, WavefrontPlanner
from robot_control.utils.wavefront_inflation_config import (
    get_wavefront_inflation_config,
)

# ─── Named constants ────────────────────────────────────────────────────

# Neighbours the flood fill steps through. EIGHT, matching the planner's own
# BFS (namo_cpp `src/wavefront/wavefront_planner.cpp:563`) and
# scripts/make_build_cards.py's FLOOD_FILL_NEIGHBOURS: the car drives
# diagonally, so a 4-connected fill seals gaps it can actually pass.
_FLOOD_FILL_NEIGHBOURS = (
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)


def find_goal_retarget(
    observation: Observation,
    robot_goal_cm: Tuple[float, float],
    cap_cm: float,
    workspace_width_cm: float,
    workspace_height_cm: float,
    robot_width_cm: float,
    robot_height_cm: float,
) -> Optional[Tuple[float, float, float]]:
    """Nearest free, robot-reachable cell to the goal point.

    Builds the navigator's own occupancy grid (same resolution and
    inflation as ``reachability_filter.unreachable_contact_points``),
    flood-fills it from the robot's cell, and returns whichever reachable
    free cell sits closest (Euclidean, cm) to ``robot_goal_cm``.

    Returns None in two cases the caller must tell apart by re-checking the
    goal cell itself if it needs to: the goal cell is already free (nothing
    to retarget), or the nearest reachable free cell is farther than
    ``cap_cm`` (something other than a single pushed object is blocking the
    goal, so retargeting would just paper over a bigger obstruction).

    Returns (x_cm, y_cm, distance_cm) otherwise.
    """
    radius_cm = robot_inflation_radius_cm(robot_width_cm, robot_height_cm)
    margin_m = get_wavefront_inflation_config().tier1_base_inflation_margin_m

    planner = WavefrontPlanner(WavefrontConfig(
        resolution=GRID_RESOLUTION_M,
        robot_radius=radius_cm / 100.0,
        inflation_margin=margin_m,
    ))
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

    goal_m = (robot_goal_cm[0] / 100.0, robot_goal_cm[1] / 100.0)
    if planner.is_free(*goal_m):
        # Goal cell is not blocked -- nothing to retarget.
        return None

    robot_m = (observation.robot_x / 100.0, observation.robot_y / 100.0)
    # The robot often sits inside its own inflation halo; without this the
    # flood fill can't even start. Same recovery reachability_filter and
    # make_build_cards.py apply before flood-filling from the robot cell.
    planner.apply_trapped_start_recovery(robot_m)

    grid = planner.get_grid()
    if grid is None:
        return None

    free = grid == WavefrontPlanner.FREE
    height, width = free.shape
    start_gi, start_gj = planner._world_to_grid(*robot_m)
    if not (0 <= start_gi < width and 0 <= start_gj < height and free[start_gj, start_gi]):
        return None

    reachable = np.zeros_like(free)
    reachable[start_gj, start_gi] = True
    queue = deque([(start_gj, start_gi)])
    while queue:
        j, i = queue.popleft()
        for dj, di in _FLOOD_FILL_NEIGHBOURS:
            nj, ni = j + dj, i + di
            if (0 <= nj < height and 0 <= ni < width
                    and free[nj, ni] and not reachable[nj, ni]):
                reachable[nj, ni] = True
                queue.append((nj, ni))

    js, is_ = np.where(reachable)
    if js.size == 0:
        return None

    res_m = planner.get_resolution()
    x_min, _, y_min, _ = planner.get_bounds()
    xs_m = x_min + (is_ + 0.5) * res_m
    ys_m = y_min + (js + 0.5) * res_m
    dists_cm = np.hypot(xs_m - goal_m[0], ys_m - goal_m[1]) * 100.0

    best_idx = int(np.argmin(dists_cm))
    best_dist_cm = float(dists_cm[best_idx])
    if best_dist_cm > cap_cm:
        return None

    return (float(xs_m[best_idx] * 100.0), float(ys_m[best_idx] * 100.0), best_dist_cm)
