"""Pure-navigation baselines: what a robot that never pushes can reach.

Every arm in the trial matrix pushes something. Nothing in it shows what a
robot achieves by driving alone, which is the floor all of them are supposed
to clear. Without that row, "the method solves hard NAMO scenes" invites the
obvious objection that some of those scenes were never NAMO problems.

Two variants, both of which drive at the goal and never plan a push:

  MOVABLE_COST_IGNORED    a movable cell costs what open floor costs, so the
                          search cannot tell them apart and drives the
                          shortest line to the goal.
  MOVABLE_COST_PENALISED  a movable cell costs more, so the route prefers
                          clear space but still crosses a block when going
                          round is far enough.

Only static geometry blocks either variant. That is the point: a movable is
something the robot may drive into, not something it must respect. On the
table it will shove the block or stall against it, and which one happens is
the measurement.

This is NOT namo_cpp's `geometric_transport_strategy`. That is a NAMO
baseline which chooses objects to push. This one chooses nothing and pushes
nothing.

`WavefrontPlanner` does the work. Its occupancy grid decides what blocks, and
its Dijkstra already multiplies every step by a per-cell entry in
`_cost_grid` (wavefront.py:615), which is exactly what the two variants
differ in. A second grid and a second Dijkstra here would put the baseline on
a different representation from the one `config/controller.yaml` selects for
real trials, and the comparison would then measure path planners rather than
pushing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from robot_control.planner.reachability_filter import (
    GRID_RESOLUTION_M,
    robot_inflation_radius_cm,
)
from robot_control.utils.wavefront import WavefrontConfig, WavefrontPlanner
from robot_control.utils.wavefront_inflation_config import get_wavefront_inflation_config

# Multipliers on the cost of entering one cell. Open floor is 1.0, which is
# what `_build_cost_grid` fills a grid with when proximity weighting is off.
COST_FREE = 1.0
# Variant 1. Identical to free space, so the search is blind to movables.
MOVABLE_COST_IGNORED = 1.0
# Variant 2. Crossing one cell of a block is worth five cells of detour.
# Arbitrary, which is why `sweep_movable_cost` exists: the number worth
# reporting is the penalty at which a scene stops driving through and starts
# going round, not this default.
MOVABLE_COST_PENALISED = 5.0

# Proximity weighting nudges routes away from obstacle edges, which is right
# for driving a real robot and wrong for a baseline. It would add cost near a
# block on top of the cost of the block itself, so a scene's threshold would
# mix two effects and report neither cleanly. Off here; trials keep their own.
PROXIMITY_WEIGHT_OFF = 0.0

# A body whose name ends this way is pushable; everything else is wall. Same
# rule the rest of the pipeline uses.
MOVABLE_SUFFIX = "_movable"

# `WavefrontPlanner.build_grid` takes objects as
# {name: (x_m, y_m, half_width_m, half_depth_m, theta_deg)}. Aliased rather
# than wrapped so callers pass what they already build for the planner.
ObjectBoxes = Dict[str, Tuple[float, float, float, float, float]]


@dataclass
class NavResult:
    """What one variant found."""

    reached: bool
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    # Cells of each movable the route crosses. Empty on a reached route means
    # the robot got there touching nothing, so the scene is navigable outright
    # and never needed a push.
    movable_cells_crossed: Dict[str, int] = field(default_factory=dict)
    failure: Optional[str] = None
    # The robot began inside inflated static geometry. `WavefrontPlanner`
    # frees a trapped start before routing, deliberately and matching the C++,
    # so this is not a failure and the route may still be good. It is recorded
    # because at the table it usually means the capture or the ArUco pose was
    # off, and a route replanned out of a wall is worth knowing about.
    start_was_trapped: bool = False

    @property
    def crosses_nothing(self) -> bool:
        return self.reached and not self.movable_cells_crossed


class NavigationBaseline:
    """One scene, planned on the same wavefront the real trials navigate on.

    Statics go into the occupancy grid, so they block. Movables go into the
    cost grid, so they are merely expensive at whatever price `plan` is given.
    """

    def __init__(
        self,
        bounds: Tuple[float, float, float, float],
        statics: ObjectBoxes,
        movables: ObjectBoxes,
        robot_width_cm: float,
        robot_height_cm: float,
        resolution_m: float = GRID_RESOLUTION_M,
    ) -> None:
        self.bounds = bounds

        # Both numbers delegate. `robot_inflation_radius_cm` carries this
        # repository's history of disagreeing with namo_cpp about the radius,
        # and the margin lives in one config file because the two wavefronts
        # disagreeing about it is BUG-001.
        radius_m = robot_inflation_radius_cm(robot_width_cm, robot_height_cm) / 100.0
        margin_m = get_wavefront_inflation_config().tier1_base_inflation_margin_m
        self._config = WavefrontConfig(
            resolution=resolution_m,
            robot_radius=radius_m,
            inflation_margin=margin_m,
            obstacle_proximity_weight=PROXIMITY_WEIGHT_OFF,
        )

        self.planner = WavefrontPlanner(self._config)
        self.planner.build_grid(bounds, dict(statics))
        free = self.planner.get_grid() == WavefrontPlanner.FREE

        # Which movable owns which cell, learned by flooding each one alone.
        # A route has to report what it drove through, not only how much.
        # Intersecting with the static-free mask means a cell a wall already
        # covers is never claimed, so a wall overlapping a block still blocks.
        self._owner_cells: Dict[str, np.ndarray] = {}
        for name, box in movables.items():
            probe = WavefrontPlanner(self._config)
            probe.build_grid(bounds, {name: box})
            self._owner_cells[name] = (
                probe.get_grid() == WavefrontPlanner.OBSTACLE
            ) & free

    def plan(
        self,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
        movable_cost: float,
    ) -> NavResult:
        """Cheapest drive from start to goal. Only static geometry blocks.

        `movable_cost` of 1.0 makes blocks invisible to the search; anything
        higher makes them worth avoiding without making them walls.
        """
        if movable_cost < COST_FREE:
            raise ValueError(
                f"movable_cost must be at least {COST_FREE}, the cost of open "
                f"floor; got {movable_cost}. A block cheaper than floor would "
                f"make the route seek obstacles out."
            )

        # Price the movables into the planner's own cost grid, then let its
        # Dijkstra do the search. Rebuilt per call because the penalty is the
        # one thing the two variants differ in.
        cost_grid = np.ones_like(self.planner._cost_grid)
        for covered in self._owner_cells.values():
            cost_grid[covered] = movable_cost
        self.planner._cost_grid = cost_grid

        trapped = not self.planner.is_free(*start_xy)
        waypoints = self.planner.plan(start_xy, goal_xy)
        if not waypoints:
            return NavResult(
                False,
                failure="no_route_around_static_geometry",
                start_was_trapped=trapped,
            )
        return NavResult(
            True, waypoints, self._crossings(waypoints), start_was_trapped=trapped
        )

    def plan_for_image(self, movable_cost: float) -> None:
        """Leave the planner's cost grid priced, so `save` can draw it.

        `plan` rewrites the cost grid on every call, so by the time a caller
        wants a picture the grid holds whichever penalty ran last. This puts
        the one being drawn back.
        """
        cost_grid = np.ones_like(self.planner._cost_grid)
        for covered in self._owner_cells.values():
            cost_grid[covered] = movable_cost
        self.planner._cost_grid = cost_grid

    def _crossings(self, waypoints: Sequence[Tuple[float, float]]) -> Dict[str, int]:
        crossed: Dict[str, int] = {}
        for x, y in waypoints:
            gi, gj = self.planner._world_to_grid(x, y)
            for name, covered in self._owner_cells.items():
                if 0 <= gj < covered.shape[0] and 0 <= gi < covered.shape[1]:
                    if covered[gj, gi]:
                        crossed[name] = crossed.get(name, 0) + 1
        return crossed


def sweep_movable_cost(
    baseline: NavigationBaseline,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    penalties: Sequence[float],
) -> Optional[float]:
    """Lowest penalty at which the route stops crossing any movable.

    A scene that goes clear at a low penalty had a cheap way round all along
    and never needed a push. One that never goes clear, however high the
    penalty, has no route to the goal except through a block. That threshold
    says more about a scene than any single penalty does and costs nothing to
    measure, so `MOVABLE_COST_PENALISED` never has to be argued for.

    Returns None when no penalty in `penalties` produces a clear route.
    """
    for penalty in sorted(penalties):
        if baseline.plan(start_xy, goal_xy, penalty).crosses_nothing:
            return penalty
    return None
