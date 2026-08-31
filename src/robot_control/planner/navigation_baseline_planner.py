"""Drive at the goal and never push. The floor the NAMO arms must clear.

`navigation_baseline` answers the question on a grid. This answers it on the
table, which is the only place the answer counts. A scene the grid calls
unreachable might still be drivable if some constant is off, and a robot that
arrives is not arguable the way a reachability check is.

Two modes, matching the two grid variants. Both plan with `NavigationBaseline`,
where static geometry blocks and a movable is merely priced:

  ignore    a movable cell costs what open floor costs, so the route is the
            shortest line and the robot drives into whatever sits on it.
  penalise  a movable cell costs five, so the route prefers clear space but
            still crosses a block when going round is far enough.

The route goes out as one `NavigateSubgoal` carrying its own path, so Runtime,
the executor, the navigation controller and the path follower drive it exactly
as they drive a NAMO subgoal. The route has to be planned here rather than by
the navigation controller because that controller takes obstacles, which block,
and the difference between these two variants is a cost, which does not.

Never emits a `PushSubgoal`, and never replans. One drive, one verdict. A run
that reaches the goal proves the scene did not need NAMO. A run that stalls is
the measured case for it, and the scene gets reset before the NAMO trial so
both runs face the same layout.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from robot_control.core.types import NavigateSubgoal, Observation, Subgoal
from robot_control.planner.base import Planner
from robot_control.planner.navigation_baseline import (
    MOVABLE_COST_IGNORED,
    MOVABLE_COST_PENALISED,
    NavigationBaseline,
)

# What each mode charges for entering a cell a movable covers.
MODE_COSTS = {
    "ignore": MOVABLE_COST_IGNORED,
    "penalise": MOVABLE_COST_PENALISED,
}
BASELINE_MODES = tuple(MODE_COSTS)

# How close counts as arrival. Matches the runtime's own goal tolerance so a
# baseline arrival and a NAMO arrival mean the same thing; a baseline judged on
# a looser bar would flatter itself against the arms it sits below.
GOAL_TOLERANCE_CM = 5.0

# A block that shifts by more than this was hit rather than jostled by camera
# noise. ArUco pose jitter sits well under a centimetre on this table.
MOVED_THRESHOLD_CM = 1.0

# Give up rather than grind. A scene this baseline cannot solve is the result,
# so a longer stall buys nothing, and the operator has a reset and a NAMO trial
# waiting.
DEFAULT_TIMEOUT_S = 90.0


@dataclass
class BaselineOutcome:
    """What the drive did, for the row that sits beside the NAMO trial."""

    mode: str
    reached: bool = False
    stopped_at_cm: Optional[Tuple[float, float]] = None
    distance_to_goal_cm: Optional[float] = None
    elapsed_s: float = 0.0
    route_waypoints: int = 0
    # Cells of each movable the planned route crosses. On the ignore variant
    # this is what the robot is about to drive into.
    route_crosses: Dict[str, int] = field(default_factory=dict)
    # Objects whose pose moved more than MOVED_THRESHOLD_CM, and by how far.
    objects_moved_cm: Dict[str, float] = field(default_factory=dict)
    failure: Optional[str] = None

    def as_row(self) -> Dict[str, Any]:
        stopped = self.stopped_at_cm
        return {
            "arm": f"nav_baseline_{self.mode}",
            "reached": self.reached,
            "stopped_x_cm": None if stopped is None else round(stopped[0], 2),
            "stopped_y_cm": None if stopped is None else round(stopped[1], 2),
            "distance_to_goal_cm": (
                None if self.distance_to_goal_cm is None
                else round(self.distance_to_goal_cm, 2)
            ),
            "elapsed_s": round(self.elapsed_s, 1),
            "route_waypoints": self.route_waypoints,
            "route_crosses": ",".join(sorted(self.route_crosses)),
            "objects_moved": ",".join(sorted(self.objects_moved_cm)),
            "objects_moved_cm": {k: round(v, 2) for k, v in self.objects_moved_cm.items()},
            "failure_cause": self.failure or "",
        }


class NavigationBaselinePlanner(Planner):
    """Emits one navigate subgoal carrying its own route, then stops."""

    def __init__(
        self,
        goal_cm: Tuple[float, float],
        workspace_bounds_m: Tuple[float, float, float, float],
        robot_width_cm: float,
        robot_height_cm: float,
        mode: str = "ignore",
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> None:
        if mode not in MODE_COSTS:
            raise ValueError(f"Unknown mode {mode!r}. Valid: {list(BASELINE_MODES)}")
        self.goal_cm = goal_cm
        self.mode = mode
        self.timeout_s = timeout_s
        self._bounds_m = workspace_bounds_m
        self._robot_w = robot_width_cm
        self._robot_h = robot_height_cm

        self.outcome = BaselineOutcome(mode=mode)
        self._started: Optional[float] = None
        self._first_poses: Dict[str, Tuple[float, float]] = {}
        self._route_cm: List[Tuple[float, float]] = []
        self._dispatched = False
        self._drive_over = False

    def plan(self, obs: Observation) -> Optional[Subgoal]:
        if self._started is None:
            self._started = time.time()
            self._first_poses = self._object_poses(obs)
            self._route_cm = self._plan_route(obs)

        # One drive is the whole plan. Re-emitting would restart it every tick,
        # and replanning after it ends would turn a stall into a retry loop,
        # which is the opposite of the measurement.
        if self._dispatched or self._drive_over or not self._route_cm:
            return None

        self._dispatched = True
        return NavigateSubgoal(
            x=self.goal_cm[0],
            y=self.goal_cm[1],
            theta=None,
            path=self._route_cm,
        )

    def is_complete(self, obs: Observation) -> bool:
        self._record(obs)
        if self.outcome.distance_to_goal_cm <= GOAL_TOLERANCE_CM:
            self.outcome.reached = True
            self.outcome.failure = None
            return True
        if not self._route_cm:
            return True
        if self._drive_over:
            self.outcome.failure = self.outcome.failure or "stopped_short_of_goal"
            return True
        if self._started is not None and time.time() - self._started > self.timeout_s:
            self.outcome.failure = "timed_out_short_of_goal"
            return True
        return False

    def notify_subgoal_done(self, obs: Observation, failed: bool = False) -> None:
        """The drive ended, so the run ends. Whether it arrived is decided by
        distance, not by this verdict: a controller reports done when it runs
        out of path, and a robot stalled against a block is exactly the case
        worth recording rather than retrying.
        """
        self._drive_over = True
        self._dispatched = False
        self._record(obs)
        if failed and self.outcome.distance_to_goal_cm > GOAL_TOLERANCE_CM:
            self.outcome.failure = "navigation_controller_gave_up"

    def _plan_route(self, obs: Observation) -> List[Tuple[float, float]]:
        statics, movables = self._split_objects(obs)
        baseline = NavigationBaseline(
            bounds=self._bounds_m,
            statics=statics,
            movables=movables,
            robot_width_cm=self._robot_w,
            robot_height_cm=self._robot_h,
        )
        result = baseline.plan(
            (obs.robot_x / 100.0, obs.robot_y / 100.0),
            (self.goal_cm[0] / 100.0, self.goal_cm[1] / 100.0),
            MODE_COSTS[self.mode],
        )
        self.outcome.route_crosses = dict(result.movable_cells_crossed)
        if not result.reached:
            self.outcome.failure = result.failure
            return []
        route = [(x * 100.0, y * 100.0) for x, y in result.waypoints]
        self.outcome.route_waypoints = len(route)
        return route

    def _split_objects(self, obs: Observation):
        """Walls block the route; movables only cost. That split is the baseline."""
        statics: Dict[str, Tuple[float, float, float, float, float]] = {}
        movables: Dict[str, Tuple[float, float, float, float, float]] = {}
        for name, pose in (obs.objects or {}).items():
            if pose.width <= 0 or pose.depth <= 0:
                continue
            # WavefrontPlanner takes (x_m, y_m, half X size, half Y size, deg),
            # where X is along heading (depth) and Y across it (width), which
            # is the conversion WavefrontPathPlanner._convert_obstacles makes.
            box = (
                pose.x / 100.0,
                pose.y / 100.0,
                pose.depth / 200.0,
                pose.width / 200.0,
                pose.theta,
            )
            (statics if pose.is_static else movables)[name] = box
        return statics, movables

    def _record(self, obs: Observation) -> None:
        self.outcome.stopped_at_cm = (obs.robot_x, obs.robot_y)
        self.outcome.distance_to_goal_cm = self._distance_to_goal(obs)
        if self._started is not None:
            self.outcome.elapsed_s = time.time() - self._started
        self.outcome.objects_moved_cm = self._moved_since_start(obs)

    def _object_poses(self, obs: Observation) -> Dict[str, Tuple[float, float]]:
        return {n: (p.x, p.y) for n, p in (obs.objects or {}).items()}

    def _moved_since_start(self, obs: Observation) -> Dict[str, float]:
        moved: Dict[str, float] = {}
        for name, (x0, y0) in self._first_poses.items():
            pose = (obs.objects or {}).get(name)
            if pose is None:
                continue
            shifted = ((pose.x - x0) ** 2 + (pose.y - y0) ** 2) ** 0.5
            if shifted > MOVED_THRESHOLD_CM:
                moved[name] = shifted
        return moved

    def _distance_to_goal(self, obs: Observation) -> float:
        return (
            (obs.robot_x - self.goal_cm[0]) ** 2 + (obs.robot_y - self.goal_cm[1]) ** 2
        ) ** 0.5

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Goal ring, and the route the baseline chose."""
        drawings: List[Dict[str, Any]] = [
            {"type": "circle", "x": self.goal_cm[0], "y": self.goal_cm[1],
             "radius": GOAL_TOLERANCE_CM, "color": "green"}
        ]
        if self._route_cm:
            drawings.append({"type": "path", "points": self._route_cm, "color": "blue"})
        return drawings

    def reset(self) -> None:
        self.outcome = BaselineOutcome(mode=self.mode)
        self._started = None
        self._first_poses = {}
        self._route_cm = []
        self._dispatched = False
        self._drive_over = False
