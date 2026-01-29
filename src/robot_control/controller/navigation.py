"""Navigation controller with RVG path planning.

Provides click-to-navigate functionality with obstacle avoidance.
Uses composition: delegates path following to FollowPathController.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from robot_control.controller.base import Controller
from robot_control.controller.config import NavigationConfig
from robot_control.controller.follow_path import FollowPathController
from robot_control.core.types import Action, Observation, Subgoal, WorkspaceConfig
from robot_control.planner.rvg_planner import RVGPlanner

Point = Tuple[float, float]
ObstacleTuple = Tuple[float, float, float, float, float]  # (x, y, theta_deg, w, h)


def _wrap_to_180(deg: float) -> float:
    """Wrap angle to [-180, 180)."""
    while deg >= 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _enforce_deadband_scale(
    vl: float, vr: float, deadband: float = 0.05
) -> Tuple[float, float]:
    """
    Scale wheel speeds to overcome motor deadband.

    If both wheels are below deadband but non-zero, scale up so the
    larger wheel reaches deadband. This prevents motors from stalling.
    """
    max_abs = max(abs(vl), abs(vr))

    # Both near zero - stop
    if max_abs < 1e-6:
        return 0.0, 0.0

    # Already above deadband - no change needed
    if max_abs >= deadband:
        return vl, vr

    # Scale up to reach deadband
    scale = deadband / max_abs
    vl2 = vl * scale
    vr2 = vr * scale

    # Ensure still in [-1, 1]
    m2 = max(abs(vl2), abs(vr2))
    if m2 > 1.0:
        s2 = 1.0 / m2
        vl2 *= s2
        vr2 *= s2

    return vl2, vr2


def _filter_duplicate_points(path: List[Point], min_dist: float = 1.0) -> List[Point]:
    """Remove consecutive duplicate points from path."""
    if not path:
        return []

    filtered = [path[0]]
    for pt in path[1:]:
        dx = pt[0] - filtered[-1][0]
        dy = pt[1] - filtered[-1][1]
        if math.hypot(dx, dy) > min_dist:
            filtered.append(pt)

    return filtered


class NavigationState(Enum):
    """Navigation state machine states."""

    IDLE = "IDLE"
    ROTATING = "ROTATING"  # Standalone rotation (no navigation)
    PRE_ROTATING = "PRE_ROTATING"  # Rotate toward first waypoint
    FOLLOWING = "FOLLOWING"  # Following path (delegated)
    POST_ROTATING = "POST_ROTATING"  # Rotate to goal theta
    FINISHED = "FINISHED"


class NavigationController(Controller):
    """
    Click-to-navigate controller with obstacle avoidance.

    Uses composition: delegates path following to FollowPathController.
    Adds: RVG planning, pre/post rotation, state machine.

    State machine:
        IDLE -> PRE_ROTATING -> FOLLOWING -> POST_ROTATING -> FINISHED
                    |              |
                    +-> FOLLOWING -+  (skip pre-rotation if aligned)
                                   |
                                   +-> FINISHED (skip post-rotation if no goal theta)

    Usage:
        planner = RVGPlanner(width, height, car_w, car_h)
        controller = NavigationController(config, planner)

        # Navigate to click position
        obstacles = [(x, y, theta, w, h), ...]
        if controller.navigate_to(goal_x, goal_y, None, (robot_x, robot_y), obstacles):
            print("Navigating!")
        else:
            print("Planning failed")

        # In control loop:
        action = controller.step(obs, subgoal=None)
        env.apply(action)
    """

    def __init__(
        self,
        config: WorkspaceConfig,
        planner: RVGPlanner,
        nav_config: Optional[NavigationConfig] = None,
        max_speed: Optional[float] = None,
    ) -> None:
        """
        Initialize navigation controller.

        Args:
            config: Workspace configuration (dimensions, robot geometry)
            planner: RVG path planner
            nav_config: Navigation parameters (from YAML). If None, uses defaults.
            max_speed: Override max speed (if None, uses nav_config value)
        """
        self._config = config
        self._planner = planner
        self._nav_config = nav_config or NavigationConfig()
        self._max_speed = max_speed if max_speed is not None else self._nav_config.max_speed

        # Path follower (delegated)
        car_size = max(config.car_width, config.car_height)
        self._path_follower = FollowPathController(
            config,
            max_speed=self._max_speed,
            goal_tolerance=self._nav_config.goal_tolerance_ratio * car_size,
        )

        # State machine
        self._state = NavigationState.IDLE

        # Rotation stable hold state
        self._rotation_stable_start: Optional[float] = None

        # Targets
        self._target_theta: Optional[float] = None  # Post-rotation target (degrees)
        self._pre_rotation_target: Optional[float] = None  # Pre-rotation target (degrees)

        # Path visualization
        self._planned_path: List[Point] = []

        # Obstacle visualization (stored when navigate_to is called)
        self._obstacles: List[ObstacleTuple] = []

    @property
    def state(self) -> NavigationState:
        """Current navigation state."""
        return self._state

    @property
    def max_speed(self) -> float:
        """Current maximum speed."""
        return self._max_speed

    def set_speed(self, speed: float) -> None:
        """
        Set maximum speed.

        Args:
            speed: Maximum wheel speed [0, 1]
        """
        self._max_speed = max(0.0, min(1.0, speed))
        self._path_follower.set_speed(speed)

    def navigate_to(
        self,
        goal_x: float,
        goal_y: float,
        goal_theta: Optional[float],
        current_pos: Point,
        obstacles: List[ObstacleTuple],
    ) -> bool:
        """
        Plan and start navigation to goal.

        Args:
            goal_x: Goal x position
            goal_y: Goal y position
            goal_theta: Optional goal orientation (degrees). If None, no post-rotation.
            current_pos: Current (x, y) position
            obstacles: List of (x, y, theta_deg, width, height) obstacles

        Returns:
            True if planning succeeded and navigation started
        """
        # Store obstacles for visualization
        self._obstacles = obstacles

        # Plan path
        raw_path = self._planner.plan(current_pos, (goal_x, goal_y), obstacles)

        # Filter duplicate points (RVG returns duplicates at start/end)
        car_size = max(self._config.car_width, self._config.car_height)
        path = _filter_duplicate_points(raw_path, min_dist=0.1 * car_size)

        if not path or len(path) < 2:
            # Planning failed or trivial path
            self._state = NavigationState.IDLE
            return False

        self._planned_path = path
        self._target_theta = goal_theta

        # Find first waypoint that's different from current position
        # RVG returns duplicate points at start/end for rotational waypoints
        first_target = None
        car_size = max(self._config.car_width, self._config.car_height)
        min_dist_threshold = 0.5 * car_size  # Half of robot size
        for pt in path:
            dx = pt[0] - current_pos[0]
            dy = pt[1] - current_pos[1]
            if math.hypot(dx, dy) > min_dist_threshold:
                first_target = pt
                break

        if first_target is None:
            # All waypoints are near current position - already at goal
            if goal_theta is not None:
                self._state = NavigationState.POST_ROTATING
            else:
                self._state = NavigationState.FINISHED
            return True

        # Calculate angle to first target
        dx = first_target[0] - current_pos[0]
        dy = first_target[1] - current_pos[1]

        # Calculate pre-rotation target angle
        target_angle = math.degrees(math.atan2(dy, dx))

        # Store path in follower
        self._path_follower.set_path(path)

        # Set state (pre-rotation will be checked in step())
        self._pre_rotation_target = target_angle
        self._state = NavigationState.PRE_ROTATING

        return True

    def rotate_to(self, target_theta: float) -> bool:
        """
        Start rotating to target heading (no navigation).

        Args:
            target_theta: Target orientation in degrees

        Returns:
            True (always succeeds, just sets up rotation)
        """
        self._target_theta = target_theta
        self._rotation_stable_start = None
        self._state = NavigationState.ROTATING
        return True

    def cancel(self) -> None:
        """Cancel navigation and stop."""
        self._state = NavigationState.IDLE
        self._path_follower.clear_path()
        self._target_theta = None
        self._pre_rotation_target = None
        self._rotation_stable_start = None
        self._planned_path = []
        self._obstacles = []

    def step(self, obs: Observation, subgoal: Subgoal) -> Action:
        """Compute action given observation."""
        if self._state == NavigationState.IDLE:
            return Action.stop()

        if self._state == NavigationState.FINISHED:
            return Action.stop()

        robot_theta = obs.robot_theta

        # ROTATING: Standalone rotate-in-place
        if self._state == NavigationState.ROTATING:
            if self._target_theta is None:
                self._state = NavigationState.FINISHED
                return Action.stop()

            heading_error = _wrap_to_180(self._target_theta - robot_theta)

            action = self._handle_rotation(heading_error)
            if action is None:
                self._rotation_stable_start = None
                self._state = NavigationState.FINISHED
                return Action.stop()
            return action

        # PRE_ROTATING: Rotate toward first waypoint
        if self._state == NavigationState.PRE_ROTATING:
            if self._pre_rotation_target is None:
                self._rotation_stable_start = None
                self._state = NavigationState.FOLLOWING
            else:
                heading_error = _wrap_to_180(self._pre_rotation_target - robot_theta)

                # Skip pre-rotation if already reasonably aligned
                if abs(heading_error) < self._nav_config.pre_rotation_skip_angle:
                    self._rotation_stable_start = None
                    self._state = NavigationState.FOLLOWING
                else:
                    # Check if within tolerance (with stable hold)
                    action = self._handle_rotation(heading_error)
                    if action is None:
                        # Rotation complete
                        self._rotation_stable_start = None
                        self._state = NavigationState.FOLLOWING
                    else:
                        return action

        # FOLLOWING: Delegate to path follower
        if self._state == NavigationState.FOLLOWING:
            if self._path_follower.is_done(obs, subgoal):
                if self._target_theta is not None:
                    self._rotation_stable_start = None
                    self._state = NavigationState.POST_ROTATING
                else:
                    self._state = NavigationState.FINISHED
                    return Action.stop()
            return self._path_follower.step(obs, subgoal)

        # POST_ROTATING: Rotate to goal orientation
        if self._state == NavigationState.POST_ROTATING:
            if self._target_theta is None:
                self._state = NavigationState.FINISHED
                return Action.stop()

            heading_error = _wrap_to_180(self._target_theta - robot_theta)

            action = self._handle_rotation(heading_error)
            if action is None:
                # Rotation complete
                self._rotation_stable_start = None
                self._state = NavigationState.FINISHED
                return Action.stop()
            return action

        return Action.stop()

    def _handle_rotation(self, heading_error: float) -> Optional[Action]:
        """
        Handle rotation with stable hold.

        Args:
            heading_error: Angle error in degrees (positive = rotate CCW)

        Returns:
            Action to take, or None if rotation is complete
        """
        if abs(heading_error) <= self._nav_config.rotation_tolerance_deg:
            # Within tolerance - check stable hold
            now = time.time()

            if self._rotation_stable_start is None:
                # Start stable hold timer
                self._rotation_stable_start = now
                return Action.stop()

            elif now - self._rotation_stable_start >= self._nav_config.rotation_stable_time:
                # Stable hold complete - rotation done
                return None

            else:
                # Still in stable hold period
                return Action.stop()
        else:
            # Not within tolerance - reset stable hold and rotate
            self._rotation_stable_start = None
            return self._calculate_rotation_action(heading_error)

    def _calculate_rotation_action(self, heading_error: float) -> Action:
        """
        Calculate in-place rotation action with variable speed.

        Speed varies based on error:
        - > 45°: rotation_speed_max
        - 15-45°: Linear interpolation
        - < 15°: rotation_speed_min

        Args:
            heading_error: Angle error in degrees (positive = rotate CCW)

        Returns:
            Differential drive action for rotation
        """
        abs_error = abs(heading_error)
        speed_max = self._nav_config.rotation_speed_max
        speed_min = self._nav_config.rotation_speed_min

        # Variable speed based on error (matching micromvp)
        if abs_error > 45.0:
            speed = speed_max
        elif abs_error > 15.0:
            # Linear interpolation between min and max
            t = (abs_error - 15.0) / 30.0
            speed = speed_min + t * (speed_max - speed_min)
        else:
            speed = speed_min

        if heading_error > 0:
            # Rotate CCW: left wheel backward, right wheel forward
            left_speed = -speed
            right_speed = speed
        else:
            # Rotate CW: left wheel forward, right wheel backward
            left_speed = speed
            right_speed = -speed

        # Apply deadband scaling
        left_speed, right_speed = _enforce_deadband_scale(
            left_speed, right_speed, self._nav_config.wheel_deadband
        )

        return Action(
            left_speed=_clamp(left_speed, -1.0, 1.0),
            right_speed=_clamp(right_speed, -1.0, 1.0),
        )

    def is_done(self, obs: Observation, subgoal: Subgoal) -> bool:
        """Check if navigation is complete."""
        return self._state == NavigationState.FINISHED

    def reset(self) -> None:
        """Reset controller state."""
        self._state = NavigationState.IDLE
        self._path_follower.reset()
        self._target_theta = None
        self._pre_rotation_target = None
        self._rotation_stable_start = None
        self._planned_path = []
        self._obstacles = []

    def get_status(self) -> str:
        """Get current status label."""
        return self._state.value

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Get drawing commands for canvas visualization."""
        drawings = []

        # Draw obstacles as the planner sees them
        for i, obs in enumerate(self._obstacles):
            x, y, theta_deg, width, depth = obs
            corners = self._obstacle_corners(x, y, theta_deg, width, depth)
            # Close the polygon by repeating first point
            corners.append(corners[0])
            drawings.append({
                "uuid": f"obstacle_{i}",
                "type": "path",
                "points": corners,
                "color": "#FF6600",  # Orange for obstacles
                "width": 2,
            })

        # Draw planned path
        if self._planned_path and len(self._planned_path) >= 2:
            drawings.append({
                "uuid": "navigation_path",
                "type": "path",
                "points": self._planned_path,
                "color": "#00FF00",
                "width": 2,
            })

        # Add path follower drawings (target point, etc.)
        if self._state == NavigationState.FOLLOWING:
            drawings.extend(self._path_follower.get_drawings())

        # Draw rotation target indicator
        if self._state == NavigationState.POST_ROTATING and self._target_theta is not None:
            # We'd need robot position for this, skip for now
            pass

        return drawings

    def _obstacle_corners(
        self, x: float, y: float, theta_deg: float, width: float, depth: float
    ) -> List[Point]:
        """Compute rotated rectangle corners for an obstacle.

        Convention: depth = along heading, width = perpendicular to heading
        In local frame: X = depth (forward), Y = width (sideways)
        """
        theta = math.radians(theta_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # Half dimensions: depth along X (heading), width along Y (perpendicular)
        hx = depth / 2   # X half-size (along heading)
        hy = width / 2   # Y half-size (perpendicular)

        # Corner offsets in local frame
        corners_local = [
            (-hx, -hy),
            (hx, -hy),
            (hx, hy),
            (-hx, hy),
        ]

        # Rotate and translate to world frame
        corners = []
        for lx, ly in corners_local:
            wx = x + lx * cos_t - ly * sin_t
            wy = y + lx * sin_t + ly * cos_t
            corners.append((wx, wy))

        return corners
