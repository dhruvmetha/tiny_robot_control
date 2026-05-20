"""Wavefront-based path planner for obstacle avoidance.

Uses grid-based Dijkstra + gradient descent for path planning.
Drop-in replacement for RVGPlanner with the same interface.
"""

from __future__ import annotations

import os
import time
from typing import List, Optional, Tuple

from robot_control.utils.wavefront_inflation_config import get_wavefront_inflation_config
from robot_control.utils.wavefront import WavefrontPlanner, WavefrontConfig
from robot_control.utils.robot_geometry import effective_robot_radius_m_from_cm

Point = Tuple[float, float]
ObstacleTuple = Tuple[float, float, float, float, float]  # (x, y, theta_deg, width, depth)


class WavefrontPathPlanner:
    """Plans obstacle-avoiding paths using wavefront (Dijkstra + gradient descent)."""

    def __init__(
        self,
        workspace_width: float,
        workspace_height: float,
        robot_width: float,
        robot_height: float,
        resolution_cm: float = 0.5,
        debug_dir: Optional[str] = None,
        obstacle_proximity_distance_cm: float = 2.0,
        obstacle_proximity_weight: float = 5.0,
    ) -> None:
        """
        Initialize the wavefront path planner.

        Args:
            workspace_width: Workspace width in cm
            workspace_height: Workspace height in cm
            robot_width: Robot width in cm
            robot_height: Robot height in cm
            resolution_cm: Grid resolution in cm (default 0.5 = 5mm)
            debug_dir: If set, save wavefront images to this directory
            obstacle_proximity_distance_cm: How far (cm) the extra cost extends from obstacles
            obstacle_proximity_weight: Max cost multiplier at obstacle edge (0 to disable)
        """
        self._width_cm = workspace_width
        self._height_cm = workspace_height
        self._robot_width_cm = robot_width
        self._robot_height_cm = robot_height
        self._resolution_cm = resolution_cm
        self._debug_dir = debug_dir
        self._plan_count = 0

        # Convert to meters for WavefrontPlanner
        self._width_m = workspace_width / 100.0
        self._height_m = workspace_height / 100.0
        self._resolution_m = resolution_cm / 100.0

        # Inflation radius in meters from runtime robot footprint.
        # See robot_geometry.effective_robot_radius_m_from_cm for the formula.
        # The C++ planner uses the same formula in
        # `namo_cpp/include/wavefront/goal_tolerance_utils.hpp`
        # (`compute_rotation_safe_robot_radius_m`) so runtime and planner
        # wavefronts agree on which corridors are passable.
        self._robot_radius_m = effective_robot_radius_m_from_cm(robot_width, robot_height)

        # Obstacle proximity cost parameters (convert to meters)
        self._obstacle_proximity_distance_m = obstacle_proximity_distance_cm / 100.0
        self._obstacle_proximity_weight = obstacle_proximity_weight

        # Keep reference to last planner for external access
        self._last_planner: Optional[WavefrontPlanner] = None
        self._last_path_m: List[Tuple[float, float]] = []

    def plan(
        self,
        start: Point,
        goal: Point,
        obstacles: List[ObstacleTuple],
    ) -> List[Point]:
        """
        Plan a collision-free path from start to goal.

        Args:
            start: (x, y) start position in cm
            goal: (x, y) goal position in cm
            obstacles: List of (x, y, theta_deg, width, depth) obstacle tuples in cm
                       width = perpendicular to heading, depth = along heading

        Returns:
            List of (x, y) waypoints from start to goal in cm.
            Returns empty list if planning fails.
        """
        # Convert obstacles to wavefront format
        # Input: (x, y, theta_deg, width, depth) in cm
        # Output: {name: (x_m, y_m, half_depth_m, half_width_m, theta_deg)}
        # Note: WavefrontPlanner uses (half_width, half_depth) = (X half-size, Y half-size)
        # where X is along heading (depth) and Y is perpendicular (width)
        wavefront_obstacles = self._convert_obstacles(obstacles)

        # Create wavefront config with standardized tier-1 inflation.
        inflation_cfg = get_wavefront_inflation_config()
        inflation_margin_m = (
            inflation_cfg.tier1_base_inflation_margin_m
            + inflation_cfg.navigation_additional_margin_m
        )
        config = WavefrontConfig(
            resolution=self._resolution_m,
            robot_radius=self._robot_radius_m,
            inflation_margin=inflation_margin_m,
            obstacle_proximity_distance=self._obstacle_proximity_distance_m,
            obstacle_proximity_weight=self._obstacle_proximity_weight,
        )

        # Create planner and build grid
        planner = WavefrontPlanner(config)
        bounds = (0.0, self._width_m, 0.0, self._height_m)
        planner.build_grid(bounds, wavefront_obstacles)

        # Convert start/goal to meters
        start_m = (start[0] / 100.0, start[1] / 100.0)
        goal_m = (goal[0] / 100.0, goal[1] / 100.0)

        # Trapped-start recovery — matches the C++ wavefront policy so
        # NAMOPlanner._is_goal_reachable (C++) and this planner (Python)
        # agree on whether a path exists. See
        # WavefrontPlanner.apply_trapped_start_recovery for the spec and
        # cross-reference to namo_cpp.
        was_blocked = not planner.is_free(start_m[0], start_m[1])
        if was_blocked:
            planner.apply_trapped_start_recovery(start_m)
            now_free = planner.is_free(start_m[0], start_m[1])
            print(
                f"[WavefrontPathPlanner] Start blocked at "
                f"({start_m[0]*100:.1f}, {start_m[1]*100:.1f}) cm — "
                f"applied trapped-start recovery (now {'FREE' if now_free else 'BLOCKED'})"
            )

        # Plan path from the original start. Recovery mutated the grid
        # so the start cell is now valid by construction.
        path_m = planner.plan(start_m, goal_m)

        # Store for external access
        self._last_planner = planner
        self._last_path_m = path_m
        self._plan_count += 1

        if self._debug_dir:
            self._save_debug_image(planner, start_m, goal_m, path_m)

        if not path_m:
            start_free = planner.is_free(start_m[0], start_m[1])
            goal_free = planner.is_free(goal_m[0], goal_m[1])
            print(f"[WavefrontPathPlanner] Path planning FAILED")
            print(
                f"  Start: ({start[0]:.1f}, {start[1]:.1f}) cm -> "
                f"{'FREE' if start_free else 'BLOCKED (recovery insufficient)'}"
            )
            print(f"  Goal:  ({goal[0]:.1f}, {goal[1]:.1f}) cm -> {'FREE' if goal_free else 'BLOCKED'}")
            print(f"  Obstacles: {len(obstacles)}")
            if not start_free:
                print(
                    f"  [!] Start cell could not be recovered — robot may be "
                    f"completely buried or off-grid"
                )
            elif not goal_free:
                print(f"  [!] Goal position is BLOCKED - target unreachable")
            else:
                print(
                    f"  [!] Start and goal are in disconnected components — "
                    f"obstacles partition the workspace"
                )
            return []

        # Convert path back to cm
        path_cm = [(x * 100.0, y * 100.0) for x, y in path_m]
        return path_cm

    def _save_debug_image(
        self,
        planner: WavefrontPlanner,
        start_m: Point,
        goal_m: Point,
        path_m: List[Point],
    ) -> None:
        """Save debug image of the wavefront grid with path."""
        os.makedirs(self._debug_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"wavefront_{timestamp}_{self._plan_count:03d}.png"
        filepath = os.path.join(self._debug_dir, filename)

        planner.save(
            filepath,
            robot_pos=start_m,
            goal_pos=goal_m,
            path=path_m if path_m else None,
            show=False,
        )

    def save_last_plan(self, filepath: str, show: bool = False) -> None:
        """Save the last planned path to an image file.

        Args:
            filepath: Output image path (.png)
            show: If True, also display in a window
        """
        if self._last_planner is None:
            print("[WavefrontPathPlanner] No plan to save")
            return

        # Get start/goal from path endpoints
        if self._last_path_m and len(self._last_path_m) >= 2:
            start_m = self._last_path_m[0]
            goal_m = self._last_path_m[-1]
        else:
            start_m = None
            goal_m = None

        self._last_planner.save(
            filepath,
            robot_pos=start_m,
            goal_pos=goal_m,
            path=self._last_path_m if self._last_path_m else None,
            show=show,
        )

    def _convert_obstacles(
        self, obstacles: List[ObstacleTuple]
    ) -> dict:
        """Convert obstacle list to wavefront format.

        Args:
            obstacles: List of (x, y, theta_deg, width, depth) in cm
                       width = perpendicular to heading (Y in local frame)
                       depth = along heading (X in local frame)

        Returns:
            Dict mapping name to (x_m, y_m, half_width_m, half_depth_m, theta_deg)
            where half_width = X half-size (depth/2), half_depth = Y half-size (width/2)
        """
        result = {}
        for i, (x, y, theta_deg, width, depth) in enumerate(obstacles):
            # Convert cm to meters
            x_m = x / 100.0
            y_m = y / 100.0
            # WavefrontPlanner expects (half_width, half_depth) where:
            # - half_width = X half-size = depth/2 (along heading)
            # - half_depth = Y half-size = width/2 (perpendicular to heading)
            half_width_m = depth / 200.0  # depth/2 in meters
            half_depth_m = width / 200.0  # width/2 in meters
            result[f"obs_{i}"] = (x_m, y_m, half_width_m, half_depth_m, theta_deg)
        return result
