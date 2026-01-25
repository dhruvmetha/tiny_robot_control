"""RVG-based path planner for obstacle avoidance.

Uses the RVG (Rotation-stacked Visibility Graph) library for SE(2)
path planning with obstacle avoidance.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

Point = Tuple[float, float]
ObstacleTuple = Tuple[float, float, float, float, float]  # (x, y, theta_deg, width, height)


class RVGPlanner:
    """Plans obstacle-avoiding paths using RVG library."""

    def __init__(
        self,
        workspace_width: float,
        workspace_height: float,
        robot_width: float,
        robot_height: float,
        robot_geometry_scale: float = 1.2,
    ) -> None:
        """
        Initialize the RVG planner.

        Args:
            workspace_width: Workspace width
            workspace_height: Workspace height
            robot_width: Robot width
            robot_height: Robot height
            robot_geometry_scale: Scale factor for robot geometry (default 1.2).
                                  Use >1.0 for more conservative paths that keep
                                  larger distance from obstacles.
        """
        self._width = workspace_width
        self._height = workspace_height
        self._robot_width = robot_width
        self._robot_height = robot_height
        self._robot_geometry_scale = robot_geometry_scale

        # Build robot geometry (scaled rectangle)
        w = (robot_width / 2) * robot_geometry_scale
        h = (robot_height / 2) * robot_geometry_scale
        self._robot_geometry = [(-w, -h), (w, -h), (w, h), (-w, h)]

    def plan(
        self,
        start: Point,
        goal: Point,
        obstacles: List[ObstacleTuple],
    ) -> List[Point]:
        """
        Plan a collision-free path from start to goal.

        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            obstacles: List of (x, y, theta_deg, width, height) obstacle tuples

        Returns:
            List of (x, y) waypoints from start to goal.
            Returns [start, goal] if planning fails or RVG unavailable.
        """
        try:
            from rvg import vertex, polygon, rvg
            import numpy as np
        except ImportError:
            print("[RVGPlanner] RVG not available, using direct path")
            return [start, goal]

        try:
            # Create workspace border polygon
            border_verts = [
                vertex(0, 0),
                vertex(self._width, 0),
                vertex(self._width, self._height),
                vertex(0, self._height),
            ]
            border = polygon(border_verts, False)

            # Create robot polygon
            robot_verts = [vertex(pt[0], pt[1]) for pt in self._robot_geometry]
            robot_poly = polygon(robot_verts, False)

            # Convert obstacles to polygons
            obstacle_polys = []
            for obs in obstacles:
                obs_poly = self._obstacle_to_polygon(obs)
                if obs_poly is not None:
                    obstacle_polys.append(obs_poly)

            # Create RVG solver
            solver = rvg(
                robot=robot_poly,
                border=border,
                obstacles=obstacle_polys,
                resolution=18,
                numThreads=1,
                verbose=False,
                fineApprox=True,
            )
            solver.setWeight(euclideanWeight=1.0, rotationalWeight=0.1)

            # Create start and goal vertices
            start_v = vertex(start[0], start[1], 0, 2 * np.pi, 0)
            goal_v = vertex(goal[0], goal[1], 0, 2 * np.pi, 0)

            # Check if start/goal are legal before calling shortestPath
            # Check ALL layers to be safe (RVG checks legality per-layer)
            layers = solver.getLayers()

            if not layers:
                print("[RVGPlanner] No layers available, using direct path")
                return [start, goal]

            for layer in layers:
                if not layer.legalConfig(start_v):
                    print(
                        f"[RVGPlanner] Start ({start[0]:.1f}, {start[1]:.1f}) "
                        "is not legal, using direct path"
                    )
                    return [start, goal]
                if not layer.legalConfig(goal_v):
                    print(
                        f"[RVGPlanner] Goal ({goal[0]:.1f}, {goal[1]:.1f}) "
                        "is not legal, using direct path"
                    )
                    return [start, goal]

            # Find shortest path (wrap in extra try for C++ exceptions)
            try:
                path_result = solver.shortestPath(start_v, goal_v)
            except (RuntimeError, SystemError, Exception) as e:
                print(f"[RVGPlanner] shortestPath failed: {e}, using direct path")
                return [start, goal]

            if path_result is None or len(path_result) == 0:
                return [start, goal]

            # Convert result to list of points
            path = [(v.getX(), v.getY()) for v in path_result]
            return path if path else [start, goal]

        except Exception as e:
            print(f"[RVGPlanner] Path planning error: {e}")
            return [start, goal]

    def _obstacle_to_polygon(self, obs: ObstacleTuple):
        """
        Convert obstacle tuple to RVG polygon.

        Args:
            obs: (x, y, theta_deg, width, height) obstacle tuple

        Returns:
            RVG polygon or None if invalid
        """
        try:
            from rvg import vertex, polygon
            import math
        except ImportError:
            return None

        x, y, theta_deg, width, height = obs
        theta = math.radians(theta_deg)

        # Half dimensions
        hw = width / 2
        hh = height / 2

        # Corner offsets in local frame
        corners_local = [
            (-hw, -hh),
            (hw, -hh),
            (hw, hh),
            (-hw, hh),
        ]

        # Rotate and translate to world frame
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        verts = []
        for lx, ly in corners_local:
            wx = x + lx * cos_t - ly * sin_t
            wy = y + lx * sin_t + ly * cos_t
            verts.append(vertex(wx, wy))

        if len(verts) >= 3:
            return polygon(verts)
        return None
