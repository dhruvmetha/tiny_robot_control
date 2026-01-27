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
        robot_geometry_scale: float = 1.27,  # Matches wavefront: 5.5/2 * 1.27 ≈ 3.5 cm
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
            Returns empty list if planning fails.
        """
        try:
            from rvg import vertex, polygon, rvg
            import numpy as np
        except ImportError:
            print("[RVGPlanner] RVG not available")
            return []

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
            import sys
            try:
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
            except Exception as e:
                print(f"[RVGPlanner] Solver creation failed: {e}")
                return []

            # Pre-check: ensure start/goal with robot footprint fits in workspace
            max_robot_extent = max(abs(p[0]) for p in self._robot_geometry)
            max_robot_extent = max(max_robot_extent, max(abs(p[1]) for p in self._robot_geometry))
            margin = max_robot_extent + 0.1  # small extra margin

            if (start[0] < margin or start[0] > self._width - margin or
                start[1] < margin or start[1] > self._height - margin):
                print(f"[RVGPlanner] Start ({start[0]:.1f}, {start[1]:.1f}) too close to boundary")
                return []
            if (goal[0] < margin or goal[0] > self._width - margin or
                goal[1] < margin or goal[1] > self._height - margin):
                print(f"[RVGPlanner] Goal ({goal[0]:.1f}, {goal[1]:.1f}) too close to boundary")
                return []

            # Create start and goal vertices
            start_v = vertex(start[0], start[1], 0, 2 * np.pi, 0)
            goal_v = vertex(goal[0], goal[1], 0, 2 * np.pi, 0)

            # Check if start/goal are legal in ALL layers before calling shortestPath
            # This prevents crashes from shortestPath checking legality across layers in parallel
            layers = solver.getLayers()
            if not layers:
                print("[RVGPlanner] No layers available")
                return []

            for layer in layers:
                try:
                    if not layer.legalConfig(start_v):
                        theta_lb = layer.getThetaLb()
                        theta_ub = layer.getThetaUb()
                        print(f"[RVGPlanner] Start ({start[0]:.1f}, {start[1]:.1f}) not legal in layer θ=[{theta_lb:.2f}, {theta_ub:.2f}]")
                        return []
                    if not layer.legalConfig(goal_v):
                        theta_lb = layer.getThetaLb()
                        theta_ub = layer.getThetaUb()
                        print(f"[RVGPlanner] Goal ({goal[0]:.1f}, {goal[1]:.1f}) not legal in layer θ=[{theta_lb:.2f}, {theta_ub:.2f}]")
                        return []
                except Exception as e:
                    print(f"[RVGPlanner] Layer check failed: {e}")
                    return []

            # Find shortest path (wrap in extra try for C++ exceptions)
            try:
                path_result = solver.shortestPath(start_v, goal_v)
            except (RuntimeError, SystemError, Exception) as e:
                print(f"[RVGPlanner] shortestPath failed: {e}")
                return []

            if path_result is None or len(path_result) == 0:
                print("[RVGPlanner] No path found")
                return []

            # Convert result to list of points
            path = [(v.getX(), v.getY()) for v in path_result]
            return path if path else []

        except Exception as e:
            print(f"[RVGPlanner] Path planning error: {e}")
            return []

    def _obstacle_to_polygon(self, obs: ObstacleTuple):
        """
        Convert obstacle tuple to RVG polygon.

        Args:
            obs: (x, y, theta_deg, width, depth) obstacle tuple
                 width = X dimension, depth = Y dimension

        Returns:
            RVG polygon or None if invalid
        """
        try:
            from rvg import vertex, polygon
            import math
        except ImportError:
            return None

        x, y, theta_deg, width, depth = obs
        theta = math.radians(theta_deg)

        # Convention: depth = along heading, width = perpendicular to heading
        # In local frame: X = depth (forward), Y = width (sideways)
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
