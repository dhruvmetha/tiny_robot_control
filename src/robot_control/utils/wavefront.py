"""Wavefront planner for occupancy-based collision resolution.

Creates a grid-based occupancy map with inflated obstacles,
then uses BFS to find free positions and reachability.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class WavefrontConfig:
    """Configuration for wavefront planner."""
    resolution: float = 0.005  # 5mm grid resolution
    robot_radius: float = 0.025  # 25mm robot radius
    inflation_margin: float = 0.005  # 5mm extra margin


class WavefrontPlanner:
    """BFS-based wavefront planner with obstacle inflation.

    Usage:
        planner = WavefrontPlanner(config)
        planner.build_grid(bounds, objects)

        # Check if position is free
        if not planner.is_free(x, y):
            # Find nearest free position
            free_x, free_y = planner.find_nearest_free(x, y)
    """

    # Grid cell values
    FREE = 0
    OBSTACLE = 1
    VISITED = 2

    def __init__(self, config: Optional[WavefrontConfig] = None):
        self._config = config or WavefrontConfig()
        self._grid: Optional[np.ndarray] = None
        self._bounds: Optional[Tuple[float, float, float, float]] = None
        self._width: int = 0
        self._height: int = 0

    def build_grid(
        self,
        bounds: Tuple[float, float, float, float],
        objects: Dict[str, Tuple[float, float, float, float, float]],
    ) -> None:
        """Build occupancy grid from objects.

        Args:
            bounds: (x_min, x_max, y_min, y_max) in meters
            objects: Dict mapping name to (x, y, half_width, half_depth, theta_deg)
        """
        self._bounds = bounds
        x_min, x_max, y_min, y_max = bounds
        res = self._config.resolution

        self._width = int(math.ceil((x_max - x_min) / res))
        self._height = int(math.ceil((y_max - y_min) / res))

        # Initialize grid as free
        self._grid = np.zeros((self._height, self._width), dtype=np.uint8)

        # Total inflation = robot radius + margin
        inflation = self._config.robot_radius + self._config.inflation_margin

        # Add each object to grid (with inflation)
        for name, (x, y, hw, hd, theta_deg) in objects.items():
            self._add_box_to_grid(x, y, hw, hd, theta_deg, inflation)

        # Add boundary walls (outside bounds is obstacle)
        # Already handled by grid being only within bounds

    def _add_box_to_grid(
        self,
        cx: float,
        cy: float,
        half_width: float,
        half_depth: float,
        theta_deg: float,
        inflation: float,
    ) -> None:
        """Add an inflated rotated box to the grid."""
        theta = math.radians(theta_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        # Inflated half dimensions
        hw = half_width + inflation
        hd = half_depth + inflation

        # Get bounding box in world coords
        corners = [
            (-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)
        ]
        world_corners = []
        for lx, ly in corners:
            wx = cx + lx * cos_t - ly * sin_t
            wy = cy + lx * sin_t + ly * cos_t
            world_corners.append((wx, wy))

        # Get axis-aligned bounding box
        min_x = min(c[0] for c in world_corners)
        max_x = max(c[0] for c in world_corners)
        min_y = min(c[1] for c in world_corners)
        max_y = max(c[1] for c in world_corners)

        # Convert to grid coords
        res = self._config.resolution
        x_min, _, y_min, _ = self._bounds

        gi_min = max(0, int((min_x - x_min) / res) - 1)
        gi_max = min(self._width - 1, int((max_x - x_min) / res) + 1)
        gj_min = max(0, int((min_y - y_min) / res) - 1)
        gj_max = min(self._height - 1, int((max_y - y_min) / res) + 1)

        # Check each cell in bounding box
        for gj in range(gj_min, gj_max + 1):
            for gi in range(gi_min, gi_max + 1):
                # Cell center in world coords
                wx = x_min + (gi + 0.5) * res
                wy = y_min + (gj + 0.5) * res

                # Transform to box-local coords
                dx, dy = wx - cx, wy - cy
                lx = dx * cos_t + dy * sin_t
                ly = -dx * sin_t + dy * cos_t

                # Check if inside inflated box
                if abs(lx) <= hw and abs(ly) <= hd:
                    self._grid[gj, gi] = self.OBSTACLE

    def is_free(self, x: float, y: float) -> bool:
        """Check if position is free (not in obstacle)."""
        gi, gj = self._world_to_grid(x, y)
        if gi < 0 or gi >= self._width or gj < 0 or gj >= self._height:
            return False
        return self._grid[gj, gi] == self.FREE

    def find_nearest_free(
        self,
        x: float,
        y: float,
        max_search_radius: float = 0.5,
    ) -> Optional[Tuple[float, float]]:
        """Find nearest free position using BFS.

        Args:
            x, y: Starting position in meters
            max_search_radius: Maximum search distance in meters

        Returns:
            (x, y) of nearest free position, or None if not found
        """
        if self._grid is None:
            return None

        start_gi, start_gj = self._world_to_grid(x, y)

        # If already free, return original position
        if self.is_free(x, y):
            return (x, y)

        # BFS to find nearest free cell
        max_cells = int(max_search_radius / self._config.resolution)
        visited = set()
        queue = deque([(start_gi, start_gj, 0)])
        visited.add((start_gi, start_gj))

        # 8-connected neighbors
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1),  (1, 0), (1, 1),
        ]

        while queue:
            gi, gj, dist = queue.popleft()

            if dist > max_cells:
                continue

            # Check if this cell is free
            if 0 <= gi < self._width and 0 <= gj < self._height:
                if self._grid[gj, gi] == self.FREE:
                    return self._grid_to_world(gi, gj)

            # Add neighbors
            for di, dj in neighbors:
                ni, nj = gi + di, gj + dj
                if (ni, nj) not in visited:
                    visited.add((ni, nj))
                    queue.append((ni, nj, dist + 1))

        return None

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coords to grid indices."""
        x_min, _, y_min, _ = self._bounds
        res = self._config.resolution
        gi = int((x - x_min) / res)
        gj = int((y - y_min) / res)
        return gi, gj

    def _grid_to_world(self, gi: int, gj: int) -> Tuple[float, float]:
        """Convert grid indices to world coords (cell center)."""
        x_min, _, y_min, _ = self._bounds
        res = self._config.resolution
        x = x_min + (gi + 0.5) * res
        y = y_min + (gj + 0.5) * res
        return x, y

    def get_grid(self) -> Optional[np.ndarray]:
        """Get the occupancy grid (for visualization)."""
        return self._grid

    def get_random_reachable_cell(
        self,
        from_pos: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        """Get a random cell reachable from the given position.

        Uses BFS to find all cells connected to from_pos, then picks randomly.

        Args:
            from_pos: (x, y) position to start reachability search from

        Returns:
            (x, y) of a random reachable cell, or None if none found
        """
        import random

        if self._grid is None:
            return None

        start_gi, start_gj = self._world_to_grid(from_pos[0], from_pos[1])

        # Check if start position is valid and free
        if not (0 <= start_gi < self._width and 0 <= start_gj < self._height):
            return None
        if self._grid[start_gj, start_gi] != self.FREE:
            return None

        # BFS to find all reachable cells
        reachable = []
        visited = set()
        queue = deque([(start_gi, start_gj)])
        visited.add((start_gi, start_gj))

        # 8-connected neighbors
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1),  (1, 0), (1, 1),
        ]

        while queue:
            gi, gj = queue.popleft()
            reachable.append((gi, gj))

            for di, dj in neighbors:
                ni, nj = gi + di, gj + dj
                if (ni, nj) not in visited:
                    if 0 <= ni < self._width and 0 <= nj < self._height:
                        if self._grid[nj, ni] == self.FREE:
                            visited.add((ni, nj))
                            queue.append((ni, nj))

        if not reachable:
            return None

        # Pick a random reachable cell
        gi, gj = random.choice(reachable)
        return self._grid_to_world(gi, gj)

    def save(self, filepath: str, robot_pos: Optional[Tuple[float, float]] = None) -> None:
        """Save the wavefront grid to file.

        Args:
            filepath: Output path (.png for image, .npy for numpy)
            robot_pos: Optional robot position to mark on grid
        """
        if self._grid is None:
            print("[Wavefront] No grid to save")
            return

        if filepath.endswith(".npy"):
            np.save(filepath, self._grid)
            print(f"[Wavefront] Saved grid to {filepath}")
        else:
            # Save as image
            import cv2

            # Create RGB image
            h, w = self._grid.shape
            img = np.zeros((h, w, 3), dtype=np.uint8)

            # Free = white, Obstacle = black
            img[self._grid == self.FREE] = [255, 255, 255]
            img[self._grid == self.OBSTACLE] = [0, 0, 0]

            # Flip Y for image coordinates (origin at bottom-left in world)
            img = cv2.flip(img, 0)

            # Mark robot position in green (AFTER flip so position is correct)
            if robot_pos is not None:
                gi, gj = self._world_to_grid(robot_pos[0], robot_pos[1])
                # After flip, Y coordinate is inverted
                gj_flipped = h - 1 - gj
                if 0 <= gi < w and 0 <= gj_flipped < h:
                    # Draw a circle for robot
                    cv2.circle(img, (gi, gj_flipped), 3, (0, 255, 0), -1)

            cv2.imwrite(filepath, img)
            print(f"[Wavefront] Saved grid image to {filepath} ({w}x{h})")

    def get_bounds(self) -> Optional[Tuple[float, float, float, float]]:
        """Get workspace bounds."""
        return self._bounds

    def get_resolution(self) -> float:
        """Get grid resolution."""
        return self._config.resolution
