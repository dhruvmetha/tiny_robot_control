"""Wavefront planner for occupancy-based collision resolution.

Creates a grid-based occupancy map with inflated obstacles,
then uses BFS to find free positions and reachability.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from robot_control.utils.wavefront_inflation_config import DEFAULT_TIER1_BASE_INFLATION_MARGIN_M

import numpy as np


@dataclass
class WavefrontConfig:
    """Configuration for wavefront planner."""
    resolution: float = 0.005  # 5mm grid resolution
    robot_radius: float = 0.025  # 25mm robot radius
    inflation_margin: float = DEFAULT_TIER1_BASE_INFLATION_MARGIN_M  # tier-1 fallback, see wavefront_inflation_config
    obstacle_proximity_distance: float = 0.02  # 20mm - cost falloff distance from obstacles
    obstacle_proximity_weight: float = 5.0  # max extra cost multiplier at obstacle edge


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
    OBSTACLE = -1
    FREE = 0

    def __init__(self, config: Optional[WavefrontConfig] = None):
        self._config = config or WavefrontConfig()
        self._grid: Optional[np.ndarray] = None
        self._cost_grid: Optional[np.ndarray] = None  # Per-cell cost multiplier
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
        self._grid = np.zeros((self._height, self._width), dtype=np.int8)

        # Total inflation = robot radius + margin
        inflation = self._config.robot_radius + self._config.inflation_margin

        # Add each object to grid (with inflation)
        for name, (x, y, hw, hd, theta_deg) in objects.items():
            self._add_box_to_grid(x, y, hw, hd, theta_deg, inflation)

        # Add boundary walls - mark cells within inflation distance of edges as obstacle
        # This prevents robot from getting too close to workspace boundaries
        inflation_cells = int(math.ceil(inflation / res))
        if inflation_cells > 0:
            # Left edge
            self._grid[:, :inflation_cells] = self.OBSTACLE
            # Right edge
            self._grid[:, -inflation_cells:] = self.OBSTACLE
            # Bottom edge
            self._grid[:inflation_cells, :] = self.OBSTACLE
            # Top edge
            self._grid[-inflation_cells:, :] = self.OBSTACLE

        # Build proximity cost grid so paths prefer staying away from obstacles
        self._build_cost_grid()

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

    def _build_cost_grid(self) -> None:
        """Build per-cell cost multiplier based on proximity to obstacles.

        Free cells near obstacles get a higher traversal cost so Dijkstra
        naturally routes paths away from obstacle edges.  Cost decays
        linearly from (1 + weight) at the obstacle boundary to 1.0 at
        proximity_distance away.
        """
        prox_dist = self._config.obstacle_proximity_distance
        weight = self._config.obstacle_proximity_weight

        # If proximity cost is disabled, use uniform cost
        if prox_dist <= 0 or weight <= 0:
            self._cost_grid = np.ones(
                (self._height, self._width), dtype=np.float32
            )
            return

        res = self._config.resolution
        max_cells = int(math.ceil(prox_dist / res))

        # Start with uniform cost
        self._cost_grid = np.ones(
            (self._height, self._width), dtype=np.float32
        )

        # Find obstacle boundary cells (obstacle cells with at least one free neighbor)
        # using vectorized shift comparisons
        is_obs = self._grid == self.OBSTACLE
        is_free = self._grid == self.FREE

        # Check 4-connected neighbors for free cells
        has_free_neighbor = np.zeros_like(is_obs)
        has_free_neighbor[:, 1:] |= is_free[:, :-1]   # free to the left
        has_free_neighbor[:, :-1] |= is_free[:, 1:]    # free to the right
        has_free_neighbor[1:, :] |= is_free[:-1, :]    # free below
        has_free_neighbor[:-1, :] |= is_free[1:, :]    # free above

        boundary = is_obs & has_free_neighbor

        # Multi-source BFS from boundary cells into free cells
        dist_cells = np.full(
            (self._height, self._width), max_cells + 1, dtype=np.int32
        )
        queue: deque = deque()

        # Seed BFS with boundary cells
        seed_js, seed_is = np.where(boundary)
        for gj, gi in zip(seed_js, seed_is):
            dist_cells[gj, gi] = 0
            queue.append((int(gi), int(gj)))

        neighbors_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        # BFS outward into free cells
        while queue:
            gi, gj = queue.popleft()
            d = dist_cells[gj, gi]
            if d >= max_cells:
                continue
            for di, dj in neighbors_4:
                ni, nj = gi + di, gj + dj
                if 0 <= ni < self._width and 0 <= nj < self._height:
                    if self._grid[nj, ni] == self.FREE and d + 1 < dist_cells[nj, ni]:
                        dist_cells[nj, ni] = d + 1
                        queue.append((ni, nj))

        # Vectorized: convert distance to cost multiplier for free cells
        mask = (dist_cells <= max_cells) & is_free
        frac = 1.0 - dist_cells[mask].astype(np.float32) / (max_cells + 1)
        self._cost_grid[mask] = 1.0 + weight * frac

    def is_free(self, x: float, y: float) -> bool:
        """Check if position is free (not in obstacle)."""
        gi, gj = self._world_to_grid(x, y)
        if gi < 0 or gi >= self._width or gj < 0 or gj >= self._height:
            return False
        return self._grid[gj, gi] == self.FREE

    # Named constants for trapped-start recovery — keep in sync with the
    # C++ side. See `apply_trapped_start_recovery` docstring.
    _TRAPPED_DILATION_RADIUS_CELLS: int = 2  # → 5×5 block (matches C++ clear_radius=2)

    def apply_trapped_start_recovery(self, start: Tuple[float, float]) -> None:
        """Mutate the grid so the wavefront BFS/Dijkstra can escape a
        blocked start position — same policy as the C++ wavefront.

        Mirrors `WavefrontPlanner::recompute_wavefront` in
        `namo_cpp/src/wavefront/wavefront_planner.cpp:303-380`. If you
        change behavior here you MUST change it on the C++ side too;
        otherwise NAMOPlanner._is_goal_reachable (C++ wavefront) and
        WavefrontPathPlanner (Python wavefront) will disagree about
        reachability — exactly the 2026-05-19 incident.

        Why this exists:
          After a push, the robot frequently ends up inside its own
          inflated footprint. A naive "find nearest free cell" fallback
          can land in a disconnected pocket of free space (different
          connected component than the goal) — the path planner then
          gives up even though the goal is genuinely reachable. The C++
          policy avoids this by mutating the grid so the original start
          cell becomes part of the goal's connected component (as long
          as any free neighbor exists).

        Policy:
          1. If start cell is already FREE → no-op.
          2. Inspect 8-neighbors:
               - If at least one neighbor is FREE → clear_radius = 0;
                 only the start cell itself is forced to FREE. BFS will
                 then expand through whichever free neighbor leads to
                 the goal's component.
               - If all 8 neighbors are OBSTACLE → clear_radius = 2;
                 the full 5×5 block centered on start is forced to FREE.
                 This is the "robot fully buried" recovery path.
          3. After mutation, rebuild the proximity-cost grid so Dijkstra
             sees the new free cells correctly.

        Args:
            start: (x, y) start position in METERS (world coordinates).
        """
        if self._grid is None:
            return

        start_gi, start_gj = self._world_to_grid(start[0], start[1])

        # Out of bounds → nothing to do; planner will reject as invalid.
        if not (0 <= start_gi < self._width and 0 <= start_gj < self._height):
            return

        # Already free → no-op (matches C++: the dilation branch only
        # runs when the start cell is the seed of recompute_wavefront,
        # which is unconditional, but the *mutation* only matters when
        # the cell is occupied).
        if self._grid[start_gj, start_gi] == self.FREE:
            return

        # Are any 8-neighbors free?
        is_trapped = True
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ni, nj = start_gi + di, start_gj + dj
                if 0 <= ni < self._width and 0 <= nj < self._height:
                    if self._grid[nj, ni] != self.OBSTACLE:
                        is_trapped = False
                        break
            if not is_trapped:
                break

        clear_radius = self._TRAPPED_DILATION_RADIUS_CELLS if is_trapped else 0

        # Force-clear the (2*radius+1)×(2*radius+1) block to FREE.
        for dj in range(-clear_radius, clear_radius + 1):
            for di in range(-clear_radius, clear_radius + 1):
                ni, nj = start_gi + di, start_gj + dj
                if 0 <= ni < self._width and 0 <= nj < self._height:
                    self._grid[nj, ni] = self.FREE

        # Always force the start cell itself to FREE (the C++ side also
        # unconditionally enqueues the start; we encode that by ensuring
        # the cell is traversable).
        self._grid[start_gj, start_gi] = self.FREE

        # Proximity-cost grid is stale after grid mutation. Cheap to
        # rebuild (O(H*W) BFS) and keeps Dijkstra honest.
        self._build_cost_grid()

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
        # Use floor (not int truncation) so coords just below x_min/y_min map
        # to negative indices, not into cell 0. int(-0.1) == 0, floor(-0.1) == -1.
        # BUG-010.
        gi = math.floor((x - x_min) / res)
        gj = math.floor((y - y_min) / res)
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

    def save(
        self,
        filepath: str,
        robot_pos: Optional[Tuple[float, float]] = None,
        goal_pos: Optional[Tuple[float, float]] = None,
        path: Optional[List[Tuple[float, float]]] = None,
        show: bool = False,
    ) -> None:
        """Save the wavefront grid to file.

        Args:
            filepath: Output path (.png for image, .npy for numpy)
            robot_pos: Optional robot position to mark on grid (green circle)
            goal_pos: Optional goal position to mark on grid (red circle)
            path: Optional path to draw (blue line), list of (x, y) in meters
            show: If True, display the image in a window
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

            # Obstacle = black
            img[self._grid == self.OBSTACLE] = [0, 0, 0]

            # Free cells: shade by proximity cost (white = no cost, yellow = high cost)
            free_mask = self._grid == self.FREE
            if self._cost_grid is not None:
                # Normalize cost to 0-1 range for coloring
                max_cost = 1.0 + self._config.obstacle_proximity_weight
                cost_norm = np.clip(
                    (self._cost_grid - 1.0) / max(max_cost - 1.0, 1e-6), 0.0, 1.0
                )
                # White (no cost) → yellow (high cost)
                r = np.where(free_mask, 255, 0).astype(np.uint8)
                g = np.where(free_mask, 255, 0).astype(np.uint8)
                b = np.where(
                    free_mask,
                    (255 * (1.0 - cost_norm)).astype(np.uint8),
                    0,
                ).astype(np.uint8)
                img[:, :, 0] = b  # OpenCV BGR
                img[:, :, 1] = g
                img[:, :, 2] = r
                # Re-apply obstacle color
                img[self._grid == self.OBSTACLE] = [0, 0, 0]
            else:
                img[free_mask] = [255, 255, 255]

            # Flip Y for image coordinates (origin at bottom-left in world)
            img = cv2.flip(img, 0)

            # Draw path in blue (AFTER flip so position is correct)
            if path and len(path) >= 2:
                for i in range(len(path) - 1):
                    gi1, gj1 = self._world_to_grid(path[i][0], path[i][1])
                    gi2, gj2 = self._world_to_grid(path[i + 1][0], path[i + 1][1])
                    gj1_flipped = h - 1 - gj1
                    gj2_flipped = h - 1 - gj2
                    cv2.line(img, (gi1, gj1_flipped), (gi2, gj2_flipped), (255, 0, 0), 1)

            # Mark goal position as red dot (AFTER flip so position is correct)
            if goal_pos is not None:
                gi, gj = self._world_to_grid(goal_pos[0], goal_pos[1])
                gj_flipped = h - 1 - gj
                if 0 <= gi < w and 0 <= gj_flipped < h:
                    cv2.circle(img, (gi, gj_flipped), 2, (0, 0, 255), -1)

            # Mark robot position as green dot (AFTER flip so position is correct)
            if robot_pos is not None:
                gi, gj = self._world_to_grid(robot_pos[0], robot_pos[1])
                gj_flipped = h - 1 - gj
                if 0 <= gi < w and 0 <= gj_flipped < h:
                    cv2.circle(img, (gi, gj_flipped), 2, (0, 255, 0), -1)

            cv2.imwrite(filepath, img)
            print(f"[Wavefront] Saved grid image to {filepath} ({w}x{h})")

            if show:
                # Scale up for better visibility
                scale = 4
                img_large = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
                cv2.imshow("Wavefront Path", img_large)
                cv2.waitKey(1)  # Non-blocking

    def get_bounds(self) -> Optional[Tuple[float, float, float, float]]:
        """Get workspace bounds."""
        return self._bounds

    def get_resolution(self) -> float:
        """Get grid resolution."""
        return self._config.resolution

    # --- Path Planning Methods ---

    # 8-direction neighbors with costs (di, dj, cost)
    # Diagonal moves cost √2, cardinal moves cost 1
    _NEIGHBORS_WITH_COSTS: List[Tuple[int, int, float]] = [
        (-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414),
        (0, -1, 1.0),                   (0, 1, 1.0),
        (1, -1, 1.414),  (1, 0, 1.0),  (1, 1, 1.414),
    ]

    def _is_valid_cell(self, gi: int, gj: int) -> bool:
        """Check if grid cell is valid and free."""
        if gi < 0 or gi >= self._width or gj < 0 or gj >= self._height:
            return False
        return self._grid[gj, gi] == self.FREE

    def _dijkstra_from_goal(
        self, goal_gi: int, goal_gj: int
    ) -> Optional[np.ndarray]:
        """Build distance grid from goal using Dijkstra's algorithm.

        Args:
            goal_gi, goal_gj: Goal grid indices

        Returns:
            2D array of distances from each cell to goal, or None if goal is invalid.
            Unreachable cells have distance np.inf.
        """
        if not self._is_valid_cell(goal_gi, goal_gj):
            return None

        # Initialize distance grid with infinity
        dist = np.full((self._height, self._width), np.inf, dtype=np.float64)
        dist[goal_gj, goal_gi] = 0.0

        # Priority queue: (distance, gi, gj)
        heap = [(0.0, goal_gi, goal_gj)]

        while heap:
            d, gi, gj = heapq.heappop(heap)

            # Skip if we've found a better path
            if d > dist[gj, gi]:
                continue

            # Expand neighbors
            for di, dj, cost in self._NEIGHBORS_WITH_COSTS:
                ni, nj = gi + di, gj + dj
                if self._is_valid_cell(ni, nj):
                    # Scale move cost by proximity penalty at destination cell
                    cell_cost = cost * self._cost_grid[nj, ni]
                    new_dist = d + cell_cost
                    if new_dist < dist[nj, ni]:
                        dist[nj, ni] = new_dist
                        heapq.heappush(heap, (new_dist, ni, nj))

        return dist

    def _gradient_descent(
        self,
        dist: np.ndarray,
        start_gi: int,
        start_gj: int,
        goal_gi: int,
        goal_gj: int,
    ) -> List[Tuple[int, int]]:
        """Follow gradient from start to goal.

        Args:
            dist: Distance grid from Dijkstra
            start_gi, start_gj: Start grid indices
            goal_gi, goal_gj: Goal grid indices

        Returns:
            List of (gi, gj) grid cells from start to goal (inclusive).
            Empty list if no path exists.
        """
        # Check if start is reachable
        if np.isinf(dist[start_gj, start_gi]):
            return []

        path = [(start_gi, start_gj)]
        gi, gj = start_gi, start_gj
        max_steps = self._width * self._height  # Prevent infinite loops

        for _ in range(max_steps):
            if gi == goal_gi and gj == goal_gj:
                break

            # Find neighbor with smallest distance
            best_ni, best_nj = gi, gj
            best_dist = dist[gj, gi]

            for di, dj, _ in self._NEIGHBORS_WITH_COSTS:
                ni, nj = gi + di, gj + dj
                if 0 <= ni < self._width and 0 <= nj < self._height:
                    if dist[nj, ni] < best_dist:
                        best_dist = dist[nj, ni]
                        best_ni, best_nj = ni, nj

            # No progress - stuck
            if best_ni == gi and best_nj == gj:
                return []

            gi, gj = best_ni, best_nj
            path.append((gi, gj))

        return path

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        """Plan a path from start to goal using Dijkstra + gradient descent.

        Args:
            start: (x, y) start position in meters
            goal: (x, y) goal position in meters

        Returns:
            List of (x, y) waypoints from start to goal in meters.
            Returns empty list if no path found.
        """
        if self._grid is None:
            return []

        # Unconditionally run trapped-start recovery so this entry point
        # matches the C++ `recompute_wavefront` semantics (single entry,
        # recovery baked in). If the caller already invoked recovery,
        # this is a no-op because the start cell is FREE. See
        # `apply_trapped_start_recovery` docstring + the cross-referenced
        # namo_cpp source.
        self.apply_trapped_start_recovery(start)

        # Convert world to grid coordinates
        start_gi, start_gj = self._world_to_grid(start[0], start[1])
        goal_gi, goal_gj = self._world_to_grid(goal[0], goal[1])

        # Validate start and goal
        if not self._is_valid_cell(start_gi, start_gj):
            return []
        if not self._is_valid_cell(goal_gi, goal_gj):
            return []

        # Build distance grid from goal
        dist = self._dijkstra_from_goal(goal_gi, goal_gj)
        if dist is None:
            return []

        # Follow gradient from start to goal
        grid_path = self._gradient_descent(dist, start_gi, start_gj, goal_gi, goal_gj)
        if not grid_path:
            return []

        # Convert grid path to world coordinates (cell centers)
        world_path = [self._grid_to_world(gi, gj) for gi, gj in grid_path]
        return world_path
