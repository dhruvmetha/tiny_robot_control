"""Regression + unit tests for trapped-start handling in WavefrontPathPlanner.

Context (2026-05-19 incident):
  - NAMOPlanner reported "Goal REACHABLE" (C++ wavefront), but
    WavefrontPathPlanner returned 0 raw points.
  - Root cause: the Python `WavefrontPlanner` (utils/wavefront.py) used a
    one-shot Euclidean `find_nearest_free` fallback that could pick a free
    cell in a disconnected pocket of free space — different connected
    component than the goal. The C++ side instead:
      (a) checks whether the robot has any free 8-neighbor (not trapped),
      (b) if fully trapped, dilates a 5×5 block around the robot to FREE,
      (c) force-enqueues the (possibly-blocked) start cell into the BFS
          so the search escapes through any free neighbor.
  - See namo_cpp/src/wavefront/wavefront_planner.cpp:303-380 for the
    authoritative behavior.

These tests assert the Python side now matches that policy.
"""
from __future__ import annotations

from typing import List, Tuple

import numpy as np
import pytest

from robot_control.utils.wavefront import (
    WavefrontConfig,
    WavefrontPlanner,
)


# ─── Named constants ────────────────────────────────────────────────────
# Tiny workspace, coarse cells. Picked so a single test grid fits in ~50
# cells per axis and we can hand-draw obstacle layouts by index.

GRID_RES_M = 0.01          # 1 cm cells — easy to reason about
WORKSPACE_M = 0.50         # 50×50 cm workspace
ROBOT_RADIUS_M = 0.02      # 2 cm (≈ 4 cm wide robot)
INFLATION_MARGIN_M = 0.005

# Disable proximity-cost shaping in tests; we want to verify connectivity
# semantics, not path aesthetics.
PROXIMITY_DIST_M = 0.0
PROXIMITY_WEIGHT = 0.0


def _make_planner() -> WavefrontPlanner:
    cfg = WavefrontConfig(
        resolution=GRID_RES_M,
        robot_radius=ROBOT_RADIUS_M,
        inflation_margin=INFLATION_MARGIN_M,
        obstacle_proximity_distance=PROXIMITY_DIST_M,
        obstacle_proximity_weight=PROXIMITY_WEIGHT,
    )
    return WavefrontPlanner(cfg)


# ─── Reproducer: the exact pattern from the field log ───────────────────


def test_blocked_start_with_free_neighbor_connecting_to_goal_finds_path():
    """The 2026-05-19 case: robot lands ON an inflated obstacle cell, but
    has at least one free 8-neighbor that connects to the goal region.

    C++ behavior: force-enqueue start, BFS escapes via free neighbor →
    Goal REACHABLE + path exists.

    Required Python behavior (after this PR): match the C++ — return a
    non-empty path.

    On main today: returns []. This test pins the bug.
    """
    planner = _make_planner()
    bounds = (0.0, WORKSPACE_M, 0.0, WORKSPACE_M)

    # One obstacle in the middle. After inflation (~2.5 cm radius), the
    # obstacle envelope swells but leaves a wide free corridor on both
    # sides — single connected component.
    objects = {
        "obs": (0.25, 0.25, 0.04, 0.04, 0.0),  # (x, y, hw, hd, theta_deg)
    }
    planner.build_grid(bounds, objects)

    # Place the start on the inflated boundary. The exact cell is OBSTACLE
    # in the grid but at least one 8-neighbor is FREE and connects to the
    # main free corridor that contains the goal.
    grid = planner.get_grid()
    assert grid is not None

    start_m = _find_blocked_cell_with_free_neighbor(planner, near_xy=(0.21, 0.25))
    assert start_m is not None, "test setup: couldn't find a suitable blocked cell"
    assert not planner.is_free(*start_m), "test setup: chosen start should be blocked"

    goal_m = (0.45, 0.45)
    assert planner.is_free(*goal_m), "test setup: goal should be free"

    path = planner.plan(start_m, goal_m)

    assert path, (
        f"Expected a non-empty path. Start {start_m} is blocked but has a "
        f"free 8-neighbor connected to the goal's component — C++ wavefront "
        f"would return REACHABLE. Python must agree."
    )
    # Sanity: path ends near the goal cell center
    assert abs(path[-1][0] - goal_m[0]) < GRID_RES_M
    assert abs(path[-1][1] - goal_m[1]) < GRID_RES_M


# ─── Unit tests for apply_trapped_start_recovery ────────────────────────


def test_recovery_noop_when_start_already_free():
    """If the start cell is already FREE, recovery must not mutate the grid.

    This protects against accidentally widening obstacles around a perfectly
    valid start position.
    """
    planner = _make_planner()
    bounds = (0.0, WORKSPACE_M, 0.0, WORKSPACE_M)
    planner.build_grid(bounds, {"obs": (0.25, 0.25, 0.04, 0.04, 0.0)})

    before = planner.get_grid().copy()
    start_m = (0.05, 0.05)
    assert planner.is_free(*start_m)

    planner.apply_trapped_start_recovery(start_m)

    assert np.array_equal(planner.get_grid(), before), (
        "Recovery must be a no-op when start is already free."
    )


def test_recovery_clears_only_start_cell_when_neighbor_is_free():
    """C++ rule: if any 8-neighbor of the blocked start is free,
    clear_radius = 0 — only the start cell itself is forced to FREE.

    This is the common case (robot wedged on inflation boundary).
    """
    planner = _make_planner()
    bounds = (0.0, WORKSPACE_M, 0.0, WORKSPACE_M)
    planner.build_grid(bounds, {"obs": (0.25, 0.25, 0.04, 0.04, 0.0)})

    start_m = _find_blocked_cell_with_free_neighbor(planner, near_xy=(0.21, 0.25))
    assert start_m is not None

    before = planner.get_grid().copy()
    start_gi, start_gj = planner._world_to_grid(*start_m)
    assert before[start_gj, start_gi] == WavefrontPlanner.OBSTACLE

    planner.apply_trapped_start_recovery(start_m)
    after = planner.get_grid()

    # Start cell now FREE
    assert after[start_gj, start_gi] == WavefrontPlanner.FREE

    # All other cells unchanged (radius=0 means no dilation)
    diff_mask = (before != after)
    n_changed = int(diff_mask.sum())
    assert n_changed == 1, (
        f"Expected exactly 1 cell mutated (the start), got {n_changed}. "
        f"With a free 8-neighbor, clear_radius must be 0 — no 5×5 dilation."
    )


def test_recovery_dilates_5x5_when_fully_boxed_in():
    """C++ rule: if ALL 8-neighbors are obstacle, clear_radius = 2 →
    a 5×5 block around the start is force-cleared to FREE.

    This is the rare "robot fully buried" case after a bad push.
    """
    planner = _make_planner()
    bounds = (0.0, WORKSPACE_M, 0.0, WORKSPACE_M)

    # Construct a scene where a chosen interior cell is surrounded by
    # obstacle in all 8 directions. Simplest: a big obstacle covering
    # most of the workspace.
    planner.build_grid(bounds, {"big": (0.25, 0.25, 0.20, 0.20, 0.0)})

    # Pick a cell well inside the obstacle.
    start_m = (0.25, 0.25)
    start_gi, start_gj = planner._world_to_grid(*start_m)
    grid = planner.get_grid()
    assert grid[start_gj, start_gi] == WavefrontPlanner.OBSTACLE
    # All 8 neighbors should also be obstacle (we're deep in the blob).
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            if di == 0 and dj == 0:
                continue
            assert grid[start_gj + dj, start_gi + di] == WavefrontPlanner.OBSTACLE

    planner.apply_trapped_start_recovery(start_m)
    after = planner.get_grid()

    # 5×5 block centered on start should now be FREE
    for dj in range(-2, 3):
        for di in range(-2, 3):
            assert after[start_gj + dj, start_gi + di] == WavefrontPlanner.FREE, (
                f"Cell at offset ({di},{dj}) should be FREE after 5×5 dilation"
            )

    # Cell just outside the 5×5 (offset 3) should still be obstacle
    assert after[start_gj, start_gi + 3] == WavefrontPlanner.OBSTACLE


def test_recovery_respects_grid_bounds():
    """Don't read or write outside the grid when start is near a corner."""
    planner = _make_planner()
    bounds = (0.0, WORKSPACE_M, 0.0, WORKSPACE_M)
    # Wall along the bottom + left edges via inflation.
    planner.build_grid(bounds, {})  # no obstacles, only boundary inflation

    # Inflated boundary makes corner cells OBSTACLE.
    start_m = (0.0, 0.0)
    # Should not raise IndexError even though some neighbors would be
    # off-grid.
    planner.apply_trapped_start_recovery(start_m)


# ─── Helpers ────────────────────────────────────────────────────────────


def _find_blocked_cell_with_free_neighbor(
    planner: WavefrontPlanner,
    near_xy: Tuple[float, float],
) -> Tuple[float, float] | None:
    """Walk outward from `near_xy` in grid space, return world coords of
    the first OBSTACLE cell that has at least one FREE 8-neighbor.

    Centralized so the reproducer + the unit test pick the same cell.
    """
    grid = planner.get_grid()
    if grid is None:
        return None
    start_gi, start_gj = planner._world_to_grid(*near_xy)
    h, w = grid.shape

    # Scan small ring outward.
    for radius in range(0, 25):
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if max(abs(di), abs(dj)) != radius:
                    continue
                gi, gj = start_gi + di, start_gj + dj
                if not (0 <= gi < w and 0 <= gj < h):
                    continue
                if grid[gj, gi] != WavefrontPlanner.OBSTACLE:
                    continue
                # Check 8-neighbors for FREE
                for ddj in (-1, 0, 1):
                    for ddi in (-1, 0, 1):
                        if ddi == 0 and ddj == 0:
                            continue
                        ngi, ngj = gi + ddi, gj + ddj
                        if not (0 <= ngi < w and 0 <= ngj < h):
                            continue
                        if grid[ngj, ngi] == WavefrontPlanner.FREE:
                            return planner._grid_to_world(gi, gj)
    return None
