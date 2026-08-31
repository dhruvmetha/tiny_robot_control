"""The pure-navigation baselines, on scenes small enough to check by hand.

Each case is a corridor the robot must cross, with a block placed so the
right answer is obvious: drive through it, go round it, or find no way at all.
"""

import pytest

from robot_control.controller.config import load_controller_configs
from robot_control.planner.navigation_baseline import PROXIMITY_WEIGHT_OFF
from robot_control.utils.wavefront import WavefrontPlanner

from robot_control.planner.navigation_baseline import (
    COST_FREE,
    MOVABLE_COST_IGNORED,
    MOVABLE_COST_PENALISED,
    NavigationBaseline,
    sweep_movable_cost,
)

# A 1 m square is large enough that a detour costs real distance and small
# enough to flood in milliseconds at the planner's 5 mm grid.
BOUNDS = (0.0, 1.0, 0.0, 1.0)
ROBOT_W_CM = 7.0
ROBOT_H_CM = 7.0

START = (0.15, 0.5)
GOAL = (0.85, 0.5)


def _grid(statics=None, movables=None, **kwargs):
    return NavigationBaseline(
        bounds=BOUNDS,
        statics=statics or {},
        movables=movables or {},
        robot_width_cm=ROBOT_W_CM,
        robot_height_cm=ROBOT_H_CM,
        **kwargs,
    )


def test_open_room_drives_straight_across_touching_nothing():
    result = _grid().plan(START, GOAL, MOVABLE_COST_IGNORED)

    assert result.reached
    assert result.crosses_nothing
    assert len(result.waypoints) >= 2


def test_a_movable_in_the_way_is_driven_through_when_it_costs_like_floor():
    """Variant 1. The search cannot see blocks, so it takes the straight line."""
    blocked = _grid(movables={"box": (0.5, 0.5, 0.06, 0.06, 0.0)})

    result = blocked.plan(START, GOAL, MOVABLE_COST_IGNORED)

    assert result.reached
    assert result.movable_cells_crossed.get("box", 0) > 0
    assert not result.crosses_nothing


def test_the_same_movable_is_driven_around_when_it_costs_five():
    """Variant 2. Going round a small block beats paying 5x to cross it."""
    blocked = _grid(movables={"box": (0.5, 0.5, 0.06, 0.06, 0.0)})

    result = blocked.plan(START, GOAL, MOVABLE_COST_PENALISED)

    assert result.reached
    assert result.crosses_nothing, (
        f"expected a detour, but the route crossed {result.movable_cells_crossed}"
    )


def test_a_wall_spanning_the_room_blocks_both_variants():
    """Static geometry is impassable however cheap movables are."""
    walled = _grid(statics={"wall": (0.5, 0.5, 0.02, 0.9, 0.0)})

    for cost in (MOVABLE_COST_IGNORED, MOVABLE_COST_PENALISED):
        result = walled.plan(START, GOAL, cost)
        assert not result.reached
        assert result.failure == "no_route_around_static_geometry"


def test_a_movable_spanning_the_room_is_still_passable():
    """The distinction the whole baseline rests on.

    Geometrically identical to the wall above. A robot that never pushes still
    gets across, because a block is something it may drive into.
    """
    plugged = _grid(movables={"plug": (0.5, 0.5, 0.02, 0.9, 0.0)})

    result = plugged.plan(START, GOAL, MOVABLE_COST_PENALISED)

    assert result.reached
    assert result.movable_cells_crossed.get("plug", 0) > 0


def test_the_route_names_which_block_it_drove_through():
    """A count alone cannot say which object the robot would hit."""
    two = _grid(movables={
        "near": (0.35, 0.5, 0.03, 0.9, 0.0),
        "far": (0.65, 0.5, 0.03, 0.9, 0.0),
    })

    result = two.plan(START, GOAL, MOVABLE_COST_IGNORED)

    assert result.reached
    assert set(result.movable_cells_crossed) == {"near", "far"}


def test_a_wall_overlapping_a_movable_reads_as_wall():
    """A cell the robot cannot enter must never be costed as one it can."""
    overlapped = _grid(
        statics={"wall": (0.5, 0.5, 0.02, 0.9, 0.0)},
        movables={"box": (0.5, 0.5, 0.06, 0.06, 0.0)},
    )

    assert not overlapped.plan(START, GOAL, MOVABLE_COST_IGNORED).reached


def test_the_sweep_finds_the_penalty_where_a_scene_stops_driving_through():
    """A cheap way round shows up as a low threshold."""
    blocked = _grid(movables={"box": (0.5, 0.5, 0.06, 0.06, 0.0)})

    threshold = sweep_movable_cost(blocked, START, GOAL, [1.0, 1.5, 2.0, 3.0, 5.0])

    assert threshold is not None
    assert threshold > MOVABLE_COST_IGNORED


def test_the_sweep_reports_no_threshold_when_there_is_no_way_round():
    """A block spanning the room is crossed at every penalty."""
    plugged = _grid(movables={"plug": (0.5, 0.5, 0.02, 0.9, 0.0)})

    assert sweep_movable_cost(plugged, START, GOAL, [1.0, 5.0, 50.0, 500.0]) is None


def test_a_block_cheaper_than_floor_is_refused():
    """It would make the route seek obstacles out rather than avoid them."""
    with pytest.raises(ValueError, match="at least"):
        _grid().plan(START, GOAL, COST_FREE - 0.1)


def test_a_start_buried_in_static_geometry_is_recorded_not_refused():
    """The planner frees a trapped start on purpose; the baseline says so.

    `apply_trapped_start_recovery` exists so a robot slightly clipping an
    obstacle can still plan, and it matches namo_cpp. Overriding it here would
    make the baseline stricter than the navigator the trials use, which is the
    wrong direction: a generous baseline that still fails is the stronger
    result. At the table a trapped start usually means the capture or the
    ArUco pose was off, so it is worth recording rather than discarding.
    """
    walled = _grid(statics={"wall": (0.15, 0.5, 0.05, 0.05, 0.0)})

    result = walled.plan(START, GOAL, MOVABLE_COST_IGNORED)

    assert result.start_was_trapped
    # Recovery frees at most a 5x5 block of cells, 2.5 cm at this resolution.
    # This wall buries the start far deeper than that, so the route still
    # fails. The flag is what separates "walled in" from "no way through".
    assert not result.reached
    assert result.failure == "no_route_around_static_geometry"


def test_an_ordinary_start_is_not_flagged_as_trapped():
    assert not _grid().plan(START, GOAL, MOVABLE_COST_IGNORED).start_was_trapped


def test_both_axes_are_navigable_in_an_open_room():
    """A sanity check that the grid is not transposed."""
    room = _grid()

    straight = room.plan((0.2, 0.5), (0.8, 0.5), MOVABLE_COST_IGNORED)
    diagonal = room.plan((0.2, 0.2), (0.8, 0.8), MOVABLE_COST_IGNORED)

    assert len(diagonal.waypoints) > 0 and len(straight.waypoints) > 0


def test_the_baseline_plans_on_the_planner_the_trials_navigate_with():
    """Not a second grid of its own.

    `config/controller.yaml` selects the wavefront planner for real runs. A
    baseline on its own representation would measure path planners rather
    than pushing, which is the comparison it exists to support.
    """
    room = _grid(movables={"box": (0.5, 0.5, 0.06, 0.06, 0.0)})

    assert isinstance(room.planner, WavefrontPlanner)
    assert room.planner.get_grid() is not None


# ─── Wall margin, and why it does not touch the movables ────────────────

# A wall and a block of identical size, far enough apart that neither one's
# margin can reach the other. Everything below probes the same two.
BAR_HALF_X = 0.10
BAR_HALF_Y = 0.02
WALL_BAR = (0.5, 0.15, BAR_HALF_X, BAR_HALF_Y, 0.0)
BLOCK_BAR = (0.5, 0.60, BAR_HALF_X, BAR_HALF_Y, 0.0)

# Measured off the grid above: at 5 mm cells with a 2 cm falloff, the free
# cell one out from the wall costs 5.0, the next 3.0, and by the third the
# margin is spent.
WALL_EDGE_Y = 0.21
WALL_NEAR_Y = 0.22
WALL_CLEAR_Y = 0.24
BLOCK_EDGE_Y = 0.66
COST_AT_WALL_EDGE = 5.0
COST_ONE_CELL_FURTHER = 3.0


def _cost_at(baseline, x, y):
    gi, gj = baseline.planner._world_to_grid(x, y)
    return float(baseline.planner._cost_grid[gj, gi])


def _bars(movable_cost=MOVABLE_COST_IGNORED):
    baseline = _grid(statics={"wall": WALL_BAR}, movables={"box": BLOCK_BAR})
    baseline.plan_for_image(movable_cost)
    return baseline


def test_the_margin_matches_what_the_navigation_controller_uses():
    """Copying the numbers is how the baseline and the navigator drift into
    driving differently, so both read the same file."""
    nav = load_controller_configs().navigation
    room = _grid()

    assert room._config.obstacle_proximity_weight == nav.obstacle_proximity_weight
    assert room._config.obstacle_proximity_distance == pytest.approx(
        nav.obstacle_proximity_distance_cm / 100.0
    )


def test_floor_beside_a_wall_costs_more_than_open_floor():
    """Without this the route scrapes the wall and the robot clips it, which
    fails the run for a driving reason rather than a reachability one."""
    bars = _bars()

    assert _cost_at(bars, 0.5, WALL_EDGE_Y) == pytest.approx(COST_AT_WALL_EDGE)
    assert _cost_at(bars, 0.5, WALL_NEAR_Y) == pytest.approx(COST_ONE_CELL_FURTHER)
    assert _cost_at(bars, 0.5, WALL_CLEAR_Y) == pytest.approx(COST_FREE)


def test_floor_beside_a_movable_costs_exactly_open_floor():
    """The margin is wall-only, and it comes out that way for free: only
    statics reach the occupancy grid, and `_build_cost_grid` floods from
    OBSTACLE cells. A block never seeds one.

    This is what keeps the wall margin and the price of a movable separable.
    A block that cost extra merely for sitting near itself would make the
    sweep threshold report two effects mixed together.
    """
    bars = _bars()

    assert _cost_at(bars, 0.5, BLOCK_EDGE_Y) == pytest.approx(COST_FREE)


def test_a_wall_and_a_movable_of_the_same_size_are_priced_differently():
    """The single claim, read straight off the grid."""
    bars = _bars()

    assert _cost_at(bars, 0.5, WALL_EDGE_Y) > _cost_at(bars, 0.5, BLOCK_EDGE_Y)


def test_the_margin_can_be_turned_off():
    """The sweep wants it off so its threshold isolates one effect."""
    room = _grid(statics={"wall": WALL_BAR},
                 wall_proximity_weight=PROXIMITY_WEIGHT_OFF)
    room.plan_for_image(MOVABLE_COST_IGNORED)

    assert _cost_at(room, 0.5, WALL_EDGE_Y) == pytest.approx(COST_FREE)


def test_a_penalised_block_costs_more_than_the_wall_margin_beside_it():
    """The price multiplies the margin rather than replacing it.

    Assigning a flat 5 would put a block cell hard against a wall BELOW the
    bare floor next to it, which costs up to 1 + weight, and the route would
    then prefer to clip the block.
    """
    bars = _bars(movable_cost=MOVABLE_COST_PENALISED)

    assert _cost_at(bars, 0.5, 0.60) == pytest.approx(MOVABLE_COST_PENALISED)
    assert _cost_at(bars, 0.5, 0.60) > _cost_at(bars, 0.5, WALL_NEAR_Y)


def test_a_trapped_start_does_not_wipe_the_price_of_the_movables():
    """Regression. `apply_trapped_start_recovery` frees cells and then
    rebuilds the cost grid from the mutated occupancy grid. Pricing before
    that call meant a trapped start silently reset every movable to the cost
    of floor, so `penalise` ran as `ignore` and nothing said so.
    """
    buried = _grid(statics={"wall": (0.15, 0.5, 0.05, 0.05, 0.0)},
                   movables={"box": BLOCK_BAR})

    result = buried.plan(START, GOAL, MOVABLE_COST_PENALISED)

    assert result.start_was_trapped
    assert _cost_at(buried, 0.5, 0.60) == pytest.approx(MOVABLE_COST_PENALISED)
