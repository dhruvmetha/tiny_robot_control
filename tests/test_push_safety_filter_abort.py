"""The push controller with and without the safety filter.

A scripted observation stream drives the real PushController. No nav
controller, so the approach is skipped and the push starts on the first tick.
The scene: a 4 x 4 cm block at (20, 30) pushed along +X from its left face,
and a brick parallel to the push line whose inner face sits 2.2 cm above the
line and whose near end is 7 cm ahead of the robot's nose. The block passes
under the brick. The 7 cm robot does not: its front corners reach the brick's
end after 0.2 cm per tick of scripted motion.

The stream keeps moving the robot +X during the retreat too. That is not
physical, and it is exactly what makes the golden sequence for the "off"
case deterministic: the same numbers came out of the controller before the
filter existed (commit 7382e59), captured with the scratch harness that
drives this same stream through that file.
"""

from __future__ import annotations

import contextlib
import io
from types import SimpleNamespace
from typing import List, Optional, Tuple

from robot_control.controller.config import PushConfig
from robot_control.controller.edge_points import get_edge_point
from robot_control.controller.push import PushController, PushState
from robot_control.controller.safety_filter import (
    HALF_CELL_DIAGONAL_CM,
    SafetyFilter,
)
from robot_control.core.types import (
    Action,
    ObjectPose,
    Observation,
    PushSubgoal,
    WorkspaceConfig,
)

WS = WorkspaceConfig(width=40.0, height=60.0, car_width=7.0, car_height=7.0, offset_w=3.5, offset_h=3.5)
MARGIN_CM = 1.5
STEP_CM = 0.2                  # scripted robot motion per tick
POINTS_PER_FACE = 15
STANDOFF_CM = 3.5 + 1.0        # robot radius + edge_offset_margin_cm, as PushController computes it
PUSH_STEPS = 2                 # x 30 ticks per step = 60 push ticks
BLOCK_X, BLOCK_Y = 20.0, 30.0
BRICK_NEAR_END_X = 24.0
BRICK_DEPTH = 12.0
BRICK_WIDTH = 5.5
BRICK_GAP_ABOVE_LINE = 2.2     # block half-width 2 passes; robot half-width 3.5 clips
ROBOT_START_X = BLOCK_X - 2.0 - STANDOFF_CM   # 13.5
NOSE_GAP_AT_START = BRICK_NEAR_END_X - (ROBOT_START_X + 3.5)  # 7 cm


def _config(**overrides) -> PushConfig:
    base = dict(
        max_speed=0.4, edge_offset_margin_cm=1.0, wheel_deadband=0.05, lookahead_ratio=1.0,
        push_steps=30, dynamic_direction=False, approach_skip_distance=1.0, approach_skip_angle=30.0,
        points_per_face=POINTS_PER_FACE, advance_speed=0.15, advance_steps=0,
        retreat_speed=0.15, retreat_steps=10, retreat_min_dist=5.0, retreat_max_dist=15.0,
        retreat_tolerance=2.0, min_push_displacement_cm=2.0, min_push_rotation_deg=5.0,
        show_wavefront=False,
    )
    base.update(overrides)
    return PushConfig(**base)


def _objects(shift: float, with_wall: bool = True):
    objs = {"obj": ObjectPose(x=BLOCK_X + shift, y=BLOCK_Y, theta=0.0, width=4.0, depth=4.0, height=4.0)}
    if with_wall:
        objs["wall_a"] = ObjectPose(
            x=BRICK_NEAR_END_X + BRICK_DEPTH / 2,
            y=BLOCK_Y + BRICK_GAP_ABOVE_LINE + BRICK_WIDTH / 2,
            theta=0.0, width=BRICK_WIDTH, depth=BRICK_DEPTH, height=5.0, is_static=True,
        )
    return objs


def _edge_idx() -> int:
    obj = _objects(0.0)["obj"]
    facing_plus_x = [
        i for i in range(4 * POINTS_PER_FACE)
        if abs(get_edge_point(obj, i, STANDOFF_CM, POINTS_PER_FACE).approach_theta) < 1e-6
    ]
    return min(facing_plus_x, key=lambda i: abs(get_edge_point(obj, i, STANDOFF_CM, POINTS_PER_FACE).position[1] - BLOCK_Y))


def _obs(tick: int, with_wall: bool = True) -> Observation:
    shift = STEP_CM * tick
    return Observation(
        robot_x=ROBOT_START_X + shift, robot_y=BLOCK_Y, robot_theta=0.0,
        objects=_objects(shift, with_wall), timestamp=tick / 30.0,
    )


def _run(controller: PushController, ticks: int, wall_from_tick: Optional[int] = None):
    """Drive the stream. Returns [(state_before, action)] and the subgoal."""
    sub = PushSubgoal(object_id="obj", edge_idx=_edge_idx(), push_steps=PUSH_STEPS)
    controller.reset()
    trace: List[Tuple[str, Action]] = []
    for t in range(ticks):
        with_wall = True if wall_from_tick is None else t < wall_from_tick
        obs = _obs(t, with_wall)
        state = controller._state.value
        with contextlib.redirect_stdout(io.StringIO()):
            action = controller.step(obs, sub)
        trace.append((state, action))
        if controller.is_done(obs, sub):
            break
    return trace, sub


def _first_tick(trace, state: str) -> Optional[int]:
    return next((i for i, (s, _) in enumerate(trace) if s == state), None)


def _filter() -> SafetyFilter:
    return SafetyFilter(WS, margin_cm=MARGIN_CM)


# Ticks between which the abort must land. The nose gap shrinks by STEP_CM
# per tick from 7 cm; the lookup reads between the true gap and one cell
# diagonal under it, so the first tick under the margin is bracketed by the
# true gap crossing MARGIN and crossing MARGIN + 2 * HALF_CELL_DIAGONAL_CM.
EARLIEST_ABORT_TICK = int((NOSE_GAP_AT_START - MARGIN_CM - 2 * HALF_CELL_DIAGONAL_CM) / STEP_CM)
LATEST_ABORT_TICK = int((NOSE_GAP_AT_START - MARGIN_CM) / STEP_CM) + 1


def test_the_filter_stops_the_push_before_the_robot_reaches_the_brick():
    controller = PushController(WS, nav_controller=None, push_config=_config(), safety_filter=_filter())
    trace, sub = _run(controller, 200)

    retreat_at = _first_tick(trace, PushState.RETREATING.value)
    assert retreat_at is not None
    abort_tick = retreat_at - 1
    assert EARLIEST_ABORT_TICK <= abort_tick <= LATEST_ABORT_TICK, abort_tick
    # The tick that fired commanded a stop, not the push action.
    assert trace[abort_tick] == (PushState.PUSHING.value, Action.stop())
    # Every earlier tick was the plain push.
    assert all(a == Action(0.4, 0.4) for s, a in trace[1:abort_tick])
    # True nose gap at the stop: never under MARGIN minus one tick of motion.
    gap = NOSE_GAP_AT_START - STEP_CM * abort_tick
    assert gap >= MARGIN_CM - STEP_CM

    assert controller.did_fail() is True
    summary = controller.get_last_push_summary(_obs(abort_tick))
    assert summary["safety_filter"] is True
    assert summary["abort_reason"] == "robot_static_clearance:wall_a"
    assert summary["abort_distance_now_cm"] < MARGIN_CM
    assert summary["abort_distance_at_start_cm"] > MARGIN_CM
    # The skip-approach path pushes on tick 0 too, so ticks = index + 1.
    assert summary["push_ticks_executed"] == abort_tick + 1


def test_a_wall_seen_only_on_the_first_frame_still_stops_the_push():
    controller = PushController(WS, nav_controller=None, push_config=_config(), safety_filter=_filter())
    trace, _ = _run(controller, 200, wall_from_tick=1)
    retreat_at = _first_tick(trace, PushState.RETREATING.value)
    assert retreat_at is not None
    assert EARLIEST_ABORT_TICK <= retreat_at - 1 <= LATEST_ABORT_TICK


def _instant_nav():
    """A navigator that accepts every target and is done on its next tick.

    ADVANCING only follows APPROACHING, and APPROACHING needs a navigator.
    """
    return SimpleNamespace(
        navigate_to=lambda *args, **kwargs: True,
        is_done=lambda obs, subgoal: True,
        step=lambda obs, subgoal: Action.stop(),
        state=SimpleNamespace(value="DONE"),
        cancel=lambda: None,
        get_drawings=lambda: [],
    )


def test_an_abort_during_advancing_still_produces_a_push_record():
    # No approach skip and 200 advance ticks: the robot reaches the brick
    # while still ADVANCING.
    controller = PushController(
        WS, nav_controller=_instant_nav(),
        push_config=_config(advance_steps=200, approach_skip_distance=0.0),
        safety_filter=_filter(),
    )
    trace, _ = _run(controller, 200)
    retreat_at = _first_tick(trace, PushState.RETREATING.value)
    assert retreat_at is not None
    assert _first_tick(trace, PushState.PUSHING.value) is None
    assert trace[retreat_at - 1][0] == PushState.ADVANCING.value
    assert trace[retreat_at - 1][1] == Action.stop()
    summary = controller.get_last_push_summary(_obs(retreat_at))
    assert summary is not None
    assert summary["abort_reason"] == "robot_static_clearance:wall_a"
    assert summary["push_ticks_executed"] == 0
    assert controller.did_fail() is True


def test_the_status_and_drawings_show_the_abort():
    controller = PushController(WS, nav_controller=None, push_config=_config(), safety_filter=_filter())
    _run(controller, 200)
    assert "CLEARANCE ABORT" in controller.get_status() or controller._state == PushState.FINISHED
    uuids = {d["uuid"] for d in controller.get_drawings()}
    assert "safety_robot_outline" in uuids
    assert "safety_static_wall_a" in uuids


# ---------------------------------------------------------------------------
# Blind retreat
# ---------------------------------------------------------------------------

def _bare_retreating_controller(safety_filter) -> PushController:
    """Only what _blind_retreat and _report_retreat_outcome read."""
    c = PushController.__new__(PushController)
    c._config = WS
    c._push_config = SimpleNamespace(retreat_steps=100, retreat_speed=0.15, retreat_tolerance=2.0)
    c._safety_filter = safety_filter
    c._clearance_at_start = None
    c._last_outline = None
    c._last_check_violated = False
    c._retreat_target = None
    c._retreat_step_count = 0
    c._retreat_start_pose = None
    c._state = PushState.RETREATING
    return c


def test_the_blind_retreat_stops_at_a_wall_behind_the_robot():
    f = SafetyFilter(WS, margin_cm=MARGIN_CM)
    f.add_statics(Observation(
        robot_x=0, robot_y=0, robot_theta=0, timestamp=0.0,
        # theta 0: the 5.5 cm depth runs along x, the 19.5 cm width along y.
        objects={"wall_b": ObjectPose(x=5.0, y=30.0, theta=0.0, width=19.5, depth=5.5, height=5.0, is_static=True)},
    ))
    c = _bare_retreating_controller(f)
    # Wall's near face at x = 7.75; robot tail starts 6 cm away and backs up.
    x = 7.75 + 3.5 + 6.0
    stopped_at = None
    for tick in range(60):
        obs = SimpleNamespace(robot_x=x, robot_y=30.0, robot_theta=0.0, objects={})
        with contextlib.redirect_stdout(io.StringIO()):
            action = c._blind_retreat(obs)
        if c._state == PushState.FINISHED:
            stopped_at = tick
            assert action == Action.stop()
            break
        assert action == Action(-0.15, -0.15)
        x -= STEP_CM
    assert stopped_at is not None
    tail_gap = (x - 3.5) - 7.75
    assert MARGIN_CM - STEP_CM <= tail_gap <= MARGIN_CM + 2 * HALF_CELL_DIAGONAL_CM + STEP_CM


def test_the_blind_retreat_without_a_filter_runs_its_budget():
    c = _bare_retreating_controller(None)
    obs = SimpleNamespace(robot_x=20.0, robot_y=30.0, robot_theta=0.0, objects={})
    with contextlib.redirect_stdout(io.StringIO()):
        actions = [c._blind_retreat(obs) for _ in range(101)]
    assert actions[:100] == [Action(-0.15, -0.15)] * 100
    assert actions[100] == Action.stop()
    assert c._state == PushState.FINISHED


# ---------------------------------------------------------------------------
# Off means unchanged
# ---------------------------------------------------------------------------

# What the pre-filter controller (commit 7382e59) emits for this stream over
# 80 ticks: 60 push ticks, the first retreat action on the tick that spends
# the budget, 19 steered reverse ticks, then the retreat timeout stop.
GOLDEN_PUSH = [("IDLE", (0.4, 0.4))] + [("PUSHING", (0.4, 0.4))] * 59
GOLDEN_RETREAT = [
    ("PUSHING", (-0.25472, -0.04528)),
    ("RETREATING", (-0.250764, -0.049236)), ("RETREATING", (-0.247087, -0.052913)),
    ("RETREATING", (-0.24366, -0.05634)), ("RETREATING", (-0.240459, -0.059541)),
    ("RETREATING", (-0.237464, -0.062536)), ("RETREATING", (-0.234656, -0.065344)),
    ("RETREATING", (-0.232018, -0.067982)), ("RETREATING", (-0.229536, -0.070464)),
    ("RETREATING", (-0.227196, -0.072804)), ("RETREATING", (-0.224987, -0.075013)),
    ("RETREATING", (-0.222899, -0.077101)), ("RETREATING", (-0.220922, -0.079078)),
    ("RETREATING", (-0.219047, -0.080953)), ("RETREATING", (-0.217268, -0.082732)),
    ("RETREATING", (-0.215576, -0.084424)), ("RETREATING", (-0.213966, -0.086034)),
    ("RETREATING", (-0.212432, -0.087568)), ("RETREATING", (-0.210969, -0.089031)),
    ("RETREATING", (0.0, 0.0)),
]


def test_without_a_filter_the_controller_emits_the_pre_filter_sequence():
    controller = PushController(WS, nav_controller=None, push_config=_config())
    trace, _ = _run(controller, 80)
    got = [(s, (round(a.left_speed, 6), round(a.right_speed, 6))) for s, a in trace]
    assert got == GOLDEN_PUSH + GOLDEN_RETREAT
    assert controller.did_fail() is False
    summary = controller.get_last_push_summary(_obs(80))
    assert summary["safety_filter"] is False
    assert summary["abort_reason"] is None
