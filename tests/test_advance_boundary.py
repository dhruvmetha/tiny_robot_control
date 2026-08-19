"""The single advance step both entry points drive.

The in-process planner and run_namo's plan-only subprocess share no other code
-- the session path spawns a fresh process per replan and never constructs
NAMOPlanner. A second copy of "resume the held boundary, or pick the next one"
would drift the way the two chain-reuse ladders in this repo already have, so
both call this.

It is deliberately storage-free: it takes the current target and returns the
one the caller should now hold. Persisting it is the caller's business, because
the two persist to different places.
"""

from types import SimpleNamespace

import pytest

from robot_control.planner.region_target import (
    ADVANCE_EXHAUSTED,
    ADVANCE_NO_BOUNDARY,
    ADVANCE_NO_PLAN,
    ADVANCE_PLANNED,
    MAX_BOUNDARY_ADVANCES,
    RegionOpeningTarget,
    advance_boundary,
)

POINTS = [(0.30, 0.40), (0.31, 0.40)]
FRACTION = 0.2
GOAL_CM = (40.0, 40.0)


def _choice(**over):
    base = dict(found=True, target_points_m=list(POINTS), blocker_real_ids=["obj_4"],
                region_path=["robot", "goal"], goal_already_reachable=False, failure_reason="")
    base.update(over)
    return SimpleNamespace(**base)


def _plan(**over):
    base = dict(subgoals=["push"], success=True, already_open=False,
                boundary_exhausted=False, failure_reason="", resolved_target="goal")
    base.update(over)
    return SimpleNamespace(**base)


class _Bridge:
    def __init__(self, choices=None, plans=None):
        self.choices = list(choices or [_choice()])
        self.plans = list(plans or [_plan()])
        self.select_calls = 0
        self.solved_targets = []

    def select_boundary(self, *a, **k):
        self.select_calls += 1
        return self.choices.pop(0) if self.choices else _choice(found=False)

    def solve_boundary(self, obs, goal, target, **k):
        self.solved_targets.append(target)
        return self.plans.pop(0) if self.plans else _plan(success=False)


def _advance(bridge, target=None):
    return advance_boundary(
        bridge, object(), GOAL_CM, target=target,
        open_fraction=FRACTION, scale_factor=1.0,
    )


def _target():
    return RegionOpeningTarget(
        target_samples_m=tuple(POINTS), blocker_real_ids=("obj_4",), open_fraction=FRACTION
    )


def test_with_no_target_it_selects_one_and_plans():
    bridge = _Bridge()

    plan, target, status = _advance(bridge)

    assert status == ADVANCE_PLANNED
    assert bridge.select_calls == 1
    assert target.target_samples_m == tuple(POINTS)
    assert plan.subgoals == ["push"]


def test_a_held_target_is_reused_without_selecting():
    """The property the whole feature exists for."""
    bridge = _Bridge()
    held = _target()

    _plan_, target, status = _advance(bridge, target=held)

    assert bridge.select_calls == 0
    assert target is held
    assert bridge.solved_targets == [held]


def test_an_already_open_boundary_advances_to_the_next():
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(already_open=True), _plan()],
    )

    _plan_, target, status = _advance(bridge, target=_target())

    assert status == ADVANCE_PLANNED
    assert target.blocker_real_ids == ("obj_9",)
    assert bridge.select_calls == 1


def test_an_exhausted_boundary_releases_the_target():
    bridge = _Bridge(plans=[_plan(success=False, boundary_exhausted=True)])

    _plan_, target, status = _advance(bridge, target=_target())

    assert status == ADVANCE_EXHAUSTED
    assert target is None


def test_a_boundary_that_simply_fails_keeps_the_target():
    """Not exhausted: worth retrying next replan with a different seed."""
    bridge = _Bridge(plans=[_plan(success=False, failure_reason="all_pushes_failed")])
    held = _target()

    _plan_, target, status = _advance(bridge, target=held)

    assert status == ADVANCE_NO_PLAN
    assert target is held


def test_a_reachable_goal_means_no_boundary():
    bridge = _Bridge(choices=[_choice(found=False, goal_already_reachable=True)])

    plan, target, status = _advance(bridge)

    assert (plan, target, status) == (None, None, ADVANCE_NO_BOUNDARY)


def test_no_selectable_boundary_is_reported_not_raised():
    bridge = _Bridge(choices=[_choice(found=False, failure_reason="region_path_exhausted")])

    assert _advance(bridge)[2] == ADVANCE_NO_BOUNDARY


def test_a_chain_of_already_open_boundaries_terminates():
    bridge = _Bridge(
        choices=[_choice() for _ in range(50)],
        plans=[_plan(already_open=True) for _ in range(50)],
    )

    _plan_, target, status = _advance(bridge)

    assert status == ADVANCE_NO_BOUNDARY
    assert bridge.select_calls <= MAX_BOUNDARY_ADVANCES


def test_the_step_does_not_persist_anything(tmp_path):
    """Storage is the caller's business; the two callers store differently."""
    bridge = _Bridge()

    _advance(bridge)

    assert list(tmp_path.iterdir()) == []
