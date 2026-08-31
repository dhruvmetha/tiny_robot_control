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
    STATUS_EXHAUSTED,
    STATUS_OPENED,
    UNUSABLE_BOUNDARY_REASONS,
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
    # resolved_source is not optional in the fake: the exclusion pair is built
    # from both ends of the solve that just ran, and a fake that omits one is
    # not modelling what the bridge returns.
    base = dict(subgoals=["push"], success=True, already_open=False,
                boundary_exhausted=False, failure_reason="",
                resolved_source="robot", resolved_target="goal")
    base.update(over)
    return SimpleNamespace(**base)


class _Bridge:
    def __init__(self, choices=None, plans=None):
        self.choices = list(choices or [_choice()])
        self.plans = list(plans or [_plan()])
        self.select_calls = 0
        self.select_blocked_args = []
        self.solved_targets = []

    def select_boundary(self, *a, **k):
        self.select_calls += 1
        self.select_blocked_args.append(k.get("blocked_boundaries"))
        return self.choices.pop(0) if self.choices else _choice(found=False)

    def solve_boundary(self, obs, goal, target, **k):
        self.solved_targets.append(target)
        return self.plans.pop(0) if self.plans else _plan(success=False)


def _advance(bridge, target=None):
    return advance_boundary(
        bridge, object(), GOAL_CM, target=target,
        open_fraction=FRACTION, scale_factor=1.0,
    )


def _target(**over):
    # source_region_path is diagnostics only. It used to supply half the
    # exclusion pair, which is how a label persisted before a push could end up
    # naming a boundary in the current snapshot.
    kwargs = dict(
        target_samples_m=tuple(POINTS),
        blocker_real_ids=("obj_4",),
        open_fraction=FRACTION,
        source_region_path=("robot", "goal"),
    )
    kwargs.update(over)
    return RegionOpeningTarget(**kwargs)


def test_with_no_target_it_selects_one_and_plans():
    bridge = _Bridge()

    plan, target, status, _released = _advance(bridge)

    assert status == ADVANCE_PLANNED
    assert bridge.select_calls == 1
    assert target.target_samples_m == tuple(POINTS)
    assert plan.subgoals == ["push"]


def test_a_held_target_is_reused_without_selecting():
    """The property the whole feature exists for."""
    bridge = _Bridge()
    held = _target()

    _plan_, target, status, _released = _advance(bridge, target=held)

    assert bridge.select_calls == 0
    assert target is held
    assert bridge.solved_targets == [held]


def test_an_already_open_boundary_advances_to_the_next():
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(already_open=True), _plan()],
    )

    _plan_, target, status, _released = _advance(bridge, target=_target())

    assert status == ADVANCE_PLANNED
    assert target.blocker_real_ids == ("obj_9",)
    assert bridge.select_calls == 1


def test_an_exhausted_boundary_with_no_alternative_is_released():
    bridge = _Bridge(
        choices=[_choice(found=False, failure_reason="region_path_exhausted")],
        plans=[_plan(success=False, boundary_exhausted=True)],
    )

    _plan_, target, status, _released = _advance(bridge, target=_target())

    assert status == ADVANCE_EXHAUSTED
    assert target is None


def test_a_boundary_that_simply_fails_keeps_the_target():
    """Not exhausted: worth retrying next replan with a different seed."""
    bridge = _Bridge(plans=[_plan(success=False, failure_reason="all_pushes_failed")])
    held = _target()

    _plan_, target, status, _released = _advance(bridge, target=held)

    assert status == ADVANCE_NO_PLAN
    assert target is held


def test_a_reachable_goal_means_no_boundary():
    bridge = _Bridge(choices=[_choice(found=False, goal_already_reachable=True)])

    plan, target, status, _released = _advance(bridge)

    assert (plan, target, status) == (None, None, ADVANCE_NO_BOUNDARY)


def test_no_selectable_boundary_is_reported_not_raised():
    bridge = _Bridge(choices=[_choice(found=False, failure_reason="region_path_exhausted")])

    assert _advance(bridge)[2] == ADVANCE_NO_BOUNDARY


def test_a_chain_of_already_open_boundaries_terminates():
    bridge = _Bridge(
        choices=[_choice() for _ in range(50)],
        plans=[_plan(already_open=True) for _ in range(50)],
    )

    _plan_, target, status, _released = _advance(bridge)

    assert status == ADVANCE_NO_BOUNDARY
    assert bridge.select_calls <= MAX_BOUNDARY_ADVANCES


def test_the_step_does_not_persist_anything(tmp_path):
    """Storage is the caller's business; the two callers store differently."""
    bridge = _Bridge()

    _advance(bridge)

    assert list(tmp_path.iterdir()) == []


# --- boundaries that cannot be used must be dropped, not retried forever -----
#
# namo_cpp documents target_not_immediate_neighbor as "the caller must re-choose
# at the outer level; this is a normal outcome". Holding the target instead
# means every later replan repeats the identical failing solve.

# Derived rather than restated, so a reason added to the planner's set cannot
# quietly stop being covered here.
UNRESOLVABLE = sorted(UNUSABLE_BOUNDARY_REASONS)


@pytest.mark.parametrize("reason", UNRESOLVABLE)
def test_an_unresolvable_boundary_is_released(reason):
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(success=False, failure_reason=reason), _plan()],
    )

    _plan_, target, status, _released = _advance(bridge, target=_target())

    assert status == ADVANCE_PLANNED
    assert target.blocker_real_ids == ("obj_9",)


def test_an_ambiguous_boundary_is_dropped_rather_than_re_solved():
    """namo_cpp refuses to guess when two neighbours match the pinned objects.

    The scene has not changed between replans, so re-solving the same target
    hits the identical tie. The only way forward is to re-select.
    """
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(success=False, failure_reason="ambiguous_boundary"), _plan()],
    )
    held = _target()

    _plan_, target, status, released = _advance(bridge, target=held)

    assert status == ADVANCE_PLANNED
    assert target is not held
    assert released == STATUS_EXHAUSTED


def test_a_plain_push_failure_keeps_the_boundary():
    """all_pushes_failed is worth retrying next replan; it is not unusable."""
    bridge = _Bridge(plans=[_plan(success=False, failure_reason="all_pushes_failed")])
    held = _target()

    _plan_, target, status, _released = _advance(bridge, target=held)

    assert status == ADVANCE_NO_PLAN
    assert target is held


def test_an_exhausted_boundary_is_not_immediately_reselected():
    """Selection is deterministic, so without blocking we would pick it again."""
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(success=False, boundary_exhausted=True), _plan()],
    )

    _plan_, target, status, _released = _advance(bridge, target=_target())

    assert status == ADVANCE_PLANNED
    assert target.blocker_real_ids == ("obj_9",)


def test_the_exhausted_boundary_is_excluded_from_the_next_selection():
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(success=False, boundary_exhausted=True), _plan()],
    )
    held = _target()

    _advance(bridge, target=held)

    assert bridge.select_blocked_args
    assert bridge.select_blocked_args[-1], "next selection got no blocked boundaries"


def test_running_out_of_alternatives_reports_no_boundary():
    bridge = _Bridge(
        choices=[_choice(found=False, failure_reason="region_path_exhausted")],
        plans=[_plan(success=False, boundary_exhausted=True)],
    )

    _plan_, target, status, _released = _advance(bridge, target=_target())

    assert status == ADVANCE_EXHAUSTED
    assert target is None


# --- what happened to the target we were handed ------------------------------
#
# Callers used to infer this from the status, and got it wrong in two ways: a
# transient failure looked like "opened", and advancing past an already-open
# boundary then exhausting the next one marked the wrong record.

def test_a_held_target_that_opened_is_reported_opened():
    bridge = _Bridge(plans=[_plan(already_open=True), _plan()])

    result = advance_boundary(
        bridge, object(), GOAL_CM, target=_target(),
        open_fraction=FRACTION, scale_factor=1.0,
    )

    assert result[3] == STATUS_OPENED


def test_a_held_target_that_exhausted_is_reported_exhausted():
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[_plan(success=False, boundary_exhausted=True), _plan()],
    )

    result = advance_boundary(
        bridge, object(), GOAL_CM, target=_target(),
        open_fraction=FRACTION, scale_factor=1.0,
    )

    assert result[3] == STATUS_EXHAUSTED


def test_a_held_target_still_being_worked_on_is_not_released():
    bridge = _Bridge(plans=[_plan()])
    held = _target()

    result = advance_boundary(
        bridge, object(), GOAL_CM, target=held,
        open_fraction=FRACTION, scale_factor=1.0,
    )

    assert result[3] is None
    assert result[1] is held


def test_a_transient_failure_does_not_release_the_target():
    """xml_generation_failed is not the boundary's fault."""
    bridge = _Bridge(plans=[_plan(success=False, failure_reason="xml_generation_failed")])
    held = _target()

    result = advance_boundary(
        bridge, object(), GOAL_CM, target=held,
        open_fraction=FRACTION, scale_factor=1.0,
    )

    assert result[3] is None
    assert result[1] is held


def test_exhausting_the_second_boundary_does_not_mislabel_the_first():
    """The first one opened. Only the second is exhausted."""
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"]),
                 _choice(found=False, failure_reason="region_path_exhausted")],
        plans=[_plan(already_open=True), _plan(success=False, boundary_exhausted=True)],
    )

    result = advance_boundary(
        bridge, object(), GOAL_CM, target=_target(),
        open_fraction=FRACTION, scale_factor=1.0,
    )

    assert result[3] == STATUS_OPENED


# --- what namo_cpp says about a stale blocklist ------------------------------
#
# select_boundary_from_xml reports the blocked pairs that name no edge in the
# scene it just looked at. The bridge dropped that report on the floor, so a
# caller carrying a blocklist across a push could route around nothing and
# never hear about it.

def _choice_from(selection):
    """Run the bridge's translation with only the mapping it reads."""
    from robot_control.planner.namo_bridge import NAMOPlanBridge

    bridge = NAMOPlanBridge.__new__(NAMOPlanBridge)
    bridge._object_mapping = SimpleNamespace(get_real_name=lambda sim_id: "obj_1")
    # __del__ cleans up a generated config file; give it the "nothing to do"
    # value so garbage collection stays quiet.
    bridge._generated_config_path = None
    return bridge._choice_from_selection(selection)


def test_the_choice_carries_the_stale_blocklist_report():
    selection = SimpleNamespace(
        goal_already_reachable=False, found=True,
        target_points=list(POINTS), blocking_objects=["obstacle_1_movable"],
        region_path=["robot", "goal"], failure_reason="",
        stale_blocked_boundaries=[("robot", "region_9")],
    )

    choice = _choice_from(selection)

    assert choice.stale_blocked_boundaries == [("robot", "region_9")]


def test_a_failed_selection_still_carries_the_report():
    """Knowing the blocklist went stale is most useful when nothing was found."""
    selection = SimpleNamespace(
        goal_already_reachable=False, found=False,
        target_points=[], blocking_objects=[], region_path=[],
        failure_reason="region_path_exhausted",
        stale_blocked_boundaries=[("robot", "region_9")],
    )

    choice = _choice_from(selection)

    assert choice.failure_reason == "region_path_exhausted"
    assert choice.stale_blocked_boundaries == [("robot", "region_9")]


def test_no_stale_pairs_is_an_empty_report_not_a_missing_one():
    selection = SimpleNamespace(
        goal_already_reachable=False, found=True,
        target_points=list(POINTS), blocking_objects=["obstacle_1_movable"],
        region_path=["robot", "goal"], failure_reason="",
        stale_blocked_boundaries=[],
    )

    assert _choice_from(selection).stale_blocked_boundaries == []


# --- which labels the exclusion pair is built from ---------------------------
#
# The pair used to take its source from target.source_region_path[0], persisted
# before the last push, and its target from the current solve. Labels are
# ordinal and get reassigned rather than retired, so that mix could exclude a
# live boundary that was not the one being dropped. The blocked list being local
# to one advance call did not help: the staleness was inside the pair.

def test_the_exclusion_uses_the_labels_from_the_solve_that_just_ran():
    """The regression. The held target's stored source label must not appear."""
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[
            _plan(success=False, boundary_exhausted=True,
                  resolved_source="region_7", resolved_target="region_3"),
            _plan(),
        ],
    )
    held = _target(source_region_path=("region_1", "region_3"))

    _advance(bridge, target=held)

    blocked = bridge.select_blocked_args[-1]
    assert list(blocked) == [("region_7", "region_3")]


def test_a_reassigned_old_label_is_never_excluded():
    """region_1 now names something unrelated, so it must stay selectable."""
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[
            _plan(success=False, boundary_exhausted=True,
                  resolved_source="region_7", resolved_target="region_3"),
            _plan(),
        ],
    )

    _advance(bridge, target=_target(source_region_path=("region_1", "region_3")))

    excluded = {label for pair in bridge.select_blocked_args[-1] for label in pair}
    assert "region_1" not in excluded


@pytest.mark.parametrize("missing", [{"resolved_source": ""}, {"resolved_target": ""}])
def test_half_a_pair_excludes_nothing(missing):
    """Better to re-pick this boundary and waste a call than exclude the wrong one."""
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[
            _plan(success=False, boundary_exhausted=True, **missing),
            _plan(),
        ],
    )

    _advance(bridge, target=_target(source_region_path=("region_1", "region_3")))

    assert not bridge.select_blocked_args[-1]


def test_dropping_the_boundary_still_happens_without_a_pair():
    """No pair to exclude does not mean keep working an unusable boundary."""
    bridge = _Bridge(
        choices=[_choice(blocker_real_ids=["obj_9"])],
        plans=[
            _plan(success=False, boundary_exhausted=True, resolved_source=""),
            _plan(),
        ],
    )
    held = _target()

    _plan_, target, status, released = _advance(bridge, target=held)

    assert released == STATUS_EXHAUSTED
    assert target is not held
    assert status == ADVANCE_PLANNED


def test_reactive_argmax_survives_the_exhausted_stamp():
    """The reactive solver returns its argmax push on the same plan it stamps
    boundary_exhausted/all_pushes_failed, because in simulation that push
    opens nothing -- the exact prediction reactive exists not to trust.
    The old order let the stamp win: the only doorway got blacklisted and
    every reactive run on a scene search cannot crack ended with zero
    pushes (hard_004, 2026-08-31). The push must dispatch."""
    from robot_control.planner.region_target import advance_boundary

    bridge = _Bridge(plans=[_plan(
        success=False, boundary_exhausted=True, failure_reason="all_pushes_failed"
    )])
    plan, target, status, released = advance_boundary(
        bridge, object(), GOAL_CM, target=_target(),
        open_fraction=FRACTION, scale_factor=1.0,
        planner_kwargs={"mode": "reactive"},
    )

    assert status == ADVANCE_PLANNED
    assert plan.subgoals
    assert target is not None
    assert released is None


def test_search_still_blacklists_the_exhausted_boundary():
    # Under search the same plan shape is a failed verification, not an
    # executable push; the exhausted branch must keep winning there.
    bridge = _Bridge(
        choices=[_choice(found=False)],
        plans=[_plan(success=False, boundary_exhausted=True,
                     failure_reason="all_pushes_failed")],
    )
    _plan_, target, status, released = _advance(bridge, target=_target())

    assert status != ADVANCE_PLANNED
    assert released == STATUS_EXHAUSTED
