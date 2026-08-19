"""The record that makes one region-opening subproblem outlive a push.

Two identity rules are the whole point, and both are the opposite of the
obvious choice. The boundary is identified by its frozen sample points, not by
a region label -- labels are ordinal and renumber whenever a push re-partitions
free space. The blocker is identified by its real id, not its simulator id --
`obstacle_N_movable` is a rank over the movables present in one observation, so
a single dropped marker shifts every name after it.

These tests pin the round trip (the record crosses a process boundary), the
lifecycle, and the accumulation of failed pushes against the subproblem rather
than against the planner instance.
"""

import json

import pytest

from robot_control.planner.region_target import (
    ACTIVE_TARGET_FILENAME,
    SCHEMA_VERSION,
    STATUS_EXHAUSTED,
    STATUS_OPENED,
    RegionOpeningTarget,
    fingerprint_samples,
    target_from_selection,
    target_path_for_run,
)

POINTS = ((0.30, 0.40), (0.31, 0.40), (0.32, 0.41))
BLOCKERS = ("obj_4", "obj_7")
CANONICAL_FRACTION = 0.2


def _target(**overrides):
    kwargs = dict(
        target_samples_m=POINTS,
        blocker_real_ids=BLOCKERS,
        open_fraction=CANONICAL_FRACTION,
        target_id="ro-0001",
        selected_iteration=1,
    )
    kwargs.update(overrides)
    return RegionOpeningTarget(**kwargs)


# --- the name the identity check compares ------------------------------------
#
# target_id was f"ro-{iteration:04d}". Acceptance criterion 6 of
# MODEL_GUIDED_RO_INTEGRATION.local.md asks that the replan after a setup push
# carry the same target_id, which a counter satisfies no matter which boundary
# the planner actually landed on. These pin the properties that make the
# comparison worth making.

def _selection(points=POINTS, blockers=BLOCKERS, region_path=("robot", "goal")):
    return type(
        "Sel", (), {"target_points_m": list(points),
                    "blocker_real_ids": list(blockers),
                    "region_path": list(region_path)}
    )()


def _freeze(points=POINTS, blockers=BLOCKERS, **over):
    kwargs = dict(open_fraction=CANONICAL_FRACTION, iteration=0)
    kwargs.update(over)
    return target_from_selection(_selection(points, blockers), **kwargs)


def test_the_same_points_get_the_same_name_at_different_iterations():
    """The regression: a counter made two pushes of one boundary look distinct."""
    first = _freeze(iteration=1)
    second = _freeze(iteration=7)

    assert first.target_id == second.target_id
    assert first.selected_iteration != second.selected_iteration


def test_a_different_boundary_cannot_borrow_the_name():
    moved = tuple((x + 0.05, y) for x, y in POINTS)

    assert _freeze().target_id != _freeze(moved).target_id


def test_reordering_the_points_is_a_different_target():
    """Order is part of the frozen criterion, so the fingerprint does not sort."""
    assert _freeze().target_id != _freeze(tuple(reversed(POINTS))).target_id


def test_the_name_survives_a_save_and_load(tmp_path):
    original = _freeze()
    path = tmp_path / ACTIVE_TARGET_FILENAME
    original.save(path)

    revived = RegionOpeningTarget.load(path)

    assert revived.target_id == original.target_id
    assert revived.target_id == fingerprint_samples(POINTS)


def test_only_the_points_decide_the_name():
    """The doc calls this the target-sample hash; blockers are checked separately."""
    other = _freeze(blockers=("obj_99",), open_fraction=0.5)

    assert other.target_id == _freeze().target_id


def test_a_difference_below_a_micron_is_the_same_target():
    """Guards the id against a change in how points get formatted on disk."""
    jittered = tuple((x + 1e-9, y - 1e-9) for x, y in POINTS)

    assert _freeze(jittered).target_id == _freeze().target_id


def test_an_explicit_name_still_wins():
    """A caller reviving an older record keeps the name it was stored under."""
    assert _freeze(target_id="ro-0001").target_id == "ro-0001"


def test_the_name_is_readable_in_a_log_line():
    name = fingerprint_samples(POINTS)

    assert name.startswith("ro-")
    assert len(name) == len("ro-") + 12
    assert name[3:].isalnum()


# --- round trip --------------------------------------------------------------

def test_round_trip_preserves_every_field():
    original = _target(
        failed_pushes=(("obj_4", 17),), physical_pushes_attempted=2, last_iteration=3
    )

    assert RegionOpeningTarget.from_dict(original.to_dict()) == original


def test_points_survive_json_exactly():
    """The criterion crosses a process boundary; drift in it is silent."""
    original = _target()

    revived = RegionOpeningTarget.from_dict(json.loads(json.dumps(original.to_dict())))

    assert revived.target_samples_m == POINTS


def test_file_round_trip(tmp_path):
    original = _target()
    path = target_path_for_run(tmp_path)

    original.save(path)

    assert path.name == ACTIVE_TARGET_FILENAME
    assert RegionOpeningTarget.load(path) == original


def test_save_leaves_no_temp_file_behind(tmp_path):
    path = target_path_for_run(tmp_path)

    _target().save(path)

    assert [p.name for p in tmp_path.iterdir()] == [ACTIVE_TARGET_FILENAME]


def test_missing_file_is_no_active_target(tmp_path):
    assert RegionOpeningTarget.load(target_path_for_run(tmp_path)) is None


def test_a_future_schema_is_refused_not_guessed(tmp_path):
    path = target_path_for_run(tmp_path)
    payload = _target().to_dict()
    payload["schema_version"] = SCHEMA_VERSION + 1
    path.write_text(json.dumps(payload))

    with pytest.raises(ValueError, match="schema_version"):
        RegionOpeningTarget.load(path)


def test_record_holds_no_paths():
    """A run directory can be relocated without rewriting this file."""
    serialized = json.dumps(_target().to_dict())

    assert "/" not in serialized.replace("\\/", "")


# --- lifecycle ---------------------------------------------------------------

def test_a_new_target_is_active():
    assert _target().is_active


@pytest.mark.parametrize("status", [STATUS_OPENED, STATUS_EXHAUSTED])
def test_released_targets_stop_being_active(status, tmp_path):
    path = target_path_for_run(tmp_path)

    _target().released(status).save(path)

    assert RegionOpeningTarget.load(path) is None


def test_releasing_to_a_nonsense_status_is_rejected():
    with pytest.raises(ValueError, match="Release status"):
        _target().released("paused")


def test_failed_pushes_accumulate_against_the_subproblem():
    target = _target().with_failed_push("obj_4", 17).with_failed_push("obj_4", 31)

    assert target.failed_pushes == (("obj_4", 17), ("obj_4", 31))


def test_recording_the_same_failure_twice_is_a_no_op():
    once = _target().with_failed_push("obj_4", 17)

    assert once.with_failed_push("obj_4", 17) is once


def test_push_attempts_advance_the_iteration_watermark():
    target = _target().with_push_attempted(5)

    assert target.physical_pushes_attempted == 1
    assert target.last_iteration == 5


def test_updates_do_not_mutate_the_original():
    original = _target()

    original.with_failed_push("obj_4", 17).with_push_attempted(2)

    assert original.failed_pushes == ()
    assert original.physical_pushes_attempted == 0


# --- validation and hand-off -------------------------------------------------

def test_a_target_without_points_has_no_criterion():
    with pytest.raises(ValueError, match="target_samples_m"):
        _target(target_samples_m=())


@pytest.mark.parametrize("bad", [-0.1, 1.5])
def test_out_of_range_fraction_is_rejected(bad):
    with pytest.raises(ValueError, match="open_fraction"):
        _target(open_fraction=bad)


def test_solve_kwargs_match_the_service_signature():
    kwargs = _target().as_solve_kwargs()

    assert kwargs == {
        "target_points": [(0.30, 0.40), (0.31, 0.40), (0.32, 0.41)],
        "blocking_objects": ["obj_4", "obj_7"],
    }


def test_freezing_a_selection_keeps_its_points_and_blockers():
    selection = type(
        "Sel", (), {"target_points_m": list(POINTS), "blocker_real_ids": list(BLOCKERS),
                    "region_path": ["robot", "goal"]}
    )()

    target = target_from_selection(selection, open_fraction=CANONICAL_FRACTION, iteration=2)

    assert target.target_samples_m == POINTS
    assert target.blocker_real_ids == BLOCKERS
    assert target.source_region_path == ("robot", "goal")
    assert target.target_id == fingerprint_samples(POINTS)
