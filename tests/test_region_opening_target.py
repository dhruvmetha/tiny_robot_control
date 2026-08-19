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
    assert target.target_id == "ro-0002"
