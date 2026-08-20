"""What the held boundary remembers about failed pushes, and what it forgets.

Two halves of one problem, and each was missing its half.

A push that fails physically has to be written down, or the next plan proposes
it again. The closed-loop driver replans in the same iteration after an
unreachable approach, reusing the run's shuffle seed against the same target, so
a deterministic strategy re-derives the identical approach. The target file is
the only state that crosses those two processes.

A push recorded before its object moved has to be erased. An edge index names a
contact point in the object's own body frame, so once the object shifts, the
exclusion points somewhere else in the world. NAMOPlanner already pruned its
in-memory blacklist that way; the persisted copy kept the stale entries and fed
them back, which can hide the finish push a setup push just made available.
"""

import json
import sys
from pathlib import Path

import pytest

from robot_control.planner.region_target import (
    ACTIVE_TARGET_FILENAME,
    RegionOpeningTarget,
)

# closed_loop_session is a script, not a package module. tests/ and scripts/ are
# siblings, so this needs no external PYTHONPATH.
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import closed_loop_session as closed_loop  # noqa: E402 - needs the path above

POINTS = ((0.30, 0.40), (0.31, 0.40))
CANONICAL_FRACTION = 0.2
PUSHED = "obj_4"
UNTOUCHED = "obj_9"


def _target(**overrides):
    kwargs = dict(
        target_samples_m=POINTS,
        blocker_real_ids=(PUSHED, UNTOUCHED),
        open_fraction=CANONICAL_FRACTION,
        target_id="ro-test",
    )
    kwargs.update(overrides)
    return RegionOpeningTarget(**kwargs)


# --- forgetting what stopped being true --------------------------------------

def test_a_moved_object_loses_its_recorded_failures():
    target = _target(failed_pushes=((PUSHED, 17), (PUSHED, 23)))

    pruned = target.forgetting_moved([PUSHED])

    assert pruned.failed_pushes == ()


def test_objects_that_did_not_move_keep_theirs():
    """Their body frames are unchanged, so those pushes are still unavailable."""
    target = _target(failed_pushes=((PUSHED, 17), (UNTOUCHED, 5)))

    pruned = target.forgetting_moved([PUSHED])

    assert pruned.failed_pushes == ((UNTOUCHED, 5),)


def test_forgetting_nothing_returns_the_same_record():
    """Frozen dataclass, so an identity check is how a caller skips a rewrite."""
    target = _target(failed_pushes=((UNTOUCHED, 5),))

    assert target.forgetting_moved([PUSHED]) is target
    assert target.forgetting_moved([]) is target


def test_forgetting_leaves_the_rest_of_the_record_alone():
    target = _target(failed_pushes=((PUSHED, 17),), physical_pushes_attempted=3)

    pruned = target.forgetting_moved([PUSHED])

    assert pruned.target_id == target.target_id
    assert pruned.target_samples_m == POINTS
    assert pruned.physical_pushes_attempted == 3


def test_a_pruned_record_round_trips(tmp_path):
    path = tmp_path / ACTIVE_TARGET_FILENAME
    _target(failed_pushes=((PUSHED, 17), (UNTOUCHED, 5))).forgetting_moved(
        [PUSHED]
    ).save(path)

    assert RegionOpeningTarget.load(path).failed_pushes == ((UNTOUCHED, 5),)


# --- writing down what just failed -------------------------------------------

def _run_dir(tmp_path, target=None):
    run_dir = tmp_path / "run"
    (run_dir / "real_push").mkdir(parents=True)
    if target is not None:
        target.save(run_dir / ACTIVE_TARGET_FILENAME)
    return run_dir


def _dispatch(run_dir, **over):
    rec = {"subgoal_id": 1, "type": "push", "object_id": PUSHED, "edge_idx": 6}
    rec.update(over)
    (run_dir / "real_push" / "subgoals.jsonl").write_text(
        json.dumps(rec) + "\n", encoding="utf-8"
    )


def test_an_unreachable_approach_is_excluded_from_the_next_plan(tmp_path):
    """The regression: the same seed and target would re-derive this push."""
    run_dir = _run_dir(tmp_path, _target())
    _dispatch(run_dir)

    pair = closed_loop._record_failed_push_on_target(run_dir, run_dir / "real_push")

    assert pair == (PUSHED, 6)
    revived = RegionOpeningTarget.load(run_dir / ACTIVE_TARGET_FILENAME)
    assert revived.failed_pushes == ((PUSHED, 6),)


def test_the_exclusion_joins_the_ones_already_recorded(tmp_path):
    run_dir = _run_dir(tmp_path, _target(failed_pushes=((UNTOUCHED, 5),)))
    _dispatch(run_dir)

    closed_loop._record_failed_push_on_target(run_dir, run_dir / "real_push")

    revived = RegionOpeningTarget.load(run_dir / ACTIVE_TARGET_FILENAME)
    assert set(revived.failed_pushes) == {(UNTOUCHED, 5), (PUSHED, 6)}


def test_recording_the_same_failure_twice_changes_nothing(tmp_path):
    run_dir = _run_dir(tmp_path, _target(failed_pushes=((PUSHED, 6),)))
    _dispatch(run_dir)

    pair = closed_loop._record_failed_push_on_target(run_dir, run_dir / "real_push")

    assert pair == (PUSHED, 6)
    assert RegionOpeningTarget.load(
        run_dir / ACTIVE_TARGET_FILENAME
    ).failed_pushes == ((PUSHED, 6),)


def test_the_last_dispatched_push_is_the_one_recorded(tmp_path):
    """An iteration can dispatch more than one subgoal; the failure is the last."""
    run_dir = _run_dir(tmp_path, _target())
    lines = [
        {"subgoal_id": 1, "type": "push", "object_id": UNTOUCHED, "edge_idx": 2},
        {"subgoal_id": 2, "type": "push", "object_id": PUSHED, "edge_idx": 6},
    ]
    (run_dir / "real_push" / "subgoals.jsonl").write_text(
        "\n".join(json.dumps(r) for r in lines) + "\n", encoding="utf-8"
    )

    assert closed_loop._record_failed_push_on_target(
        run_dir, run_dir / "real_push"
    ) == (PUSHED, 6)


def test_a_navigate_subgoal_is_not_a_push_to_exclude(tmp_path):
    run_dir = _run_dir(tmp_path, _target())
    (run_dir / "real_push" / "subgoals.jsonl").write_text(
        json.dumps({"subgoal_id": 1, "type": "navigate"}) + "\n", encoding="utf-8"
    )

    assert closed_loop._record_failed_push_on_target(run_dir, run_dir / "real_push") is None


@pytest.mark.parametrize("missing", ["target", "subgoals"])
def test_nothing_to_record_against_is_not_an_error(tmp_path, missing):
    run_dir = _run_dir(tmp_path, None if missing == "target" else _target())
    if missing != "subgoals":
        _dispatch(run_dir)

    assert closed_loop._record_failed_push_on_target(run_dir, run_dir / "real_push") is None
