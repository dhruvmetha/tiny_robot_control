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
from types import SimpleNamespace

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


# --- counting the pushes actually spent on this boundary ---------------------
#
# with_push_attempted existed with no production caller, so
# physical_pushes_attempted stayed at 0 and last_iteration stayed at the
# iteration the target was created in, for the whole life of the subproblem.
# Both get read back from disk by the next process.

def _planner_with_target(target, plan_count):
    from robot_control.planner.namo_planner import NAMOPlanner

    planner = NAMOPlanner.__new__(NAMOPlanner)
    planner._active_target = target
    planner._plan_count = plan_count
    planner._active_target_path = None
    planner._verbose = False
    planner._execution_mode = "mpc"
    planner._observation_at_commit = None
    planner._committed_chain = []
    planner._committed_chain_origin = None
    planner._subgoals = []
    planner._current_idx = 0
    planner._plan_generated = False
    planner._pending_reuse_chain = None
    planner._pending_reuse_origin = None
    planner._copy_push_chain = list
    planner._emit_sim_capture_for_plan = lambda **kw: None
    return planner


def _commit_one_push(planner, chain_length=1):
    chain = [
        SimpleNamespace(object_id=PUSHED, edge_idx=6 + i, push_steps=2)
        for i in range(chain_length)
    ]
    planner._queue_mpc_chain(
        obs=SimpleNamespace(), chain=chain, origin="region_target", attempt_index=0
    )


def test_committing_a_push_counts_it_against_the_held_boundary():
    planner = _planner_with_target(_target(), plan_count=3)

    _commit_one_push(planner)

    assert planner._active_target.physical_pushes_attempted == 1
    assert planner._active_target.last_iteration == 3


def test_the_count_accumulates_across_pushes():
    """A two-push boundary must read as two, which is what a cap would use."""
    planner = _planner_with_target(_target(), plan_count=1)

    _commit_one_push(planner)
    planner._plan_count = 2
    _commit_one_push(planner)

    assert planner._active_target.physical_pushes_attempted == 2
    assert planner._active_target.last_iteration == 2


def test_a_multi_push_chain_still_counts_one_push():
    """MPC queues only the first push of a chain, so only one is attempted."""
    planner = _planner_with_target(_target(), plan_count=1)

    _commit_one_push(planner, chain_length=3)

    assert planner._active_target.physical_pushes_attempted == 1


def test_committing_without_a_held_boundary_is_fine():
    planner = _planner_with_target(None, plan_count=1)

    _commit_one_push(planner)

    assert planner._active_target is None


def test_the_counters_survive_a_save_and_load(tmp_path):
    planner = _planner_with_target(_target(), plan_count=4)
    _commit_one_push(planner)

    path = tmp_path / ACTIVE_TARGET_FILENAME
    planner._active_target.save(path)

    revived = RegionOpeningTarget.load(path)
    assert revived.physical_pushes_attempted == 1
    assert revived.last_iteration == 4


# --- the same bookkeeping on the path that drives the robot -------------------
#
# NAMOPlanner does all of this in-process, and the session workflow never
# constructs it: closed_loop_session spawns run_namo per replan. So both halves
# were missing on the only path that touches real hardware.

def _scene(dir_path, poses):
    dir_path.mkdir(parents=True, exist_ok=True)
    rec = {
        "robot_pose_cm": [0.0, 0.0, 0.0],
        "objects": {
            name: {"x_cm": x, "y_cm": y, "theta_deg": t} for name, (x, y, t) in poses.items()
        },
    }
    (dir_path / "mid_obs.jsonl").write_text(json.dumps(rec) + "\n", encoding="utf-8")
    return dir_path


def _settle(tmp_path, target, before, after, iteration=2):
    run_dir = tmp_path / "run"
    run_dir.mkdir(parents=True, exist_ok=True)
    target.save(run_dir / ACTIVE_TARGET_FILENAME)
    result = closed_loop._settle_target_after_push(
        run_dir,
        iteration,
        _scene(tmp_path / "before", before),
        _scene(tmp_path / "after", after),
    )
    return RegionOpeningTarget.load(run_dir / ACTIVE_TARGET_FILENAME), result


STILL = {PUSHED: (10.0, 20.0, 0.0), UNTOUCHED: (30.0, 40.0, 0.0)}


def test_a_moved_object_loses_its_exclusions_on_the_real_path(tmp_path):
    """The regression: only the in-process planner used to do this."""
    after = dict(STILL, **{PUSHED: (10.0, 26.0, 0.0)})

    revived, result = _settle(
        tmp_path, _target(failed_pushes=((PUSHED, 17), (UNTOUCHED, 5))), STILL, after
    )

    assert revived.failed_pushes == ((UNTOUCHED, 5),)
    assert result["moved_objects"] == [PUSHED]
    assert result["forgotten_pushes"] == [[PUSHED, 17]]


def test_a_shoved_neighbour_loses_its_exclusions_too(tmp_path):
    """A push moves more than its target, and their edges are just as stale."""
    after = {PUSHED: (10.0, 26.0, 0.0), UNTOUCHED: (30.0, 47.0, 0.0)}

    revived, _ = _settle(
        tmp_path, _target(failed_pushes=((PUSHED, 17), (UNTOUCHED, 5))), STILL, after
    )

    assert revived.failed_pushes == ()


def test_marker_jitter_does_not_clear_the_exclusions(tmp_path):
    """Below the tolerance nothing moved, and forgetting here loses real memory."""
    after = {PUSHED: (10.2, 20.2, 2.0), UNTOUCHED: (30.0, 40.0, 0.0)}

    revived, result = _settle(
        tmp_path, _target(failed_pushes=((PUSHED, 17),)), STILL, after
    )

    assert revived.failed_pushes == ((PUSHED, 17),)
    assert result["moved_objects"] == []


def test_the_real_path_counts_the_push(tmp_path):
    revived, _ = _settle(tmp_path, _target(), STILL, STILL, iteration=4)

    assert revived.physical_pushes_attempted == 1
    assert revived.last_iteration == 4


def test_the_count_advances_even_when_nothing_moved(tmp_path):
    """A push that shifted nothing still cost an attempt."""
    revived, _ = _settle(tmp_path, _target(), STILL, STILL)

    assert revived.physical_pushes_attempted == 1


def test_settling_without_a_held_boundary_is_fine(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()

    result = closed_loop._settle_target_after_push(
        run_dir, 1, _scene(tmp_path / "b", STILL), _scene(tmp_path / "a", STILL)
    )

    assert result == {"moved_objects": None, "forgotten_pushes": None}


def test_a_missing_scene_capture_forgets_nothing(tmp_path):
    """No comparison possible, so keep the memory rather than guessing."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _target(failed_pushes=((PUSHED, 17),)).save(run_dir / ACTIVE_TARGET_FILENAME)

    closed_loop._settle_target_after_push(
        run_dir, 1, tmp_path / "absent", _scene(tmp_path / "a", STILL)
    )

    revived = RegionOpeningTarget.load(run_dir / ACTIVE_TARGET_FILENAME)
    assert revived.failed_pushes == ((PUSHED, 17),)
    assert revived.physical_pushes_attempted == 1


# --- what counts as movement -------------------------------------------------
#
# objects_that_moved claimed to mirror closed_loop_session's scene-to-XML
# matcher and did not. It compared each axis on its own instead of the straight
# line, and subtracted headings without wrapping. Both thresholds were right;
# the arithmetic under them was not, and the two errors pull opposite ways.

from robot_control.planner.region_target import (  # noqa: E402
    OBJECT_MOVED_TOLERANCE_CM,
    OBJECT_ROTATED_TOLERANCE_DEG,
    angle_diff_deg,
    objects_that_moved,
    pose_delta,
)

AT_REST = {PUSHED: (10.0, 20.0, 0.0)}


def _moved(x, y, theta, start=(10.0, 20.0, 0.0)):
    return objects_that_moved({PUSHED: start}, {PUSHED: (x, y, theta)})


@pytest.mark.parametrize("before,after", [(359.0, 1.0), (1.0, 359.0)])
def test_a_heading_crossing_zero_is_not_a_rotation(before, after):
    """Two degrees, not 358. Unwrapped, this cleared valid failure memory."""
    assert _moved(10.0, 20.0, after, start=(10.0, 20.0, before)) == set()


def test_a_real_rotation_still_counts():
    assert _moved(10.0, 20.0, 6.0) == {PUSHED}


def test_a_rotation_at_the_threshold_does_not_count():
    """Strict >, so exactly the tolerance is still at rest."""
    assert _moved(10.0, 20.0, OBJECT_ROTATED_TOLERANCE_DEG) == set()


def test_a_diagonal_shove_counts_even_when_each_axis_is_small():
    """0.4 on each axis is 0.57 cm of real movement. Per-axis called it unmoved."""
    assert _moved(10.4, 20.4, 0.0) == {PUSHED}


def test_a_move_of_exactly_the_tolerance_does_not_count():
    """(0.3, 0.4) is 0.5 cm on the nose, and the boundary stays exclusive."""
    assert _moved(10.3, 20.4, 0.0) == set()


def test_jitter_below_the_tolerance_does_not_count():
    assert _moved(10.2, 20.2, 2.0) == set()


@pytest.mark.parametrize("a,b,expected", [(359.0, 1.0, 2.0), (0.0, 180.0, 180.0),
                                          (90.0, 0.0, 90.0), (-179.0, 179.0, 2.0)])
def test_the_angle_difference_wraps(a, b, expected):
    assert angle_diff_deg(a, b) == pytest.approx(expected)


def test_the_pose_delta_is_a_straight_line_not_two_axes():
    distance, turn = pose_delta((10.4, 20.4, 359.0), (10.0, 20.0, 1.0))

    assert distance == pytest.approx(0.5657, abs=1e-3)
    assert turn == pytest.approx(2.0)


def test_the_settlement_clears_a_diagonally_shoved_object(tmp_path):
    """The whole point: a shove the old rule missed kept a stale exclusion."""
    revived, result = _settle(
        tmp_path,
        _target(failed_pushes=((PUSHED, 17),)),
        AT_REST,
        {PUSHED: (10.4, 20.4, 0.0)},
    )

    assert revived.failed_pushes == ()
    assert result["moved_objects"] == [PUSHED]


def test_the_settlement_keeps_memory_across_a_zero_crossing(tmp_path):
    """And a heading wrap the old rule read as a full turn wiped a good one."""
    revived, _ = _settle(
        tmp_path,
        _target(failed_pushes=((PUSHED, 17),)),
        {PUSHED: (10.0, 20.0, 359.0)},
        {PUSHED: (10.0, 20.0, 1.0)},
    )

    assert revived.failed_pushes == ((PUSHED, 17),)
