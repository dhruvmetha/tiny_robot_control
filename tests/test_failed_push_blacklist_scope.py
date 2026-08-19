"""Which blacklist entries survive a successful push.

When a push fails on the real robot the planner records (object_id, edge_idx)
so the next plan does not propose it again. edge_idx is in the object's BODY
frame, so once that object moves its own entries are meaningless -- edge 17 is
a different world-frame approach afterwards -- and they must be dropped.

The planner used to drop the entire set on any successful push, including
entries for objects that had not moved. That discarded the only failure memory
it has: with one push executed per replan, a push that physically failed could
be re-proposed on the very next plan, and the loop makes no progress.

These tests pin the narrower rule: drop the pushed object's entries, keep
everyone else's.
"""

from __future__ import annotations

from robot_control.core.types import Observation, PushSubgoal
from robot_control.planner import namo_planner as planner_mod

PUSHED = "obj_1"
UNTOUCHED = "obj_2"


class _FakeBridge:
    def __init__(self, *args, **kwargs):
        self.last_search_time_ms = 0.0
        self.last_algorithm_stats = {}
        self.last_xml_content = "<mujoco/>"

    def analyze_reachability(self, **kwargs):
        return {"goal_reachable": False, "objects": {}}


def _obs():
    return Observation(
        robot_x=0.0, robot_y=0.0, robot_theta=0.0, objects={}, timestamp=0.0
    )


def _planner(monkeypatch):
    monkeypatch.setattr(planner_mod, "NAMOPlanBridge", _FakeBridge)
    planner = planner_mod.NAMOPlanner(
        robot_goal_cm=(40.0, 40.0),
        namo_config_path="unused.yaml",
        execution_mode="mpc",
        verbose=False,
    )
    planner._is_goal_reachable = lambda obs: False
    return planner


def _succeed_push_on(planner, object_id):
    """Drive one successful push of `object_id` through notify_subgoal_done."""
    subgoal = PushSubgoal(object_id, 4, 1)
    planner._subgoals = [subgoal]
    planner._current_idx = 0
    planner._committed_chain = [subgoal]
    planner.notify_subgoal_done(_obs(), failed=False)


def test_pushed_objects_entries_are_dropped(monkeypatch):
    planner = _planner(monkeypatch)
    planner._failed_pushes = {(PUSHED, 4), (PUSHED, 17)}

    _succeed_push_on(planner, PUSHED)

    assert planner._failed_pushes == set()


def test_other_objects_entries_survive(monkeypatch):
    """The regression: these used to be wiped too."""
    planner = _planner(monkeypatch)
    planner._failed_pushes = {(PUSHED, 4), (UNTOUCHED, 9), (UNTOUCHED, 31)}

    _succeed_push_on(planner, PUSHED)

    assert planner._failed_pushes == {(UNTOUCHED, 9), (UNTOUCHED, 31)}


def test_a_failure_still_records_the_pair(monkeypatch):
    planner = _planner(monkeypatch)
    subgoal = PushSubgoal(UNTOUCHED, 22, 1)
    planner._subgoals = [subgoal]
    planner._current_idx = 0

    planner.notify_subgoal_done(_obs(), failed=True)

    assert (UNTOUCHED, 22) in planner._failed_pushes


def test_failures_accumulate_across_a_success_on_another_object(monkeypatch):
    """Two objects fail, a third push succeeds; both failures must remain."""
    planner = _planner(monkeypatch)
    planner._failed_pushes = {(UNTOUCHED, 9)}

    failed = PushSubgoal(UNTOUCHED, 31, 1)
    planner._subgoals = [failed]
    planner._current_idx = 0
    planner.notify_subgoal_done(_obs(), failed=True)

    _succeed_push_on(planner, PUSHED)

    assert planner._failed_pushes == {(UNTOUCHED, 9), (UNTOUCHED, 31)}


def test_reset_still_clears_everything(monkeypatch):
    planner = _planner(monkeypatch)
    planner._failed_pushes = {(PUSHED, 4), (UNTOUCHED, 9)}

    planner.reset()

    assert planner._failed_pushes == set()
