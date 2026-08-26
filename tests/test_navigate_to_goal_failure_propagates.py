"""A failed navigate-to-goal must make the planner replan, never sit silent.

Real logs, 2026-08-23 (real_trials/easy_002/trial2/run.log,
real_trials/hard_004/trial1/run.log): the executor reports a failed
NavigateSubgoal-to-goal, and NAMOPlanner.notify_subgoal_done() used to do
this:

    if self._navigating_to_goal:
        return

unconditionally -- for success AND failure alike. A failed arrival left
_navigating_to_goal True, no replan-attempt counter touched, and nothing
reset for the next plan() call. Combined with the NavigationController bug
(see test_navigation_failure_is_terminal.py) that meant the runtime never
even got this far; this test pins the planner's half of the fix on its own,
independent of the controller, since the controller fix alone only gets you
to "did_fail() now reports True" -- it's this method that decides what
happens next.

The fix: a failed navigate-to-goal clears _navigating_to_goal, increments
the same _replan_attempt budget push failures already use, and clears
_plan_generated so the next plan() call re-runs reachability (and the
goal-retarget check bug 2 adds) from scratch. Bounded by
_max_replan_attempts, same as every other failure path in this planner --
so a navigate that keeps failing eventually gives up (is_complete() ->
True) instead of looping forever.

No binding, no checkpoint, no hardware -- NAMOPlanBridge is faked out, the
same pattern test_namo_planner_chain_reuse.py uses.

To verify:
  cd robot_control && pytest tests/test_navigate_to_goal_failure_propagates.py -v
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path


def _load_module(module_name: str):
    """Import a robot_control module, standing in for optional dependencies.

    Mirrors conftest.py / test_namo_planner_chain_reuse.py's loader so this
    file collects the same way whether or not the package is pip-installed.
    """
    try:
        return importlib.import_module(module_name)
    except ImportError:
        pass

    root = Path(__file__).resolve().parents[1] / "src" / "robot_control"
    robot_control_pkg = sys.modules.get("robot_control")
    if robot_control_pkg is None:
        robot_control_pkg = types.ModuleType("robot_control")
        robot_control_pkg.__path__ = [str(root)]
        sys.modules["robot_control"] = robot_control_pkg

    for subpkg in ("core", "planner", "camera", "utils"):
        full_name = f"robot_control.{subpkg}"
        if full_name not in sys.modules:
            pkg = types.ModuleType(full_name)
            pkg.__path__ = [str(root / subpkg)]
            sys.modules[full_name] = pkg

    utils_pkg = sys.modules["robot_control.utils"]
    if not hasattr(utils_pkg, "NAMOXMLGenerator"):
        xml_generator_mod = importlib.import_module("robot_control.utils.xml_generator")
        utils_pkg.NAMOXMLGenerator = xml_generator_mod.NAMOXMLGenerator

    return importlib.import_module(module_name)


types_mod = _load_module("robot_control.core.types")
ObjectPose = types_mod.ObjectPose
Observation = types_mod.Observation
NavigateSubgoal = types_mod.NavigateSubgoal
planner_mod = _load_module("robot_control.planner.namo_planner")


class _FakeBridge:
    """Never actually asked to plan a push in these tests."""

    def __init__(self, *args, **kwargs):
        self.plan_calls = []

    def plan(self, **kwargs):
        self.plan_calls.append(kwargs)
        return []

    def analyze_reachability(self, **kwargs):
        return {"goal_reachable": True, "objects": {}}


def _obs() -> Observation:
    return Observation(
        robot_x=10.0, robot_y=10.0, robot_theta=90.0,
        objects={}, timestamp=0.0,
    )


def _make_planner(monkeypatch):
    monkeypatch.setattr(planner_mod, "NAMOPlanBridge", _FakeBridge)
    planner = planner_mod.NAMOPlanner(
        robot_goal_cm=(40.0, 40.0),
        namo_config_path="unused.yaml",
        execution_mode="mpc",
        verbose=False,
    )
    # Goal is nominally reachable and there is nothing to retarget to --
    # isolates this test to the propagation bug, independent of bug 2's
    # retarget selection (covered separately in test_goal_retarget.py).
    planner._is_goal_reachable = lambda obs: True
    planner._select_goal_retarget = lambda obs: None
    return planner


# ─── Tests ──────────────────────────────────────────────────────────────


def test_a_failed_navigate_to_goal_is_not_silently_swallowed(monkeypatch):
    planner = _make_planner(monkeypatch)
    obs = _obs()

    subgoal = planner.plan(obs)
    assert isinstance(subgoal, NavigateSubgoal)
    assert planner._navigating_to_goal is True
    assert planner._replan_attempt == 0

    planner.notify_subgoal_done(obs, failed=True)

    # The old code's early `if self._navigating_to_goal: return` left every
    # one of these untouched. Any of them staying at its pre-failure value
    # is the freeze.
    assert planner._navigating_to_goal is False
    assert planner._replan_attempt == 1
    assert planner._plan_generated is False
    assert planner.is_complete(obs) is False, "one failure must not give up early"


def test_the_planner_asks_for_a_fresh_subgoal_after_the_failure(monkeypatch):
    """Not just internal state -- plan() must actually hand back a next step."""
    planner = _make_planner(monkeypatch)
    obs = _obs()

    planner.plan(obs)
    planner.notify_subgoal_done(obs, failed=True)
    next_subgoal = planner.plan(obs)

    assert isinstance(next_subgoal, NavigateSubgoal)


def test_repeated_navigate_failures_give_up_instead_of_freezing(monkeypatch):
    """The bounded version of "must never silently loop".

    A navigate that keeps failing for reasons this planner can't fix (e.g.
    a persistently wrong wavefront disagreement) must not spin forever --
    it has to exhaust the same replan budget every other failure path uses
    and report the run complete (failed), so the runtime loop actually
    exits instead of dispatching subgoal #21.
    """
    planner = _make_planner(monkeypatch)
    obs = _obs()

    for _ in range(planner._max_replan_attempts):
        subgoal = planner.plan(obs)
        assert not planner.is_complete(obs), "gave up before exhausting the budget"
        assert isinstance(subgoal, NavigateSubgoal)
        planner.notify_subgoal_done(obs, failed=True)

    assert planner._planning_failed is True
    assert planner.is_complete(obs) is True


def test_a_successful_navigate_to_goal_is_unaffected(monkeypatch):
    """The success path this method also handles must not regress."""
    planner = _make_planner(monkeypatch)
    obs = _obs()

    planner.plan(obs)
    assert planner._navigating_to_goal is True

    planner.notify_subgoal_done(obs, failed=False)

    # Success is picked up by is_complete()'s distance check, not by this
    # call resetting anything -- _navigating_to_goal must stay True so the
    # next is_complete(obs) at the goal position reports done.
    assert planner._navigating_to_goal is True
    assert planner._replan_attempt == 0
