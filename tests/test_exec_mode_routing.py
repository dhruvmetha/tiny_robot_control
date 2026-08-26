"""The decision rule you asked for is the one that runs, or the run refuses to start.

``test_search_flag_routing`` covers the same class of bug one layer up: a search
selection forwarded into ``algorithm_params``, which namo_cpp treats as an opaque
bag, where a key no planner reads is dropped without a word. On 2026-08-21 that
cost a hardware run 2,959 primitives and 394 s with no ranker loaded, and it read
exactly like a hard scene.

Execution mode is the sharper version. A misrouted search flag makes a run slow,
which somebody eventually investigates. A misrouted exec mode makes a run
*normal*: it completes, it produces an outcome, and that outcome is filed under
an arm it never ran. With mode crossed against the model-versus-uniform arm there
are four ways to mislabel a row rather than two, and the analysis is paired
within scene, so one miscounted cell does not add noise, it breaks the test.

``mode`` is a named parameter of ``solve_boundary_from_xml``, and only the
held-boundary loop calls that method. The whole-problem path goes through
``plan_from_xml``, which does not name it. So the one route that silently drops a
mode is asking for one outside held mode, and that is the refusal that matters
here.

Six properties, each with the failure it catches:

  policy outside held mode refuses    the silent drop itself: the mode reaches
                                      plan_from_xml, which never reads it, and
                                      the run searches while the log says policy
  policy on region_bfs refuses        that planner sweeps every edge and depth
                                      and builds no ranked pool, so there is no
                                      argmax to take
  working combinations still pass     parametrized, so the guard cannot be
                                      tightened into blocking real runs
  the banner always names the mode    including the plain default. A person at
                                      the table should read which cell of the
                                      design they are filling off the console,
                                      not out of a command string afterwards
  the mode reaches the held path only  validating it and then failing to send
                                      it, or sending it down the path that
                                      drops it, both pass every check above
  both repositories spell it the same  a translation table between them would be
                                      one more place to write the wrong arm

Pure argument plumbing, so no checkpoint and no hardware. The last one needs
namo_cpp importable and sits out without it.

To verify:
  cd robot_control && pytest tests/test_exec_mode_routing.py -v
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from robot_control.core.types import Observation
from robot_control.planner import (
    DEFAULT_EXEC_MODE,
    EXEC_MODE_CHOICES,
    LocalSearchConfig,
    check_search_reaches_planner,
    describe_effective_search,
)
from robot_control.planner.namo_bridge import NAMOPlanBridge
from robot_control.planner.namo_planner import NAMOPlanner


# ─── Named constants ────────────────────────────────────────────────────

MODE_SEARCH = "search"
MODE_POLICY = "policy"

CKPT = "/models/HY5U_s2.ckpt"

SWEEP_ALGORITHM = "region_opening"
PRIMITIVE_STRATEGY = "primitive"

# The algorithm the existing guard lets best_first through on. On the held path
# --algorithm is not actually consulted -- solve_boundary_from_xml names
# local_search as its own parameter and the bridge renames the key for it -- but
# check_search_reaches_planner still refuses region_opening + best_first, which
# is territory test_search_flag_routing owns. These cases stay on the
# combination that guard permits rather than re-litigating it here.
BEST_FIRST_ALGORITHM = "full_namo"

HELD = True
WHOLE_PROBLEM = False

BASE_SEED = 4242


# ─── Helpers ────────────────────────────────────────────────────────────


def _reactive(**overrides):
    """A valid reactive selection: policy needs the ranked pool best_first builds."""
    return LocalSearchConfig(
        **{
            "local_search": "best_first",
            "best_first_prior": "model",
            "scorer_ckpt": CKPT,
            "exec_mode": MODE_POLICY,
            **overrides,
        }
    )


def _planner(**overrides):
    """A planner with only the attributes the kwargs builders read."""
    planner = NAMOPlanner.__new__(NAMOPlanner)
    attrs = dict(
        _goal_strategy=PRIMITIVE_STRATEGY,
        _max_chain_depth=2,
        _frontier_beam_width=10000,
        _chain_link_cost=11,
        _selection_strategy="cost_first",
        _goals_per_region=100,
        _shuffle_edges=True,
        _shuffle_seed=BASE_SEED,
        _rollout_samples_per_state=None,
        _ml_goal_model_path=None,
        _ml_device=None,
        _ml_samples=None,
        _ml_num_steps=None,
        _ml_sampler_method=None,
        _local_search=LocalSearchConfig(),
    )
    attrs.update(overrides)
    for name, value in attrs.items():
        setattr(planner, name, value)
    return planner


class _RecordingService:
    """Captures what the bridge hands the planning service."""

    def __init__(self):
        self.solve_kwargs = None

    def solve_boundary_from_xml(self, xml_path, robot_goal, **kwargs):
        self.solve_kwargs = kwargs
        return SimpleNamespace(
            success=False, already_open=False, boundary_exhausted=False,
            actions=[], failure_reason="none", resolved_target="",
            search_time_ms=1.0, simulations_used=0, target_summary=None,
        )


@pytest.fixture
def bridge_and_service(monkeypatch, tmp_path):
    bridge = NAMOPlanBridge(namo_config_path="unused.yaml", robot_model="car")
    service = _RecordingService()
    monkeypatch.setattr(bridge, "_get_planning_service", lambda: service)
    monkeypatch.setattr(bridge, "_generate_xml", lambda obs, goal: "<mujoco/>")
    xml = tmp_path / "scene.xml"
    xml.write_text("<mujoco/>")
    monkeypatch.setattr(bridge, "_write_xml", lambda content: str(xml))
    monkeypatch.setattr(bridge, "_debug_xml_path", str(xml))
    return bridge, service


def _obs():
    return Observation(
        robot_x=37.0, robot_y=67.0, robot_theta=90.0, objects={}, timestamp=0.0
    )


def _target():
    return SimpleNamespace(
        as_solve_kwargs=lambda: {"target_points": [(0.3, 0.4)], "blocking_objects": []},
        failed_pushes=(),
    )


# ─── Tests ──────────────────────────────────────────────────────────────


def test_policy_outside_held_mode_refuses():
    """The silent drop: plan_from_xml never reads the mode, so the run searches.

    The message names the way out, because a person reading only the refusal
    should not have to work out which flag turns the held loop on.
    """
    with pytest.raises(ValueError) as excinfo:
        check_search_reaches_planner(
            BEST_FIRST_ALGORITHM, PRIMITIVE_STRATEGY, _reactive(),
            held_boundary=WHOLE_PROBLEM,
        )

    assert "--hold-region-target" in str(excinfo.value)


def test_policy_on_the_sweeping_search_refuses():
    """No ranked pool means no argmax, so there is nothing for the policy to take."""
    with pytest.raises(ValueError) as excinfo:
        _reactive(local_search="region_bfs", best_first_prior="uniform", scorer_ckpt=None)

    assert "best_first" in str(excinfo.value)


@pytest.mark.parametrize("mode", ["reactive", "argmax", "polciy", "", "Policy"])
def test_an_unknown_mode_refuses(mode):
    """Anything outside the vocabulary is a mistake, not a synonym for the default."""
    with pytest.raises(ValueError, match="exec_mode"):
        LocalSearchConfig(exec_mode=mode)


@pytest.mark.parametrize(
    "algorithm,config,held",
    [
        (SWEEP_ALGORITHM, LocalSearchConfig(), WHOLE_PROBLEM),
        (SWEEP_ALGORITHM, LocalSearchConfig(), HELD),
        (SWEEP_ALGORITHM, LocalSearchConfig(exec_mode=MODE_SEARCH), HELD),
        (BEST_FIRST_ALGORITHM, _reactive(), HELD),
        (BEST_FIRST_ALGORITHM, _reactive(best_first_prior="uniform", scorer_ckpt=None), HELD),
    ],
)
def test_working_combinations_still_run(algorithm, config, held):
    """The guard must refuse mistakes, not real runs."""
    check_search_reaches_planner(
        algorithm, PRIMITIVE_STRATEGY, config, held_boundary=held
    )


@pytest.mark.parametrize(
    "algorithm,config,held",
    [
        (SWEEP_ALGORITHM, LocalSearchConfig(), WHOLE_PROBLEM),
        (SWEEP_ALGORITHM, LocalSearchConfig(), HELD),
        (BEST_FIRST_ALGORITHM, _reactive(), HELD),
    ],
)
def test_the_banner_always_names_the_mode(algorithm, config, held):
    """Silence here is what made a misrouted run look normal the first time."""
    line = describe_effective_search(
        algorithm, PRIMITIVE_STRATEGY, config, held_boundary=held
    )

    assert "exec mode:" in line
    expected = config.exec_mode if held else MODE_SEARCH
    assert f"exec mode: {expected}" in line


def test_the_mode_goes_down_the_held_path_and_not_the_other():
    """Sending it where it is dropped passes every check that only reads config."""
    planner = _planner(_local_search=_reactive())

    held = planner._held_mode_planner_kwargs()
    whole = planner._search_planner_kwargs(BASE_SEED)

    assert held["mode"] == MODE_POLICY
    assert "mode" not in whole, (
        "plan_from_xml does not name mode; sending it there drops it in silence"
    )


def test_the_bridge_forwards_the_mode_to_the_service(bridge_and_service):
    bridge, service = bridge_and_service

    bridge.solve_boundary(_obs(), (40.0, 40.0), _target(), mode=MODE_POLICY)

    assert service.solve_kwargs["mode"] == MODE_POLICY


def test_the_bridge_records_which_mode_ran(bridge_and_service):
    """Read back from the call that was made, so a run log needs no command parsing."""
    bridge, service = bridge_and_service

    bridge.solve_boundary(_obs(), (40.0, 40.0), _target(), mode=MODE_POLICY)

    assert bridge.last_algorithm_stats["exec_mode"] == MODE_POLICY


def test_a_run_that_never_names_a_mode_records_the_default(bridge_and_service):
    bridge, service = bridge_and_service

    bridge.solve_boundary(_obs(), (40.0, 40.0), _target())

    assert bridge.last_algorithm_stats["exec_mode"] == DEFAULT_EXEC_MODE


def test_both_repositories_spell_the_modes_the_same_way():
    """Forwarded verbatim, so the two vocabularies have to be one vocabulary.

    Cross-repo, so it needs namo_cpp importable. Without the binding the rest of
    this file still runs; only this contract check sits out.
    """
    pytest.importorskip("namo_rl", reason="needs the compiled namo_cpp binding")
    from namo.services.planning_service import BOUNDARY_MODES, DEFAULT_BOUNDARY_MODE

    assert set(EXEC_MODE_CHOICES) == set(BOUNDARY_MODES)
    assert DEFAULT_EXEC_MODE == DEFAULT_BOUNDARY_MODE
