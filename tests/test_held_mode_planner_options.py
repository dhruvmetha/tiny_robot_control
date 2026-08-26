"""Holding a boundary must not search differently from planning the whole problem.

The two paths reach namo_cpp through different service methods. bridge.plan
names its options; solve_boundary forwards them as kwargs. Held mode built its
own shorter list of what to send, and anything it left out fell back to the
openers' defaults.

region_max_chain_depth is the one that mattered. Its default is 1, and at depth
1 there is no chain of a setup push followed by a finish push. That chain is the
only reason to hold a boundary across pushes, so held mode was quietly unable to
do the thing it exists for. The gap was invisible on the best_first opener,
whose depth is pinned separately at a canonical 2, and fatal on region_bfs,
which is the default.

These tests compare the two option sets rather than pinning a list, so an option
added to one path and forgotten on the other fails here.
"""

import pytest

from robot_control.planner.namo_planner import NAMOPlanner
from robot_control.planner.search_config import LocalSearchConfig

BASE_SEED = 4242

# Options the held path sends and the whole-problem path must not. `mode` is a
# named parameter of solve_boundary_from_xml and means nothing to plan_from_xml,
# where it would ride into algorithm_params and be dropped without a word. Pinned
# as a set so the exception stays deliberate: anything else appearing on one path
# and not the other still fails below.
HELD_ONLY_KEYS = {"mode"}


def _planner(**overrides):
    """A planner with only the attributes the kwargs builder reads."""
    planner = NAMOPlanner.__new__(NAMOPlanner)
    attrs = dict(
        _goal_strategy="primitive",
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


def test_held_mode_sends_the_same_options_as_the_whole_problem_path():
    """The regression, stated as the property rather than a list of keys.

    Held mode may add options the other path cannot receive, and must never
    drop or change one it can. Stated that way round so the original regression
    is still caught while the deliberate extras stay visible.
    """
    planner = _planner()

    whole = planner._search_planner_kwargs(BASE_SEED)
    held = planner._held_mode_planner_kwargs()

    assert set(whole) <= set(held)
    assert {key: held[key] for key in whole} == whole
    assert set(held) - set(whole) == HELD_ONLY_KEYS


def test_the_chain_depth_reaches_the_held_path():
    """At the openers' default of 1 a setup-then-finish chain cannot exist."""
    held = _planner(_max_chain_depth=2)._held_mode_planner_kwargs()

    assert held["max_chain_depth"] == 2


@pytest.mark.parametrize(
    "key,value",
    [
        ("frontier_beam_width", 25),
        ("chain_link_cost", 7),
        ("selection_strategy", "depth_first"),
        ("goals_per_region", 50),
        ("goal_strategy", "ml"),
    ],
)
def test_every_search_option_reaches_the_held_path(key, value):
    held = _planner(**{f"_{key}": value})._held_mode_planner_kwargs()

    assert held[key] == value


def test_the_opener_choice_reaches_the_held_path():
    """local_search decides which search runs; dropping it changes everything."""
    local = LocalSearchConfig(local_search="best_first", best_first_prior="uniform")

    held = _planner(_local_search=local)._held_mode_planner_kwargs()

    assert held["full_namo_local_search"] == "best_first"


def test_ml_inference_options_reach_the_held_path():
    """A held replan under --strategy ml must sample the way the first one did."""
    held = _planner(
        _ml_goal_model_path="/ckpt.pt",
        _ml_device="cuda",
        _ml_samples=8,
        _ml_num_steps=32,
        _ml_sampler_method="ddim",
    )._held_mode_planner_kwargs()

    assert held["ml_goal_model_path"] == "/ckpt.pt"
    assert held["ml_device"] == "cuda"
    assert (held["ml_samples"], held["ml_num_steps"]) == (8, 32)
    assert held["ml_sampler_method"] == "ddim"


def test_ml_options_stay_out_when_no_model_is_configured():
    held = _planner()._held_mode_planner_kwargs()

    assert "ml_goal_model_path" not in held
    assert "ml_samples" not in held


def test_held_mode_uses_the_base_seed_not_a_retry_seed():
    """The whole-problem path bumps the seed per retry; held mode re-solves."""
    planner = _planner()

    assert planner._held_mode_planner_kwargs()["shuffle_seed"] == BASE_SEED
    assert planner._search_planner_kwargs(99)["shuffle_seed"] == 99


def test_the_options_are_the_ones_the_service_accepts():
    """Named on both service methods, so neither needs the region_* keys.

    Cross-repo, so it needs namo_cpp importable. Without the binding the rest
    of this file still runs; only this contract check sits out.
    """
    import inspect

    pytest.importorskip("namo_rl", reason="needs the compiled namo_cpp binding")
    from namo.services import NAMOPlanningService

    accepted = set(
        inspect.signature(NAMOPlanningService.solve_boundary_from_xml).parameters
    )
    held = _planner()._held_mode_planner_kwargs()
    named = set(held) - {"shuffle_edges", "shuffle_seed"}

    assert named <= accepted, f"solve_boundary_from_xml cannot name {named - accepted}"
