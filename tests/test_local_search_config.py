"""Selecting namo_cpp's local search from robot_control.

`full_namo` hands one region boundary at a time to a local search. Until now
robot_control could only ever get the default (`region_bfs`): the kwargs road
existed end to end, but no flag drove on it. These tests pin the mapping from
CLI options to the keys `plan_from_xml` forwards into `algorithm_params`.

The important property is that the default forwards *nothing*, so a run that
does not ask for best-first is byte-identical to before the option existed.
"""

from types import SimpleNamespace

import pytest

from robot_control.planner import (
    DEFAULT_BEST_FIRST_PRIOR,
    DEFAULT_LOCAL_SEARCH,
    LocalSearchConfig,
)

CKPT = "/models/HY5U_s1.ckpt"


def _args(**overrides):
    base = dict(
        local_search=DEFAULT_LOCAL_SEARCH,
        best_first_prior=DEFAULT_BEST_FIRST_PRIOR,
        scorer_ckpt=None,
        best_first_hmax=None,
        keyhole_simulation_budget=None,
        ml_device="cpu",
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def test_default_forwards_nothing():
    """An unchanged run must reach the planner exactly as it did before."""
    assert LocalSearchConfig().as_planner_kwargs() == {}


def test_default_is_region_bfs():
    cfg = LocalSearchConfig()

    assert cfg.local_search == "region_bfs"
    assert not cfg.uses_best_first
    assert not cfg.uses_ranker


def test_uniform_best_first_needs_no_checkpoint():
    cfg = LocalSearchConfig(local_search="best_first", best_first_prior="uniform")

    assert cfg.as_planner_kwargs() == {
        "full_namo_local_search": "best_first",
        "best_first_prior": "uniform",
    }
    assert cfg.uses_best_first and not cfg.uses_ranker


def test_ranker_without_a_checkpoint_fails_before_the_robot_moves():
    with pytest.raises(ValueError, match="scorer checkpoint"):
        LocalSearchConfig(local_search="best_first", best_first_prior="model")


def test_ranker_forwards_its_checkpoint():
    cfg = LocalSearchConfig(
        local_search="best_first", best_first_prior="model", scorer_ckpt=CKPT
    )

    assert cfg.as_planner_kwargs()["scorer_ckpt"] == CKPT
    assert cfg.uses_ranker


def test_optional_protocol_keys_are_omitted_unless_set():
    """Omitted means 'use namo_cpp's canonical value', not 'use ours'."""
    kwargs = LocalSearchConfig(
        local_search="best_first", best_first_prior="uniform"
    ).as_planner_kwargs()

    assert "best_first_hmax" not in kwargs
    assert "full_namo_keyhole_simulation_budget" not in kwargs


def test_protocol_keys_are_forwarded_when_set():
    kwargs = LocalSearchConfig(
        local_search="best_first",
        best_first_prior="uniform",
        best_first_hmax=2,
        keyhole_simulation_budget=900,
    ).as_planner_kwargs()

    assert kwargs["best_first_hmax"] == 2
    assert kwargs["full_namo_keyhole_simulation_budget"] == 900


@pytest.mark.parametrize(
    "kwargs,match",
    [
        ({"local_search": "beam"}, "local_search"),
        ({"local_search": "best_first", "best_first_prior": "greedy"}, "best_first_prior"),
        ({"local_search": "best_first", "best_first_prior": "uniform", "best_first_hmax": 0}, "hmax"),
        (
            {"local_search": "best_first", "best_first_prior": "uniform", "keyhole_simulation_budget": 0},
            "budget",
        ),
    ],
)
def test_invalid_selections_are_rejected(kwargs, match):
    with pytest.raises(ValueError, match=match):
        LocalSearchConfig(**kwargs)


def test_describe_names_the_search_for_the_run_log():
    assert "region_bfs" in LocalSearchConfig().describe()

    ranked = LocalSearchConfig(
        local_search="best_first", best_first_prior="model", scorer_ckpt=CKPT
    ).describe()
    assert "best_first/model" in ranked
    assert CKPT in ranked
    assert "canonical" in ranked  # hmax/budget deferred to namo_cpp


def test_args_mapping_round_trips():
    from importlib import import_module
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    run_namo = import_module("run_namo")

    cfg = run_namo.local_search_from_args(
        _args(local_search="best_first", best_first_prior="uniform", best_first_hmax=2)
    )

    assert cfg.as_planner_kwargs() == {
        "full_namo_local_search": "best_first",
        "best_first_prior": "uniform",
        "best_first_hmax": 2,
        "ml_device": "cpu",
    }


@pytest.mark.parametrize(
    "scale_factor,robot_model",
    [(6.0, "car"), (1.0, "sphere"), (6.0, "sphere")],
)
def test_run_namo_rejects_every_noncanonical_robot_profile(
    scale_factor, robot_model
):
    from importlib import import_module
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    run_namo = import_module("run_namo")

    with pytest.raises(ValueError, match="only --robot-model car"):
        run_namo.find_namo_config(scale_factor, robot_model)
