"""The search you asked for is the search that runs, or the run refuses to start.

Selecting a local search means forwarding keys into ``algorithm_params``, which
namo_cpp treats as an opaque bag. A key no planner reads is dropped without a
word. Only ``full_namo`` reads ``full_namo_local_search``
(namo_cpp full_namo_planner.py:125); ``region_opening`` runs its own exhaustive
sweep over every edge and depth and ignores it, taking a ranker only through
``goal_strategy="scorer"`` (namo_cpp region_opening.py:670).

That gap is not theoretical. On 2026-08-21 a hardware run was launched with
``--algorithm region_opening --local-search best_first --scorer-ckpt HY5U_s2``
and every one of those flags was discarded. The search swept 2,959 primitives
in 394 s with no ranker loaded, which reads exactly like a hard scene rather
than a misrouted flag. The same scene through ``solve_boundary_from_xml``,
which does honour best-first, solved in 0.6 s.

So two properties, and each catches that failure:

  a misrouted selection refuses    passing best_first to a planner that cannot
                                   read it fails before the robot moves, and
                                   the message names the two ways out
  every run says what it ran       including the plain default, because a log
                                   that does not record which search produced
                                   a push cannot be diagnosed afterwards

Pure argument plumbing, so no binding, no checkpoint, no hardware.

To verify:
  cd robot_control && pytest tests/test_search_flag_routing.py -v
"""

from __future__ import annotations

import pytest

from robot_control.planner.search_config import (
    BEST_FIRST_AWARE_ALGORITHMS,
    SCORER_GOAL_STRATEGY,
    LocalSearchConfig,
    check_search_reaches_planner,
    describe_effective_search,
)

# ─── Named constants ────────────────────────────────────────────────────

# The planner the misrouted run used. Not in BEST_FIRST_AWARE_ALGORITHMS.
SWEEP_ALGORITHM = "region_opening"
# The goal strategy that leaves region_opening without a ranker.
UNRANKED_STRATEGY = "primitive"
# Stand-in for the deployed checkpoint. Never loaded, so the path need not exist.
CKPT = "/models/HY5U_s2.ckpt"

BEST_FIRST = LocalSearchConfig(
    local_search="best_first", best_first_prior="model", scorer_ckpt=CKPT
)
DEFAULT_SEARCH = LocalSearchConfig()


# ─── A misrouted selection refuses ──────────────────────────────────────


def test_best_first_aimed_at_a_planner_that_ignores_it_is_refused():
    """The exact flag combination from the 2026-08-21 run."""
    with pytest.raises(ValueError) as exc:
        check_search_reaches_planner(SWEEP_ALGORITHM, UNRANKED_STRATEGY, BEST_FIRST)

    message = str(exc.value)
    assert "no effect" in message
    # Both escape routes named, or the error tells you nothing actionable.
    assert "full_namo" in message
    assert f"--strategy {SCORER_GOAL_STRATEGY}" in message


def test_a_checkpoint_nothing_will_load_is_refused():
    """--scorer-ckpt on the sweep planner without the scorer strategy."""
    unused_ckpt = LocalSearchConfig(scorer_ckpt=CKPT)

    with pytest.raises(ValueError, match="never loaded"):
        check_search_reaches_planner(SWEEP_ALGORITHM, UNRANKED_STRATEGY, unused_ckpt)


@pytest.mark.parametrize("algorithm", BEST_FIRST_AWARE_ALGORITHMS)
def test_best_first_passes_on_the_planner_that_reads_it(algorithm):
    check_search_reaches_planner(algorithm, UNRANKED_STRATEGY, BEST_FIRST)


@pytest.mark.parametrize(
    "algorithm,strategy,config",
    [
        (SWEEP_ALGORITHM, UNRANKED_STRATEGY, DEFAULT_SEARCH),
        (SWEEP_ALGORITHM, SCORER_GOAL_STRATEGY, LocalSearchConfig(scorer_ckpt=CKPT)),
    ],
    ids=["sweep-with-no-ranker", "sweep-with-scorer-strategy"],
)
def test_combinations_that_actually_work_are_allowed(algorithm, strategy, config):
    check_search_reaches_planner(algorithm, strategy, config)


# ─── Every run says what it ran ─────────────────────────────────────────


def test_the_sweep_planner_admits_it_has_no_ranker():
    line = describe_effective_search(SWEEP_ALGORITHM, UNRANKED_STRATEGY, DEFAULT_SEARCH)

    assert SWEEP_ALGORITHM in line
    assert "exhaustive sweep" in line
    assert "no ranker" in line


def test_the_sweep_planner_names_the_checkpoint_when_it_has_one():
    line = describe_effective_search(
        SWEEP_ALGORITHM, SCORER_GOAL_STRATEGY, LocalSearchConfig(scorer_ckpt=CKPT)
    )

    assert CKPT in line
    assert "no ranker" not in line


def test_best_first_reports_its_search_and_checkpoint():
    line = describe_effective_search("full_namo", UNRANKED_STRATEGY, BEST_FIRST)

    assert "full_namo" in line
    assert "best_first/model" in line
    assert CKPT in line


@pytest.mark.parametrize(
    "algorithm,strategy,config",
    [
        (SWEEP_ALGORITHM, UNRANKED_STRATEGY, DEFAULT_SEARCH),
        (SWEEP_ALGORITHM, SCORER_GOAL_STRATEGY, LocalSearchConfig(scorer_ckpt=CKPT)),
        ("full_namo", UNRANKED_STRATEGY, DEFAULT_SEARCH),
        ("full_namo", UNRANKED_STRATEGY, BEST_FIRST),
    ],
)
def test_the_banner_always_names_the_planner(algorithm, strategy, config):
    """Silence on the default is what let the misrouted run look normal."""
    line = describe_effective_search(algorithm, strategy, config)

    assert line.startswith(f"planner: {algorithm}")
