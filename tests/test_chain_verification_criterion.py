"""What a simulated chain has to achieve to count as verified.

Chain reuse replays a previously-planned chain instead of searching again, and
accepts it if simulation says it still works. "Still works" used to mean one
thing only: the final state makes the robot's goal reachable.

While a boundary is held that is the wrong question. A chain can make the goal
reachable while abandoning the boundary being opened -- and accepting it would
silently undo the subproblem, which is the bug the held-boundary work exists to
prevent. That is why reuse was disabled in held mode; these tests are what let
it be switched back on.
"""

import math
from types import SimpleNamespace

import pytest

from robot_control.planner.namo_bridge import evaluate_chain_outcome
from robot_control.planner.region_target import RegionOpeningTarget

POINTS = [(0.30, 0.40), (0.31, 0.40), (0.32, 0.41), (0.33, 0.42), (0.34, 0.43)]


class _Env:
    """Answers the two post-chain questions independently, so they can disagree."""

    def __init__(self, goal_reachable, reachable_count=0):
        self._goal_reachable = goal_reachable
        self._reachable_count = reachable_count
        self.counted_against = None

    def is_robot_goal_reachable(self):
        return self._goal_reachable

    def count_reachable_points(self, points):
        self.counted_against = list(points)
        return self._reachable_count, 0


# --- no target: unchanged ----------------------------------------------------

@pytest.mark.parametrize("goal_reachable", [True, False])
def test_without_target_points_the_goal_decides(goal_reachable):
    env = _Env(goal_reachable=goal_reachable)

    succeeded, goal_after, target_after = evaluate_chain_outcome(env)

    assert succeeded is goal_reachable
    assert goal_after is goal_reachable
    assert target_after is None


def test_without_target_points_nothing_is_counted():
    env = _Env(goal_reachable=True)

    evaluate_chain_outcome(env)

    assert env.counted_against is None


# --- with a target: the boundary decides -------------------------------------

def test_a_chain_that_opens_neither_the_boundary_nor_the_goal_fails():
    """The reason reuse needed a boundary-aware criterion at all.

    Without it the check was "did the goal become reachable", which a chain can
    fail while still being the right chain for this subproblem, and vice versa.
    """
    env = _Env(goal_reachable=False, reachable_count=0)

    succeeded, goal_after, target_after = evaluate_chain_outcome(env, POINTS, min_reachable=1)

    assert succeeded is False
    assert target_after is False
    assert goal_after is False


def test_an_opened_boundary_verifies_even_if_the_goal_is_still_blocked():
    """Opening one boundary of several does not make the final goal reachable."""
    env = _Env(goal_reachable=False, reachable_count=len(POINTS))

    succeeded, goal_after, target_after = evaluate_chain_outcome(env, POINTS, min_reachable=1)

    assert succeeded is True
    assert target_after is True
    assert goal_after is False


def test_the_chain_is_graded_against_the_supplied_points():
    env = _Env(goal_reachable=False, reachable_count=5)

    evaluate_chain_outcome(env, POINTS, min_reachable=1)

    assert env.counted_against == POINTS


@pytest.mark.parametrize(
    "count,needed,expected", [(1, 2, False), (2, 2, True), (3, 2, True)]
)
def test_the_bar_is_a_threshold_not_a_boolean(count, needed, expected):
    env = _Env(goal_reachable=False, reachable_count=count)

    assert evaluate_chain_outcome(env, POINTS, min_reachable=needed)[0] is expected


def test_a_missing_bar_falls_back_to_one_point():
    env = _Env(goal_reachable=False, reachable_count=1)

    assert evaluate_chain_outcome(env, POINTS, min_reachable=None)[0] is True


# --- where the bar comes from ------------------------------------------------

def _target(n_points, fraction):
    return RegionOpeningTarget(
        target_samples_m=tuple((i / 100.0, 0.0) for i in range(n_points)),
        blocker_real_ids=("obj_4",),
        open_fraction=fraction,
    )


def test_the_canonical_bar_is_twenty_of_a_hundred():
    assert _target(100, 0.2).minimum_reachable() == 20


def test_the_bar_is_ceilinged_not_truncated():
    assert _target(11, 0.2).minimum_reachable() == math.ceil(0.2 * 11) == 3


def test_a_small_region_still_needs_one_point():
    assert _target(2, 0.2).minimum_reachable() == 1


def test_the_bar_comes_from_the_target_not_from_current_config():
    """A subproblem in flight must not be re-graded underneath itself."""
    lenient = _target(100, 0.05)

    assert lenient.minimum_reachable() == 5


# --- reaching the mission goal ends the problem ------------------------------

def test_reaching_the_mission_goal_counts_even_if_the_boundary_missed():
    """The goal is the terminal condition of the whole system.

    A push can re-partition free space and open a different route than the one
    being worked on. Rejecting that chain for missing the boundary's bar throws
    away a solution to the actual problem.
    """
    env = _Env(goal_reachable=True, reachable_count=0)

    succeeded, goal_after, target_after = evaluate_chain_outcome(env, POINTS, min_reachable=1)

    assert succeeded is True
    assert goal_after is True
    assert target_after is False


def test_the_boundary_still_counts_on_its_own():
    env = _Env(goal_reachable=False, reachable_count=len(POINTS))

    assert evaluate_chain_outcome(env, POINTS, min_reachable=1)[0] is True


def test_neither_means_failure():
    env = _Env(goal_reachable=False, reachable_count=0)

    assert evaluate_chain_outcome(env, POINTS, min_reachable=1)[0] is False
