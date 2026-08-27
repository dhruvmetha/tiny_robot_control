"""Planning diagnostics keep simulation counts stable across planner paths."""

from robot_control.planner.namo_planner import (
    _filter_algorithm_stats_for_diagnostics,
)


def test_whole_problem_budget_is_recorded_as_simulations_used():
    filtered = _filter_algorithm_stats_for_diagnostics(
        {
            "simulation_budget_scope": "keyhole",
            "simulation_budget_used_total": 6,
            "total_primitives_attempted": 6,
        }
    )

    assert filtered["simulations_used"] == 6
    assert filtered["total_primitives_attempted"] == 6


def test_held_boundary_simulation_count_remains_authoritative():
    filtered = _filter_algorithm_stats_for_diagnostics(
        {
            "simulations_used": 4,
            "simulation_budget_used_total": 99,
            "total_primitives_attempted": 99,
        }
    )

    assert filtered["simulations_used"] == 4
