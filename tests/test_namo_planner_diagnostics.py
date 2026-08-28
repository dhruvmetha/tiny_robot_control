"""Planning diagnostics keep simulation counts stable across planner paths."""

from types import SimpleNamespace

from robot_control.planner.namo_planner import (
    NAMOPlanner,
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


def test_fresh_plan_record_has_outer_time_and_top_level_simulation_count():
    records = []
    planner = NAMOPlanner.__new__(NAMOPlanner)
    planner._diag = SimpleNamespace(record_plan=records.append)
    planner._bridge = SimpleNamespace(
        last_search_time_ms=6.25,
        last_algorithm_stats={"simulation_budget_used_total": 5},
    )
    planner._total_planning_ms = 6.25
    planner._failed_pushes = set()
    obs = SimpleNamespace(
        robot_x=10.0,
        robot_y=20.0,
        robot_theta=90.0,
        objects={},
    )

    planner._record_plan_diagnostics(
        obs,
        attempt_index=1,
        attempt_seed=None,
        subgoals=[],
        success=False,
        planning_wall_time_ms=7.5,
        extra={"origin": "fresh_plan"},
    )

    assert records == [{
        "attempt_index": 1,
        "attempt_seed": None,
        "search_time_ms": 6.25,
        "cumulative_ms": 6.25,
        "planning_operation": "fresh_search",
        "planning_wall_time_ms": 7.5,
        "simulations_used": 5,
        "model_warmup_ms": 0.0,
        "model_warmup_excluded_from_planning_time": False,
        "success": False,
        "subgoals_returned": 0,
        "first_subgoal": None,
        "blacklist_size_before": 0,
        "algorithm_stats": {"simulations_used": 5},
        "robot_pose_cm": [10.0, 20.0, 90.0],
        "object_poses_cm": {},
        "origin": "fresh_plan",
    }]
