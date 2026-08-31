"""Planning diagnostics keep simulation counts stable across planner paths."""

import json
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


def test_boundary_exhaustion_count_reaches_the_log():
    """A boundary the opener gave up on is where a policy fallback would fire.

    The allowlist drops what it does not name, so this scalar reached no trial
    row before it was added.
    """
    filtered = _filter_algorithm_stats_for_diagnostics(
        {"boundary_exhaustions": 1, "simulations_used": 132}
    )

    assert filtered["boundary_exhaustions"] == 1


def test_iteration_trace_survives_as_indexable_json():
    """Nested records keep their structure instead of becoming Python reprs.

    The previous coercion mapped ``str`` over list elements, so a list of dicts
    logged as a list of strings and no reader could ask which boundary was
    exhausted without re-parsing a repr.
    """
    trace = [
        {
            "iteration": 1,
            "chosen_target_region": "goal",
            "target_summary": {"boundary_exhausted": True},
            "rejection_breakdown": {"push_collided_with_wall": 100},
        },
        {"iteration": 2, "outcome": "opened_target"},
    ]

    filtered = _filter_algorithm_stats_for_diagnostics({"iteration_trace": trace})

    assert filtered["iteration_trace"][0]["chosen_target_region"] == "goal"
    assert filtered["iteration_trace"][0]["target_summary"]["boundary_exhausted"] is True
    assert filtered["iteration_trace"][0]["rejection_breakdown"]["push_collided_with_wall"] == 100
    assert filtered["iteration_trace"][1]["outcome"] == "opened_target"
    json.dumps(filtered)


def test_values_with_no_json_equivalent_still_degrade_to_strings():
    """C++ Action objects are why the coercion exists; keep that behaviour."""

    class _Opaque:
        def __repr__(self):
            return "<Action>"

    filtered = _filter_algorithm_stats_for_diagnostics(
        {"regions_opened": ["goal", _Opaque()]}
    )

    assert filtered["regions_opened"] == ["goal", "<Action>"]
    json.dumps(filtered)


def test_budget_rule_reaches_the_trial_row():
    """Two runs can report the same budget number and mean different things.

    namo_cpp names its constant per-keyhole while defaulting to one allowance
    shared across every boundary. Without the scope in the row, a reader has to
    guess which rule produced the number, and the constant's name points at the
    wrong guess.
    """
    filtered = _filter_algorithm_stats_for_diagnostics(
        {"simulation_budget_scope": "full_problem", "simulations_used": 900}
    )

    assert filtered["simulation_budget_scope"] == "full_problem"
