"""Planning metrics are aggregated while plan diagnostics are recorded."""

from robot_control.diagnostics import DiagnosticsRecorder


def test_planning_metrics_split_fresh_search_from_reuse(tmp_path):
    recorder = DiagnosticsRecorder(tmp_path / "run", verbose=False)

    recorder.record_plan({
        "planning_operation": "fresh_search",
        "planning_wall_time_ms": 11.5,
        "simulations_used": 5,
        "success": True,
    })
    recorder.record_plan({
        "planning_operation": "reuse_verification",
        "planning_wall_time_ms": 2.25,
        "simulations_used": 1,
        "success": False,
    })
    recorder.record_plan({
        "planning_operation": "decision_only",
        "planning_wall_time_ms": 0.0,
        "simulations_used": 0,
        "success": True,
    })

    assert recorder.planning == {
        "wall_time_ms_total": 13.75,
        "wall_time_ms_fresh_search": 11.5,
        "wall_time_ms_reuse_verification": 2.25,
        "simulations_used_total": 6,
        "simulations_used_fresh_search": 5,
        "simulations_used_reuse_verification": 1,
    }
    assert recorder.totals["plan_calls"] == 3
    recorder.close()


def test_disabled_recorder_keeps_zero_planning_metrics():
    recorder = DiagnosticsRecorder(verbose=False)

    recorder.record_plan({
        "planning_operation": "fresh_search",
        "planning_wall_time_ms": 10.0,
        "simulations_used": 4,
        "success": True,
    })

    assert recorder.planning == {
        "wall_time_ms_total": 0.0,
        "wall_time_ms_fresh_search": 0.0,
        "wall_time_ms_reuse_verification": 0.0,
        "simulations_used_total": 0,
        "simulations_used_fresh_search": 0,
        "simulations_used_reuse_verification": 0,
    }
