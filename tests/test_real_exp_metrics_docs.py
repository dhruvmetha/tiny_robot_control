"""The paper experiment directory defines its recorded planning metrics."""

from pathlib import Path


REAL_EXP = Path(__file__).resolve().parents[1] / "real_exp"


def test_real_experiment_readme_links_metric_definitions():
    readme = (REAL_EXP / "README.md").read_text()

    assert "[Planning metrics](METRICS.md)" in readme
    assert '--shuffle-seed "$seed"' in readme
    for trial, seed in enumerate(range(5), start=1):
        assert f"| `trial{trial}` | `{seed}` |" in readme
    assert "model_search" in readme
    assert "model_pure_policy" in readme
    assert "1628d1ff195047246315aa81a7808ba5300bc379" in readme


def test_formal_runs_use_distinct_formal_v2_arm_subtrees():
    readme = (REAL_EXP / "README.md").read_text()

    assert "`results/formal_v2/<axis>/<build_id>/<arm>/trialN/`" in readme
    assert 'arm=model_search' in readme
    assert 'arm=model_pure_policy' in readme
    assert 'diag_path="real_exp/results/formal_v2/${axis}/${build_id}/${arm}"' in readme


def test_real_experiment_docs_exclude_recorded_warmup_from_planning_time():
    metrics = (REAL_EXP / "METRICS.md").read_text()
    assert "`model_warmup_ms`" in metrics
    assert "warmup completes before the first operation" in metrics
    assert "clock begins" in metrics
    assert "Never add it to the" in metrics
    assert "paper planning-time metric" in metrics


def test_metric_reference_defines_fields_and_pure_policy_accounting():
    metrics = (REAL_EXP / "METRICS.md").read_text()

    for field in (
        "planning_operation",
        "planning_wall_time_ms",
        "simulations_used",
        "wall_time_ms_total",
        "simulations_used_total",
        "search_time_ms",
    ):
        assert f"`{field}`" in metrics

    assert "returns the ranked arg-max without calling `env.step`" in metrics
    assert "`simulations_used_total` is zero" in metrics
    assert "`planning_wall_time_ms` remains nonzero" in metrics
    assert "`totals.pushes_attempted`" in metrics
