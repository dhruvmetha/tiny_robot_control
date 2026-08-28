"""The paper experiment directory defines its recorded planning metrics."""

from pathlib import Path


REAL_EXP = Path(__file__).resolve().parents[1] / "real_exp"


def test_real_experiment_readme_links_metric_definitions():
    readme = (REAL_EXP / "README.md").read_text()

    assert "[Planning metrics](METRICS.md)" in readme
    assert "incomplete opening-success attempt" not in readme
    assert "`0`, `1`, `2`, `3`, and `4`" in readme
    assert '`--shuffle-seed "$seed"`' in readme
    assert "Uniform best-first consumes the seed" in readme
    assert "model-prior best-first is deterministic" in readme
    assert "seed 42" in readme
    assert "valid pilot" in readme


def test_real_experiment_docs_exclude_recorded_warmup_from_planning_time():
    metrics = (REAL_EXP / "METRICS.md").read_text()
    assert "`model_warmup_ms`" in metrics
    assert "excluded from `planning_wall_time_ms`" in metrics
    assert "not included in `wall_time_ms_total`" in metrics


def test_metric_reference_defines_fields_and_completed_easy_trial():
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

    assert "8 simulator transitions" in metrics
    assert "7 fresh-search" in metrics
    assert "1 reuse-verification" in metrics
    assert "6129.758 ms" in metrics
    assert "cannot be reconstructed" in metrics
    assert "does not invalidate the physical trial" in metrics
    assert "rerun it later for directly comparable timing" in metrics
