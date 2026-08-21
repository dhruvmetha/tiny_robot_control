"""Carrying the held boundary across the session's process boundary.

closed_loop_session spawns run_namo.py as a separate process for every replan
attempt, so nothing the in-process planner remembers survives. Without the
target file the child re-derives which boundary to open each time, which is the
behaviour the whole feature exists to remove.

The file lives at RUN level, not iteration level: creating iter_N+1 rewrites
status.json from a fixed literal and regenerates scene_after/ wholesale, so
anything stored under an iteration is lost at the boundary.

Opt-in, so existing runs are untouched.
"""

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_session_module():
    """Same importlib shim scripts/seed_iter001_worker.py uses."""
    spec = importlib.util.spec_from_file_location(
        "closed_loop_session_under_test", REPO_ROOT / "scripts" / "closed_loop_session.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


cls_mod = _load_session_module()


def _run_dir(tmp_path, *, meta=None):
    run_dir = tmp_path / "session" / "primitive_run1"
    run_dir.mkdir(parents=True)
    if meta is not None:
        (run_dir / "run_meta.json").write_text(json.dumps(meta))
    return run_dir


# --- opt-in ------------------------------------------------------------------

def test_off_by_default(tmp_path):
    """A run that never asked for a held boundary must behave exactly as before."""
    assert not cls_mod._hold_region_target_enabled(_run_dir(tmp_path))


def test_enabled_by_run_meta(tmp_path):
    run_dir = _run_dir(tmp_path, meta={"hold_region_target": True})

    assert cls_mod._hold_region_target_enabled(run_dir)


def test_an_existing_target_keeps_the_run_in_held_mode(tmp_path):
    """A subproblem already in flight is not abandoned because a flag was dropped."""
    run_dir = _run_dir(tmp_path, meta={"run_name": "primitive_run1"})
    (run_dir / cls_mod.ACTIVE_TARGET_FILENAME).write_text("{}")

    assert cls_mod._hold_region_target_enabled(run_dir)


def test_unreadable_run_meta_does_not_break_the_check(tmp_path):
    run_dir = _run_dir(tmp_path)
    (run_dir / "run_meta.json").write_text("{not json")

    assert not cls_mod._hold_region_target_enabled(run_dir)


# --- placement ---------------------------------------------------------------

def test_the_target_is_a_run_level_artifact(tmp_path):
    """Iteration creation rewrites status.json and scene_after wholesale."""
    run_dir = _run_dir(tmp_path)
    target_path = run_dir / cls_mod.ACTIVE_TARGET_FILENAME

    assert target_path.parent == run_dir
    assert "iter_" not in str(target_path)


def test_iteration_creation_leaves_the_target_alone(tmp_path):
    """What advance-iteration does to iter_N+1 must not reach a run-level file."""
    import shutil

    run_dir = _run_dir(tmp_path)
    target_path = run_dir / cls_mod.ACTIVE_TARGET_FILENAME
    target_path.write_text('{"schema_version": 1}')

    next_iter = run_dir / "iter_002"
    next_iter.mkdir()
    (next_iter / "scene_after").mkdir()
    shutil.rmtree(next_iter)  # the --allow-overwrite path
    (next_iter / "sim_candidates").mkdir(parents=True)
    cls_mod._write_run_status(next_iter, 2, "awaiting_replan", [])

    assert target_path.read_text() == '{"schema_version": 1}'


def test_the_module_and_the_planner_agree_on_the_filename():
    """Two files name this; a mismatch would silently create two targets."""
    from robot_control.planner.region_target import ACTIVE_TARGET_FILENAME

    assert cls_mod.ACTIVE_TARGET_FILENAME == ACTIVE_TARGET_FILENAME


# --- the process boundary ----------------------------------------------------

class _RecordingRun:
    """Stands in for subprocess.run, capturing the argv the child would get."""

    def __init__(self):
        self.argv = None

    def __call__(self, cmd, **kwargs):
        self.argv = list(cmd)

        class _Completed:
            returncode = 1  # no candidate produced; the loop then gives up

        return _Completed()


def _minimal_iteration(tmp_path, *, hold):
    session_dir = tmp_path / "session"
    run_dir = session_dir / "primitive_run1"
    iter_dir = run_dir / "iter_001"
    (iter_dir / "scene_before").mkdir(parents=True)
    (iter_dir / "scene_before" / "env.xml").write_text("<mujoco/>")
    (iter_dir / "sim_candidates").mkdir(parents=True)
    meta = {"run_name": "primitive_run1", "strategy": "primitive", "shuffle_seed": 7}
    if hold:
        meta["hold_region_target"] = True
    (run_dir / "run_meta.json").write_text(json.dumps(meta))
    cls_mod._write_run_status(iter_dir, 1, "awaiting_replan", [])
    return session_dir, run_dir


def _drive_replan(monkeypatch, tmp_path, *, hold):
    session_dir, run_dir = _minimal_iteration(tmp_path, hold=hold)
    recorder = _RecordingRun()
    monkeypatch.setattr(cls_mod.subprocess, "run", recorder)
    monkeypatch.setattr(
        cls_mod, "_planner_car_1x_config_path", lambda _s: Path("namo_config.yaml")
    )
    cls_mod._run_full_replan_search(session_dir, "primitive_run1", 1)
    return recorder.argv, run_dir


def test_the_child_is_told_where_the_target_lives(monkeypatch, tmp_path):
    """Without this the subprocess re-derives the boundary on every replan."""
    argv, run_dir = _drive_replan(monkeypatch, tmp_path, hold=True)

    assert "--active-target" in argv
    assert argv[argv.index("--active-target") + 1] == str(
        run_dir / cls_mod.ACTIVE_TARGET_FILENAME
    )


def test_the_path_handed_over_is_absolute(monkeypatch, tmp_path):
    """The child runs with cwd=robot_control root, so a relative path would miss."""
    argv, _run_dir = _drive_replan(monkeypatch, tmp_path, hold=True)

    assert Path(argv[argv.index("--active-target") + 1]).is_absolute()


def test_a_run_that_did_not_opt_in_gets_the_old_argv(monkeypatch, tmp_path):
    argv, _run_dir = _drive_replan(monkeypatch, tmp_path, hold=False)

    assert "--active-target" not in argv


def test_goal_probe_loads_a_portable_scene_copy(monkeypatch, tmp_path):
    """The child must never receive the capture's stale absolute include."""
    from types import SimpleNamespace
    from robot_control.utils import scene_xml

    source = tmp_path / "capture" / "env.xml"
    source.parent.mkdir()
    source.write_text(
        '<mujoco><include file="/old/namo_cpp/test_xml/car/little_car.xml"/></mujoco>'
    )
    namo_dir = tmp_path / "checkout_named_namo"
    config = namo_dir / "config" / "namo_config.yaml"
    config.parent.mkdir(parents=True)
    config.write_text("planning: {}")
    captured = {}

    def fake_run(argv, **_kwargs):
        loadable = Path(argv[4])
        captured["path"] = loadable
        captured["xml"] = loadable.read_text()
        captured["namo_dir"] = argv[12]
        return SimpleNamespace(returncode=0, stdout="1\n", stderr="")

    monkeypatch.setattr(scene_xml, "resolve_namo_cpp_dir", lambda _anchor: namo_dir)
    monkeypatch.setattr(cls_mod, "resolve_namo_cpp_dir", lambda _anchor: namo_dir)
    monkeypatch.setattr(cls_mod, "_planner_car_1x_config_path", lambda _session: config)
    monkeypatch.setattr(cls_mod, "_goal_probe_python_bin", lambda: Path("python"))
    monkeypatch.setattr(cls_mod.subprocess, "run", fake_run)

    reachable = cls_mod._goal_wavefront_reachable(
        tmp_path, source, (10.0, 20.0, 30.0), (40.0, 50.0)
    )

    assert reachable is True
    assert f'{namo_dir}/test_xml/car/little_car.xml' in captured["xml"]
    assert captured["namo_dir"] == str(namo_dir)
    assert not captured["path"].exists()


# --- reuse is graded against the held boundary -------------------------------

def _write_target(run_dir, points=((0.3, 0.4), (0.31, 0.4)), fraction=0.2):
    from robot_control.planner.region_target import RegionOpeningTarget

    target = RegionOpeningTarget(
        target_samples_m=tuple(points), blocker_real_ids=("obj_4",), open_fraction=fraction
    )
    target.save(run_dir / cls_mod.ACTIVE_TARGET_FILENAME)
    return target


def test_a_held_boundary_is_loaded_from_the_run_directory(tmp_path):
    run_dir = _run_dir(tmp_path)
    written = _write_target(run_dir)

    loaded = cls_mod._load_active_region_target(run_dir)

    assert loaded == written


def test_no_target_file_means_no_held_boundary(tmp_path):
    assert cls_mod._load_active_region_target(_run_dir(tmp_path)) is None


def test_a_released_boundary_is_not_loaded(tmp_path):
    from robot_control.planner.region_target import STATUS_OPENED

    run_dir = _run_dir(tmp_path)
    _write_target(run_dir).released(STATUS_OPENED).save(
        run_dir / cls_mod.ACTIVE_TARGET_FILENAME
    )

    assert cls_mod._load_active_region_target(run_dir) is None


def test_the_session_ladder_uses_the_same_bar_as_the_planner(tmp_path):
    """Both ladders must ask the same question, or reuse means two things."""
    run_dir = _run_dir(tmp_path)
    target = _write_target(run_dir, points=[(i / 100.0, 0.0) for i in range(100)])

    loaded = cls_mod._load_active_region_target(run_dir)

    assert loaded.minimum_reachable() == target.minimum_reachable() == 20


# --- a damaged or foreign target must not kill the command -------------------

def test_a_truncated_target_file_is_treated_as_no_target(tmp_path):
    """A job killed mid-write leaves half a file. The session still has work."""
    run_dir = _run_dir(tmp_path)
    (run_dir / cls_mod.ACTIVE_TARGET_FILENAME).write_text('{"schema_version": 1, "targ')

    assert cls_mod._load_active_region_target(run_dir) is None


def test_a_future_schema_is_treated_as_no_target(tmp_path):
    import json

    run_dir = _run_dir(tmp_path)
    payload = {"schema_version": 99, "target_samples_m": [[0.1, 0.2]],
               "blocker_real_ids": ["obj_4"], "open_fraction": 0.2}
    (run_dir / cls_mod.ACTIVE_TARGET_FILENAME).write_text(json.dumps(payload))

    assert cls_mod._load_active_region_target(run_dir) is None
