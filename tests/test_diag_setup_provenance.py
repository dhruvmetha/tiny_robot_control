"""Diagnostics records the exact paired source revisions before writing output."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from scripts import _diag_setup as diag


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _clean_git_repo(path: Path) -> Path:
    path.mkdir()
    _git(path, "init", "-q")
    _git(path, "config", "user.name", "Experiment Test")
    _git(path, "config", "user.email", "experiment-test@example.invalid")
    _git(path, "config", "commit.gpgsign", "false")
    (path / "tracked.txt").write_text("frozen\n", encoding="utf-8")
    _git(path, "add", "tracked.txt")
    _git(path, "commit", "-q", "-m", "freeze")
    return path


def test_bootstrap_snapshots_both_repositories_before_creating_output(
    tmp_path, monkeypatch,
):
    robot_repo = _clean_git_repo(tmp_path / "robot")
    namo_repo = _clean_git_repo(tmp_path / "namo")
    monkeypatch.setattr(diag, "ROBOT_CONTROL_REPO", robot_repo, raising=False)
    monkeypatch.setenv("NAMO_REPO", str(namo_repo))
    diag_root = robot_repo / "real_exp" / "results" / "formal_v2"
    args = argparse.Namespace(
        diag_path=str(diag_root),
        run_name="trial1",
        allow_overwrite=False,
        goal=[11.0, 67.6],
        sim=False,
        config=None,
        objects=None,
        strategy="primitive",
        algorithm="full_namo",
        capture_scene=False,
    )

    old_out, old_err = sys.stdout, sys.stderr
    recorder = log_file = None
    try:
        recorder, log_file = diag.bootstrap_diagnostics(args)
    finally:
        sys.stdout, sys.stderr = old_out, old_err
        if log_file is not None:
            log_file.close()
        if recorder is not None:
            recorder.close()

    payload = json.loads((diag_root / "trial1" / "config.json").read_text())
    robot = payload["repositories"]["robot_control"]
    backend = payload["repositories"]["namo_cpp"]
    assert robot["commit"] == _git(robot_repo, "rev-parse", "--short", "HEAD")
    assert backend["commit"] == _git(namo_repo, "rev-parse", "--short", "HEAD")
    assert robot["dirty"] is False
    assert backend["dirty"] is False
    assert payload["git"] == {
        "commit": robot["commit"],
        "dirty": robot["dirty"],
        "branch": robot["branch"],
    }
