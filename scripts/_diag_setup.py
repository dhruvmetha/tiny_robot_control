"""Bootstrap helpers for the diagnostics pipeline used by run_namo.py.

Kept separate from run_namo.py to avoid bloating the entry-point script.
Two things live here:
  - resolve_run_name(template, args) — substitute placeholders into --run-name
  - bootstrap_diagnostics(args)      — install Tee, build recorder, write config

Both are import-safe and never break the run when diagnostics are disabled.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from robot_control.diagnostics import DiagnosticsRecorder, Tee


ROBOT_CONTROL_REPO = Path(__file__).resolve().parent.parent


# --------------------------------------------------------------------- template


def resolve_run_name(template: str, args: argparse.Namespace) -> str:
    """Substitute placeholders in --run-name. Slashes in the result create
    nested subdirectories under --diag-path. Unknown placeholders raise
    KeyError so typos surface immediately rather than silently disappearing.
    """
    now = time.time()
    local = _dt.datetime.fromtimestamp(now)
    goal = getattr(args, "goal", None)
    goal_str = "_".join(str(int(round(g))) for g in goal) if goal else "nogoal"
    mode = "sim" if getattr(args, "sim", False) or not getattr(args, "config", None) else "real"
    git_short, _ = _git_status()

    placeholders: Dict[str, str] = {
        "timestamp": local.strftime("%Y%m%d_%H%M%S"),
        "date":      local.strftime("%Y%m%d"),
        "time":      local.strftime("%H%M%S"),
        "epoch":     str(int(now)),
        "strategy":  str(getattr(args, "strategy", "primitive")),
        "algorithm": str(getattr(args, "algorithm", "full_namo")),
        "goal":      goal_str,
        "mode":      mode,
        "git":       git_short,
    }

    try:
        return template.format(**placeholders)
    except KeyError as exc:
        valid = ", ".join(sorted(placeholders))
        raise ValueError(
            f"Unknown placeholder {exc} in --run-name template. "
            f"Valid placeholders: {valid}"
        ) from None


def _git_snapshot(repo: Optional[Path]) -> Dict[str, Any]:
    """Return one repository's revision state without raising."""
    unavailable: Dict[str, Any] = {
        "commit": "nogit",
        "dirty": False,
        "branch": "unknown",
        "available": False,
    }
    if repo is None:
        return unavailable

    try:
        def git(*args: str) -> str:
            return subprocess.run(
                ["git", "-C", str(repo), *args],
                capture_output=True,
                text=True,
                check=True,
                timeout=2.0,
            ).stdout.strip()

        return {
            "commit": git("rev-parse", "--short", "HEAD"),
            "dirty": git("status", "--porcelain") != "",
            "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "available": True,
        }
    except (OSError, subprocess.SubprocessError):
        return unavailable


def _repository_snapshots() -> Dict[str, Dict[str, Any]]:
    """Snapshot the paired runtime repositories used by a real experiment."""
    backend = os.environ.get("NAMO_REPO")
    return {
        "robot_control": _git_snapshot(ROBOT_CONTROL_REPO),
        "namo_cpp": _git_snapshot(
            Path(backend).expanduser() if backend else None
        ),
    }


def _git_status() -> Tuple[str, bool]:
    """Return the robot-control (short commit, dirty) compatibility tuple."""
    snapshot = _git_snapshot(ROBOT_CONTROL_REPO)
    return str(snapshot["commit"]), bool(snapshot["dirty"])


# --------------------------------------------------------------- bootstrap entry


def bootstrap_diagnostics(
    args: argparse.Namespace,
) -> Tuple[Optional[DiagnosticsRecorder], Optional[object]]:
    """Resolve flags, create directory, install Tee, build recorder.

    Returns (recorder_or_None, log_file_handle_or_None). Caller is
    responsible for keeping the log file handle alive (so Tee continues
    writing) until just before process exit, then closing it.

    If --diag-path is not set, returns (None, None) without side effects.
    """
    if not getattr(args, "diag_path", None):
        return None, None

    if not getattr(args, "run_name", None):
        raise SystemExit(
            "Error: --run-name is required when --diag-path is set."
        )

    # Capture source state before creating output inside this repository.
    # Otherwise the run directory itself makes a clean checkout look dirty.
    repositories = _repository_snapshots()

    # Resolve template
    try:
        resolved = resolve_run_name(args.run_name, args)
    except ValueError as exc:
        raise SystemExit(f"Error: {exc}")

    diag_root = Path(args.diag_path).expanduser().resolve()
    run_dir = diag_root / resolved

    # Build the recorder. It will validate-and-create the directory, raising
    # FileExistsError if it exists and --allow-overwrite was not passed.
    try:
        recorder = DiagnosticsRecorder(
            root_dir=run_dir,
            allow_overwrite=getattr(args, "allow_overwrite", False),
        )
    except FileExistsError as exc:
        raise SystemExit(f"Error: {exc}")

    # Install Tee on stdout + stderr so the run.log file mirrors everything.
    # Line-buffered for crash safety; uses utf-8 to handle the unicode
    # arrows / emojis we already print elsewhere.
    log_path = run_dir / "run.log"
    log_file = open(log_path, "w", buffering=1, encoding="utf-8")
    sys.stdout = Tee(sys.stdout, log_file)
    sys.stderr = Tee(sys.stderr, log_file)

    # Write config.json now — captures pre-run state so even an immediate
    # crash leaves a reproducible record of what was being attempted.
    recorder.write_config(
        _build_config_payload(args, resolved, run_dir, repositories)
    )

    if not repositories["namo_cpp"]["available"]:
        print(
            "[DIAG] WARNING: NAMO backend Git provenance unavailable; "
            "set NAMO_REPO to the active namo_cpp checkout before a formal run.",
            flush=True,
        )

    print(f"[DIAG] Diagnostics enabled: {run_dir}", flush=True)
    return recorder, log_file


def _build_config_payload(
    args: argparse.Namespace,
    resolved_run_name: str,
    run_dir: Path,
    repositories: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    """Snapshot CLI args, parsed YAMLs, and paired source provenance."""
    now = time.time()
    iso = _dt.datetime.fromtimestamp(now, tz=_dt.timezone.utc).isoformat(
        timespec="milliseconds"
    ).replace("+00:00", "Z")

    robot_git = repositories["robot_control"]
    compatibility_git = {
        "commit": robot_git["commit"],
        "dirty": robot_git["dirty"],
        "branch": robot_git["branch"],
    }

    # Snapshot YAML configs if present.
    config_yaml = None
    if getattr(args, "config", None):
        try:
            with open(args.config) as f:
                config_yaml = yaml.safe_load(f)
        except Exception as exc:
            config_yaml = {"_error": f"failed to load: {exc!r}"}

    objects_yaml = None
    objects_path = getattr(args, "objects", None)
    if objects_path and Path(objects_path).exists():
        try:
            with open(objects_path) as f:
                objects_yaml = yaml.safe_load(f)
        except Exception as exc:
            objects_yaml = {"_error": f"failed to load: {exc!r}"}

    # controller.yaml too: the push speeds and the safety_filter block are
    # what a run's behaviour depends on, and the flag alone does not say
    # whether the yaml had the filter on.
    controller_yaml = None
    try:
        from robot_control.controller.config import DEFAULT_CONTROLLER_YAML

        with open(DEFAULT_CONTROLLER_YAML) as f:
            controller_yaml = yaml.safe_load(f)
    except Exception as exc:
        controller_yaml = {"_error": f"failed to load: {exc!r}"}

    return {
        "run_name": resolved_run_name,
        "started_at_epoch": now,
        "started_at_utc": iso,
        "git": compatibility_git,
        "repositories": repositories,
        "command_line": [shlex.quote(a) for a in sys.argv],
        "args": _args_to_dict(args),
        "config_yaml_path": getattr(args, "config", None),
        "config_yaml": config_yaml,
        "objects_yaml_path": getattr(args, "objects", None),
        "objects_yaml": objects_yaml,
        "controller_yaml": controller_yaml,
        "diagnostics": {
            "capture_scene": bool(getattr(args, "capture_scene", False)),
            "diag_path": str(Path(args.diag_path).expanduser().resolve()),
            "resolved_run_dir": str(run_dir),
            "allow_overwrite": bool(getattr(args, "allow_overwrite", False)),
        },
    }


def _args_to_dict(args: argparse.Namespace) -> Dict[str, Any]:
    """Argparse Namespace → plain dict, JSON-safe."""
    out: Dict[str, Any] = {}
    for k, v in vars(args).items():
        if isinstance(v, Path):
            out[k] = str(v)
        else:
            out[k] = v
    return out
