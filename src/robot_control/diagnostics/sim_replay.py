"""Sim replay of a NAMO push chain into a continuous MuJoCo MP4.

Implementation detail: rendering happens in a child process
(``sim_replay_subprocess.py``) because per-tick qpos comes from the C++
side's ``NAMO_QPOS_DUMP`` mechanism, which uses a static FILE pointer
initialised the first time ``dump_qpos()`` runs. Once initialised, the
target path is locked for the lifetime of that process. Doing the
replay in the main runtime would either reuse a stale state (if the
planner already exercised dump_qpos) or pollute the dump with the
planner's own search activity. Subprocess gives us a clean state.

This module is a thin wrapper that prepares argv + waits on the child.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional

from robot_control.core.types import WorkspaceConfig


def render_chain_to_mp4(
    start_xml: str,
    namo_config: Optional[str],
    chain: List[Dict[str, Any]],
    output_mp4: str,
    artifact_dir: Optional[str] = None,
    workspace: Optional[WorkspaceConfig] = None,
    hold_frames: int = 0,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    starting_robot_pose_sim: Optional[tuple] = None,
    skip_video: bool = False,
) -> Optional[str]:
    """Render ``chain`` from ``start_xml`` to ``output_mp4`` as a continuous MP4.

    Args:
        start_xml: Path to the MuJoCo XML for the initial scene.
        namo_config: Path to namo_rl YAML config. ``None`` falls back to ""
            (C++ has its own default-config lookup).
        chain: List of dicts with keys ``object_id`` / ``sim_object_id``,
            ``edge_idx``, ``push_steps``, optionally ``depth``.
        output_mp4: Where to write the MP4.
        workspace, hold_frames, fps, width, height: Retained for API compat
            with earlier callers. ``hold_frames`` is ignored (rendering is
            continuous per-tick, not frozen-state). The others are forwarded
            into the subprocess if/when we make them configurable; currently
            it uses its own defaults.
        skip_video: If True, skip the MP4 encoding step. Physics + Tier 2
            artifact extraction still run. Cuts ~5-10 s per push and
            avoids the GPU-bound renderer init — useful for tuning loops.

    Returns:
        Output path on success, ``None`` on failure (errors logged, never raised).
        When ``skip_video=True``, returns the requested ``output_mp4`` path even
        though no file was written there, so callers can still locate the
        artifact dir alongside it.
    """
    if not chain:
        print("[sim_replay] empty chain; nothing to render", flush=True)
        return None

    # Hand the chain to the subprocess via a temp JSON. Direct argv passing
    # is fragile when the chain has nested dicts; a file is portable.
    chain_payload: Dict[str, Any] = {"chain": chain}
    if starting_robot_pose_sim is not None:
        # [x_m, y_m, theta_rad] — fed straight to env.set_robot_pose().
        chain_payload["starting_robot_pose_sim"] = list(starting_robot_pose_sim)
    if artifact_dir is not None:
        chain_payload["artifact_dir"] = artifact_dir
    if skip_video:
        chain_payload["skip_video"] = True
    fd, chain_path = tempfile.mkstemp(suffix=".json", prefix="sim_replay_chain_")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(chain_payload, f)
    except Exception as exc:
        print(f"[sim_replay] failed to write chain temp: {exc!r}", flush=True)
        try:
            os.close(fd)
        except OSError:
            pass
        return None

    args = [
        sys.executable,
        "-m",
        "robot_control.diagnostics.sim_replay_subprocess",
        start_xml,
        chain_path,
        output_mp4,
        namo_config or "",
    ]

    try:
        # Capture stdout/stderr and re-emit via print() so the subprocess's
        # log lines flow through the parent's Tee into run.log. Inheriting
        # fds doesn't work for that path: Tee.fileno() returns the tty fd,
        # so subprocess.run() with the default inheritance bypasses the
        # Tee and only the terminal sees the lines. Inherit the parent's
        # CWD — callers chdir into namo_cpp/ before invoking, so
        # motion-primitive paths in the config resolve.
        proc = subprocess.run(
            args,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if proc.stdout:
            # Strip trailing newline so print()'s newline doesn't double up.
            for line in proc.stdout.splitlines():
                print(line, flush=True)
    except Exception as exc:
        print(f"[sim_replay] subprocess raised: {exc!r}", flush=True)
        return None
    finally:
        try:
            os.unlink(chain_path)
        except OSError:
            pass

    if proc.returncode != 0:
        print(
            f"[sim_replay] subprocess exited with status {proc.returncode}; "
            f"see [sim_replay_subprocess] lines above for details",
            flush=True,
        )
        return None

    if not skip_video and not Path(output_mp4).exists():
        print(f"[sim_replay] subprocess returned 0 but {output_mp4} is missing",
              flush=True)
        return None

    return output_mp4
