"""Subprocess that renders a NAMO push chain into a continuous MuJoCo MP4.

Why a subprocess? The C++ side dumps per-tick qpos via NAMO_QPOS_DUMP, but
the file path is read into a static FILE pointer the very first time
dump_qpos() runs in a process. Once initialized, it cannot be retargeted.
The main runtime process has already exercised the planner's env (which
either set or did not set NAMO_QPOS_DUMP — in the production path it's
unset, so the static is initialized to "no dump"). To get a fresh
per-replay dump, we run the chain in a child process whose static state
starts from scratch.

Invocation:
    python -m robot_control.diagnostics.sim_replay_subprocess
        <start_xml>
        <chain.json>
        <output_mp4>
        [<namo_config>]

The subprocess inherits CWD from its parent. Callers should chdir to
``namo_cpp/`` first so motion-primitive paths in the config resolve
(same workaround NAMOPlanBridge uses).
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

# ------------------------------------------------------------------ NAMO_QPOS_DUMP
# Allocate the dump path and export the env var BEFORE any import that
# transitively loads the namo_rl C++ extension. The dump_qpos() function
# in src/navigation/qpos_dump.cpp reads NAMO_QPOS_DUMP exactly once via
# a static-init guard.
_QPOS_FD, _QPOS_PATH = tempfile.mkstemp(suffix=".qpos.txt", prefix="sim_replay_")
os.close(_QPOS_FD)
os.environ["NAMO_QPOS_DUMP"] = _QPOS_PATH


# Render resolution. Bump if you need higher quality; remember to also
# increase the offscreen framebuffer (injected below).
_WIDTH = 1280
_HEIGHT = 720
_FPS = 30
# Stride is derived from the loaded model's timestep so the MP4 plays at
# 1× wall-clock — sphere uses 0.01s/tick (stride≈3), car uses 0.002s/tick
# (stride≈17). Same pattern as namo_cpp/scripts/render_qpos_dump.py.
# Camera padding factor — lookat distance = workspace_extent_m * this.
_CAMERA_DISTANCE_FACTOR = 1.6


def _inject_offscreen_size(xml_str: str, width: int, height: int) -> str:
    """Inject ``<visual><global offwidth=.. offheight=../></visual>``."""
    root = ET.fromstring(xml_str)
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    glob = visual.find("global")
    if glob is None:
        glob = ET.SubElement(visual, "global")
    glob.set("offwidth", str(width))
    glob.set("offheight", str(height))
    return ET.tostring(root, encoding="unicode")


def _read_qpos_dump(path: str) -> list:
    """Parse the dump file. Each line: ``phase_id nq q0 q1 ... q(nq-1)``."""
    frames = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            try:
                nq = int(parts[1])
                q = [float(x) for x in parts[2:2 + nq]]
            except ValueError:
                continue
            if q:
                frames.append(q)
    return frames


def main() -> int:
    if len(sys.argv) < 4:
        print(
            "usage: sim_replay_subprocess.py <xml> <chain.json> <output_mp4> [namo_config]",
            file=sys.stderr,
        )
        return 2

    xml_path = sys.argv[1]
    chain_path = sys.argv[2]
    output_mp4 = sys.argv[3]
    namo_config = sys.argv[4] if len(sys.argv) >= 5 else ""

    chain_doc = json.loads(Path(chain_path).read_text())
    chain = chain_doc.get("chain", chain_doc)
    if not chain:
        print("[sim_replay_subprocess] empty chain; nothing to render", flush=True)
        return 1

    # Optional starting robot pose in sim units (meters + radians). Required
    # for the car robot — its body lives inside little_car.xml with a freejoint
    # spawn pos that can't be parameterized through a top-level <include>, so
    # the replay env starts the car at the spawn pos baked into the XML. The
    # production planning path (NAMOPlanningService) teleports via
    # set_robot_pose after construction; we mirror that here. Sphere XMLs bake
    # the pose into the geom and don't need this — chain.json will omit the
    # field for sphere runs.
    starting_robot_pose_sim = chain_doc.get("starting_robot_pose_sim")

    from robot_control.planner.namo_binding_loader import load_canonical_namo_rl
    namo_rl, _, _ = load_canonical_namo_rl(Path(__file__))

    defer_warmup = starting_robot_pose_sim is not None
    try:
        env = namo_rl.RLEnvironment(xml_path, namo_config, False, defer_warmup)
    except Exception as exc:
        print(f"[sim_replay_subprocess] RLEnvironment ctor failed: {exc!r}", flush=True)
        return 1

    if defer_warmup:
        try:
            env.set_robot_pose(*starting_robot_pose_sim)
            env.warm_up()
        except Exception as exc:
            print(
                f"[sim_replay_subprocess] set_robot_pose/warm_up failed: {exc!r} "
                f"(pose={starting_robot_pose_sim})",
                flush=True,
            )
            return 1

    # Mirror the planner-side setting so the push primitive doesn't abort
    # on wall contact and the qpos dump captures the full chain motion.
    try:
        env.set_collision_checking(False)
    except Exception as exc:
        print(
            f"[sim_replay_subprocess] set_collision_checking failed: {exc!r}",
            flush=True,
        )

    for idx, step in enumerate(chain, start=1):
        action = namo_rl.Action()
        action.object_id = step.get("sim_object_id") or step["object_id"]
        action.edge_idx = int(step["edge_idx"])
        action.depth = int(step.get("depth", step["push_steps"] - 1))
        action.x = 0.0
        action.y = 0.0
        action.theta = 0.0
        try:
            env.step(action)
        except Exception as exc:
            print(
                f"[sim_replay_subprocess] step {idx} raised: {exc!r}",
                flush=True,
            )
            return 1

    qpos_frames = _read_qpos_dump(_QPOS_PATH)
    if not qpos_frames:
        print(
            "[sim_replay_subprocess] qpos dump was empty — the C++ push "
            "primitive may not have run dump_qpos. Check that NAMO_QPOS_DUMP "
            f"is honoured by the build at {_QPOS_PATH}",
            flush=True,
        )
        return 1

    # MuJoCo rendering ----------------------------------------------------
    import mujoco
    import cv2

    xml_str = _inject_offscreen_size(Path(xml_path).read_text(), _WIDTH, _HEIGHT)
    try:
        model = mujoco.MjModel.from_xml_string(xml_str)
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, height=_HEIGHT, width=_WIDTH)
    except Exception as exc:
        print(
            f"[sim_replay_subprocess] mujoco model/renderer init failed: {exc!r}",
            flush=True,
        )
        return 1

    # Stride = ticks per video frame so playback is 1× wall-clock. Read the
    # timestep off the loaded model (car: 0.002s → 17, sphere: 0.01s → 3).
    stride = max(1, int(round((1.0 / _FPS) / model.opt.timestep)))
    qpos_frames = qpos_frames[::stride]

    bounds = env.get_world_bounds()
    cx = 0.5 * (bounds[0] + bounds[1])
    cy = 0.5 * (bounds[2] + bounds[3])
    extent = max(bounds[1] - bounds[0], bounds[3] - bounds[2])
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [cx, cy, 0.0]
    camera.distance = extent * _CAMERA_DISTANCE_FACTOR
    camera.azimuth = 90.0
    camera.elevation = -90.0

    Path(output_mp4).parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_mp4, fourcc, _FPS, (_WIDTH, _HEIGHT))
    if not writer.isOpened():
        print(
            f"[sim_replay_subprocess] VideoWriter open failed at {output_mp4}",
            flush=True,
        )
        return 1

    for q in qpos_frames:
        nq = min(len(q), model.nq)
        if nq > 0:
            data.qpos[:nq] = q[:nq]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera)
        rgb = renderer.render()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        writer.write(bgr)
    writer.release()
    try:
        renderer.close()
    except Exception:
        pass
    try:
        os.unlink(_QPOS_PATH)
    except OSError:
        pass

    print(
        f"[sim_replay_subprocess] wrote {output_mp4} "
        f"({len(qpos_frames)} frames @ {_FPS} fps, stride={stride})",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
