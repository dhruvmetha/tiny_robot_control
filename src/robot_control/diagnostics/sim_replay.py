"""Sim replay of a successful real-robot push chain into an MP4.

Given the scene XML captured at runtime start and the list of executed
PushSubgoals (object_id, edge_idx, depth), this module spins up a headless
NAMO RLEnvironment, re-executes each push, and stitches a synthetic
top-down MP4 of the resulting state progression.

The MP4 is "frozen state" between pushes (one held frame per push, padded
to a configurable hold duration) rather than a smooth physics replay —
RLEnvironment.step() executes each push as a single C++ call and does not
surface intermediate qpos, so we render the before/after snapshots only.
That's enough for a visual diff against the real-side per-subgoal MP4.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from robot_control.core.types import Observation, ObjectPose, WorkspaceConfig
from robot_control.diagnostics.sim_renderer import render_top_down
from robot_control.planner.namo_binding_loader import load_canonical_namo_rl


# Half-extents (MuJoCo geom_size convention) in meters → full extents in cm.
_HALF_M_TO_FULL_CM = 200.0
# Meters → centimeters for positions.
_M_TO_CM = 100.0
# Held frames per push step. At 30 fps this is ~1 s per push; tweak via arg.
_DEFAULT_HOLD_FRAMES = 30
_DEFAULT_FPS = 30


def render_chain_to_mp4(
    start_xml: str,
    namo_config: Optional[str],
    chain: List[Dict[str, Any]],
    output_mp4: str,
    workspace: Optional[WorkspaceConfig] = None,
    hold_frames: int = _DEFAULT_HOLD_FRAMES,
    fps: int = _DEFAULT_FPS,
) -> Optional[str]:
    """Replay `chain` against `start_xml` in sim and write `output_mp4`.

    Args:
        start_xml: Path to MuJoCo XML for the initial scene.
        namo_config: Path to namo_rl YAML config. If None, the helper will
            still try to construct RLEnvironment with an empty string (the
            C++ side has its own default-config fallback).
        chain: List of dicts with keys {object_id, edge_idx, push_steps, depth}.
        output_mp4: Where to write the MP4.
        workspace: Optional WorkspaceConfig (cm). If None, derived from the
            env's world bounds — robot size falls back to a tiny placeholder
            so the synthetic robot dot stays visible.
        hold_frames: Number of frames to write per push state (one before
            push #1 then one after each push).
        fps: Output video frame rate.

    Returns:
        Output path on success, None on failure (errors are logged, not raised).
    """
    if not chain:
        print("[sim_replay] empty chain; nothing to render", flush=True)
        return None

    try:
        namo_rl, _, _ = load_canonical_namo_rl(Path(__file__))
    except Exception as exc:
        print(f"[sim_replay] failed to load namo_rl: {exc!r}", flush=True)
        return None

    try:
        env = namo_rl.RLEnvironment(start_xml, namo_config or "", False)
    except Exception as exc:
        print(f"[sim_replay] RLEnvironment ctor failed: {exc!r}", flush=True)
        return None

    if workspace is None:
        workspace = _workspace_from_env(env)

    # Encode the initial state.
    writer: Optional[cv2.VideoWriter] = None
    output_path = Path(output_mp4)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    def _write_state(label: str) -> None:
        nonlocal writer
        obs = _observation_from_env(env)
        jpg_bytes = render_top_down(workspace, obs, title=label)
        frame = cv2.imdecode(np.frombuffer(jpg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        if writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(output_path), fourcc, fps, (w, h))
            if not writer.isOpened():
                print(f"[sim_replay] could not open VideoWriter at {output_path}",
                      flush=True)
                writer = None
                return
        for _ in range(hold_frames):
            writer.write(frame)

    _write_state("start")

    for idx, step in enumerate(chain, start=1):
        action = namo_rl.Action()
        action.object_id = step["object_id"]
        action.edge_idx = int(step["edge_idx"])
        action.depth = int(step.get("depth", step["push_steps"] - 1))
        # x/y/theta are only used when edge_idx/depth are not set; pass
        # zeros so the C++ side resolves the primitive purely from
        # (edge_idx, depth).
        action.x = 0.0
        action.y = 0.0
        action.theta = 0.0

        try:
            result = env.step(action)
        except Exception as exc:
            print(f"[sim_replay] step {idx} ({step['object_id']} e{action.edge_idx} "
                  f"d{action.depth}) raised: {exc!r}", flush=True)
            break
        info = getattr(result, "info", {}) or {}
        label = f"step {idx}: {step['object_id']} e{action.edge_idx} d{action.depth}"
        if info:
            label += f"  [{', '.join(f'{k}={v}' for k, v in list(info.items())[:2])}]"
        _write_state(label)

    if writer is not None:
        writer.release()
        print(f"[sim_replay] wrote {output_path}", flush=True)
        return str(output_path)
    print("[sim_replay] no frames written; nothing to release", flush=True)
    return None


def _workspace_from_env(env) -> WorkspaceConfig:
    # World bounds come back in meters as [xmin, xmax, ymin, ymax]. The
    # renderer assumes the workspace origin is its bottom-left corner, so
    # we shift the bounds to start at (0, 0) when we sample object poses.
    bounds = env.get_world_bounds()
    width_cm = (bounds[1] - bounds[0]) * _M_TO_CM
    height_cm = (bounds[3] - bounds[2]) * _M_TO_CM

    # Robot size is best-effort; pull from get_object_info() if present.
    car_w = car_h = 7.0  # cm — matches the 7×7 default in real.yaml
    info = env.get_object_info()
    robot_info = info.get("robot")
    if isinstance(robot_info, dict) and "size_x" in robot_info:
        car_w = robot_info["size_x"] * _HALF_M_TO_FULL_CM
        car_h = robot_info["size_y"] * _HALF_M_TO_FULL_CM

    return WorkspaceConfig(
        width=width_cm,
        height=height_cm,
        car_width=car_w,
        car_height=car_h,
        offset_w=0.0,
        offset_h=0.0,
    )


def _observation_from_env(env) -> Observation:
    # env.get_observation() returns SE2 poses keyed by "<entity>_pose" in
    # METERS and RADIANS (e.g. "robot_pose", "obstacle_1_movable_pose").
    # The matching entries in get_object_info() are keyed by the bare entity
    # name, so we strip the suffix before crossing tables.
    # Sizes are MuJoCo half-extents (meters); positions are meters.
    bounds = env.get_world_bounds()
    x_shift = -bounds[0] * _M_TO_CM
    y_shift = -bounds[2] * _M_TO_CM

    poses = env.get_observation()
    info = env.get_object_info()

    def _strip_pose(key: str) -> str:
        return key[:-len("_pose")] if key.endswith("_pose") else key

    robot_pose = poses.get("robot_pose", [0.0, 0.0, 0.0])
    robot_x = robot_pose[0] * _M_TO_CM + x_shift
    robot_y = robot_pose[1] * _M_TO_CM + y_shift
    robot_theta_deg = math.degrees(robot_pose[2])

    objects: Dict[str, ObjectPose] = {}
    for pose_key, pose in poses.items():
        if pose_key == "robot_pose":
            continue
        name = _strip_pose(pose_key)
        size = info.get(name, {})
        size_x_cm = size.get("size_x", 0.05) * _HALF_M_TO_FULL_CM
        size_y_cm = size.get("size_y", 0.05) * _HALF_M_TO_FULL_CM
        size_z_cm = size.get("size_z", 0.0) * _HALF_M_TO_FULL_CM
        # Static objects have pos/quat baked into get_object_info(); use those
        # since they may not appear in get_observation() if the env didn't add
        # them to the world_state. (Movable objects always do.)
        is_static = "pos_x" in size
        if is_static and name not in poses:
            obj_x = size["pos_x"] * _M_TO_CM + x_shift
            obj_y = size["pos_y"] * _M_TO_CM + y_shift
            # Quaternion → yaw (radians) → degrees. Right-handed Z-up frame.
            qw, qx, qy, qz = (size["quat_w"], size["quat_x"],
                              size["quat_y"], size["quat_z"])
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            obj_theta_deg = math.degrees(yaw)
        else:
            obj_x = pose[0] * _M_TO_CM + x_shift
            obj_y = pose[1] * _M_TO_CM + y_shift
            obj_theta_deg = math.degrees(pose[2])

        objects[name] = ObjectPose(
            x=obj_x,
            y=obj_y,
            theta=obj_theta_deg,
            width=size_y_cm,
            depth=size_x_cm,
            height=size_z_cm,
            is_static=is_static,
        )

    # Statics that the env never publishes via get_observation() — pull them
    # from object_info directly.
    for name, size in info.items():
        if name == "robot" or name in objects:
            continue
        if "pos_x" not in size:
            continue
        obj_x = size["pos_x"] * _M_TO_CM + x_shift
        obj_y = size["pos_y"] * _M_TO_CM + y_shift
        qw, qx, qy, qz = (size["quat_w"], size["quat_x"],
                          size["quat_y"], size["quat_z"])
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        objects[name] = ObjectPose(
            x=obj_x,
            y=obj_y,
            theta=math.degrees(yaw),
            width=size.get("size_y", 0.05) * _HALF_M_TO_FULL_CM,
            depth=size.get("size_x", 0.05) * _HALF_M_TO_FULL_CM,
            height=size.get("size_z", 0.0) * _HALF_M_TO_FULL_CM,
            is_static=True,
        )

    return Observation(
        robot_x=robot_x,
        robot_y=robot_y,
        robot_theta=robot_theta_deg,
        objects=objects,
        timestamp=0.0,
    )
