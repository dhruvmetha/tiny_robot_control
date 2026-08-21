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

import datetime as dt
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

# ------------------------------------------------------------------ NAMO_QPOS_DUMP
# Allocate the dump path and export the env var BEFORE any import that
# transitively loads the namo_rl C++ extension. The dump_qpos() function
# in src/navigation/qpos_dump.cpp reads NAMO_QPOS_DUMP exactly once via
# a static-init guard.
_QPOS_FD, _QPOS_PATH = tempfile.mkstemp(suffix=".qpos.txt", prefix="sim_replay_")
os.close(_QPOS_FD)
os.environ["NAMO_QPOS_DUMP"] = _QPOS_PATH


def _configure_headless_mujoco_backend() -> None:
    """Prefer EGL for headless replay renders when no backend was requested.

    On this stack, leaving MUJOCO_GL unset under a no-DISPLAY session makes
    mujoco.Renderer fall through to GLFW/X11 and fail before the MP4 render
    begins. EGL works headlessly, so adopt it automatically unless the caller
    already pinned a backend explicitly.
    """
    if os.environ.get("MUJOCO_GL"):
        return
    if os.environ.get("DISPLAY"):
        return
    os.environ["MUJOCO_GL"] = "egl"
    print(
        "[sim_replay_subprocess] DISPLAY is unset; defaulting MUJOCO_GL=egl "
        "for headless rendering",
        flush=True,
    )


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


def _maybe_reencode_mp4_for_vscode_compat(path: str) -> None:
    """Best-effort MP4 re-encode to H.264 for stricter previewers like VS Code.

    OpenCV's ``mp4v`` writer produces MPEG-4 Part 2 video, which many players
    accept but VS Code's preview often rejects. If ``ffmpeg`` is available, we
    re-encode in place to ``avc1``/H.264. Failure is non-fatal: callers still
    keep the original MP4.
    """
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        return

    src = Path(path)
    if not src.exists() or src.suffix.lower() != ".mp4":
        return

    tmp = src.with_name(f"{src.stem}.h264_tmp{src.suffix}")
    cmd = [
        ffmpeg,
        "-y",
        "-i",
        str(src),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(tmp),
    ]
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except Exception as exc:
        print(
            f"[sim_replay_subprocess] WARN: ffmpeg re-encode raised: {exc!r}",
            flush=True,
        )
        return

    if proc.returncode != 0 or not tmp.exists():
        print(
            "[sim_replay_subprocess] WARN: ffmpeg re-encode failed; "
            "leaving original mp4v file in place",
            flush=True,
        )
        if proc.stdout:
            for line in proc.stdout.splitlines()[-10:]:
                print(f"[sim_replay_subprocess] ffmpeg: {line}", flush=True)
        try:
            tmp.unlink()
        except OSError:
            pass
        return

    try:
        os.replace(tmp, src)
        print(
            f"[sim_replay_subprocess] re-encoded {src.name} to H.264 for compatibility",
            flush=True,
        )
    except Exception as exc:
        print(
            f"[sim_replay_subprocess] WARN: replacing re-encoded video failed: {exc!r}",
            flush=True,
        )
        try:
            tmp.unlink()
        except OSError:
            pass


def _write_jsonl(path: Path, records: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for rec in records:
            f.write(json.dumps(rec) + "\n")


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


def _quat_z_to_theta_deg(qw: float, qz: float) -> float:
    theta_deg = math.degrees(2.0 * math.atan2(qz, qw))
    return ((theta_deg + 180.0) % 360.0) - 180.0


def _body_pose_cm_deg(mujoco, model, data, body_name: str) -> Optional[Tuple[float, float, float]]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return None
    xpos = data.xpos[body_id]
    xquat = data.xquat[body_id]
    return (
        float(xpos[0] * 100.0),
        float(xpos[1] * 100.0),
        _quat_z_to_theta_deg(float(xquat[0]), float(xquat[3])),
    )


def _geom_half_extents_cm(mujoco, model, geom_name: str) -> Optional[Tuple[float, float, float]]:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0:
        return None
    size = model.geom_size[geom_id]
    return (
        float(size[0] * 100.0),
        float(size[1] * 100.0),
        float(size[2] * 100.0),
    )


# Mirrors namo_push_controller.cpp:12 (kPushLookaheadRatio). If that constant
# changes on the C++ side, update this to match.
PUSH_LOOKAHEAD_RATIO = 1.0


def _car_size_m(mujoco, model) -> float:
    """Bounding-box max dimension of the diff-drive car chassis, in meters.

    Mirrors namo_push_controller.cpp:59-63:
        car_size = max(full_width, full_height) over the chassis geoms.

    Walks the front/rear chassis collision geoms (canonical names from
    little_car.xml). Falls back to 0.07 m (the 7-cm production car) if
    neither is present.
    """
    xs: List[float] = []
    ys: List[float] = []
    for name in ("front_chassis_collision", "rear_chassis_collision"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            continue
        pos = model.geom_pos[geom_id]
        size = model.geom_size[geom_id]
        xs.extend([float(pos[0] - size[0]), float(pos[0] + size[0])])
        ys.extend([float(pos[1] - size[1]), float(pos[1] + size[1])])
    if not xs or not ys:
        return 0.07
    return max(max(xs) - min(xs), max(ys) - min(ys))


def _display_name_for_sim_object(sim_name: str) -> str:
    match = re.fullmatch(r"obstacle_(\d+)_movable", sim_name)
    if match:
        return f"obj_{match.group(1)}"
    return sim_name


def _collect_scene_object_specs(mujoco, model) -> List[Dict[str, Any]]:
    specs: List[Dict[str, Any]] = []
    for geom_id in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if not geom_name:
            continue
        if not (geom_name.startswith("obstacle_") or geom_name.startswith("wall_")):
            continue
        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or geom_name
        size = model.geom_size[geom_id]
        specs.append(
            {
                "sim_name": geom_name,
                "body_name": body_name,
                "display_name": _display_name_for_sim_object(geom_name),
                "width_cm": float(size[1] * 200.0),
                "depth_cm": float(size[0] * 200.0),
                "height_cm": float(size[2] * 200.0),
                "is_static": geom_name.startswith("wall_"),
            }
        )
    specs.sort(key=lambda spec: str(spec["sim_name"]))
    return specs


def _load_skill_knobs(namo_config: str) -> Dict[str, Any]:
    if not namo_config:
        return {}
    path = Path(namo_config)
    if not path.exists():
        return {}
    doc = yaml.safe_load(path.read_text()) or {}
    return (doc.get("skill") or {}) if isinstance(doc, dict) else {}


def _parse_nav_log(
    nav_log_path: Path,
    wheel_radius_m: float,
    k_car_wheel_max_speed_ms: float,
    push_start_obs_timestamp: float,
    object_id: str,
    edge_idx: int,
    push_steps: int,
) -> tuple[List[Dict[str, Any]], Optional[List[List[float]]]]:
    commands: List[Dict[str, Any]] = []
    push_path_cm: Optional[List[List[float]]] = None
    if not nav_log_path.exists():
        return commands, push_path_cm

    with open(nav_log_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("[PUSH_PATH] "):
                parts = line.split()
                if len(parts) >= 4 and push_path_cm is None:
                    parsed: List[List[float]] = []
                    for token in parts[2:]:
                        try:
                            x_str, y_str = token.split(",", 1)
                            parsed.append([float(x_str) * 100.0, float(y_str) * 100.0])
                        except ValueError:
                            parsed = []
                            break
                    if parsed:
                        push_path_cm = parsed
                continue
            if not line.startswith("[PUSH_CTRL] "):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                tick = int(parts[1])
                left_omega = float(parts[2])
                right_omega = float(parts[3])
            except ValueError:
                continue
            mode = parts[4]
            left_cmd = left_omega * wheel_radius_m / k_car_wheel_max_speed_ms
            right_cmd = right_omega * wheel_radius_m / k_car_wheel_max_speed_ms
            commands.append(
                {
                    "t_s": tick * 0.0,  # Filled later by caller if needed.
                    "obs_timestamp": push_start_obs_timestamp + tick * 0.0,
                    "left_cmd": left_cmd,
                    "right_cmd": right_cmd,
                    "left_omega_rad_s": left_omega,
                    "right_omega_rad_s": right_omega,
                    "mode": mode,
                    "state": "PUSHING",
                    "path_index": 0,
                    "heading_error_deg": None,
                    "target_point_cm": None,
                    "max_speed_cap": None,
                    "object_id": object_id,
                    "edge_idx": edge_idx,
                    "push_steps": push_steps,
                    "obs_reused": False,
                    "_tick": tick,
                }
            )
    return commands, push_path_cm


def _path_unit_vec_from_path(path_cm: Optional[List[List[float]]]) -> Optional[List[float]]:
    if not path_cm or len(path_cm) < 2:
        return None
    sx, sy = path_cm[0]
    ex, ey = path_cm[-1]
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return [dx / length, dy / length]


def _write_calibration_artifacts(
    artifact_dir: Path,
    mujoco,
    model,
    full_qpos_frames: List[List[float]],
    chain_step: Dict[str, Any],
    namo_config: str,
    output_mp4: str,
    skill_knobs: Dict[str, Any],
    wheel_radius_m: float,
    k_car_wheel_max_speed_ms: float,
) -> None:
    if not full_qpos_frames:
        return

    object_id = str(chain_step["object_id"])
    sim_object_id = str(chain_step.get("sim_object_id") or object_id)
    edge_idx = int(chain_step["edge_idx"])
    push_steps = int(chain_step["push_steps"])

    robot_body_name = "car"
    object_body_name = sim_object_id

    push_tracker_max_speed = float(skill_knobs.get("push_tracker_max_speed", 0.3))
    dynamic_direction = bool(skill_knobs.get("dynamic_direction", True))
    control_steps_per_push = int(skill_knobs.get("control_steps_per_push", 0) or 0)

    timestep_s = float(model.opt.timestep)
    lookahead_distance_cm = PUSH_LOOKAHEAD_RATIO * _car_size_m(mujoco, model) * 100.0

    nav_log_path = artifact_dir / "cxx_nav_raw.log"
    initial_push_start_ts = 0.0
    raw_commands, push_path_cm = _parse_nav_log(
        nav_log_path=nav_log_path,
        wheel_radius_m=wheel_radius_m,
        k_car_wheel_max_speed_ms=k_car_wheel_max_speed_ms,
        push_start_obs_timestamp=initial_push_start_ts,
        object_id=object_id,
        edge_idx=edge_idx,
        push_steps=push_steps,
    )
    push_tick_count = len(raw_commands)
    if push_tick_count == 0 and control_steps_per_push > 0:
        push_tick_count = push_steps * control_steps_per_push
    if push_tick_count <= 0:
        push_tick_count = len(full_qpos_frames)

    settle_total = max(0, len(full_qpos_frames) - push_tick_count)
    pre_settle = settle_total // 2
    post_settle = settle_total - pre_settle
    push_start_idx = pre_settle
    push_end_idx = min(len(full_qpos_frames) - 1, push_start_idx + push_tick_count - 1)
    push_start_obs_timestamp = push_start_idx * timestep_s
    push_end_obs_timestamp = push_end_idx * timestep_s
    wall_end_epoch = dt.datetime.now(tz=dt.timezone.utc).timestamp()
    wall_zero_epoch = wall_end_epoch - max(0, len(full_qpos_frames) - 1) * timestep_s

    def _wall_epoch_for_obs(obs_timestamp: float) -> float:
        return wall_zero_epoch + obs_timestamp

    def _wall_iso_for_obs(obs_timestamp: float) -> str:
        return dt.datetime.fromtimestamp(
            _wall_epoch_for_obs(obs_timestamp),
            tz=dt.timezone.utc,
        ).isoformat().replace("+00:00", "Z")

    for rec in raw_commands:
        tick = int(rec.pop("_tick"))
        rec["t_s"] = tick * timestep_s
        rec["obs_timestamp"] = push_start_obs_timestamp + tick * timestep_s
        rec["max_speed_cap"] = push_tracker_max_speed
        rec["at_epoch"] = _wall_epoch_for_obs(float(rec["obs_timestamp"]))
        rec["at_utc_iso"] = _wall_iso_for_obs(float(rec["obs_timestamp"]))

    data = mujoco.MjData(model)
    mid_obs_records: List[Dict[str, Any]] = []
    scene_object_specs = _collect_scene_object_specs(mujoco, model)

    push_start_robot_pose = None
    push_end_robot_pose = None
    push_start_object_pose = None
    push_end_object_pose = None

    for frame_idx, q in enumerate(full_qpos_frames):
        nq = min(len(q), model.nq)
        if nq > 0:
            data.qpos[:nq] = q[:nq]
        mujoco.mj_forward(model, data)

        robot_pose = _body_pose_cm_deg(mujoco, model, data, robot_body_name)
        if robot_pose is None:
            continue
        objects: Dict[str, Any] = {}
        object_pose = None
        for spec in scene_object_specs:
            pose = _body_pose_cm_deg(mujoco, model, data, str(spec["body_name"]))
            if pose is None:
                continue
            objects[str(spec["display_name"])] = {
                "x_cm": pose[0],
                "y_cm": pose[1],
                "theta_deg": pose[2],
                "width_cm": float(spec["width_cm"]),
                "depth_cm": float(spec["depth_cm"]),
                "height_cm": float(spec["height_cm"]),
                "is_static": bool(spec["is_static"]),
            }
            if spec["sim_name"] == sim_object_id:
                object_pose = pose
        if object_pose is None:
            object_pose = _body_pose_cm_deg(mujoco, model, data, object_body_name)
        if object_pose is None:
            continue

        if frame_idx < push_start_idx:
            push_state = "PRE_SETTLE"
        elif frame_idx <= push_end_idx:
            push_state = "PUSHING"
        else:
            push_state = "POST_SETTLE"
        obs_timestamp = frame_idx * timestep_s

        rec = {
            "at_epoch": _wall_epoch_for_obs(obs_timestamp),
            "at_utc_iso": _wall_iso_for_obs(obs_timestamp),
            "t_since_logger_start_s": obs_timestamp,
            "observation_timestamp": obs_timestamp,
            "robot_pose_cm": [robot_pose[0], robot_pose[1], robot_pose[2]],
            "objects": objects,
            "push_state": push_state,
            "active_subgoal": {
                "object_id": object_id,
                "edge_idx": edge_idx,
                "push_steps": push_steps,
            },
        }
        mid_obs_records.append(rec)

        if frame_idx == push_start_idx:
            push_start_robot_pose = [robot_pose[0], robot_pose[1], robot_pose[2]]
            push_start_object_pose = [object_pose[0], object_pose[1], object_pose[2]]
        if frame_idx == push_end_idx:
            push_end_robot_pose = [robot_pose[0], robot_pose[1], robot_pose[2]]
            push_end_object_pose = [object_pose[0], object_pose[1], object_pose[2]]

    if push_start_robot_pose is None or push_end_robot_pose is None:
        return
    if push_start_object_pose is None or push_end_object_pose is None:
        return

    dx = push_end_object_pose[0] - push_start_object_pose[0]
    dy = push_end_object_pose[1] - push_start_object_pose[1]
    dtheta_deg = ((push_end_object_pose[2] - push_start_object_pose[2] + 180.0) % 360.0) - 180.0
    path_unit_vec = _path_unit_vec_from_path(push_path_cm)
    path_length_cm = None
    if push_path_cm and len(push_path_cm) >= 2:
        sx, sy = push_path_cm[0]
        ex, ey = push_path_cm[-1]
        path_length_cm = math.hypot(ex - sx, ey - sy)
    push_start_wall_epoch = _wall_epoch_for_obs(push_start_obs_timestamp)
    push_end_wall_epoch = _wall_epoch_for_obs(push_end_obs_timestamp)
    push_start_wall_iso = _wall_iso_for_obs(push_start_obs_timestamp)
    push_end_wall_iso = _wall_iso_for_obs(push_end_obs_timestamp)

    subgoal_rec = {
        "subgoal_id": 1,
        "type": "push",
        "object_id": object_id,
        "edge_idx": edge_idx,
        "push_steps": push_steps,
        "dispatched": {
            "at_epoch": push_start_wall_epoch,
            "at_utc_iso": push_start_wall_iso,
        },
        "completed": {
            "at_epoch": push_end_wall_epoch,
            "at_utc_iso": push_end_wall_iso,
        },
        "duration_sec": push_end_obs_timestamp - push_start_obs_timestamp,
        "outcome": "success",
        "dispatched_robot_pose_cm": push_start_robot_pose,
        "completed_robot_pose_cm": push_end_robot_pose,
        "dispatched_obs_timestamp": push_start_obs_timestamp,
        "completed_obs_timestamp": push_end_obs_timestamp,
    }

    push_rec = {
        "at_epoch": push_end_wall_epoch,
        "at_utc_iso": push_end_wall_iso,
        "object_id": object_id,
        "expected_edge": edge_idx,
        "expected_push_steps": push_steps,
        "object_pose_before": push_start_object_pose,
        "object_pose_after": push_end_object_pose,
        "robot_pose_before_cm_deg": push_start_robot_pose,
        "robot_pose_after_cm_deg": push_end_robot_pose,
        "push_start_obs_timestamp": push_start_obs_timestamp,
        "push_end_obs_timestamp": push_end_obs_timestamp,
        "push_path_cm": push_path_cm,
        "push_target_cm": (push_path_cm[-1] if push_path_cm else None),
        "push_path_unit_vec": path_unit_vec,
        "push_path_length_cm": path_length_cm,
        "push_controller_max_speed": push_tracker_max_speed,
        "push_lookahead_distance_cm": lookahead_distance_cm,
        "push_dynamic_direction": dynamic_direction,
        "push_ticks_executed": push_tick_count,
        "delta_pos_cm": [dx, dy],
        "delta_pos_magnitude_cm": math.hypot(dx, dy),
        "delta_theta_deg": dtheta_deg,
        "stuck": False,
        "stuck_threshold_cm": 2.0,
        "stuck_threshold_deg": 15.0,
        "subgoal_id": 1,
        "_sim": {
            "namo_config": namo_config,
            "output_mp4": output_mp4,
            "sim_object_id": sim_object_id,
            "wheel_radius_m": wheel_radius_m,
            "k_car_wheel_max_speed_ms": k_car_wheel_max_speed_ms,
            "control_steps_per_push": control_steps_per_push,
            "push_tracker_max_speed": push_tracker_max_speed,
            "dynamic_direction": dynamic_direction,
            "pre_settle_steps": pre_settle,
            "post_settle_steps": post_settle,
        },
    }

    artifact_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(_QPOS_PATH, artifact_dir / "qpos_dump_full.txt")
    _write_jsonl(artifact_dir / "mid_obs.jsonl", mid_obs_records)
    _write_jsonl(artifact_dir / "wheel_commands.jsonl", raw_commands)
    _write_jsonl(artifact_dir / "subgoals.jsonl", [subgoal_rec])
    _write_jsonl(artifact_dir / "pushes.jsonl", [push_rec])
    _write_json(
        artifact_dir / "sim_replay_artifacts.json",
        {
            "timestep_s": timestep_s,
            "push_tick_count": push_tick_count,
            "pre_settle_steps": pre_settle,
            "post_settle_steps": post_settle,
            "n_qpos_frames": len(full_qpos_frames),
            "n_scene_objects": len(scene_object_specs),
            "wheel_radius_m": wheel_radius_m,
            "k_car_wheel_max_speed_ms": k_car_wheel_max_speed_ms,
        },
    )


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
    artifact_dir_value = chain_doc.get("artifact_dir")
    artifact_dir = Path(artifact_dir_value) if artifact_dir_value else None
    nav_log_path = artifact_dir / "cxx_nav_raw.log" if artifact_dir is not None else None
    # When skip_video is set, we skip the cv2/mujoco.Renderer init and the
    # per-frame MP4 encoding entirely. Physics + qpos dump + Tier 2 artifact
    # extraction still run. Saves ~5-10 s per push and avoids GPU
    # initialization in headless tuning loops.
    skip_video = bool(chain_doc.get("skip_video", False))

    if not skip_video:
        _configure_headless_mujoco_backend()

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

    prior_nav_log = os.environ.get("NAMO_NAV_LOG")
    saved_stderr_fd: Optional[int] = None
    nav_log_fd: Optional[int] = None
    failure_result: Optional[Dict[str, Any]] = None
    try:
        if artifact_dir is not None and nav_log_path is not None:
            artifact_dir.mkdir(parents=True, exist_ok=True)
            os.environ["NAMO_NAV_LOG"] = "1"
            try:
                sys.stderr.flush()
            except Exception:
                pass
            saved_stderr_fd = os.dup(2)
            nav_log_fd = os.open(str(nav_log_path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
            os.dup2(nav_log_fd, 2)

        for idx, step in enumerate(chain, start=1):
            action = namo_rl.Action()
            action.object_id = step.get("sim_object_id") or step["object_id"]
            action.edge_idx = int(step["edge_idx"])
            action.depth = int(step.get("depth", step["push_steps"] - 1))
            action.x = 0.0
            action.y = 0.0
            action.theta = 0.0
            try:
                step_result = env.step(action)
            except Exception as exc:
                print(
                    f"[sim_replay_subprocess] step {idx} raised: {exc!r}",
                    flush=True,
                )
                return 1
            # env.step() returns StepResult{done, reward, info} even when the
            # skill logically rejects (e.g. "Requested edge N not reachable" at
            # namo_push_skill.cpp:170). `done` is the success bit
            # (rl_env.cpp:278 sets done = result.success). Previously the
            # return value was discarded and the chain plowed past silent
            # failures from a state the downstream pushes weren't expecting.
            # Stop the chain on the first logical failure so the artifact set
            # reflects what actually ran, not a half-evolved scene.
            if not step_result.done:
                reason = (step_result.info or {}).get(
                    "failure_reason", "unspecified"
                )
                print(
                    f"[sim_replay_subprocess] step {idx} "
                    f"(object={action.object_id} edge={action.edge_idx} "
                    f"depth={action.depth}) FAILED: {reason}. "
                    f"Halting chain (remaining {len(chain) - idx} step(s) skipped).",
                    flush=True,
                )
                failure_result = {
                    "step_index": idx,
                    "object_id": action.object_id,
                    "edge_idx": action.edge_idx,
                    "depth": action.depth,
                    "info": dict(step_result.info or {}),
                }
                break
    finally:
        if saved_stderr_fd is not None:
            try:
                sys.stderr.flush()
            except Exception:
                pass
            os.dup2(saved_stderr_fd, 2)
            os.close(saved_stderr_fd)
        if nav_log_fd is not None:
            os.close(nav_log_fd)
        if prior_nav_log is None:
            os.environ.pop("NAMO_NAV_LOG", None)
        else:
            os.environ["NAMO_NAV_LOG"] = prior_nav_log

    qpos_frames = _read_qpos_dump(_QPOS_PATH)
    if failure_result is not None and not qpos_frames:
        try:
            current_state = env.get_full_state()
            if getattr(current_state, "qpos", None):
                qpos_frames = [list(current_state.qpos)]
                print(
                    "[sim_replay_subprocess] failure produced no qpos dump; "
                    "using current env full_state for a static failure frame",
                    flush=True,
                )
        except Exception as exc:
            print(
                f"[sim_replay_subprocess] WARN: failed to capture full_state "
                f"after failure: {exc!r}",
                flush=True,
            )
    if not qpos_frames:
        print(
            "[sim_replay_subprocess] qpos dump was empty — the C++ push "
            "primitive may not have run dump_qpos. Check that NAMO_QPOS_DUMP "
            f"is honoured by the build at {_QPOS_PATH}",
            flush=True,
        )
        return 1

    # MuJoCo model load (needed for both video rendering and artifact extraction).
    # Renderer + cv2 are only needed for video — skipped under skip_video.
    import mujoco

    xml_str = _inject_offscreen_size(Path(xml_path).read_text(), _WIDTH, _HEIGHT)
    try:
        model = mujoco.MjModel.from_xml_string(xml_str)
        data = mujoco.MjData(model)
    except Exception as exc:
        print(
            f"[sim_replay_subprocess] mujoco model init failed: {exc!r}",
            flush=True,
        )
        return 1

    full_qpos_frames = list(qpos_frames)

    if not skip_video:
        import cv2

        try:
            renderer = mujoco.Renderer(model, height=_HEIGHT, width=_WIDTH)
        except Exception as exc:
            print(
                f"[sim_replay_subprocess] mujoco renderer init failed: {exc!r}",
                flush=True,
            )
            return 1

        # Stride = ticks per video frame so playback is 1× wall-clock. Read the
        # timestep off the loaded model (car: 0.002s → 17, sphere: 0.01s → 3).
        stride = max(1, int(round((1.0 / _FPS) / model.opt.timestep)))
        qpos_frames_video = qpos_frames[::stride]

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

        for q in qpos_frames_video:
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
        _maybe_reencode_mp4_for_vscode_compat(output_mp4)
    else:
        print("[sim_replay_subprocess] skip_video set; bypassing MP4 encoding",
              flush=True)
    if artifact_dir is not None:
        wheel_radius_m = 0.015
        wheel_geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "left_wheel_collision",
        )
        if wheel_geom_id >= 0:
            wheel_radius_m = float(model.geom_size[wheel_geom_id][0])
        try:
            _write_calibration_artifacts(
                artifact_dir=artifact_dir,
                mujoco=mujoco,
                model=model,
                full_qpos_frames=full_qpos_frames,
                chain_step=chain[0],
                namo_config=namo_config,
                output_mp4=output_mp4,
                skill_knobs=_load_skill_knobs(namo_config),
                wheel_radius_m=wheel_radius_m,
                k_car_wheel_max_speed_ms=1.0,
            )
        except Exception as exc:
            print(
                f"[sim_replay_subprocess] calibration artifact export failed: {exc!r}",
                flush=True,
            )
            return 1
        if failure_result is not None:
            _write_json(
                artifact_dir / "failure_result.json",
                failure_result,
            )
    try:
        os.unlink(_QPOS_PATH)
    except OSError:
        pass

    if failure_result is not None:
        status_msg = (
            f"[sim_replay_subprocess] rendered failure context to {output_mp4} "
            f"(failed at step {failure_result['step_index']})"
            if not skip_video else
            "[sim_replay_subprocess] captured failure context without video"
        )
        print(status_msg, flush=True)
        return 2

    if skip_video:
        print(
            f"[sim_replay_subprocess] no video written (skip_video); "
            f"{len(qpos_frames)} qpos frames processed",
            flush=True,
        )
    else:
        print(
            f"[sim_replay_subprocess] wrote {output_mp4} "
            f"({len(qpos_frames)} frames @ {_FPS} fps, stride={stride})",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
