#!/usr/bin/env python3
"""Run NAMO planning and execution.

This script runs the full NAMO pipeline:
1. Captures current robot/object state
2. Generates MuJoCo XML for NAMO planning
3. Runs NAMO planning (RegionOpeningPlanner)
4. Executes generated PushSubgoals on robot

Goal can be specified via --goal or detected from goal marker (ArUco 6x6, ID 0).
In simulation mode, --goal is required. In real mode, goal marker detection is used by default.

Usage:
    # Real robot mode (uses detected goal marker)
    python scripts/run_namo.py --config config/real.yaml

    # Interactive mode with step-by-step prompts
    python scripts/run_namo.py --config config/real.yaml --interactive

    # Real robot with manual goal override
    python scripts/run_namo.py --config config/real.yaml --goal 50 40

    # Simulation mode (--goal required)
    python scripts/run_namo.py --sim --goal 50 40

    # Simulation with verbose output
    python scripts/run_namo.py --sim --goal 50 40 -v

    # Debug mode (save XML to file)
    python scripts/run_namo.py --sim --goal 50 40 --debug-xml /tmp/namo_env.xml
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from robot_control import Runtime, RuntimeConfig, SimConfig
from robot_control.core.object_defs import ObjectDef
from robot_control.planner.namo_planner import CANONICAL_OPEN_FRACTION
from robot_control.planner.region_target import (
    ADVANCE_EXHAUSTED,
    ADVANCE_NO_BOUNDARY,
    ADVANCE_PLANNED,
    STATUS_EXHAUSTED,
    STATUS_OPENED,
    RegionOpeningTarget,
    advance_boundary,
)
from robot_control.planner import (
    BEST_FIRST_PRIOR_CHOICES,
    DEFAULT_BEST_FIRST_PRIOR,
    DEFAULT_LOCAL_SEARCH,
    LOCAL_SEARCH_CHOICES,
    LocalSearchConfig,
    NAMOPlanner,
)
from robot_control.planner.namo_binding_loader import load_canonical_namo_rl


# Sentinel for --record-video when passed with no value. Resolved to
# <diag-root>/recordings/ after diagnostics bootstrap.
_RECORD_VIDEO_DEFAULT_SENTINEL = "__USE_DIAG_PATH__"


def local_search_from_args(args) -> LocalSearchConfig:
    """Resolve the local-search selection from parsed CLI args.

    Constructed once per run so an invalid combination (best_first + model with
    no checkpoint) raises before the robot moves, rather than deep inside the
    planner on the first replan.
    """
    return LocalSearchConfig(
        local_search=args.local_search,
        best_first_prior=args.best_first_prior,
        scorer_ckpt=args.scorer_ckpt,
        best_first_hmax=args.best_first_hmax,
        keyhole_simulation_budget=args.keyhole_simulation_budget,
        ml_device=args.ml_device,
    )


def find_namo_config(scale_factor: float = 6.0, robot_model: str = "sphere") -> str:
    """Find NAMO config file relative to this script.

    Picks the config based on (scale_factor, robot_model):
      sphere + 1×   → namo_config_complete_skill15_1x.yaml
      sphere + 6×   → namo_config_complete_skill15.yaml  (legacy)
      car    + 1×   → namo_config_complete_skill15_car_1x.yaml
      car    + 6×   → unsupported (only the 1× car primitives exist)

    The car config sets robot_type: diff_drive so the C++ side instantiates
    DiffDriveAdapter, and points to the car-specific motion_primitives_1x_car
    .dat files. Mixing sphere config with --robot-model car would load
    sphere primitives — useless for car physics.
    """
    one_x = abs(scale_factor - 1.0) < 1e-9
    if robot_model == "car":
        if not one_x:
            raise ValueError(
                f"--robot-model car only supports --scale-factor 1.0 today "
                f"(got {scale_factor}); the 6× car primitive set was not "
                f"generated."
            )
        config_name = "namo_config_complete_skill15_car_1x.yaml"
    elif one_x:
        config_name = "namo_config_complete_skill15_1x.yaml"
    else:
        config_name = "namo_config_complete_skill15.yaml"

    script_dir = Path(__file__).parent
    robot_control_dir = script_dir.parent

    # Try relative to robot_control
    namo_config = robot_control_dir.parent / "namo_cpp" / "config" / config_name
    if namo_config.exists():
        return str(namo_config)

    # Try from workspace root
    workspace_root = robot_control_dir.parent
    namo_config = workspace_root / "namo_cpp" / "config" / config_name
    if namo_config.exists():
        return str(namo_config)

    raise FileNotFoundError(
        f"Could not find {config_name}. "
        "Please specify --namo-config path."
    )


def find_primitive_data_dir() -> str:
    """Find primitive data directory relative to this script."""
    script_dir = Path(__file__).parent
    robot_control_dir = script_dir.parent

    # Try relative to robot_control
    data_dir = robot_control_dir.parent / "namo_cpp" / "data"
    if data_dir.exists():
        return str(data_dir)

    # Fallback to "data" (relative to namo_cpp)
    return "data"


def get_namo_paths() -> Tuple[Path, Path, Path]:
    """Get important paths for NAMO integration."""
    script_dir = Path(__file__).parent
    robot_control_dir = script_dir.parent
    namo_root = robot_control_dir.parent
    namo_cpp_dir = namo_root / "namo_cpp"
    return robot_control_dir, namo_root, namo_cpp_dir


def setup_namo_imports():
    """Setup Python path for NAMO imports."""
    load_canonical_namo_rl(Path(__file__).resolve())


def load_camera_config(config_path: str, objects_path: str):
    """Load camera and observer configuration."""
    from robot_control.camera import ObserverConfig
    from robot_control.camera.observer import ObjectDefinition
    from robot_control.nodes import CameraConfig

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    camera_cfg = config.get("camera", {})
    robot_cfg = config.get("robot", {})
    workspace_cfg = config.get("workspace", {})

    # Parse marker offset
    marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
    if isinstance(marker_offset, list) and len(marker_offset) == 2:
        marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
    else:
        marker_offset_tuple = (0.0, 0.0)

    # Load object definitions
    object_defs = {}
    objects_file = Path(objects_path)
    if objects_file.exists():
        with open(objects_file, "r") as f:
            obj_config = yaml.safe_load(f)
        for name, obj_cfg in obj_config.get("objects", {}).items():
            obj_type = obj_cfg.get("type", "movable")
            shape = obj_cfg.get("shape", {})
            offset = obj_cfg.get("marker_offset", {})
            object_defs[name] = ObjectDefinition(
                marker_id=obj_cfg["marker_id"],
                is_static=(obj_type == "static"),
                is_goal=(obj_type == "goal"),
                width_cm=shape.get("width", 0.0),
                depth_cm=shape.get("depth", 0.0),
                height_cm=shape.get("height", 0.0),
                marker_offset_x_cm=offset.get("x", 0.0),
                marker_offset_y_cm=offset.get("y", 0.0),
            )

    object_marker_size = config.get("object_marker_size_mm") or robot_cfg.get("object_marker_size_mm", 30.0)

    camera_config = CameraConfig(
        camera_device=camera_cfg.get("device", 0),
        resolution=camera_cfg.get("resolution", "720p"),
        fps=camera_cfg.get("fps", 60),
        exposure=camera_cfg.get("exposure", -6),
        calibration_file=camera_cfg.get("calibration_file", ""),
    )

    observer_config = ObserverConfig(
        calibration_file=camera_cfg.get("calibration_file", ""),
        robot_marker_id=robot_cfg.get("marker_id", 1),
        robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
        marker_to_wheel_offset_cm=marker_offset_tuple,
        object_defs=object_defs,
        object_marker_size_mm=object_marker_size,
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    return camera_config, observer_config


# --------------------------------------------------------------------- --sim-xml support


def _quat_to_yaw_deg(quat_str: str) -> float:
    """Yaw (degrees) from MuJoCo quat string 'w x y z'. Z-axis rotation only."""
    parts = quat_str.split()
    if len(parts) != 4:
        return 0.0
    w, x, y, z = (float(v) for v in parts)
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def _robot_pose_from_real_run(run_dir: Path) -> Optional[Tuple[float, float, float]]:
    """Read the first ArUco-populated frame in mid_obs.jsonl and return
    (x_cm, y_cm, theta_deg). Mirrors execute_sim_push._load_initial_scene_state
    intentionally — keeping run_namo self-contained instead of cross-importing
    from another script.
    """
    mid_obs_path = run_dir / "mid_obs.jsonl"
    if not mid_obs_path.exists():
        return None
    with open(mid_obs_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(rec.get("objects"), dict) or not rec["objects"]:
                continue
            for key in ("robot_pose_cm_deg", "robot_pose_cm"):
                val = rec.get(key)
                if isinstance(val, list) and len(val) >= 2:
                    return (
                        float(val[0]),
                        float(val[1]),
                        float(val[2]) if len(val) >= 3 else 0.0,
                    )
    return None


def _first_scene_record_from_mid_obs(run_dir: Path) -> Optional[Dict[str, Any]]:
    """Return the first scene-bearing record from ``mid_obs.jsonl``."""
    mid_obs_path = run_dir / "mid_obs.jsonl"
    if not mid_obs_path.exists():
        return None
    with open(mid_obs_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(rec.get("objects"), dict) and rec["objects"]:
                return rec
    return None


def _robot_pose_from_scene_record(
    scene_record: Dict[str, Any],
) -> Optional[Tuple[float, float, float]]:
    """Extract ``(x_cm, y_cm, theta_deg)`` from a scene record."""
    for key in ("robot_pose_cm_deg", "robot_pose_cm"):
        val = scene_record.get(key)
        if isinstance(val, list) and len(val) >= 2:
            return (
                float(val[0]),
                float(val[1]),
                float(val[2]) if len(val) >= 3 else 0.0,
            )
    return None


def _goal_from_scene_record(
    scene_record: Dict[str, Any],
) -> Optional[Tuple[float, float]]:
    """Extract ``(x_cm, y_cm)`` goal from a scene record."""
    goal = scene_record.get("goal_cm")
    if isinstance(goal, list) and len(goal) >= 2:
        return float(goal[0]), float(goal[1])
    return None


def _goal_from_xml(xml_path: Path) -> Optional[Tuple[float, float]]:
    """Read the goal site from a MuJoCo XML, if present."""
    root = ET.parse(str(xml_path)).getroot()
    for site in root.iter("site"):
        if site.get("name") == "goal":
            pos = [float(v) for v in site.get("pos", "0 0 0").split()]
            return pos[0] * 100.0, pos[1] * 100.0
    return None


def _observation_from_scene_record(
    scene_record: Dict[str, Any],
    default_robot_pose_cm_deg: Optional[Tuple[float, float, float]] = None,
    default_goal_cm: Optional[Tuple[float, float]] = None,
):
    """Build an Observation from a captured mid_obs scene record."""
    from robot_control.core.types import Observation, ObjectPose

    robot_pose = _robot_pose_from_scene_record(scene_record) or default_robot_pose_cm_deg
    if robot_pose is None:
        return None

    goal_cm = _goal_from_scene_record(scene_record) or default_goal_cm
    if goal_cm is None:
        return None

    scene_objects = scene_record.get("objects")
    if not isinstance(scene_objects, dict) or not scene_objects:
        return None

    objects: Dict[str, ObjectPose] = {}
    for name, raw_obj in scene_objects.items():
        if not isinstance(raw_obj, dict):
            continue
        if "pose_cm" in raw_obj:
            pose = raw_obj.get("pose_cm")
            if not isinstance(pose, list) or len(pose) < 3:
                continue
            x_cm = float(pose[0])
            y_cm = float(pose[1])
            theta_deg = float(pose[2])
        else:
            try:
                x_cm = float(raw_obj["x_cm"])
                y_cm = float(raw_obj["y_cm"])
            except (KeyError, TypeError, ValueError):
                continue
            theta_deg = float(raw_obj.get("theta_deg", 0.0))
        try:
            width_cm = float(raw_obj["width_cm"])
            depth_cm = float(raw_obj["depth_cm"])
            height_cm = float(raw_obj["height_cm"])
        except (KeyError, TypeError, ValueError):
            continue
        objects[name] = ObjectPose(
            x=x_cm,
            y=y_cm,
            theta=theta_deg,
            width=width_cm,
            depth=depth_cm,
            height=height_cm,
            is_static=bool(raw_obj.get("is_static", False)),
        )

    if not objects:
        return None

    return Observation(
        robot_x=float(robot_pose[0]),
        robot_y=float(robot_pose[1]),
        robot_theta=float(robot_pose[2]),
        objects=objects,
        timestamp=0.0,
        goal_x=float(goal_cm[0]),
        goal_y=float(goal_cm[1]),
    )


def _sim_config_from_xml(
    xml_path: Path,
    robot_pose_cm_deg: Optional[Tuple[float, float, float]] = None,
    car_width_cm: float = 7.0,
    car_height_cm: float = 7.0,
) -> Tuple[SimConfig, Optional[Tuple[float, float]]]:
    """Build a SimConfig from a car-format MuJoCo XML.

    Designed for the initial_scene.xml that execute_sim_push --trial-spec
    emits per real_test_envs scene. Extracts:

      * Workspace bounds from boundary walls (wall_1..wall_4 — same
        convention NAMOXMLGenerator emits and the C++ env uses).
      * Movables from every ``<body name='obstacle_N_movable'>``.
      * Statics from every ``<body name='wall_N'>`` with N >= 5
        (wall_1..4 are boundaries, already encoded in workspace dims).
      * Goal from ``<site name='goal'>`` if present; returned separately
        so the caller can override via --goal.

    Robot pose: the car body's freejoint spawn is baked at (0,0) inside
    little_car.xml and can't be overridden via include — exactly the
    constraint that drove _build_initial_scene_xml in execute_sim_push.
    Caller must pass ``robot_pose_cm_deg`` (typically from
    ``--sim-real-run-dir/mid_obs.jsonl[0]``). Falls back to workspace
    center with a warning if None.
    """
    root = ET.parse(str(xml_path)).getroot()

    # Workspace bounds from wall_1..wall_4.
    walls: Dict[str, Tuple[List[float], List[float]]] = {}
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if name in ("wall_1", "wall_2", "wall_3", "wall_4"):
            pos = [float(v) for v in geom.get("pos", "0 0 0").split()]
            size = [float(v) for v in geom.get("size", "0 0 0").split()]
            walls[name] = (pos, size)
    if len(walls) != 4:
        raise ValueError(
            f"{xml_path}: expected 4 boundary walls wall_1..wall_4, "
            f"found {sorted(walls.keys())}"
        )
    # Inner workspace edges (walls are placed outside the bounds):
    #   wall_1 = left:   inner edge x = pos.x + size.x
    #   wall_2 = right:  inner edge x = pos.x - size.x
    #   wall_3 = bottom: inner edge y = pos.y + size.y
    #   wall_4 = top:    inner edge y = pos.y - size.y
    w1_pos, w1_sz = walls["wall_1"]
    w2_pos, w2_sz = walls["wall_2"]
    w3_pos, w3_sz = walls["wall_3"]
    w4_pos, w4_sz = walls["wall_4"]
    x_min = (w1_pos[0] + w1_sz[0]) * 100.0
    x_max = (w2_pos[0] - w2_sz[0]) * 100.0
    y_min = (w3_pos[1] + w3_sz[1]) * 100.0
    y_max = (w4_pos[1] - w4_sz[1]) * 100.0
    workspace_w_cm = x_max - x_min
    workspace_h_cm = y_max - y_min

    # Movables and interior statics.
    objects: Dict[str, Tuple[float, float, float]] = {}
    object_defs: Dict[str, ObjectDef] = {}
    for body in root.iter("body"):
        bname = body.get("name", "")
        is_obstacle = bname.startswith("obstacle_") and bname.endswith("_movable")
        is_wall_n = (
            bname.startswith("wall_") and bname[5:].isdigit() and int(bname[5:]) >= 5
        )
        if not (is_obstacle or is_wall_n):
            continue
        geom = body.find("geom")
        if geom is None:
            continue
        pos = [float(v) for v in geom.get("pos", "0 0 0").split()]
        size = [float(v) for v in geom.get("size", "0 0 0").split()]
        theta_deg = _quat_to_yaw_deg(geom.get("quat", "1 0 0 0"))
        # NAMOXMLGenerator emits: half_width=depth/200, half_depth=width/200.
        # Invert: depth_cm = size[0]*2*100, width_cm = size[1]*2*100.
        depth_cm = size[0] * 200.0
        width_cm = size[1] * 200.0
        height_cm = size[2] * 200.0
        x_cm = pos[0] * 100.0
        y_cm = pos[1] * 100.0
        objects[bname] = (x_cm, y_cm, theta_deg)
        object_defs[bname] = ObjectDef(
            name=bname,
            width=width_cm,
            depth=depth_cm,
            height=height_cm,
            is_static=is_wall_n,
        )

    # Goal site (optional). The user-baked value is at (0.37, 0.67, 0.0) m.
    goal_cm = _goal_from_xml(xml_path)

    # Robot pose: prefer the caller-supplied (real-side ground truth);
    # otherwise default to workspace center with a warning.
    if robot_pose_cm_deg is not None:
        rx, ry, rtheta = robot_pose_cm_deg
    else:
        rx = x_min + workspace_w_cm / 2.0
        ry = y_min + workspace_h_cm / 2.0
        rtheta = 0.0
        print(
            f"[run_namo] WARNING: --sim-xml without --sim-real-run-dir; "
            f"defaulting robot to workspace center "
            f"({rx:.1f},{ry:.1f}) — pass --sim-real-run-dir for the "
            f"real-side ArUco starting pose.",
            flush=True,
        )

    sim_config = SimConfig(
        x=rx, y=ry, theta=rtheta,
        width=workspace_w_cm,
        height=workspace_h_cm,
        car_width=car_width_cm,
        car_height=car_height_cm,
        offset_w=car_width_cm / 2.0,
        offset_h=car_height_cm / 2.0,
        wheel_base=car_width_cm,
        objects=objects,
        object_defs=object_defs,
    )
    return sim_config, goal_cm


def _robot_dims_from_namo_config(namo_config_path: str) -> Tuple[float, float]:
    """Read planning.robot_size from a namo_cpp config and return
    ``(width_cm, height_cm)`` (full extents). The C++ config stores
    half-extents in meters under planning.robot_size = [half_x, half_y].
    """
    with open(namo_config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}
    planning = (cfg.get("planning") or {})
    rs = planning.get("robot_size")
    if not isinstance(rs, list) or len(rs) != 2:
        raise ValueError(
            f"{namo_config_path}: planning.robot_size missing or malformed "
            f"(expected [half_x_m, half_y_m]); got {rs!r}"
        )
    half_x_m = float(rs[0])
    half_y_m = float(rs[1])
    return (half_x_m * 200.0, half_y_m * 200.0)  # half_m -> full_cm


def _render_plan_only_mp4(
    diag_root: Path,
    scene_xml_path: Path,
    plan_subgoals: list,
    namo_config_path: str,
    observation_robot_pose_cm_deg: Tuple[float, float, float],
    real_to_sim: Optional[Dict[str, str]] = None,
) -> None:
    """Render the planner-returned chain into ``<diag_root>/sim_push.mp4``.

    Uses the same C++ NAMOPushController + sim_replay pipeline that
    ``execute_sim_push --trial-spec`` uses for its videos. The plan is
    already MuJoCo-verified by the planner, so this is a faithful replay
    of what the planner saw — not a separate re-execution under
    different physics.

    ``observation_robot_pose_cm_deg`` must be the same pose the planner
    saw (= mid_obs[0] robot pose). namo_bridge.py:400-419 teleports the
    car there before plan_from_xml runs the chain. If we teleport
    somewhere else (e.g. the edge point of push 1), push 1's end state
    diverges, push 2 starts wrong, and the chain visually fails to
    reach the goal even though the planner verified it succeeds.
    """
    import math as _math
    from robot_control.diagnostics.sim_replay import render_chain_to_mp4

    if not plan_subgoals:
        return

    # Chain payload format render_chain_to_mp4 / sim_replay_subprocess expect.
    chain: List[Dict[str, Any]] = []
    for sg in plan_subgoals:
        oid = getattr(sg, "object_id", None)
        sim_oid = (
            str(real_to_sim.get(str(oid), oid))
            if real_to_sim is not None
            else str(oid)
        )
        edge = int(getattr(sg, "edge_idx"))
        steps = int(getattr(sg, "push_steps"))
        chain.append({
            "object_id": oid,
            "sim_object_id": sim_oid,
            "edge_idx": edge,
            "push_steps": steps,
            "depth": steps - 1,
        })

    # Starting pose = the planner's observation pose, converted to sim
    # units (meters + radians). Matches what the planner used during
    # verification — see _cm_to_sim equivalent in namo_bridge.py.
    rx_cm, ry_cm, rtheta_deg = observation_robot_pose_cm_deg
    starting_pose_sim = (
        float(rx_cm) / 100.0,
        float(ry_cm) / 100.0,
        _math.radians(float(rtheta_deg)),
    )

    # Resolve every path to absolute BEFORE the chdir, otherwise any
    # relative input (e.g. "real_test_envs/...") gets re-rooted under
    # namo_cpp/ and load fails.
    abs_sim_xml = str(Path(scene_xml_path).resolve())
    abs_namo_config = str(Path(namo_config_path).resolve())
    abs_output_mp4 = str((diag_root / "sim_push.mp4").resolve())
    abs_artifact_dir = str(diag_root.resolve())

    # sim_replay_subprocess resolves motion-primitive db paths from cwd
    # (same constraint NAMOPlanBridge has). Chdir into namo_cpp/ for the
    # duration of the call so those resolve correctly.
    here = Path(__file__).resolve()
    namo_cpp_dir = here.parents[2] / "namo_cpp"
    prior_cwd = os.getcwd()
    if namo_cpp_dir.exists():
        os.chdir(str(namo_cpp_dir))
    try:
        rendered = render_chain_to_mp4(
            start_xml=abs_sim_xml,
            namo_config=abs_namo_config,
            chain=chain,
            output_mp4=abs_output_mp4,
            artifact_dir=abs_artifact_dir,
            starting_robot_pose_sim=starting_pose_sim,
            skip_video=False,
        )
    finally:
        os.chdir(prior_cwd)
    if rendered is None:
        print(
            "[run_namo plan-only] sim replay returned None — see "
            "[sim_replay_subprocess] lines above. MP4 not written.",
            flush=True,
        )
        return
    print(f"[run_namo plan-only] sim_push.mp4 -> {abs_output_mp4}", flush=True)


def _emit_plan_only_solution_yaml(
    diag_root: Path,
    goal_cm: Tuple[float, float],
    algorithm: str,
    strategy: str,
    plan_subgoals: list,
    search_time_ms: float,
    sim_pushes_tried: Optional[int] = None,
    object_mapping: Optional[Dict[str, Dict[str, str]]] = None,
    planner_scene_xml: Optional[str] = None,
    outcome_override: Optional[str] = None,
) -> None:
    """Write solution.yaml from a planner-returned chain (plan-only mode).

    The planner's NAMOPushSkill executes every primitive through the C++
    MuJoCo controller before deeming the chain a "plan". So a non-empty
    plan is the sim-success result — we record it as-is, no separate
    runtime execution / SimEnv pass needed (and the runtime's kinematic
    SimEnv physics doesn't match MuJoCo anyway, which is what was making
    valid plans look like execution failures).
    """
    plan_list: List[Dict[str, Any]] = []
    for sg in plan_subgoals:
        plan_list.append({
            "object_id": getattr(sg, "object_id", None),
            "edge_idx": int(getattr(sg, "edge_idx")) if hasattr(sg, "edge_idx") else None,
            "push_steps": int(getattr(sg, "push_steps")) if hasattr(sg, "push_steps") else None,
        })
    success = bool(plan_list)
    payload = {
        "success": success,
        "outcome": outcome_override
        or ("success" if success else "planner returned no plan"),
        "goal_cm": [float(goal_cm[0]), float(goal_cm[1])],
        "algorithm": algorithm,
        "strategy": strategy,
        "plan": plan_list,
        "search_stats": {
            "search_time_ms": float(search_time_ms),
            "pushes_in_plan": len(plan_list),
            "sim_pushes_tried": (
                int(sim_pushes_tried) if sim_pushes_tried is not None else None
            ),
        },
    }
    if object_mapping is not None:
        payload["object_mapping"] = object_mapping
    if planner_scene_xml is not None:
        payload["planner_scene_xml"] = planner_scene_xml
    diag_root.mkdir(parents=True, exist_ok=True)
    out_path = diag_root / "solution.yaml"
    out_path.write_text(yaml.safe_dump(payload, sort_keys=False))
    print(
        f"[run_namo plan-only] solution.yaml -> {out_path} "
        f"(success={success}, pushes={len(plan_list)}, "
        f"search={search_time_ms:.0f}ms, "
        f"sim_pushes_tried={sim_pushes_tried})",
        flush=True,
    )


def detect_goal_from_camera(
    config_path: str,
    objects_path: str = "config/objects.yaml",
    timeout: float = 5.0,
    camera_service: Optional[str] = None,
) -> Optional[Tuple[float, float]]:
    """Detect goal marker position from camera.

    If camera_service address is provided, connects via ZMQ instead of
    creating a local camera + observer.
    """
    print("\n" + "=" * 60)
    print("STEP 0: Detecting Goal Marker")
    print("=" * 60)
    print("Looking for goal marker (ArUco 6x6, ID 0)...")

    if camera_service:
        from robot_control.nodes.remote_observer import RemoteObserverNode

        node = RemoteObserverNode(camera_service)
        if not node.start():
            print("  ERROR: Failed to connect to camera service!")
            return None

        goal_pos = None
        start_time = time.time()
        try:
            while time.time() - start_time < timeout:
                obs = node.get()
                if obs is not None and obs.goal_x is not None and obs.goal_y is not None:
                    goal_pos = (obs.goal_x, obs.goal_y)
                    break
                time.sleep(0.05)
        finally:
            node.stop()
    else:
        from robot_control.camera import ArucoObserver
        from robot_control.nodes import CameraSensorNode

        camera_config, observer_config = load_camera_config(config_path, objects_path)

        camera = CameraSensorNode(camera_config)
        if not camera.start():
            print("  ERROR: Failed to start camera!")
            return None

        observer = ArucoObserver(observer_config)
        if not observer.start():
            print("  ERROR: Failed to start observer!")
            camera.stop()
            return None

        goal_pos = None
        start_time = time.time()
        try:
            while time.time() - start_time < timeout:
                obs = observer.get()
                if obs is not None and obs.goal_x is not None and obs.goal_y is not None:
                    goal_pos = (obs.goal_x, obs.goal_y)
                    break
                time.sleep(0.05)
        finally:
            observer.stop()
            camera.stop()

    if goal_pos:
        print(f"  SUCCESS: Goal marker detected at ({goal_pos[0]:.1f}, {goal_pos[1]:.1f}) cm")
    else:
        print("  ERROR: Goal marker not detected!")

    return goal_pos


def capture_scene_interactive(
    config_path: str,
    objects_path: str,
    stable_frames: int = 10,
    camera_service: Optional[str] = None,
):
    """Capture scene from camera with visual feedback.

    If camera_service address is provided, polls the remote observer
    and uses terminal-based capture (press Enter) instead of OpenCV window.
    The operator can monitor the camera via the camera_service --show window.
    """
    print("\n" + "=" * 60)
    print("STEP 1: Capture Scene from Camera")
    print("=" * 60)

    if camera_service:
        from robot_control.nodes.remote_observer import RemoteObserverNode

        print("Connecting to camera service for scene capture...")
        print("Monitor the camera in the camera_service window.")
        print("Press Enter to capture when ready, or 'q'+Enter to quit.")

        node = RemoteObserverNode(camera_service)
        if not node.start():
            print("  ERROR: Failed to connect to camera service!")
            return None

        stable_count = 0
        last_obs = None
        captured_obs = None

        import select
        import sys

        try:
            while captured_obs is None:
                obs = node.get()
                if obs is not None:
                    if last_obs is not None:
                        dx = abs(obs.robot_x - last_obs.robot_x)
                        dy = abs(obs.robot_y - last_obs.robot_y)
                        if dx < 1.0 and dy < 1.0:
                            stable_count += 1
                        else:
                            stable_count = 0
                    last_obs = obs

                    goal_str = (
                        f"({obs.goal_x:.1f}, {obs.goal_y:.1f})"
                        if obs.goal_x is not None
                        else "N/A"
                    )
                    print(
                        f"\r  Robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ "
                        f"{obs.robot_theta:.1f} | Objects: {len(obs.objects)} | "
                        f"Goal: {goal_str} | Stable: {stable_count}/{stable_frames}  ",
                        end="",
                        flush=True,
                    )

                # Non-blocking stdin check
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    line = sys.stdin.readline().strip()
                    if line.lower() == "q":
                        print("\n\nCancelled by user.")
                        break
                    elif obs is not None:
                        captured_obs = obs
                        print("\n\n  SUCCESS: Scene captured!")
                else:
                    time.sleep(0.05)
        finally:
            node.stop()

        return captured_obs

    # Local camera mode (original behavior)
    import cv2
    from robot_control.camera import ArucoObserver
    from robot_control.nodes import CameraSensorNode

    camera_config, observer_config = load_camera_config(config_path, objects_path)

    print("Starting camera and ArUco detection...")
    print("Press 'c' to capture, 'q' to quit")

    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("  ERROR: Failed to start camera!")
        return None

    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("  ERROR: Failed to start observer!")
        camera.stop()
        return None

    cv2.namedWindow("Capture Scene", cv2.WINDOW_NORMAL)

    stable_count = 0
    last_obs = None
    captured_obs = None

    try:
        while captured_obs is None:
            vis = observer.get_vis_frame()
            if vis is not None:
                cv2.putText(vis, f"Stable: {stable_count}/{stable_frames} - Press 'c' to capture",
                           (10, vis.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Capture Scene", vis)

            obs = observer.get()
            if obs is not None:
                if last_obs is not None:
                    dx = abs(obs.robot_x - last_obs.robot_x)
                    dy = abs(obs.robot_y - last_obs.robot_y)
                    if dx < 1.0 and dy < 1.0:
                        stable_count += 1
                    else:
                        stable_count = 0
                last_obs = obs

                goal_str = f"({obs.goal_x:.1f}, {obs.goal_y:.1f})" if obs.goal_x is not None else "N/A"
                print(f"\r  Robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ {obs.robot_theta:.1f}° | "
                      f"Objects: {len(obs.objects)} | Goal: {goal_str} | Stable: {stable_count}/{stable_frames}  ",
                      end="", flush=True)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                print("\n\nCancelled by user.")
                break
            elif key == ord('c') and obs is not None:
                captured_obs = obs
                print("\n\n  SUCCESS: Scene captured!")

            time.sleep(0.01)

    finally:
        cv2.destroyAllWindows()
        observer.stop()
        camera.stop()

    return captured_obs


def run_interactive_mode(args):
    """Run NAMO in interactive step-by-step mode.

    Uses NAMOPlanner for execution, which includes:
    - Reachability checking before/after pushes
    - Automatic navigation to goal when reachable
    - Re-planning if goal becomes blocked
    """
    robot_control_dir, namo_root, namo_cpp_dir = get_namo_paths()

    # Change to robot_control directory
    os.chdir(str(robot_control_dir))

    print("\n" + "=" * 60)
    print("NAMO Interactive Mode")
    print("=" * 60)
    print(f"Mode: real")
    print(f"Config: {args.config}")
    print(f"Objects: {args.objects}")
    print("=" * 60)

    input("\n[Press Enter to start STEP 1: Capture Scene...]")

    # =========================================================================
    # STEP 1: Capture scene
    # =========================================================================
    obs = capture_scene_interactive(
        args.config, args.objects,
        camera_service=getattr(args, "camera_service", None),
    )
    if obs is None:
        print("ERROR: No observation captured!")
        return 1

    # Determine goal
    if args.goal is not None:
        goal_cm = (args.goal[0], args.goal[1])
        goal_source = "command line"
    elif obs.goal_x is not None and obs.goal_y is not None:
        goal_cm = (obs.goal_x, obs.goal_y)
        goal_source = "detected marker"
    else:
        print("\nERROR: No goal specified!")
        print("Either provide --goal X Y or place goal marker (ArUco 6x6, ID 0) in the scene.")
        return 1

    input("\n[Press Enter to continue to STEP 2: Scene Summary...]")

    # =========================================================================
    # STEP 2: Scene summary
    # =========================================================================
    print("\n" + "=" * 60)
    print("STEP 2: Scene Summary")
    print("=" * 60)
    print(f"\nRobot:")
    print(f"  Position: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) cm")
    print(f"  Orientation: {obs.robot_theta:.1f}°")

    print(f"\nGoal:")
    print(f"  Position: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm ({goal_source})")

    print(f"\nObjects ({len(obs.objects)}):")
    movable_count = 0
    static_count = 0
    for name, obj in sorted(obs.objects.items()):
        type_str = "[STATIC]" if obj.is_static else "[MOVABLE]"
        if obj.is_static:
            static_count += 1
        else:
            movable_count += 1
        print(f"  {type_str} {name}:")
        print(f"    Position: ({obj.x:.1f}, {obj.y:.1f}) cm @ {obj.theta:.1f}°")
        print(f"    Size: {obj.width:.1f} x {obj.depth:.1f} x {obj.height:.1f} cm")

    print(f"\nSummary: {movable_count} movable, {static_count} static objects")

    if args.dry_run:
        print("\n  DRY RUN MODE - Not sending commands to robot")
        return 0

    confirm = input("\n  Execute NAMO on robot? [y/N]: ").strip().lower()
    if confirm != 'y':
        print("  Cancelled by user.")
        return 0

    # =========================================================================
    # STEP 3: Create NAMOPlanner and Execute
    # =========================================================================
    print("\n" + "=" * 60)
    print("STEP 3: Execute with NAMOPlanner")
    print("=" * 60)

    # Find NAMO config (1× or 6× variant, based on --scale-factor)
    namo_config_path = (
        args.namo_config
        if args.namo_config
        else find_namo_config(args.scale_factor, args.robot_model)
    )
    primitive_data_dir = find_primitive_data_dir()

    # Get workspace and robot dimensions for reachability checking
    from robot_control.camera.workspace import WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM

    # Load robot dimensions from config
    with open(args.config, "r") as f:
        real_config = yaml.safe_load(f)
    robot_cfg = real_config.get("robot", {})
    robot_width_cm = robot_cfg.get("width_cm", 8.0)
    robot_height_cm = robot_cfg.get("height_cm", 10.0)

    print(f"\n  NAMO config: {namo_config_path}")
    print(f"  Algorithm: {args.algorithm}")
    print(f"  Execution mode: {args.execution_mode}")
    print(f"  Goal strategy: {args.strategy}")
    print(f"  Max chain depth: {args.max_chain_depth}")
    print(f"  Workspace: {WORKSPACE_WIDTH_CM}x{WORKSPACE_HEIGHT_CM} cm")
    print(f"  Robot: {robot_width_cm}x{robot_height_cm} cm")

    # Create NAMOPlanner (same as automatic mode)
    planner = NAMOPlanner(
        robot_goal_cm=goal_cm,
        namo_config_path=namo_config_path,
        algorithm=args.algorithm,
        execution_mode=args.execution_mode,
        goal_strategy=args.strategy,
        scale_factor=args.scale_factor,
        primitive_data_dir=primitive_data_dir,
        replan_on_completion=not args.no_replan,
        max_chain_depth=args.max_chain_depth,
        frontier_beam_width=args.frontier_beam_width,
        chain_link_cost=args.chain_link_cost,
        selection_strategy=args.selection_strategy,
        goals_per_region=args.goals_per_region,
        verbose=args.verbose,
        debug_xml_path=args.debug_xml,
        enable_viewer=args.viewer,
        pause_after_load=args.pause,
        ml_goal_model_path=args.ml_goal_model_path,
        ml_device=args.ml_device,
        ml_samples=args.ml_samples,
        ml_num_steps=args.ml_num_steps,
        ml_sampler_method=args.ml_sampler_method,
        local_search=local_search_from_args(args),
        hold_region_target=bool(args.hold_region_target or args.active_target),
        active_target_path=args.active_target,
        max_planning_retries=args.max_planning_retries,
        max_replan_attempts=args.max_replan_attempts,
        shuffle_edges=not args.no_shuffle_edges,
        shuffle_seed=args.shuffle_seed,
        rollout_samples_per_state=args.rollout_samples_per_state,
        # Workspace config for reachability (must match navigation planner)
        workspace_width_cm=WORKSPACE_WIDTH_CM,
        workspace_height_cm=WORKSPACE_HEIGHT_CM,
        robot_width_cm=robot_width_cm,
        robot_height_cm=robot_height_cm,
        robot_model=args.robot_model,
        manual_primitives_file=args.manual_primitives_file,
    )

    print("\n  NAMOPlanner will:")
    print("    1. Check if goal is already reachable")
    print("    2. If blocked, run NAMO planning and execute pushes")
    print("    3. Re-check reachability after each push sequence")
    print("    4. Navigate to goal when reachable")

    # Create and run runtime
    runtime_config = RuntimeConfig(
        mode="real",
        config_path=args.config,
        planner=planner,
        dry_run=args.dry_run,
        quit_on_complete=True,
        camera_service_address=getattr(args, "camera_service", None),
        record_video_dir=getattr(args, "record_video", None),
        nav_speed_override=args.nav_speed,
        push_speed_override=args.push_speed,
        step_confirm=args.step_confirm,
    )

    # Thread diagnostics through to the runtime (no-op if not enabled).
    runtime_config.diagnostics_recorder = getattr(args, "_diagnostics_recorder", None)
    runtime_config.capture_scene = bool(getattr(args, "capture_scene", False))
    runtime_config.capture_sim_success = bool(getattr(args, "capture_sim_success", False))

    print("\n  Starting robot execution...")
    print("  Press ESCAPE to abort")
    print("  " + "-" * 40)

    runtime = Runtime(runtime_config)
    runtime.run()

    print("\n" + "=" * 60)
    print("Execution Complete")
    print("=" * 60)

    return 0


def _run_plan_only_mode(args) -> int:
    """Plan once against --sim-xml, write solution.yaml, exit. No Runtime.

    Inputs:
      --sim-xml             (required) car-format MuJoCo XML — typically
                            <env>/env.xml.
      --sim-real-run-dir    (required) directory containing mid_obs.jsonl;
                            mid_obs[0] supplies the robot's starting pose
                            (car XML's freejoint default at (0,0) isn't
                            meaningful). Typically <env>.
      --diag-path/--run-name diagnostics dir for solution.yaml.

    Source of truth for the plan: NAMOPlanBridge.plan() returns a list of
    PushSubgoals. The bridge dispatches to the C++ NAMOPlanningService,
    which validates every primitive by executing it in MuJoCo via
    NAMOPushSkill before adding it to the chain. So a non-empty return =
    sim-success. We dump it to solution.yaml and stop.
    """
    from robot_control.planner.namo_bridge import NAMOPlanBridge

    if not args.sim_xml:
        print("Error: --plan-only path entered without --sim-xml.",
              file=sys.stderr)
        return 2
    if not args.sim_real_run_dir:
        print(
            "Error: --sim-xml requires --sim-real-run-dir (for the robot's "
            "starting pose from mid_obs[0]).",
            file=sys.stderr,
        )
        return 2

    sim_xml_path = Path(args.sim_xml)
    real_run_dir = Path(args.sim_real_run_dir)

    scene_record = _first_scene_record_from_mid_obs(real_run_dir)
    robot_pose = (
        _robot_pose_from_scene_record(scene_record)
        if scene_record is not None
        else None
    )
    if robot_pose is None:
        robot_pose = _robot_pose_from_real_run(real_run_dir)
    if robot_pose is None:
        print(
            f"Error: --sim-real-run-dir {real_run_dir}: no parseable "
            "robot pose in mid_obs.jsonl[0].",
            file=sys.stderr,
        )
        return 2

    scene_goal_cm = (
        _goal_from_scene_record(scene_record)
        if scene_record is not None
        else None
    )
    try:
        xml_goal_cm = _goal_from_xml(sim_xml_path)
    except (ET.ParseError, FileNotFoundError) as exc:
        print(f"Error: --sim-xml {sim_xml_path}: {exc}", file=sys.stderr)
        return 2

    # Goal: --goal wins over the XML's <site name='goal'>.
    if args.goal is not None:
        goal_cm = (float(args.goal[0]), float(args.goal[1]))
        goal_source = "command line"
    elif scene_goal_cm is not None:
        goal_cm = scene_goal_cm
        goal_source = "mid_obs.jsonl"
    elif xml_goal_cm is not None:
        goal_cm = xml_goal_cm
        goal_source = "sim-xml goal site"
    else:
        print(
            f"Error: {real_run_dir / 'mid_obs.jsonl'} has no goal_cm, "
            f"{sim_xml_path} has no <site name='goal'>, and --goal was not provided.",
            file=sys.stderr,
        )
        return 2

    # Resolve namo_cpp config + primitive dir.
    if args.namo_config:
        namo_config_path = args.namo_config
    else:
        try:
            namo_config_path = find_namo_config(args.scale_factor, args.robot_model)
        except FileNotFoundError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
    primitive_data_dir = find_primitive_data_dir()

    # Robot dims come from the namo_cpp config — NOT from a separate runtime
    # default that would silently override planning.robot_size with a wrong
    # value. (Long story: NAMOBridge._build_effective_namo_config patches
    # planning.robot_size with whatever we pass; the legacy run_namo
    # hardcoded 8x10 was over-inflating wavefront obstacles for a phantom
    # large robot, making narrow passes look impassable.)
    rw_cm, rh_cm = _robot_dims_from_namo_config(namo_config_path)

    # Prefer the captured scene record so NAMOBridge sees the same real object
    # IDs the live path uses. Fall back to the XML body names only for legacy
    # artifacts that lack object geometry in mid_obs.jsonl.
    obs = (
        _observation_from_scene_record(
            scene_record,
            default_robot_pose_cm_deg=robot_pose,
            default_goal_cm=goal_cm,
        )
        if scene_record is not None
        else None
    )
    scene_source = "mid_obs.jsonl scene record"
    if obs is None:
        try:
            sim_config, _ = _sim_config_from_xml(
                sim_xml_path,
                robot_pose_cm_deg=robot_pose,
                car_width_cm=rw_cm,
                car_height_cm=rh_cm,
            )
        except (ValueError, FileNotFoundError) as exc:
            print(f"Error: --sim-xml {sim_xml_path}: {exc}", file=sys.stderr)
            return 2
        from robot_control.core.types import Observation, ObjectPose

        objects: Dict[str, ObjectPose] = {}
        for name, (x, y, t) in sim_config.objects.items():
            d = sim_config.object_defs[name]
            objects[name] = ObjectPose(
                x=x,
                y=y,
                theta=t,
                width=d.width,
                depth=d.depth,
                height=d.height,
                is_static=d.is_static,
            )
        obs = Observation(
            robot_x=sim_config.x,
            robot_y=sim_config.y,
            robot_theta=sim_config.theta,
            objects=objects,
            timestamp=0.0,
            goal_x=goal_cm[0],
            goal_y=goal_cm[1],
        )
        scene_source = "sim-xml bodies (legacy fallback)"
    robot_pose = (float(obs.robot_x), float(obs.robot_y), float(obs.robot_theta))

    if args.verbose:
        print()
        print("=" * 60)
        print("NAMO Plan-Only (sim, MuJoCo-verified via planner internals)")
        print("=" * 60)
        print(f"  sim_xml:          {sim_xml_path}")
        print(f"  sim_real_run_dir: {real_run_dir}")
        print(f"  scene source:     {scene_source}")
        print(f"  robot:            ({obs.robot_x:.2f}, {obs.robot_y:.2f}, "
              f"θ={obs.robot_theta:.1f}°) [from mid_obs[0]]")
        print(f"  goal:             ({goal_cm[0]:.2f}, {goal_cm[1]:.2f}) cm [{goal_source}]")
        print(f"  robot footprint:  {rw_cm:.1f} x {rh_cm:.1f} cm "
              "(from namo_cpp config)")
        print(f"  namo_config:      {namo_config_path}")
        print(f"  algorithm:        {args.algorithm}")
        print(f"  strategy:         {args.strategy}")
        print(f"  max_chain_depth:  {args.max_chain_depth}")
        print("=" * 60)
        sys.stdout.flush()

    bridge = NAMOPlanBridge(
        namo_config_path=namo_config_path,
        scale_factor=args.scale_factor,
        primitive_data_dir=primitive_data_dir,
        verbose=args.verbose,
        robot_width_cm=rw_cm,
        robot_height_cm=rh_cm,
        robot_model=args.robot_model,
    )

    # Every search option both paths must send. The held path used to build its
    # own shorter list, so a held boundary fell back to namo_cpp's opener
    # defaults; region_max_chain_depth defaults to 1, at which no
    # setup-then-finish chain exists, which is the only reason to hold one.
    # Mirrors NAMOPlanner._search_planner_kwargs, which the in-process path uses.
    extra_kwargs: Dict[str, Any] = {
        "max_chain_depth": args.max_chain_depth,
        "frontier_beam_width": args.frontier_beam_width,
        "chain_link_cost": args.chain_link_cost,
        "selection_strategy": args.selection_strategy,
        "goals_per_region": args.goals_per_region,
    }
    if getattr(args, "rollout_samples_per_state", None) is not None:
        extra_kwargs["rollout_samples_per_state"] = args.rollout_samples_per_state
    if getattr(args, "shuffle_seed", None) is not None:
        extra_kwargs["shuffle_seed"] = args.shuffle_seed
    # Region "opened" success bar (forwarded to RegionOpeningPlanner via the
    # service's algo_params). Default 1 = legacy behavior.
    if getattr(args, "region_success_min_reachable", None) is not None:
        extra_kwargs["region_success_min_reachable"] = args.region_success_min_reachable
    # ML strategies need the model path + inference knobs forwarded to the
    # bridge (and on to NAMOPlanningService). The Runtime/NAMOPlanner path
    # passes these; plan-only must too, or `--strategy ml` reaches the planner
    # with no model and fails instantly. Mirrors NAMOPlanner._plan extra_kwargs.
    if getattr(args, "ml_goal_model_path", None):
        extra_kwargs["ml_goal_model_path"] = args.ml_goal_model_path
        extra_kwargs["ml_device"] = args.ml_device
        if getattr(args, "ml_samples", None) is not None:
            extra_kwargs["ml_samples"] = args.ml_samples
        if getattr(args, "ml_num_steps", None) is not None:
            extra_kwargs["ml_num_steps"] = args.ml_num_steps
        if getattr(args, "ml_sampler_method", None) is not None:
            extra_kwargs["ml_sampler_method"] = args.ml_sampler_method

    extra_kwargs.update(local_search_from_args(args).as_planner_kwargs())

    # Held-boundary mode: resume the subproblem the previous process left,
    # rather than re-deriving which boundary to open. Same advance step the
    # in-process planner uses -- the two share no other code.
    active_target_path = Path(args.active_target) if args.active_target else None
    if active_target_path is not None or args.hold_region_target:
        held = RegionOpeningTarget.load(active_target_path) if active_target_path else None
        boundary_plan, target, status, released = advance_boundary(
            bridge,
            obs,
            goal_cm,
            target=held,
            open_fraction=CANONICAL_OPEN_FRACTION,
            scale_factor=args.scale_factor,
            planner_kwargs={"goal_strategy": args.strategy, **extra_kwargs},
        )
        if active_target_path is not None:
            # `released` names what happened to the target we were handed, so a
            # transient failure leaves a live subproblem alone instead of
            # marking it opened.
            if held is not None and released is not None:
                held.released(released).save(active_target_path)
            if target is not None:
                target.save(active_target_path)

        plan_subgoals = list(boundary_plan.subgoals) if status == ADVANCE_PLANNED else []
        boundary_outcome = {
            ADVANCE_PLANNED: None,
            ADVANCE_EXHAUSTED: "boundary exhausted",
            ADVANCE_NO_BOUNDARY: "no boundary left to open",
        }.get(status, "boundary produced no plan")
        print(
            f"[run_namo plan-only] held-boundary status={status} "
            f"target={getattr(target, 'target_id', None)} pushes={len(plan_subgoals)}"
        )
    else:
        boundary_outcome = None
        plan_subgoals = bridge.plan(
            observation=obs,
            robot_goal_cm=goal_cm,
            algorithm=args.algorithm,
            goal_strategy=args.strategy,
            failed_pushes=set(),
            **extra_kwargs,
        )

    recorder = getattr(args, "_diagnostics_recorder", None)
    if recorder is None or not getattr(recorder, "enabled", False):
        print(
            "Error: --plan-only requires --diag-path so solution.yaml has a "
            "home. (Pass --diag-path <env>/solution/eps --run-name run.)",
            file=sys.stderr,
        )
        return 2
    diag_root = Path(recorder.root)
    object_mapping_obj = bridge.get_object_mapping()
    object_mapping = {
        "real_to_sim": dict(object_mapping_obj.real_to_sim),
        "sim_to_real": dict(object_mapping_obj.sim_to_real),
    }
    planner_scene_path: Optional[Path] = None
    if bridge.last_xml_content:
        diag_root.mkdir(parents=True, exist_ok=True)
        planner_scene_path = diag_root / "planner_scene.xml"
        planner_scene_path.write_text(bridge.last_xml_content)
    _emit_plan_only_solution_yaml(
        diag_root,
        goal_cm,
        algorithm=args.algorithm,
        strategy=args.strategy,
        plan_subgoals=plan_subgoals,
        search_time_ms=bridge.last_search_time_ms,
        sim_pushes_tried=bridge.last_sim_pushes_tried,
        object_mapping=object_mapping,
        planner_scene_xml=planner_scene_path.name if planner_scene_path is not None else None,
        outcome_override=boundary_outcome,
    )

    # If a plan was found, render the MP4 of the chain executing in MuJoCo
    # — same call execute_sim_push --trial-spec uses. This lands the visual
    # evidence next to solution.yaml so reviewers can confirm at a glance.
    # Pass the *observation pose* (mid_obs[0]) as the starting robot pose so
    # the replay env matches the planner's verification env byte-for-byte
    # (namo_bridge.py:400-419 also passes observation pose to plan_from_xml).
    # Earlier this used the edge-point standoff pose, which started push 1
    # from a different state — push 1's end then diverged, push 2 started
    # from a wrong state, and the MP4 silently showed a chain that didn't
    # actually solve the problem even though solution.yaml was correct.
    if plan_subgoals:
        try:
            _render_plan_only_mp4(
                diag_root=diag_root,
                scene_xml_path=planner_scene_path or sim_xml_path,
                plan_subgoals=plan_subgoals,
                namo_config_path=namo_config_path,
                observation_robot_pose_cm_deg=robot_pose,
                real_to_sim=object_mapping["real_to_sim"],
            )
        except Exception as exc:
            print(
                f"[run_namo plan-only] WARNING: MP4 render failed: {exc!r}",
                file=sys.stderr,
            )

    return 0 if plan_subgoals else 1


def run_automatic_mode(args):
    """Run NAMO in automatic mode (original behavior)."""
    robot_control_dir, _, _ = get_namo_paths()

    # Determine mode
    if args.config:
        mode = "real"
    else:
        mode = "sim"
        args.sim = True

    # Determine goal.
    # --goal always wins. In --sim-xml mode the XML's <site name='goal'>
    # is the secondary source (deferred to the sim_config build below —
    # we set goal_cm here only if --goal was passed). In real mode we
    # detect from the camera. Otherwise plain --sim needs --goal.
    sim_xml_path: Optional[str] = getattr(args, "sim_xml", None)
    goal_cm: Optional[Tuple[float, float]] = None
    goal_source: Optional[str] = None
    if args.goal is not None:
        goal_cm = (args.goal[0], args.goal[1])
        goal_source = "command line"
    elif mode == "real":
        os.chdir(str(robot_control_dir))
        goal_cm = detect_goal_from_camera(
            args.config, args.objects,
            camera_service=getattr(args, "camera_service", None),
        )
        if goal_cm is None:
            print("\nError: No goal specified!")
            print("Either provide --goal X Y or place goal marker (ArUco 6x6, ID 0) in the scene.")
            return 1
        goal_source = "detected marker"
    elif sim_xml_path:
        # Peek the XML for the <site name='goal'> so the planner gets a
        # real goal at construction time (line ~884), not None. The
        # sim_config_from_xml call later returns the same value.
        try:
            _peek_root = ET.parse(str(sim_xml_path)).getroot()
        except (ET.ParseError, FileNotFoundError) as exc:
            print(f"Error: --sim-xml {sim_xml_path}: {exc}", file=sys.stderr)
            return 1
        for site in _peek_root.iter("site"):
            if site.get("name") == "goal":
                pos = [float(v) for v in site.get("pos", "0 0 0").split()]
                goal_cm = (pos[0] * 100.0, pos[1] * 100.0)
                goal_source = "sim-xml goal site"
                break
        if goal_cm is None:
            print(
                f"Error: --sim-xml {sim_xml_path} has no <site name='goal'> "
                "and --goal not provided.",
                file=sys.stderr,
            )
            return 1
    else:
        print(
            "Error: --goal is required in simulation mode without --sim-xml "
            "(no camera to detect goal marker; XML has no goal site to read)."
        )
        return 1

    # Find NAMO config
    if args.namo_config:
        namo_config_path = args.namo_config
    else:
        try:
            namo_config_path = find_namo_config(args.scale_factor, args.robot_model)
        except FileNotFoundError as e:
            print(f"Error: {e}")
            return 1

    # Find primitive data directory
    primitive_data_dir = find_primitive_data_dir()

    if args.verbose:
        print("\n" + "=" * 60)
        print("NAMO Planning and Execution")
        print("=" * 60)
        print(f"Mode: {mode}")
        print(f"Goal: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm ({goal_source})")
        print(f"NAMO config: {namo_config_path}")
        print(f"Algorithm: {args.algorithm}")
        print(f"Strategy: {args.strategy}")
        print(f"{local_search_from_args(args).describe()}")
        print(f"Max chain depth: {args.max_chain_depth}")
        print(f"Frontier beam width: {args.frontier_beam_width}")
        print(f"Chain link cost: {args.chain_link_cost}")
        print(f"Selection strategy: {args.selection_strategy}")
        print(
            "Goals per region: "
            + (str(args.goals_per_region) if args.goals_per_region is not None
               else "canonical (namo_cpp default)")
        )
        print(f"Replan on completion: {not args.no_replan}")
        print("=" * 60)
        sys.stdout.flush()

    # Get workspace and robot dimensions for reachability checking
    # Must match what navigation planner uses
    from robot_control.camera.workspace import WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM

    # Load robot dimensions from config.
    #
    # The hardcoded sim default (8x10) is the legacy 2-box sim scene's
    # robot. For --sim-xml runs the actual car footprint is 7x7
    # (config/real.yaml), and the planner's wavefront inflation has to
    # match — otherwise the wavefront over-inflates obstacles for a
    # phantom larger robot and rejects passages the 7x7 car would
    # comfortably fit through. Symptom: planner reports NO SUBGOALS even
    # for trivial 1-push scenes. Read from real.yaml in both real and
    # --sim-xml modes; only the legacy --sim hardcoded scene falls back
    # to the 8x10 defaults.
    robot_width_cm = 8.0  # Default (legacy hardcoded --sim scene)
    robot_height_cm = 10.0
    real_yaml_for_robot = None
    if mode == "real" and args.config:
        real_yaml_for_robot = args.config
    elif sim_xml_path:
        # --sim-xml workflow always targets the real-test-env scenes
        # captured against config/real.yaml. Pull the real car size from
        # there so wavefront inflation lines up with execute_sim_push.
        real_yaml_for_robot = "config/real.yaml"
    if real_yaml_for_robot and Path(real_yaml_for_robot).exists():
        with open(real_yaml_for_robot, "r") as f:
            _robot_yaml = yaml.safe_load(f) or {}
        _robot_cfg = (_robot_yaml.get("robot") or {})
        robot_width_cm = float(_robot_cfg.get("width_cm", robot_width_cm))
        robot_height_cm = float(_robot_cfg.get("height_cm", robot_height_cm))

    if args.verbose:
        print(f"Workspace: {WORKSPACE_WIDTH_CM}x{WORKSPACE_HEIGHT_CM} cm")
        print(f"Robot: {robot_width_cm}x{robot_height_cm} cm")

    # Create NAMO planner
    planner = NAMOPlanner(
        robot_goal_cm=goal_cm,
        namo_config_path=namo_config_path,
        algorithm=args.algorithm,
        execution_mode=args.execution_mode,
        goal_strategy=args.strategy,
        scale_factor=args.scale_factor,
        primitive_data_dir=primitive_data_dir,
        replan_on_completion=not args.no_replan,
        max_chain_depth=args.max_chain_depth,
        frontier_beam_width=args.frontier_beam_width,
        chain_link_cost=args.chain_link_cost,
        selection_strategy=args.selection_strategy,
        goals_per_region=args.goals_per_region,
        verbose=args.verbose,
        debug_xml_path=args.debug_xml,
        enable_viewer=args.viewer,
        pause_after_load=args.pause,
        ml_goal_model_path=args.ml_goal_model_path,
        ml_device=args.ml_device,
        ml_samples=args.ml_samples,
        ml_num_steps=args.ml_num_steps,
        ml_sampler_method=args.ml_sampler_method,
        local_search=local_search_from_args(args),
        hold_region_target=bool(args.hold_region_target or args.active_target),
        active_target_path=args.active_target,
        max_planning_retries=args.max_planning_retries,
        max_replan_attempts=args.max_replan_attempts,
        shuffle_edges=not args.no_shuffle_edges,
        shuffle_seed=args.shuffle_seed,
        rollout_samples_per_state=args.rollout_samples_per_state,
        # Workspace config for reachability (must match navigation planner)
        workspace_width_cm=WORKSPACE_WIDTH_CM,
        workspace_height_cm=WORKSPACE_HEIGHT_CM,
        robot_width_cm=robot_width_cm,
        robot_height_cm=robot_height_cm,
        robot_model=args.robot_model,
        manual_primitives_file=args.manual_primitives_file,
    )

    # Create runtime config
    if mode == "sim":
        if sim_xml_path:
            # --sim-xml mode: build SimConfig from a captured real_test_envs
            # initial scene. Robot pose comes from --sim-real-run-dir/mid_obs[0]
            # since the car XML's freejoint spawn at (0,0) isn't meaningful.
            robot_pose = None
            if args.sim_real_run_dir:
                robot_pose = _robot_pose_from_real_run(Path(args.sim_real_run_dir))
                if robot_pose is None:
                    print(
                        f"Error: --sim-real-run-dir {args.sim_real_run_dir} "
                        "has no parseable mid_obs.jsonl[0] robot pose.",
                        file=sys.stderr,
                    )
                    return 1
            try:
                sim_config, _ = _sim_config_from_xml(
                    Path(sim_xml_path),
                    robot_pose_cm_deg=robot_pose,
                )
            except (ValueError, FileNotFoundError) as exc:
                print(f"Error: --sim-xml: {exc}", file=sys.stderr)
                return 1
            # goal_cm was already filled from the same XML during the goal-
            # determination block (or overridden by --goal). Discard the
            # second read above.
        else:
            sim_config = SimConfig(
                car_width=8,
                car_height=10,
                x=10,
                y=10,
                theta=0,
                objects={
                    # SimConfig objects are (x_cm, y_cm, theta_deg)
                    "box_1": (25, 30, 0),
                    "box_2": (35, 40, 45),
                },
                # Provide geometry for synthetic simulation objects so XML
                # generation never emits zero-sized geoms during NAMO planning.
                object_defs={
                    "box_1": ObjectDef(name="box_1", width=8.0, depth=8.0, height=4.0, is_static=False),
                    "box_2": ObjectDef(name="box_2", width=8.0, depth=8.0, height=4.0, is_static=False),
                },
            )

        runtime_config = RuntimeConfig(
            mode="sim",
            sim_config=sim_config,
            planner=planner,
            initial_speed=args.speed,
            nav_speed_override=args.nav_speed,
            push_speed_override=args.push_speed,
            quit_on_complete=not args.no_quit,
            step_confirm=args.step_confirm,
            show_gui=not args.headless,
        )
    else:
        runtime_config = RuntimeConfig(
            mode="real",
            config_path=args.config,
            planner=planner,
            dry_run=args.dry_run,
            quit_on_complete=not args.no_quit,
            camera_service_address=getattr(args, "camera_service", None),
            record_video_dir=getattr(args, "record_video", None),
            nav_speed_override=args.nav_speed,
            push_speed_override=args.push_speed,
            step_confirm=args.step_confirm,
            show_gui=not args.headless,
            show_camera=not args.headless,
        )

    # Thread diagnostics through to the runtime (no-op if not enabled).
    runtime_config.diagnostics_recorder = getattr(args, "_diagnostics_recorder", None)
    runtime_config.capture_scene = bool(getattr(args, "capture_scene", False))
    runtime_config.capture_sim_success = bool(getattr(args, "capture_sim_success", False))

    # Run
    print("\nStarting NAMO execution...")
    print("Press ESCAPE to quit")
    print("=" * 50)

    runtime = Runtime(runtime_config)
    runtime.run()

    # Note: the --sim-xml case is intercepted at main() dispatch and routed
    # to _run_plan_only_mode (no Runtime, no SimEnv). solution.yaml is
    # written there straight from NAMOPlanBridge.plan(). The earlier
    # post-runtime emit that used to live here read from success_chain.json
    # — the kinematic SimEnv outcome — which disagreed with the planner's
    # MuJoCo verification.

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Run NAMO planning and execution",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # Mode selection
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Run in simulation mode (default if --config not specified)",
    )
    parser.add_argument(
        "--sim-xml",
        type=str,
        default=None,
        metavar="PATH",
        help=(
            "In --sim mode, build the SimConfig from a car-format MuJoCo XML "
            "(typically <env>/env.xml). "
            "When set, --goal is optional — the XML's <site name='goal'> is "
            "used if present. Recommended companion: --sim-real-run-dir for "
            "the robot's starting pose (the car XML can't carry the freejoint "
            "pose). Without --sim-xml, --sim falls back to the legacy "
            "hardcoded 2-box SimConfig."
        ),
    )
    parser.add_argument(
        "--sim-real-run-dir",
        type=str,
        default=None,
        metavar="DIR",
        help=(
            "Directory whose first mid_obs.jsonl frame supplies the robot's "
            "starting pose for --sim-xml mode. Typically <env>. Ignored "
            "unless --sim-xml is also set."
        ),
    )
    parser.add_argument(
        "--config", "-c",
        type=str,
        default=None,
        help="Path to real robot config (enables real mode)",
    )
    parser.add_argument(
        "--interactive", "-i",
        action="store_true",
        help="Run in interactive mode with step-by-step prompts",
    )
    parser.add_argument(
        "--camera-service",
        type=str,
        default=None,
        metavar="ADDRESS",
        help="ZMQ address of camera service (e.g. tcp://localhost:5556). "
             "Skips local camera creation.",
    )
    parser.add_argument(
        "--record-video",
        type=str,
        nargs="?",
        default=None,
        const=_RECORD_VIDEO_DEFAULT_SENTINEL,
        metavar="DIR",
        help="Ask the --camera-service to record video for the duration of "
             "this run_namo session. Pass with no value to default to "
             "<diag-path>/<run-name>/recordings/ (requires --diag-path); "
             "pass with an explicit DIR to override. Requires --camera-service; "
             "the service writes the MP4 itself. Also writes per-subgoal "
             "meta JSON + XML snapshots for sim replay.",
    )
    # Goal specification
    parser.add_argument(
        "--goal", "-g",
        type=float,
        nargs=2,
        default=None,
        metavar=("X", "Y"),
        help="Goal position in cm (default: detect from goal marker in real mode)",
    )
    parser.add_argument(
        "--objects",
        type=str,
        default="config/objects.yaml",
        help="Path to objects definition file (default: config/objects.yaml)",
    )

    # NAMO config
    parser.add_argument(
        "--namo-config",
        type=str,
        default=None,
        help="Path to NAMO config YAML",
    )
    parser.add_argument(
        "--scale-factor",
        type=float,
        default=1.0,
        help=(
            "Multiplier from real cm to MuJoCo simulation units (default: 1.0). "
            "1.0 keeps the planner in real-world meters/cm (production path). "
            "Pass 6.0 to use the legacy 6×-scaled config + primitives for "
            "regression or fallback. The scale picks which namo config and "
            "primitive .dat set get loaded; see SCALE_UNIFICATION_PLAN.md."
        ),
    )
    parser.add_argument(
        "--robot-model",
        type=str,
        default="car",
        choices=["sphere", "car"],
        help=(
            "Robot body model for the planning simulator (default: car). "
            "'car' = diff-drive little_car body, matches real-robot physics, "
            "uses the 1x_car primitive set under "
            "namo_cpp/data/motion_primitives_1x_car_{square,wide,tall}.dat. "
            "'sphere' = legacy holonomic point robot (faster search, less "
            "physically accurate). Both use a 7 cm footprint."
        ),
    )
    parser.add_argument(
        "--algorithm",
        type=str,
        default="full_namo",
        choices=["full_namo", "region_opening"],
        help="Planning algorithm (default: full_namo)",
    )
    parser.add_argument(
        "--execution-mode",
        type=str,
        default="mpc",
        choices=["open_loop", "mpc"],
        help=(
            "Plan-level execution strategy (default: mpc). "
            "'mpc' executes only the first push from each plan, then replans "
            "from a fresh observation (closed-loop at the plan level). "
            "'open_loop' executes the entire planned sequence before replanning."
        ),
    )
    parser.add_argument(
        "--strategy",
        type=str,
        default="primitive",
        help=(
            "Goal strategy (default: primitive). Options: "
            "'primitive' (exhaustive depth-first enumeration of 600 primitive "
            "slots per state), 'ml_primitive'/'ml' (ML-scored primitives first, "
            "primitives as fallback), 'random_rollout'/'random' (random ordering "
            "of primitives, optional thinning via --rollout-samples-per-state), "
            "'geometric' (geometric transport heuristic), 'manual_primitives' "
            "(read a hand-authored (object_id, edge_idx, push_steps) sequence "
            "from --manual-primitives-file, verify in sim, dispatch to real "
            "robot only on sim success — forces --execution-mode open_loop)."
        ),
    )
    parser.add_argument(
        "--manual-primitives-file",
        type=str,
        default=None,
        metavar="PATH",
        help=(
            "Path to a YAML file listing manual primitives to try when "
            "--strategy manual_primitives is set. Schema:\n"
            "  primitives:\n"
            "    - object_id: obj_4\n"
            "      edge_idx: 19\n"
            "      push_steps: 9\n"
            "    - object_id: obj_4\n"
            "      edge_idx: 23\n"
            "      push_steps: 2\n"
            "Each entry: real-robot object id, edge index 0..(4*points_per_face-1) "
            "(0..59 with default points_per_face=15), and push_steps >= 1 "
            "(depth = push_steps - 1). The bridge applies the chain in a fresh "
            "RLEnvironment and dispatches to the real robot only if the post-"
            "chain state has the robot goal reachable."
        ),
    )
    parser.add_argument(
        "--rollout-samples-per-state",
        type=int,
        default=None,
        help=(
            "For --strategy random_rollout: cap on candidates per state "
            "(default: None = use all ~600 primitives in random order). "
            "Smaller values give thinner trials and faster planning per call, "
            "at the cost of less per-state coverage."
        ),
    )
    parser.add_argument(
        "--ml-goal-model-path",
        type=str,
        default=None,
        help="Path to trained ML goal model (required for ml strategies)",
    )
    parser.add_argument(
        "--ml-device",
        type=str,
        default="cuda",
        help="PyTorch device for ML model (default: cuda)",
    )
    parser.add_argument(
        "--ml-samples",
        type=int,
        default=None,
        help="Number of diffusion samples per inference call (default: 32)",
    )
    parser.add_argument(
        "--ml-num-steps",
        type=int,
        default=None,
        help="Number of denoising / integration steps (default: 20)",
    )
    parser.add_argument(
        "--ml-sampler-method",
        type=str,
        default=None,
        help="Sampler method: ddpm/ddim (diffusion) or euler/midpoint/rk4/dopri5 (flow matching)",
    )
    parser.add_argument(
        "--hold-region-target",
        action="store_true",
        help="Keep working on one region boundary until it opens, instead of "
             "re-choosing after every push. Implied by --active-target.",
    )
    parser.add_argument(
        "--active-target",
        type=str,
        default=None,
        help="Path to the active-target JSON. Persists the held boundary so a "
             "later process (e.g. closed_loop_session's per-replan subprocess) "
             "resumes the same subproblem instead of starting a new one.",
    )
    parser.add_argument(
        "--local-search",
        type=str,
        default=DEFAULT_LOCAL_SEARCH,
        choices=list(LOCAL_SEARCH_CHOICES),
        help="Local search run per region boundary. region_bfs = chain BFS "
             "(default); best_first = single global priority queue over pushes.",
    )
    parser.add_argument(
        "--best-first-prior",
        type=str,
        default=DEFAULT_BEST_FIRST_PRIOR,
        choices=list(BEST_FIRST_PRIOR_CHOICES),
        help="Ordering for best_first: model = learned ranker (needs "
             "--scorer-ckpt); uniform = random order, the ablation baseline.",
    )
    parser.add_argument(
        "--scorer-ckpt",
        type=str,
        default=None,
        help="Ranker checkpoint for --best-first-prior model. Seeds are "
             "independent models: pick one explicitly, never average them.",
    )
    parser.add_argument(
        "--best-first-hmax",
        type=int,
        default=None,
        help="Max pushes per local plan for best_first. Omit to use namo_cpp's "
             "canonical value (2), which every registered evaluation used.",
    )
    parser.add_argument(
        "--keyhole-simulation-budget",
        type=int,
        default=None,
        help="Simulator calls allowed per region boundary. Omit to use "
             "namo_cpp's canonical value (900).",
    )
    parser.add_argument(
        "--max-chain-depth",
        type=int,
        default=2,
        help="Maximum chain depth for multi-push solutions (default: 2)",
    )
    parser.add_argument(
        "--frontier-beam-width",
        type=int,
        default=10000,
        help="Beam width for frontier search (default: 10000)",
    )
    parser.add_argument(
        "--chain-link-cost",
        type=int,
        default=11,
        help="Additional cost per chain link beyond first push (default: 11)",
    )
    parser.add_argument(
        "--selection-strategy",
        type=str,
        default="cost_first",
        choices=["cost_first", "ml_first"],
        help="Frontier priority (default: cost_first)",
    )
    parser.add_argument(
        "--goals-per-region",
        type=int,
        default=None,
        help="Goal samples per region for validation. Omit to use "
             "namo_cpp's canonical 100, which the 20%% opening bar assumes.",
    )
    parser.add_argument(
        "--region-success-min-reachable",
        type=int,
        default=1,
        help=(
            "Minimum number of sampled region-goal points that must be "
            "reachable for a region to count as opened/solved (default: 1). "
            "Pair with --goals-per-region to set the bar, e.g. "
            "--goals-per-region 100 --region-success-min-reachable 20 "
            "requires 20 of 100 sampled goals reachable."
        ),
    )
    parser.add_argument(
        "--max-planning-retries",
        type=int,
        default=5,
        help=("Max retries per plan() call with different shuffle seeds when "
              "the planner returns NO SUBGOALS (default: 5). Lower to fail "
              "faster when planning is genuinely impossible."),
    )
    parser.add_argument(
        "--no-shuffle-edges",
        action="store_true",
        help=("Disable edge-ordering randomization in the planner. Pushes "
              "are enumerated in a fixed deterministic order."),
    )
    parser.add_argument(
        "--shuffle-seed",
        type=int,
        default=None,
        help=("Explicit shuffle seed for planner edge/random ordering. "
              "If omitted, planner uses its default behavior."),
    )
    parser.add_argument(
        "--max-replan-attempts",
        type=int,
        default=20,
        help=("Max replans for the same failed subgoal before giving up "
              "(default: 20). Lower to abort a stuck push sequence faster."),
    )

    # Execution options
    parser.add_argument(
        "--no-replan",
        action="store_true",
        help="Don't replan when subgoal queue is exhausted",
    )
    parser.add_argument(
        "--no-quit",
        action="store_true",
        help="Don't quit when plan completes",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI window (useful for server/headless execution)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't send commands to robot (real mode only)",
    )
    parser.add_argument(
        "--step-confirm",
        action="store_true",
        help=(
            "Pause for stdin confirmation before each subgoal dispatch. "
            "ENTER executes, 's' skips and blacklists the push, 'q' aborts the run."
        ),
    )

    # Debugging
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    parser.add_argument(
        "--debug-xml",
        type=str,
        default=None,
        help="Save generated XML to this path for debugging",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Enable MuJoCo visualization window during NAMO planning",
    )
    parser.add_argument(
        "--pause",
        action="store_true",
        help="Pause after loading XML for interactive viewer inspection (requires --viewer)",
    )

    # Simulation options
    # Diagnostics flags. When --diag-path is set, the run produces a directory
    # of structured outputs (config/summary JSON + JSONL event streams + tee'd
    # run.log + optional scene snapshots). See robot_control/diagnostics/.
    parser.add_argument(
        "--diag-path",
        type=str,
        default=None,
        help="Root directory for diagnostics output. Enables diagnostics when set.",
    )
    parser.add_argument(
        "--run-name",
        type=str,
        default=None,
        help=("Subdir name under --diag-path. Supports placeholders: "
              "{timestamp} {date} {time} {epoch} {strategy} {algorithm} "
              "{goal} {mode} {git}. Slashes create nested subdirs. Required "
              "if --diag-path is set."),
    )
    parser.add_argument(
        "--capture-scene",
        action="store_true",
        help="Save scene snapshots (jpg + json + xml) at run start and end.",
    )
    parser.add_argument(
        "--capture-sim-success",
        action="store_true",
        help="On a successful real run, record the executed push chain in "
             "sim. Writes success_chain.json + success_sim_replay.mp4 under "
             "the diag root. On fail/abort, writes partial_chain.json only "
             "(no sim replay). Requires --diag-path.",
    )
    parser.add_argument(
        "--allow-overwrite",
        action="store_true",
        help="Allow --diag-path/--run-name to overwrite an existing directory.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Max speed for keyboard/follow-path controllers (0-1, default: 0.3). "
             "Does NOT override navigation/push speeds — use --nav-speed/--push-speed for those.",
    )
    parser.add_argument(
        "--nav-speed",
        type=float,
        default=None,
        help="Override max speed for navigation controller (0-1). "
             "If unset, uses navigation.max_speed from controller.yaml.",
    )
    parser.add_argument(
        "--push-speed",
        type=float,
        default=None,
        help="Override max speed for push controller (0-1). "
             "If unset, uses push.max_speed from controller.yaml.",
    )

    args = parser.parse_args()

    # Bootstrap diagnostics before any real work — installs Tee on stdout/
    # stderr so all subsequent prints land in run.log, and writes config.json
    # capturing the pre-run state. Returns (None, None) if --diag-path unset.
    sys.path.insert(0, str(Path(__file__).parent))
    from _diag_setup import bootstrap_diagnostics  # type: ignore
    recorder, log_file = bootstrap_diagnostics(args)
    args._diagnostics_recorder = recorder  # stashed for downstream helpers

    # Resolve --record-video and --capture-sim-success against the diag root.
    # Both flags can default to subpaths under <diag-path>/<run-name>/ when
    # the user passes them bare; without --diag-path that's an error.
    diag_root = recorder.root if (recorder is not None and recorder.enabled) else None
    if getattr(args, "record_video", None) == _RECORD_VIDEO_DEFAULT_SENTINEL:
        if diag_root is None:
            print("Error: --record-video with no DIR requires --diag-path "
                  "to be set (or pass an explicit DIR).")
            return 1
        args.record_video = str(diag_root / "recordings")
    if getattr(args, "capture_sim_success", False) and diag_root is None:
        print("Error: --capture-sim-success requires --diag-path to be set "
              "(sim replay artifacts are written under the diag root).")
        return 1

    try:
        try:
            setup_namo_imports()
        except Exception as exc:
            print(f"Error: {exc}")
            return 1

        # Interactive mode requires real robot config
        if args.interactive:
            if not args.config:
                print("Error: --interactive mode requires --config for real robot")
                return 1
            return run_interactive_mode(args)
        # --sim-xml always uses plan-only (planner verifies plans through the
        # C++ MuJoCo controller, so a returned plan IS a sim-success result —
        # no point running it again through the kinematic SimEnv, which has
        # weaker physics and silently turned valid plans into "stuck" failures).
        if getattr(args, "sim_xml", None):
            return _run_plan_only_mode(args)
        return run_automatic_mode(args)
    finally:
        # Best-effort summary write + cleanup. Anything that goes wrong here
        # must not mask the original return code.
        try:
            if recorder is not None and recorder.enabled:
                _write_run_summary(args, recorder)
                recorder.close()
        except Exception as exc:
            print(f"[DIAG] ⚠️ summary write failed: {exc!r}", flush=True)
        if log_file is not None:
            try:
                log_file.flush()
                log_file.close()
            except Exception:
                pass


def _write_run_summary(args, recorder):
    """Fallback summary writer — only fires if the Runtime didn't already
    write one (e.g. crashed before reaching its finally block). Runtime's
    summary is richer; this one is a bare-bones marker that the run
    existed and exited abnormally.
    """
    if recorder is None or not recorder.enabled:
        return
    summary_path = recorder.root / "summary.json"
    if summary_path.exists():
        # Runtime wrote a richer summary already. Don't overwrite.
        return

    import datetime as _dt
    import time as _time

    now = _time.time()
    payload = {
        "run_name": recorder.root.name,
        "outcome": "crashed",
        "outcome_reason": "runtime did not write summary (probable crash before finally)",
        "ended_at_epoch": now,
        "ended_at_utc": _dt.datetime.fromtimestamp(now, tz=_dt.timezone.utc)
                          .isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "mode": "sim" if getattr(args, "sim", False) or not getattr(args, "config", None) else "real",
        "strategy": getattr(args, "strategy", None),
        "algorithm": getattr(args, "algorithm", None),
        "goal_target_cm": list(args.goal) if getattr(args, "goal", None) else None,
        "totals": dict(recorder.totals),
        "scene_capture": {},
    }
    recorder.write_summary(payload)


if __name__ == "__main__":
    exit(main())
