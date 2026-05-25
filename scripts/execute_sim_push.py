"""Execute a single (edge_idx, depth) push in MuJoCo sim via C++ replay.

What this does:

    * Loads a captured MuJoCo XML (the same one ``capture_to_xml.py
      --robot-model car`` writes) and runs full MuJoCo physics.
    * Dispatches the push through the C++ ``NAMOPushController`` replay path.
    * Writes both the rendered MP4 and the Tier 2 calibration artifacts:
      ``mid_obs.jsonl``, ``pushes.jsonl``, ``subgoals.jsonl``,
      ``wheel_commands.jsonl``, ``qpos_dump_full.txt``, and
      ``tier2_push_trials/``.
    * Takes ``--edge-idx`` and ``--depth`` directly on the command line.
      No primitives-DB lookup. One push per invocation.

Example:
    # 1) capture the current real scene as a car-format XML
    python scripts/capture_to_xml.py \\
        --camera-service tcp://localhost:5556 \\
        --robot-model car --scale-factor 1.0 \\
        --output /tmp/sim_scene.xml

    # 2) execute edge 29, depth 9 in MuJoCo replay
    python scripts/execute_sim_push.py \\
        --mujoco-xml /tmp/sim_scene.xml \\
        --edge-idx 29 --depth 9
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Tuple

import yaml

# Make the script importable both from the repo root and after `pip install -e .`
HERE = Path(__file__).resolve().parent
ROBOT_CONTROL_ROOT = HERE.parent
SRC = ROBOT_CONTROL_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from _diag_setup import bootstrap_diagnostics  # type: ignore  # noqa: E402

from execute_real_push import export_tier2_push_trials  # noqa: E402

from robot_control.controller.edge_points import get_edge_point  # noqa: E402
from robot_control.utils import NAMOXMLGenerator  # noqa: E402


# ----------------------------------------------- real-run rewind helpers
#
# The "rewind real" mode reconstructs the push-start state of a real-side
# ``execute_real_push`` run so the sim starts from the exact same
# configuration. Reads recorded diagnostics from the real run directory:
#
#   - pushes.jsonl                  : pushed object pose + exact robot pose at PUSH start
#   - subgoals.jsonl                : subgoal metadata / fallback timestamps
#   - mid_obs.jsonl                 : full scene state (robot + all visible objects)


def _load_jsonl(path: Path) -> list[dict]:
    records = []
    if not path.exists():
        return records
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        records.append(json.loads(line))
    return records


def _safe_float(value):
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def _push_signature_matches(active: object, push_rec: dict) -> bool:
    if not isinstance(active, dict):
        return False
    try:
        return (
            active.get("object_id") == push_rec.get("object_id")
            and int(active.get("edge_idx")) == int(push_rec.get("expected_edge"))
            and int(active.get("push_steps")) == int(push_rec.get("expected_push_steps"))
        )
    except (TypeError, ValueError):
        return False


def _object_sort_key(name: str) -> tuple[int, int, str]:
    if name.startswith("obj_"):
        suffix = name[4:]
        if suffix.isdigit():
            return (0, int(suffix), name)
    if name.startswith("wall_"):
        suffix = name[5:]
        if suffix.isdigit():
            return (1, int(suffix), name)
    return (2, 0, name)


def _select_rewind_scene_record(mid_obs: list[dict], push_rec: dict) -> dict | None:
    start_ts = _safe_float(push_rec.get("push_start_obs_timestamp"))
    if start_ts is None:
        return None

    best = None
    best_key = None
    for rec in mid_obs:
        ts = _safe_float(rec.get("observation_timestamp"))
        objects = rec.get("objects")
        if ts is None or not isinstance(objects, dict) or not objects:
            continue
        matches_push = _push_signature_matches(rec.get("active_subgoal"), push_rec)
        is_pushing = rec.get("push_state") == "PUSHING"
        key = (
            0 if matches_push else 1,
            0 if is_pushing else 1,
            abs(ts - start_ts),
        )
        if best_key is None or key < best_key:
            best = rec
            best_key = key
    return best


def _load_real_run_state(run_dir: Path, push_index: int = 0) -> dict:
    """Read one push-start scene from a real run directory.

    Returns the exact pushed-object pose, robot pose, selected edge/depth,
    and the recorded scene objects for the chosen push.
    """
    pushes_path = run_dir / "pushes.jsonl"
    subgoals_path = run_dir / "subgoals.jsonl"
    mid_obs_path = run_dir / "mid_obs.jsonl"
    if not pushes_path.exists():
        raise FileNotFoundError(f"{pushes_path} not found — was the real run aborted?")
    if not subgoals_path.exists():
        raise FileNotFoundError(f"{subgoals_path} not found")

    pushes = _load_jsonl(pushes_path)
    subgoals = _load_jsonl(subgoals_path)
    mid_obs = _load_jsonl(mid_obs_path)
    if not pushes:
        raise RuntimeError(f"{pushes_path} has no records — the real push never completed")
    if not subgoals:
        raise RuntimeError(f"{subgoals_path} has no records")
    if push_index < 0 or push_index >= len(pushes):
        raise RuntimeError(
            f"push index {push_index} out of range for {pushes_path} "
            f"(have {len(pushes)} push record(s))"
        )

    push_rec = pushes[push_index]
    # Match the subgoal record to this push by subgoal_id (typically 1 for
    # a one-trial spec, but be explicit).
    push_sgid = push_rec.get("subgoal_id")
    sg_rec = next((s for s in subgoals if s.get("subgoal_id") == push_sgid), subgoals[0])

    robot_pose = push_rec.get("robot_pose_before_cm_deg") or sg_rec.get("dispatched_robot_pose_cm")
    if robot_pose is None:
        raise RuntimeError(
            f"run is missing both robot_pose_before_cm_deg and dispatched_robot_pose_cm "
            f"({pushes_path}, {subgoals_path})"
        )

    scene_rec = _select_rewind_scene_record(mid_obs, push_rec) if mid_obs else None
    if not isinstance(scene_rec, dict):
        raise RuntimeError(
            f"real run is missing a rewindable scene record in {mid_obs_path}. "
            "This replay path now requires mid_obs.jsonl from execute_real_push."
        )
    if not isinstance(scene_rec.get("objects"), dict) or not scene_rec.get("objects"):
        raise RuntimeError(
            f"selected rewind scene in {mid_obs_path} has no objects payload "
            f"for push index {push_index}"
        )

    return {
        "object_id": push_rec["object_id"],
        "object_pose_before": tuple(push_rec["object_pose_before"]),  # (x, y, θ_deg)
        "robot_pose": tuple(robot_pose),                              # (x, y, θ_deg)
        "edge_idx": int(push_rec["expected_edge"]),
        "push_steps": int(push_rec["expected_push_steps"]),
        "push_index": push_index,
        "subgoal_id": push_sgid,
        "goal_cm": (
            tuple(scene_rec["goal_cm"])
            if isinstance(scene_rec.get("goal_cm"), list)
            and len(scene_rec.get("goal_cm")) >= 2
            else None
        ),
        "scene_objects": scene_rec.get("objects"),
        "scene_obs_timestamp": _safe_float(scene_rec.get("observation_timestamp")),
    }


def _load_initial_scene_state(run_dir: Path) -> dict:
    """Read the first scene-bearing frame of mid_obs.jsonl from a real run.

    Unlike :func:`_load_real_run_state`, this does not need pushes.jsonl or
    subgoals.jsonl — it captures the scene at the moment ArUco started
    publishing observations, which is the natural "initial state" of a real
    execution. Used by the --trial-spec mode where we want to start sim from
    the same scene the real run started from (not from a specific push's
    pre-state).
    """
    mid_obs_path = run_dir / "mid_obs.jsonl"
    if not mid_obs_path.exists():
        raise FileNotFoundError(
            f"{mid_obs_path} not found — --trial-spec mode reads the initial "
            "scene from the matching execute_real_push run."
        )
    mid_obs = _load_jsonl(mid_obs_path)
    if not mid_obs:
        raise RuntimeError(f"{mid_obs_path} has no records")

    initial_rec = None
    for rec in mid_obs:
        objects = rec.get("objects")
        if isinstance(objects, dict) and objects:
            initial_rec = rec
            break
    if initial_rec is None:
        raise RuntimeError(
            f"{mid_obs_path} has no frames with a populated 'objects' field — "
            "real-side ArUco may not have detected anything"
        )

    robot_pose = None
    for key in ("robot_pose_cm_deg", "robot_pose_cm"):
        val = initial_rec.get(key)
        if isinstance(val, list) and len(val) >= 2:
            robot_pose = (
                float(val[0]),
                float(val[1]),
                float(val[2]) if len(val) >= 3 else 0.0,
            )
            break
    if robot_pose is None:
        raise RuntimeError(
            f"first mid_obs frame in {mid_obs_path} lacks a robot pose field "
            "(robot_pose_cm_deg / robot_pose_cm)"
        )

    return {
        "robot_pose": robot_pose,                           # (x_cm, y_cm, θ_deg)
        "scene_objects": initial_rec["objects"],
        "goal_cm": (
            tuple(initial_rec["goal_cm"])
            if isinstance(initial_rec.get("goal_cm"), list)
            and len(initial_rec.get("goal_cm")) >= 2
            else None
        ),
        "scene_obs_timestamp": _safe_float(initial_rec.get("observation_timestamp")),
    }


def _build_initial_scene_xml(
    scene_state: dict,
    real_yaml_path: Path,
    output_xml: Path,
) -> dict:
    """Write a car XML for the initial scene; return obj_N -> sim_object_id map.

    Sibling of :func:`_build_rewind_xml`, but without the pushed-object
    pose override. Used by --trial-spec mode where the whole chain runs from
    the recorded *initial* scene, and per-push positions evolve in sim
    physics rather than being seeded from real-side pushes.jsonl.

    The mapping reflects how NAMOXMLGenerator.from_observation renames keys:
    the N-th movable in iteration order becomes ``obstacle_N_movable`` (and
    the N-th static becomes ``wall_{4+N}``). We iterate in the same sorted
    order :func:`_object_sort_key` produces (obj_N sorted by N first, then
    wall_N), so the mapping is deterministic and matches the generated XML.
    """
    recorded_scene_objects = scene_state.get("scene_objects")
    if not isinstance(recorded_scene_objects, dict) or not recorded_scene_objects:
        raise RuntimeError("scene_state has no scene_objects payload")

    objects: dict = {}
    obj_to_sim_id: dict = {}
    movable_count = 0
    for name in sorted(recorded_scene_objects.keys(), key=_object_sort_key):
        o = recorded_scene_objects[name]
        if not isinstance(o, dict):
            continue
        is_static = bool(o.get("is_static", False))
        objects[name] = (
            float(o["x_cm"]),
            float(o["y_cm"]),
            float(o["theta_deg"]),
            float(o["width_cm"]),
            float(o["depth_cm"]),
            float(o["height_cm"]),
            is_static,
        )
        if not is_static:
            movable_count += 1
            obj_to_sim_id[name] = f"obstacle_{movable_count}_movable"

    rx, ry, _ = scene_state["robot_pose"]

    ws_data = yaml.safe_load(real_yaml_path.read_text()) if real_yaml_path.exists() else {}
    ws_cfg = (ws_data.get("workspace") or {}) if ws_data else {}
    workspace_w = float(ws_cfg.get("width_cm", 49.0))
    workspace_h = float(ws_cfg.get("height_cm", 77.5))

    goal_cm = scene_state.get("goal_cm")
    if isinstance(goal_cm, tuple) and len(goal_cm) >= 2:
        goal_x = float(goal_cm[0])
        goal_y = float(goal_cm[1])
    else:
        goal_x = workspace_w - 5 if rx < workspace_w / 2 else 5
        goal_y = workspace_h - 5 if ry < workspace_h / 2 else 5

    gen = NAMOXMLGenerator(scale_factor=1.0, robot_model="car")
    xml_str = gen.from_observation(
        robot_x_cm=rx,
        robot_y_cm=ry,
        objects=objects,
        goal_x_cm=goal_x,
        goal_y_cm=goal_y,
        workspace_bounds_cm=(0.0, workspace_w, 0.0, workspace_h),
    )
    gen.save(xml_str, str(output_xml))

    print(
        f"[initial-scene] wrote {output_xml}  robot=({rx:.2f},{ry:.2f}) "
        f"movables={list(obj_to_sim_id.keys())} "
        f"-> sim={list(obj_to_sim_id.values())}",
        flush=True,
    )
    return obj_to_sim_id


def _build_rewind_xml(
    real_state: dict,
    real_yaml_path: Path,
    output_xml: Path,
) -> None:
    """Write a MuJoCo car XML matching the push-start state of a real run.

    Source is the recorded `mid_obs.jsonl` frame nearest the real push-start
    timestamp.
    """
    pushed_id = real_state["object_id"]
    bx, by, bth = real_state["object_pose_before"]
    recorded_scene_objects = real_state.get("scene_objects")

    objects: dict = {}
    scene_source = "recorded mid_obs"
    if not isinstance(recorded_scene_objects, dict) or not recorded_scene_objects:
        raise RuntimeError("recorded scene_objects payload is missing")
    for name in sorted(recorded_scene_objects.keys(), key=_object_sort_key):
        o = recorded_scene_objects[name]
        if not isinstance(o, dict):
            continue
        x = float(o["x_cm"])
        y = float(o["y_cm"])
        theta = float(o["theta_deg"])
        width = float(o["width_cm"])
        depth = float(o["depth_cm"])
        height = float(o["height_cm"])
        is_static = bool(o.get("is_static", False))
        if name == pushed_id:
            x, y, theta = bx, by, bth
        objects[name] = (x, y, theta, width, depth, height, is_static)

    rx, ry, _ = real_state["robot_pose"]

    # Workspace dims from real.yaml — same source capture_to_xml uses.
    ws_data = yaml.safe_load(real_yaml_path.read_text()) if real_yaml_path.exists() else {}
    ws_cfg = (ws_data.get("workspace") or {}) if ws_data else {}
    workspace_w = float(ws_cfg.get("width_cm", 49.0))
    workspace_h = float(ws_cfg.get("height_cm", 77.5))

    goal_cm = real_state.get("goal_cm")
    if isinstance(goal_cm, tuple) and len(goal_cm) >= 2:
        goal_x = float(goal_cm[0])
        goal_y = float(goal_cm[1])
    else:
        # Auto-goal: opposite corner from robot, same fallback as capture_to_xml.
        goal_x = workspace_w - 5 if rx < workspace_w / 2 else 5
        goal_y = workspace_h - 5 if ry < workspace_h / 2 else 5

    gen = NAMOXMLGenerator(scale_factor=1.0, robot_model="car")
    xml_str = gen.from_observation(
        robot_x_cm=rx,
        robot_y_cm=ry,
        objects=objects,
        goal_x_cm=goal_x,
        goal_y_cm=goal_y,
        workspace_bounds_cm=(0.0, workspace_w, 0.0, workspace_h),
    )
    gen.save(xml_str, str(output_xml))
    print(
        f"[rewind] wrote {output_xml}  robot=({rx:.2f},{ry:.2f}) "
        f"{pushed_id}={bx:.2f},{by:.2f},θ={bth:.1f}° "
        f"(push_index={real_state.get('push_index', 0)}, "
        f"subgoal_id={real_state.get('subgoal_id')}, "
        f"scene={scene_source})",
        flush=True,
    )


def _compute_edge_starting_pose_sim(
    scene_state: dict,
    object_id: str,
    edge_idx: int,
) -> Tuple[float, float, float]:
    """Return (x_m, y_m, θ_rad) of the edge-point teleport pose for a push.

    Same standoff / points-per-face the C++ NAMOPushSkill uses internally
    (points_per_face=15, standoff = 0.6 * effective_robot_size(7,7) = 3.6 cm).
    Pre-teleporting the car here means the C++ wavefront reachability check
    starts from a free cell — see the long comment in the single-push branch
    of :func:`main` for why this matters.
    """
    import math as _math
    from robot_control.utils.robot_geometry import effective_robot_size_cm
    from robot_control.core.types import ObjectPose

    scene_objects = scene_state.get("scene_objects") or {}
    o = scene_objects.get(object_id)
    if not isinstance(o, dict):
        raise KeyError(
            f"object {object_id!r} not in scene_state.scene_objects "
            f"(have {list(scene_objects.keys())})"
        )
    obj_pose = ObjectPose(
        x=float(o["x_cm"]),
        y=float(o["y_cm"]),
        theta=float(o["theta_deg"]),
        width=float(o["width_cm"]),
        depth=float(o["depth_cm"]),
        height=float(o["height_cm"]),
        is_static=False,
    )
    car_size_cm = effective_robot_size_cm(7.0, 7.0)
    standoff_cm = 0.6 * car_size_cm
    ep = get_edge_point(obj_pose, int(edge_idx), standoff_cm, 15)
    return (
        float(ep.position[0]) / 100.0,
        float(ep.position[1]) / 100.0,
        _math.radians(float(ep.approach_theta)),
    )


def _build_chain_from_trial_spec(
    trials: list,
    obj_to_sim_id: dict,
) -> list:
    """Validate trials + resolve obj_N -> sim_object_id; return chain payload."""
    movables = list(obj_to_sim_id.keys())
    chain: list = []
    for i, trial in enumerate(trials, start=1):
        if not isinstance(trial, dict):
            raise ValueError(f"trial {i} is not a mapping: {trial!r}")
        obj_id = trial.get("object_id")
        if not obj_id:
            if len(movables) == 1:
                obj_id = movables[0]
            else:
                raise ValueError(
                    f"trial {i} omits object_id but scene has "
                    f"{len(movables)} movables {movables}; pin one explicitly"
                )
        if obj_id not in obj_to_sim_id:
            raise ValueError(
                f"trial {i} object_id={obj_id!r} not in scene movables "
                f"{movables}"
            )
        try:
            edge_idx = int(trial["edge_idx"])
        except (KeyError, ValueError, TypeError):
            raise ValueError(f"trial {i} missing/invalid edge_idx")
        if "push_steps" in trial:
            push_steps = int(trial["push_steps"])
        elif "depth" in trial:
            push_steps = int(trial["depth"]) + 1
        else:
            raise ValueError(f"trial {i} missing depth/push_steps")
        chain.append({
            "object_id": obj_id,
            "sim_object_id": obj_to_sim_id[obj_id],
            "edge_idx": edge_idx,
            "push_steps": push_steps,
            "depth": push_steps - 1,
        })
    return chain


# Top-down render constants mirror sim_replay_subprocess so the still
# image lines up exactly with what the MP4 (and run_namo --viewer) shows:
# elevation -90° looks straight down; azimuth 90° gives +X right / +Y up;
# distance is the workspace extent scaled by 1.6× (same as the MP4).
_FINAL_PNG_WIDTH = 1280
_FINAL_PNG_HEIGHT = 720
_FINAL_PNG_CAMERA_DISTANCE_FACTOR = 1.6


def _read_last_qpos(qpos_dump_path: Path) -> Optional[list]:
    """Parse the qpos dump and return the last valid line's qpos vector."""
    if not qpos_dump_path.exists():
        return None
    last_q: Optional[list] = None
    with open(qpos_dump_path) as f:
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
                last_q = q
    return last_q


def _render_final_scene_image(
    xml_path: Path,
    qpos_dump_path: Path,
    output_path: Path,
) -> None:
    """Top-down MuJoCo PNG matching sim_push.mp4's camera setup.

    Same renderer / camera that ``sim_replay_subprocess`` uses for the
    chain MP4 (and that ``run_namo.py --viewer`` shows live): free camera,
    looking straight down at the workspace center, framed by the world
    bounds. If ``qpos_dump_path`` has parsable frames, we set the model's
    qpos to the last one (= final state at chain halt or completion);
    otherwise we render the XML's baked initial pose (the chain failed
    before any push dumped state).
    """
    import xml.etree.ElementTree as _ET
    import numpy as _np
    import mujoco

    # Inject offscreen framebuffer size, same as sim_replay_subprocess does.
    # MuJoCo's default is 640×480 — anything larger errors out at Renderer
    # init unless <visual><global offwidth/offheight/></visual> is set.
    root = _ET.parse(str(xml_path)).getroot()
    visual = root.find("visual")
    if visual is None:
        visual = _ET.SubElement(root, "visual")
    glob = visual.find("global")
    if glob is None:
        glob = _ET.SubElement(visual, "global")
    glob.set("offwidth", str(_FINAL_PNG_WIDTH))
    glob.set("offheight", str(_FINAL_PNG_HEIGHT))
    xml_str = _ET.tostring(root, encoding="unicode")

    model = mujoco.MjModel.from_xml_string(xml_str)
    data = mujoco.MjData(model)

    # Apply final qpos if available. Skip on dim mismatch — better to render
    # the initial pose than to crash.
    last_q = _read_last_qpos(qpos_dump_path)
    if last_q is not None and len(last_q) == model.nq:
        data.qpos[:] = last_q
    mujoco.mj_forward(model, data)

    # Workspace bounds from boundary walls (wall_1 / wall_2 / wall_3 /
    # wall_4 — same convention NAMOXMLGenerator emits and the C++ side's
    # env.get_world_bounds() uses).
    def _geom_pos(name: str):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        return None if gid < 0 else _np.asarray(model.geom_pos[gid])
    p1 = _geom_pos("wall_1")
    p2 = _geom_pos("wall_2")
    p3 = _geom_pos("wall_3")
    p4 = _geom_pos("wall_4")
    if p1 is not None and p2 is not None and p3 is not None and p4 is not None:
        cx = 0.5 * (p1[0] + p2[0])
        cy = 0.5 * (p3[1] + p4[1])
        extent = max(p2[0] - p1[0], p4[1] - p3[1])
    else:
        # Fall back to the data extents if walls aren't named as expected.
        cx, cy = 0.0, 0.0
        extent = 1.0

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [cx, cy, 0.0]
    camera.distance = extent * _FINAL_PNG_CAMERA_DISTANCE_FACTOR
    camera.azimuth = 90.0
    camera.elevation = -90.0

    renderer = mujoco.Renderer(model, height=_FINAL_PNG_HEIGHT, width=_FINAL_PNG_WIDTH)
    try:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render()
    finally:
        renderer.close()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image as _PIL_Image
        _PIL_Image.fromarray(rgb).save(str(output_path))
    except ImportError:
        import cv2 as _cv2
        _cv2.imwrite(str(output_path), rgb[:, :, ::-1])


# --------------------------------------------------------------------- main

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--mujoco-xml",
        default=None,
        help="Path to a captured MuJoCo car XML (from capture_to_xml.py "
             "--robot-model car --scale-factor 1.0). Required unless "
             "--from-real-run is set, in which case the XML is built from "
             "the real run's recorded pre-push state.",
    )
    p.add_argument(
        "--edge-idx",
        type=int,
        default=None,
        help="Edge index to push (0..4*points_per_face-1; 0..59 with "
             "points_per_face=15). See scripts/show_edges.py to visualise. "
             "Auto-filled from real's pushes.jsonl when --from-real-run is set.",
    )
    p.add_argument(
        "--depth",
        type=int,
        default=None,
        help="Push depth. push_steps fed to the controller = depth + 1. "
             "Auto-filled from real's pushes.jsonl when --from-real-run is set.",
    )
    p.add_argument(
        "--from-real-run",
        type=str,
        default=None,
        metavar="DIR",
        help="Path to a execute_real_push run directory (the one "
             "containing pushes.jsonl + subgoals.jsonl + mid_obs.jsonl. "
             "When set, the sim starts from the recorded "
             "PUSH-start state of that real run — same robot pose, same "
             "pushed-object pose, same recorded scene objects — so sim/real "
             "are directly diff-able. Overrides --mujoco-xml, --edge-idx, "
             "--depth, --object-id, --robot-pose with values read from the "
             "real run.",
    )
    p.add_argument(
        "--push-index",
        type=int,
        default=0,
        help="0-based push record index to replay from --from-real-run. "
             "Useful when one execute_real_push run contains many pushes "
             "(default: 0).",
    )
    p.add_argument(
        "--camera-service",
        type=str,
        default="tcp://localhost:5556",
        metavar="ADDRESS",
        help="Deprecated no-op in execute_sim_push. Retained for CLI "
             "compatibility. Default: tcp://localhost:5556.",
    )
    p.add_argument(
        "--object-id",
        default=None,
        help="Name of the object to push. Defaults to auto-detect when there "
             "is exactly one movable in the scene.",
    )
    p.add_argument(
        "--mujoco-viewer",
        action="store_true",
        help="Deprecated no-op in C++ replay mode. Inspect sim_push.mp4 instead.",
    )
    p.add_argument(
        "--robot-pose",
        type=float,
        nargs=3,
        metavar=("X_CM", "Y_CM", "THETA_DEG"),
        default=None,
        help="Starting robot pose in workspace coords. Required for car "
             "XMLs (little_car.xml bakes a 0,0 spawn that won't match the "
             "captured scene). Copy this from capture_to_xml.py's summary "
             "line (e.g. 'Robot: (37.97, 5.53) cm @ 83.2°').",
    )
    p.add_argument(
        "--no-gui",
        action="store_true",
        help="Deprecated no-op in C++ replay mode.",
    )
    p.add_argument(
        "--config",
        default="config/real.yaml",
        help="real.yaml — only used by --from-real-run to rebuild the rewind "
             "XML with the right workspace bounds. (default: config/real.yaml)",
    )
    p.add_argument(
        "--diag-path",
        default="/tmp/sim_push",
        help="Root diagnostics directory (default: /tmp/sim_push — throwaway).",
    )
    p.add_argument(
        "--run-name",
        default="sim_push_{date}_{time}",
        help="Subdirectory name under --diag-path.",
    )
    p.add_argument("--push-speed", type=float, default=None,
                   help="Deprecated no-op in C++ replay mode.")
    p.add_argument("--nav-speed", type=float, default=None,
                   help="Deprecated no-op in C++ replay mode.")
    p.add_argument("--allow-overwrite", action="store_true",
                   help="Reuse existing --run-name directory.")
    p.add_argument(
        "--no-video",
        action="store_true",
        help="Skip the MP4 encoding step. Physics + Tier 2 artifact "
             "extraction still run. Cuts ~5-10 s per push and avoids the "
             "GPU-bound renderer init — use during tuning loops.",
    )
    p.add_argument(
        "--log-mid-pos",
        action="store_true",
        help="Deprecated no-op. C++ replay now always writes mid_obs.jsonl "
             "when diagnostics are enabled.",
    )
    p.add_argument(
        "--trial-spec",
        type=str,
        default=None,
        metavar="YAML",
        help="Path to a trial_spec.yaml (the same format execute_real_push "
             "consumes). When set, every trial in the spec runs as a single "
             "continuous chain in one sim subprocess — physics state evolves "
             "between trials, no XML regeneration between pushes. Requires "
             "--real-run-dir for the initial scene. Mutually exclusive with "
             "--from-real-run, --mujoco-xml, --edge-idx, --depth.",
    )
    p.add_argument(
        "--real-run-dir",
        type=str,
        default=None,
        metavar="DIR",
        help="execute_real_push run directory whose first mid_obs.jsonl frame "
             "becomes the initial scene for --trial-spec mode. Typically "
             "<env>/solution/real_push_execution/.",
    )
    return p.parse_args()


def _run_trial_spec_mode(args, recorder, log_file) -> int:
    """--trial-spec dispatch: run all trials as one continuous sim chain.

    Source of truth for the initial scene is the first ArUco-detected frame
    of --real-run-dir/mid_obs.jsonl. Sim physics propagates state between
    pushes — no XML regeneration between trials. On the first push-skill
    rejection (e.g. edge not reachable), the chain halts and the final
    scene reflects the state where it stopped (see sim_replay_subprocess
    halt logic).
    """
    if (args.from_real_run or args.mujoco_xml or args.edge_idx is not None
            or args.depth is not None or args.robot_pose is not None):
        print(
            "Error: --trial-spec is mutually exclusive with --from-real-run / "
            "--mujoco-xml / --edge-idx / --depth / --robot-pose.",
            file=sys.stderr,
        )
        return 2
    if not args.real_run_dir:
        print(
            "Error: --trial-spec requires --real-run-dir (path to an "
            "execute_real_push run containing mid_obs.jsonl).",
            file=sys.stderr,
        )
        return 2

    spec_path = Path(args.trial_spec)
    if not spec_path.exists():
        print(f"Error: --trial-spec file not found: {spec_path}", file=sys.stderr)
        return 2
    spec = yaml.safe_load(spec_path.read_text()) or {}
    trials = spec.get("trials") or []
    if not trials:
        print(f"Error: no 'trials' in {spec_path}", file=sys.stderr)
        return 2

    real_run_dir = Path(args.real_run_dir)
    try:
        scene_state = _load_initial_scene_state(real_run_dir)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"Error: --real-run-dir: {exc}", file=sys.stderr)
        return 2

    initial_xml = Path(recorder.root) / "initial_scene.xml"
    try:
        obj_to_sim_id = _build_initial_scene_xml(
            scene_state, Path(args.config), initial_xml
        )
    except Exception as exc:
        print(f"Error: failed to build initial-scene XML: {exc!r}",
              file=sys.stderr)
        return 2

    try:
        chain = _build_chain_from_trial_spec(trials, obj_to_sim_id)
    except ValueError as exc:
        print(f"Error: --trial-spec: {exc}", file=sys.stderr)
        return 2

    try:
        starting_pose_sim = _compute_edge_starting_pose_sim(
            scene_state, chain[0]["object_id"], chain[0]["edge_idx"]
        )
    except KeyError as exc:
        print(f"Error: edge-point teleport: {exc}", file=sys.stderr)
        return 2

    print(
        f"[trial-spec] {len(chain)} push(es) "
        f"chain={[(c['object_id'], c['edge_idx'], c['depth']) for c in chain]} "
        f"start_pose_sim=({starting_pose_sim[0]:.3f}m, "
        f"{starting_pose_sim[1]:.3f}m, "
        f"{starting_pose_sim[2]:.3f}rad)",
        flush=True,
    )

    from robot_control.diagnostics.sim_replay import render_chain_to_mp4

    output_mp4 = str(Path(recorder.root) / "sim_push.mp4")
    namo_cpp_config = (
        ROBOT_CONTROL_ROOT.parent / "namo_cpp" / "config"
        / "namo_config_complete_skill15_car_1x.yaml"
    )
    if not namo_cpp_config.exists():
        print(f"Error: required C++ config not found: {namo_cpp_config}",
              file=sys.stderr)
        return 2

    import os as _os
    prior_cwd = _os.getcwd()
    namo_cpp_dir = ROBOT_CONTROL_ROOT.parent / "namo_cpp"
    if namo_cpp_dir.exists():
        _os.chdir(str(namo_cpp_dir))
    try:
        rendered = render_chain_to_mp4(
            start_xml=str(initial_xml),
            namo_config=str(namo_cpp_config),
            chain=chain,
            output_mp4=output_mp4,
            artifact_dir=str(Path(recorder.root)),
            starting_robot_pose_sim=starting_pose_sim,
            skip_video=bool(args.no_video),
        )
    finally:
        _os.chdir(prior_cwd)

    diag_root = Path(recorder.root)
    chain_failed = rendered is None
    if chain_failed:
        print(
            "[exec-sim-push] chain halted before completion — see "
            "[sim_replay_subprocess] lines above for the failure reason.",
            file=sys.stderr,
        )

    # Top-down MuJoCo render of the final scene, matching the camera setup
    # sim_replay_subprocess uses for sim_push.mp4 (and run_namo.py --viewer).
    # Sources qpos from qpos_dump_full.txt (the C++ side writes it during
    # every push; partial chains preserve it via the failure-halt copy in
    # sim_replay_subprocess.py). If no qpos is available the renderer falls
    # back to the XML's baked initial pose, so a deliverable always lands.
    qpos_dump = diag_root / "qpos_dump_full.txt"
    final_image = diag_root / "final_scene.png"
    try:
        _render_final_scene_image(initial_xml, qpos_dump, final_image)
        print(f"[exec-sim-push] wrote {final_image}", flush=True)
    except Exception as exc:
        print(f"[exec-sim-push] WARN: final_scene render failed: {exc!r}",
              file=sys.stderr)

    if log_file:
        try:
            log_file.close()
        except Exception:
            pass

    print()
    print("=" * 60)
    status = "FAILED" if chain_failed else "OK"
    print(f"Chain status: {status}. Records under {diag_root}")
    if final_image.exists():
        print(f"  - {final_image}     (deliverable: top-down final scene)")
    print(f"  - {initial_xml}")
    if not args.no_video and not chain_failed:
        print(f"  - {output_mp4}")
    print(f"  - {diag_root / 'pushes.jsonl'}  (one row per completed push)")
    print(f"  - {diag_root / 'mid_obs.jsonl'}")
    print(f"  - {diag_root / 'run.log'}")
    print("=" * 60)
    return 1 if chain_failed else 0


def main() -> int:
    args = parse_args()

    # Same diagnostics bootstrap execute_real_push uses.
    recorder, log_file = bootstrap_diagnostics(args)
    if recorder is None or not recorder.enabled:
        print("Error: --diag-path is required.", file=sys.stderr)
        return 2
    args._diagnostics_recorder = recorder

    # --trial-spec mode: take an execute_real_push-style trial spec, build a
    # car XML from the matching real run's recorded initial scene, and run
    # the whole trial chain in one sim subprocess. Single-push --from-real-run
    # and --mujoco-xml modes (below) are untouched.
    if args.trial_spec:
        return _run_trial_spec_mode(args, recorder, log_file)

    # --from-real-run mode: rewind sim to the exact pre-push state of a
    # execute_real_push run. Auto-fills every CLI arg that's normally
    # required (xml, edge, depth, object_id, robot_pose). Writes an XML to
    # the diag dir so the run is self-contained.
    if args.from_real_run:
        real_dir = Path(args.from_real_run)
        try:
            real_state = _load_real_run_state(real_dir, push_index=int(args.push_index))
        except (FileNotFoundError, RuntimeError) as exc:
            print(f"Error: --from-real-run: {exc}", file=sys.stderr)
            return 2
        rewind_xml = Path(recorder.root) / "rewind_scene.xml"
        try:
            _build_rewind_xml(
                real_state,
                Path(args.config),
                rewind_xml,
            )
        except Exception as exc:
            print(f"Error: --from-real-run: failed to build rewind XML: {exc!r}",
                  file=sys.stderr)
            return 2
        # Overlay onto args so the rest of main() doesn't need branching.
        args.mujoco_xml = str(rewind_xml)
        args.edge_idx = real_state["edge_idx"]
        args.depth = real_state["push_steps"] - 1
        args.object_id = real_state["object_id"]
        args.robot_pose = list(real_state["robot_pose"])
        print(
            f"[rewind] sim starts at: robot={tuple(args.robot_pose)}  "
            f"object={args.object_id} pose={real_state['object_pose_before']}  "
            f"edge={args.edge_idx} depth={args.depth} "
            f"push_index={real_state.get('push_index', 0)} "
            f"subgoal_id={real_state.get('subgoal_id')}",
            flush=True,
        )

    # Now the standard required-arg validation (after rewind has filled them).
    if args.mujoco_xml is None:
        print("Error: --mujoco-xml is required (or use --from-real-run).",
              file=sys.stderr)
        return 2
    if args.edge_idx is None:
        print("Error: --edge-idx is required (or use --from-real-run).",
              file=sys.stderr)
        return 2
    if args.depth is None:
        print("Error: --depth is required (or use --from-real-run).",
              file=sys.stderr)
        return 2
    if args.depth < 0:
        print(f"Error: --depth must be >= 0, got {args.depth}", file=sys.stderr)
        return 2
    push_steps = args.depth + 1

    # ===== C++ NAMOPushController execution =====
    # We dispatch the push through the C++ NAMOPushController (via sim_replay)
    # rather than running a Python pure-pursuit loop in this process. The
    # Python loop ran at the Runtime control rate (30 Hz) while MuJoCo physics
    # ran at 500 Hz, holding each wheel command for ~17 physics ticks —
    # producing the speed-oscillation and contact-bouncing artifacts we used
    # to see in execute_sim_push videos.
    #
    # The C++ NAMOPushController runs the same PushPathFollower pure-pursuit
    # algorithm but updates wheel commands every single physics tick. It also
    # does:
    #   * env_.set_zero_velocity()  — zero every velocity in the scene
    #   * 100-tick pre-push settle  — let chassis drop onto wheels, kill
    #                                 contact transients
    # before the push loop begins. Mirroring sim_replay through
    # render_chain_to_mp4() gives us the same control loop here.
    from robot_control.diagnostics.sim_replay import render_chain_to_mp4  # noqa: E402

    # Build the chain entry the way sim_replay expects.
    # object_id is the planner-side display name (e.g., "obj_1").
    # sim_object_id is the MuJoCo body name (e.g., "obstacle_1_movable")
    # — required by the C++ env_.get_object_state() lookup.
    object_id = args.object_id or "obj_1"
    if object_id.startswith("obj_") and object_id[4:].isdigit():
        sim_object_id = f"obstacle_{object_id[4:]}_movable"
    else:
        sim_object_id = object_id  # caller supplied a body name directly

    chain = [{
        "object_id": object_id,
        "sim_object_id": sim_object_id,
        "edge_idx": int(args.edge_idx),
        "push_steps": int(push_steps),
        "depth": int(args.depth),
    }]

    # Starting robot pose in sim units (meters + radians).
    #
    # WHY WE TELEPORT TO THE EDGE POINT (not the captured real-robot pose):
    # The C++ NAMOPushSkill (namo_push_skill.cpp:163-170) runs
    # `get_reachable_edges_with_wavefront(object_name)` BEFORE the push and
    # rejects with "Requested edge N not reachable" if the wavefront can't
    # find a path from the robot's current pose to the requested edge. For
    # --from-real-run, the captured real-robot pose can be inside obstacle
    # inflation (the Python rewind builder logs "Trapped-start recovery"
    # when this happens). The C++ wavefront, rebuilt from the XML at
    # runtime, doesn't have that local-cell clearing applied, so it sees
    # the start as trapped and refuses every edge.
    #
    # Fix: pre-teleport the car to the edge point itself, with the push
    # heading. The wavefront's start cell is then the edge point (a free
    # cell, since the standoff distance puts it outside the object's
    # inflation), reachability check passes, and the C++ controller's own
    # set_robot_se2() teleport at namo_push_controller.cpp:441 becomes a
    # no-op. The push primitive then runs normally.
    starting_pose_sim: Optional[Tuple[float, float, float]] = None
    import math as _math
    if args.robot_pose is not None:
        # Compute the edge point from the rewind XML's object state and
        # the requested edge_idx. Use the same defaults as the C++ skill:
        # points_per_face=15 (matches PushConfig.points_per_face), and a
        # standoff that maps the C++ "push_offset_margin_" semantics. For
        # the default car_size=7cm and standoff_multiplier=0.6, that's
        # 0.6 * effective_robot_size_cm(7,7) = 0.6 * 6 = 3.6 cm.
        from robot_control.utils.robot_geometry import effective_robot_size_cm
        from robot_control.core.types import ObjectPose

        # Pull the pushed object's pose out of the rewind XML so the edge
        # point matches what the C++ env will compute internally.
        import xml.etree.ElementTree as _ET
        _tree = _ET.parse(args.mujoco_xml)
        _root = _tree.getroot()
        _obj_geom = None
        for _g in _root.iter("geom"):
            if _g.get("name") == sim_object_id:
                _obj_geom = _g
                break
        if _obj_geom is None:
            print(f"Error: could not find geom {sim_object_id!r} in "
                  f"{args.mujoco_xml}", file=sys.stderr)
            return 2
        _pos = [float(v) for v in _obj_geom.get("pos", "0 0 0").split()]
        _quat = [float(v) for v in _obj_geom.get("quat", "1 0 0 0").split()]
        _size = [float(v) for v in _obj_geom.get("size", "0 0 0").split()]
        # Yaw from quaternion (w, x, y, z); rotation is about Z only here.
        _yaw_rad = _math.atan2(
            2.0 * (_quat[0] * _quat[3] + _quat[1] * _quat[2]),
            1.0 - 2.0 * (_quat[2] * _quat[2] + _quat[3] * _quat[3]),
        )
        # ObjectPose convention: depth=X-extent, width=Y-extent (in cm).
        _obj_pose = ObjectPose(
            x=_pos[0] * 100.0,
            y=_pos[1] * 100.0,
            theta=_math.degrees(_yaw_rad),
            width=_size[1] * 2.0 * 100.0,   # Y-extent → width
            depth=_size[0] * 2.0 * 100.0,   # X-extent → depth
            height=_size[2] * 2.0 * 100.0,
            is_static=False,
        )
        _car_size_cm = effective_robot_size_cm(7.0, 7.0)  # default 7×7 robot
        _standoff_cm = 0.6 * _car_size_cm
        _ep = get_edge_point(_obj_pose, int(args.edge_idx),
                             _standoff_cm, 15)
        # Convert cm + degrees → meters + radians for the C++ env API.
        starting_pose_sim = (
            float(_ep.position[0]) / 100.0,
            float(_ep.position[1]) / 100.0,
            _math.radians(float(_ep.approach_theta)),
        )
        print(f"[exec-sim-push] pre-teleport to edge point: "
              f"({_ep.position[0]:.2f}, {_ep.position[1]:.2f}) cm, "
              f"θ={_ep.approach_theta:.2f}° "
              f"(standoff {_standoff_cm:.1f} cm from face)", flush=True)

    output_mp4 = str(Path(recorder.root) / "sim_push.mp4")

    # CRITICAL: pass the same C++ config run_namo.py uses (skill15_car_1x).
    # Without a config, C++ falls back to points_per_edge=3 (12 edge points
    # total), so any edge >= 12 is reported as "not reachable". For
    # points_per_face=15 (60 edge points), use the skill15_car_1x config.
    namo_cpp_config = (
        ROBOT_CONTROL_ROOT.parent / "namo_cpp" / "config"
        / "namo_config_complete_skill15_car_1x.yaml"
    )
    if not namo_cpp_config.exists():
        print(f"Error: required C++ config not found: {namo_cpp_config}",
              file=sys.stderr)
        return 2
    print(
        f"[exec-sim-push] dispatching to C++ NAMOPushController via "
        f"render_chain_to_mp4: xml={args.mujoco_xml} chain={chain} "
        f"start_pose_sim={starting_pose_sim} namo_config={namo_cpp_config.name}",
        flush=True,
    )

    # CWD must be namo_cpp/ when the subprocess runs because motion-primitive
    # paths in the C++ config are resolved relative to that directory
    # (sim_replay_subprocess inherits CWD; same constraint NAMOPlanBridge has).
    import os as _os
    prior_cwd = _os.getcwd()
    namo_cpp_dir = ROBOT_CONTROL_ROOT.parent / "namo_cpp"
    if namo_cpp_dir.exists():
        _os.chdir(str(namo_cpp_dir))
    try:
        rendered = render_chain_to_mp4(
            start_xml=str(args.mujoco_xml),
            namo_config=str(namo_cpp_config),
            chain=chain,
            output_mp4=output_mp4,
            artifact_dir=str(Path(recorder.root)),
            starting_robot_pose_sim=starting_pose_sim,
            skip_video=bool(args.no_video),
        )
    finally:
        _os.chdir(prior_cwd)

    if rendered is None:
        print("[exec-sim-push] render_chain_to_mp4 failed (see [sim_replay] "
              "lines above)", file=sys.stderr)
        if log_file:
            try:
                log_file.close()
            except Exception:
                pass
        return 1

    diag_root = Path(recorder.root)
    wheel_log_path = diag_root / "wheel_commands.jsonl"
    try:
        tier2_summary = export_tier2_push_trials(
            diag_root,
            wheel_log_path,
            source="sim",
        )
    except Exception as exc:
        print(f"[exec-sim-push] tier2 export failed: {exc!r}", file=sys.stderr)
        if log_file:
            try:
                log_file.close()
            except Exception:
                pass
        return 1

    # These args are leftovers from the legacy Python-runtime path. The
    # replay subprocess always writes the full pose stream now, so
    # --log-mid-pos is redundant rather than unsupported.
    for unused_arg, val in (
        ("nav_speed", args.nav_speed),
        ("push_speed", args.push_speed),
        ("mujoco_viewer", args.mujoco_viewer),
        ("no_gui", args.no_gui),
    ):
        if val:
            print(f"[exec-sim-push] WARNING: --{unused_arg.replace('_','-')} "
                  f"has no effect in C++-controller mode (sim_replay path).",
                  flush=True)
    if args.log_mid_pos:
        print(
            "[exec-sim-push] NOTE: --log-mid-pos is redundant here; "
            "mid_obs.jsonl is written automatically.",
            flush=True,
        )

    if log_file:
        try:
            log_file.close()
        except Exception:
            pass

    print()
    print("=" * 60)
    print(f"Done. Records under {diag_root}")
    print(f"  - {output_mp4}     (C++ NAMOPushController, 500 Hz pure pursuit + pre-settle)")
    print(f"  - {diag_root / 'pushes.jsonl'}")
    print(f"  - {diag_root / 'subgoals.jsonl'}")
    print(f"  - {diag_root / 'mid_obs.jsonl'}")
    print(f"  - {diag_root / 'wheel_commands.jsonl'}")
    print(f"  - {diag_root / 'qpos_dump_full.txt'}")
    print(f"  - {diag_root / 'tier2_push_trials'}  ({tier2_summary.get('exported_trial_count', 0)} trials)")
    print(f"  - {diag_root / 'run.log'}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
