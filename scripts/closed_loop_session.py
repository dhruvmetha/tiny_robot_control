#!/usr/bin/env python3
"""Manage closed-loop NAMO session directories under ``closed_loop_sessions``.

This wrapper is intentionally additive: it prepares per-run closed-loop
artifacts and shells out to the existing scripts for sim planning and real
execution without mutating ``real_test_envs`` or changing runtime logic.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
import time
import types
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional
from xml.etree import ElementTree as ET

import yaml


HERE = Path(__file__).resolve().parent
ROBOT_CONTROL_ROOT = HERE.parent
SRC = ROBOT_CONTROL_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


LAYOUT_VERSION = 2
DEFAULT_SESSION_GLOB = "bootstrap_from_real_test_envs_*"
GOAL_TOLERANCE_CM = 5.0
RESET_TOLERANCE_CM = 3.0
EXPECTED_SCENE_FILES = ("env.xml", "mid_obs.jsonl", "env.png", "scene.jpg")


def _utc_now() -> tuple[float, str]:
    epoch = time.time()
    iso = datetime.fromtimestamp(epoch, tz=timezone.utc).isoformat().replace("+00:00", "Z")
    return epoch, iso


def _safe_float(value: Any) -> Optional[float]:
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def _quat_to_yaw_deg(quat_str: str) -> float:
    parts = quat_str.split()
    if len(parts) != 4:
        return 0.0
    w, x, y, z = (float(v) for v in parts)
    return math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )


def _read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    with open(path, "r") as f:
        return json.load(f)


def _write_json_atomic(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".tmp_{path.name}")
    with open(tmp, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def _write_yaml_atomic(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".tmp_{path.name}")
    with open(tmp, "w") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
    os.replace(tmp, path)


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    if not path.exists():
        return records
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if isinstance(rec, dict):
                records.append(rec)
    return records


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".tmp_{path.name}")
    with open(tmp, "w") as f:
        for rec in records:
            f.write(json.dumps(rec) + "\n")
    os.replace(tmp, path)


def _read_text(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text()


def _copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _clear_dir(path: Path) -> None:
    if path.exists() or path.is_symlink():
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
        else:
            path.unlink()
    path.mkdir(parents=True, exist_ok=True)


def _copy_tree_contents(src_dir: Path, dst_dir: Path) -> None:
    dst_dir.mkdir(parents=True, exist_ok=True)
    for child in sorted(src_dir.iterdir(), key=lambda p: p.name):
        dst = dst_dir / child.name
        if child.is_dir():
            shutil.copytree(child, dst, dirs_exist_ok=True)
        else:
            shutil.copy2(child, dst)


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


def _run_sort_key(name: str) -> tuple[int, str]:
    if name.startswith("run") and name[3:].isdigit():
        return (int(name[3:]), name)
    return (10**9, name)


def _iter_sort_key(name: str) -> tuple[int, str]:
    if name.startswith("iter_") and name[5:].isdigit():
        return (int(name[5:]), name)
    return (10**9, name)


def _iter_dir(run_dir: Path, iteration: int) -> Path:
    return run_dir / f"iter_{iteration:03d}"


def _bootstrap_sessions(closed_loop_root: Path) -> list[Path]:
    pattern = f"*/*/*/sessions/{DEFAULT_SESSION_GLOB}"
    return sorted(
        p for p in closed_loop_root.glob(pattern)
        if p.is_dir()
    )


def _workspace_from_real_yaml(real_yaml_path: Path) -> dict[str, float]:
    data = yaml.safe_load(real_yaml_path.read_text()) if real_yaml_path.exists() else {}
    ws = (data.get("workspace") or {}) if isinstance(data, dict) else {}
    return {
        "width_cm": float(ws.get("width_cm", 49.0)),
        "height_cm": float(ws.get("height_cm", 77.5)),
    }


def _optional_path(value: Any) -> Optional[Path]:
    if isinstance(value, str) and value.strip():
        return Path(value)
    return None


def _load_namo_xml_generator_class():
    """Import NAMOXMLGenerator without executing robot_control/__init__.py.

    The package __init__ imports GUI modules unconditionally, which makes
    simple headless helpers depend on PySide6. We only need the utils
    package here, so seed lightweight namespace-package stubs first.
    """
    robot_control_pkg = sys.modules.get("robot_control")
    if robot_control_pkg is None:
        robot_control_pkg = types.ModuleType("robot_control")
        robot_control_pkg.__path__ = [str(SRC / "robot_control")]
        sys.modules["robot_control"] = robot_control_pkg

    utils_pkg = sys.modules.get("robot_control.utils")
    if utils_pkg is None:
        utils_pkg = types.ModuleType("robot_control.utils")
        utils_pkg.__path__ = [str(SRC / "robot_control" / "utils")]
        sys.modules["robot_control.utils"] = utils_pkg

    module = importlib.import_module("robot_control.utils.xml_generator")
    return module.NAMOXMLGenerator


def _planner_car_1x_config_path() -> Path:
    config_path = (
        ROBOT_CONTROL_ROOT.parent
        / "namo_cpp"
        / "config"
        / "namo_config_complete_skill15_car_1x.yaml"
    )
    if not config_path.exists():
        raise FileNotFoundError(f"{config_path} not found")
    return config_path


def _goal_probe_python_bin() -> Path:
    env_bin = os.environ.get("PYTHON_BIN")
    if env_bin:
        return Path(env_bin)
    if sys.version_info[:2] == (3, 12):
        return Path(sys.executable)
    common_env = Path.home() / "miniconda3" / "envs" / "namo312" / "bin" / "python"
    if common_env.exists():
        return common_env
    return Path(sys.executable)


def _goal_wavefront_reachable(
    env_xml_path: Path,
    robot_pose_cm: tuple[float, float, float],
    goal_cm: tuple[float, float],
) -> bool:
    """Check planner-side goal reachability from the current robot pose.

    Uses the same 1x car NAMO config as the sim random-rollout planner, so
    this is the closed-loop wrapper's unified reachability criterion.
    """
    python_bin = _goal_probe_python_bin()
    config_path = _planner_car_1x_config_path()
    loader_path = SRC / "robot_control" / "planner" / "namo_binding_loader.py"
    namo_cpp_dir = ROBOT_CONTROL_ROOT.parent / "namo_cpp"
    probe_code = r"""
import importlib.util
import math
import os
import sys
from pathlib import Path

anchor = Path(sys.argv[1])
xml_path = Path(sys.argv[2])
config_path = Path(sys.argv[3])
robot_x_cm = float(sys.argv[4])
robot_y_cm = float(sys.argv[5])
robot_theta_deg = float(sys.argv[6])
goal_x_cm = float(sys.argv[7])
goal_y_cm = float(sys.argv[8])
loader_path = Path(sys.argv[9])
namo_cpp_dir = Path(sys.argv[10])

spec = importlib.util.spec_from_file_location("_closed_loop_namo_binding_loader", loader_path)
if spec is None or spec.loader is None:
    raise RuntimeError(f"could not load namo_binding_loader from {loader_path}")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
namo_rl, _, _ = module.load_canonical_namo_rl(anchor)

original_cwd = os.getcwd()
os.chdir(str(namo_cpp_dir))
try:
    env = namo_rl.RLEnvironment(str(xml_path), str(config_path), False, True)
    env.set_robot_pose(robot_x_cm / 100.0, robot_y_cm / 100.0, math.radians(robot_theta_deg))
    env.warm_up()
    env.set_robot_goal(goal_x_cm / 100.0, goal_y_cm / 100.0, 0.0)
    print("1" if env.is_robot_goal_reachable() else "0")
finally:
    os.chdir(original_cwd)
"""
    proc = subprocess.run(
        [
            str(python_bin),
            "-c",
            probe_code,
            str(Path(__file__).resolve()),
            str(env_xml_path),
            str(config_path),
            str(robot_pose_cm[0]),
            str(robot_pose_cm[1]),
            str(robot_pose_cm[2]),
            str(goal_cm[0]),
            str(goal_cm[1]),
            str(loader_path),
            str(namo_cpp_dir),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "goal wavefront probe failed "
            f"(python={python_bin}, code={proc.returncode}): "
            f"{proc.stderr.strip() or proc.stdout.strip()}"
        )
    return proc.stdout.strip().splitlines()[-1].strip() == "1"


def _real_push_failure_mode(real_push_dir: Path) -> Optional[str]:
    run_log = _read_text(real_push_dir / "run.log")
    if "Approach position unreachable:" in run_log:
        return "approach_unreachable"
    return None


def _extract_goal_from_env_xml(env_xml_path: Path) -> Optional[tuple[float, float]]:
    if not env_xml_path.exists():
        return None
    root = ET.parse(env_xml_path).getroot()
    for site in root.findall(".//site"):
        if site.get("name") != "goal":
            continue
        pos = site.get("pos")
        if not pos:
            continue
        parts = pos.split()
        if len(parts) < 2:
            continue
        return float(parts[0]) * 100.0, float(parts[1]) * 100.0
    return None


def _first_scene_record(mid_obs_path: Path) -> dict[str, Any]:
    for rec in _load_jsonl(mid_obs_path):
        if isinstance(rec.get("objects"), dict) and rec.get("objects"):
            return rec
    raise RuntimeError(f"{mid_obs_path} has no scene-bearing record")


def _canonical_goal_from_start_state(start_state_dir: Path) -> tuple[float, float]:
    mid_obs_path = start_state_dir / "mid_obs.jsonl"
    if mid_obs_path.exists():
        rec = _first_scene_record(mid_obs_path)
        goal = rec.get("goal_cm")
        if isinstance(goal, list) and len(goal) >= 2:
            return float(goal[0]), float(goal[1])
    goal = _extract_goal_from_env_xml(start_state_dir / "env.xml")
    if goal is not None:
        return goal
    raise RuntimeError(
        f"could not determine canonical goal from {start_state_dir / 'mid_obs.jsonl'} "
        f"or {start_state_dir / 'env.xml'}"
    )


def _scene_before_state(scene_before_dir: Path) -> dict[str, Any]:
    rec = _first_scene_record(scene_before_dir / "mid_obs.jsonl")
    robot_pose = None
    for key in ("robot_pose_cm", "robot_pose_cm_deg"):
        val = rec.get(key)
        if isinstance(val, list) and len(val) >= 3:
            robot_pose = (float(val[0]), float(val[1]), float(val[2]))
            break
    if robot_pose is None:
        raise RuntimeError(f"{scene_before_dir / 'mid_obs.jsonl'} missing robot_pose_cm")
    scene_objects = rec.get("objects")
    if not isinstance(scene_objects, dict) or not scene_objects:
        raise RuntimeError(f"{scene_before_dir / 'mid_obs.jsonl'} missing objects")
    goal = rec.get("goal_cm")
    goal_cm = (
        (float(goal[0]), float(goal[1]))
        if isinstance(goal, list) and len(goal) >= 2
        else None
    )
    return {
        "robot_pose": robot_pose,
        "scene_objects": scene_objects,
        "goal_cm": goal_cm,
        "observation_timestamp": _safe_float(rec.get("observation_timestamp")),
        "at_epoch": _safe_float(rec.get("at_epoch")),
    }


def _mapping_from_scene_objects(scene_objects: dict[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    real_to_sim: dict[str, str] = {}
    sim_to_real: dict[str, str] = {}
    movable_count = 0
    for name in sorted(scene_objects.keys(), key=_object_sort_key):
        obj = scene_objects[name]
        if not isinstance(obj, dict):
            continue
        if bool(obj.get("is_static", False)):
            continue
        movable_count += 1
        sim_name = f"obstacle_{movable_count}_movable"
        real_to_sim[name] = sim_name
        sim_to_real[sim_name] = name
    return real_to_sim, sim_to_real


def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
    diff = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return abs(diff)


def _mapping_from_scene_env_xml(
    scene_objects: dict[str, Any],
    env_xml_path: Path,
) -> tuple[dict[str, str], dict[str, str]]:
    if not env_xml_path.exists():
        raise FileNotFoundError(f"{env_xml_path} not found")

    real_specs: dict[str, dict[str, float]] = {}
    for real_name, raw_obj in scene_objects.items():
        if not isinstance(raw_obj, dict) or bool(raw_obj.get("is_static", False)):
            continue
        if "pose_cm" in raw_obj:
            pose = raw_obj.get("pose_cm")
            if not isinstance(pose, list) or len(pose) < 3:
                continue
            x_cm = float(pose[0])
            y_cm = float(pose[1])
            theta_deg = float(pose[2])
        else:
            x_cm = float(raw_obj["x_cm"])
            y_cm = float(raw_obj["y_cm"])
            theta_deg = float(raw_obj.get("theta_deg", 0.0))
        real_specs[real_name] = {
            "x_cm": x_cm,
            "y_cm": y_cm,
            "theta_deg": theta_deg,
            "width_cm": float(raw_obj["width_cm"]),
            "depth_cm": float(raw_obj["depth_cm"]),
        }

    sim_specs: dict[str, dict[str, float]] = {}
    root = ET.parse(env_xml_path).getroot()
    for body in root.iter("body"):
        sim_name = body.get("name", "")
        if not (sim_name.startswith("obstacle_") and sim_name.endswith("_movable")):
            continue
        geom = body.find("geom")
        if geom is None:
            continue
        pos = [float(v) for v in geom.get("pos", "0 0 0").split()]
        size = [float(v) for v in geom.get("size", "0 0 0").split()]
        sim_specs[sim_name] = {
            "x_cm": pos[0] * 100.0,
            "y_cm": pos[1] * 100.0,
            "theta_deg": _quat_to_yaw_deg(geom.get("quat", "1 0 0 0")),
            "width_cm": size[1] * 200.0,
            "depth_cm": size[0] * 200.0,
        }

    unmatched_real = set(real_specs.keys())
    real_to_sim: dict[str, str] = {}
    sim_to_real: dict[str, str] = {}
    for sim_name, sim_spec in sorted(sim_specs.items()):
        best_real: Optional[str] = None
        best_score: Optional[tuple[float, float, float]] = None
        for real_name in sorted(unmatched_real, key=_object_sort_key):
            real_spec = real_specs[real_name]
            pos_err = math.hypot(
                sim_spec["x_cm"] - real_spec["x_cm"],
                sim_spec["y_cm"] - real_spec["y_cm"],
            )
            size_err = max(
                abs(sim_spec["width_cm"] - real_spec["width_cm"]),
                abs(sim_spec["depth_cm"] - real_spec["depth_cm"]),
            )
            theta_err = _angle_diff_deg(sim_spec["theta_deg"], real_spec["theta_deg"])
            score = (pos_err, size_err, theta_err)
            if best_score is None or score < best_score:
                best_score = score
                best_real = real_name
        if best_real is None or best_score is None:
            raise RuntimeError(f"could not match {sim_name} from {env_xml_path}")
        pos_err, size_err, theta_err = best_score
        if pos_err > 0.5 or size_err > 0.5 or theta_err > 5.0:
            raise RuntimeError(
                f"{env_xml_path}: best match for {sim_name} was {best_real} "
                f"but geometry mismatch too large "
                f"(pos_err={pos_err:.3f}cm size_err={size_err:.3f}cm "
                f"theta_err={theta_err:.3f}deg)"
            )
        real_to_sim[best_real] = sim_name
        sim_to_real[sim_name] = best_real
        unmatched_real.remove(best_real)

    if unmatched_real:
        raise RuntimeError(
            f"{env_xml_path}: unmatched real movable objects {sorted(unmatched_real)}"
        )
    return real_to_sim, sim_to_real


def _mapping_from_solution(
    solution: dict[str, Any],
    scene_objects: dict[str, Any],
) -> Optional[tuple[dict[str, str], dict[str, str]]]:
    raw_mapping = solution.get("object_mapping")
    if not isinstance(raw_mapping, dict):
        return None
    raw_real_to_sim = raw_mapping.get("real_to_sim")
    raw_sim_to_real = raw_mapping.get("sim_to_real")
    if not isinstance(raw_real_to_sim, dict) or not isinstance(raw_sim_to_real, dict):
        raise RuntimeError("solution.yaml object_mapping is malformed")

    real_to_sim: dict[str, str] = {}
    for real_name, sim_name in raw_real_to_sim.items():
        if not isinstance(real_name, str) or not isinstance(sim_name, str):
            raise RuntimeError("solution.yaml object_mapping.real_to_sim must map strings to strings")
        if real_name not in scene_objects:
            raise RuntimeError(
                f"solution.yaml object_mapping references unknown real object {real_name!r}"
            )
        real_to_sim[real_name] = sim_name

    sim_to_real: dict[str, str] = {}
    for sim_name, real_name in raw_sim_to_real.items():
        if not isinstance(sim_name, str) or not isinstance(real_name, str):
            raise RuntimeError("solution.yaml object_mapping.sim_to_real must map strings to strings")
        if real_name not in real_to_sim:
            raise RuntimeError(
                f"solution.yaml sim_to_real references real object {real_name!r} "
                "missing from real_to_sim"
            )
        if real_to_sim[real_name] != sim_name:
            raise RuntimeError(
                f"solution.yaml object_mapping is inconsistent for {real_name!r}: "
                f"{real_to_sim[real_name]!r} vs {sim_name!r}"
            )
        sim_to_real[sim_name] = real_name
    return real_to_sim, sim_to_real


def _scene_objects_to_generator_input(scene_objects: dict[str, Any]) -> dict[str, tuple[float, float, float, float, float, float, bool]]:
    converted: dict[str, tuple[float, float, float, float, float, float, bool]] = {}
    for name in sorted(scene_objects.keys(), key=_object_sort_key):
        obj = scene_objects[name]
        if not isinstance(obj, dict):
            continue
        if "pose_cm" in obj:
            pose = obj["pose_cm"]
            if not isinstance(pose, list) or len(pose) < 3:
                raise RuntimeError(f"object {name} has malformed pose_cm")
            x_cm = float(pose[0])
            y_cm = float(pose[1])
            theta_deg = float(pose[2])
            width_cm = float(obj["width_cm"])
            depth_cm = float(obj["depth_cm"])
            height_cm = float(obj["height_cm"])
            is_static = bool(obj["is_static"])
        else:
            x_cm = float(obj["x_cm"])
            y_cm = float(obj["y_cm"])
            theta_deg = float(obj["theta_deg"])
            width_cm = float(obj["width_cm"])
            depth_cm = float(obj["depth_cm"])
            height_cm = float(obj["height_cm"])
            is_static = bool(obj["is_static"])
        converted[name] = (
            x_cm,
            y_cm,
            theta_deg,
            width_cm,
            depth_cm,
            height_cm,
            is_static,
        )
    return converted


def _is_successful_seed_run(run_dir: Path) -> bool:
    solution_path = run_dir / "solution.yaml"
    if not solution_path.exists():
        return False
    data = yaml.safe_load(solution_path.read_text()) or {}
    return bool(data.get("success"))


def _seed_sources_from_pre_migration(session_dir: Path, old_meta: dict[str, Any]) -> dict[str, Path]:
    sim_candidates_dir = session_dir / "iter_000" / "sim_candidates"
    sources: dict[str, Path] = {}
    if sim_candidates_dir.is_dir():
        for child in sorted(sim_candidates_dir.iterdir(), key=lambda p: _run_sort_key(p.name)):
            if not child.exists():
                continue
            resolved = child.resolve()
            if resolved.is_dir() and _is_successful_seed_run(resolved):
                sources[child.name] = resolved
    root = _optional_path(old_meta.get("sim_candidate_source"))
    if root is not None:
        if root.is_dir():
            for child in sorted(root.iterdir(), key=lambda p: _run_sort_key(p.name)):
                if child.is_dir() and _is_successful_seed_run(child):
                    sources.setdefault(child.name, child.resolve())
    return sources


def _seed_sources_from_migrated_meta(meta: dict[str, Any]) -> dict[str, Path]:
    sources: dict[str, Path] = {}
    root = _optional_path(meta.get("source_random_rollout_root"))
    if root is not None:
        if root.is_dir():
            for child in sorted(root.iterdir(), key=lambda p: _run_sort_key(p.name)):
                if child.is_dir() and _is_successful_seed_run(child):
                    sources[child.name] = child.resolve()
    return sources


def _session_layout_version(meta: dict[str, Any]) -> Optional[int]:
    value = meta.get("layout_version")
    try:
        return None if value is None else int(value)
    except (TypeError, ValueError):
        return None


def _build_start_state(session_dir: Path, old_scene_before_dir: Path) -> Path:
    start_state_dir = session_dir / "start_state"
    _clear_dir(start_state_dir)
    for filename in EXPECTED_SCENE_FILES:
        src = old_scene_before_dir / filename
        if src.exists():
            _copy_file(src, start_state_dir / filename)
    return start_state_dir


def _write_session_meta(
    session_dir: Path,
    session_name: str,
    source_env: str,
    source_random_rollout_root: Path,
    canonical_goal_cm: tuple[float, float],
    workspace_cm: dict[str, float],
    initial_seed_runs: list[str],
) -> None:
    _, now_iso = _utc_now()
    payload = {
        "layout_version": LAYOUT_VERSION,
        "session_name": session_name,
        "source_env": source_env,
        "source_random_rollout_root": str(source_random_rollout_root),
        "canonical_goal_cm": [canonical_goal_cm[0], canonical_goal_cm[1]],
        "workspace_cm": workspace_cm,
        "initial_seed_runs": initial_seed_runs,
        "start_state_dir": "start_state",
        "migration_timestamp_utc": now_iso,
    }
    _write_json_atomic(session_dir / "session_meta.json", payload)


def _write_run_status(iter_dir: Path, iteration: int, state: str, successful_sim_candidates: list[str]) -> None:
    payload = {
        "iteration": iteration,
        "state": state,
        "scene_before_ready": True,
        "successful_sim_candidates": successful_sim_candidates,
        "selected_candidate": None,
        "real_push_completed": False,
        "scene_after_ready": False,
        "goal_reached": False,
        "goal_wavefront_reachable": False,
        "goal_success_criterion": None,
        "real_failure_mode": None,
        "replan_same_iteration": False,
        "final_distance_to_goal_cm": None,
    }
    _write_json_atomic(iter_dir / "status.json", payload)


def _materialize_seed_run(
    session_dir: Path,
    start_state_dir: Path,
    run_name: str,
    seed_source: Path,
) -> None:
    run_dir = session_dir / run_name
    iter1_dir = _iter_dir(run_dir, 1)
    scene_before_dir = iter1_dir / "scene_before"
    candidate_dir = iter1_dir / "sim_candidates" / "candidate1"
    real_push_dir = iter1_dir / "real_push"
    scene_after_dir = iter1_dir / "scene_after"

    if not run_dir.exists():
        run_dir.mkdir(parents=True, exist_ok=True)

    if not scene_before_dir.exists():
        _clear_dir(scene_before_dir)
        _copy_tree_contents(start_state_dir, scene_before_dir)

    if not candidate_dir.exists():
        candidate_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(seed_source, candidate_dir)

    real_push_dir.mkdir(parents=True, exist_ok=True)
    scene_after_dir.mkdir(parents=True, exist_ok=True)

    solution_path = candidate_dir / "solution.yaml"
    solution = yaml.safe_load(solution_path.read_text()) if solution_path.exists() else {}
    _, now_iso = _utc_now()
    run_meta = {
        "run_name": run_name,
        "seed_source_run": run_name,
        "seed_source_path": str(seed_source),
        "iter_001_candidate_dir": "iter_001/sim_candidates/candidate1",
        "seed_solution_success": bool((solution or {}).get("success")),
        "created_at_utc": now_iso,
    }
    _write_json_atomic(run_dir / "run_meta.json", run_meta)

    if not (iter1_dir / "status.json").exists():
        _write_run_status(iter1_dir, 1, "seeded", ["candidate1"])


def migrate_bootstrap_session(session_dir: Path) -> dict[str, Any]:
    session_dir = session_dir.resolve()
    session_name = session_dir.name
    meta_path = session_dir / "session_meta.json"
    existing_meta = _read_json(meta_path, default={}) or {}
    workspace_cm = _workspace_from_real_yaml(ROBOT_CONTROL_ROOT / "config" / "real.yaml")

    old_iter_dir = session_dir / "iter_000"
    old_scene_before_dir = old_iter_dir / "scene_before"
    migration_path = session_dir / "migration_from_bootstrap.json"

    if old_scene_before_dir.is_dir():
        source_env = str(existing_meta.get("source_env") or "")
        seed_sources = _seed_sources_from_pre_migration(session_dir, existing_meta)
        if not source_env:
            raise RuntimeError(f"{meta_path} missing source_env")

        start_state_dir = _build_start_state(session_dir, old_scene_before_dir)
        canonical_goal_cm = _canonical_goal_from_start_state(start_state_dir)

        source_random_rollout_root = _optional_path(existing_meta.get("sim_candidate_source"))
        if source_random_rollout_root is None:
            raise RuntimeError(f"{meta_path} missing sim_candidate_source")
        seed_runs = sorted(seed_sources.keys(), key=_run_sort_key)

        migration_payload = {
            "migrated_at_utc": _utc_now()[1],
            "source_paths": {
                "session_dir": str(session_dir),
                "old_iter_000_dir": str(old_iter_dir),
                "old_scene_before_dir": str(old_scene_before_dir),
                "source_random_rollout_root": str(source_random_rollout_root),
            },
            "old_session_meta": existing_meta,
            "old_iter_000_status": _read_json(old_iter_dir / "status.json", default={}) or {},
            "seed_sources": {run_name: str(path) for run_name, path in seed_sources.items()},
        }
        _write_json_atomic(migration_path, migration_payload)
        _write_session_meta(
            session_dir=session_dir,
            session_name=session_name,
            source_env=source_env,
            source_random_rollout_root=source_random_rollout_root,
            canonical_goal_cm=canonical_goal_cm,
            workspace_cm=workspace_cm,
            initial_seed_runs=seed_runs,
        )
        for run_name in seed_runs:
            _materialize_seed_run(session_dir, start_state_dir, run_name, seed_sources[run_name])
        shutil.rmtree(old_iter_dir)
        return {
            "session_dir": str(session_dir),
            "migrated": True,
            "seed_runs": seed_runs,
            "mode": "from_bootstrap_iter_000",
        }

    if _session_layout_version(existing_meta) != LAYOUT_VERSION:
        raise RuntimeError(
            f"{session_dir} is neither an old bootstrap session nor a migrated "
            f"layout_version={LAYOUT_VERSION} session"
        )

    start_state_dir = session_dir / "start_state"
    if not start_state_dir.is_dir():
        raise RuntimeError(f"{start_state_dir} missing from migrated session")

    seed_sources = _seed_sources_from_migrated_meta(existing_meta)
    seed_runs = sorted(seed_sources.keys(), key=_run_sort_key)
    for run_name in seed_runs:
        _materialize_seed_run(session_dir, start_state_dir, run_name, seed_sources[run_name])

    canonical_goal = tuple(existing_meta.get("canonical_goal_cm") or _canonical_goal_from_start_state(start_state_dir))
    source_random_rollout_root = _optional_path(existing_meta.get("source_random_rollout_root"))
    if source_random_rollout_root is None:
        raise RuntimeError(f"{meta_path} missing source_random_rollout_root")
    _write_session_meta(
        session_dir=session_dir,
        session_name=str(existing_meta.get("session_name") or session_name),
        source_env=str(existing_meta.get("source_env") or ""),
        source_random_rollout_root=source_random_rollout_root,
        canonical_goal_cm=(float(canonical_goal[0]), float(canonical_goal[1])),
        workspace_cm=existing_meta.get("workspace_cm") or workspace_cm,
        initial_seed_runs=seed_runs,
    )
    return {
        "session_dir": str(session_dir),
        "migrated": False,
        "seed_runs": seed_runs,
        "mode": "refresh_existing_layout",
    }


def _load_session_meta(session_dir: Path) -> dict[str, Any]:
    meta = _read_json(session_dir / "session_meta.json", default=None)
    if not isinstance(meta, dict):
        raise RuntimeError(f"{session_dir / 'session_meta.json'} missing or malformed")
    if _session_layout_version(meta) != LAYOUT_VERSION:
        raise RuntimeError(f"{session_dir} has unsupported layout_version")
    return meta


def _load_iteration_status(iter_dir: Path) -> dict[str, Any]:
    status = _read_json(iter_dir / "status.json", default={}) or {}
    if not isinstance(status, dict):
        raise RuntimeError(f"{iter_dir / 'status.json'} malformed")
    return status


def _update_status(iter_dir: Path, patch: dict[str, Any]) -> dict[str, Any]:
    status = _load_iteration_status(iter_dir)
    status.update(patch)
    _write_json_atomic(iter_dir / "status.json", status)
    return status


def _resolve_real_object_id(
    plan_object_id: str,
    scene_objects: dict[str, Any],
    solution: Optional[dict[str, Any]] = None,
    env_xml_path: Optional[Path] = None,
) -> tuple[str, dict[str, str], dict[str, str]]:
    real_to_sim: dict[str, str]
    sim_to_real: dict[str, str]
    if isinstance(solution, dict):
        resolved = _mapping_from_solution(solution, scene_objects)
        if resolved is not None:
            real_to_sim, sim_to_real = resolved
        elif env_xml_path is not None:
            real_to_sim, sim_to_real = _mapping_from_scene_env_xml(scene_objects, env_xml_path)
        else:
            real_to_sim, sim_to_real = _mapping_from_scene_objects(scene_objects)
    elif env_xml_path is not None:
        real_to_sim, sim_to_real = _mapping_from_scene_env_xml(scene_objects, env_xml_path)
    else:
        real_to_sim, sim_to_real = _mapping_from_scene_objects(scene_objects)
    if plan_object_id in sim_to_real:
        return sim_to_real[plan_object_id], real_to_sim, sim_to_real
    if plan_object_id in real_to_sim:
        return plan_object_id, real_to_sim, sim_to_real
    raise RuntimeError(
        f"plan object_id {plan_object_id!r} is not present in scene mapping "
        f"(real={sorted(real_to_sim.keys())}, sim={sorted(sim_to_real.keys())})"
    )


def prepare_real_push(session_dir: Path, run_name: str, iteration: int) -> dict[str, Any]:
    session_meta = _load_session_meta(session_dir)
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    candidate_dir = iter_dir / "sim_candidates" / "candidate1"
    solution_path = candidate_dir / "solution.yaml"
    if not solution_path.exists():
        raise FileNotFoundError(f"{solution_path} not found")

    solution = yaml.safe_load(solution_path.read_text()) or {}
    if not bool(solution.get("success")):
        raise RuntimeError(f"{solution_path} is not a successful plan")

    plan = solution.get("plan")
    if not isinstance(plan, list) or not plan:
        raise RuntimeError(f"{solution_path} has no push plan entries to execute")

    first_push = plan[0]
    if not isinstance(first_push, dict):
        raise RuntimeError(f"first plan entry in {solution_path} is malformed")

    scene_before = _scene_before_state(iter_dir / "scene_before")
    scene_objects = scene_before["scene_objects"]
    real_object_id, real_to_sim, sim_to_real = _resolve_real_object_id(
        str(first_push["object_id"]),
        scene_objects,
        solution=solution,
        env_xml_path=iter_dir / "scene_before" / "env.xml",
    )
    target = scene_objects.get(real_object_id)
    if not isinstance(target, dict):
        raise RuntimeError(f"{real_object_id!r} missing from scene_before objects")

    obstacle_target = [float(target["x_cm"]), float(target["y_cm"])]
    edge_idx = int(first_push["edge_idx"])
    push_steps = int(first_push["push_steps"])
    goal_cm = session_meta.get("canonical_goal_cm")
    trial_spec = {
        "obstacle_target": obstacle_target,
        "reset_tolerance_cm": RESET_TOLERANCE_CM,
        "trials": [
            {
                "object_id": real_object_id,
                "edge_idx": edge_idx,
                "push_steps": push_steps,
            }
        ],
    }
    selected_plan = {
        "prepared_at_utc": _utc_now()[1],
        "run_name": run_name,
        "iteration": iteration,
        "candidate_name": "candidate1",
        "solution_path": str(solution_path),
        "goal_cm": goal_cm,
        "plan_success": True,
        "full_plan": plan,
        "selected_push_index": 0,
        "selected_push": {
            "sim_object_id": str(first_push["object_id"]),
            "real_object_id": real_object_id,
            "edge_idx": edge_idx,
            "push_steps": push_steps,
        },
        "object_mapping": {
            "real_to_sim": real_to_sim,
            "sim_to_real": sim_to_real,
        },
        "obstacle_target_cm": obstacle_target,
    }

    trial_spec_path = iter_dir / "selected_trial_spec.yaml"
    selected_plan_path = iter_dir / "selected_plan.json"
    launch_script_path = iter_dir / "launch_real_push.sh"
    _write_yaml_atomic(trial_spec_path, trial_spec)
    _write_json_atomic(selected_plan_path, selected_plan)

    launch_lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        'PYTHON_BIN="${PYTHON_BIN:-python}"',
        f"cd {shlex_quote(str(ROBOT_CONTROL_ROOT))}",
        f'"$PYTHON_BIN" {shlex_quote(str(HERE / "execute_real_push.py"))} \\',
        f"  --config {shlex_quote(str(ROBOT_CONTROL_ROOT / 'config' / 'real.yaml'))} \\",
        "  --camera-service tcp://localhost:5556 \\",
        f"  --trial-spec {shlex_quote(str(trial_spec_path))} \\",
        f"  --diag-path {shlex_quote(str(iter_dir))} \\",
        "  --run-name real_push \\",
        "  --capture-scene \\",
        "  --record-video \\",
        "  --nav-speed 0.4 \\",
        "  --push-speed 0.4 \\",
        "  --no-reset-check \\",
        "  --allow-overwrite",
        "",
    ]
    launch_script_path.write_text("\n".join(launch_lines))
    launch_script_path.chmod(0o755)

    _update_status(
        iter_dir,
        {
            "state": "real_push_prepared",
            "selected_candidate": "candidate1",
            "selected_plan_path": "selected_plan.json",
            "selected_trial_spec_path": "selected_trial_spec.yaml",
            "launch_real_push_script": "launch_real_push.sh",
        },
    )
    return {
        "session_dir": str(session_dir),
        "run_name": run_name,
        "iteration": iteration,
        "selected_real_object_id": real_object_id,
        "edge_idx": edge_idx,
        "push_steps": push_steps,
        "launch_script": str(launch_script_path),
    }


def shlex_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _robot_and_objects_from_scene_after(scene_after: dict[str, Any]) -> tuple[tuple[float, float, float], dict[str, Any]]:
    robot = scene_after.get("robot")
    objects = scene_after.get("objects")
    if not isinstance(robot, dict):
        raise RuntimeError("scene_after.json missing robot payload")
    pose = robot.get("pose_cm")
    if not isinstance(pose, list) or len(pose) < 3:
        raise RuntimeError("scene_after.json robot.pose_cm is malformed")
    if not isinstance(objects, dict) or not objects:
        raise RuntimeError("scene_after.json missing objects payload")
    robot_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
    return robot_pose, objects


def _generate_scene_bundle(
    scene_after_payload: dict[str, Any],
    goal_cm: tuple[float, float],
    workspace_cm: dict[str, float],
    output_dir: Path,
) -> tuple[float, dict[str, Any]]:
    NAMOXMLGenerator = _load_namo_xml_generator_class()
    robot_pose, objects = _robot_and_objects_from_scene_after(scene_after_payload)
    generator_input = _scene_objects_to_generator_input(objects)
    gen = NAMOXMLGenerator(scale_factor=1.0, robot_model="car")
    xml_str = gen.from_observation(
        robot_x_cm=robot_pose[0],
        robot_y_cm=robot_pose[1],
        objects=generator_input,
        goal_x_cm=goal_cm[0],
        goal_y_cm=goal_cm[1],
        workspace_bounds_cm=(0.0, workspace_cm["width_cm"], 0.0, workspace_cm["height_cm"]),
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    env_xml_path = output_dir / "env.xml"
    env_png_path = output_dir / "env.png"
    gen.save(xml_str, str(env_xml_path))
    resolved = gen.get_resolved_robot_pos()
    if resolved is not None:
        robot_pos_m = resolved
    else:
        robot_pos_m = (robot_pose[0] / 100.0, robot_pose[1] / 100.0)
    gen.save_wavefront(str(env_png_path), robot_pos_m)

    captured_epoch = _safe_float(scene_after_payload.get("captured_at_epoch")) or time.time()
    captured_iso = datetime.fromtimestamp(captured_epoch, tz=timezone.utc).isoformat().replace("+00:00", "Z")
    rec = {
        "at_epoch": captured_epoch,
        "at_utc_iso": captured_iso,
        "t_since_logger_start_s": 0.0,
        "observation_timestamp": _safe_float(scene_after_payload.get("captured_at_observation_epoch")) or captured_epoch,
        "robot_pose_cm": [robot_pose[0], robot_pose[1], robot_pose[2]],
        "objects": {},
        "goal_cm": [goal_cm[0], goal_cm[1]],
        "push_state": "IDLE",
        "_note": (
            "Synthetic closed-loop snapshot derived from execute_real_push "
            "scene_after.json so env.xml and robot_pose_cm stay in sync."
        ),
    }
    for name in sorted(objects.keys(), key=_object_sort_key):
        obj = objects[name]
        if not isinstance(obj, dict):
            continue
        pose = obj.get("pose_cm")
        if not isinstance(pose, list) or len(pose) < 3:
            raise RuntimeError(f"scene_after object {name!r} has malformed pose_cm")
        rec["objects"][name] = {
            "x_cm": float(pose[0]),
            "y_cm": float(pose[1]),
            "theta_deg": float(pose[2]),
            "width_cm": float(obj["width_cm"]),
            "depth_cm": float(obj["depth_cm"]),
            "height_cm": float(obj["height_cm"]),
            "is_static": bool(obj["is_static"]),
        }
    _write_jsonl(output_dir / "mid_obs.jsonl", [rec])
    dist = math.hypot(robot_pose[0] - goal_cm[0], robot_pose[1] - goal_cm[1])
    return dist, rec


def advance_iteration(session_dir: Path, run_name: str, iteration: int, allow_overwrite: bool) -> dict[str, Any]:
    session_meta = _load_session_meta(session_dir)
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    real_push_dir = iter_dir / "real_push"
    scene_after_json = real_push_dir / "scene_after.json"
    scene_after_jpg = real_push_dir / "scene_after.jpg"
    if not scene_after_json.exists():
        raise FileNotFoundError(f"{scene_after_json} not found")
    if not scene_after_jpg.exists():
        raise FileNotFoundError(f"{scene_after_jpg} not found")

    scene_after_payload = _read_json(scene_after_json, default=None)
    if not isinstance(scene_after_payload, dict):
        raise RuntimeError(f"{scene_after_json} is malformed")

    goal = session_meta.get("canonical_goal_cm")
    if not isinstance(goal, list) or len(goal) < 2:
        raise RuntimeError(f"{session_dir / 'session_meta.json'} missing canonical_goal_cm")
    goal_cm = (float(goal[0]), float(goal[1]))
    workspace_cm = session_meta.get("workspace_cm") or _workspace_from_real_yaml(ROBOT_CONTROL_ROOT / "config" / "real.yaml")

    scene_after_dir = iter_dir / "scene_after"
    if scene_after_dir.exists() and any(scene_after_dir.iterdir()) and not allow_overwrite:
        raise RuntimeError(
            f"{scene_after_dir} already contains artifacts; pass --allow-overwrite to regenerate"
        )
    _clear_dir(scene_after_dir)
    dist, synthetic_rec = _generate_scene_bundle(scene_after_payload, goal_cm, workspace_cm, scene_after_dir)
    _copy_file(scene_after_jpg, scene_after_dir / "scene.jpg")

    robot_pose_cm = (
        float(synthetic_rec["robot_pose_cm"][0]),
        float(synthetic_rec["robot_pose_cm"][1]),
        float(synthetic_rec["robot_pose_cm"][2]),
    )
    goal_reached_by_distance = dist < GOAL_TOLERANCE_CM
    goal_wavefront_reachable = _goal_wavefront_reachable(
        scene_after_dir / "env.xml",
        robot_pose_cm,
        goal_cm,
    )
    goal_reached = goal_reached_by_distance or goal_wavefront_reachable
    goal_success_criterion = (
        "distance"
        if goal_reached_by_distance
        else "wavefront"
        if goal_wavefront_reachable
        else None
    )
    real_failure_mode = _real_push_failure_mode(real_push_dir)
    replan_same_iteration = (not goal_reached) and (real_failure_mode == "approach_unreachable")
    _update_status(
        iter_dir,
        {
            "state": "goal_reached" if goal_reached else "awaiting_replan",
            "real_push_completed": True,
            "scene_after_ready": True,
            "goal_reached": goal_reached,
            "goal_wavefront_reachable": goal_wavefront_reachable,
            "goal_success_criterion": goal_success_criterion,
            "real_failure_mode": real_failure_mode,
            "replan_same_iteration": replan_same_iteration,
            "final_distance_to_goal_cm": dist,
        },
    )

    created_next_iter = None
    if not goal_reached and not replan_same_iteration:
        next_iter = iteration + 1
        next_iter_dir = _iter_dir(run_dir, next_iter)
        if next_iter_dir.exists():
            if not allow_overwrite:
                raise RuntimeError(
                    f"{next_iter_dir} already exists; pass --allow-overwrite to replace it"
                )
            shutil.rmtree(next_iter_dir)
        (next_iter_dir / "sim_candidates").mkdir(parents=True, exist_ok=True)
        (next_iter_dir / "real_push").mkdir(parents=True, exist_ok=True)
        (next_iter_dir / "scene_after").mkdir(parents=True, exist_ok=True)
        _copy_tree_contents(scene_after_dir, next_iter_dir / "scene_before")
        _write_run_status(next_iter_dir, next_iter, "awaiting_replan", [])
        created_next_iter = str(next_iter_dir)

    return {
        "session_dir": str(session_dir),
        "run_name": run_name,
        "iteration": iteration,
        "goal_reached": goal_reached,
        "goal_wavefront_reachable": goal_wavefront_reachable,
        "goal_success_criterion": goal_success_criterion,
        "real_failure_mode": real_failure_mode,
        "replan_same_iteration": replan_same_iteration,
        "final_distance_to_goal_cm": dist,
        "next_iteration_dir": created_next_iter,
        "scene_after_robot_pose_cm": synthetic_rec["robot_pose_cm"],
    }


def replan_iteration(session_dir: Path, run_name: str, iteration: int) -> dict[str, Any]:
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    scene_before_dir = iter_dir / "scene_before"
    sim_xml_path = scene_before_dir / "env.xml"
    if not sim_xml_path.exists():
        raise FileNotFoundError(f"{sim_xml_path} not found")

    candidate_dir = iter_dir / "sim_candidates" / "candidate1"
    if candidate_dir.exists():
        shutil.rmtree(candidate_dir)
    sim_candidates_dir = iter_dir / "sim_candidates"
    sim_candidates_dir.mkdir(parents=True, exist_ok=True)
    driver_log = iter_dir / "replan_candidate1_driver.log"
    python_bin = os.environ.get("PYTHON_BIN") or sys.executable
    cmd = [
        python_bin,
        "scripts/run_namo.py",
        "--sim",
        "--sim-xml",
        str(sim_xml_path),
        "--sim-real-run-dir",
        str(scene_before_dir),
        "--strategy",
        "random_rollout",
        "--rollout-samples-per-state",
        "36000",
        "--diag-path",
        str(sim_candidates_dir),
        "--run-name",
        "candidate1",
        "--allow-overwrite",
    ]
    child_env = os.environ.copy()
    existing_pythonpath = child_env.get("PYTHONPATH")
    child_env["PYTHONPATH"] = (
        f"{SRC}{os.pathsep}{existing_pythonpath}"
        if existing_pythonpath
        else str(SRC)
    )

    print(f"[closed-loop] replan {run_name} iter_{iteration:03d}")
    with open(driver_log, "w") as log_file:
        proc = subprocess.run(
            cmd,
            cwd=ROBOT_CONTROL_ROOT,
            env=child_env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )

    successful = candidate_dir.is_dir() and _is_successful_seed_run(candidate_dir)
    _update_status(
        iter_dir,
        {
            "state": "planned" if successful else "planning_failed",
            "successful_sim_candidates": ["candidate1"] if successful else [],
            "selected_candidate": None,
            "last_replan_exit_code": proc.returncode,
            "last_replan_driver_log": driver_log.name,
        },
    )
    return {
        "session_dir": str(session_dir),
        "run_name": run_name,
        "iteration": iteration,
        "candidate_dir": str(candidate_dir),
        "driver_log": str(driver_log),
        "exit_code": proc.returncode,
        "successful_plan": successful,
    }


def session_status(session_dir: Path, run_name: Optional[str]) -> dict[str, Any]:
    meta = _load_session_meta(session_dir)
    runs_root = []
    if run_name is not None:
        runs_root = [session_dir / run_name]
    else:
        runs_root = [p for p in session_dir.iterdir() if p.is_dir() and p.name.startswith("run")]
        runs_root.sort(key=lambda p: _run_sort_key(p.name))

    runs_payload: dict[str, Any] = {}
    for run_dir in runs_root:
        iter_dirs = [p for p in run_dir.iterdir() if p.is_dir() and p.name.startswith("iter_")]
        iter_dirs.sort(key=lambda p: _iter_sort_key(p.name))
        iterations_payload: dict[str, Any] = {}
        latest_status = None
        for iter_dir in iter_dirs:
            status = _load_iteration_status(iter_dir)
            iterations_payload[iter_dir.name] = status
            latest_status = status
        runs_payload[run_dir.name] = {
            "run_meta": _read_json(run_dir / "run_meta.json", default={}) or {},
            "latest_iteration": iter_dirs[-1].name if iter_dirs else None,
            "latest_status": latest_status,
            "iterations": iterations_payload,
        }

    return {
        "session_dir": str(session_dir),
        "session_meta": meta,
        "runs": runs_payload,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    migrate = sub.add_parser("migrate-bootstrap", help="Restructure old bootstrap sessions in place.")
    migrate.add_argument("--session-dir", type=Path, default=None, help="One bootstrap session directory to migrate.")
    migrate.add_argument(
        "--all-under-root",
        action="store_true",
        help="Migrate every bootstrap session under closed_loop_sessions.",
    )
    migrate.add_argument(
        "--closed-loop-root",
        type=Path,
        default=ROBOT_CONTROL_ROOT / "closed_loop_sessions",
        help="closed_loop_sessions root (default: repo-local path).",
    )

    prep = sub.add_parser("prepare-real-push", help="Write selected_plan/trial_spec/launch script for one iteration.")
    prep.add_argument("--session-dir", type=Path, required=True)
    prep.add_argument("--run", type=str, required=True)
    prep.add_argument("--iteration", type=int, required=True)

    advance = sub.add_parser("advance-iteration", help="Build scene_after and optionally iter_{k+1} from real push output.")
    advance.add_argument("--session-dir", type=Path, required=True)
    advance.add_argument("--run", type=str, required=True)
    advance.add_argument("--iteration", type=int, required=True)
    advance.add_argument("--allow-overwrite", action="store_true")

    replan = sub.add_parser("replan", help="Run one sim replan into sim_candidates/candidate1.")
    replan.add_argument("--session-dir", type=Path, required=True)
    replan.add_argument("--run", type=str, required=True)
    replan.add_argument("--iteration", type=int, required=True)

    status = sub.add_parser("status", help="Print session status JSON.")
    status.add_argument("--session-dir", type=Path, required=True)
    status.add_argument("--run", type=str, default=None)

    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    if args.command == "migrate-bootstrap":
        if args.session_dir is None and not args.all_under_root:
            raise SystemExit("migrate-bootstrap requires --session-dir or --all-under-root")
        if args.session_dir is not None and args.all_under_root:
            raise SystemExit("use either --session-dir or --all-under-root, not both")
        targets = [args.session_dir.resolve()] if args.session_dir is not None else _bootstrap_sessions(args.closed_loop_root.resolve())
        results = [migrate_bootstrap_session(target) for target in targets]
        print(json.dumps({"migrated_sessions": results}, indent=2))
        return 0

    if args.command == "prepare-real-push":
        result = prepare_real_push(args.session_dir.resolve(), args.run, args.iteration)
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "advance-iteration":
        result = advance_iteration(args.session_dir.resolve(), args.run, args.iteration, args.allow_overwrite)
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "replan":
        result = replan_iteration(args.session_dir.resolve(), args.run, args.iteration)
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "status":
        result = session_status(args.session_dir.resolve(), args.run)
        print(json.dumps(result, indent=2))
        return 0

    raise SystemExit(f"unknown command {args.command!r}")


if __name__ == "__main__":
    raise SystemExit(main())
