#!/usr/bin/env python3
"""Manage closed-loop NAMO session directories under ``closed_loop_sessions``.

This wrapper is intentionally additive: it prepares per-run closed-loop
artifacts and shells out to the existing scripts for sim planning and real
execution without mutating ``real_test_envs`` or changing runtime logic.
"""

from __future__ import annotations

import argparse
import hashlib
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

from robot_control.planner.namo_binding_loader import resolve_namo_cpp_dir  # noqa: E402
from robot_control.utils.scene_xml import portable_scene_path  # noqa: E402


LAYOUT_VERSION = 2
DEFAULT_SESSION_GLOB = "bootstrap_from_real_test_envs_*"
GOAL_TOLERANCE_CM = 5.0
RESET_TOLERANCE_CM = 3.0
MIN_EXECUTION_PUSH_STEPS = 2

# The held region-opening target lives at RUN level, not iteration level.
# Creating iter_N+1 rewrites status.json from a fixed literal and regenerates
# scene_after/ wholesale, so anything stored there is lost at the boundary. A
# run-level file also survives `shutil.rmtree(next_iter_dir)` and rides along
# in the Amarel submit rsync, which copies the whole run directory.
ACTIVE_TARGET_FILENAME = "active_region_opening.json"
MAX_REPLAN_ATTEMPTS = 8
EXPECTED_SCENE_FILES = ("env.xml", "mid_obs.jsonl", "env.png", "scene.jpg")
RUN_STRATEGIES = ("primitive", "random_rollout")
RUNS_PER_STRATEGY = 3


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


def _strategy_run_name(strategy: str, index: int) -> str:
    return f"{strategy}_run{index}"


def _parse_run_name(name: str) -> Optional[tuple[str, int, bool]]:
    if name.startswith("run") and name[3:].isdigit():
        return ("random_rollout", int(name[3:]), True)
    for strategy in RUN_STRATEGIES:
        prefix = f"{strategy}_run"
        if name.startswith(prefix) and name[len(prefix):].isdigit():
            return (strategy, int(name[len(prefix):]), False)
    return None


def _run_sort_key(name: str) -> tuple[int, int, str]:
    parsed = _parse_run_name(name)
    if parsed is None:
        return (10**9, 10**9, name)
    strategy, index, is_legacy = parsed
    strategy_order = {strategy_name: idx for idx, strategy_name in enumerate(RUN_STRATEGIES)}
    if is_legacy:
        return (strategy_order.get("random_rollout", 10**9), index, name)
    return (strategy_order.get(strategy, 10**9), index, name)


def _iter_sort_key(name: str) -> tuple[int, str]:
    if name.startswith("iter_") and name[5:].isdigit():
        return (int(name[5:]), name)
    return (10**9, name)


def _is_run_dir(path: Path) -> bool:
    if not path.is_dir():
        return False
    if (path / "run_meta.json").exists():
        return True
    return _parse_run_name(path.name) is not None


def _load_run_meta(run_dir: Path) -> dict[str, Any]:
    meta = _read_json(run_dir / "run_meta.json", default={}) or {}
    if isinstance(meta, dict):
        return meta
    return {}


def _load_active_region_target(run_dir: Path):
    """The boundary this run is holding, or None.

    Imported lazily so the pure-filesystem subcommands keep working without the
    planner package on sys.path.
    """
    try:
        from robot_control.planner.region_target import RegionOpeningTarget
    except ImportError:
        return None
    try:
        return RegionOpeningTarget.load(run_dir / ACTIVE_TARGET_FILENAME)
    except (ValueError, json.JSONDecodeError, OSError) as exc:
        # A job killed mid-write leaves half a file; a schema bump raises. The
        # session still has work to do, so treat either as no held target rather
        # than killing the command.
        print(f"[closed_loop] ignoring unreadable active target: {exc}")
        return None


def _dispatched_push(real_push_dir: Path) -> Optional[tuple[str, int, Optional[float]]]:
    """The last push the executor dispatched: (object, edge, dispatch epoch).

    subgoals.jsonl records what went out, in real naming. The dispatch timestamp
    identifies the physical attempt rather than the run of the settlement code,
    which is what makes settling twice safe. It is written once and read back
    from JSON, so comparing it exactly is comparing the same number.
    """
    path = real_push_dir / "subgoals.jsonl"
    if not path.is_file():
        return None
    for line in reversed(path.read_text(encoding="utf-8").splitlines()):
        if not line.strip():
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if rec.get("type") != "push":
            continue
        object_id, edge_idx = rec.get("object_id"), rec.get("edge_idx")
        if object_id is None or edge_idx is None:
            return None
        dispatched = rec.get("dispatched") or {}
        at_epoch = dispatched.get("at_epoch")
        return (
            str(object_id),
            int(edge_idx),
            None if at_epoch is None else float(at_epoch),
        )
    return None


def _scene_object_poses(scene_dir: Path) -> Optional[dict]:
    """{name: (x_cm, y_cm, theta_deg)} from a captured scene's first frame."""
    path = scene_dir / "mid_obs.jsonl"
    if not path.is_file():
        return None
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        objects = rec.get("objects")
        if not objects:
            continue
        return {
            str(name): (
                float(o.get("x_cm", 0.0)),
                float(o.get("y_cm", 0.0)),
                float(o.get("theta_deg", 0.0)),
            )
            for name, o in objects.items()
        }
    return None


def _settle_target_after_push(
    run_dir: Path,
    iteration: int,
    scene_before_dir: Path,
    scene_after_dir: Path,
    real_push_dir: Optional[Path] = None,
) -> dict:
    """Advance the held boundary's bookkeeping for a push that physically ran.

    The in-process planner does this in NAMOPlanner, which the session workflow
    never constructs: it spawns run_namo per replan, so both halves were missing
    on the path that actually drives the robot.

    Two things happen. Exclusions recorded against an object that has since moved
    are dropped, because an edge index names a contact point in that object's own
    frame and stops meaning the same place once it shifts. Keeping them can hide
    the finish push the setup push just made available. And the attempt counters
    advance, so physical_pushes_attempted stops reading zero on real runs.
    """
    from robot_control.planner.region_target import objects_that_moved

    target = _load_active_region_target(run_dir)
    if target is None:
        return {
            "moved_objects": None,
            "forgotten_pushes": None,
            "settled_dispatch_epoch": None,
            "counted_attempt": False,
        }

    moved = sorted(
        objects_that_moved(
            _scene_object_poses(scene_before_dir), _scene_object_poses(scene_after_dir)
        )
    )
    # Pruning runs every time because it is already idempotent: an object that
    # moved has no entries left to drop on the second pass. Counting is not, so
    # it keys on the dispatch that produced this scene.
    dispatched = _dispatched_push(real_push_dir) if real_push_dir else None
    at_epoch = dispatched[2] if dispatched else None
    updated = target.forgetting_moved(moved).with_push_attempted(iteration, at_epoch)
    forgotten = sorted(set(target.failed_pushes) - set(updated.failed_pushes))
    updated.save(run_dir / ACTIVE_TARGET_FILENAME)
    if forgotten:
        print(
            f"[closed_loop] dropped {len(forgotten)} exclusion(s) for {moved} "
            f"(their body frames moved)"
        )
    return {
        "moved_objects": moved,
        "forgotten_pushes": [list(e) for e in forgotten] or None,
        "settled_dispatch_epoch": at_epoch,
        "counted_attempt": updated.physical_pushes_attempted
        != target.physical_pushes_attempted,
    }


def _record_failed_push_on_target(run_dir: Path, real_push_dir: Path) -> Optional[tuple[str, int]]:
    """Exclude a push that failed physically from the held boundary's next plan.

    A replan in the same iteration re-solves the same target with the same
    run-level shuffle seed, so a deterministic strategy proposes the identical
    approach again and fails the identical way. Only the target survives between
    those two processes, so the failure has to land there.

    Returns the pair recorded, or None when there is nothing to record against.
    """
    target = _load_active_region_target(run_dir)
    if target is None:
        return None
    dispatched = _dispatched_push(real_push_dir)
    if dispatched is None:
        return None
    pair = (dispatched[0], dispatched[1])
    updated = target.with_failed_push(*pair)
    if updated is target:
        return pair
    updated.save(run_dir / ACTIVE_TARGET_FILENAME)
    return pair


def _hold_region_target_enabled(run_dir: Path) -> bool:
    """Whether this run keeps working on one boundary across pushes.

    Enabled by run_meta.json's "hold_region_target", or by the target file
    already existing (so a run that has started a subproblem keeps driving it
    even if the flag is dropped from a later invocation).
    """
    meta_path = run_dir / "run_meta.json"
    if meta_path.is_file():
        try:
            if bool(_read_json(meta_path).get("hold_region_target", False)):
                return True
        except Exception:
            pass
    return (run_dir / ACTIVE_TARGET_FILENAME).is_file()


def _run_strategy(run_dir: Path) -> str:
    run_meta = _load_run_meta(run_dir)
    strategy = run_meta.get("strategy")
    if isinstance(strategy, str) and strategy in RUN_STRATEGIES:
        return strategy
    parsed = _parse_run_name(run_dir.name)
    if parsed is not None:
        return parsed[0]
    return "random_rollout"


def _default_run_shuffle_seed(session_dir: Path, run_name: str) -> int:
    raw = f"{session_dir.resolve()}::{run_name}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(raw).digest()[:4], "big")


def _run_shuffle_seed(run_dir: Path) -> int:
    run_meta = _load_run_meta(run_dir)
    value = run_meta.get("shuffle_seed")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
    return _default_run_shuffle_seed(run_dir.parent, run_dir.name)


def _replace_path_prefix(payload: Any, old_prefix: str, new_prefix: str) -> Any:
    if isinstance(payload, str):
        if payload.startswith(old_prefix):
            return new_prefix + payload[len(old_prefix):]
        return payload
    if isinstance(payload, list):
        return [_replace_path_prefix(item, old_prefix, new_prefix) for item in payload]
    if isinstance(payload, dict):
        return {
            key: _replace_path_prefix(value, old_prefix, new_prefix)
            for key, value in payload.items()
        }
    return payload


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


def _real_runtime_settings_from_diag(real_push_dir: Path) -> dict[str, Any]:
    config_path = real_push_dir / "config.json"
    payload = _read_json(config_path, default={}) or {}
    args = payload.get("args") if isinstance(payload, dict) else {}
    config_yaml = None
    camera_service = None
    if isinstance(args, dict):
        raw_cfg = args.get("config")
        raw_camera = args.get("camera_service")
        if isinstance(raw_cfg, str) and raw_cfg.strip():
            config_yaml = Path(raw_cfg)
        if isinstance(raw_camera, str) and raw_camera.strip():
            camera_service = raw_camera
    if config_yaml is None:
        config_yaml = ROBOT_CONTROL_ROOT / "config" / "real.yaml"
    if camera_service is None:
        camera_service = "tcp://localhost:5556"
    return {
        "config_yaml_path": config_yaml,
        "camera_service": camera_service,
    }


def _real_runtime_scene_config(real_yaml_path: Path) -> dict[str, Any]:
    data = yaml.safe_load(real_yaml_path.read_text()) if real_yaml_path.exists() else {}
    if not isinstance(data, dict):
        data = {}
    workspace = data.get("workspace") or {}
    robot = data.get("robot") or {}
    origin = workspace.get("origin_offset") or [0.0, 0.0]
    if not isinstance(origin, list) or len(origin) < 2:
        origin = [0.0, 0.0]
    return {
        "workspace_width_cm": float(workspace.get("width_cm", 49.0)),
        "workspace_height_cm": float(workspace.get("height_cm", 77.5)),
        "workspace_origin_offset_cm": [float(origin[0]), float(origin[1])],
        "robot_width_cm": float(robot.get("width_cm", 7.0)),
        "robot_height_cm": float(robot.get("height_cm", 7.0)),
        "robot_marker_id": int(robot.get("marker_id", 1)),
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


def _ensure_robot_control_namespace_package(*subpackages: str) -> None:
    """Seed lightweight namespace-package stubs without running __init__.py."""
    robot_control_pkg = sys.modules.get("robot_control")
    if robot_control_pkg is None:
        robot_control_pkg = types.ModuleType("robot_control")
        robot_control_pkg.__path__ = [str(SRC / "robot_control")]
        sys.modules["robot_control"] = robot_control_pkg

    for subpkg in subpackages:
        module_name = f"robot_control.{subpkg}"
        if module_name in sys.modules:
            continue
        pkg = types.ModuleType(module_name)
        pkg.__path__ = [str(SRC / "robot_control" / subpkg.replace(".", "/"))]
        sys.modules[module_name] = pkg


def _load_robot_control_core_types():
    _ensure_robot_control_namespace_package("core")
    return importlib.import_module("robot_control.core.types")


def _load_namo_bridge_class():
    _ensure_robot_control_namespace_package(
        "camera",
        "core",
        "planner",
        "utils",
    )
    utils_pkg = sys.modules.get("robot_control.utils")
    if utils_pkg is not None and not hasattr(utils_pkg, "NAMOXMLGenerator"):
        utils_pkg.NAMOXMLGenerator = _load_namo_xml_generator_class()
    module = importlib.import_module("robot_control.planner.namo_bridge")
    return module.NAMOPlanBridge


def _planner_car_1x_config_path(session_dir: Optional[Path] = None) -> Path:
    override = os.environ.get("CLOSED_LOOP_NAMO_CONFIG_PATH")
    if override:
        config_path = Path(override).expanduser().resolve()
        if not config_path.exists():
            raise FileNotFoundError(f"{config_path} not found")
        return config_path

    if session_dir is not None:
        session_meta = _read_json(session_dir / "session_meta.json", default={}) or {}
        if isinstance(session_meta, dict):
            raw = session_meta.get("planner_namo_config_path")
            if isinstance(raw, str) and raw.strip():
                config_path = Path(raw).expanduser()
                if not config_path.is_absolute():
                    config_path = (session_dir / config_path).resolve()
                else:
                    config_path = config_path.resolve()
                if not config_path.exists():
                    fallback = (
                        resolve_namo_cpp_dir(Path(__file__).resolve())
                        / "config"
                        / config_path.name
                    )
                    if fallback.exists():
                        return fallback.resolve()
                if not config_path.exists():
                    raise FileNotFoundError(f"{config_path} not found")
                return config_path

    config_path = (
        resolve_namo_cpp_dir(Path(__file__).resolve())
        / "config"
        / "namo_config_complete_skill15_car_1x.yaml"
    )
    if not config_path.exists():
        raise FileNotFoundError(f"{config_path} not found")
    return config_path


def _planner_robot_dims_from_config(config_path: Path) -> tuple[float, float]:
    cfg = yaml.safe_load(config_path.read_text()) or {}
    planning = (cfg.get("planning") or {}) if isinstance(cfg, dict) else {}
    robot_size = planning.get("robot_size")
    if not isinstance(robot_size, list) or len(robot_size) != 2:
        raise RuntimeError(f"{config_path} missing planning.robot_size")
    half_x_m = float(robot_size[0])
    half_y_m = float(robot_size[1])
    return half_x_m * 200.0, half_y_m * 200.0


def _goal_probe_python_bin() -> Path:
    env_bin = os.environ.get("PYTHON_BIN")
    if env_bin:
        env_path = Path(env_bin)
        if env_path.exists():
            return env_path
    if sys.version_info[:2] == (3, 12):
        return Path(sys.executable)
    common_env = Path.home() / "miniconda3" / "envs" / "namo312" / "bin" / "python"
    if common_env.exists():
        return common_env
    return Path(sys.executable)


def _goal_wavefront_reachable(
    session_dir: Path,
    env_xml_path: Path,
    robot_pose_cm: tuple[float, float, float],
    goal_cm: tuple[float, float],
) -> bool:
    """Check planner-side goal reachability from the current robot pose.

    Uses the same 1x car NAMO config as the sim random-rollout planner, so
    this is the closed-loop wrapper's unified reachability criterion.
    """
    python_bin = _goal_probe_python_bin()
    config_path = _planner_car_1x_config_path(session_dir)
    loader_path = SRC / "robot_control" / "planner" / "namo_binding_loader.py"
    namo_cpp_dir = resolve_namo_cpp_dir(Path(__file__).resolve())
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
    with portable_scene_path(env_xml_path) as loadable_xml:
        proc = subprocess.run(
            [
                str(python_bin),
                "-c",
                probe_code,
                str(Path(__file__).resolve()),
                str(loadable_xml),
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


def _capture_live_observation_from_camera_service(
    camera_service: str,
    reference_objects: dict[str, Any],
    timeout_s: float = 10.0,
    stable_secs: float = 1.0,
):
    _ensure_robot_control_namespace_package("nodes")
    module = importlib.import_module("robot_control.nodes.remote_observer")
    RemoteObserverNode = module.RemoteObserverNode
    ObjectSizeInfo = module.ObjectSizeInfo

    object_sizes: dict[str, Any] = {}
    for name, raw_obj in reference_objects.items():
        if not isinstance(raw_obj, dict):
            continue
        object_sizes[name] = ObjectSizeInfo(
            width=float(raw_obj.get("width_cm", 0.0)),
            depth=float(raw_obj.get("depth_cm", 0.0)),
            height=float(raw_obj.get("height_cm", 0.0)),
            is_static=bool(raw_obj.get("is_static", False)),
        )

    observer = RemoteObserverNode(address=camera_service, object_sizes=object_sizes)
    if not observer.start():
        raise RuntimeError(
            f"failed to connect to camera_service at {camera_service}; "
            "is it running?"
        )
    try:
        deadline = time.time() + timeout_s
        obs = None
        while time.time() < deadline:
            obs = observer.get()
            if obs is not None:
                break
            time.sleep(0.1)
        if obs is None:
            raise RuntimeError(
                f"camera_service at {camera_service} published no observation "
                f"within {timeout_s:.1f}s"
            )
        time.sleep(stable_secs)
        obs = observer.get() or obs
        return obs
    finally:
        observer.stop()


def _build_scene_after_payload_from_live_observation(
    obs: Any,
    reference_scene_objects: dict[str, Any],
    goal_cm: tuple[float, float],
    scene_cfg: dict[str, Any],
) -> dict[str, Any]:
    observed_objects = getattr(obs, "objects", None)
    if not isinstance(observed_objects, dict):
        raise RuntimeError("live observation missing objects payload")

    payload: dict[str, Any] = {
        "captured_at_epoch": time.time(),
        "captured_at_observation_epoch": float(getattr(obs, "timestamp", time.time())),
        "mode": "real",
        "workspace": {
            "width_cm": float(scene_cfg["workspace_width_cm"]),
            "height_cm": float(scene_cfg["workspace_height_cm"]),
            "origin_offset_cm": [
                float(scene_cfg["workspace_origin_offset_cm"][0]),
                float(scene_cfg["workspace_origin_offset_cm"][1]),
            ],
        },
        "robot": {
            "pose_cm": [
                float(getattr(obs, "robot_x")),
                float(getattr(obs, "robot_y")),
                float(getattr(obs, "robot_theta")),
            ],
            "width_cm": float(scene_cfg["robot_width_cm"]),
            "height_cm": float(scene_cfg["robot_height_cm"]),
            "marker_id": int(scene_cfg["robot_marker_id"]),
        },
        "goal_cm": [float(goal_cm[0]), float(goal_cm[1])],
        "objects": {},
        "_recovered_from_live_camera_service": True,
        "_recovered_reason": "missing scene_after artifacts during closed-loop advance",
    }

    missing_movable: list[str] = []
    objects_out: dict[str, Any] = {}
    for name, raw_obj in reference_scene_objects.items():
        if not isinstance(raw_obj, dict):
            continue
        live_obj = observed_objects.get(name)
        if live_obj is None:
            if bool(raw_obj.get("is_static", False)):
                objects_out[name] = {
                    "pose_cm": [
                        float(raw_obj["x_cm"]),
                        float(raw_obj["y_cm"]),
                        float(raw_obj.get("theta_deg", 0.0)),
                    ],
                    "width_cm": float(raw_obj["width_cm"]),
                    "depth_cm": float(raw_obj["depth_cm"]),
                    "height_cm": float(raw_obj["height_cm"]),
                    "is_static": True,
                }
                continue
            missing_movable.append(name)
            continue
        objects_out[name] = {
            "pose_cm": [float(live_obj.x), float(live_obj.y), float(live_obj.theta)],
            "width_cm": float(raw_obj.get("width_cm", getattr(live_obj, "width", 0.0))),
            "depth_cm": float(raw_obj.get("depth_cm", getattr(live_obj, "depth", 0.0))),
            "height_cm": float(raw_obj.get("height_cm", getattr(live_obj, "height", 0.0))),
            "is_static": bool(raw_obj.get("is_static", getattr(live_obj, "is_static", False))),
        }

    if missing_movable:
        raise RuntimeError(
            "live observation missing movable object markers: "
            + ", ".join(sorted(missing_movable))
        )

    payload["objects"] = objects_out
    return payload


def recover_scene_after(
    session_dir: Path,
    run_name: str,
    iteration: int,
    *,
    allow_overwrite: bool = False,
) -> dict[str, Any]:
    session_meta = _load_session_meta(session_dir)
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    real_push_dir = iter_dir / "real_push"
    if not real_push_dir.exists():
        raise FileNotFoundError(f"{real_push_dir} not found")

    settings = _real_runtime_settings_from_diag(real_push_dir)
    scene_cfg = _real_runtime_scene_config(Path(settings["config_yaml_path"]))
    scene_before_state = _scene_before_state(iter_dir / "scene_before")
    goal = session_meta.get("canonical_goal_cm")
    if not isinstance(goal, list) or len(goal) < 2:
        raise RuntimeError(f"{session_dir / 'session_meta.json'} missing canonical_goal_cm")
    goal_cm = (float(goal[0]), float(goal[1]))

    camera_service = str(settings["camera_service"])
    obs = _capture_live_observation_from_camera_service(
        camera_service,
        scene_before_state["scene_objects"],
    )
    payload = _build_scene_after_payload_from_live_observation(
        obs,
        scene_before_state["scene_objects"],
        goal_cm,
        scene_cfg,
    )

    _ensure_robot_control_namespace_package("diagnostics")
    capture_mod = importlib.import_module("robot_control.diagnostics.capture")
    jpg_bytes = capture_mod.request_camera_frame(camera_service, kind="vis", timeout_sec=2.0)
    if jpg_bytes is None:
        raise RuntimeError(
            f"camera_service at {camera_service} did not return a diagnostic frame; "
            "restart camera_service if needed"
        )

    scene_after_json = real_push_dir / "scene_after.json"
    scene_after_jpg = real_push_dir / "scene_after.jpg"
    if (scene_after_json.exists() or scene_after_jpg.exists()) and not allow_overwrite:
        raise RuntimeError(
            f"{real_push_dir} already contains scene_after artifacts; "
            "pass --allow-overwrite to replace them"
        )
    _write_json_atomic(scene_after_json, payload)
    scene_after_jpg.write_bytes(jpg_bytes)
    return {
        "session_dir": str(session_dir),
        "run_name": run_name,
        "iteration": iteration,
        "camera_service": camera_service,
        "scene_after_json": str(scene_after_json),
        "scene_after_jpg": str(scene_after_jpg),
        "recovered_from_live_camera_service": True,
    }


def _observation_from_scene_before(
    scene_before_dir: Path,
    fallback_goal_cm: Optional[tuple[float, float]] = None,
):
    types_mod = _load_robot_control_core_types()
    Observation = types_mod.Observation
    ObjectPose = types_mod.ObjectPose

    state = _scene_before_state(scene_before_dir)
    goal_cm = state["goal_cm"] or fallback_goal_cm
    if goal_cm is None:
        raise RuntimeError(f"{scene_before_dir} missing goal_cm")

    objects: dict[str, Any] = {}
    for name, raw_obj in state["scene_objects"].items():
        if not isinstance(raw_obj, dict):
            continue
        objects[name] = ObjectPose(
            x=float(raw_obj["x_cm"]),
            y=float(raw_obj["y_cm"]),
            theta=float(raw_obj["theta_deg"]),
            width=float(raw_obj["width_cm"]),
            depth=float(raw_obj["depth_cm"]),
            height=float(raw_obj["height_cm"]),
            is_static=bool(raw_obj.get("is_static", False)),
        )

    robot_pose = state["robot_pose"]
    return Observation(
        robot_x=float(robot_pose[0]),
        robot_y=float(robot_pose[1]),
        robot_theta=float(robot_pose[2]),
        objects=objects,
        timestamp=float(state["observation_timestamp"] or 0.0),
        goal_x=float(goal_cm[0]),
        goal_y=float(goal_cm[1]),
    )


def _mapping_from_scene_objects(scene_objects: dict[str, Any]) -> tuple[dict[str, str], dict[str, str]]:
    real_to_sim: dict[str, str] = {}
    sim_to_real: dict[str, str] = {}
    movable_count = 0
    for name in sorted(scene_objects.keys()):
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


def _mapping_from_scene_env_xml(
    scene_objects: dict[str, Any],
    env_xml_path: Path,
) -> tuple[dict[str, str], dict[str, str]]:
    if not env_xml_path.exists():
        raise FileNotFoundError(f"{env_xml_path} not found")

    # Same arithmetic objects_that_moved uses, against the same thresholds.
    # This matcher is where those thresholds came from, and the two drifted
    # apart on both the distance and the angle when the rule was copied.
    from robot_control.planner.region_target import pose_delta

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
        for real_name in sorted(unmatched_real):
            real_spec = real_specs[real_name]
            pos_err, theta_err = pose_delta(
                (sim_spec["x_cm"], sim_spec["y_cm"], sim_spec["theta_deg"]),
                (real_spec["x_cm"], real_spec["y_cm"], real_spec["theta_deg"]),
            )
            size_err = max(
                abs(sim_spec["width_cm"] - real_spec["width_cm"]),
                abs(sim_spec["depth_cm"] - real_spec["depth_cm"]),
            )
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
    for name in sorted(scene_objects.keys()):
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


def _solution_first_push_steps(solution: dict[str, Any]) -> Optional[int]:
    plan = solution.get("plan")
    if not isinstance(plan, list) or not plan:
        return None
    first_push = plan[0]
    if not isinstance(first_push, dict):
        return None
    try:
        return int(first_push["push_steps"])
    except (KeyError, TypeError, ValueError):
        return None


def _closed_loop_solution_rejection_reason(solution: dict[str, Any]) -> Optional[str]:
    if not bool(solution.get("success")):
        return "plan_not_successful"
    first_push_steps = _solution_first_push_steps(solution)
    if first_push_steps is None:
        return "missing_first_push_steps"
    return None


def _instantiate_closed_loop_bridge(session_dir: Optional[Path] = None):
    NAMOPlanBridge = _load_namo_bridge_class()
    config_path = _planner_car_1x_config_path(session_dir)
    robot_width_cm, robot_height_cm = _planner_robot_dims_from_config(config_path)
    return NAMOPlanBridge(
        namo_config_path=str(config_path),
        scale_factor=1.0,
        verbose=False,
        robot_width_cm=robot_width_cm,
        robot_height_cm=robot_height_cm,
        robot_model="car",
    )


def _write_reused_candidate(
    *,
    candidate_dir: Path,
    goal_cm: tuple[float, float],
    verified_subgoals: list[Any],
    verification_time_ms: float,
    sim_pushes_tried: int,
    planner_scene_xml: str,
    object_mapping: dict[str, Any],
    source_solution: Optional[dict[str, Any]],
    origin_kind: str,
    source_iteration: int,
) -> None:
    candidate_dir.mkdir(parents=True, exist_ok=True)
    planner_scene_path = candidate_dir / "planner_scene.xml"
    planner_scene_path.write_text(planner_scene_xml)

    plan_list: list[dict[str, Any]] = []
    for sg in verified_subgoals:
        plan_list.append(
            {
                "object_id": str(getattr(sg, "object_id")),
                "edge_idx": int(getattr(sg, "edge_idx")),
                "push_steps": int(getattr(sg, "push_steps")),
            }
        )

    payload = {
        "success": True,
        "outcome": "success",
        "goal_cm": [float(goal_cm[0]), float(goal_cm[1])],
        "algorithm": str((source_solution or {}).get("algorithm") or "full_namo"),
        "strategy": str((source_solution or {}).get("strategy") or "random_rollout"),
        "plan": plan_list,
        "search_stats": {
            "search_time_ms": float(verification_time_ms),
            "pushes_in_plan": len(plan_list),
            "sim_pushes_tried": int(sim_pushes_tried),
        },
        "object_mapping": object_mapping,
        "planner_scene_xml": planner_scene_path.name,
        "plan_origin": {
            "kind": origin_kind,
            "source": "previous_committed_chain",
            "source_iteration": int(source_iteration),
        },
    }
    _write_yaml_atomic(candidate_dir / "solution.yaml", payload)


def _attempt_reuse_from_previous_plan(
    session_dir: Path,
    run_name: str,
    iteration: int,
) -> Optional[dict[str, Any]]:
    if iteration <= 1:
        return None

    previous_iter_dir = _iter_dir(session_dir / run_name, iteration - 1)
    selected_plan_path = previous_iter_dir / "selected_plan.json"
    if not selected_plan_path.exists():
        return None

    selected_plan = _read_json(selected_plan_path, default=None)
    if not isinstance(selected_plan, dict):
        return None

    full_plan = selected_plan.get("full_plan")
    if not isinstance(full_plan, list) or not full_plan:
        return None

    source_solution: Optional[dict[str, Any]] = None
    source_solution_path = selected_plan.get("solution_path")
    if isinstance(source_solution_path, str) and source_solution_path.strip():
        source_path = Path(source_solution_path)
        if source_path.exists():
            loaded = yaml.safe_load(source_path.read_text()) or {}
            if isinstance(loaded, dict):
                source_solution = loaded

    session_meta = _load_session_meta(session_dir)
    goal = session_meta.get("canonical_goal_cm")
    if not isinstance(goal, list) or len(goal) < 2:
        raise RuntimeError(f"{session_dir / 'session_meta.json'} missing canonical_goal_cm")
    goal_cm = (float(goal[0]), float(goal[1]))

    iter_dir = _iter_dir(session_dir / run_name, iteration)
    observation = _observation_from_scene_before(iter_dir / "scene_before", fallback_goal_cm=goal_cm)
    bridge = _instantiate_closed_loop_bridge(session_dir)

    # While this run is holding a boundary, a reused chain has to still open
    # THAT boundary. Verifying against the final goal instead would accept a
    # chain that abandons the subproblem -- the same reason the in-process
    # ladder passes these two arguments.
    held_target = _load_active_region_target(session_dir / run_name)

    def _verify(chain: list[Any], origin_kind: str) -> Optional[dict[str, Any]]:
        result = bridge.verify_chain(
            observation=observation,
            robot_goal_cm=goal_cm,
            chain=chain,
            target_points=list(held_target.target_samples_m) if held_target else None,
            min_reachable=held_target.minimum_reachable() if held_target else None,
        )
        if not result.success:
            return {
                "success": False,
                "origin_kind": origin_kind,
                "failed_step_index": result.failed_step_index,
                "failure_reason": result.failure_reason,
                "verification_time_ms": result.verification_time_ms,
                "sim_pushes_tried": int(result.sim_pushes_tried),
            }

        candidate_dir = iter_dir / "sim_candidates" / "candidate1"
        if candidate_dir.exists():
            shutil.rmtree(candidate_dir)
        _write_reused_candidate(
            candidate_dir=candidate_dir,
            goal_cm=goal_cm,
            verified_subgoals=result.verified_subgoals,
            verification_time_ms=result.verification_time_ms,
            sim_pushes_tried=int(result.sim_pushes_tried),
            planner_scene_xml=result.planner_scene_xml,
            object_mapping=result.object_mapping,
            source_solution=source_solution,
            origin_kind=origin_kind,
            source_iteration=iteration - 1,
        )
        return {
            "success": True,
            "origin_kind": origin_kind,
            "failed_step_index": result.failed_step_index,
            "failure_reason": result.failure_reason,
            "verification_time_ms": result.verification_time_ms,
            "sim_pushes_tried": int(result.sim_pushes_tried),
            "candidate_dir": str(candidate_dir),
        }

    reuse_attempts: list[dict[str, Any]] = []

    if len(full_plan) > 1:
        suffix_attempt = _verify(full_plan[1:], "reuse_suffix")
        if suffix_attempt is not None:
            reuse_attempts.append(suffix_attempt)
        if suffix_attempt and suffix_attempt.get("success"):
            return suffix_attempt
        if suffix_attempt and suffix_attempt.get("failed_step_index") == 0:
            full_attempt = _verify(full_plan, "reuse_full")
            if full_attempt is not None:
                reuse_attempts.append(full_attempt)
            if full_attempt and full_attempt.get("success"):
                return full_attempt
        return {
            "success": False,
            "reuse_attempts": reuse_attempts,
        }

    full_attempt = _verify(full_plan, "reuse_full")
    if full_attempt is not None:
        reuse_attempts.append(full_attempt)
    if full_attempt and full_attempt.get("success"):
        return full_attempt
    return {
        "success": False,
        "reuse_attempts": reuse_attempts,
    }


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


def _write_run_meta(
    run_dir: Path,
    *,
    session_dir: Path,
    run_name: str,
    strategy: str,
    seed_source_run: Optional[str],
    seed_source_path: Optional[Path],
    seed_solution_success: bool,
) -> None:
    _, now_iso = _utc_now()
    run_meta = {
        "run_name": run_name,
        "strategy": strategy,
        "seed_source_run": seed_source_run,
        "seed_source_path": str(seed_source_path) if seed_source_path is not None else None,
        "iter_001_candidate_dir": "iter_001/sim_candidates/candidate1",
        "seed_solution_success": seed_solution_success,
        "shuffle_seed": _default_run_shuffle_seed(session_dir, run_name),
        "created_at_utc": now_iso,
    }
    _write_json_atomic(run_dir / "run_meta.json", run_meta)


def _materialize_run(
    session_dir: Path,
    start_state_dir: Path,
    *,
    run_name: str,
    strategy: str,
    seed_source: Optional[Path],
    seed_source_run: Optional[str],
) -> None:
    run_dir = session_dir / run_name
    iter1_dir = _iter_dir(run_dir, 1)
    scene_before_dir = iter1_dir / "scene_before"
    candidate_dir = iter1_dir / "sim_candidates" / "candidate1"
    real_push_dir = iter1_dir / "real_push"
    scene_after_dir = iter1_dir / "scene_after"

    run_dir.mkdir(parents=True, exist_ok=True)
    _clear_dir(scene_before_dir)
    _copy_tree_contents(start_state_dir, scene_before_dir)

    if seed_source is not None:
        if candidate_dir.exists():
            shutil.rmtree(candidate_dir)
        candidate_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(seed_source, candidate_dir)
        solution_path = candidate_dir / "solution.yaml"
        solution = yaml.safe_load(solution_path.read_text()) if solution_path.exists() else {}
        seed_solution_success = bool((solution or {}).get("success"))
        state = "seeded"
        successful_candidates = ["candidate1"]
    else:
        if candidate_dir.parent.exists():
            shutil.rmtree(candidate_dir.parent)
        seed_solution_success = False
        state = "awaiting_replan"
        successful_candidates = []

    real_push_dir.mkdir(parents=True, exist_ok=True)
    scene_after_dir.mkdir(parents=True, exist_ok=True)
    _write_run_meta(
        run_dir,
        session_dir=session_dir,
        run_name=run_name,
        strategy=strategy,
        seed_source_run=seed_source_run,
        seed_source_path=seed_source,
        seed_solution_success=seed_solution_success,
    )
    _write_run_status(iter1_dir, 1, state, successful_candidates)


def _materialize_seed_run(
    session_dir: Path,
    start_state_dir: Path,
    run_name: str,
    seed_source: Path,
) -> None:
    _materialize_run(
        session_dir,
        start_state_dir,
        run_name=run_name,
        strategy="random_rollout",
        seed_source=seed_source,
        seed_source_run=run_name,
    )


def _patch_relocated_run_artifacts(old_run_dir: Path, new_run_dir: Path, new_run_name: str) -> None:
    old_prefix = str(old_run_dir)
    new_prefix = str(new_run_dir)
    for selected_plan_path in new_run_dir.glob("iter_*/selected_plan.json"):
        payload = _read_json(selected_plan_path, default=None)
        if isinstance(payload, dict):
            payload = _replace_path_prefix(payload, old_prefix, new_prefix)
            payload["run_name"] = new_run_name
            _write_json_atomic(selected_plan_path, payload)
    for launch_script_path in new_run_dir.glob("iter_*/launch_real_push.sh"):
        text = _read_text(launch_script_path)
        if text:
            launch_script_path.write_text(text.replace(old_prefix, new_prefix))
            launch_script_path.chmod(0o755)


def _desired_run_specs(session_meta: dict[str, Any]) -> list[dict[str, Any]]:
    random_root = _optional_path(session_meta.get("source_random_rollout_root"))
    specs: list[dict[str, Any]] = []
    for strategy in RUN_STRATEGIES:
        for index in range(1, RUNS_PER_STRATEGY + 1):
            seed_source_run: Optional[str] = None
            seed_source_path: Optional[Path] = None
            if strategy == "random_rollout":
                seed_source_run = f"run{index}"
                if random_root is not None:
                    candidate = random_root / seed_source_run
                    if candidate.is_dir() and _is_successful_seed_run(candidate):
                        seed_source_path = candidate.resolve()
            specs.append(
                {
                    "run_name": _strategy_run_name(strategy, index),
                    "strategy": strategy,
                    "seed_source_run": seed_source_run,
                    "seed_source_path": seed_source_path,
                    "index": index,
                }
            )
    return specs


def normalize_strategy_run_layout(session_dir: Path) -> dict[str, Any]:
    session_dir = session_dir.resolve()
    session_meta = _load_session_meta(session_dir)
    start_state_dir = session_dir / str(session_meta.get("start_state_dir") or "start_state")
    if not start_state_dir.is_dir():
        raise RuntimeError(f"{start_state_dir} missing")

    desired_specs = _desired_run_specs(session_meta)
    desired_names = {str(spec["run_name"]) for spec in desired_specs}
    removed_runs: list[str] = []
    renamed_runs: list[dict[str, str]] = []
    created_runs: list[str] = []
    refreshed_runs: list[str] = []

    existing_run_dirs = [p for p in session_dir.iterdir() if _is_run_dir(p)]
    for run_dir in sorted(existing_run_dirs, key=lambda p: _run_sort_key(p.name)):
        iter_dirs = [p for p in run_dir.iterdir() if p.is_dir() and p.name.startswith("iter_")]
        if len(iter_dirs) > 1:
            shutil.rmtree(run_dir)
            removed_runs.append(run_dir.name)

    for spec in desired_specs:
        run_name = str(spec["run_name"])
        strategy = str(spec["strategy"])
        index = int(spec["index"])
        seed_source_run = spec["seed_source_run"]
        seed_source_path = spec["seed_source_path"]
        target_dir = session_dir / run_name
        legacy_dir = session_dir / f"run{index}"

        if target_dir.exists():
            _write_run_meta(
                target_dir,
                session_dir=session_dir,
                run_name=run_name,
                strategy=strategy,
                seed_source_run=seed_source_run,
                seed_source_path=seed_source_path,
                seed_solution_success=bool(seed_source_path),
            )
            refreshed_runs.append(run_name)
            continue

        if strategy == "random_rollout" and legacy_dir.exists():
            legacy_dir.rename(target_dir)
            _patch_relocated_run_artifacts(legacy_dir, target_dir, run_name)
            _write_run_meta(
                target_dir,
                session_dir=session_dir,
                run_name=run_name,
                strategy=strategy,
                seed_source_run=seed_source_run,
                seed_source_path=seed_source_path,
                seed_solution_success=bool(seed_source_path),
            )
            renamed_runs.append({"from": legacy_dir.name, "to": run_name})
            continue

        _materialize_run(
            session_dir,
            start_state_dir,
            run_name=run_name,
            strategy=strategy,
            seed_source=seed_source_path,
            seed_source_run=seed_source_run,
        )
        created_runs.append(run_name)

    for run_dir in sorted([p for p in session_dir.iterdir() if _is_run_dir(p)], key=lambda p: _run_sort_key(p.name)):
        if run_dir.name not in desired_names:
            shutil.rmtree(run_dir)
            removed_runs.append(run_dir.name)

    session_meta["strategy_seed_roots"] = {
        "random_rollout": session_meta.get("source_random_rollout_root"),
        "primitive": None,
    }
    session_meta["run_specs"] = [
        {
            "run_name": spec["run_name"],
            "strategy": spec["strategy"],
            "seed_source_run": spec["seed_source_run"],
            "seed_source_path": str(spec["seed_source_path"]) if spec["seed_source_path"] is not None else None,
        }
        for spec in desired_specs
    ]
    session_meta["initial_seed_runs"] = [str(spec["run_name"]) for spec in desired_specs]
    _write_json_atomic(session_dir / "session_meta.json", session_meta)

    return {
        "session_dir": str(session_dir),
        "removed_runs": removed_runs,
        "renamed_runs": renamed_runs,
        "created_runs": created_runs,
        "refreshed_runs": refreshed_runs,
        "run_names": [str(spec["run_name"]) for spec in desired_specs],
    }


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
        raise RuntimeError(
            f"{session_dir / 'session_meta.json'} missing or malformed. "
            "Pass the session root directory that contains session_meta.json. "
            "If you are using $SESSION, make sure it is set in the current shell; "
            "an empty value resolves to the current working directory."
        )
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


def _clear_prepared_real_push_artifacts(iter_dir: Path) -> None:
    for path in (
        iter_dir / "selected_plan.json",
        iter_dir / "selected_trial_spec.yaml",
        iter_dir / "launch_real_push.sh",
    ):
        if path.exists():
            path.unlink()


def _unprepared_status_patch() -> dict[str, Any]:
    return {
        "state": "awaiting_replan",
        "selected_candidate": None,
        "selected_plan_path": None,
        "selected_trial_spec_path": None,
        "launch_real_push_script": None,
        "selected_sim_push_steps": None,
        "selected_real_push_steps": None,
        "selected_push_steps_overridden_for_real": None,
        "last_replan_exit_code": None,
        "last_replan_driver_log": None,
        "last_replan_rejections": [],
        "last_replan_accepted_attempt": None,
        "last_replan_strategy": None,
        "last_replan_verification_ms": None,
    }


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
    rejection_reason = _closed_loop_solution_rejection_reason(solution)
    if rejection_reason is not None:
        raise RuntimeError(
            f"{solution_path} is not an acceptable closed-loop plan: {rejection_reason}"
        )

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
    sim_push_steps = int(first_push["push_steps"])
    push_steps = max(MIN_EXECUTION_PUSH_STEPS, sim_push_steps)
    push_steps_overridden = push_steps != sim_push_steps
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
            "plan_object_id": str(first_push["object_id"]),
            "sim_object_id": real_to_sim.get(real_object_id, str(first_push["object_id"])),
            "real_object_id": real_object_id,
            "edge_idx": edge_idx,
            "push_steps": push_steps,
            "sim_push_steps": sim_push_steps,
            "push_steps_overridden_for_real": push_steps_overridden,
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
        "  --headless \\",
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
            "selected_sim_push_steps": sim_push_steps,
            "selected_real_push_steps": push_steps,
            "selected_push_steps_overridden_for_real": push_steps_overridden,
        },
    )
    return {
        "session_dir": str(session_dir),
        "run_name": run_name,
        "iteration": iteration,
        "selected_real_object_id": real_object_id,
        "edge_idx": edge_idx,
        "push_steps": push_steps,
        "sim_push_steps": sim_push_steps,
        "push_steps_overridden_for_real": push_steps_overridden,
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
    for name in sorted(objects.keys()):
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
    if not scene_after_json.exists() or not scene_after_jpg.exists():
        recover_scene_after(
            session_dir,
            run_name,
            iteration,
            allow_overwrite=True,
        )
    if not scene_after_json.exists():
        raise FileNotFoundError(
            f"{scene_after_json} not found even after live recovery attempt. "
            "Do not move the scene; make sure camera_service is still running "
            "and the post-push scene is unchanged, then retry."
        )
    if not scene_after_jpg.exists():
        raise FileNotFoundError(
            f"{scene_after_jpg} not found even after live recovery attempt. "
            "Do not move the scene; make sure camera_service is still running "
            "and the post-push scene is unchanged, then retry."
        )

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
        session_dir,
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
    # The push physically ran, so the held boundary's bookkeeping advances
    # whatever the outcome was. Only the in-process planner used to do this, and
    # the session workflow never constructs it.
    settled = _settle_target_after_push(
        run_dir, iteration, iter_dir / "scene_before", scene_after_dir, real_push_dir
    )
    excluded_push = None
    if replan_same_iteration:
        # The replan below reuses this run's shuffle seed and resumes the same
        # target, so without recording the failure the next plan can propose the
        # approach that just proved physically unreachable.
        excluded_push = _record_failed_push_on_target(run_dir, real_push_dir)
        if excluded_push is not None:
            print(
                f"[closed_loop] excluded ({excluded_push[0]}, edge={excluded_push[1]}) "
                f"from the held boundary: approach unreachable"
            )
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
            "excluded_push": list(excluded_push) if excluded_push else None,
            "moved_objects": settled["moved_objects"],
            "forgotten_pushes": settled["forgotten_pushes"],
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
        "excluded_push": list(excluded_push) if excluded_push else None,
        "moved_objects": settled["moved_objects"],
        "forgotten_pushes": settled["forgotten_pushes"],
        "final_distance_to_goal_cm": dist,
        "next_iteration_dir": created_next_iter,
        "scene_after_robot_pose_cm": synthetic_rec["robot_pose_cm"],
    }


def _run_full_replan_search(session_dir: Path, run_name: str, iteration: int) -> dict[str, Any]:
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    scene_before_dir = iter_dir / "scene_before"
    sim_xml_path = scene_before_dir / "env.xml"
    if not sim_xml_path.exists():
        raise FileNotFoundError(f"{sim_xml_path} not found")

    _clear_prepared_real_push_artifacts(iter_dir)
    _update_status(iter_dir, _unprepared_status_patch())
    candidate_dir = iter_dir / "sim_candidates" / "candidate1"
    if candidate_dir.exists():
        shutil.rmtree(candidate_dir)
    sim_candidates_dir = iter_dir / "sim_candidates"
    sim_candidates_dir.mkdir(parents=True, exist_ok=True)
    driver_log = iter_dir / "replan_candidate1_driver.log"
    strategy = _run_strategy(run_dir)

    python_bin = os.environ.get("PYTHON_BIN") or sys.executable
    planner_config_path = _planner_car_1x_config_path(session_dir)
    cmd = [
        python_bin,
        "scripts/run_namo.py",
        "--sim",
        "--sim-xml",
        str(sim_xml_path),
        "--sim-real-run-dir",
        str(scene_before_dir),
        "--namo-config",
        str(planner_config_path),
        "--strategy",
        strategy,
        "--shuffle-seed",
        str(_run_shuffle_seed(run_dir)),
        "--diag-path",
        str(sim_candidates_dir),
        "--run-name",
        "candidate1",
        "--allow-overwrite",
    ]
    # Hand the child the run-level target file. It resumes the boundary the
    # previous process was opening instead of re-deriving one, and writes back
    # the target to hold next time. Opt-in: without the marker file this is a
    # normal whole-problem replan, exactly as before.
    active_target_path = run_dir / ACTIVE_TARGET_FILENAME
    if _hold_region_target_enabled(run_dir):
        cmd.extend(["--active-target", str(active_target_path)])
    if strategy == "random_rollout":
        cmd.extend(
            [
                "--rollout-samples-per-state",
                "36000",
            ]
        )
    child_env = os.environ.copy()
    existing_pythonpath = child_env.get("PYTHONPATH")
    child_env["PYTHONPATH"] = (
        f"{SRC}{os.pathsep}{existing_pythonpath}"
        if existing_pythonpath
        else str(SRC)
    )

    print(f"[closed-loop] replan {run_name} iter_{iteration:03d}")
    rejection_reasons: list[str] = []
    proc: Optional[subprocess.CompletedProcess[str]] = None
    successful = False
    accepted_attempt: Optional[int] = None

    with open(driver_log, "w") as log_file:
        for attempt in range(1, MAX_REPLAN_ATTEMPTS + 1):
            if candidate_dir.exists():
                shutil.rmtree(candidate_dir)
            log_file.write(f"\n=== replan attempt {attempt}/{MAX_REPLAN_ATTEMPTS} ===\n")
            log_file.flush()
            proc = subprocess.run(
                cmd,
                cwd=ROBOT_CONTROL_ROOT,
                env=child_env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )

            if not (candidate_dir.is_dir() and _is_successful_seed_run(candidate_dir)):
                rejection_reasons.append(f"attempt_{attempt}: plan_not_successful")
                continue

            solution_path = candidate_dir / "solution.yaml"
            solution = yaml.safe_load(solution_path.read_text()) or {}
            rejection_reason = _closed_loop_solution_rejection_reason(solution)
            if rejection_reason is None:
                successful = True
                accepted_attempt = attempt
                break

            rejection_reasons.append(f"attempt_{attempt}: {rejection_reason}")
            log_file.write(
                f"[closed-loop] rejecting candidate1 from attempt {attempt}: "
                f"{rejection_reason}\n"
            )
            log_file.flush()
            shutil.rmtree(candidate_dir)

    if proc is None:
        raise RuntimeError("replan_iteration did not launch any subprocess attempt")

    _update_status(
        iter_dir,
        {
            **_unprepared_status_patch(),
            "state": "planned" if successful else "planning_failed",
            "successful_sim_candidates": ["candidate1"] if successful else [],
            "last_replan_exit_code": proc.returncode,
            "last_replan_driver_log": driver_log.name,
            "last_replan_rejections": rejection_reasons,
            "last_replan_accepted_attempt": accepted_attempt,
            "last_replan_strategy": strategy,
            "last_replan_verification_ms": None,
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
        "accepted_attempt": accepted_attempt,
        "rejections": rejection_reasons,
        "strategy": strategy,
    }


def replan_iteration(session_dir: Path, run_name: str, iteration: int) -> dict[str, Any]:
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    _clear_prepared_real_push_artifacts(iter_dir)
    _update_status(iter_dir, _unprepared_status_patch())
    candidate_dir = iter_dir / "sim_candidates" / "candidate1"

    reuse_result = _attempt_reuse_from_previous_plan(session_dir, run_name, iteration)
    if reuse_result and reuse_result.get("success"):
        _update_status(
            iter_dir,
            {
                **_unprepared_status_patch(),
                "state": "planned",
                "successful_sim_candidates": ["candidate1"],
                "last_replan_exit_code": 0,
                "last_replan_driver_log": None,
                "last_replan_rejections": [],
                "last_replan_accepted_attempt": 0,
                "last_replan_strategy": reuse_result["origin_kind"],
                "last_replan_verification_ms": reuse_result["verification_time_ms"],
            },
        )
        return {
            "session_dir": str(session_dir),
            "run_name": run_name,
            "iteration": iteration,
            "candidate_dir": str(candidate_dir),
            "driver_log": None,
            "exit_code": 0,
            "successful_plan": True,
            "accepted_attempt": 0,
            "rejections": [],
            "reused_plan": True,
            "reuse_kind": reuse_result["origin_kind"],
            "verification_time_ms": reuse_result["verification_time_ms"],
        }

    return _run_full_replan_search(session_dir, run_name, iteration)


def _reuse_attempt_rejections(reuse_attempts: list[dict[str, Any]]) -> list[str]:
    reasons: list[str] = []
    for attempt in reuse_attempts:
        if not isinstance(attempt, dict):
            continue
        origin = str(attempt.get("origin_kind") or "reuse")
        if attempt.get("success"):
            reasons.append(f"{origin}: success")
            continue
        failed_step = attempt.get("failed_step_index")
        failure_reason = attempt.get("failure_reason") or "unknown_failure"
        sim_pushes_tried = attempt.get("sim_pushes_tried")
        reasons.append(
            f"{origin}: failed_step={failed_step}, reason={failure_reason}, "
            f"sim_pushes_tried={sim_pushes_tried}"
        )
    return reasons


def replan_reuse_only(session_dir: Path, run_name: str, iteration: int) -> dict[str, Any]:
    run_dir = session_dir / run_name
    iter_dir = _iter_dir(run_dir, iteration)
    scene_before_dir = iter_dir / "scene_before"
    sim_xml_path = scene_before_dir / "env.xml"
    if not sim_xml_path.exists():
        raise FileNotFoundError(f"{sim_xml_path} not found")

    _clear_prepared_real_push_artifacts(iter_dir)
    _update_status(iter_dir, _unprepared_status_patch())
    candidate_dir = iter_dir / "sim_candidates" / "candidate1"
    if candidate_dir.exists():
        shutil.rmtree(candidate_dir)
    (iter_dir / "sim_candidates").mkdir(parents=True, exist_ok=True)

    reuse_result = _attempt_reuse_from_previous_plan(session_dir, run_name, iteration)
    if reuse_result and reuse_result.get("success"):
        _update_status(
            iter_dir,
            {
                **_unprepared_status_patch(),
                "state": "planned",
                "successful_sim_candidates": ["candidate1"],
                "last_replan_exit_code": 0,
                "last_replan_driver_log": None,
                "last_replan_rejections": [],
                "last_replan_accepted_attempt": 0,
                "last_replan_strategy": reuse_result["origin_kind"],
                "last_replan_verification_ms": reuse_result["verification_time_ms"],
            },
        )
        return {
            "session_dir": str(session_dir),
            "run_name": run_name,
            "iteration": iteration,
            "candidate_dir": str(candidate_dir),
            "successful_plan": True,
            "accepted_attempt": 0,
            "reused_plan": True,
            "reuse_kind": reuse_result["origin_kind"],
            "verification_time_ms": reuse_result["verification_time_ms"],
            "sim_pushes_tried": reuse_result.get("sim_pushes_tried"),
        }

    reuse_attempts = []
    if isinstance(reuse_result, dict):
        raw_attempts = reuse_result.get("reuse_attempts")
        if isinstance(raw_attempts, list):
            reuse_attempts = raw_attempts

    _update_status(
        iter_dir,
        {
            **_unprepared_status_patch(),
            "state": "awaiting_remote_replan",
            "successful_sim_candidates": [],
            "last_replan_exit_code": None,
            "last_replan_driver_log": None,
            "last_replan_rejections": _reuse_attempt_rejections(reuse_attempts),
            "last_replan_accepted_attempt": None,
            "last_replan_strategy": "reuse_failed",
            "last_replan_verification_ms": None,
        },
    )
    return {
        "session_dir": str(session_dir),
        "run_name": run_name,
        "iteration": iteration,
        "successful_plan": False,
        "reused_plan": False,
        "needs_remote_search": True,
        "reuse_attempts": reuse_attempts,
    }


def replan_full_search_only(session_dir: Path, run_name: str, iteration: int) -> dict[str, Any]:
    return _run_full_replan_search(session_dir, run_name, iteration)


def session_status(session_dir: Path, run_name: Optional[str]) -> dict[str, Any]:
    meta = _load_session_meta(session_dir)
    runs_root = []
    if run_name is not None:
        runs_root = [session_dir / run_name]
    else:
        runs_root = [p for p in session_dir.iterdir() if _is_run_dir(p)]
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

    recover = sub.add_parser(
        "recover-scene-after",
        help="Recover missing real_push/scene_after.{json,jpg} from the live camera_service.",
    )
    recover.add_argument("--session-dir", type=Path, required=True)
    recover.add_argument("--run", type=str, required=True)
    recover.add_argument("--iteration", type=int, required=True)
    recover.add_argument("--allow-overwrite", action="store_true")

    replan = sub.add_parser("replan", help="Run one sim replan into sim_candidates/candidate1.")
    replan.add_argument("--session-dir", type=Path, required=True)
    replan.add_argument("--run", type=str, required=True)
    replan.add_argument("--iteration", type=int, required=True)

    replan_reuse = sub.add_parser(
        "replan-reuse-only",
        help="Try only reuse ((a2..an), then (a1..an)) and stop before full search.",
    )
    replan_reuse.add_argument("--session-dir", type=Path, required=True)
    replan_reuse.add_argument("--run", type=str, required=True)
    replan_reuse.add_argument("--iteration", type=int, required=True)

    replan_full = sub.add_parser(
        "replan-full-search-only",
        help="Skip reuse and run only the base-strategy full search into sim_candidates/candidate1.",
    )
    replan_full.add_argument("--session-dir", type=Path, required=True)
    replan_full.add_argument("--run", type=str, required=True)
    replan_full.add_argument("--iteration", type=int, required=True)

    normalize = sub.add_parser(
        "normalize-run-layout",
        help="Convert a session to 3 primitive runs + 3 random_rollout runs.",
    )
    normalize.add_argument("--session-dir", type=Path, default=None, help="One session directory to normalize.")
    normalize.add_argument(
        "--all-under-root",
        action="store_true",
        help="Normalize every migrated session under closed_loop_sessions.",
    )
    normalize.add_argument(
        "--closed-loop-root",
        type=Path,
        default=ROBOT_CONTROL_ROOT / "closed_loop_sessions",
        help="closed_loop_sessions root (default: repo-local path).",
    )

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

    if args.command == "recover-scene-after":
        result = recover_scene_after(
            args.session_dir.resolve(),
            args.run,
            args.iteration,
            allow_overwrite=args.allow_overwrite,
        )
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "replan":
        result = replan_iteration(args.session_dir.resolve(), args.run, args.iteration)
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "replan-reuse-only":
        result = replan_reuse_only(args.session_dir.resolve(), args.run, args.iteration)
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "replan-full-search-only":
        result = replan_full_search_only(args.session_dir.resolve(), args.run, args.iteration)
        print(json.dumps(result, indent=2))
        return 0

    if args.command == "normalize-run-layout":
        if args.session_dir is None and not args.all_under_root:
            raise SystemExit("normalize-run-layout requires --session-dir or --all-under-root")
        if args.session_dir is not None and args.all_under_root:
            raise SystemExit("use either --session-dir or --all-under-root, not both")
        targets = [args.session_dir.resolve()] if args.session_dir is not None else _bootstrap_sessions(args.closed_loop_root.resolve())
        results = [normalize_strategy_run_layout(target) for target in targets]
        print(json.dumps({"normalized_sessions": results}, indent=2))
        return 0

    if args.command == "status":
        result = session_status(args.session_dir.resolve(), args.run)
        print(json.dumps(result, indent=2))
        return 0

    raise SystemExit(f"unknown command {args.command!r}")


if __name__ == "__main__":
    raise SystemExit(main())
