"""Execute real-robot push primitives for sim-real calibration.

For each (edge_idx, push_steps) trial listed in a YAML spec, drives the
real robot through the FULL push skill (the one ``run_namo.py`` uses) and
records the obstacle Δ to disk. Between trials, prompts a human to slide
the obstacle back to its start position.

Internally this is a one-line tweak of ``run_namo``: same Runtime, same
controllers, same diagnostics — only the planner is replaced. Instead of
``NAMOPlanner`` (which asks namo_cpp for push sequences), we plug in a
``TrialListPlanner`` that emits the next push from a YAML list, prompting
between trials.

Output:
  <diag-path>/<run-name>/pushes.jsonl   — one record per push (controller's
                                          own Δ measurement; same schema
                                          run_namo writes for every push)
  <diag-path>/<run-name>/subgoals.jsonl — per-trial dispatch + outcome
  <diag-path>/<run-name>/run.log        — tee'd stdout/stderr
  <diag-path>/<run-name>/config.json    — args + git state snapshot
  <diag-path>/<run-name>/mid_obs.jsonl  — every camera observation during
                                          the run, for post-hoc analysis
  <diag-path>/<run-name>/wheel_commands.jsonl
                                        — raw per-tick wheel commands across
                                          the whole run (same push path as
                                          NAMO_PUSH_WHEEL_LOG, plus timestamps)
  <diag-path>/<run-name>/tier2_push_trials/
                                        — one exported bundle per push with
                                          poses.jsonl, commands.jsonl,
                                          trial_meta.json, path.json, metrics

Example:
  python scripts/execute_real_push.py \\
      --config config/real.yaml \\
      --camera-service tcp://localhost:5556 \\
      --diag-path ./recordings \\
      --run-name "{date}_{time}_real_prims_iter1" \\
      --trial-spec config/real_primitive_trials_example.yaml
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml
from pubsub import pub

# Make the script importable both from the repo root and after `pip install -e .`
HERE = Path(__file__).resolve().parent
ROBOT_CONTROL_ROOT = HERE.parent
SRC = ROBOT_CONTROL_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from _diag_setup import bootstrap_diagnostics  # type: ignore  # noqa: E402

from robot_control.core.topics import Topics  # noqa: E402
from robot_control.core.types import ObjectPose, Observation, PushSubgoal, Subgoal  # noqa: E402
from robot_control.diagnostics.chassis_motion import (  # noqa: E402
    CommandSample,
    PoseSample,
    TrialMeta,
    cruise_forward_velocity,
    fit_straight,
    fit_turn,
    pose_dedupe,
    write_command_log,
    write_trial,
)
from robot_control.planner.base import Planner  # noqa: E402
from robot_control.runtime import Runtime, RuntimeConfig  # noqa: E402


# Sentinel for --record-video when passed with no value. Resolved to
# <diag-root>/recordings/ after diagnostics bootstrap, matching run_namo.py.
_RECORD_VIDEO_DEFAULT_SENTINEL = "__USE_DIAG_PATH__"


# ----------------------------------------------------------- MidObsLogger
#
# Writes one JSON record per camera observation to <diag-root>/mid_obs.jsonl
# (JSONL = one JSON object per line). No frequency knob — we log every
# observation the camera_service delivers, so the cadence naturally tracks
# whatever rate the camera is publishing at (typically ~30 Hz, can vary
# under load). Each record carries an epoch timestamp so downstream
# analyzers can:
#
#   - Cross-reference observations with subgoals.jsonl + pushes.jsonl by
#     timestamp (both share at_epoch in seconds since the Unix epoch).
#   - Compute "time since push dispatch" by subtracting any subgoal's
#     dispatched.at_epoch from a mid-obs at_epoch.
#   - Compute inter-observation intervals (= camera publish rate).
#   - Filter to a subgoal's window by [dispatched, completed] from
#     subgoals.jsonl.
#
# The push controller's internal state machine (APPROACHING / ADVANCING /
# PUSHING / RETREATING / ...) is read live from runtime._controllers["push"]
# if available, so each observation is tagged with what phase it occurred
# during. That's hard to reconstruct from timestamps alone.


class MidObsLogger:
    """Subscribes to the runtime's WORLD_STATE pub-sub topic and writes
    every observation to a JSONL file.

    Threading: pub-sub callbacks run on the camera/sensor thread that
    publishes the observation. We hold a lock so the write doesn't
    interleave with another publisher's. Each write also flushes so the
    file is readable mid-run (useful when debugging crashes).
    """

    def __init__(
        self,
        output_path: Path,
        push_controller_ref: Optional[Any] = None,
    ) -> None:
        self._path = Path(output_path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = open(self._path, "w")
        self._lock = threading.Lock()
        # Late-bound: the push controller doesn't exist until Runtime._setup
        # has run, but we want to subscribe early so we don't miss the first
        # frames. attach_push_controller() fills this in afterward.
        self._push_controller = push_controller_ref
        self._count = 0
        self._started_at = time.time()
        pub.subscribe(self._on_world, Topics.WORLD_STATE)
        print(f"[mid-obs] logging every camera observation → {self._path}",
              flush=True)

    def attach_push_controller(self, push_controller: Any) -> None:
        """Called once the runtime has built the push controller, so
        subsequent records get a ``push_state`` field tagging which phase
        they fell into."""
        self._push_controller = push_controller

    def _on_world(self, obs) -> None:
        """Pub-sub callback. Caught + logged on error so a misbehaving
        record never breaks the runtime."""
        try:
            now = time.time()
            rec: Dict[str, Any] = {
                "at_epoch": now,
                "at_utc_iso": datetime.fromtimestamp(now, tz=timezone.utc).isoformat().replace("+00:00", "Z"),
                "t_since_logger_start_s": now - self._started_at,
                "observation_timestamp": (
                    float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
                ),
                "robot_pose_cm": [
                    float(obs.robot_x),
                    float(obs.robot_y),
                    float(obs.robot_theta),
                ],
                "objects": {
                    name: {
                        "x_cm": float(o.x),
                        "y_cm": float(o.y),
                        "theta_deg": float(o.theta),
                        "width_cm": float(o.width),
                        "depth_cm": float(o.depth),
                        "height_cm": float(o.height),
                        "is_static": bool(o.is_static),
                    }
                    for name, o in obs.objects.items()
                },
            }
            if obs.goal_x is not None and obs.goal_y is not None:
                rec["goal_cm"] = [float(obs.goal_x), float(obs.goal_y)]
            if self._push_controller is not None:
                # PushState is an Enum, .value is the human-readable string.
                state = getattr(self._push_controller, "_state", None)
                if state is not None:
                    rec["push_state"] = getattr(state, "value", str(state))
                sub = getattr(self._push_controller, "_current_subgoal", None)
                if sub is not None:
                    rec["active_subgoal"] = {
                        "object_id": getattr(sub, "object_id", None),
                        "edge_idx": getattr(sub, "edge_idx", None),
                        "push_steps": getattr(sub, "push_steps", None),
                    }
            with self._lock:
                self._fp.write(json.dumps(rec) + "\n")
                self._fp.flush()
                self._count += 1
        except Exception as exc:
            # Never let a logger error propagate into the camera thread.
            print(f"[mid-obs] WARN: dropped one observation due to "
                  f"{type(exc).__name__}: {exc}", flush=True)

    def close(self) -> None:
        try:
            pub.unsubscribe(self._on_world, Topics.WORLD_STATE)
        except Exception:
            pass
        with self._lock:
            try:
                self._fp.close()
            except Exception:
                pass
        print(f"[mid-obs] wrote {self._count} observations → {self._path}",
              flush=True)


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    """Best-effort JSONL reader that never raises on a bad line."""
    records: List[Dict[str, Any]] = []
    if not path.exists():
        return records
    with open(path, "r") as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as exc:
                print(f"[tier2] WARN: skipped malformed JSON in {path}:{lineno}: {exc}")
                continue
            if isinstance(rec, dict):
                records.append(rec)
    return records


def _write_json_atomic(path: Path, payload: Any) -> None:
    tmp = path.with_name(".tmp_" + path.name)
    with open(tmp, "w") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, path)


def _float_or_none(value: Any) -> Optional[float]:
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def _int_or_none(value: Any) -> Optional[int]:
    try:
        return None if value is None else int(value)
    except (TypeError, ValueError):
        return None


def _utc_iso_from_epoch(epoch_s: Optional[float]) -> str:
    if epoch_s is None:
        return ""
    return datetime.fromtimestamp(epoch_s, tz=timezone.utc).isoformat().replace("+00:00", "Z")


def _safe_tag(text: Any) -> str:
    raw = str(text)
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in raw)


def _merge_wheel_log_env(existing: Optional[str], new_path: Path) -> str:
    paths: List[str] = []
    if existing:
        paths.extend([p for p in existing.split(os.pathsep) if p])
    new_path_str = str(new_path)
    if new_path_str not in paths:
        paths.append(new_path_str)
    return os.pathsep.join(paths)


def _push_signature_matches(active: Optional[Dict[str, Any]], push_rec: Dict[str, Any]) -> bool:
    if not isinstance(active, dict):
        return False
    object_id = active.get("object_id")
    edge_idx = _int_or_none(active.get("edge_idx"))
    push_steps = _int_or_none(active.get("push_steps"))
    return (
        object_id == push_rec.get("object_id")
        and edge_idx == _int_or_none(push_rec.get("expected_edge"))
        and push_steps == _int_or_none(push_rec.get("expected_push_steps"))
    )


def _path_unit_vec_from_path(path_cm: Any) -> Optional[List[float]]:
    if not isinstance(path_cm, list) or len(path_cm) < 2:
        return None
    try:
        sx, sy = float(path_cm[0][0]), float(path_cm[0][1])
        ex, ey = float(path_cm[-1][0]), float(path_cm[-1][1])
    except (TypeError, ValueError, IndexError):
        return None
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return [dx / length, dy / length]


def _mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / len(values)


def _pstdev(values: List[float]) -> Optional[float]:
    if not values:
        return None
    mu = _mean(values)
    if mu is None:
        return None
    return math.sqrt(sum((v - mu) ** 2 for v in values) / len(values))


def _path_length_from_poses(poses: List[PoseSample]) -> float:
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for prev, cur in zip(poses, poses[1:]):
        total += math.hypot(cur.x_cm - prev.x_cm, cur.y_cm - prev.y_cm)
    return total


def _build_pose_samples(
    push_rec: Dict[str, Any],
    mid_obs_records: List[Dict[str, Any]],
) -> List[PoseSample]:
    start_ts = _float_or_none(push_rec.get("push_start_obs_timestamp"))
    end_ts = _float_or_none(push_rec.get("push_end_obs_timestamp"))
    if start_ts is None or end_ts is None:
        return []

    poses: List[PoseSample] = []
    for rec in mid_obs_records:
        if rec.get("push_state") != "PUSHING":
            continue
        ts = _float_or_none(rec.get("observation_timestamp"))
        robot_pose = rec.get("robot_pose_cm")
        if ts is None or not isinstance(robot_pose, list) or len(robot_pose) < 3:
            continue
        if ts < start_ts or ts > end_ts:
            continue
        if not _push_signature_matches(rec.get("active_subgoal"), push_rec):
            continue
        poses.append(
            PoseSample(
                t_s=ts - start_ts,
                x_cm=float(robot_pose[0]),
                y_cm=float(robot_pose[1]),
                theta_deg=float(robot_pose[2]),
            )
        )
    poses.sort(key=lambda p: p.t_s)
    return pose_dedupe(poses)


def _build_command_samples(
    push_rec: Dict[str, Any],
    wheel_records: List[Dict[str, Any]],
) -> tuple[List[Dict[str, Any]], List[CommandSample]]:
    start_ts = _float_or_none(push_rec.get("push_start_obs_timestamp"))
    end_ts = _float_or_none(push_rec.get("push_end_obs_timestamp"))
    if start_ts is None or end_ts is None:
        return [], []

    raw: List[Dict[str, Any]] = []
    samples: List[CommandSample] = []
    for rec in wheel_records:
        if rec.get("state") != "PUSHING":
            continue
        ts = _float_or_none(rec.get("obs_timestamp"))
        if ts is None or ts < start_ts or ts > end_ts:
            continue
        if not _push_signature_matches(rec, push_rec):
            continue
        raw.append(rec)
        samples.append(
            CommandSample(
                t_s=ts - start_ts,
                left_cmd=float(rec.get("left_cmd", 0.0)),
                right_cmd=float(rec.get("right_cmd", 0.0)),
                mode=str(rec.get("mode", "")),
            )
        )
    samples.sort(key=lambda c: c.t_s)
    raw.sort(key=lambda r: float(r.get("obs_timestamp", 0.0)))
    return raw, samples


def _compute_command_metrics(raw_commands: List[Dict[str, Any]]) -> Dict[str, Any]:
    left = [float(rec.get("left_cmd", 0.0)) for rec in raw_commands]
    right = [float(rec.get("right_cmd", 0.0)) for rec in raw_commands]
    avg = [(l + r) / 2.0 for l, r in zip(left, right)]
    diff = [r - l for l, r in zip(left, right)]
    mag = [max(abs(l), abs(r)) for l, r in zip(left, right)]
    return {
        "n_command_samples": len(raw_commands),
        "command_left_mean": _mean(left),
        "command_left_std": _pstdev(left),
        "command_right_mean": _mean(right),
        "command_right_std": _pstdev(right),
        "command_avg_wheel_mean": _mean(avg),
        "command_avg_wheel_std": _pstdev(avg),
        "command_max_wheel_mean": _mean(mag),
        "command_max_wheel_peak": max(mag) if mag else None,
        "command_wheel_diff_mean": _mean(diff),
        "command_wheel_diff_std": _pstdev(diff),
        "command_obs_reused_count": sum(1 for rec in raw_commands if rec.get("obs_reused")),
        "command_obs_reused_fraction": (
            sum(1 for rec in raw_commands if rec.get("obs_reused")) / len(raw_commands)
            if raw_commands
            else None
        ),
    }


def _compute_push_metrics(
    push_rec: Dict[str, Any],
    subgoal_rec: Optional[Dict[str, Any]],
    poses: List[PoseSample],
    raw_commands: List[Dict[str, Any]],
) -> Dict[str, Any]:
    start_ts = _float_or_none(push_rec.get("push_start_obs_timestamp"))
    end_ts = _float_or_none(push_rec.get("push_end_obs_timestamp"))
    path_cm = push_rec.get("push_path_cm")
    path_unit_vec = push_rec.get("push_path_unit_vec") or _path_unit_vec_from_path(path_cm)
    metrics: Dict[str, Any] = {
        "subgoal_id": push_rec.get("subgoal_id"),
        "object_id": push_rec.get("object_id"),
        "expected_edge": push_rec.get("expected_edge"),
        "expected_push_steps": push_rec.get("expected_push_steps"),
        "stuck": bool(push_rec.get("stuck")),
        "push_duration_s": (
            end_ts - start_ts if start_ts is not None and end_ts is not None else None
        ),
        "n_pose_samples": len(poses),
        "push_path_length_cm": push_rec.get("push_path_length_cm"),
        "push_controller_max_speed": push_rec.get("push_controller_max_speed"),
        "push_lookahead_distance_cm": push_rec.get("push_lookahead_distance_cm"),
        "push_dynamic_direction": push_rec.get("push_dynamic_direction"),
        "push_ticks_executed": push_rec.get("push_ticks_executed"),
    }
    metrics.update(_compute_command_metrics(raw_commands))

    if subgoal_rec is not None:
        dispatch_ts = _float_or_none(subgoal_rec.get("dispatched_obs_timestamp"))
        completed_ts = _float_or_none(subgoal_rec.get("completed_obs_timestamp"))
        metrics["subgoal_duration_s"] = subgoal_rec.get("duration_sec")
        metrics["approach_plus_advance_duration_s"] = (
            start_ts - dispatch_ts if start_ts is not None and dispatch_ts is not None else None
        )
        metrics["retreat_duration_s"] = (
            completed_ts - end_ts if completed_ts is not None and end_ts is not None else None
        )

    if poses:
        metrics["pose_path_length_cm"] = _path_length_from_poses(poses)
        first_pose = poses[0]
        last_pose = poses[-1]
        metrics["robot_net_delta_cm"] = [
            last_pose.x_cm - first_pose.x_cm,
            last_pose.y_cm - first_pose.y_cm,
        ]
        metrics["robot_net_displacement_cm"] = math.hypot(
            last_pose.x_cm - first_pose.x_cm,
            last_pose.y_cm - first_pose.y_cm,
        )
        metrics["robot_heading_delta_deg"] = last_pose.theta_deg - first_pose.theta_deg

        if path_unit_vec is not None:
            start_point = None
            if isinstance(path_cm, list) and path_cm:
                try:
                    start_point = (float(path_cm[0][0]), float(path_cm[0][1]))
                except (TypeError, ValueError, IndexError):
                    start_point = None
            if start_point is None:
                start_point = (first_pose.x_cm, first_pose.y_cm)
            ux, uy = float(path_unit_vec[0]), float(path_unit_vec[1])
            metrics["path_unit_vec"] = [ux, uy]
            metrics["forward_net_progress_cm"] = (
                (last_pose.x_cm - start_point[0]) * ux
                + (last_pose.y_cm - start_point[1]) * uy
            )
            try:
                cruise = cruise_forward_velocity(poses, start_point, (ux, uy))
                metrics["forward_cruise_cm_s"] = cruise.forward_cruise_cm_s
                metrics["forward_cruise_window_s"] = cruise.cruise_window_s
                metrics["forward_cruise_arc_length_cm"] = cruise.arc_length_cm
            except ValueError as exc:
                metrics["forward_cruise_error"] = str(exc)

        if len(poses) >= 5:
            duration = poses[-1].t_s - poses[0].t_s
            settle_s = max(0.0, min(0.5, duration * 0.15))
            try:
                straight = fit_straight(poses, settle_s=settle_s, min_samples=5)
                metrics["linear_fit_speed_cm_s"] = straight.v_linear_cm_s
                metrics["linear_fit_speed_se_cm_s"] = straight.se_v_cm_s
                metrics["linear_fit_vx_cm_s"] = straight.vx_cm_s
                metrics["linear_fit_vy_cm_s"] = straight.vy_cm_s
                metrics["linear_fit_residual_rms_cm"] = straight.fit_residual_rms_cm
            except ValueError as exc:
                metrics["linear_fit_error"] = str(exc)
            try:
                turn = fit_turn(poses, settle_s=settle_s, min_samples=5)
                metrics["omega_fit_deg_s"] = turn.omega_deg_s
                metrics["omega_fit_se_deg_s"] = turn.se_omega_deg_s
                metrics["omega_fit_residual_rms_deg"] = turn.fit_residual_rms_deg
            except ValueError as exc:
                metrics["omega_fit_error"] = str(exc)

    return metrics


def _build_trial_meta(
    push_rec: Dict[str, Any],
    subgoal_rec: Optional[Dict[str, Any]],
    poses: List[PoseSample],
    metrics: Dict[str, Any],
    source: str = "real",
) -> TrialMeta:
    start_ts = _float_or_none(push_rec.get("push_start_obs_timestamp"))
    path_cm = push_rec.get("push_path_cm")
    path_unit_vec = push_rec.get("push_path_unit_vec") or _path_unit_vec_from_path(path_cm)
    duration_s = poses[-1].t_s if poses else float(metrics.get("push_duration_s") or 0.0)
    phase_timestamps = {
        "dispatched_obs_timestamp": (
            _float_or_none(subgoal_rec.get("dispatched_obs_timestamp"))
            if subgoal_rec is not None
            else None
        ),
        "push_start_obs_timestamp": start_ts,
        "push_end_obs_timestamp": _float_or_none(push_rec.get("push_end_obs_timestamp")),
        "completed_obs_timestamp": (
            _float_or_none(subgoal_rec.get("completed_obs_timestamp"))
            if subgoal_rec is not None
            else None
        ),
    }
    phase_durations_s = {
        "approach_plus_advance": metrics.get("approach_plus_advance_duration_s"),
        "pushing": metrics.get("push_duration_s"),
        "retreat": metrics.get("retreat_duration_s"),
        "subgoal_total": metrics.get("subgoal_duration_s"),
    }
    shared_meta = {
        "controller": "PushController",
        "push_state": "PUSHING",
        "subgoal_id": push_rec.get("subgoal_id"),
        "object_id": push_rec.get("object_id"),
        "edge_idx": push_rec.get("expected_edge"),
        "push_steps": push_rec.get("expected_push_steps"),
        "max_speed": push_rec.get("push_controller_max_speed"),
        "lookahead_distance_cm": push_rec.get("push_lookahead_distance_cm"),
        "dynamic_direction": push_rec.get("push_dynamic_direction"),
        "path_start_cm": (path_cm[0] if isinstance(path_cm, list) and path_cm else None),
        "path_end_cm": (path_cm[-1] if isinstance(path_cm, list) and path_cm else None),
        "path_unit_vec": path_unit_vec,
        "start_pose_cm_deg": push_rec.get("robot_pose_before_cm_deg"),
        "end_pose_cm_deg": push_rec.get("robot_pose_after_cm_deg"),
        "stuck": bool(push_rec.get("stuck")),
        "phase_timestamps": phase_timestamps,
        "phase_durations_s": phase_durations_s,
    }
    if source == "real":
        return TrialMeta(
            trial_kind="straight",
            source="real",
            left_cmd=float(push_rec.get("push_controller_max_speed", 0.0) or 0.0),
            right_cmd=float(push_rec.get("push_controller_max_speed", 0.0) or 0.0),
            cmd_units="pwm",
            duration_s=float(duration_s),
            started_at_utc_iso=_utc_iso_from_epoch(start_ts),
            n_samples=len(poses),
            real_meta=shared_meta,
        )

    if source == "sim":
        sim_meta = dict(shared_meta)
        sim_meta["controller"] = "NAMOPushController"
        sim_meta.update(push_rec.get("_sim") or {})
        return TrialMeta(
            trial_kind="straight",
            source="sim",
            left_cmd=float(push_rec.get("push_controller_max_speed", 0.0) or 0.0),
            right_cmd=float(push_rec.get("push_controller_max_speed", 0.0) or 0.0),
            cmd_units="fraction",
            duration_s=float(duration_s),
            started_at_utc_iso=str(
                ((subgoal_rec or {}).get("dispatched") or {}).get("at_utc_iso")
                or push_rec.get("at_utc_iso")
                or ""
            ),
            n_samples=len(poses),
            sim_meta=sim_meta,
        )

    raise ValueError(f"unsupported tier2 export source: {source!r}")


def export_tier2_push_trials(
    diag_root: Path,
    wheel_log_path: Path,
    source: str = "real",
) -> Dict[str, Any]:
    pushes = _load_jsonl(diag_root / "pushes.jsonl")
    subgoals = _load_jsonl(diag_root / "subgoals.jsonl")
    mid_obs = _load_jsonl(diag_root / "mid_obs.jsonl")
    wheel_records = _load_jsonl(wheel_log_path)
    subgoal_by_id = {
        rec.get("subgoal_id"): rec
        for rec in subgoals
        if rec.get("subgoal_id") is not None
    }

    out_root = diag_root / "tier2_push_trials"
    out_root.mkdir(parents=True, exist_ok=True)
    exported: List[Dict[str, Any]] = []

    for idx, push_rec in enumerate(pushes, start=1):
        subgoal_id = push_rec.get("subgoal_id")
        subgoal_rec = subgoal_by_id.get(subgoal_id)
        object_id = _safe_tag(push_rec.get("object_id", "obj"))
        edge_idx = _int_or_none(push_rec.get("expected_edge"))
        push_steps = _int_or_none(push_rec.get("expected_push_steps"))
        tag = (
            f"trial_{idx:03d}_subgoal_{int(subgoal_id or idx):03d}_"
            f"{object_id}_e{edge_idx if edge_idx is not None else 'x'}_"
            f"s{push_steps if push_steps is not None else 'x'}"
        )
        trial_dir = out_root / tag
        trial_dir.mkdir(parents=True, exist_ok=True)

        poses = _build_pose_samples(push_rec, mid_obs)
        raw_commands, commands = _build_command_samples(push_rec, wheel_records)
        metrics = _compute_push_metrics(push_rec, subgoal_rec, poses, raw_commands)
        meta = _build_trial_meta(push_rec, subgoal_rec, poses, metrics, source=source)

        write_trial(trial_dir, meta, poses)
        write_command_log(trial_dir / "commands.jsonl", commands)
        _write_json_atomic(
            trial_dir / "path.json",
            {"path_cm": push_rec.get("push_path_cm") or []},
        )
        _write_json_atomic(trial_dir / "metrics.json", metrics)

        exported.append(
            {
                "trial_dir": str(trial_dir.relative_to(diag_root)),
                "subgoal_id": subgoal_id,
                "object_id": push_rec.get("object_id"),
                "expected_edge": push_rec.get("expected_edge"),
                "expected_push_steps": push_rec.get("expected_push_steps"),
                "n_pose_samples": len(poses),
                "n_command_samples": len(commands),
                "forward_cruise_cm_s": metrics.get("forward_cruise_cm_s"),
                "linear_fit_speed_cm_s": metrics.get("linear_fit_speed_cm_s"),
                "omega_fit_deg_s": metrics.get("omega_fit_deg_s"),
                "stuck": bool(push_rec.get("stuck")),
            }
        )

    summary = {
        "exported_trial_count": len(exported),
        "source": source,
        "source_files": {
            "pushes": "pushes.jsonl",
            "subgoals": "subgoals.jsonl",
            "mid_obs": "mid_obs.jsonl",
            "wheel_commands": str(wheel_log_path.relative_to(diag_root)),
        },
        "trials": exported,
    }
    _write_json_atomic(out_root / "index.json", summary)
    return summary


# ----------------------------------------------------- MarkerLockFilter
#
# Defends against ArUco bit-decode misreads. ArUco DICT_6X6_50 has Hamming-
# close IDs (e.g. marker 5 and marker 6 differ by only a couple of bits). A
# bit flip under glare/motion/oblique-angle conditions makes the decoder
# confidently return the wrong ID — same physical sticker shows up as obj_2
# in obs.objects instead of obj_1. Each obj_X has different dimensions in
# objects.yaml, so the push controller's edge-point math jumps + the GUI
# rectangle changes size. Result: robot wiggles trying to chase a contact
# point that's moving frame-to-frame.
#
# Fix here is downstream of ArUco — we wrap the runtime's observer with a
# filter that, once locked onto a target object name, drops every other
# movable from the observation and synthesizes the locked object from any
# misread that comes through. Controllers + GUI never see the misread.


class MarkerLockFilter:
    """Filters obs.objects to enforce a single locked movable identity.

    Use:
        f = MarkerLockFilter()
        # ...later, once the planner picks one:
        f.set_lock("obj_1", template_pose)
        # then every obs passed through f.apply(obs) only contains obj_1
        # (plus all statics, untouched).
    """

    def __init__(self) -> None:
        self._lock_name: Optional[str] = None
        # Stores the canonical dims for the locked name. Captured from the
        # first successful detection — those dims came from objects.yaml
        # via the observer's per-frame ObjectDefinition lookup, so they're
        # guaranteed correct for that name.
        self._lock_template: Optional[ObjectPose] = None
        self._remap_count = 0  # frames where we synthesized a remap
        self._drop_count = 0   # frames where we dropped a stray movable

    def set_lock(self, name: str, template: ObjectPose) -> None:
        if self._lock_name is not None:
            return  # don't re-lock; ignore subsequent sets
        self._lock_name = name
        # Clone template, zero pose — we only need width/depth/height/static.
        self._lock_template = ObjectPose(
            x=0.0, y=0.0, theta=0.0,
            width=template.width,
            depth=template.depth,
            height=template.height,
            is_static=False,
        )
        print(
            f"[lock] locked to {name!r} "
            f"(width={template.width}, depth={template.depth}). "
            "Stray movables in obs will be dropped / remapped."
        )

    def stats(self) -> Dict[str, int]:
        return {"remaps": self._remap_count, "drops": self._drop_count}

    def apply(self, obs: Optional[Observation]) -> Optional[Observation]:
        if obs is None or self._lock_name is None or self._lock_template is None:
            return obs

        kept_objects: Dict[str, ObjectPose] = {}
        # Pick out statics + the locked movable (if present), tally strays.
        first_stray: Optional[ObjectPose] = None
        for name, o in obs.objects.items():
            if getattr(o, "is_static", False):
                kept_objects[name] = o
                continue
            if name == self._lock_name:
                # ALWAYS override dims with the locked template, even when
                # the camera-detected name matches. Defends against another
                # publisher (e.g. a second RemoteObserverNode without
                # object_sizes populated) putting an obs on pubsub with
                # width=0 / depth=0. Without this override, the GUI would
                # fall back to default_size and flicker.
                kept_objects[name] = ObjectPose(
                    x=o.x,
                    y=o.y,
                    theta=o.theta,
                    width=self._lock_template.width,
                    depth=self._lock_template.depth,
                    height=self._lock_template.height,
                    is_static=False,
                )
            else:
                # Stray movable — drop it. Remember the first one in case we
                # need to remap (when the locked name is missing).
                self._drop_count += 1
                if first_stray is None:
                    first_stray = o

        # If the locked name is missing this frame but a stray was seen,
        # take the stray's position and pretend it's the locked object.
        if self._lock_name not in kept_objects and first_stray is not None:
            self._remap_count += 1
            kept_objects[self._lock_name] = ObjectPose(
                x=first_stray.x,
                y=first_stray.y,
                theta=first_stray.theta,
                width=self._lock_template.width,
                depth=self._lock_template.depth,
                height=self._lock_template.height,
                is_static=False,
            )

        return Observation(
            robot_x=obs.robot_x,
            robot_y=obs.robot_y,
            robot_theta=obs.robot_theta,
            objects=kept_objects,
            timestamp=obs.timestamp,
        )


# ----------------------------------------------------------- TrialListPlanner


class TrialListPlanner(Planner):
    """Emit a fixed YAML-driven sequence of PushSubgoals, with a human-in-the-
    loop obstacle reset prompt between trials.

    Plugged into Runtime in place of NAMOPlanner. Runtime calls ``plan(obs)``
    every time the currently-executing controller reports ``is_done()``. So
    each call corresponds to the boundary between trials:

      - First call: print "trial 1/N", verify obstacle is at target, return
        the first PushSubgoal.
      - Subsequent calls: the previous trial just finished. Verify obstacle
        is at target (prompt user to reset if not), return the next subgoal.
      - List exhausted: ``is_complete`` returns True, Runtime shuts down.
    """

    def __init__(
        self,
        trials: List[Dict[str, Any]],
        obstacle_target_xy: tuple,
        reset_tolerance_cm: float,
        lock_filter: Optional["MarkerLockFilter"] = None,
        skip_reset: bool = False,
    ) -> None:
        self._trials = trials
        self._idx = 0
        self._obstacle_target = obstacle_target_xy
        self._reset_tolerance = reset_tolerance_cm
        # When True, ``_wait_for_obstacle_reset`` does not block on a
        # tolerance check or prompt the operator. The push fires from
        # wherever the object currently is. Δpose is still meaningful
        # because ``pushes.jsonl`` records ``object_pose_before`` and
        # ``object_pose_after`` (both include θ).
        self._skip_reset = skip_reset
        # Cache the auto-detected object id so we don't re-prompt the camera
        # every trial when the spec doesn't pin one explicitly.
        self._auto_object_id: Optional[str] = None
        # Once auto-detection picks an obj_X, we tell this filter to
        # exclude every other movable identity from downstream
        # observations. Defends against ArUco misreads of the same
        # physical sticker.
        self._lock_filter = lock_filter
        # Fresh-obs callable injected by main() once the runtime's
        # _world has been constructed (after _setup runs). Returns the
        # lock-filtered Observation. We use this instead of running our
        # own RemoteObserverNode — a second ZMQ subscriber publishing to
        # the same pubsub topic caused a race in WorldState that made
        # the GUI rectangle flicker between correct dims and default.
        self._fresh_obs_source = None  # set by attach_obs_source
        # Set by the main-thread SIGINT handler so Ctrl+C breaks us out of
        # the blocking stdin wait inside _read_input_with_live_display. The
        # planner runs on the Runtime control thread, and Python only
        # delivers signals to the main thread, so the planner can't see
        # SIGINT directly — it has to poll this flag between select() ticks.
        self._shutdown_requested = threading.Event()

    def attach_obs_source(self, fresh_get) -> None:
        """Wire in the runtime's WorldState.get() callable. main() does this
        after Runtime._setup() has constructed _world.

        fresh_get returns the lock-filtered Observation — the right thing
        for both live display (we want the canonical obstacle) and the
        reset-tolerance check.
        """
        self._fresh_obs_source = fresh_get

    def close(self) -> None:
        """Nothing to clean up — planner no longer owns any threads or
        sockets. Kept as a no-op so main()'s finally block stays uniform."""
        return

    # -- Planner interface --

    def plan(self, obs: Observation) -> Optional[Subgoal]:
        if self._idx >= len(self._trials):
            return None  # is_complete will report True

        live_obs = self._fresh_obs(obs)
        trial = self._trials[self._idx]
        trial_num = self._idx + 1
        total = len(self._trials)

        # Object resolution: prefer YAML pin; else auto-detect single movable.
        if trial.get("object_id"):
            object_id = str(trial["object_id"])
            # If a pin is given, lock to it the first time we see it in obs
            # (so we capture its canonical dims).
            if (
                self._lock_filter is not None
                and self._auto_object_id is None
                and object_id in live_obs.objects
            ):
                self._auto_object_id = object_id
                self._lock_filter.set_lock(object_id, live_obs.objects[object_id])
        else:
            object_id = self._resolve_object_id(live_obs)
            if object_id is None:
                print(
                    f"[trial {trial_num}/{total}] no usable obstacle in view; "
                    "skipping."
                )
                self._idx += 1
                return self.plan(live_obs)

        edge_idx = int(trial["edge_idx"])
        if "push_steps" in trial:
            push_steps = int(trial["push_steps"])
        elif "depth" in trial:
            push_steps = int(trial["depth"]) + 1
        else:
            print(f"[trial {trial_num}/{total}] missing depth/push_steps; skipping.")
            self._idx += 1
            return self.plan(obs)

        print()
        print(
            f"=== Trial {trial_num}/{total} === "
            f"object={object_id} edge_idx={edge_idx} push_steps={push_steps}"
        )

        # Wait for the obstacle to be back at target. The robot is allowed
        # to be anywhere — the push skill's APPROACHING phase will
        # navigate to the right contact point from wherever the robot
        # ended up. Only the obstacle pose matters for the measurement.
        ready_obs = self._wait_for_obstacle_reset(live_obs, object_id)
        self._sync_observation(obs, ready_obs)

        self._idx += 1
        return PushSubgoal(object_id=object_id, edge_idx=edge_idx, push_steps=push_steps)

    def is_complete(self, obs: Observation) -> bool:
        # Tell Runtime to shut down once we've dispatched everything.
        return self._idx >= len(self._trials)

    def reset(self) -> None:
        # Don't reset trial progress; otherwise re-running would replay
        # trials we already finished. If a true reset is wanted, build a
        # fresh planner.
        return

    # -- internal --

    def _resolve_object_id(self, obs: Observation) -> Optional[str]:
        if self._auto_object_id is not None:
            return self._auto_object_id
        movables = [
            name for name, o in obs.objects.items()
            if not getattr(o, "is_static", False)
        ]
        if len(movables) == 1:
            self._auto_object_id = movables[0]
            print(f"[planner] auto-detected obstacle: {self._auto_object_id!r}")
            # Lock the filter to this name (captures the YAML-defined dims
            # from the current detection — those dims came through the
            # observer's lookup, so they're canonical for this name).
            if self._lock_filter is not None:
                template = obs.objects[self._auto_object_id]
                self._lock_filter.set_lock(self._auto_object_id, template)
            return self._auto_object_id
        if len(movables) == 0:
            print(
                "[planner] no movable object visible. "
                f"Visible: {list(obs.objects.keys())}"
            )
            return None
        print(
            f"[planner] multiple movable objects visible: {movables}. "
            "Pin 'object_id' explicitly in the trial spec."
        )
        return None

    def request_shutdown(self) -> None:
        """Called from the main thread (e.g. SIGINT handler) to unblock the
        planner from any stdin wait so the Runtime can shut down cleanly."""
        self._shutdown_requested.set()

    def _wait_for_obstacle_reset(
        self, obs: Observation, object_id: str
    ) -> Observation:
        """Block until the obstacle is within tolerance of the target.

        Reprints obstacle position at 1Hz while waiting. Robot can be
        anywhere — the push skill's APPROACHING phase handles navigating
        to the contact point from wherever the robot is parked.

        When ``skip_reset`` is True, this just fetches one fresh obs,
        prints the recorded start pose for the log, and returns.
        """
        if self._skip_reset:
            latest = self._fresh_obs(obs)
            obj = latest.objects.get(object_id)
            if obj is not None:
                print(
                    f"  [skip_reset] start pose: ({obj.x:.2f}, {obj.y:.2f}) "
                    f"θ={obj.theta:.2f}°. pushing..."
                )
            return latest

        target_x, target_y = self._obstacle_target
        latest = obs
        while True:
            if self._shutdown_requested.is_set():
                raise SystemExit("[planner] shutdown requested (Ctrl+C)")
            latest = self._fresh_obs(latest)
            obj = latest.objects.get(object_id)
            if obj is None:
                print(
                    f"  Object {object_id!r} not in view "
                    f"(visible: {list(latest.objects.keys())})."
                )
            else:
                d = math.hypot(obj.x - target_x, obj.y - target_y)
                if d <= self._reset_tolerance:
                    print(
                        f"  obstacle at ({obj.x:.1f},{obj.y:.1f}) cm — OK "
                        f"({d:.1f} cm from target).  "
                        f"robot at ({latest.robot_x:.1f},{latest.robot_y:.1f}) "
                        f"θ={latest.robot_theta:.1f}°. pushing..."
                    )
                    return latest
                print(
                    f"  obstacle at ({obj.x:.1f},{obj.y:.1f}) cm — "
                    f"{d:.1f} cm from target {self._obstacle_target}.  "
                    f"robot at ({latest.robot_x:.1f},{latest.robot_y:.1f}) "
                    f"θ={latest.robot_theta:.1f}°."
                )
            print("  reset obstacle (and move robot if you want); press ENTER when done (or 'q' to quit)")
            try:
                ans = self._read_input_with_live_pose(object_id, target_x, target_y)
            except EOFError:
                ans = "q"
            if ans == "q":
                raise SystemExit("[planner] user quit")

    def _read_input_with_live_pose(
        self, object_id: str, target_x: float, target_y: float
    ) -> str:
        """Block on stdin while reprinting the obstacle's live pose at 1Hz."""
        if self._fresh_obs_source is None:
            return input("  > ").strip().lower()
        import select
        while True:
            if self._shutdown_requested.is_set():
                return "q"
            r, _, _ = select.select([sys.stdin], [], [], 1.0)
            if r:
                return sys.stdin.readline().strip().lower()
            o = self._fresh_obs_source()
            if o is None:
                continue
            obj = o.objects.get(object_id)
            if obj is None:
                continue
            d = math.hypot(obj.x - target_x, obj.y - target_y)
            print(
                f"    [live] obj ({obj.x:.1f},{obj.y:.1f}) cm "
                f"({d:.1f} cm from target)   "
                f"robot ({o.robot_x:.1f},{o.robot_y:.1f}) θ={o.robot_theta:.1f}°",
                flush=True,
            )

    def _fresh_obs(self, fallback: Observation) -> Observation:
        """Return the runtime's freshest lock-filtered obs, falling back to
        the supplied obs if the source isn't wired yet."""
        if self._fresh_obs_source is None:
            return fallback
        o = self._fresh_obs_source()
        return o if o is not None else fallback

    @staticmethod
    def _sync_observation(dst: Observation, src: Observation) -> None:
        """Mutate ``dst`` so the caller keeps using the freshest scene."""
        dst.robot_x = src.robot_x
        dst.robot_y = src.robot_y
        dst.robot_theta = src.robot_theta
        dst.objects = src.objects
        dst.timestamp = src.timestamp
        dst.goal_x = src.goal_x
        dst.goal_y = src.goal_y


# --------------------------------------------------------------------- main


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Three required flags. Everything else is optional with sane defaults
    # or lives in the trial-spec YAML.
    p.add_argument("--config", required=True, help="Path to real.yaml")
    p.add_argument(
        "--camera-service",
        required=True,
        metavar="ADDRESS",
        help="ZMQ address of the camera service (e.g. tcp://localhost:5556).",
    )
    p.add_argument("--trial-spec", required=True, help="YAML file listing trials.")
    p.add_argument(
        "--diag-path",
        default="./recordings",
        help="Root diagnostics directory (default: ./recordings).",
    )
    p.add_argument(
        "--run-name",
        default="real_prims_{date}_{time}",
        help="Subdirectory name under --diag-path. Supports placeholders "
             "{date} {time} {timestamp} etc. (default: real_prims_{date}_{time}).",
    )
    p.add_argument(
        "--record-video",
        type=str,
        nargs="?",
        default=None,
        const=_RECORD_VIDEO_DEFAULT_SENTINEL,
        metavar="DIR",
        help="Ask the --camera-service to record video for the duration of "
             "this execute_real_push session. Pass with no value to default to "
             "<diag-path>/<run-name>/recordings/; pass an explicit DIR to "
             "override. Mirrors run_namo.py.",
    )
    p.add_argument(
        "--capture-scene",
        action="store_true",
        help="Save scene snapshots (jpg + json + xml) at run start and end. "
             "Mirrors run_namo.py.",
    )
    p.add_argument(
        "--push-speed",
        type=float,
        default=None,
        help="Override push.max_speed from controller.yaml (PWM scale [0,1]). "
             "Calibration knob — handy for sweeping speeds across runs "
             "without editing YAML.",
    )
    p.add_argument(
        "--nav-speed",
        type=float,
        default=None,
        help="Override navigation.max_speed from controller.yaml.",
    )
    p.add_argument(
        "--allow-overwrite",
        action="store_true",
        help="Reuse an existing --run-name directory instead of erroring out. "
             "Handy when re-running the same calibration iteration.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't send wheel commands. Useful for testing the prompt loop "
             "without a robot.",
    )
    p.add_argument(
        "--no-reset-check",
        action="store_true",
        help="Skip the obstacle-target tolerance check. Push fires from "
             "wherever the object currently is. pushes.jsonl records "
             "object_pose_before (with θ) so per-trial Δpose is still "
             "meaningful — operator no longer needs to slide the object "
             "back to a fixed target between trials.",
    )
    p.add_argument(
        "--log-mid-pos",
        action="store_true",
        help="Deprecated no-op. mid_obs.jsonl is now always written when "
             "diagnostics are enabled so every robot/object pose sample is "
             "available later for calibration and replay.",
    )
    p.add_argument(
        "--marker-lock",
        action="store_true",
        help="Enable the calibration-only observation lock filter that "
             "stabilizes object identity under ArUco misreads. Leave unset "
             "to match run_namo.py's live observation behavior more closely.",
    )
    p.add_argument(
        "--headless",
        action="store_true",
        help="Run without the Runtime GUI window. Recommended for closed-loop "
             "session execution so shutdown/capture does not depend on the Qt "
             "event loop.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    # Same diagnostics bootstrap run_namo uses — gives us config.json + the
    # stdout/stderr Tee to run.log.
    recorder, log_file = bootstrap_diagnostics(args)
    if recorder is None or not recorder.enabled:
        print("Error: --diag-path is required.", file=sys.stderr)
        return 2
    args._diagnostics_recorder = recorder  # Runtime picks this up

    # Resolve run_namo-style runtime-side diagnostics flags against the
    # diag root. We keep the semantics identical so execute_real_push can
    # be launched with the same wrappers as run_namo.py.
    diag_root = recorder.root
    wheel_log_path = Path(diag_root) / "wheel_commands.jsonl"
    if wheel_log_path.exists():
        wheel_log_path.unlink()
    prev_wheel_log_env = os.environ.get("NAMO_PUSH_WHEEL_LOG")
    os.environ["NAMO_PUSH_WHEEL_LOG"] = _merge_wheel_log_env(prev_wheel_log_env, wheel_log_path)
    print(f"[tier2] wheel commands will be logged to {wheel_log_path}")
    if getattr(args, "record_video", None) == _RECORD_VIDEO_DEFAULT_SENTINEL:
        if diag_root is None:
            print("Error: --record-video with no DIR requires --diag-path "
                  "to be set (or pass an explicit DIR).")
            return 1
        args.record_video = str(diag_root / "recordings")
    # Load trials.
    spec = yaml.safe_load(Path(args.trial_spec).read_text()) or {}
    trials = spec.get("trials") or []
    if not trials:
        print(f"Error: no 'trials' in {args.trial_spec}", file=sys.stderr)
        return 2
    obstacle_target = spec.get("obstacle_target") or [25.0, 35.0]
    reset_tolerance = float(spec.get("reset_tolerance_cm") or 3.0)
    print(f"[collect] {len(trials)} trial(s) loaded from {args.trial_spec}")
    print(
        f"[collect] obstacle_target = {tuple(obstacle_target)} cm  "
        f"reset_tolerance = {reset_tolerance} cm"
    )

    # The marker-misread filter. Wraps the runtime's observer once we know
    # which obj_X the planner locked onto. This is opt-in so the default
    # observation path stays closer to run_namo.py.
    lock_filter = MarkerLockFilter() if args.marker_lock else None

    # The planner.
    planner = TrialListPlanner(
        trials=trials,
        obstacle_target_xy=(float(obstacle_target[0]), float(obstacle_target[1])),
        reset_tolerance_cm=reset_tolerance,
        lock_filter=lock_filter,
        skip_reset=args.no_reset_check,
    )

    # Build a Runtime exactly the way run_namo does for real mode. The only
    # difference between this script and `run_namo --config config/real.yaml`
    # is which Planner gets plugged in.
    runtime_config = RuntimeConfig(
        mode="real",
        config_path=args.config,
        planner=planner,
        dry_run=args.dry_run,
        quit_on_complete=True,
        show_gui=not args.headless,
        show_camera=not args.headless,
        camera_service_address=args.camera_service,
        record_video_dir=getattr(args, "record_video", None),
        nav_speed_override=args.nav_speed,
        push_speed_override=args.push_speed,
    )
    runtime_config.diagnostics_recorder = recorder
    runtime_config.capture_scene = bool(getattr(args, "capture_scene", False))

    runtime = Runtime(runtime_config)

    # Mid-observation logger. Constructed BEFORE runtime.run() so it
    # captures every observation including the early frames before the
    # first subgoal dispatch. Push controller gets attached inside the
    # patched _setup() once Runtime has built the controllers.
    mid_obs_logger: Optional[MidObsLogger] = MidObsLogger(
        output_path=Path(recorder.root) / "mid_obs.jsonl",
    )

    # IMPORTANT: runtime._world is None right after construction — it gets
    # created inside runtime._setup(), which runtime.run() calls RIGHT
    # BEFORE the control loop starts. So we can't wire planner helpers
    # here directly; we have to defer them until after _setup() has run.
    #
    # We do that by monkey-patching _setup() itself: when the runtime
    # calls _setup() from run(), our wrapper runs first (calls the real
    # _setup), then installs the hooks on the freshly-created _world
    # before the control thread starts.

    orig_setup = runtime._setup

    def _patched_setup():
        orig_setup()  # builds _world, controllers, etc.

        world = runtime._world

        # 1) Lock filter on WorldState.get(). WorldState is what the
        # control loop polls each tick. Wrapping .get() means every
        # consumer (push controller, GUI, nav) sees the filtered obs.
        if world is not None:
            if lock_filter is not None:
                original_world_get = world.get

                def _filtered_world_get():
                    return lock_filter.apply(original_world_get())

                world.get = _filtered_world_get
                print("[lock] installed MarkerLockFilter on runtime._world.get()")

            # Give the planner the runtime's WorldState as its fresh-obs
            # source. Replaces the (removed) second RemoteObserverNode the
            # planner used to spin up — see TrialListPlanner.__init__ doc.
            planner.attach_obs_source(world.get)
            print("[fresh] planner.attach_obs_source(world.get) wired")
        else:
            print("[lock] WARN: runtime._world is still None after _setup().")

        # 2) Hand the mid-obs logger a reference to the push controller so
        # subsequent records get a push_state field. Done after _setup so
        # _controllers exists.
        if mid_obs_logger is not None:
            push_ctrl = (runtime._controllers or {}).get("push")
            if push_ctrl is not None:
                mid_obs_logger.attach_push_controller(push_ctrl)

    runtime._setup = _patched_setup


    # SIGINT handler: Runtime runs the planner on a background control
    # thread, so the planner can't catch SIGINT itself. The handler flips
    # the planner's shutdown flag (unblocks select()) AND stops Runtime so
    # its main-thread busy loop exits. Without this, Ctrl+C either does
    # nothing or wedges _shutdown() trying to join the stdin-blocked
    # control thread.
    def _handle_sigint(signum, frame):
        print("\n[collect] SIGINT received — shutting down...", flush=True)
        planner.request_shutdown()
        try:
            runtime.stop()
        except Exception:
            pass
    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        runtime.run()
    except KeyboardInterrupt:
        pass  # already handled by SIGINT handler
    finally:
        # Marker-misread filter stats, if it caught anything.
        stats = lock_filter.stats() if lock_filter is not None else {"remaps": 0, "drops": 0}
        if stats["remaps"] or stats["drops"]:
            print()
            print("=" * 60)
            print(
                f"[lock] filter caught {stats['remaps']} misread frame(s) "
                f"(remapped to locked obj) and {stats['drops']} stray "
                f"movable frame(s) (dropped). Push behavior was protected "
                "from these."
            )
            print("=" * 60)
        planner.close()
        if mid_obs_logger is not None:
            mid_obs_logger.close()
        if log_file:
            try:
                log_file.close()
            except Exception:
                pass
        if prev_wheel_log_env is None:
            os.environ.pop("NAMO_PUSH_WHEEL_LOG", None)
        else:
            os.environ["NAMO_PUSH_WHEEL_LOG"] = prev_wheel_log_env

    tier2_summary: Optional[Dict[str, Any]] = None
    try:
        tier2_summary = export_tier2_push_trials(Path(recorder.root), wheel_log_path)
        print(
            f"[tier2] exported {tier2_summary['exported_trial_count']} push trial bundle(s) "
            f"under {Path(recorder.root) / 'tier2_push_trials'}"
        )
    except Exception as exc:
        print(f"[tier2] WARN: failed to export Tier 2 bundles: {type(exc).__name__}: {exc}")

    diag_root = Path(recorder.root)
    print()
    print("=" * 60)
    print(f"Done. Records under {diag_root}")
    print(f"  - {diag_root / 'pushes.jsonl'}      (per-push controller summary: object_id, edge, push_steps, Δxy, Δθ, stuck)")
    print(f"  - {diag_root / 'subgoals.jsonl'}    (per-trial dispatch + outcome)")
    print(f"  - {diag_root / 'run.log'}           (tee'd stdout/stderr)")
    print(f"  - {diag_root / 'mid_obs.jsonl'}     (every camera observation, tagged with push_state)")
    print(f"  - {diag_root / 'wheel_commands.jsonl'} (raw per-tick wheel commands + timestamps)")
    print(f"  - {diag_root / 'tier2_push_trials'} (per-push Tier 2 export bundles)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
