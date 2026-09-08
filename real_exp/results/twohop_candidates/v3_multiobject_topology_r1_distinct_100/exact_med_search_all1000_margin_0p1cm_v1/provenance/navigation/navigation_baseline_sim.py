"""Deterministic MuJoCo execution of the real pure-navigation baseline."""

from __future__ import annotations

import hashlib
import json
import math
import os
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, Optional, TextIO

from robot_control.controller.config import ControllerConfigs, load_controller_configs
from robot_control.controller.factory import create_navigation_controller
from robot_control.core.types import Action, Observation, WorkspaceConfig
from robot_control.executor import SubgoalExecutor
from robot_control.planner.navigation_baseline_planner import (
    BASELINE_MODES,
    DEFAULT_TIMEOUT_S,
    NavigationBaselinePlanner,
)
from robot_control.utils.wavefront_inflation_config import (
    DEFAULT_WAVEFRONT_INFLATION_YAML,
    get_wavefront_inflation_config,
)


REAL_CONTROL_HZ = 30.0
DEFAULT_NAV_SPEED = 0.4
DEFAULT_MAX_SIM_TIME_S = 120.0


class NavigationBaselineSimulation:
    """Run Runtime's planner/executor/controller lifecycle on MuJoCo ticks."""

    def __init__(
        self,
        *,
        environment: Any,
        workspace_config: WorkspaceConfig,
        mode: str,
        nav_speed: float = DEFAULT_NAV_SPEED,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        controller_configs: Optional[ControllerConfigs] = None,
        controller_config_path: Optional[Path | str] = None,
        max_ticks: Optional[int] = None,
        render: bool = False,
    ) -> None:
        if mode not in BASELINE_MODES:
            raise ValueError(f"unknown navigation baseline mode {mode!r}")
        if not math.isfinite(nav_speed) or not 0.0 <= nav_speed <= 1.0:
            raise ValueError("nav_speed must be finite and in [0, 1]")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        if not math.isclose(
            float(environment.control_hz), REAL_CONTROL_HZ, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                f"simulation must run at the real control rate of {REAL_CONTROL_HZ:g} Hz"
            )

        config_path = (
            Path(controller_config_path).expanduser().resolve()
            if controller_config_path is not None
            else None
        )
        if controller_configs is not None:
            self.controller_configs = controller_configs
        else:
            self.controller_configs = load_controller_configs(config_path)

        self.environment = environment
        self.workspace_config = workspace_config
        self.mode = mode
        self.nav_speed = float(nav_speed)
        self.timeout_s = float(timeout_s)
        self.controller_config_path = config_path
        self.render = bool(render)
        self.max_ticks = (
            int(math.ceil(DEFAULT_MAX_SIM_TIME_S * REAL_CONTROL_HZ))
            if max_ticks is None
            else max_ticks
        )
        if self.max_ticks <= 0:
            raise ValueError("max_ticks must be positive")

        now_fn = lambda: float(self.environment.simulation_time)
        goal_cm = tuple(float(value) for value in self.environment.goal_cm)
        self.planner = NavigationBaselinePlanner(
            goal_cm=goal_cm,
            workspace_bounds_m=(
                0.0,
                workspace_config.width / 100.0,
                0.0,
                workspace_config.height / 100.0,
            ),
            robot_width_cm=workspace_config.car_width,
            robot_height_cm=workspace_config.car_height,
            mode=mode,
            timeout_s=self.timeout_s,
            now_fn=now_fn,
            navigation_config=self.controller_configs.navigation,
        )
        self.navigation_controller = create_navigation_controller(
            workspace_config,
            nav_speed_override=self.nav_speed,
            now_fn=now_fn,
            controller_configs=self.controller_configs,
        )
        self.planner.attach_navigation_controller(self.navigation_controller)
        self.executor = SubgoalExecutor(
            workspace_config,
            self.navigation_controller,
        )
        self._ticks = 0
        self._planning_calls = 0
        self._planning_wall_clock_s = 0.0

    def run(self, output_dir: Path | str) -> Dict[str, Any]:
        wall_clock_started = time.perf_counter()
        output = Path(output_dir).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=True)
        trajectory_path = output / "trajectory.jsonl"
        trajectory_work_path = output / "trajectory.jsonl.tmp"
        partial_trajectory_path = output / "trajectory.partial.jsonl"
        result_path = output / "result.json"
        result_work_path = output / "result.json.tmp"
        # A failed rerun must never leave a prior result looking current.
        result_path.unlink(missing_ok=True)
        result_work_path.unlink(missing_ok=True)
        trajectory_work_path.unlink(missing_ok=True)
        partial_trajectory_path.unlink(missing_ok=True)
        obs = self.environment.observe()
        initial_robot_pose = _robot_pose_dict(obs)
        status = "failure"
        reason = "planner returned no subgoal while goal was unreachable"

        try:
            with trajectory_work_path.open("w", encoding="utf-8") as trajectory:
                self._write_trajectory(trajectory, obs, action=None)
                try:
                    while True:
                        if self.executor.is_done(obs):
                            if self.executor.has_active_subgoal():
                                failed = self.executor.did_fail()
                                self.planner.notify_subgoal_done(obs, failed=failed)

                            if self.planner.is_complete(obs):
                                status = "success"
                                reason = "goal reached"
                                obs = self._apply_and_record(Action.stop(), trajectory)
                                break

                            planning_started = time.perf_counter()
                            try:
                                subgoal = self.planner.plan(obs)
                            finally:
                                self._planning_calls += 1
                                self._planning_wall_clock_s += (
                                    time.perf_counter() - planning_started
                                )
                            if subgoal is not None:
                                self.executor.set_subgoal(subgoal, obs)
                            else:
                                if self.planner.is_complete(obs):
                                    status = "success"
                                    reason = "goal reached"
                                else:
                                    status = "failure"
                                    reason = (
                                        self.planner.outcome.failure
                                        or "planner returned no subgoal while goal was unreachable"
                                    )
                                obs = self._apply_and_record(Action.stop(), trajectory)
                                break

                        # Reserve the final configured control tick for the stop
                        # command. The harness therefore terminates at, never one
                        # physics step after, its declared limit.
                        if self._ticks >= self.max_ticks - 1:
                            reason = "simulation_harness_tick_limit_reached"
                            self.planner.record_external_failure(obs, reason)
                            obs = self._apply_and_record(Action.stop(), trajectory)
                            break

                        action = self.executor.step(obs)
                        obs = self._apply_and_record(action, trajectory)
                except BaseException:
                    try:
                        self._apply_and_record(Action.stop(), trajectory)
                    except BaseException:
                        pass
                    raise
        except BaseException:
            if trajectory_work_path.exists():
                os.replace(trajectory_work_path, partial_trajectory_path)
            raise

        wall_clock_elapsed_s = time.perf_counter() - wall_clock_started
        payload = self._result_payload(
            status=status,
            reason=reason,
            initial_robot_pose=initial_robot_pose,
            final_obs=obs,
            trajectory_path=trajectory_path,
            wall_clock_elapsed_s=wall_clock_elapsed_s,
        )
        result_work_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        # Publish the trajectory first and the result last. A result is the
        # reducer's completeness marker, so it must never point at a partial run.
        os.replace(trajectory_work_path, trajectory_path)
        os.replace(result_work_path, result_path)
        return payload

    def _apply_and_record(self, action: Action, trajectory: TextIO) -> Observation:
        self.environment.apply(action)
        self._ticks += 1
        obs = self.environment.observe()
        if self.render:
            self.environment.render()
        self._write_trajectory(trajectory, obs, action=action)
        return obs

    def _write_trajectory(
        self,
        handle: TextIO,
        obs: Observation,
        *,
        action: Optional[Action],
    ) -> None:
        row = {
            "tick": self._ticks,
            "simulation_time_s": float(obs.timestamp),
            "robot": {
                "x_cm": float(obs.robot_x),
                "y_cm": float(obs.robot_y),
                "theta_deg": float(obs.robot_theta),
            },
            "objects": {
                name: {
                    "x_cm": float(pose.x),
                    "y_cm": float(pose.y),
                    "theta_deg": float(pose.theta),
                    "is_static": bool(pose.is_static),
                }
                for name, pose in sorted(obs.objects.items())
            },
            "action": (
                None
                if action is None
                else {
                    "left_speed": float(action.left_speed),
                    "right_speed": float(action.right_speed),
                }
            ),
            "controller_state": self.navigation_controller.state.value,
        }
        handle.write(json.dumps(row, sort_keys=True) + "\n")
        handle.flush()

    def _result_payload(
        self,
        *,
        status: str,
        reason: str,
        initial_robot_pose: Dict[str, float],
        final_obs: Observation,
        trajectory_path: Path,
        wall_clock_elapsed_s: float,
    ) -> Dict[str, Any]:
        outcome = self.planner.outcome.as_row()
        inflation_path = DEFAULT_WAVEFRONT_INFLATION_YAML.resolve()
        inflation_config = get_wavefront_inflation_config()
        payload = dict(outcome)
        payload.update(
            {
                "schema_version": 1,
                "arm": f"nav_baseline_{self.mode}",
                "mode": self.mode,
                "status": status,
                "reason": reason,
                "scene_xml": str(self.environment.xml_path),
                "namo_config": _optional_path(self.environment.namo_config_path),
                "namo_rl_binding": _optional_path(self.environment.binding_path),
                "controller_config": _optional_path(self.controller_config_path),
                "controller_stack": {
                    "planner": type(self.planner).__name__,
                    "executor": type(self.executor).__name__,
                    "navigation_controller": type(
                        self.navigation_controller
                    ).__name__,
                    "path_follower": type(
                        self.navigation_controller._path_follower
                    ).__name__,
                },
                "controller_parameters": {
                    "navigation": asdict(self.controller_configs.navigation),
                    "push_retreat": asdict(self.controller_configs.push),
                },
                "wavefront_inflation": asdict(inflation_config),
                "wavefront_inflation_config": str(inflation_path),
                "wavefront_inflation_config_sha256": _optional_sha256(
                    inflation_path
                ),
                "nav_speed": self.nav_speed,
                "control_hz": float(self.environment.control_hz),
                "control_ticks": self._ticks,
                "simulation_time_s": float(self.environment.simulation_time),
                "wall_clock_elapsed_s": float(wall_clock_elapsed_s),
                "planning_calls": self._planning_calls,
                "planning_wall_clock_s": float(self._planning_wall_clock_s),
                "n_sims": 0,
                "harness_limit_ticks": self.max_ticks,
                "harness_limit_reached": (
                    reason == "simulation_harness_tick_limit_reached"
                ),
                "wheel_command_limit_rad_s": float(
                    self.environment.wheel_command_limit_rad_s
                ),
                "goal_cm": [float(value) for value in self.environment.goal_cm],
                "initial_robot_pose": initial_robot_pose,
                "final_robot_pose": _robot_pose_dict(final_obs),
                "planned_route_cm": [
                    list(point) for point in self.planner._route_cm
                ],
                "outcome": outcome,
                "trajectory": str(trajectory_path),
            }
        )
        return payload


def _robot_pose_dict(obs: Observation) -> Dict[str, float]:
    return {
        "x_cm": float(obs.robot_x),
        "y_cm": float(obs.robot_y),
        "theta_deg": float(obs.robot_theta),
    }


def _optional_path(value: Any) -> Optional[str]:
    return None if value is None else str(value)


def _optional_sha256(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
