#!/usr/bin/env python3
"""Run the real pure-navigation baselines in the existing MuJoCo simulator."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Sequence, Tuple

import yaml

from robot_control.controller.config import load_controller_configs
from robot_control.core.types import WorkspaceConfig
from robot_control.environment.namo_mujoco import NamoMujocoEnvironment
from robot_control.navigation_baseline_sim import (
    DEFAULT_MAX_SIM_TIME_S,
    DEFAULT_NAV_SPEED,
    REAL_CONTROL_HZ,
    NavigationBaselineSimulation,
)
from robot_control.planner.navigation_baseline_planner import DEFAULT_TIMEOUT_S


OWNED_RUN_ARTIFACTS = (
    "result.json",
    "result.json.tmp",
    "trajectory.jsonl",
    "trajectory.jsonl.tmp",
    "trajectory.partial.jsonl",
)


def _clear_owned_run_artifacts(mode_dir: Path) -> None:
    for name in OWNED_RUN_ARTIFACTS:
        artifact = mode_dir / name
        if artifact.is_dir():
            raise IsADirectoryError(f"owned run artifact is a directory: {artifact}")
        artifact.unlink(missing_ok=True)


def selected_modes(mode: str) -> Tuple[str, ...]:
    if mode == "both":
        return "ignore", "penalise"
    if mode in ("ignore", "penalise"):
        return (mode,)
    raise ValueError(f"unknown mode {mode!r}")


def prepare_mode_directory(
    output_root: Path | str,
    mode: str,
    *,
    allow_overwrite: bool,
) -> Path:
    mode_dir = Path(output_root).expanduser().resolve() / f"nav_baseline_{mode}"
    if mode_dir.is_dir() and any(mode_dir.iterdir()) and not allow_overwrite:
        raise FileExistsError(
            f"{mode_dir} is not empty; pass --allow-overwrite to replace result files"
        )
    mode_dir.mkdir(parents=True, exist_ok=True)
    if allow_overwrite:
        _clear_owned_run_artifacts(mode_dir)
    return mode_dir


def prepare_mode_directories(
    output_root: Path | str,
    modes: Sequence[str],
    *,
    allow_overwrite: bool,
) -> Tuple[Path, ...]:
    """Validate every requested arm before creating any output directory."""
    root = Path(output_root).expanduser().resolve()
    mode_dirs = tuple(root / f"nav_baseline_{mode}" for mode in modes)
    if not allow_overwrite:
        for mode_dir in mode_dirs:
            if mode_dir.is_dir() and any(mode_dir.iterdir()):
                raise FileExistsError(
                    f"{mode_dir} is not empty; pass --allow-overwrite "
                    "to replace result files"
                )
    for mode_dir in mode_dirs:
        mode_dir.mkdir(parents=True, exist_ok=True)
        if allow_overwrite:
            _clear_owned_run_artifacts(mode_dir)
    return mode_dirs


def load_real_workspace(config_path: Path | str) -> WorkspaceConfig:
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"real robot config does not exist: {path}")
    with path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    try:
        width = float(config["workspace"]["width_cm"])
        height = float(config["workspace"]["height_cm"])
        car_width = float(config["robot"]["width_cm"])
        car_height = float(config["robot"]["height_cm"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"{path} must define workspace width/height and robot width/height in cm"
        ) from exc
    return WorkspaceConfig(
        width=width,
        height=height,
        car_width=car_width,
        car_height=car_height,
        offset_w=car_width / 2.0,
        offset_h=car_height / 2.0,
        wheel_base=car_width,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run nav_baseline_ignore and/or nav_baseline_penalise with the "
            "real planner/controller/executor stack driving MuJoCo."
        )
    )
    parser.add_argument("--xml", required=True, help="MuJoCo scene XML")
    parser.add_argument("--out", required=True, help="output root for mode directories")
    parser.add_argument(
        "--mode",
        choices=("ignore", "penalise", "both"),
        default="both",
    )
    parser.add_argument("--real-config", default="config/real.yaml")
    parser.add_argument("--controller-config", default="config/controller.yaml")
    parser.add_argument("--namo-config", default=None)
    parser.add_argument(
        "--nav-speed",
        "--speed",
        dest="nav_speed",
        type=float,
        default=DEFAULT_NAV_SPEED,
        help=f"real normalized navigation speed (default: {DEFAULT_NAV_SPEED})",
    )
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument(
        "--max-sim-time",
        type=float,
        default=DEFAULT_MAX_SIM_TIME_S,
        help=(
            "harness guard in simulated seconds; not a baseline timeout "
            f"(default: {DEFAULT_MAX_SIM_TIME_S:g})"
        ),
    )
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--allow-overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        scene = Path(args.xml).expanduser().resolve()
        if not scene.is_file():
            raise FileNotFoundError(f"MuJoCo scene does not exist: {scene}")
        real_config = Path(args.real_config).expanduser().resolve()
        controller_config = Path(args.controller_config).expanduser().resolve()
        if not controller_config.is_file():
            raise FileNotFoundError(
                f"controller config does not exist: {controller_config}"
            )
        workspace = load_real_workspace(real_config)
        controller_configs = load_controller_configs(controller_config)
        if not math.isfinite(args.max_sim_time) or args.max_sim_time <= 0.0:
            raise ValueError("--max-sim-time must be finite and positive")
        max_ticks = int(math.ceil(args.max_sim_time * REAL_CONTROL_HZ))

        modes = selected_modes(args.mode)
        mode_dirs = prepare_mode_directories(
            args.out,
            modes,
            allow_overwrite=args.allow_overwrite,
        )
        for mode, mode_dir in zip(modes, mode_dirs):
            environment = NamoMujocoEnvironment(
                scene,
                namo_config_path=args.namo_config,
                control_hz=REAL_CONTROL_HZ,
                visualize=args.visualize,
            )
            session = NavigationBaselineSimulation(
                environment=environment,
                workspace_config=workspace,
                mode=mode,
                nav_speed=args.nav_speed,
                timeout_s=args.timeout,
                controller_configs=controller_configs,
                controller_config_path=controller_config,
                max_ticks=max_ticks,
                render=args.visualize,
            )
            result = session.run(mode_dir)
            outcome = result["outcome"]
            print(
                f"{result['arm']}: status={result['status']} "
                f"reached={outcome['reached']} "
                f"distance_to_goal_cm={outcome['distance_to_goal_cm']} "
                f"sim_time_s={result['simulation_time_s']:.3f} -> {mode_dir}"
            )
        return 0
    except Exception as exc:
        print(f"navigation baseline simulation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
