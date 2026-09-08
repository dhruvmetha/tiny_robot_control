#!/usr/bin/env python3
"""Measure short, fresh-state wheel turns in representative MuJoCo scenes."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import namo_rl


def wrapped_delta(after: float, before: float) -> float:
    return (after - before + math.pi) % (2.0 * math.pi) - math.pi


def measure(xml: Path, config: Path, left: float, right: float) -> dict:
    env = namo_rl.RLEnvironment(str(xml), str(config), False)
    start = env.get_observation()["robot_pose"]
    previous_yaw = start[2]
    accumulated_yaw = 0.0
    for _ in range(100):
        env.step_wheels(left, right, 0.002)
        current_yaw = env.get_observation()["robot_pose"][2]
        accumulated_yaw += wrapped_delta(current_yaw, previous_yaw)
        previous_yaw = current_yaw
    end = env.get_observation()["robot_pose"]
    return {
        "xml": str(xml),
        "left": left,
        "right": right,
        "wheel_command_limit_rad_s": env.get_wheel_command_limit_rad_s(),
        "start": start,
        "end": end,
        "distance_m": math.hypot(end[0] - start[0], end[1] - start[1]),
        "wrapped_endpoint_yaw_delta_deg": math.degrees(
            wrapped_delta(end[2], start[2])
        ),
        "accumulated_yaw_delta_deg": math.degrees(accumulated_yaw),
    }


def main() -> int:
    repo = Path(sys.argv[1]).resolve()
    candidate = Path(sys.argv[2]).resolve()
    config = repo / "config" / "namo_config_complete_skill15_car_1x.yaml"
    scenes = (
        repo / "test_xml" / "little-car-modeling-package" / "artifacts" / "nav_env.xml",
        candidate,
    )
    commands = ((-0.4, 0.4), (0.4, -0.4), (0.4, 0.4))
    print(json.dumps([
        measure(scene, config, left, right)
        for scene in scenes
        for left, right in commands
    ], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
