#!/usr/bin/env python3
"""Test autonomous control with waypoint sequence."""

import argparse

from robot_control import Runtime, RuntimeConfig, SimConfig
from robot_control.planner import SequencePlanner, SequencePlannerConfig


def main():
    parser = argparse.ArgumentParser(description="Test autonomous waypoint navigation")
    parser.add_argument("--loop", action="store_true", help="Loop waypoints continuously")
    parser.add_argument("--speed", type=float, default=0.3, help="Max speed (0-1)")
    parser.add_argument("--no-quit", action="store_true", help="Don't quit when plan completes")
    args = parser.parse_args()

    # Define patrol waypoints (corners of workspace with some margin)
    waypoints = [
        (10, 10, None),    # Bottom-left
        (35, 10, None),    # Bottom-right
        (35, 55, None),    # Top-right
        (10, 55, None),    # Top-left
    ]

    # Create planner
    planner = SequencePlanner(SequencePlannerConfig(
        waypoints=waypoints,
        loop=args.loop,
    ))

    # Create simulation config (no obstacles for simple test)
    sim_config = SimConfig(
        car_width=8,
        car_height=10,
        x=10,
        y=10,
        theta=0,
        objects={},  # No obstacles
    )

    # Create runtime with planner (autonomous mode)
    runtime = Runtime(RuntimeConfig(
        mode="sim",
        sim_config=sim_config,
        planner=planner,
        initial_speed=args.speed,
        quit_on_complete=not args.no_quit,
    ))

    print("Autonomous Navigation Test")
    print("=" * 40)
    print(f"Waypoints: {len(waypoints)}")
    print(f"Loop: {args.loop}")
    print(f"Speed: {args.speed}")
    print("=" * 40)
    print("Press +/- to adjust speed")
    print("Press SPACE for emergency stop")
    print("Press ESCAPE to quit")
    print("=" * 40)

    runtime.run()


if __name__ == "__main__":
    main()
