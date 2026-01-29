#!/usr/bin/env python3
"""Test autonomous waypoint navigation on real robot."""

import argparse

from robot_control import Runtime, RuntimeConfig
from robot_control.planner import SequencePlanner, SequencePlannerConfig


def main():
    parser = argparse.ArgumentParser(description="Test autonomous navigation on real robot")

    # Waypoint options
    parser.add_argument("--loop", action="store_true", help="Loop waypoints continuously")
    parser.add_argument("--waypoints", type=str, default=None,
                        help="Custom waypoints as 'x1,y1;x2,y2;...' (cm)")

    # Speed
    parser.add_argument("--speed", type=float, default=0.3, help="Max speed (0-1)")

    # Config paths
    parser.add_argument("--config", type=str, default="config/real.yaml",
                        help="Path to real.yaml config")
    parser.add_argument("--objects", type=str, default="config/objects.yaml",
                        help="Path to objects.yaml config")

    # Serial
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port override (e.g., /dev/ttyACM0)")

    # Testing options
    parser.add_argument("--dry-run", action="store_true",
                        help="Don't send commands to robot (test vision only)")
    parser.add_argument("--no-camera", action="store_true",
                        help="Hide camera view in GUI")
    parser.add_argument("--no-quit", action="store_true",
                        help="Don't quit when plan completes (stay open)")

    args = parser.parse_args()

    # Parse waypoints
    if args.waypoints:
        # Parse custom waypoints: "x1,y1;x2,y2;x3,y3,theta3"
        waypoints = []
        for wp in args.waypoints.split(";"):
            parts = [float(p) for p in wp.split(",")]
            if len(parts) == 2:
                waypoints.append((parts[0], parts[1], None))
            elif len(parts) == 3:
                waypoints.append((parts[0], parts[1], parts[2]))
            else:
                parser.error(f"Invalid waypoint format: {wp}")
    else:
        # Default: small square in center of workspace (safe for 45x65 cm workspace)
        # Workspace is 45cm wide x 65cm tall
        waypoints = [
            (15, 20, None),   # Bottom-left area
            (30, 20, None),   # Bottom-right area
            (30, 45, None),   # Top-right area
            (15, 45, None),   # Top-left area
        ]

    # Create planner
    planner = SequencePlanner(SequencePlannerConfig(
        waypoints=waypoints,
        loop=args.loop,
    ))

    # Create runtime config for real robot
    runtime = Runtime(RuntimeConfig(
        mode="real",
        config_path=args.config,
        objects_path=args.objects,
        serial_port=args.port,
        dry_run=args.dry_run,
        show_camera=not args.no_camera,
        planner=planner,
        initial_speed=args.speed,
        quit_on_complete=not args.no_quit,
    ))

    print("Autonomous Navigation - Real Robot")
    print("=" * 40)
    print(f"Waypoints: {len(waypoints)}")
    for i, (x, y, theta) in enumerate(waypoints):
        theta_str = f"{theta}°" if theta is not None else "any"
        print(f"  {i+1}. ({x}, {y}) theta={theta_str}")
    print(f"Loop: {args.loop}")
    print(f"Speed: {args.speed}")
    print(f"Dry run: {args.dry_run}")
    print("=" * 40)
    print("Press +/- to adjust speed")
    print("Press SPACE for emergency stop")
    print("Press R to toggle recording")
    print("Press ESCAPE to quit")
    print("=" * 40)

    runtime.run()


if __name__ == "__main__":
    main()
