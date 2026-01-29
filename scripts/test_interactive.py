#!/usr/bin/env python3
"""Interactive robot control using Runtime.

Usage:
    python scripts/test_interactive.py              # Simulation
    python scripts/test_interactive.py --real       # Real robot
    python scripts/test_interactive.py --real --dry-run  # Real vision, no motors
"""

import argparse

from robot_control import Runtime, RuntimeConfig, SimConfig


def main():
    parser = argparse.ArgumentParser(description="Interactive robot control")
    parser.add_argument("--real", action="store_true", help="Use real robot (default: simulation)")
    parser.add_argument("--speed", type=float, default=0.3, help="Initial speed (0-1)")
    parser.add_argument("--dry-run", action="store_true", help="Real mode: vision only, no motor commands")
    parser.add_argument("--config", type=str, default="config/real.yaml", help="Real mode: config path")
    parser.add_argument("--port", type=str, default=None, help="Real mode: serial port override")
    args = parser.parse_args()

    if args.real:
        config = RuntimeConfig(
            mode="real",
            config_path=args.config,
            serial_port=args.port,
            dry_run=args.dry_run,
            initial_speed=args.speed,
        )
        mode_str = "Real Robot" + (" (dry-run)" if args.dry_run else "")
    else:
        config = RuntimeConfig(
            mode="sim",
            sim_config=SimConfig(
                car_width=8,
                car_height=10,
                x=10,
                y=10,
                theta=0,
                objects={
                    "box1": (22, 32, 0),
                    "box2": (12, 45, 45),
                }
            ),
            initial_speed=args.speed,
        )
        mode_str = "Simulation"

    print(f"\nInteractive Control - {mode_str}")
    print("=" * 40)
    print("Controllers (select from right panel):")
    print("  Keyboard   : WASD to drive")
    print("  Navigation : Click to navigate")
    print("  Follow Path: Draw path to follow")
    print("=" * 40)
    print("Press +/- to adjust speed")
    print("Press SPACE for emergency stop")
    print("Press ESCAPE to quit")
    print("=" * 40 + "\n")

    runtime = Runtime(config)
    runtime.run()


if __name__ == "__main__":
    main()
