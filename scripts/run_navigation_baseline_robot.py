#!/usr/bin/env python3
"""Drive the pure-navigation baseline on the real table.

Same stack `run_namo.py` drives, minus the NAMO. Same camera, same runtime,
same navigation controller, same path follower, same GUI. The only thing
swapped out is the planner: this one plans a route that treats a movable as
something to drive into rather than something to push, and it never emits a
push.

Usage:
    # Detect the goal marker, drive at it, ignoring the blocks
    python scripts/run_navigation_baseline_robot.py --config config/real.yaml

    # Prefer clear space, but cross a block rather than take a long way round
    python scripts/run_navigation_baseline_robot.py --mode penalise

    # Plan and show the route without moving the robot
    python scripts/run_navigation_baseline_robot.py --dry-run

    # Through a running camera service, writing the result row
    python scripts/run_navigation_baseline_robot.py \\
        --camera-service tcp://localhost:5556 --out real_trials/nav_baseline

Two modes:

  ignore    a movable cell costs what open floor costs, so the route is the
            shortest line and the robot drives into whatever sits on it.
  penalise  a movable cell costs five, so the route prefers clear space but
            crosses a block when going round is far enough.

Reaching the goal means the scene never needed NAMO. Stalling short of it is
the measured case for NAMO on that exact layout. Reset the scene afterwards
and run the NAMO trial on it, so both runs face the same arrangement.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import yaml

from robot_control import Runtime, RuntimeConfig
from robot_control.planner.navigation_baseline_planner import (
    BASELINE_MODES,
    DEFAULT_TIMEOUT_S,
    NavigationBaselinePlanner,
)

# run_namo's own goal-marker detection, reused rather than reimplemented so
# both arms read the same marker the same way.
sys.path.insert(0, str(Path(__file__).parent))
from _diag_setup import bootstrap_diagnostics  # noqa: E402
from run_namo import detect_goal_from_camera  # noqa: E402

GOAL_DETECT_TIMEOUT_S = 5.0


def read_workspace(config_path: str) -> Tuple[float, float, float, float]:
    """Table size and robot footprint, in cm, from the real config."""
    with open(config_path, "r") as handle:
        config = yaml.safe_load(handle)
    workspace = config.get("workspace", {})
    robot = config.get("robot", {})
    return (
        float(workspace["width_cm"]),
        float(workspace["height_cm"]),
        float(robot["width_cm"]),
        float(robot["height_cm"]),
    )


def resolve_goal(args) -> Optional[Tuple[float, float]]:
    if args.goal:
        print(f"Goal given: ({args.goal[0]:.1f}, {args.goal[1]:.1f}) cm")
        return (args.goal[0], args.goal[1])
    return detect_goal_from_camera(
        args.config,
        args.objects,
        timeout=GOAL_DETECT_TIMEOUT_S,
        camera_service=args.camera_service,
    )


def report(outcome, mode: str) -> None:
    row = outcome.as_row()
    print("\n" + "=" * 60)
    print(f"  arm                : {row['arm']}")
    print(f"  reached            : {row['reached']}")
    print(f"  stopped at         : ({row['stopped_x_cm']}, {row['stopped_y_cm']}) cm")
    print(f"  distance to goal   : {row['distance_to_goal_cm']} cm")
    print(f"  route waypoints    : {row['route_waypoints']}")
    print(f"  route crossed      : {row['route_crosses'] or 'nothing'}")
    print(f"  objects shoved     : {row['objects_moved'] or 'none'}")
    print(f"  elapsed            : {row['elapsed_s']} s")
    print(f"  failure            : {row['failure_cause'] or 'none'}")
    print("=" * 60)
    if outcome.reached:
        print("\n  Driving alone reached the goal. This scene did NOT need NAMO.")
    else:
        print("\n  Driving alone did not reach the goal. Reset the scene, then")
        print("  run the NAMO trial on the same layout.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the pure-navigation baseline on the real robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--mode", choices=list(BASELINE_MODES), default="ignore",
        help="ignore: a movable costs what floor costs. "
             "penalise: it costs five. (default: ignore)",
    )
    parser.add_argument("--config", "-c", default="config/real.yaml")
    parser.add_argument("--objects", default="config/objects.yaml")
    parser.add_argument(
        "--goal", "-g", nargs=2, type=float, default=None, metavar=("X", "Y"),
        help="Goal in cm. Without this the goal marker is detected from camera.",
    )
    parser.add_argument("--camera-service", default=None,
                        help="ZMQ address of a running camera_service, e.g. tcp://localhost:5556")
    parser.add_argument("--record-video", default=None,
                        help="Ask the camera service to record video into this directory.")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"Give up after this long (default: {DEFAULT_TIMEOUT_S:g} s)")
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--nav-speed", type=float, default=None,
                        help="Override navigation max speed (0-1). Unset uses controller.yaml.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Plan and show the route without sending wheel commands.")
    parser.add_argument("--no-quit", action="store_true",
                        help="Don't quit when the drive ends.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--step-confirm", action="store_true",
                        help="Pause for confirmation before the drive starts.")
    parser.add_argument("--out", default=None,
                        help="Directory to write the result row into.")

    # Same flag names run_namo declares, so a baseline run and a NAMO run land
    # in the same directory layout and an operator types the same thing twice.
    parser.add_argument("--diag-path", type=str, default=None,
                        help="Root directory for diagnostics output. Enables diagnostics when set.")
    parser.add_argument("--run-name", type=str, default=None,
                        help="Subdir under --diag-path. Supports {timestamp} {date} {epoch}.")
    parser.add_argument("--capture-scene", action="store_true",
                        help="Save scene snapshots (jpg + json + xml) at run start and end. "
                             "This is what proves the baseline and the NAMO trial faced the "
                             "same layout.")
    parser.add_argument("--allow-overwrite", action="store_true",
                        help="Allow --diag-path/--run-name to overwrite an existing directory.")
    args = parser.parse_args()

    # run_name templates may carry {algorithm} and {strategy}. This arm runs no
    # search, so name it after the variant rather than leaving the field unset.
    args.algorithm = f"nav_baseline_{args.mode}"
    args.strategy = "none"

    # Installs the Tee so run.log captures everything printed below, writes
    # config.json with a git snapshot of both repositories, and returns the
    # recorder Runtime fills with plans, subgoals and scene captures. Returns
    # (None, None) unless --diag-path is set, so a plain run costs nothing.
    recorder, log_handle = bootstrap_diagnostics(args)

    width_cm, height_cm, robot_w, robot_h = read_workspace(args.config)

    goal = resolve_goal(args)
    if goal is None:
        print("\nNo goal. Place the goal marker where the camera can see it, "
              "or pass --goal X Y.")
        return 1

    planner = NavigationBaselinePlanner(
        goal_cm=goal,
        workspace_bounds_m=(0.0, width_cm / 100.0, 0.0, height_cm / 100.0),
        robot_width_cm=robot_w,
        robot_height_cm=robot_h,
        mode=args.mode,
        timeout_s=args.timeout,
    )

    runtime_config = RuntimeConfig(
        mode="real",
        config_path=args.config,
        objects_path=args.objects,
        planner=planner,
        dry_run=args.dry_run,
        quit_on_complete=not args.no_quit,
        camera_service_address=args.camera_service,
        record_video_dir=args.record_video,
        nav_speed_override=args.nav_speed,
        initial_speed=args.speed,
        step_confirm=args.step_confirm,
        show_gui=not args.headless,
        show_camera=not args.headless,
    )
    # Runtime writes summary.json from these, and attaches the recorder to the
    # planner behind a try/except AttributeError, so the baseline planner needs
    # no diagnostics method of its own.
    runtime_config.diagnostics_recorder = recorder
    runtime_config.capture_scene = bool(args.capture_scene)

    print(f"\nPure navigation baseline, mode '{args.mode}'. No pushes will be planned.")
    print("Press ESCAPE to quit")
    print("=" * 60)

    runtime = Runtime(runtime_config)

    # Nothing in the stack terminates a robot wedged against a block, which is
    # this baseline's expected outcome: FollowPathController finishes only when
    # it consumes its path, a stuck robot never advances the path index, and
    # Runtime polls is_complete only between subgoals. So the run is meant to
    # be capped from outside, with `timeout 120 python scripts/...`.
    #
    # `timeout` sends SIGTERM. Taking it here turns the kill into an ordinary
    # shutdown, so Runtime writes summary.json on its normal path and the
    # result below still gets printed and saved. Without this the row is lost
    # exactly on the runs that matter most.
    def stop_on_signal(signum, _frame):
        print(f"\n[baseline] signal {signum}, stopping and recording where we got to")
        planner.outcome.failure = planner.outcome.failure or "wall_clock_timeout"
        runtime.stop()

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop_on_signal)

    runtime.run()

    report(planner.outcome, args.mode)

    if args.out:
        out_dir = Path(args.out)
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"nav_baseline_{args.mode}_{int(time.time())}.json"
        path.write_text(json.dumps(planner.outcome.as_row(), indent=2))
        print(f"\nwrote {path}")

    if log_handle is not None:
        # Held open until here so the Tee captures the result block above.
        log_handle.close()

    return 0 if planner.outcome.reached else 2


if __name__ == "__main__":
    sys.exit(main())
