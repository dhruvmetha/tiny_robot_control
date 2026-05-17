#!/usr/bin/env python3
"""Run NAMO planning and execution.

This script runs the full NAMO pipeline:
1. Captures current robot/object state
2. Generates MuJoCo XML for NAMO planning
3. Runs NAMO planning (RegionOpeningPlanner)
4. Executes generated PushSubgoals on robot

Goal can be specified via --goal or detected from goal marker (ArUco 6x6, ID 0).
In simulation mode, --goal is required. In real mode, goal marker detection is used by default.

Usage:
    # Real robot mode (uses detected goal marker)
    python scripts/run_namo.py --config config/real.yaml

    # Interactive mode with step-by-step prompts
    python scripts/run_namo.py --config config/real.yaml --interactive

    # Real robot with manual goal override
    python scripts/run_namo.py --config config/real.yaml --goal 50 40

    # Simulation mode (--goal required)
    python scripts/run_namo.py --sim --goal 50 40

    # Simulation with verbose output
    python scripts/run_namo.py --sim --goal 50 40 -v

    # Debug mode (save XML to file)
    python scripts/run_namo.py --sim --goal 50 40 --debug-xml /tmp/namo_env.xml
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from robot_control import Runtime, RuntimeConfig, SimConfig
from robot_control.core.object_defs import ObjectDef
from robot_control.planner import NAMOPlanner
from robot_control.planner.namo_binding_loader import load_canonical_namo_rl


def find_namo_config() -> str:
    """Find NAMO config file relative to this script."""
    script_dir = Path(__file__).parent
    robot_control_dir = script_dir.parent

    # Try relative to robot_control
    namo_config = robot_control_dir.parent / "namo_cpp" / "config" / "namo_config_complete_skill15.yaml"
    if namo_config.exists():
        return str(namo_config)

    # Try from workspace root
    workspace_root = robot_control_dir.parent
    namo_config = workspace_root / "namo_cpp" / "config" / "namo_config_complete_skill15.yaml"
    if namo_config.exists():
        return str(namo_config)

    raise FileNotFoundError(
        "Could not find namo_config_complete_skill15.yaml. "
        "Please specify --namo-config path."
    )


def find_primitive_data_dir() -> str:
    """Find primitive data directory relative to this script."""
    script_dir = Path(__file__).parent
    robot_control_dir = script_dir.parent

    # Try relative to robot_control
    data_dir = robot_control_dir.parent / "namo_cpp" / "data"
    if data_dir.exists():
        return str(data_dir)

    # Fallback to "data" (relative to namo_cpp)
    return "data"


def get_namo_paths() -> Tuple[Path, Path, Path]:
    """Get important paths for NAMO integration."""
    script_dir = Path(__file__).parent
    robot_control_dir = script_dir.parent
    namo_root = robot_control_dir.parent
    namo_cpp_dir = namo_root / "namo_cpp"
    return robot_control_dir, namo_root, namo_cpp_dir


def setup_namo_imports():
    """Setup Python path for NAMO imports."""
    load_canonical_namo_rl(Path(__file__).resolve())


def load_camera_config(config_path: str, objects_path: str):
    """Load camera and observer configuration."""
    from robot_control.camera import ObserverConfig
    from robot_control.camera.observer import ObjectDefinition
    from robot_control.nodes import CameraConfig

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    camera_cfg = config.get("camera", {})
    robot_cfg = config.get("robot", {})
    workspace_cfg = config.get("workspace", {})

    # Parse marker offset
    marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
    if isinstance(marker_offset, list) and len(marker_offset) == 2:
        marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
    else:
        marker_offset_tuple = (0.0, 0.0)

    # Load object definitions
    object_defs = {}
    objects_file = Path(objects_path)
    if objects_file.exists():
        with open(objects_file, "r") as f:
            obj_config = yaml.safe_load(f)
        for name, obj_cfg in obj_config.get("objects", {}).items():
            obj_type = obj_cfg.get("type", "movable")
            shape = obj_cfg.get("shape", {})
            offset = obj_cfg.get("marker_offset", {})
            object_defs[name] = ObjectDefinition(
                marker_id=obj_cfg["marker_id"],
                is_static=(obj_type == "static"),
                is_goal=(obj_type == "goal"),
                width_cm=shape.get("width", 0.0),
                depth_cm=shape.get("depth", 0.0),
                height_cm=shape.get("height", 0.0),
                marker_offset_x_cm=offset.get("x", 0.0),
                marker_offset_y_cm=offset.get("y", 0.0),
            )

    object_marker_size = config.get("object_marker_size_mm") or robot_cfg.get("object_marker_size_mm", 30.0)

    camera_config = CameraConfig(
        camera_device=camera_cfg.get("device", 0),
        resolution=camera_cfg.get("resolution", "720p"),
        fps=camera_cfg.get("fps", 60),
        exposure=camera_cfg.get("exposure", -6),
        calibration_file=camera_cfg.get("calibration_file", ""),
    )

    observer_config = ObserverConfig(
        calibration_file=camera_cfg.get("calibration_file", ""),
        robot_marker_id=robot_cfg.get("marker_id", 1),
        robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
        marker_to_wheel_offset_cm=marker_offset_tuple,
        object_defs=object_defs,
        object_marker_size_mm=object_marker_size,
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    return camera_config, observer_config


def detect_goal_from_camera(
    config_path: str,
    objects_path: str = "config/objects.yaml",
    timeout: float = 5.0,
    camera_service: Optional[str] = None,
) -> Optional[Tuple[float, float]]:
    """Detect goal marker position from camera.

    If camera_service address is provided, connects via ZMQ instead of
    creating a local camera + observer.
    """
    print("\n" + "=" * 60)
    print("STEP 0: Detecting Goal Marker")
    print("=" * 60)
    print("Looking for goal marker (ArUco 6x6, ID 0)...")

    if camera_service:
        from robot_control.nodes.remote_observer import RemoteObserverNode

        node = RemoteObserverNode(camera_service)
        if not node.start():
            print("  ERROR: Failed to connect to camera service!")
            return None

        goal_pos = None
        start_time = time.time()
        try:
            while time.time() - start_time < timeout:
                obs = node.get()
                if obs is not None and obs.goal_x is not None and obs.goal_y is not None:
                    goal_pos = (obs.goal_x, obs.goal_y)
                    break
                time.sleep(0.05)
        finally:
            node.stop()
    else:
        from robot_control.camera import ArucoObserver
        from robot_control.nodes import CameraSensorNode

        camera_config, observer_config = load_camera_config(config_path, objects_path)

        camera = CameraSensorNode(camera_config)
        if not camera.start():
            print("  ERROR: Failed to start camera!")
            return None

        observer = ArucoObserver(observer_config)
        if not observer.start():
            print("  ERROR: Failed to start observer!")
            camera.stop()
            return None

        goal_pos = None
        start_time = time.time()
        try:
            while time.time() - start_time < timeout:
                obs = observer.get()
                if obs is not None and obs.goal_x is not None and obs.goal_y is not None:
                    goal_pos = (obs.goal_x, obs.goal_y)
                    break
                time.sleep(0.05)
        finally:
            observer.stop()
            camera.stop()

    if goal_pos:
        print(f"  SUCCESS: Goal marker detected at ({goal_pos[0]:.1f}, {goal_pos[1]:.1f}) cm")
    else:
        print("  ERROR: Goal marker not detected!")

    return goal_pos


def capture_scene_interactive(
    config_path: str,
    objects_path: str,
    stable_frames: int = 10,
    camera_service: Optional[str] = None,
):
    """Capture scene from camera with visual feedback.

    If camera_service address is provided, polls the remote observer
    and uses terminal-based capture (press Enter) instead of OpenCV window.
    The operator can monitor the camera via the camera_service --show window.
    """
    print("\n" + "=" * 60)
    print("STEP 1: Capture Scene from Camera")
    print("=" * 60)

    if camera_service:
        from robot_control.nodes.remote_observer import RemoteObserverNode

        print("Connecting to camera service for scene capture...")
        print("Monitor the camera in the camera_service window.")
        print("Press Enter to capture when ready, or 'q'+Enter to quit.")

        node = RemoteObserverNode(camera_service)
        if not node.start():
            print("  ERROR: Failed to connect to camera service!")
            return None

        stable_count = 0
        last_obs = None
        captured_obs = None

        import select
        import sys

        try:
            while captured_obs is None:
                obs = node.get()
                if obs is not None:
                    if last_obs is not None:
                        dx = abs(obs.robot_x - last_obs.robot_x)
                        dy = abs(obs.robot_y - last_obs.robot_y)
                        if dx < 1.0 and dy < 1.0:
                            stable_count += 1
                        else:
                            stable_count = 0
                    last_obs = obs

                    goal_str = (
                        f"({obs.goal_x:.1f}, {obs.goal_y:.1f})"
                        if obs.goal_x is not None
                        else "N/A"
                    )
                    print(
                        f"\r  Robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ "
                        f"{obs.robot_theta:.1f} | Objects: {len(obs.objects)} | "
                        f"Goal: {goal_str} | Stable: {stable_count}/{stable_frames}  ",
                        end="",
                        flush=True,
                    )

                # Non-blocking stdin check
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    line = sys.stdin.readline().strip()
                    if line.lower() == "q":
                        print("\n\nCancelled by user.")
                        break
                    elif obs is not None:
                        captured_obs = obs
                        print("\n\n  SUCCESS: Scene captured!")
                else:
                    time.sleep(0.05)
        finally:
            node.stop()

        return captured_obs

    # Local camera mode (original behavior)
    import cv2
    from robot_control.camera import ArucoObserver
    from robot_control.nodes import CameraSensorNode

    camera_config, observer_config = load_camera_config(config_path, objects_path)

    print("Starting camera and ArUco detection...")
    print("Press 'c' to capture, 'q' to quit")

    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("  ERROR: Failed to start camera!")
        return None

    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("  ERROR: Failed to start observer!")
        camera.stop()
        return None

    cv2.namedWindow("Capture Scene", cv2.WINDOW_NORMAL)

    stable_count = 0
    last_obs = None
    captured_obs = None

    try:
        while captured_obs is None:
            vis = observer.get_vis_frame()
            if vis is not None:
                cv2.putText(vis, f"Stable: {stable_count}/{stable_frames} - Press 'c' to capture",
                           (10, vis.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Capture Scene", vis)

            obs = observer.get()
            if obs is not None:
                if last_obs is not None:
                    dx = abs(obs.robot_x - last_obs.robot_x)
                    dy = abs(obs.robot_y - last_obs.robot_y)
                    if dx < 1.0 and dy < 1.0:
                        stable_count += 1
                    else:
                        stable_count = 0
                last_obs = obs

                goal_str = f"({obs.goal_x:.1f}, {obs.goal_y:.1f})" if obs.goal_x is not None else "N/A"
                print(f"\r  Robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ {obs.robot_theta:.1f}° | "
                      f"Objects: {len(obs.objects)} | Goal: {goal_str} | Stable: {stable_count}/{stable_frames}  ",
                      end="", flush=True)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                print("\n\nCancelled by user.")
                break
            elif key == ord('c') and obs is not None:
                captured_obs = obs
                print("\n\n  SUCCESS: Scene captured!")

            time.sleep(0.01)

    finally:
        cv2.destroyAllWindows()
        observer.stop()
        camera.stop()

    return captured_obs


def run_interactive_mode(args):
    """Run NAMO in interactive step-by-step mode.

    Uses NAMOPlanner for execution, which includes:
    - Reachability checking before/after pushes
    - Automatic navigation to goal when reachable
    - Re-planning if goal becomes blocked
    """
    robot_control_dir, namo_root, namo_cpp_dir = get_namo_paths()

    # Change to robot_control directory
    os.chdir(str(robot_control_dir))

    print("\n" + "=" * 60)
    print("NAMO Interactive Mode")
    print("=" * 60)
    print(f"Mode: real")
    print(f"Config: {args.config}")
    print(f"Objects: {args.objects}")
    print("=" * 60)

    input("\n[Press Enter to start STEP 1: Capture Scene...]")

    # =========================================================================
    # STEP 1: Capture scene
    # =========================================================================
    obs = capture_scene_interactive(
        args.config, args.objects,
        camera_service=getattr(args, "camera_service", None),
    )
    if obs is None:
        print("ERROR: No observation captured!")
        return 1

    # Determine goal
    if args.goal is not None:
        goal_cm = (args.goal[0], args.goal[1])
        goal_source = "command line"
    elif obs.goal_x is not None and obs.goal_y is not None:
        goal_cm = (obs.goal_x, obs.goal_y)
        goal_source = "detected marker"
    else:
        print("\nERROR: No goal specified!")
        print("Either provide --goal X Y or place goal marker (ArUco 6x6, ID 0) in the scene.")
        return 1

    input("\n[Press Enter to continue to STEP 2: Scene Summary...]")

    # =========================================================================
    # STEP 2: Scene summary
    # =========================================================================
    print("\n" + "=" * 60)
    print("STEP 2: Scene Summary")
    print("=" * 60)
    print(f"\nRobot:")
    print(f"  Position: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) cm")
    print(f"  Orientation: {obs.robot_theta:.1f}°")

    print(f"\nGoal:")
    print(f"  Position: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm ({goal_source})")

    print(f"\nObjects ({len(obs.objects)}):")
    movable_count = 0
    static_count = 0
    for name, obj in sorted(obs.objects.items()):
        type_str = "[STATIC]" if obj.is_static else "[MOVABLE]"
        if obj.is_static:
            static_count += 1
        else:
            movable_count += 1
        print(f"  {type_str} {name}:")
        print(f"    Position: ({obj.x:.1f}, {obj.y:.1f}) cm @ {obj.theta:.1f}°")
        print(f"    Size: {obj.width:.1f} x {obj.depth:.1f} x {obj.height:.1f} cm")

    print(f"\nSummary: {movable_count} movable, {static_count} static objects")

    if args.dry_run:
        print("\n  DRY RUN MODE - Not sending commands to robot")
        return 0

    confirm = input("\n  Execute NAMO on robot? [y/N]: ").strip().lower()
    if confirm != 'y':
        print("  Cancelled by user.")
        return 0

    # =========================================================================
    # STEP 3: Create NAMOPlanner and Execute
    # =========================================================================
    print("\n" + "=" * 60)
    print("STEP 3: Execute with NAMOPlanner")
    print("=" * 60)

    # Find NAMO config
    namo_config_path = args.namo_config if args.namo_config else find_namo_config()
    primitive_data_dir = find_primitive_data_dir()

    # Get workspace and robot dimensions for reachability checking
    from robot_control.camera.workspace import WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM

    # Load robot dimensions from config
    with open(args.config, "r") as f:
        real_config = yaml.safe_load(f)
    robot_cfg = real_config.get("robot", {})
    robot_width_cm = robot_cfg.get("width_cm", 8.0)
    robot_height_cm = robot_cfg.get("height_cm", 10.0)

    print(f"\n  NAMO config: {namo_config_path}")
    print(f"  Algorithm: {args.algorithm}")
    print(f"  Goal strategy: {args.strategy}")
    print(f"  Max chain depth: {args.max_chain_depth}")
    print(f"  Workspace: {WORKSPACE_WIDTH_CM}x{WORKSPACE_HEIGHT_CM} cm")
    print(f"  Robot: {robot_width_cm}x{robot_height_cm} cm")

    # Create NAMOPlanner (same as automatic mode)
    planner = NAMOPlanner(
        robot_goal_cm=goal_cm,
        namo_config_path=namo_config_path,
        algorithm=args.algorithm,
        goal_strategy=args.strategy,
        scale_factor=6.0,
        primitive_data_dir=primitive_data_dir,
        replan_on_completion=not args.no_replan,
        max_chain_depth=args.max_chain_depth,
        allow_collisions=args.allow_collisions,
        frontier_beam_width=args.frontier_beam_width,
        chain_link_cost=args.chain_link_cost,
        selection_strategy=args.selection_strategy,
        goals_per_region=args.goals_per_region,
        verbose=args.verbose,
        debug_xml_path=args.debug_xml,
        enable_viewer=args.viewer,
        pause_after_load=args.pause,
        ml_goal_model_path=args.ml_goal_model_path,
        ml_device=args.ml_device,
        # Workspace config for reachability (must match navigation planner)
        workspace_width_cm=WORKSPACE_WIDTH_CM,
        workspace_height_cm=WORKSPACE_HEIGHT_CM,
        robot_width_cm=robot_width_cm,
        robot_height_cm=robot_height_cm,
    )

    print("\n  NAMOPlanner will:")
    print("    1. Check if goal is already reachable")
    print("    2. If blocked, run NAMO planning and execute pushes")
    print("    3. Re-check reachability after each push sequence")
    print("    4. Navigate to goal when reachable")

    # Create and run runtime
    runtime_config = RuntimeConfig(
        mode="real",
        config_path=args.config,
        planner=planner,
        dry_run=args.dry_run,
        quit_on_complete=True,
        camera_service_address=getattr(args, "camera_service", None),
    )

    print("\n  Starting robot execution...")
    print("  Press ESCAPE to abort")
    print("  " + "-" * 40)

    runtime = Runtime(runtime_config)
    runtime.run()

    print("\n" + "=" * 60)
    print("Execution Complete")
    print("=" * 60)

    return 0


def run_automatic_mode(args):
    """Run NAMO in automatic mode (original behavior)."""
    robot_control_dir, _, _ = get_namo_paths()

    # Determine mode
    if args.config:
        mode = "real"
    else:
        mode = "sim"
        args.sim = True

    # Determine goal
    if args.goal is not None:
        goal_cm = (args.goal[0], args.goal[1])
        goal_source = "command line"
    elif mode == "real":
        # Detect goal from camera in real mode
        os.chdir(str(robot_control_dir))
        goal_cm = detect_goal_from_camera(
            args.config, args.objects,
            camera_service=getattr(args, "camera_service", None),
        )
        if goal_cm is None:
            print("\nError: No goal specified!")
            print("Either provide --goal X Y or place goal marker (ArUco 6x6, ID 0) in the scene.")
            return 1
        goal_source = "detected marker"
    else:
        # Simulation mode requires --goal
        print("Error: --goal is required in simulation mode (no camera to detect goal marker)")
        return 1

    # Find NAMO config
    if args.namo_config:
        namo_config_path = args.namo_config
    else:
        try:
            namo_config_path = find_namo_config()
        except FileNotFoundError as e:
            print(f"Error: {e}")
            return 1

    # Find primitive data directory
    primitive_data_dir = find_primitive_data_dir()

    if args.verbose:
        print("\n" + "=" * 60)
        print("NAMO Planning and Execution")
        print("=" * 60)
        print(f"Mode: {mode}")
        print(f"Goal: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm ({goal_source})")
        print(f"NAMO config: {namo_config_path}")
        print(f"Algorithm: {args.algorithm}")
        print(f"Strategy: {args.strategy}")
        print(f"Max chain depth: {args.max_chain_depth}")
        print(f"Allow collisions: {args.allow_collisions}")
        print(f"Frontier beam width: {args.frontier_beam_width}")
        print(f"Chain link cost: {args.chain_link_cost}")
        print(f"Selection strategy: {args.selection_strategy}")
        print(f"Goals per region: {args.goals_per_region}")
        print(f"Replan on completion: {not args.no_replan}")
        print("=" * 60)
        sys.stdout.flush()

    # Get workspace and robot dimensions for reachability checking
    # Must match what navigation planner uses
    from robot_control.camera.workspace import WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM

    # Load robot dimensions from config
    robot_width_cm = 8.0  # Default
    robot_height_cm = 10.0  # Default
    if mode == "real" and args.config:
        with open(args.config, "r") as f:
            real_config = yaml.safe_load(f)
        robot_cfg = real_config.get("robot", {})
        robot_width_cm = robot_cfg.get("width_cm", 8.0)
        robot_height_cm = robot_cfg.get("height_cm", 10.0)

    if args.verbose:
        print(f"Workspace: {WORKSPACE_WIDTH_CM}x{WORKSPACE_HEIGHT_CM} cm")
        print(f"Robot: {robot_width_cm}x{robot_height_cm} cm")

    # Create NAMO planner
    planner = NAMOPlanner(
        robot_goal_cm=goal_cm,
        namo_config_path=namo_config_path,
        algorithm=args.algorithm,
        goal_strategy=args.strategy,
        scale_factor=6.0,
        primitive_data_dir=primitive_data_dir,
        replan_on_completion=not args.no_replan,
        max_chain_depth=args.max_chain_depth,
        allow_collisions=args.allow_collisions,
        frontier_beam_width=args.frontier_beam_width,
        chain_link_cost=args.chain_link_cost,
        selection_strategy=args.selection_strategy,
        goals_per_region=args.goals_per_region,
        verbose=args.verbose,
        debug_xml_path=args.debug_xml,
        enable_viewer=args.viewer,
        pause_after_load=args.pause,
        ml_goal_model_path=args.ml_goal_model_path,
        ml_device=args.ml_device,
        # Workspace config for reachability (must match navigation planner)
        workspace_width_cm=WORKSPACE_WIDTH_CM,
        workspace_height_cm=WORKSPACE_HEIGHT_CM,
        robot_width_cm=robot_width_cm,
        robot_height_cm=robot_height_cm,
    )

    # Create runtime config
    if mode == "sim":
        sim_config = SimConfig(
            car_width=8,
            car_height=10,
            x=10,
            y=10,
            theta=0,
            objects={
                # SimConfig objects are (x_cm, y_cm, theta_deg)
                "box_1": (25, 30, 0),
                "box_2": (35, 40, 45),
            },
            # Provide geometry for synthetic simulation objects so XML generation
            # never emits zero-sized geoms during NAMO planning.
            object_defs={
                "box_1": ObjectDef(name="box_1", width=8.0, depth=8.0, height=4.0, is_static=False),
                "box_2": ObjectDef(name="box_2", width=8.0, depth=8.0, height=4.0, is_static=False),
            },
        )

        runtime_config = RuntimeConfig(
            mode="sim",
            sim_config=sim_config,
            planner=planner,
            initial_speed=args.speed,
            quit_on_complete=not args.no_quit,
            show_gui=not args.headless,
        )
    else:
        runtime_config = RuntimeConfig(
            mode="real",
            config_path=args.config,
            planner=planner,
            dry_run=args.dry_run,
            quit_on_complete=not args.no_quit,
            camera_service_address=getattr(args, "camera_service", None),
            show_gui=not args.headless,
            show_camera=not args.headless,
        )

    # Run
    print("\nStarting NAMO execution...")
    print("Press ESCAPE to quit")
    print("=" * 50)

    runtime = Runtime(runtime_config)
    runtime.run()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Run NAMO planning and execution",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # Mode selection
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Run in simulation mode (default if --config not specified)",
    )
    parser.add_argument(
        "--config", "-c",
        type=str,
        default=None,
        help="Path to real robot config (enables real mode)",
    )
    parser.add_argument(
        "--interactive", "-i",
        action="store_true",
        help="Run in interactive mode with step-by-step prompts",
    )
    parser.add_argument(
        "--camera-service",
        type=str,
        default=None,
        metavar="ADDRESS",
        help="ZMQ address of camera service (e.g. tcp://localhost:5556). "
             "Skips local camera creation.",
    )
    # Goal specification
    parser.add_argument(
        "--goal", "-g",
        type=float,
        nargs=2,
        default=None,
        metavar=("X", "Y"),
        help="Goal position in cm (default: detect from goal marker in real mode)",
    )
    parser.add_argument(
        "--objects",
        type=str,
        default="config/objects.yaml",
        help="Path to objects definition file (default: config/objects.yaml)",
    )

    # NAMO config
    parser.add_argument(
        "--namo-config",
        type=str,
        default=None,
        help="Path to NAMO config YAML",
    )
    parser.add_argument(
        "--algorithm",
        type=str,
        default="region_opening",
        help="Planning algorithm (default: region_opening)",
    )
    parser.add_argument(
        "--strategy",
        type=str,
        default="primitive",
        help="Goal strategy (default: primitive)",
    )
    parser.add_argument(
        "--ml-goal-model-path",
        type=str,
        default=None,
        help="Path to trained ML goal model (required for ml strategies)",
    )
    parser.add_argument(
        "--ml-device",
        type=str,
        default="cuda",
        help="PyTorch device for ML model (default: cuda)",
    )
    parser.add_argument(
        "--max-chain-depth",
        type=int,
        default=1,
        help="Maximum chain depth for multi-push solutions (default: 1)",
    )
    parser.add_argument(
        "--allow-collisions",
        action="store_true",
        default=True,
        help="Allow collisions during push (default: True)",
    )
    parser.add_argument(
        "--no-allow-collisions",
        dest="allow_collisions",
        action="store_false",
        help="Terminate push on collision",
    )
    parser.add_argument(
        "--frontier-beam-width",
        type=int,
        default=10000,
        help="Beam width for frontier search (default: 10000)",
    )
    parser.add_argument(
        "--chain-link-cost",
        type=int,
        default=11,
        help="Additional cost per chain link beyond first push (default: 11)",
    )
    parser.add_argument(
        "--selection-strategy",
        type=str,
        default="cost_first",
        choices=["cost_first", "ml_first"],
        help="Frontier priority (default: cost_first)",
    )
    parser.add_argument(
        "--goals-per-region",
        type=int,
        default=10,
        help="Goal samples per region for validation (default: 10)",
    )

    # Execution options
    parser.add_argument(
        "--no-replan",
        action="store_true",
        help="Don't replan when subgoal queue is exhausted",
    )
    parser.add_argument(
        "--no-quit",
        action="store_true",
        help="Don't quit when plan completes",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI window (useful for server/headless execution)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't send commands to robot (real mode only)",
    )

    # Debugging
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    parser.add_argument(
        "--debug-xml",
        type=str,
        default=None,
        help="Save generated XML to this path for debugging",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Enable MuJoCo visualization window during NAMO planning",
    )
    parser.add_argument(
        "--pause",
        action="store_true",
        help="Pause after loading XML for interactive viewer inspection (requires --viewer)",
    )

    # Simulation options
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Max speed (0-1, default: 0.3)",
    )

    args = parser.parse_args()

    try:
        setup_namo_imports()
    except Exception as exc:
        print(f"Error: {exc}")
        return 1

    # Interactive mode requires real robot config
    if args.interactive:
        if not args.config:
            print("Error: --interactive mode requires --config for real robot")
            return 1
        return run_interactive_mode(args)
    else:
        return run_automatic_mode(args)


if __name__ == "__main__":
    exit(main())
