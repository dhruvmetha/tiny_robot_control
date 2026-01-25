"""Test real robot navigation with camera sensing and GUI.

Full pipeline:
  CameraSensorNode -> ArucoObserver -> WorldState -> Controller -> RealEnv

Usage:
    python scripts/test_real_navigation.py
    python scripts/test_real_navigation.py --config config/real.yaml
    python scripts/test_real_navigation.py --port /dev/ttyACM0

Controls:
    Click   : Navigate to clicked position
    R       : Toggle rotation mode
    C       : Cancel navigation
    Space   : Emergency stop
    Escape  : Quit
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Dict, Tuple

import yaml

from robot_control.camera import (
    ArucoObserver,
    ObserverConfig,
    WORKSPACE_WIDTH_CM,
    WORKSPACE_HEIGHT_CM,
    make_real_workspace_config,
)
from robot_control.controller import NavigationController
from robot_control.core.world_state import WorldState
from robot_control.environment.real import RealEnv, RealEnvConfig
from robot_control.gui import Window
from robot_control.nodes import CameraSensorNode, CameraConfig
from robot_control.planner import RVGPlanner


def load_config(config_path: str) -> Tuple[CameraConfig, ObserverConfig, RealEnvConfig, Dict]:
    """Load configuration from YAML file."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Build objects dict: name -> (marker_id, width, height)
    objects = {}
    for name, obj_config in config.get("objects", {}).items():
        marker_id = obj_config["marker_id"]
        size = obj_config.get("size_mm", [40, 40])
        objects[name] = (marker_id, size[0], size[1])

    camera_cfg = config.get("camera", {})
    robot_cfg = config.get("robot", {})
    workspace_cfg = config.get("workspace", {})
    serial_cfg = config.get("serial", {})

    # Parse marker offset
    marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
    if isinstance(marker_offset, list) and len(marker_offset) == 2:
        marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
    else:
        marker_offset_tuple = (0.0, 0.0)

    # Camera sensor config
    camera_config = CameraConfig(
        camera_device=camera_cfg.get("device", 0),
        resolution=camera_cfg.get("resolution", "720p"),
        fps=camera_cfg.get("fps", 60),
        exposure=camera_cfg.get("exposure", -6),
    )

    # ArUco observer config
    observer_config = ObserverConfig(
        calibration_file=camera_cfg.get("calibration_file", ""),
        undistort=camera_cfg.get("undistort", False),
        robot_marker_id=robot_cfg.get("marker_id", 1),
        robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
        marker_to_wheel_offset_cm=marker_offset_tuple,
        objects=objects,
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    # Real environment config
    real_env_config = RealEnvConfig(
        port=serial_cfg.get("port", "/dev/ttyACM0"),
        baudrate=serial_cfg.get("baudrate", 115200),
        robot_id=robot_cfg.get("marker_id", 1),
        send_hz=serial_cfg.get("send_hz", 30.0),
        invert_right_wheel=serial_cfg.get("invert_right_wheel", False),
    )

    return camera_config, observer_config, real_env_config, config


def main():
    parser = argparse.ArgumentParser(description="Test real robot navigation")
    parser.add_argument(
        "--config", "-c",
        type=str,
        default="config/real.yaml",
        help="Path to config file"
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Override serial port (e.g., /dev/ttyACM0)"
    )
    parser.add_argument(
        "--speed", "-s",
        type=float,
        default=0.3,
        help="Maximum speed (default: 0.3)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't send commands to robot (test camera only)"
    )
    args = parser.parse_args()

    # Load config
    config_path = Path(args.config)
    if config_path.exists():
        print(f"Loading config from {config_path}")
        camera_config, observer_config, real_env_config, raw_config = load_config(str(config_path))
    else:
        print(f"Config not found: {config_path}, using defaults")
        camera_config = CameraConfig()
        observer_config = ObserverConfig()
        real_env_config = RealEnvConfig()
        raw_config = {}

    # Override serial port if specified
    if args.port:
        real_env_config.port = args.port

    # Robot dimensions (approximate, in cm)
    robot_width_cm = 8.0
    robot_height_cm = 10.0

    # Create workspace config for real camera
    workspace_config = make_real_workspace_config(
        car_width=robot_width_cm,
        car_height=robot_height_cm,
    )

    # Create RVG planner
    planner = RVGPlanner(
        workspace_width=workspace_config.width,
        workspace_height=workspace_config.height,
        robot_width=workspace_config.car_width,
        robot_height=workspace_config.car_height,
        robot_geometry_scale=1.2,
    )

    # Create navigation controller
    controller = NavigationController(workspace_config, planner, max_speed=args.speed)

    # Create world state (subscribes to SENSOR_VISION)
    world = WorldState()

    # Create camera sensor node
    print("\nStarting CameraSensorNode...")
    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("Failed to start camera!")
        return

    # Create ArUco observer
    print("Starting ArucoObserver...")
    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("Failed to start observer!")
        camera.stop()
        return

    # Create real environment
    real_env = RealEnv(real_env_config)
    if not args.dry_run:
        if not real_env.start():
            print("Failed to start RealEnv!")
            observer.stop()
            camera.stop()
            return
    else:
        print("[DRY RUN] Not sending commands to robot")

    # Create GUI with camera view, connection status, and settings panel
    window = Window(
        workspace_config,
        show_camera=True,
        show_connection=not args.dry_run,
        show_settings=True,
        target_robot_id=real_env_config.robot_id if not args.dry_run else None,
    )
    window.enable_canvas_click(True)
    window.set_controller("Navigation")
    window.update_speed(args.speed)  # Show initial speed

    # State
    running = True
    rotation_mode = False

    def on_canvas_click(x: float, y: float):
        """Handle canvas click - navigate to position."""
        nonlocal rotation_mode

        obs = world.get()
        if obs is None:
            print("No observation available")
            return

        # Build obstacle list from observed objects
        obstacles = [
            (o.x, o.y, o.theta, o.width if o.width > 0 else 4.0, o.height if o.height > 0 else 4.0)
            for o in obs.objects.values()
        ]

        current_pos = (obs.robot_x, obs.robot_y)

        # Calculate goal theta if rotation mode enabled
        goal_theta = None
        if rotation_mode:
            dx = x - obs.robot_x
            dy = y - obs.robot_y
            if math.hypot(dx, dy) > 1.0:
                goal_theta = math.degrees(math.atan2(dy, dx))
                if goal_theta < 0:
                    goal_theta += 360

        # Navigate
        if controller.navigate_to(x, y, goal_theta, current_pos, obstacles):
            if goal_theta is not None:
                print(f"Navigating to ({x:.1f}, {y:.1f}) facing {goal_theta:.0f}°")
            else:
                print(f"Navigating to ({x:.1f}, {y:.1f})")
        else:
            print(f"Planning failed to ({x:.1f}, {y:.1f})")

    def on_key_press(key: str):
        nonlocal running, rotation_mode
        key = key.lower()

        if key == "escape":
            running = False
            window.close_window()
        elif key == "r":
            rotation_mode = not rotation_mode
            mode_str = "ON (face travel direction)" if rotation_mode else "OFF"
            print(f"Rotation mode: {mode_str}")
        elif key == "c":
            controller.cancel()
            print("Navigation cancelled")
        elif key == "space":
            controller.cancel()
            print("Emergency stop")

    window.register_callback("on_canvas_click", on_canvas_click)
    window.register_callback("on_key_press", on_key_press)

    # Control loop
    control_rate = 30.0
    control_dt = 1.0 / control_rate

    def control_loop():
        nonlocal running
        while running:
            loop_start = time.time()

            obs = world.get()
            if obs is not None:
                # Compute action
                action = controller.step(obs, subgoal=None)

                # Send to robot (if not dry run)
                if not args.dry_run:
                    real_env.apply(action)

                # Update GUI
                window.update(obs)
                window.set_status(controller.get_status())
                window.update_drawings(controller.get_drawings())
                window.update_action(action)  # Show current motor commands

            # Update camera view in GUI
            vis_frame = observer.get_vis_frame()
            if vis_frame is not None:
                window.update_camera(vis_frame)

            # Update connection status (if not dry run)
            if not args.dry_run:
                window.update_connection(real_env.get_status())

            # Maintain rate
            elapsed = time.time() - loop_start
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()

    print("\n" + "=" * 50)
    print("Real Robot Navigation")
    print("=" * 50)
    print(f"Serial port: {real_env_config.port}")
    print(f"Robot ID: {real_env_config.robot_id}")
    print(f"Max speed: {args.speed}")
    print(f"Dry run: {args.dry_run}")
    print("=" * 50)
    print("Click on canvas to navigate")
    print("Press 'r' to toggle rotation mode")
    print("Press 'c' to cancel navigation")
    print("Press 'space' for emergency stop")
    print("Press 'escape' to quit")
    print("=" * 50 + "\n")

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    controller.cancel()
    if not args.dry_run:
        real_env.stop()
    observer.stop()
    camera.stop()
    world.unsubscribe()
    print("Done.")


if __name__ == "__main__":
    main()
