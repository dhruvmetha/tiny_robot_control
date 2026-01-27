"""Unified robot control test with multiple controller modes.

Supports switching between:
  - Keyboard (WASD): Manual control
  - Navigation: Click-to-navigate with obstacle avoidance
  - Follow Path: Draw path on canvas to follow

Usage:
    # Simulation mode
    python scripts/test_robot.py --sim

    # Real robot mode
    python scripts/test_robot.py --config config/real.yaml

    # Real robot with custom port
    python scripts/test_robot.py --config config/real.yaml --port /dev/ttyACM0

Controls:
    Keyboard Mode:
        W/A/S/D : Move robot
    Navigation Mode:
        Click   : Navigate to clicked position
    Follow Path Mode:
        Drag    : Draw path on canvas
    All Modes:
        +/-     : Increase/decrease speed
        R       : Toggle camera recording (real robot only)
        C       : Cancel current action
        Space   : Emergency stop
        Escape  : Quit
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path
from typing import Dict, Tuple

import yaml

from robot_control.controller import (
    KeyboardController,
    NavigationController,
    FollowPathController,
)
from robot_control.coordinator import ControlCoordinator
from robot_control.core.types import WorkspaceConfig
from robot_control.core.world_state import WorldState
from robot_control.gui import Window
from robot_control.planner import RVGPlanner

# Conditional imports for real robot
try:
    from robot_control.camera import (
        ArucoObserver,
        ObserverConfig,
        make_real_workspace_config,
    )
    from robot_control.camera.observer import ObjectDefinition
    from robot_control.environment.real import RealEnv, RealEnvConfig
    from robot_control.nodes import CameraSensorNode, CameraConfig
    from robot_control.utils import CameraRecorder
    REAL_AVAILABLE = True
except ImportError:
    REAL_AVAILABLE = False

# Conditional imports for simulation
try:
    from robot_control.environment.sim import SimEnv, SimConfig
    from robot_control.nodes import SimSensorNode
    SIM_AVAILABLE = True
except ImportError:
    SIM_AVAILABLE = False


def load_objects_yaml(objects_path: str) -> Dict[str, "ObjectDefinition"]:
    """Load object definitions from objects.yaml."""
    with open(objects_path, "r") as f:
        data = yaml.safe_load(f)

    object_defs = {}
    for name, obj in data.get("objects", {}).items():
        obj_type = obj.get("type", "movable")
        shape = obj.get("shape", {})
        offset = obj.get("marker_offset", {})
        object_defs[name] = ObjectDefinition(
            marker_id=obj["marker_id"],
            is_static=(obj_type == "static"),
            is_goal=(obj_type == "goal"),
            width_cm=shape.get("width", 0.0),
            depth_cm=shape.get("depth", 0.0),
            height_cm=shape.get("height", 0.0),
            marker_offset_x_cm=offset.get("x", 0.0),
            marker_offset_y_cm=offset.get("y", 0.0),
        )
    return object_defs


def load_real_config(config_path: str, objects_path: str = "config/objects.yaml") -> Tuple:
    """Load configuration from YAML files for real robot."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

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

    # Load object definitions from objects.yaml
    object_defs = {}
    if Path(objects_path).exists():
        object_defs = load_objects_yaml(objects_path)

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
        object_defs=object_defs,  # From objects.yaml
        object_marker_size_mm=robot_cfg.get("object_marker_size_mm", 30.0),
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

    # Robot dimensions
    robot_width = robot_cfg.get("width_cm", 8.0)
    robot_height = robot_cfg.get("height_cm", 10.0)

    return camera_config, observer_config, real_env_config, robot_width, robot_height


def main():
    parser = argparse.ArgumentParser(description="Unified robot control test")
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Run in simulation mode"
    )
    parser.add_argument(
        "--config", "-c",
        type=str,
        default="config/real.yaml",
        help="Path to config file (for real robot mode)"
    )
    parser.add_argument(
        "--objects",
        type=str,
        default="config/objects.yaml",
        help="Path to objects definition file"
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
        help="Initial max speed (default: 0.3)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't send commands to robot (test camera/GUI only)"
    )
    args = parser.parse_args()

    # Validate mode
    if args.sim:
        if not SIM_AVAILABLE:
            print("Simulation mode not available (missing SimEnv)")
            return
        run_simulation(args)
    else:
        if not REAL_AVAILABLE:
            print("Real robot mode not available (missing camera/real env)")
            return
        run_real_robot(args)


def run_simulation(args):
    """Run in simulation mode."""
    # Create simulation config
    sim_config = SimConfig(
        car_width=8,
        car_height=10,
        x=10,
        y=10,
        theta=0,
        objects={
            "box1": (22, 32, 0),    # Center (5x5 cm from objects.yaml)
            "box2": (12, 45, 45),   # Upper left
        }
    )
    workspace_config = sim_config.workspace_config

    # Create simulation environment
    env = SimEnv(sim_config)

    # Create sensor node (publishes to WorldState)
    sensor = SimSensorNode(env, rate=30.0)
    world = WorldState()

    # Create planner and controllers
    planner = RVGPlanner(
        workspace_width=workspace_config.width,
        workspace_height=workspace_config.height,
        robot_width=workspace_config.car_width,
        robot_height=workspace_config.car_height,
    )

    controllers = {
        "keyboard": KeyboardController(max_speed=args.speed),
        "navigation": NavigationController(workspace_config, planner, max_speed=args.speed),
        "follow_path": FollowPathController(workspace_config, max_speed=args.speed),
    }

    # Create coordinator
    coordinator = ControlCoordinator(
        controllers=controllers,
        world=world,
        initial_controller="keyboard",
        initial_speed=args.speed,
    )

    # Create GUI
    window = Window(
        workspace_config,
        show_camera=False,
        show_connection=False,
        show_settings=True,
    )
    window.enable_canvas_click(True)
    window.set_controller("Keyboard")

    # Bind coordinator to window
    coordinator.bind_to_window(window)

    # State
    running = True

    def on_quit():
        nonlocal running
        running = False
        window.close_window()

    def on_emergency_stop():
        print("Emergency stop")

    coordinator.on_quit(on_quit)
    coordinator.on_emergency_stop(on_emergency_stop)

    # Start environment and sensor
    env.start()
    sensor.start()

    # Control loop
    control_rate = 30.0
    control_dt = 1.0 / control_rate

    def control_loop():
        while running:
            loop_start = time.time()

            obs = world.get()
            if obs is not None:
                # Get action from coordinator
                action = coordinator.step(obs)

                # Apply action
                env.apply(action)

                # Update GUI
                window.update(obs)
                window.set_status(coordinator.get_status())
                window.update_drawings(coordinator.get_drawings())
                window.update_action(action)

            # Maintain rate
            elapsed = time.time() - loop_start
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()

    print("\n" + "=" * 50)
    print("Robot Control - Simulation")
    print("=" * 50)
    print(f"Speed: {coordinator.current_speed}")
    print("=" * 50)
    print("Select controller from right panel")
    print("Press +/- to change speed")
    print("Press 'escape' to quit")
    print("=" * 50 + "\n")

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    sensor.stop()
    env.stop()
    world.unsubscribe()
    print("Done.")


def run_real_robot(args):
    """Run with real robot."""
    # Load config
    config_path = Path(args.config)
    objects_path = Path(args.objects)
    if config_path.exists():
        print(f"Loading config from {config_path}")
        if objects_path.exists():
            print(f"Loading objects from {objects_path}")
        camera_config, observer_config, real_env_config, robot_width, robot_height = load_real_config(
            str(config_path), str(objects_path)
        )
    else:
        print(f"Config not found: {config_path}, using defaults")
        camera_config = CameraConfig()
        observer_config = ObserverConfig()
        real_env_config = RealEnvConfig()
        robot_width = 8.0
        robot_height = 10.0

    # Override serial port if specified
    if args.port:
        real_env_config.port = args.port

    # Create workspace config
    workspace_config = make_real_workspace_config(
        car_width=robot_width,
        car_height=robot_height,
    )

    # Create planner and controllers
    planner = RVGPlanner(
        workspace_width=workspace_config.width,
        workspace_height=workspace_config.height,
        robot_width=workspace_config.car_width,
        robot_height=workspace_config.car_height,
    )

    controllers = {
        "keyboard": KeyboardController(max_speed=args.speed),
        "navigation": NavigationController(workspace_config, planner, max_speed=args.speed),
        "follow_path": FollowPathController(workspace_config, max_speed=args.speed),
    }

    # Create world state
    world = WorldState()

    # Create coordinator
    coordinator = ControlCoordinator(
        controllers=controllers,
        world=world,
        initial_controller="keyboard",
        initial_speed=args.speed,
    )

    # Create camera sensor node
    print("\nStarting CameraSensorNode...")
    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("Failed to start camera!")
        return

    # Create camera recorder (subscribes to raw camera frames)
    recorder = CameraRecorder(output_dir="recordings", fps=30.0)
    recorder.subscribe()

    # Create ArUco observer
    print("Starting ArucoObserver...")
    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("Failed to start observer!")
        recorder.unsubscribe()
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

    # Create GUI
    window = Window(
        workspace_config,
        show_camera=True,
        show_connection=not args.dry_run,
        show_settings=True,
        target_robot_id=real_env_config.robot_id if not args.dry_run else None,
    )
    window.enable_canvas_click(False)  # Start with keyboard mode
    window.set_controller("Keyboard")

    # Bind coordinator to window
    coordinator.bind_to_window(window)

    # State
    running = True

    def on_quit():
        nonlocal running
        running = False
        window.close_window()

    def on_emergency_stop():
        if not args.dry_run:
            real_env.stop_robot()
        print("Emergency stop")

    def on_record_toggled(recording: bool):
        """Handle record button toggle from GUI."""
        if recording:
            recorder.start()
        else:
            recorder.stop()

    def on_key_r():
        """Handle R key for recording toggle."""
        is_recording = recorder.toggle()
        window.set_recording(is_recording)

    coordinator.on_quit(on_quit)
    coordinator.on_emergency_stop(on_emergency_stop)

    # Register recording callback (separate from coordinator)
    window.register_callback("on_record_toggled", on_record_toggled)

    # Override key handler to add R key for recording
    original_key_handler = coordinator._on_key_press

    def extended_key_handler(key: str):
        if key.lower() == "r":
            on_key_r()
        else:
            original_key_handler(key)

    window.register_callback("on_key_press", extended_key_handler)

    # Control loop
    control_rate = 30.0
    control_dt = 1.0 / control_rate

    def control_loop():
        while running:
            loop_start = time.time()

            obs = world.get()
            if obs is not None:
                # Get action from coordinator
                action = coordinator.step(obs)

                if not args.dry_run:
                    real_env.apply(action)

                window.update(obs)
                window.set_status(coordinator.get_status())
                window.update_drawings(coordinator.get_drawings())
                window.update_action(action)

            # Update camera view
            vis_frame = observer.get_vis_frame()
            if vis_frame is not None:
                window.update_camera(vis_frame)

            # Update connection status
            if not args.dry_run:
                window.update_connection(real_env.get_status())

            elapsed = time.time() - loop_start
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()

    print("\n" + "=" * 50)
    print("Robot Control - Real Robot")
    print("=" * 50)
    print(f"Serial port: {real_env_config.port}")
    print(f"Robot ID: {real_env_config.robot_id}")
    print(f"Speed: {coordinator.current_speed}")
    print(f"Dry run: {args.dry_run}")
    print("=" * 50)
    print("Select controller from right panel")
    print("Press +/- to change speed")
    print("Press 'r' to toggle recording")
    print("Press 'space' for emergency stop")
    print("Press 'escape' to quit")
    print("=" * 50 + "\n")

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    recorder.unsubscribe()
    if not args.dry_run:
        real_env.stop()
    observer.stop()
    camera.stop()
    world.unsubscribe()
    print("Done.")


if __name__ == "__main__":
    main()
