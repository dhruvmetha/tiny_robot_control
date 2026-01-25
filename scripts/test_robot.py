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
import math
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import yaml

from robot_control.controller import (
    KeyboardController,
    NavigationController,
    FollowPathController,
)
from robot_control.core.types import Action, Observation, WorkspaceConfig
from robot_control.core.world_state import WorldState
from robot_control.gui import Window
from robot_control.planner import RVGPlanner

# Conditional imports for real robot
try:
    from robot_control.camera import (
        ArucoObserver,
        ObserverConfig,
        WORKSPACE_WIDTH_CM,
        WORKSPACE_HEIGHT_CM,
        make_real_workspace_config,
    )
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


# Speed presets (max 0.9 for safety)
SPEED_PRESETS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 0.9]


def load_real_config(config_path: str) -> Tuple:
    """Load configuration from YAML file for real robot."""
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
        width=640,
        height=480,
        car_width=30,
        car_height=40,
        x=320,
        y=240,
        theta=0,
        objects={
            "box1": (450, 300, 0),
            "box2": (200, 350, 45),
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
    active_controller = "keyboard"
    current_speed = args.speed

    # Create GUI
    window = Window(
        workspace_config,
        show_camera=False,
        show_connection=False,
        show_settings=True,
    )
    window.enable_canvas_click(True)
    window.set_controller("Keyboard")
    window.update_speed(current_speed)

    # State
    running = True
    pressed_keys: Set[str] = set()
    rotation_mode = False

    def on_controller_changed(controller_id: str):
        nonlocal active_controller

        # Reset all controllers when switching (clears paths, goals, etc.)
        for ctrl in controllers.values():
            if hasattr(ctrl, "cancel"):
                ctrl.cancel()

        # Clear drawings
        window.update_drawings([])

        active_controller = controller_id

        # Update UI based on controller type
        if controller_id == "keyboard":
            window.set_controller("Keyboard")
            window.enable_canvas_click(False)
            window.enable_draw(False)
            print("Switched to Keyboard mode (WASD)")
        elif controller_id == "navigation":
            window.set_controller("Navigation")
            window.enable_canvas_click(True)
            window.enable_draw(False)
            print("Switched to Navigation mode (click to navigate)")
        elif controller_id == "follow_path":
            window.set_controller("Follow Path")
            window.enable_canvas_click(False)
            window.enable_draw(True)
            print("Switched to Follow Path mode (draw path)")

    def on_canvas_click(x: float, y: float):
        if active_controller != "navigation":
            return

        obs = world.get()
        if obs is None:
            print("No observation available")
            return

        # Build obstacle list
        obstacles = [
            (o.x, o.y, o.theta, o.width if o.width > 0 else 4.0, o.height if o.height > 0 else 4.0)
            for o in obs.objects.values()
        ]

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
        nav = controllers["navigation"]
        if nav.navigate_to(x, y, goal_theta, (obs.robot_x, obs.robot_y), obstacles):
            print(f"Navigating to ({x:.1f}, {y:.1f})")
        else:
            print(f"Planning failed to ({x:.1f}, {y:.1f})")

    def on_curve_drawn(points: List[Tuple[float, float]]):
        if active_controller != "follow_path":
            return

        if len(points) < 2:
            print("Path too short")
            return

        controllers["follow_path"].set_path(points)
        print(f"Following path with {len(points)} points")

    def on_navigation_goal(x: float, y: float, theta):
        """Handle navigation goal from UI."""
        # Switch to navigation mode if not already
        nonlocal active_controller
        if active_controller != "navigation":
            active_controller = "navigation"
            window.set_controller("Navigation")
            window.enable_canvas_click(True)
            window.enable_draw(False)

        # Cancel current plan before starting new one
        controllers["navigation"].cancel()
        window.update_drawings([])

        obs = world.get()
        if obs is None:
            print("No observation available")
            return

        # Build obstacle list
        obstacles = [
            (o.x, o.y, o.theta, o.width if o.width > 0 else 4.0, o.height if o.height > 0 else 4.0)
            for o in obs.objects.values()
        ]

        # Navigate with provided theta (can be None)
        nav = controllers["navigation"]
        theta_str = f", θ={theta:.1f}°" if theta is not None else ""
        if nav.navigate_to(x, y, theta, (obs.robot_x, obs.robot_y), obstacles):
            print(f"Navigating to ({x:.1f}, {y:.1f}{theta_str})")
        else:
            print(f"Planning failed to ({x:.1f}, {y:.1f})")

    def on_cancel():
        """Handle cancel button - stop robot and clear current plan."""
        ctrl = controllers[active_controller]
        if hasattr(ctrl, "cancel"):
            ctrl.cancel()
        window.update_drawings([])
        print("Cancelled")

    def change_speed(direction: int):
        """Change speed by direction (-1 = decrease, +1 = increase)."""
        nonlocal current_speed
        if direction > 0:
            # Increase speed
            idx = 0
            for i, preset in enumerate(SPEED_PRESETS):
                if current_speed <= preset:
                    idx = min(i + 1, len(SPEED_PRESETS) - 1)
                    break
            current_speed = SPEED_PRESETS[idx]
        else:
            # Decrease speed
            idx = len(SPEED_PRESETS) - 1
            for i, preset in enumerate(SPEED_PRESETS):
                if current_speed <= preset:
                    idx = max(i - 1, 0)
                    break
            current_speed = SPEED_PRESETS[idx]

        for ctrl in controllers.values():
            if hasattr(ctrl, "set_speed"):
                ctrl.set_speed(current_speed)
            elif hasattr(ctrl, "max_speed"):
                ctrl.max_speed = current_speed
        window.update_speed(current_speed)
        print(f"Speed: {current_speed}")

    def on_speed_change(direction: int):
        """Handle speed change from UI buttons."""
        change_speed(direction)

    def on_key_press(key: str):
        nonlocal running, rotation_mode
        key = key.lower()

        if key == "escape":
            running = False
            window.close_window()
        elif key == "r":
            rotation_mode = not rotation_mode
            mode_str = "ON" if rotation_mode else "OFF"
            print(f"Rotation mode: {mode_str}")
        elif key == "c":
            # Cancel current action
            if active_controller == "navigation":
                controllers["navigation"].cancel()
            elif active_controller == "follow_path":
                controllers["follow_path"].cancel()
            print("Cancelled")
        elif key == "space":
            # Emergency stop
            for ctrl in controllers.values():
                if hasattr(ctrl, "cancel"):
                    ctrl.cancel()
            print("Emergency stop")
        elif key in ("=", "+"):
            change_speed(+1)
        elif key in ("-", "_"):
            change_speed(-1)
        else:
            pressed_keys.add(key)
            if active_controller == "keyboard":
                controllers["keyboard"].set_keys(pressed_keys)

    def on_key_release(key: str):
        key = key.lower()
        pressed_keys.discard(key)
        if active_controller == "keyboard":
            controllers["keyboard"].set_keys(pressed_keys)

    window.register_callback("on_controller_changed", on_controller_changed)
    window.register_callback("on_canvas_click", on_canvas_click)
    window.register_callback("on_curve_drawn", on_curve_drawn)
    window.register_callback("on_key_press", on_key_press)
    window.register_callback("on_key_release", on_key_release)
    window.register_callback("on_speed_change", on_speed_change)
    window.register_callback("on_navigation_goal", on_navigation_goal)
    window.register_callback("on_cancel", on_cancel)

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
                # Get action from active controller
                ctrl = controllers[active_controller]
                action = ctrl.step(obs, subgoal=None)

                # Apply action
                env.apply(action)

                # Update GUI
                window.update(obs)
                if hasattr(ctrl, "get_status"):
                    window.set_status(ctrl.get_status())
                if hasattr(ctrl, "get_drawings"):
                    window.update_drawings(ctrl.get_drawings())
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
    print(f"Speed: {current_speed}")
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
    if config_path.exists():
        print(f"Loading config from {config_path}")
        camera_config, observer_config, real_env_config, robot_width, robot_height = load_real_config(str(config_path))
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
    active_controller = "keyboard"
    current_speed = args.speed

    # Create world state
    world = WorldState()

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
    window.update_speed(current_speed)

    # State
    running = True
    pressed_keys: Set[str] = set()
    rotation_mode = False

    def on_controller_changed(controller_id: str):
        nonlocal active_controller

        # Reset all controllers when switching (clears paths, goals, etc.)
        for ctrl in controllers.values():
            if hasattr(ctrl, "cancel"):
                ctrl.cancel()

        # Clear drawings
        window.update_drawings([])

        active_controller = controller_id

        if controller_id == "keyboard":
            window.set_controller("Keyboard")
            window.enable_canvas_click(False)
            window.enable_draw(False)
            print("Switched to Keyboard mode (WASD)")
        elif controller_id == "navigation":
            window.set_controller("Navigation")
            window.enable_canvas_click(True)
            window.enable_draw(False)
            print("Switched to Navigation mode (click to navigate)")
        elif controller_id == "follow_path":
            window.set_controller("Follow Path")
            window.enable_canvas_click(False)
            window.enable_draw(True)
            print("Switched to Follow Path mode (draw path)")

    def on_canvas_click(x: float, y: float):
        if active_controller != "navigation":
            return

        obs = world.get()
        if obs is None:
            print("No observation available")
            return

        obstacles = [
            (o.x, o.y, o.theta, o.width if o.width > 0 else 4.0, o.height if o.height > 0 else 4.0)
            for o in obs.objects.values()
        ]

        goal_theta = None
        if rotation_mode:
            dx = x - obs.robot_x
            dy = y - obs.robot_y
            if math.hypot(dx, dy) > 1.0:
                goal_theta = math.degrees(math.atan2(dy, dx))
                if goal_theta < 0:
                    goal_theta += 360

        nav = controllers["navigation"]
        if nav.navigate_to(x, y, goal_theta, (obs.robot_x, obs.robot_y), obstacles):
            print(f"Navigating to ({x:.1f}, {y:.1f})")
        else:
            print(f"Planning failed to ({x:.1f}, {y:.1f})")

    def on_curve_drawn(points: List[Tuple[float, float]]):
        if active_controller != "follow_path":
            return

        if len(points) < 2:
            print("Path too short")
            return

        controllers["follow_path"].set_path(points)
        print(f"Following path with {len(points)} points")

    def on_navigation_goal(x: float, y: float, theta):
        """Handle navigation goal from UI."""
        # Switch to navigation mode if not already
        nonlocal active_controller
        if active_controller != "navigation":
            active_controller = "navigation"
            window.set_controller("Navigation")
            window.enable_canvas_click(True)
            window.enable_draw(False)

        # Cancel current plan before starting new one
        controllers["navigation"].cancel()
        window.update_drawings([])

        obs = world.get()
        if obs is None:
            print("No observation available")
            return

        # Build obstacle list
        obstacles = [
            (o.x, o.y, o.theta, o.width if o.width > 0 else 4.0, o.height if o.height > 0 else 4.0)
            for o in obs.objects.values()
        ]

        # Navigate with provided theta (can be None)
        nav = controllers["navigation"]
        theta_str = f", θ={theta:.1f}°" if theta is not None else ""
        if nav.navigate_to(x, y, theta, (obs.robot_x, obs.robot_y), obstacles):
            print(f"Navigating to ({x:.1f}, {y:.1f}{theta_str})")
        else:
            print(f"Planning failed to ({x:.1f}, {y:.1f})")

    def on_cancel():
        """Handle cancel button - stop robot and clear current plan."""
        ctrl = controllers[active_controller]
        if hasattr(ctrl, "cancel"):
            ctrl.cancel()
        window.update_drawings([])
        print("Cancelled")

    def on_record_toggled(recording: bool):
        """Handle record button toggle from GUI."""
        if recording:
            recorder.start()
        else:
            recorder.stop()

    def change_speed(direction: int):
        """Change speed by direction (-1 = decrease, +1 = increase)."""
        nonlocal current_speed
        if direction > 0:
            # Increase speed
            idx = 0
            for i, preset in enumerate(SPEED_PRESETS):
                if current_speed <= preset:
                    idx = min(i + 1, len(SPEED_PRESETS) - 1)
                    break
            current_speed = SPEED_PRESETS[idx]
        else:
            # Decrease speed
            idx = len(SPEED_PRESETS) - 1
            for i, preset in enumerate(SPEED_PRESETS):
                if current_speed <= preset:
                    idx = max(i - 1, 0)
                    break
            current_speed = SPEED_PRESETS[idx]

        for ctrl in controllers.values():
            if hasattr(ctrl, "set_speed"):
                ctrl.set_speed(current_speed)
            elif hasattr(ctrl, "max_speed"):
                ctrl.max_speed = current_speed
        window.update_speed(current_speed)
        print(f"Speed: {current_speed}")

    def on_speed_change(direction: int):
        """Handle speed change from UI buttons."""
        change_speed(direction)

    def on_key_press(key: str):
        nonlocal running, rotation_mode
        key = key.lower()

        if key == "escape":
            running = False
            window.close_window()
        elif key == "r":
            # Toggle recording
            is_recording = recorder.toggle()
            window.set_recording(is_recording)
        elif key == "c":
            if active_controller == "navigation":
                controllers["navigation"].cancel()
            elif active_controller == "follow_path":
                controllers["follow_path"].cancel()
            print("Cancelled")
        elif key == "space":
            for ctrl in controllers.values():
                if hasattr(ctrl, "cancel"):
                    ctrl.cancel()
            if not args.dry_run:
                real_env.stop_robot()
            print("Emergency stop")
        elif key in ("=", "+"):
            change_speed(+1)
        elif key in ("-", "_"):
            change_speed(-1)
        else:
            pressed_keys.add(key)
            if active_controller == "keyboard":
                controllers["keyboard"].set_keys(pressed_keys)

    def on_key_release(key: str):
        key = key.lower()
        pressed_keys.discard(key)
        if active_controller == "keyboard":
            controllers["keyboard"].set_keys(pressed_keys)

    window.register_callback("on_controller_changed", on_controller_changed)
    window.register_callback("on_canvas_click", on_canvas_click)
    window.register_callback("on_curve_drawn", on_curve_drawn)
    window.register_callback("on_key_press", on_key_press)
    window.register_callback("on_key_release", on_key_release)
    window.register_callback("on_speed_change", on_speed_change)
    window.register_callback("on_navigation_goal", on_navigation_goal)
    window.register_callback("on_cancel", on_cancel)
    window.register_callback("on_record_toggled", on_record_toggled)

    # Control loop
    control_rate = 30.0
    control_dt = 1.0 / control_rate

    def control_loop():
        while running:
            loop_start = time.time()

            obs = world.get()
            if obs is not None:
                ctrl = controllers[active_controller]
                action = ctrl.step(obs, subgoal=None)

                if not args.dry_run:
                    real_env.apply(action)

                window.update(obs)
                if hasattr(ctrl, "get_status"):
                    window.set_status(ctrl.get_status())
                if hasattr(ctrl, "get_drawings"):
                    window.update_drawings(ctrl.get_drawings())
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
    print(f"Speed: {current_speed}")
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
