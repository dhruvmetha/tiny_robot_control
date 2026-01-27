"""Test click-to-navigate with obstacle avoidance.

Usage:
    # With RVG library available:
    PYTHONPATH=/home/omen/dhruv/namo/micromvp_push_dhruv/rvg:$PYTHONPATH \
    python scripts/test_navigation.py

Controls:
    Click   : Navigate to clicked position (plans around obstacles)
    R       : Toggle rotation mode (face travel direction at goal)
    C       : Cancel navigation
    Space   : Emergency stop
    Escape  : Quit
"""

import argparse
import math
import threading
import time

from robot_control import SimEnv, SimConfig
from robot_control.controller import NavigationController
from robot_control.core.world_state import WorldState
from robot_control.nodes.sim_sensor import SimSensorNode
from robot_control.planner import RVGPlanner
from robot_control.gui import Window


def main():
    parser = argparse.ArgumentParser(description="Test navigation controller")
    parser.add_argument(
        "--speed", "-s",
        type=float,
        default=0.4,
        help="Maximum speed (default: 0.4)"
    )
    args = parser.parse_args()

    # Environment with obstacles
    # Object names must match objects.yaml - dimensions loaded automatically
    # Workspace is 45x65 cm (matching real robot)
    config = SimConfig(
        x=10, y=10, theta=0,
        objects={
            "box1": (22, 32, 0),    # Center obstacle (5x5 cm from objects.yaml)
            "box2": (12, 45, 30),   # Upper left
            "box3": (35, 25, -15),  # Right side
        }
    )

    # Environment
    env = SimEnv(config)

    # Sensor node publishes observations
    sensor = SimSensorNode(env, rate=30.0)

    # WorldState aggregates sensor data
    world = WorldState()

    # GUI (with settings panel for rotate button)
    window = Window(config.workspace_config, show_settings=True)
    window.enable_canvas_click(True)

    # Navigation controller with RVG planner
    workspace_config = config.workspace_config
    planner = RVGPlanner(
        workspace_width=workspace_config.width,
        workspace_height=workspace_config.height,
        robot_width=workspace_config.car_width,
        robot_height=workspace_config.car_height,
        robot_geometry_scale=1.27,  # Matches wavefront (3.5 cm effective radius)
    )
    controller = NavigationController(workspace_config, planner, max_speed=args.speed)

    running = True
    rotation_mode = False  # When True, robot faces travel direction at goal

    def on_canvas_click(x: float, y: float):
        """Handle canvas click - navigate to position."""
        nonlocal rotation_mode

        obs = world.get()
        if obs is None:
            print("No observation available")
            return

        # Build obstacle list from observed objects
        # Format: (x, y, theta_deg, width, depth)
        # Dimensions come from objects.yaml via SimEnv
        obstacles = [
            (o.x, o.y, o.theta, o.width, o.depth)
            for o in obs.objects.values()
            if o.width > 0 and o.depth > 0  # Skip objects without dimensions
        ]

        current_pos = (obs.robot_x, obs.robot_y)

        # Calculate goal theta if rotation mode is enabled
        goal_theta = None
        if rotation_mode:
            # Face the direction of travel (from current pos to goal)
            dx = x - obs.robot_x
            dy = y - obs.robot_y
            if math.hypot(dx, dy) > 1.0:  # Only if not clicking on robot
                goal_theta = math.degrees(math.atan2(dy, dx))
                # Normalize to [0, 360)
                if goal_theta < 0:
                    goal_theta += 360

        # Navigate to clicked position
        if controller.navigate_to(x, y, goal_theta, current_pos, obstacles):
            if goal_theta is not None:
                print(f"Navigating to ({x:.0f}, {y:.0f}) facing {goal_theta:.0f}°")
            else:
                print(f"Navigating to ({x:.0f}, {y:.0f})")
        else:
            print(f"Planning failed to ({x:.0f}, {y:.0f})")

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

    def on_rotation_goal(theta: float):
        """Handle rotation button click - rotate in place."""
        controller.rotate_to(theta)
        print(f"Rotating to {theta:.0f}°")

    window.register_callback("on_canvas_click", on_canvas_click)
    window.register_callback("on_key_press", on_key_press)
    window.register_callback("on_rotation_goal", on_rotation_goal)
    window.set_controller("Navigation")

    # Start systems
    env.start()
    sensor.start()

    # Control parameters
    control_rate = 30.0  # Hz
    control_dt = 1.0 / control_rate
    max_obs_age = 0.2  # Stop if observation older than 200ms

    def control_loop():
        while running:
            loop_start = time.time()

            # Get latest observation (non-blocking)
            obs = world.get()

            if obs is not None:
                # Check observation freshness
                obs_age = time.time() - obs.timestamp
                if obs_age < max_obs_age:
                    # Fresh observation - compute new action
                    action = controller.step(obs, subgoal=None)
                else:
                    # Stale observation - keep last action
                    action = controller.step(obs, subgoal=None)

                env.apply(action)

                # Update GUI
                window.update(obs)
                window.set_status(controller.get_status())
                window.update_drawings(controller.get_drawings())

            # Maintain control rate
            elapsed = time.time() - loop_start
            sleep_time = control_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()

    # Wait for first observation
    obs = world.wait_for_update(timeout=1.0)
    if obs:
        window.update(obs)

    print("\n" + "=" * 50)
    print("Navigation Controller (RVG Path Planning)")
    print("=" * 50)
    print("Click  : Navigate to position (plans around obstacles)")
    print("C      : Cancel navigation")
    print("Space  : Emergency stop")
    print("Escape : Quit")
    print("=" * 50 + "\n")

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    sensor.stop()
    env.stop()
    world.unsubscribe()


if __name__ == "__main__":
    main()
