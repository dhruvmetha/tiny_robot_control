"""Test robot control with different controllers.

Usage:
    python scripts/test_control.py --controller keyboard
    python scripts/test_control.py --controller follow_path

Controls (Keyboard mode):
    WASD    : Drive robot
    Escape  : Quit

Controls (FollowPath mode):
    Draw    : Robot follows the drawn path
    C       : Clear path
    Space   : Stop
    Escape  : Quit
"""

import argparse
import threading
import time

from robot_control import SimEnv, SimConfig
from robot_control.controller import KeyboardController, FollowPathController
from robot_control.core.world_state import WorldState
from robot_control.nodes.sim_sensor import SimSensorNode
from robot_control.gui import Window


def run_keyboard(config: SimConfig):
    """Run with keyboard controller."""
    # Environment
    env = SimEnv(config)

    # Sensor node publishes observations
    sensor = SimSensorNode(env, rate=30.0)

    # WorldState aggregates sensor data
    world = WorldState()

    # GUI and controller
    window = Window(config.workspace_config)
    controller = KeyboardController(max_speed=1.0)

    pressed_keys = set()
    running = True

    def on_key_press(key):
        nonlocal running
        if key == "escape":
            running = False
            window.close_window()
        else:
            pressed_keys.add(key)
            controller.set_keys(pressed_keys)

    def on_key_release(key):
        pressed_keys.discard(key)
        controller.set_keys(pressed_keys)

    window.register_callback("on_key_press", on_key_press)
    window.register_callback("on_key_release", on_key_release)
    window.set_controller("Keyboard")

    # Start systems
    env.start()
    sensor.start()

    # Control parameters
    control_rate = 30.0  # Hz
    control_dt = 1.0 / control_rate

    def control_loop():
        while running:
            loop_start = time.time()

            # Get latest observation (non-blocking)
            obs = world.get()

            if obs is not None:
                # Compute and apply action
                action = controller.step(obs, subgoal=None)
                env.apply(action)

                # Update GUI
                window.update(obs)
                window.set_status(controller.get_status())

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

    print("\n" + "=" * 40)
    print("Keyboard Controller")
    print("=" * 40)
    print("WASD   : Drive robot")
    print("Escape : Quit")
    print("=" * 40 + "\n")

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    sensor.stop()
    env.stop()
    world.unsubscribe()


def run_follow_path(config: SimConfig):
    """Run with follow path controller."""
    # Environment
    env = SimEnv(config)

    # Sensor node publishes observations
    sensor = SimSensorNode(env, rate=30.0)

    # WorldState aggregates sensor data
    world = WorldState()

    # GUI and controller
    window = Window(config.workspace_config)
    controller = FollowPathController(config.workspace_config, max_speed=0.4)

    window.enable_draw(True)
    running = True

    def on_curve_drawn(points):
        print(f"Path drawn with {len(points)} points")
        controller.set_path(points)

    def on_key_press(key):
        nonlocal running
        if key == "escape":
            running = False
            window.close_window()
        elif key == "c":
            controller.clear_path()
            print("Path cleared")
        elif key == "space":
            controller.clear_path()
            print("Stopped")

    window.register_callback("on_curve_drawn", on_curve_drawn)
    window.register_callback("on_key_press", on_key_press)
    window.set_controller("FollowPath")

    # Start systems
    env.start()
    sensor.start()

    # Control parameters
    control_rate = 30.0  # Hz
    control_dt = 1.0 / control_rate
    max_obs_age = 0.2  # Stop if observation older than 200ms

    def control_loop():
        last_action = None

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
                    last_action = action
                else:
                    # Stale observation - stop for safety
                    action = last_action if last_action else controller.step(obs, subgoal=None)

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

    print("\n" + "=" * 40)
    print("FollowPath Controller (PP + CTE-PD)")
    print("=" * 40)
    print("Draw   : Robot follows the path")
    print("C      : Clear path")
    print("Space  : Stop")
    print("Escape : Quit")
    print("=" * 40 + "\n")

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    sensor.stop()
    env.stop()
    world.unsubscribe()


def main():
    parser = argparse.ArgumentParser(description="Test robot control")
    parser.add_argument(
        "--controller", "-c",
        choices=["keyboard", "follow_path"],
        default="keyboard",
        help="Controller to use (default: keyboard)"
    )
    args = parser.parse_args()

    config = SimConfig(
        x=320, y=240, theta=0,
        objects={"box1": (450, 300, 0), "box2": (200, 350, 45)}
    )

    if args.controller == "keyboard":
        run_keyboard(config)
    else:
        run_follow_path(config)


if __name__ == "__main__":
    main()
