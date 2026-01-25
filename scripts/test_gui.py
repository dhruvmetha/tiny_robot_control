"""Test GUI with keyboard control."""

import threading
import time

from robot_control import SimEnv, SimConfig
from robot_control.controller import KeyboardController
from robot_control.gui import Window


def main():
    # Create environment with micromvp-matching defaults
    # Default workspace: 640x480, robot: 36x52, wheel_base: 30, max_speed: 100
    config = SimConfig(
        x=320, y=240, theta=0,  # Start in center
        objects={"box1": (450, 300, 0), "box2": (200, 350, 45)}
    )
    env = SimEnv(config)

    # Create window and controller
    window = Window(config.workspace_config)
    controller = KeyboardController(max_speed=1.0)

    # Track pressed keys
    pressed = set()

    def on_key_press(key):
        pressed.add(key)
        controller.set_keys(pressed)

    def on_key_release(key):
        pressed.discard(key)
        controller.set_keys(pressed)

    window.register_callback("on_key_press", on_key_press)
    window.register_callback("on_key_release", on_key_release)

    # Set controller name in sidebar
    window.set_controller("Keyboard")

    # Start physics
    env.start()

    # Control loop in separate thread
    running = True

    def control_loop():
        while running:
            obs = env.observe()
            action = controller.step(obs, subgoal=None)
            env.apply(action)
            window.update(obs)
            window.set_status(controller.get_status())
            time.sleep(1/30)

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()

    # Initial update
    window.update(env.observe())
    window.set_status(controller.get_status())

    # Run GUI (blocks)
    window.run()
    running = False
    env.stop()


if __name__ == "__main__":
    main()
