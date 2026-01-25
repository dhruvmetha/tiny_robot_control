"""Test pub/sub architecture with simulation."""

import threading

from pubsub import pub

from robot_control import SimConfig, SimEnv
from robot_control.controller import KeyboardController
from robot_control.core import Topics, WorldState
from robot_control.gui import Window
from robot_control.nodes import SimSensorNode


def main():
    # Create environment
    config = SimConfig(
        x=320,
        y=240,
        theta=0,
        objects={"box1": (450, 300, 0), "box2": (200, 350, 45)},
    )
    env = SimEnv(config)

    # Create pub/sub nodes
    sensor_node = SimSensorNode(env, rate=30.0)
    world = WorldState()

    # Create window and controller
    window = Window(config)
    controller = KeyboardController(max_speed=1.0)

    # GUI subscribes to world state topic
    # Note: Must use named function, not lambda - pypubsub uses weak refs
    def on_world_state(obs):
        window.update(obs)
    pub.subscribe(on_world_state, Topics.WORLD_STATE)

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
    window.set_controller("Keyboard (PubSub)")

    # Start physics and sensor node
    env.start()
    sensor_node.start()

    # Sequential control loop - blocks until new observation
    running = True

    def control_loop():
        while running:
            obs = world.wait_for_update(timeout=0.1)
            if obs:
                action = controller.step(obs, subgoal=None)
                env.apply(action)
                window.set_status(controller.get_status())

    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()

    # Initial update
    window.update(env.observe())
    window.set_status(controller.get_status())

    # Run GUI (blocks)
    window.run()

    # Cleanup
    running = False
    sensor_node.stop()
    env.stop()
    world.unsubscribe()


if __name__ == "__main__":
    main()
