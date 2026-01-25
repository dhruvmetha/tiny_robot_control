"""Simulation sensor node - publishes observations at fixed rate."""

from __future__ import annotations

import time
from threading import Thread
from typing import TYPE_CHECKING, Optional

from pubsub import pub

from robot_control.core.topics import Topics

if TYPE_CHECKING:
    from robot_control.environment.sim import SimEnv


class SimSensorNode:
    """
    Fake camera node - publishes sim observations at fixed rate.

    Mirrors real robot where sensors publish independently.
    The control loop subscribes via WorldState.

    Usage:
        sensor = SimSensorNode(env, rate=30.0)
        sensor.start()
        # ... run control loop ...
        sensor.stop()
    """

    def __init__(self, env: SimEnv, rate: float = 30.0) -> None:
        """
        Args:
            env: Simulation environment to poll.
            rate: Publishing rate in Hz.
        """
        self._env = env
        self._interval = 1.0 / rate
        self._running = False
        self._thread: Optional[Thread] = None

    def start(self) -> None:
        """Start the sensor node (spawns background thread)."""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the sensor node."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self) -> None:
        """Main loop: poll env and publish observations."""
        while self._running:
            obs = self._env.observe()
            pub.sendMessage(Topics.SENSOR_SIM, obs=obs)
            time.sleep(self._interval)

    @property
    def is_running(self) -> bool:
        """Check if sensor node is running."""
        return self._running
