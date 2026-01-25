"""Central state aggregator with pub/sub."""

from __future__ import annotations

from threading import Event, Lock
from typing import Optional

from pubsub import pub

from robot_control.core.topics import Topics
from robot_control.core.types import Observation


class WorldState:
    """
    Central state aggregator with pub/sub.

    Subscribes to sensor topics, caches the latest observation,
    and provides both blocking and non-blocking access.

    Usage:
        world = WorldState()

        # Non-blocking
        obs = world.get()

        # Blocking (for control loop)
        obs = world.wait_for_update(timeout=1.0)
    """

    def __init__(self) -> None:
        self._obs: Optional[Observation] = None
        self._lock = Lock()
        self._update_event = Event()

        # Subscribe to all sensor sources
        pub.subscribe(self._on_sensor, Topics.SENSOR_SIM)
        pub.subscribe(self._on_sensor, Topics.SENSOR_VISION)

    def _on_sensor(self, obs: Observation) -> None:
        """Handle incoming sensor observation."""
        with self._lock:
            self._obs = obs
        self._update_event.set()
        # Republish for async subscribers (GUI)
        pub.sendMessage(Topics.WORLD_STATE, obs=obs)

    def get(self) -> Optional[Observation]:
        """Non-blocking: get latest observation."""
        with self._lock:
            return self._obs

    def wait_for_update(self, timeout: Optional[float] = None) -> Optional[Observation]:
        """
        Blocking: wait for next observation.

        Args:
            timeout: Max seconds to wait. None = wait forever.

        Returns:
            Latest observation, or None if timeout expired.
        """
        self._update_event.clear()
        if self._update_event.wait(timeout):
            with self._lock:
                return self._obs
        return None

    def unsubscribe(self) -> None:
        """Unsubscribe from all topics. Call on shutdown."""
        pub.unsubscribe(self._on_sensor, Topics.SENSOR_SIM)
        pub.unsubscribe(self._on_sensor, Topics.SENSOR_VISION)
