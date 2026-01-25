"""Real environment for robot control.

This module provides RealEnv, which:
1. Uses the ActionSender interface (defined by robot_control)
2. Defaults to MicromvpAdapter (which wraps micromvp's SerialActionSender)

The actual serial protocol code lives in micromvp and is NOT duplicated here.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

from pubsub import pub

from robot_control.core.topics import Topics
from robot_control.core.types import Action
from robot_control.environment.action_sender import ActionSender, SenderStatus

# Import adapter (which imports from micromvp)
try:
    from robot_control.environment.micromvp_adapter import (
        MicromvpAdapter,
        MicromvpAdapterConfig,
        MICROMVP_AVAILABLE,
    )
except ImportError:
    MICROMVP_AVAILABLE = False
    MicromvpAdapter = None
    MicromvpAdapterConfig = None


@dataclass
class RealEnvConfig:
    """Configuration for real robot environment."""

    # Serial port settings
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200

    # Robot ID (single robot)
    robot_id: int = 1

    # Send frequency
    send_hz: float = 30.0

    # Motor polarity (some robots have reversed wiring)
    invert_right_wheel: bool = False

    # Status publishing (Hz). Set to 0 to disable.
    status_publish_hz: float = 2.0


class RealEnv:
    """
    Real robot environment.

    Uses the ActionSender interface to communicate with robots.
    By default, uses MicromvpAdapter which wraps micromvp's
    battle-tested SerialActionSender.

    You can inject a different ActionSender implementation for testing
    or alternative backends.

    Usage:
        # Default: uses MicromvpAdapter
        env = RealEnv(RealEnvConfig(robot_id=1))
        env.start()
        env.apply(Action(left_speed=0.5, right_speed=0.5))
        env.stop()

        # Custom sender (for testing or different backend)
        env = RealEnv(config, sender=MockActionSender())
    """

    def __init__(
        self,
        config: RealEnvConfig,
        sender: Optional[ActionSender] = None,
    ) -> None:
        """
        Initialize real environment.

        Args:
            config: Environment configuration
            sender: Optional custom ActionSender. If None, creates MicromvpAdapter.
        """
        self._config = config
        self._sender = sender
        self._owns_sender = sender is None  # Did we create the sender?
        self._running = False

        # Status publishing
        self._status_thread: Optional[threading.Thread] = None
        self._status_running = False

    def start(self) -> bool:
        """Start the environment."""
        if self._running:
            return True

        # Create default sender if not provided
        if self._sender is None:
            if not MICROMVP_AVAILABLE:
                raise ImportError(
                    "micromvp not available. Ensure conda environment is activated.\n"
                    "Try: conda deactivate && conda activate namo-cpp"
                )

            adapter_config = MicromvpAdapterConfig(
                port=self._config.port,
                baudrate=self._config.baudrate,
                send_hz=self._config.send_hz,
                invert_right_wheel=self._config.invert_right_wheel,
                robot_ids=[self._config.robot_id],
            )
            self._sender = MicromvpAdapter(adapter_config)
            self._owns_sender = True

        if not self._sender.start():
            print("[RealEnv] Failed to start sender")
            return False

        self._running = True

        # Start status publishing thread
        if self._config.status_publish_hz > 0:
            self._status_running = True
            self._status_thread = threading.Thread(
                target=self._status_loop,
                daemon=True,
                name="RealEnv-StatusPublisher",
            )
            self._status_thread.start()

        print(f"[RealEnv] Started (robot_id={self._config.robot_id})")
        return True

    def stop(self) -> None:
        """Stop the environment."""
        if not self._running:
            return

        self._running = False

        # Stop status thread
        self._status_running = False
        if self._status_thread is not None:
            self._status_thread.join(timeout=1.0)
            self._status_thread = None

        # Only stop sender if we created it
        if self._sender is not None and self._owns_sender:
            self._sender.stop()
            self._sender = None

        print("[RealEnv] Stopped")

    def apply(self, action: Action) -> None:
        """
        Send action to robot.

        Args:
            action: Differential drive action with wheel speeds in [-1, 1]
        """
        if not self._running or self._sender is None:
            return

        self._sender.send(
            self._config.robot_id,
            action.left_speed,
            action.right_speed,
        )

    def apply_immediate(self, action: Action) -> bool:
        """
        Send action immediately (bypasses rate limiting).
        Use for emergency stops.

        Args:
            action: Differential drive action

        Returns:
            True if send succeeded
        """
        if not self._running or self._sender is None:
            return False

        return self._sender.send_immediate(
            self._config.robot_id,
            action.left_speed,
            action.right_speed,
        )

    def stop_robot(self) -> None:
        """Send immediate stop command to robot."""
        if self._sender is not None:
            self._sender.stop_all()

    def get_status(self) -> SenderStatus:
        """Get sender status (connected robots, events, etc.)."""
        if self._sender is None:
            return SenderStatus(is_connected=False)
        return self._sender.get_status()

    @property
    def is_running(self) -> bool:
        """Check if environment is running."""
        return self._running

    @property
    def robot_id(self) -> int:
        """Get the robot ID."""
        return self._config.robot_id

    def _status_loop(self) -> None:
        """Background thread that publishes status updates."""
        interval = 1.0 / self._config.status_publish_hz
        while self._status_running:
            loop_start = time.time()

            status = self.get_status()
            pub.sendMessage(Topics.ROBOT_STATUS, status=status)

            # Maintain publish rate
            elapsed = time.time() - loop_start
            if elapsed < interval:
                time.sleep(interval - elapsed)


__all__ = ["RealEnv", "RealEnvConfig"]
