"""Abstract interface for robot action sending.

This module defines the protocol (interface) that robot_control expects
for sending actions to real robots. Implementations can use different
backends (serial, UDP, mock, etc.).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Protocol, runtime_checkable


@dataclass
class SenderStatus:
    """Status information from the action sender."""

    is_connected: bool = False
    alive_robot_ids: List[int] = None
    recent_events: List[str] = None
    last_update_time: float = 0.0

    def __post_init__(self):
        if self.alive_robot_ids is None:
            self.alive_robot_ids = []
        if self.recent_events is None:
            self.recent_events = []


@runtime_checkable
class ActionSender(Protocol):
    """
    Protocol (interface) for sending actions to robots.

    robot_control defines this interface. Implementations can use
    different backends (micromvp SerialActionSender, UDP, mock, etc.).

    This is a Protocol, so any class with matching methods works -
    no inheritance required.
    """

    def start(self) -> bool:
        """Start the sender. Returns True if successful."""
        ...

    def stop(self) -> None:
        """Stop the sender and release resources."""
        ...

    def send(self, robot_id: int, left_speed: float, right_speed: float) -> None:
        """
        Send wheel speeds to a robot.

        Args:
            robot_id: Robot identifier
            left_speed: Left wheel speed in [-1, 1]
            right_speed: Right wheel speed in [-1, 1]
        """
        ...

    def send_immediate(self, robot_id: int, left_speed: float, right_speed: float) -> bool:
        """
        Send wheel speeds immediately (bypass rate limiting).
        Use for emergency stops.

        Returns:
            True if send succeeded
        """
        ...

    def stop_all(self) -> None:
        """Send stop command to all robots."""
        ...

    def get_status(self) -> SenderStatus:
        """Get current status (connected robots, events, etc.)."""
        ...
