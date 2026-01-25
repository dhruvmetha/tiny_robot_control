"""Keyboard controller for manual robot control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Set

from robot_control.controller.base import Controller
from robot_control.core.types import Action, Observation, Subgoal


@dataclass
class WASDInput:
    """WASD keyboard input state."""

    w: bool = False  # Forward
    a: bool = False  # Turn left
    s: bool = False  # Backward
    d: bool = False  # Turn right

    @classmethod
    def from_keys(cls, keys: Set[str]) -> WASDInput:
        """Create WASDInput from a set of pressed key names."""
        return cls(
            w="w" in keys,
            a="a" in keys,
            s="s" in keys,
            d="d" in keys,
        )

    def is_idle(self) -> bool:
        """Check if no keys are pressed."""
        return not (self.w or self.a or self.s or self.d)


class KeyboardController(Controller):
    """
    WASD keyboard controller for manual robot control.

    Translates keyboard WASD inputs to wheel speeds:
    - W: Forward (left=1.0, right=1.0)
    - S: Backward (left=-1.0, right=-1.0)
    - A: Turn left in place (left=-1.0, right=1.0)
    - D: Turn right in place (left=1.0, right=-1.0)
    - No keys: Stop (left=0.0, right=0.0)

    Supports key combinations:
    - W+A: (0, 1) pivot around left wheel
    - W+D: (1, 0) pivot around right wheel
    - S+A: (-1, 0) backward left arc
    - S+D: (0, -1) backward right arc
    """

    def __init__(self, max_speed: float = 0.3) -> None:
        """
        Initialize keyboard controller.

        Args:
            max_speed: Maximum speed multiplier [0.0, 1.0] (default: 0.3)
        """
        self._wasd_input = WASDInput()
        self._max_speed = max(0.0, min(1.0, max_speed))

    @property
    def max_speed(self) -> float:
        """Get current maximum speed multiplier."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, value: float) -> None:
        """Set maximum speed multiplier [0.0, 1.0]."""
        self._max_speed = max(0.0, min(1.0, value))

    def set_keys(self, keys: Set[str]) -> None:
        """
        Update keyboard state from pressed keys.

        Args:
            keys: Set of lowercase key names (e.g., {"w", "a"})
        """
        self._wasd_input = WASDInput.from_keys(keys)

    def set_wasd(self, wasd: WASDInput) -> None:
        """
        Set WASD input state directly.

        Args:
            wasd: Current WASD input state
        """
        self._wasd_input = wasd

    def clear_input(self) -> None:
        """Clear all WASD input (stop)."""
        self._wasd_input = WASDInput()

    def cancel(self) -> None:
        """Cancel keyboard control. Alias for clear_input()."""
        self.clear_input()

    def step(self, obs: Observation, subgoal: Subgoal) -> Action:
        """
        Compute action from keyboard state (ignores subgoal).

        Args:
            obs: Current observation (unused)
            subgoal: Target subgoal (unused - keyboard control is manual)

        Returns:
            Action based on current WASD input
        """
        wasd = self._wasd_input

        # No input -> stop
        if wasd.is_idle():
            return Action.stop()

        # Initialize speeds
        left_speed = 0.0
        right_speed = 0.0

        # Forward/backward component
        if wasd.w and not wasd.s:
            left_speed += 1.0
            right_speed += 1.0
        elif wasd.s and not wasd.w:
            left_speed -= 1.0
            right_speed -= 1.0

        # Left/right component
        if wasd.a and not wasd.d:
            left_speed -= 1.0
            right_speed += 1.0
        elif wasd.d and not wasd.a:
            left_speed += 1.0
            right_speed -= 1.0

        # Clamp to [-1, 1] and scale by max_speed
        left_speed = max(-1.0, min(1.0, left_speed)) * self._max_speed
        right_speed = max(-1.0, min(1.0, right_speed)) * self._max_speed

        return Action(left_speed=left_speed, right_speed=right_speed)

    def is_done(self, obs: Observation, subgoal: Subgoal) -> bool:
        """Always False - keyboard control never completes."""
        return False

    def get_status(self) -> str:
        """Get current status based on WASD input."""
        wasd = self._wasd_input

        if wasd.is_idle():
            return "IDLE"

        if wasd.w and wasd.a:
            return "FORWARD_LEFT"
        elif wasd.w and wasd.d:
            return "FORWARD_RIGHT"
        elif wasd.s and wasd.a:
            return "BACKWARD_LEFT"
        elif wasd.s and wasd.d:
            return "BACKWARD_RIGHT"
        elif wasd.w:
            return "FORWARD"
        elif wasd.s:
            return "BACKWARD"
        elif wasd.a:
            return "TURN_LEFT"
        elif wasd.d:
            return "TURN_RIGHT"
        else:
            return "IDLE"

    def reset(self) -> None:
        """Reset controller state."""
        self._wasd_input = WASDInput()
