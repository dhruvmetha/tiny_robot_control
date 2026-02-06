"""Base controller interface."""

from __future__ import annotations

from abc import ABC, abstractmethod

from robot_control.core.types import Action, Observation, Subgoal


class Controller(ABC):
    """Abstract base class for controllers."""

    @abstractmethod
    def step(self, obs: Observation, subgoal: Subgoal) -> Action:
        """Compute action given observation and subgoal."""
        ...

    @abstractmethod
    def is_done(self, obs: Observation, subgoal: Subgoal) -> bool:
        """Check if subgoal is achieved."""
        ...

    def did_fail(self) -> bool:
        """Check if controller failed (vs succeeded).

        Only meaningful when is_done() returns True.
        Default implementation returns False (success).
        """
        return False

    def reset(self) -> None:
        """Reset controller state."""
        pass
