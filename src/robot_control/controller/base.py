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

    def reset(self) -> None:
        """Reset controller state."""
        pass
