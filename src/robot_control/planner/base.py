"""Base planner interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from robot_control.core.types import Observation, Subgoal


class Planner(ABC):
    """Abstract base class for planners."""

    @abstractmethod
    def plan(self, obs: Observation) -> Optional[Subgoal]:
        """Generate next subgoal from current observation."""
        ...

    @abstractmethod
    def is_complete(self, obs: Observation) -> bool:
        """Check if overall task is complete."""
        ...

    def reset(self) -> None:
        """Reset planner state."""
        pass
