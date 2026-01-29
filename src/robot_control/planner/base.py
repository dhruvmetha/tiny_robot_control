"""Base planner interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

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

    def notify_subgoal_done(self, obs: Observation) -> None:
        """Called when current subgoal is completed. Advance to next."""
        pass

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Return drawings for full plan visualization."""
        return []

    def reset(self) -> None:
        """Reset planner state."""
        pass
