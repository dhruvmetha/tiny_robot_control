"""Base environment interface."""

from __future__ import annotations

from abc import ABC, abstractmethod

from robot_control.core.types import Action, Observation


class Environment(ABC):
    """Abstract base class for environments."""

    @abstractmethod
    def observe(self) -> Observation:
        """Get current observation from the environment."""
        ...

    @abstractmethod
    def apply(self, action: Action) -> None:
        """Apply an action to the environment."""
        ...

    def close(self) -> None:
        """Clean up resources."""
        pass
