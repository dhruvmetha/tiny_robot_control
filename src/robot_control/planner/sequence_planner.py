"""Sequence planner for executing predefined waypoint sequences."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from robot_control.core.types import NavigateSubgoal, Observation
from robot_control.planner.base import Planner


@dataclass
class SequencePlannerConfig:
    """Configuration for SequencePlanner."""

    waypoints: List[Tuple[float, float, Optional[float]]] = field(
        default_factory=list
    )  # [(x, y, theta), ...]
    loop: bool = False


class SequencePlanner(Planner):
    """
    Simple planner that executes a predefined sequence of waypoints.

    Usage:
        planner = SequencePlanner(SequencePlannerConfig(
            waypoints=[(10, 10, None), (30, 30, 90)],
            loop=True
        ))
    """

    def __init__(self, config: SequencePlannerConfig) -> None:
        self._waypoints = list(config.waypoints)
        self._loop = config.loop
        self._current_idx = 0

    def plan(self, obs: Observation) -> Optional[NavigateSubgoal]:
        """Return current waypoint as NavigateSubgoal."""
        if self._current_idx >= len(self._waypoints):
            return None
        x, y, theta = self._waypoints[self._current_idx]
        return NavigateSubgoal(x=x, y=y, theta=theta)

    def notify_subgoal_done(self, obs: Observation) -> None:
        """Advance to next waypoint."""
        self._current_idx += 1
        if self._loop and self._current_idx >= len(self._waypoints):
            self._current_idx = 0

    def is_complete(self, obs: Observation) -> bool:
        """Complete when all waypoints visited (unless looping)."""
        return self._current_idx >= len(self._waypoints) and not self._loop

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Visualize all waypoints with status coloring."""
        drawings = []

        # Draw path connecting waypoints
        if len(self._waypoints) >= 2:
            points = [(x, y) for x, y, _ in self._waypoints]
            drawings.append({
                "uuid": "plan_path",
                "type": "path",
                "points": points,
                "color": "#666666",
                "width": 1,
            })

        # Draw waypoints
        for i, (x, y, theta) in enumerate(self._waypoints):
            is_current = i == self._current_idx
            is_done = i < self._current_idx

            if is_current:
                color, radius, fill = "#00FF00", 8, "#00FF00"
            elif is_done:
                color, radius, fill = "#888888", 5, "#888888"
            else:
                color, radius, fill = "#FFFF00", 6, None

            drawings.append({
                "uuid": f"waypoint_{i}",
                "type": "point",
                "position": (x, y),
                "radius": radius,
                "color": color,
                "fill": fill,
            })

        return drawings

    def reset(self) -> None:
        """Reset to first waypoint."""
        self._current_idx = 0
