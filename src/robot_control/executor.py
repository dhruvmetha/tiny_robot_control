"""Subgoal executor that dispatches to appropriate controllers."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple, Union

from robot_control.controller.navigation import NavigationController
from robot_control.controller.push import PushController
from robot_control.core.types import (
    Action,
    NavigateSubgoal,
    Observation,
    PushSubgoal,
    Subgoal,
    WorkspaceConfig,
)

ObstacleTuple = Tuple[float, float, float, float, float]  # (x, y, theta_deg, w, h)


class SubgoalExecutor:
    """
    Executes subgoals by dispatching to appropriate controllers.

    Separates concerns:
    - Planner decides WHAT to do (returns Subgoal)
    - Executor decides HOW to do it (picks Controller)
    - Controller executes the subgoal

    PushController uses NavigationController internally for approach navigation.

    Usage:
        executor = SubgoalExecutor(workspace_config, nav_controller)
        # PushController created internally with nav_controller for approach phase
        executor.set_subgoal(NavigateSubgoal(x=10, y=20), obs)

        while not executor.is_done(obs):
            action = executor.step(obs)
            env.apply(action)
    """

    def __init__(
        self,
        workspace_config: WorkspaceConfig,
        nav_controller: NavigationController,
        push_controller: Optional[PushController] = None,
    ) -> None:
        """
        Initialize executor.

        Args:
            workspace_config: Workspace configuration
            nav_controller: Navigation controller (also used by PushController for approach)
            push_controller: Optional push controller. If not provided but pushes are needed,
                           one will be created using nav_controller for approach navigation.
        """
        self._config = workspace_config
        self._nav = nav_controller
        self._push = push_controller
        self._current_subgoal: Optional[Subgoal] = None
        self._active_controller: Optional[Union[NavigationController, PushController]] = None

    def set_subgoal(self, subgoal: Subgoal, obs: Observation) -> bool:
        """
        Set new subgoal and configure appropriate controller.

        Returns True if subgoal was accepted.
        """
        self._current_subgoal = subgoal

        if isinstance(subgoal, NavigateSubgoal):
            self._active_controller = self._nav
            obstacles = self._build_obstacles(obs)
            return self._nav.navigate_to(
                subgoal.x,
                subgoal.y,
                subgoal.theta,
                (obs.robot_x, obs.robot_y),
                obstacles,
            )
        elif isinstance(subgoal, PushSubgoal):
            if self._push is None:
                return False
            self._active_controller = self._push
            self._push.reset()  # Reset step counter for new push
            return True  # Push controller uses subgoal directly in step()

        return False

    def step(self, obs: Observation) -> Action:
        """Execute one step of current subgoal."""
        if self._active_controller is None:
            return Action.stop()
        return self._active_controller.step(obs, self._current_subgoal)

    def is_done(self, obs: Observation) -> bool:
        """Check if current subgoal is complete."""
        if self._active_controller is None:
            return True
        return self._active_controller.is_done(obs, self._current_subgoal)

    def has_active_subgoal(self) -> bool:
        """Check if executor has an active subgoal."""
        return self._current_subgoal is not None

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Get execution drawings (path, target point, etc.)."""
        if self._active_controller is None:
            return []
        if hasattr(self._active_controller, "get_drawings"):
            return self._active_controller.get_drawings()
        return []

    def get_status(self) -> str:
        """Get status from active controller."""
        if self._active_controller is None:
            return "Idle"
        if hasattr(self._active_controller, "get_status"):
            return self._active_controller.get_status()
        return "Executing"

    def cancel(self) -> None:
        """Cancel current execution."""
        if self._active_controller and hasattr(self._active_controller, "cancel"):
            self._active_controller.cancel()
        self._current_subgoal = None
        self._active_controller = None

    def _build_obstacles(self, obs: Observation) -> List[ObstacleTuple]:
        """Build obstacle list from observation."""
        return [
            (o.x, o.y, o.theta, o.width, o.depth)
            for o in obs.objects.values()
            if o.width > 0 and o.depth > 0
        ]
