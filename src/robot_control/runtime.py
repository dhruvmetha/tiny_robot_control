"""Runtime loop for sense-plan-act orchestration."""

from __future__ import annotations

import time
from typing import Optional

from robot_control.controller.base import Controller
from robot_control.core.types import Subgoal
from robot_control.environment.base import Environment
from robot_control.planner.base import Planner


class Runtime:
    """Orchestrates the sense-plan-act loop."""

    def __init__(
        self,
        env: Environment,
        controller: Controller,
        planner: Planner,
        frequency: float = 30.0,
    ) -> None:
        self._env = env
        self._controller = controller
        self._planner = planner
        self._frequency = frequency
        self._current_subgoal: Optional[Subgoal] = None

    def step(self) -> bool:
        """
        Execute one step of the control loop.

        Returns:
            True if task is complete, False otherwise.
        """
        obs = self._env.observe()

        if self._planner.is_complete(obs):
            return True

        if self._current_subgoal is None or self._controller.is_done(
            obs, self._current_subgoal
        ):
            self._current_subgoal = self._planner.plan(obs)
            self._controller.reset()

        if self._current_subgoal is not None:
            action = self._controller.step(obs, self._current_subgoal)
            self._env.apply(action)

        return False

    def run(self) -> bool:
        """
        Run the control loop until task is complete.

        Returns:
            True if task completed successfully.
        """
        period = 1.0 / self._frequency
        while True:
            start = time.perf_counter()
            if self.step():
                return True
            elapsed = time.perf_counter() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
