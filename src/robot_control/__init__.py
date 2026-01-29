"""robot_control - Core abstractions for robot control."""

from robot_control.core.types import (
    Action,
    NavigateSubgoal,
    ObjectPose,
    Observation,
    PushSubgoal,
    Subgoal,
)
from robot_control.controller.base import Controller
from robot_control.coordinator import ControlCoordinator
from robot_control.environment.base import Environment
from robot_control.environment.sim import SimConfig, SimEnv
from robot_control.executor import SubgoalExecutor
from robot_control.planner.base import Planner
from robot_control.planner.sequence_planner import SequencePlanner, SequencePlannerConfig
from robot_control.runtime import Runtime, RuntimeConfig

__all__ = [
    "Action",
    "ControlCoordinator",
    "Controller",
    "Environment",
    "NavigateSubgoal",
    "ObjectPose",
    "Observation",
    "Planner",
    "PushSubgoal",
    "Runtime",
    "RuntimeConfig",
    "SequencePlanner",
    "SequencePlannerConfig",
    "SimConfig",
    "SimEnv",
    "Subgoal",
    "SubgoalExecutor",
]

# RealEnv requires micromvp - import conditionally
try:
    from robot_control.environment.real import RealEnv, RealEnvConfig
    __all__.extend(["RealEnv", "RealEnvConfig"])
except ImportError:
    pass
