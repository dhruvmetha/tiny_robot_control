"""robot_control - Core abstractions for robot control."""

from robot_control.core.types import Action, ObjectPose, Observation, Subgoal
from robot_control.controller.base import Controller
from robot_control.environment.base import Environment
from robot_control.environment.sim import SimConfig, SimEnv
from robot_control.planner.base import Planner
from robot_control.runtime import Runtime

__all__ = [
    "Action",
    "Controller",
    "Environment",
    "ObjectPose",
    "Observation",
    "Planner",
    "Runtime",
    "SimConfig",
    "SimEnv",
    "Subgoal",
]

# RealEnv requires micromvp - import conditionally
try:
    from robot_control.environment.real import RealEnv, RealEnvConfig
    __all__.extend(["RealEnv", "RealEnvConfig"])
except ImportError:
    pass
