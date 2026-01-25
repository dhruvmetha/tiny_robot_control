"""Environment abstractions."""

from robot_control.environment.base import Environment
from robot_control.environment.sim import SimConfig, SimEnv

# RealEnv requires micromvp - import conditionally
try:
    from robot_control.environment.real import RealEnv, RealEnvConfig
    __all__ = ["Environment", "RealEnv", "RealEnvConfig", "SimConfig", "SimEnv"]
except ImportError:
    # micromvp not available - RealEnv won't be usable
    RealEnv = None
    RealEnvConfig = None
    __all__ = ["Environment", "SimConfig", "SimEnv"]
