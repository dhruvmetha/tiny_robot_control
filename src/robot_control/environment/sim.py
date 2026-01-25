"""Simulation environment with DDR kinematics."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from threading import Lock, Thread
from typing import Dict, Optional, Tuple

from robot_control.core.types import Action, ObjectPose, Observation, WorkspaceConfig
from robot_control.environment.base import Environment


def _normalize_angle_deg(angle: float) -> float:
    """Normalize angle to [0, 360)."""
    return angle % 360.0


def _simulate_step(
    x: float,
    y: float,
    theta_deg: float,
    v_left: float,
    v_right: float,
    wheel_base: float,
    dt: float,
) -> Tuple[float, float, float]:
    """
    Simulate one time step of differential-drive kinematics.

    Uses the ICC (Instantaneous Center of Curvature) model.

    Args:
        x: Current x position
        y: Current y position
        theta_deg: Current heading in degrees [0, 360)
        v_left: Left wheel velocity (units/second)
        v_right: Right wheel velocity (units/second)
        wheel_base: Distance between wheels
        dt: Time step (seconds)

    Returns:
        Tuple of (new_x, new_y, new_theta_deg)
    """
    if dt <= 0.0 or (v_left == 0.0 and v_right == 0.0):
        return x, y, theta_deg

    theta = math.radians(theta_deg)
    v = 0.5 * (v_right + v_left)
    omega = (v_right - v_left) / wheel_base

    if abs(omega) < 1e-9:
        new_x = x + v * dt * math.cos(theta)
        new_y = y + v * dt * math.sin(theta)
        return new_x, new_y, theta_deg

    new_theta = theta + omega * dt
    r = v / omega
    new_x = x + r * (math.sin(new_theta) - math.sin(theta))
    new_y = y - r * (math.cos(new_theta) - math.cos(theta))
    new_theta_deg = _normalize_angle_deg(math.degrees(new_theta))

    return new_x, new_y, new_theta_deg


@dataclass
class SimConfig:
    """Configuration for simulation environment."""

    # Robot initial pose
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # degrees

    # Workspace dimensions
    width: float = 640.0
    height: float = 480.0

    # Robot geometry
    car_width: float = 36.0
    car_height: float = 52.0
    offset_w: float = 18.0  # Center offset from left edge
    offset_h: float = 26.0  # Center offset from bottom edge

    # Robot dynamics
    wheel_base: float = 30.0
    max_wheel_speed: float = 100.0

    # Simulation parameters
    speed_scale: float = 1.0  # 0=pause, 1=realtime, 2=2x speed
    physics_hz: float = 500.0  # Physics update rate

    # Static objects: {object_id: (x, y, theta)}
    objects: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)

    @property
    def workspace_config(self) -> WorkspaceConfig:
        """Create WorkspaceConfig for GUI and controllers from this SimConfig."""
        return WorkspaceConfig(
            width=self.width,
            height=self.height,
            car_width=self.car_width,
            car_height=self.car_height,
            offset_w=self.offset_w,
            offset_h=self.offset_h,
            wheel_base=self.wheel_base,
        )


class SimEnv(Environment):
    """
    Simulated environment with DDR kinematics.

    Physics runs in its own thread at a fixed timestep.
    observe() samples current state, apply() sets action.

    Usage:
        env = SimEnv(config)
        env.start()  # Start physics thread
        ...
        env.stop()   # Stop physics thread
    """

    def __init__(self, config: SimConfig) -> None:
        self._config = config
        self._x = config.x
        self._y = config.y
        self._theta = config.theta
        self._action: Action = Action.stop()
        self._speed_scale = config.speed_scale

        # Threading
        self._lock = Lock()
        self._running = False
        self._thread: Optional[Thread] = None
        self._physics_dt = 1.0 / config.physics_hz

    @property
    def config(self) -> SimConfig:
        """Static configuration (geometry, bounds)."""
        return self._config

    def start(self) -> None:
        """Start the physics thread."""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._physics_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the physics thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _physics_loop(self) -> None:
        """Main physics loop - runs at fixed timestep."""
        last_time = time.perf_counter()

        while self._running:
            now = time.perf_counter()
            elapsed = now - last_time

            if elapsed >= self._physics_dt:
                with self._lock:
                    self._advance_physics(elapsed)
                last_time = now
            else:
                # Sleep for remaining time (with some margin)
                sleep_time = self._physics_dt - elapsed
                if sleep_time > 0.0001:
                    time.sleep(sleep_time * 0.5)

    def observe(self) -> Observation:
        """Sample current state. Does not advance physics."""
        with self._lock:
            x, y, theta = self._x, self._y, self._theta

        objects = {
            obj_id: ObjectPose(x=pose[0], y=pose[1], theta=pose[2])
            for obj_id, pose in self._config.objects.items()
        }

        return Observation(
            robot_x=x,
            robot_y=y,
            robot_theta=theta,
            objects=objects,
            timestamp=time.time(),
        )

    def _advance_physics(self, dt: float) -> None:
        """Advance physics by dt seconds. Must hold lock."""
        if dt <= 0.0 or self._speed_scale <= 0.0:
            return

        effective_dt = dt * self._speed_scale

        v_left = self._action.left_speed * self._config.max_wheel_speed
        v_right = self._action.right_speed * self._config.max_wheel_speed

        new_x, new_y, new_theta = _simulate_step(
            self._x,
            self._y,
            self._theta,
            v_left,
            v_right,
            self._config.wheel_base,
            effective_dt,
        )

        self._x = max(0, min(new_x, self._config.width))
        self._y = max(0, min(new_y, self._config.height))
        self._theta = new_theta

    def apply(self, action: Action) -> None:
        """Set action for physics. Thread-safe."""
        with self._lock:
            self._action = action

    def set_speed_scale(self, scale: float) -> None:
        """Set simulation speed (0=pause, 1=realtime, 2=2x)."""
        with self._lock:
            self._speed_scale = max(0.0, scale)

    def get_speed_scale(self) -> float:
        """Get current speed scale."""
        with self._lock:
            return self._speed_scale

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Directly set robot pose (for testing/reset)."""
        with self._lock:
            self._x = x
            self._y = y
            self._theta = _normalize_angle_deg(theta)

    def get_pose(self) -> Tuple[float, float, float]:
        """Get current robot pose."""
        with self._lock:
            return (self._x, self._y, self._theta)

    @property
    def is_running(self) -> bool:
        """Check if physics thread is running."""
        return self._running
