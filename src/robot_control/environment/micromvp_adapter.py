"""Adapter that wraps micromvp's SerialActionSender.

This adapter implements the ActionSender protocol using micromvp's
battle-tested serial communication code. The actual protocol
implementation is NOT duplicated - we delegate to micromvp.

IMPORTANT: Requires micromvp on PYTHONPATH (configured in conda activate.d)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from robot_control.environment.action_sender import ActionSender, SenderStatus

# Import micromvp's implementation
# This is the ONLY place in robot_control that imports micromvp serial code
try:
    from micromvp.env.real_push_env import SerialActionSender, SerialActionConfig
    from micromvp.env.real_push_env.serial_action import APStatusInfo
    from micromvp.core.models import Action as MicromvpAction
    MICROMVP_AVAILABLE = True
except ImportError:
    MICROMVP_AVAILABLE = False
    SerialActionSender = None
    SerialActionConfig = None
    APStatusInfo = None
    MicromvpAction = None


@dataclass
class MicromvpAdapterConfig:
    """Configuration for the micromvp adapter."""

    # Serial port settings
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200

    # Send frequency (Hz)
    send_hz: float = 30.0

    # Motor polarity correction
    invert_right_wheel: bool = False

    # Initial robot IDs
    robot_ids: List[int] = None

    def __post_init__(self):
        if self.robot_ids is None:
            self.robot_ids = []


class MicromvpAdapter:
    """
    Adapter that wraps micromvp's SerialActionSender.

    Implements the ActionSender protocol for robot_control while
    delegating all actual communication to micromvp's battle-tested code.

    Usage:
        adapter = MicromvpAdapter(MicromvpAdapterConfig(
            port="/dev/ttyACM0",
            robot_ids=[1],
        ))
        adapter.start()
        adapter.send(robot_id=1, left_speed=0.5, right_speed=0.5)
        adapter.stop()
    """

    def __init__(self, config: MicromvpAdapterConfig) -> None:
        if not MICROMVP_AVAILABLE:
            raise ImportError(
                "micromvp not available. Ensure it's on PYTHONPATH.\n"
                "The conda environment should configure this automatically.\n"
                "Try: conda deactivate && conda activate namo-cpp"
            )

        self._config = config
        self._sender: Optional[SerialActionSender] = None

    def start(self) -> bool:
        """Start the serial sender."""
        if self._sender is not None:
            return True

        # Create micromvp's config
        serial_config = SerialActionConfig(
            port=self._config.port,
            baudrate=self._config.baudrate,
            send_hz=self._config.send_hz,
            invert_right_wheel=self._config.invert_right_wheel,
            initial_robot_ids=list(self._config.robot_ids),
        )

        self._sender = SerialActionSender(serial_config)
        return self._sender.start()

    def stop(self) -> None:
        """Stop the serial sender."""
        if self._sender is not None:
            self._sender.stop()
            self._sender = None

    def send(self, robot_id: int, left_speed: float, right_speed: float) -> None:
        """Send wheel speeds to a robot (rate-limited by send_hz)."""
        if self._sender is None:
            return

        action = MicromvpAction(left_speed=left_speed, right_speed=right_speed)
        self._sender.set_action(robot_id, action)

    def send_immediate(self, robot_id: int, left_speed: float, right_speed: float) -> bool:
        """Send wheel speeds immediately (bypasses rate limiting)."""
        if self._sender is None:
            return False

        action = MicromvpAction(left_speed=left_speed, right_speed=right_speed)
        return self._sender.send_immediate(robot_id, action)

    def stop_all(self) -> None:
        """Send stop command to all robots."""
        if self._sender is not None:
            self._sender.stop_all()

    def get_status(self) -> SenderStatus:
        """Get current status from the AP."""
        if self._sender is None:
            return SenderStatus(is_connected=False)

        ap_status = self._sender.get_ap_status()
        return SenderStatus(
            is_connected=ap_status.is_connected,
            alive_robot_ids=list(ap_status.alive_robot_ids),
            recent_events=list(ap_status.recent_events),
            last_update_time=ap_status.last_update_time,
        )

    def add_robot(self, robot_id: int) -> None:
        """Register a robot."""
        if self._sender is not None:
            self._sender.add_robot(robot_id)

    def remove_robot(self, robot_id: int) -> None:
        """Unregister a robot."""
        if self._sender is not None:
            self._sender.remove_robot(robot_id)


# Type check: MicromvpAdapter implements ActionSender protocol
def _check_protocol() -> None:
    """Static check that MicromvpAdapter implements ActionSender."""
    adapter: ActionSender = MicromvpAdapter(MicromvpAdapterConfig())  # type: ignore
    _ = adapter
