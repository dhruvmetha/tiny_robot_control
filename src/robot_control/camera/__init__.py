"""Camera module for ArUco-based robot and object detection."""

from robot_control.camera.observer import ArucoObserver, ObserverConfig
from robot_control.camera.workspace import (
    MARKER_SIZE_CM,
    WORKSPACE_HEIGHT_CM,
    WORKSPACE_MARKERS,
    WORKSPACE_WIDTH_CM,
)
from robot_control.core.types import WorkspaceConfig


def make_real_workspace_config(
    car_width: float = 5.0,
    car_height: float = 5.0,
    offset_w: float | None = None,
    offset_h: float | None = None,
    wheel_base: float | None = None,
) -> WorkspaceConfig:
    """
    Create a WorkspaceConfig for the real camera workspace.

    Uses the fixed workspace dimensions (40x60 cm) from workspace.py.

    Args:
        car_width: Robot width in cm (default: 5.0)
        car_height: Robot height in cm (default: 5.0)
        offset_w: Robot center offset from left edge in cm (default: half of car_width)
        offset_h: Robot center offset from bottom edge in cm (default: half of car_height)
        wheel_base: Distance between wheels in cm (default: car_width)

    Returns:
        WorkspaceConfig for use with GUI and controllers
    """
    # Default offsets to center of robot
    if offset_w is None:
        offset_w = car_width / 2.0
    if offset_h is None:
        offset_h = car_height / 2.0
    if wheel_base is None:
        wheel_base = car_width  # Wheels at edges of robot width

    return WorkspaceConfig(
        width=WORKSPACE_WIDTH_CM,
        height=WORKSPACE_HEIGHT_CM,
        car_width=car_width,
        car_height=car_height,
        offset_w=offset_w,
        offset_h=offset_h,
        wheel_base=wheel_base,
    )


__all__ = [
    "ArucoObserver",
    "ObserverConfig",
    "WORKSPACE_WIDTH_CM",
    "WORKSPACE_HEIGHT_CM",
    "MARKER_SIZE_CM",
    "WORKSPACE_MARKERS",
    "make_real_workspace_config",
]
