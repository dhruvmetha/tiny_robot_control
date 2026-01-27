"""Workspace marker definitions for ArUco detection.

Workspace is 45cm x 65cm with 5x5 ArUco boundary markers.
Coordinate system: origin at bottom-left, +X right, +Y up.
"""

from __future__ import annotations

from typing import Dict, Tuple

# Workspace dimensions in cm
WORKSPACE_WIDTH_CM = 45.0
WORKSPACE_HEIGHT_CM = 65.0

# Marker size in cm (5x5 ArUco boundary markers)
MARKER_SIZE_CM = 4.0

# Origin offset: physical (0,0) relative to marker 8's top-left corner
# Add this to all detected positions
ORIGIN_OFFSET_CM = (2.0, 2.0)

# Workspace boundary marker positions
# Maps marker_id -> (x_cm, y_cm) of marker top-left corner
WORKSPACE_MARKERS: Dict[int, Tuple[float, float]] = {
    8: (0, 0),       # top-left
    10: (36, 0),     # top-right
    7: (18, 7),      # top-center
    0: (10, 20),     # upper-left
    2: (26, 20),     # upper-right
    4: (0, 28),      # middle-left
    6: (36, 28),     # middle-right
    1: (10, 36),     # lower-left
    3: (26, 36),     # lower-right
    5: (18, 49),     # bottom-center
    9: (0, 56),      # bottom-left
    11: (36, 56),    # bottom-right
}


def get_marker_corners_cm(marker_id: int) -> Tuple[float, float, float, float] | None:
    """
    Get marker corners in workspace coordinates.

    Args:
        marker_id: ArUco marker ID

    Returns:
        (x, y, x+size, y+size) or None if marker not found
    """
    if marker_id not in WORKSPACE_MARKERS:
        return None
    x, y = WORKSPACE_MARKERS[marker_id]
    return (x, y, x + MARKER_SIZE_CM, y + MARKER_SIZE_CM)
