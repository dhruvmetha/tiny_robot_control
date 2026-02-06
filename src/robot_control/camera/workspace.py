"""Workspace marker definitions for ArUco detection.

Workspace dimensions are loaded from config/real.yaml.
Coordinate system: origin at bottom-left, +X right, +Y up.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

import yaml

# Default workspace dimensions in cm (overridden by config)
_DEFAULT_WIDTH_CM = 45.0
_DEFAULT_HEIGHT_CM = 65.0

# Config path
_CONFIG_PATH = Path(__file__).parent.parent.parent.parent / "config" / "real.yaml"


def _load_workspace_config() -> Tuple[float, float, Tuple[float, float]]:
    """Load workspace config from config/real.yaml."""
    default_offset = (0.5, 1.0)
    if _CONFIG_PATH.exists():
        try:
            with open(_CONFIG_PATH) as f:
                config = yaml.safe_load(f)
            workspace = config.get("workspace", {})
            width = workspace.get("width_cm", _DEFAULT_WIDTH_CM)
            height = workspace.get("height_cm", _DEFAULT_HEIGHT_CM)
            offset = workspace.get("origin_offset", list(default_offset))
            if isinstance(offset, list) and len(offset) == 2:
                offset = (float(offset[0]), float(offset[1]))
            else:
                offset = default_offset
            return width, height, offset
        except Exception:
            pass
    return _DEFAULT_WIDTH_CM, _DEFAULT_HEIGHT_CM, default_offset


# Workspace config (loaded from config/real.yaml)
WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM, ORIGIN_OFFSET_CM = _load_workspace_config()

# Marker size in cm (5x5 ArUco boundary markers)
MARKER_SIZE_CM = 4.0

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
