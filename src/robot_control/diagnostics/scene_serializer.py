"""Structured scene-state snapshot from an Observation.

Produces a JSON-serializable dict capturing everything needed to reproduce
the scene later: workspace dimensions, robot pose + size, goal, and every
object's pose + size + static flag. Companion to the rendered JPG and the
MuJoCo XML that the planner generates.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from robot_control.core.types import Observation, WorkspaceConfig


def build_scene_state(
    mode: str,
    workspace: WorkspaceConfig,
    observation: Observation,
    workspace_origin_offset_cm: Optional[tuple[float, float]] = None,
    robot_marker_id: Optional[int] = None,
) -> Dict[str, Any]:
    """Build a JSON-ready scene snapshot.

    Args:
        mode: "sim" or "real" — recorded in the payload for downstream use.
        workspace: Workspace dimensions.
        observation: Current Observation (robot + objects + goal).
        workspace_origin_offset_cm: Optional (x_cm, y_cm) origin offset from
            the physical corner — present in real-mode config but absent in
            sim. Passes through to the payload when known.
        robot_marker_id: Optional ArUco marker ID for the robot — surfaced
            to make the snapshot useful for replay tooling.

    Returns:
        Dict ready to json.dump().
    """
    workspace_dict: Dict[str, Any] = {
        "width_cm": workspace.width,
        "height_cm": workspace.height,
    }
    if workspace_origin_offset_cm is not None:
        workspace_dict["origin_offset_cm"] = list(workspace_origin_offset_cm)

    robot_dict: Dict[str, Any] = {
        "pose_cm": [observation.robot_x, observation.robot_y, observation.robot_theta],
        "width_cm": workspace.car_width,
        "height_cm": workspace.car_height,
    }
    if robot_marker_id is not None:
        robot_dict["marker_id"] = robot_marker_id

    objects: Dict[str, Any] = {}
    for name, obj in observation.objects.items():
        objects[name] = {
            "pose_cm": [obj.x, obj.y, obj.theta],
            "width_cm": obj.width,
            "depth_cm": obj.depth,
            "height_cm": obj.height,
            "is_static": obj.is_static,
        }

    goal_cm: Optional[list] = None
    if observation.goal_x is not None and observation.goal_y is not None:
        goal_cm = [observation.goal_x, observation.goal_y]

    return {
        "captured_at_epoch": time.time(),
        "captured_at_observation_epoch": observation.timestamp,
        "mode": mode,
        "workspace": workspace_dict,
        "robot": robot_dict,
        "goal_cm": goal_cm,
        "objects": objects,
    }
