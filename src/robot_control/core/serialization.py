"""JSON serialization for Observation objects over ZeroMQ."""

from __future__ import annotations

import json
from typing import Dict

from robot_control.core.types import ObjectPose, Observation


def obs_to_bytes(obs: Observation) -> bytes:
    """Serialize an Observation to JSON bytes for ZMQ transport."""
    objects = {}
    for name, obj in obs.objects.items():
        objects[name] = {
            "x": obj.x,
            "y": obj.y,
            "theta": obj.theta,
            "width": obj.width,
            "depth": obj.depth,
            "height": obj.height,
            "is_static": obj.is_static,
        }

    data = {
        "robot_x": obs.robot_x,
        "robot_y": obs.robot_y,
        "robot_theta": obs.robot_theta,
        "timestamp": obs.timestamp,
        "goal_x": obs.goal_x,
        "goal_y": obs.goal_y,
        "objects": objects,
    }
    return json.dumps(data).encode("utf-8")


def bytes_to_obs(data: bytes) -> Observation:
    """Deserialize JSON bytes into an Observation."""
    d = json.loads(data.decode("utf-8"))

    objects: Dict[str, ObjectPose] = {}
    for name, obj_d in d.get("objects", {}).items():
        objects[name] = ObjectPose(
            x=obj_d["x"],
            y=obj_d["y"],
            theta=obj_d["theta"],
            width=obj_d.get("width", 0.0),
            depth=obj_d.get("depth", 0.0),
            height=obj_d.get("height", 0.0),
            is_static=obj_d.get("is_static", False),
        )

    return Observation(
        robot_x=d["robot_x"],
        robot_y=d["robot_y"],
        robot_theta=d["robot_theta"],
        timestamp=d["timestamp"],
        goal_x=d.get("goal_x"),
        goal_y=d.get("goal_y"),
        objects=objects,
    )
