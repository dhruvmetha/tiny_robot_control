"""Shared object definitions loader for real and simulated environments."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import yaml


@dataclass
class ObjectDef:
    """Object definition with shape and type."""

    name: str
    width: float = 0.0  # cm
    depth: float = 0.0  # cm
    height: float = 0.0  # cm
    is_static: bool = False
    is_goal: bool = False


def load_object_defs(yaml_path: Path) -> Dict[str, ObjectDef]:
    """Load object definitions from objects.yaml.

    Args:
        yaml_path: Path to objects.yaml file

    Returns:
        Dictionary mapping object name to ObjectDef
    """
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    defs = {}
    for name, obj in data.get("objects", {}).items():
        obj_type = obj.get("type", "movable")
        shape = obj.get("shape", {})
        defs[name] = ObjectDef(
            name=name,
            width=shape.get("width", 0.0),
            depth=shape.get("depth", 0.0),
            height=shape.get("height", 0.0),
            is_static=(obj_type == "static"),
            is_goal=(obj_type == "goal"),
        )
    return defs


# Default path: robot_control/config/objects.yaml
DEFAULT_OBJECTS_YAML = Path(__file__).parent.parent.parent.parent / "config" / "objects.yaml"
