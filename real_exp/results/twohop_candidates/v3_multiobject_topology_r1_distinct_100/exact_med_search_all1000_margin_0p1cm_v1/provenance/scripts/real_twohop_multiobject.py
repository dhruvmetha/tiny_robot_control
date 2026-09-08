#!/usr/bin/env python3
"""Shared contracts for physically buildable, multi-object two-hop scenes.

The source cards are search hints, not transferred labels.  This module only
imports immutable source geometry and certifies the local static topology used
to decide whether a two-object room is an eligible composition donor.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "pipeline"))

import gen_real_buildable_scenes as base  # noqa: E402


PROFILES = ("hard1-hard1", "med2-med2")
REQUIRED_SOURCE_TOPOLOGY = {
    "closed_with_both": True,
    "target_opens_alone": True,
    "auxiliary_opens_alone": False,
    "open_with_both_removed": True,
}

_MOVABLE_RE = re.compile(r"obstacle_\d+_movable$")
_GEOMETRY_TOLERANCE_M = 1e-6


def normalize_gate_contract(
    value: Mapping[str, Any] | None,
    *,
    gate_objects: tuple[str, str] | list[str] | None = None,
) -> dict[str, list[list[str]]]:
    """Normalize the pure two-gate membership contract without importing MuJoCo."""
    if value is None:
        if gate_objects is None or len(gate_objects) != 2:
            raise ValueError("legacy gate contract requires two inferred gate objects")
        value = {
            "boundary_objects": [[gate_objects[0]], [gate_objects[1]]],
            "gate_members": [[gate_objects[0]], [gate_objects[1]]],
        }
    if not isinstance(value, Mapping):
        raise ValueError("gate contract must be a mapping")
    raw_boundaries = value.get("boundary_objects")
    raw_members = value.get("gate_members")
    if not isinstance(raw_boundaries, (list, tuple)) or not isinstance(
        raw_members, (list, tuple)
    ):
        raise ValueError("gate contract requires boundary_objects and gate_members")
    boundaries = [
        list(group) if isinstance(group, (list, tuple)) else []
        for group in raw_boundaries
    ]
    members = [
        list(group) if isinstance(group, (list, tuple)) else []
        for group in raw_members
    ]
    if len(boundaries) != 2 or any(len(group) != 1 for group in boundaries):
        raise ValueError("gate contract requires two singleton boundary openers")
    if len(members) != 2 or any(not group for group in members):
        raise ValueError("gate contract requires two non-empty gate member groups")
    if any(
        not isinstance(object_id, str) or not object_id
        for group in boundaries + members
        for object_id in group
    ):
        raise ValueError("gate contract object IDs must be non-empty strings")
    if any(len(set(group)) != len(group) for group in members):
        raise ValueError("gate contract member groups cannot contain duplicates")
    if any(boundaries[index][0] not in members[index] for index in range(2)):
        raise ValueError("each boundary opener must belong to its gate")
    if set(members[0]) & set(members[1]):
        raise ValueError("gate member groups must be disjoint")
    if boundaries[0][0] == boundaries[1][0]:
        raise ValueError("boundary openers must be distinct")
    return {"boundary_objects": boundaries, "gate_members": members}


@dataclass(frozen=True)
class SourceRect:
    name: str
    cx: float
    cy: float
    hx: float
    hy: float
    yaw: float


@dataclass(frozen=True)
class SourceMovable:
    sim_id: str
    physical_shape: str
    cx: float
    cy: float
    hx: float
    hy: float
    hz: float
    yaw: float


@dataclass(frozen=True)
class MultiObjectDonor:
    source_host_path: str
    source_family: str
    source_scene_id: str
    card_path: str
    card_object_id: str
    xml_sha256: str
    card_sha256: str
    solve_rate_1push: float
    solve_rate: float
    walls: tuple[SourceRect, ...]
    target: SourceMovable
    auxiliary: SourceMovable
    robot_start: tuple[float, float]
    goal: tuple[float, float]
    source_hint_target_aligned: bool = True
    source_hint_profiles: tuple[str, ...] = ()


def _finite_numbers(raw: str | None, expected: int, *, context: str) -> list[float]:
    try:
        values = [float(value) for value in (raw or "").split()]
    except ValueError as exc:
        raise ValueError(f"{context}: non-numeric value") from exc
    if len(values) != expected or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{context}: expected {expected} finite values")
    return values


def _yaw(root: ET.Element, geom: ET.Element, *, context: str) -> float:
    quaternion = geom.get("quat")
    if quaternion is not None:
        w, x, y, z = _finite_numbers(quaternion, 4, context=f"{context} quaternion")
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm <= 0.0:
            raise ValueError(f"{context}: zero quaternion")
        w, x, y, z = (value / norm for value in (w, x, y, z))
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    euler = _finite_numbers(geom.get("euler", "0 0 0"), 3, context=f"{context} Euler angles")
    compiler = root.find("compiler")
    angle_unit = (compiler.get("angle") if compiler is not None else None) or "degree"
    if angle_unit not in {"degree", "radian"}:
        raise ValueError(f"{context}: unsupported compiler angle unit {angle_unit!r}")
    return math.radians(euler[2]) if angle_unit == "degree" else euler[2]


def _pose_and_size(
    root: ET.Element,
    geom: ET.Element,
    *,
    context: str,
) -> tuple[float, float, float, float, float, float]:
    position = _finite_numbers(geom.get("pos"), 3, context=f"{context} position")
    size = _finite_numbers(geom.get("size"), 3, context=f"{context} size")
    if any(value <= 0.0 for value in size):
        raise ValueError(f"{context}: box half-extents must be positive")
    return position[0], position[1], size[0], size[1], size[2], _yaw(root, geom, context=context)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _shape_name(hx: float, hy: float, hz: float) -> str | None:
    for name in ("obj_1", "obj_4"):
        expected = base.MOVABLES[name]
        if all(
            math.isclose(value, wanted, rel_tol=0.0, abs_tol=_GEOMETRY_TOLERANCE_M)
            for value, wanted in zip((hx, hy, hz), expected)
        ):
            return name
    return None


def _read_rate(meta: Mapping[str, Any], key: str, *, context: str) -> float:
    try:
        value = float(meta[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{context}: missing or invalid {key}") from exc
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{context}: {key} must be a finite rate in [0, 1]")
    return value


def source_matches_profile(profile: str, solve_rate_1push: float, solve_rate: float) -> bool:
    """Apply the canonical source-hint bounds for one requested local profile."""
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {PROFILES}")
    if profile == "hard1-hard1":
        return 0.0 < solve_rate_1push < 0.05
    return solve_rate_1push == 0.0 and 0.05 <= solve_rate < 0.30


def load_source_donor(
    xml_path: str | Path,
    card_path: str | Path,
    *,
    source_family: str,
) -> MultiObjectDonor:
    """Load and inventory-check one two-wall, obj_1-plus-obj_4 source donor."""
    xml_path = Path(xml_path).resolve()
    card_path = Path(card_path).resolve()
    context = f"{source_family}/{xml_path.parent.name}"
    try:
        root = ET.parse(xml_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"{context}: cannot parse {xml_path}") from exc
    try:
        card = json.loads(card_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as exc:
        raise ValueError(f"{context}: cannot parse {card_path}") from exc
    meta = card.get("meta")
    if not isinstance(meta, Mapping):
        raise ValueError(f"{context}: gallery card has no meta object")
    target_id = meta.get("object_id")
    if not isinstance(target_id, str) or not _MOVABLE_RE.fullmatch(target_id):
        raise ValueError(f"{context}: invalid gallery-card object_id")

    walls: list[SourceRect] = []
    for geom in root.findall(".//geom"):
        name = geom.get("name") or ""
        if not name.startswith("wall_inner_"):
            continue
        cx, cy, hx, hy, _hz, yaw = _pose_and_size(root, geom, context=f"{context}/{name}")
        if not (
            math.isclose(hx, base.BRICK_HALF[0], rel_tol=0.0, abs_tol=_GEOMETRY_TOLERANCE_M)
            and math.isclose(hy, base.BRICK_HALF[1], rel_tol=0.0, abs_tol=_GEOMETRY_TOLERANCE_M)
        ):
            raise ValueError(f"{context}: inner walls must use the real brick footprint")
        walls.append(SourceRect(name, cx, cy, hx, hy, yaw))
    if len(walls) != 2:
        raise ValueError(f"{context}: expected exactly 2 inner walls, found {len(walls)}")

    movables: dict[str, SourceMovable] = {}
    for geom in root.findall(".//geom"):
        sim_id = geom.get("name") or ""
        if not _MOVABLE_RE.fullmatch(sim_id):
            continue
        if sim_id in movables:
            raise ValueError(f"{context}: duplicate movable geom {sim_id!r}")
        cx, cy, hx, hy, hz, yaw = _pose_and_size(root, geom, context=f"{context}/{sim_id}")
        shape = _shape_name(hx, hy, hz)
        if shape is None:
            raise ValueError(f"{context}: movables must have obj_1 and obj_4 footprints")
        movables[sim_id] = SourceMovable(sim_id, shape, cx, cy, hx, hy, hz, yaw)
    if len(movables) != 2 or {movable.physical_shape for movable in movables.values()} != {
        "obj_1",
        "obj_4",
    }:
        raise ValueError(f"{context}: expected exactly 2 movables with obj_1 and obj_4 footprints")
    if target_id not in movables:
        raise ValueError(f"{context}: card target {target_id!r} is absent from XML")
    auxiliary = next(movable for sim_id, movable in movables.items() if sim_id != target_id)

    robot = root.find(".//body[@name='car']")
    goal = root.find(".//site[@name='goal']")
    if robot is None or goal is None:
        raise ValueError(f"{context}: XML must contain car body and goal site")
    robot_pos = _finite_numbers(robot.get("pos"), 3, context=f"{context}/car position")
    goal_pos = _finite_numbers(goal.get("pos"), 3, context=f"{context}/goal position")

    solve_rate_1push = _read_rate(meta, "solve_rate_1push", context=context)
    solve_rate = _read_rate(meta, "solve_rate", context=context)
    return MultiObjectDonor(
        source_host_path=str(xml_path),
        source_family=source_family,
        source_scene_id=xml_path.parent.name,
        card_path=str(card_path),
        card_object_id=target_id,
        xml_sha256=_sha256(xml_path),
        card_sha256=_sha256(card_path),
        solve_rate_1push=solve_rate_1push,
        solve_rate=solve_rate,
        walls=tuple(sorted(walls, key=lambda wall: wall.name)),
        target=movables[target_id],
        auxiliary=auxiliary,
        robot_start=(robot_pos[0], robot_pos[1]),
        goal=(goal_pos[0], goal_pos[1]),
        source_hint_target_aligned=True,
        source_hint_profiles=tuple(
            profile
            for profile in PROFILES
            if source_matches_profile(profile, solve_rate_1push, solve_rate)
        ),
    )


def _base_rect(rect: SourceRect | SourceMovable) -> base.Rect:
    name = rect.name if isinstance(rect, SourceRect) else rect.sim_id
    kind = "wall" if isinstance(rect, SourceRect) else "movable"
    return base.Rect(rect.cx, rect.cy, rect.hx, rect.hy, rect.yaw, name, kind)


def _reachable(rects: list[base.Rect], start: tuple[float, float], goal: tuple[float, float]) -> bool:
    blocked = base._blocked_mask(rects, base.INFLATE_R)
    start_cell = base._cell(start)
    goal_cell = base._cell(goal)
    nx, ny = blocked.shape
    if not (
        0 <= start_cell[0] < nx
        and 0 <= start_cell[1] < ny
        and 0 <= goal_cell[0] < nx
        and 0 <= goal_cell[1] < ny
    ):
        return False
    if blocked[start_cell] or blocked[goal_cell]:
        return False
    return base._connected(blocked, start_cell, goal_cell)


def source_solo_topology(donor: MultiObjectDonor) -> dict[str, bool]:
    """Return the four counterfactuals defining an eligible solo-opener donor."""
    walls = [_base_rect(wall) for wall in donor.walls]
    target = _base_rect(donor.target)
    auxiliary = _base_rect(donor.auxiliary)
    start, goal = donor.robot_start, donor.goal
    return {
        "closed_with_both": not _reachable(walls + [target, auxiliary], start, goal),
        "target_opens_alone": _reachable(walls + [auxiliary], start, goal),
        "auxiliary_opens_alone": _reachable(walls + [target], start, goal),
        "open_with_both_removed": _reachable(walls, start, goal),
    }


def orient_unique_solo_opener(donor: MultiObjectDonor) -> MultiObjectDonor | None:
    """Orient a closed single-boundary donor toward its unique solo opener."""
    topology = source_solo_topology(donor)
    if topology == REQUIRED_SOURCE_TOPOLOGY:
        return donor
    auxiliary_only = {
        "closed_with_both": True,
        "target_opens_alone": False,
        "auxiliary_opens_alone": True,
        "open_with_both_removed": True,
    }
    if topology != auxiliary_only:
        return None
    return replace(
        donor,
        target=donor.auxiliary,
        auxiliary=donor.target,
        source_hint_target_aligned=False,
        source_hint_profiles=(),
    )


def transform_rect(
    rect: SourceRect | SourceMovable,
    *,
    target_y: float,
    opener_y: float,
    mirror_x: bool,
    name: str | None = None,
    kind: str | None = None,
) -> base.Rect:
    """Rigidly translate a donor to a gate and optionally reflect it across the table center."""
    cx = base.ARENA_W - rect.cx if mirror_x else rect.cx
    yaw = math.pi - rect.yaw if mirror_x else rect.yaw
    source_name = rect.name if isinstance(rect, SourceRect) else rect.sim_id
    source_kind = "brick" if isinstance(rect, SourceRect) else rect.physical_shape
    return base.Rect(
        cx,
        rect.cy + target_y - opener_y,
        rect.hx,
        rect.hy,
        yaw,
        name or source_name,
        kind or source_kind,
    )


def connectivity_tuple(
    rects: Iterable[base.Rect],
    scene: Mapping[str, Any],
    inflate: float,
) -> tuple[bool, bool, bool]:
    """Return start-middle, middle-goal, and start-goal connectivity."""
    blocked = base._blocked_mask(list(rects), inflate)
    cells = [base._cell(scene[key]) for key in ("start", "middle_anchor", "goal")]
    nx, ny = blocked.shape

    def connected(left: int, right: int) -> bool:
        a, b = cells[left], cells[right]
        if not all(0 <= x < nx and 0 <= y < ny for x, y in (a, b)):
            return False
        if blocked[a] or blocked[b]:
            return False
        return bool(base._connected(blocked, a, b))

    return connected(0, 1), connected(1, 2), connected(0, 2)


def three_object_counterfactual_trace(
    scene: Mapping[str, Any],
    margin_r: float,
) -> dict[str, tuple[bool, bool, bool]]:
    """Evaluate both gate openings while retaining the multi-object gate's helper."""
    blockers = [rect for rect, _shape in scene["blockers"]]
    if len(blockers) != 3:
        raise ValueError(f"three-object trace needs 3 movables, found {len(blockers)}")
    k1, k2, helper = blockers
    statics = list(scene["statics"])
    return {
        "initial": connectivity_tuple(statics + [k1, k2, helper], scene, base.INFLATE_R),
        "post_k1": connectivity_tuple(statics + [k2, helper], scene, base.INFLATE_R),
        "remove_auxiliary_only": connectivity_tuple(statics + [k1, k2], scene, base.INFLATE_R),
        "remove_k2_only": connectivity_tuple(statics + [k1, helper], scene, base.INFLATE_R),
        "both_openers_removed": connectivity_tuple(statics + [helper], scene, base.INFLATE_R),
        "margin_both_openers_removed": connectivity_tuple(statics + [helper], scene, margin_r),
    }


def donor_to_json(donor: MultiObjectDonor) -> dict[str, Any]:
    record = asdict(donor)
    if donor.source_hint_target_aligned and not donor.source_hint_profiles:
        record["source_hint_profiles"] = [
            profile
            for profile in PROFILES
            if source_matches_profile(
                profile,
                donor.solve_rate_1push,
                donor.solve_rate,
            )
        ]
    return record


def donor_from_json(record: Mapping[str, Any]) -> MultiObjectDonor:
    walls = tuple(SourceRect(**wall) for wall in record["walls"])
    target = SourceMovable(**record["target"])
    auxiliary = SourceMovable(**record["auxiliary"])
    source_hint_target_aligned = bool(record.get("source_hint_target_aligned", True))
    raw_hint_profiles = record.get("source_hint_profiles")
    if raw_hint_profiles is None:
        raw_hint_profiles = (
            profile
            for profile in PROFILES
            if source_hint_target_aligned
            and source_matches_profile(
                profile,
                float(record["solve_rate_1push"]),
                float(record["solve_rate"]),
            )
        )
    return MultiObjectDonor(
        source_host_path=str(record["source_host_path"]),
        source_family=str(record["source_family"]),
        source_scene_id=str(record["source_scene_id"]),
        card_path=str(record["card_path"]),
        card_object_id=str(record["card_object_id"]),
        xml_sha256=str(record["xml_sha256"]),
        card_sha256=str(record["card_sha256"]),
        solve_rate_1push=float(record["solve_rate_1push"]),
        solve_rate=float(record["solve_rate"]),
        walls=walls,
        target=target,
        auxiliary=auxiliary,
        robot_start=tuple(float(value) for value in record["robot_start"]),
        goal=tuple(float(value) for value in record["goal"]),
        source_hint_target_aligned=source_hint_target_aligned,
        source_hint_profiles=tuple(str(value) for value in raw_hint_profiles),
    )
