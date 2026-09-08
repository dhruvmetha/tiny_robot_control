#!/usr/bin/env python3
"""Generate independent two-keyhole scenes from the confirmed real-table inventory.

The older real-buildable generator is deliberately single-hop. Its two-movable mode places both
objects in one doorway as a domino, and its two-band mode leaves the second band open. Combining
those flags therefore does not produce two independent keyholes.

This generator constructs the desired topology directly:

    robot room -> K1 -> middle room -> K2 -> goal room

Two measured two-brick modules are translated into the three-room layout. Their bars are assigned
to the four configured physical IDs wall_9 through wall_12. Wall_9 is a 19.5 cm by 5.5 cm clone of
wall_10, and the confirmed movables obj_1 and obj_4 are used once each.

The requested paper profiles are selection targets, not generator labels. Every emitted sheet says
``classification_status: unlabelled`` until exhaustive simulation labels K1 on the original XML and
K2 on the post-K1 XML emitted by ``materialize_keyhole2.py``.

Example:

  python scripts/pipeline/gen_real_buildable_twohop.py \
      --out-dir /path/to/real_exp/environments/twohop_candidates/hard1-hard1 \
      --profile hard1-hard1 --donor-csv /path/to/sheets_v2/1push/hard.csv \
      --num 1 --seed 4100
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import gen_real_buildable_scenes as base


OBJECT_ORDERS = (("obj_1", "obj_4"), ("obj_4", "obj_1"))
MIN_GATE_SEPARATION_M = 0.24
INDEPENDENCE_CLEARANCE_M = 0.08
WALL_MARKERS_2 = ("wall_10", "wall_11")
WALL_MARKERS_3 = ("wall_10", "wall_11", "wall_9")
WALL_MARKERS_4 = ("wall_9", "wall_10", "wall_11", "wall_12")
EXTRA_WALL_JOINT_GAP_M = 0.010
EXTRA_WALL_EDGE_INSET_M = 0.001
LOWER_GATE_Y_OPTIONS = (0.22, 0.24, 0.26, 0.28)
UPPER_GATE_Y_OPTIONS = (0.49, 0.51, 0.53, 0.55)

PROFILES = {
    "hard1-hard1": {
        "axis": "1push",
        "tier": "hard",
        "push_kind": "one_push",
    },
    "med2-med2": {
        "axis": "hmax2",
        "tier": "med",
        "push_kind": "needs_2_chain",
    },
}


@dataclass
class DonorModule:
    build_id: str
    bricks: list[dict]
    blocker: dict
    source_label: dict

    @property
    def object_name(self) -> str:
        return self.blocker["marker_hint"]


def donor_from_rows(rows: list[dict]) -> DonorModule:
    """Parse one build-sheet group without changing its measured local label."""
    if not rows:
        raise ValueError("a donor module needs at least one row")
    build_ids = {row["build_id"] for row in rows}
    if len(build_ids) != 1:
        raise ValueError(f"donor rows mix build IDs: {sorted(build_ids)}")
    bricks = [row for row in rows if row.get("item") == "brick"]
    blockers = [row for row in rows if row.get("item") == "block"]
    if len(bricks) != 2 or len(blockers) != 1:
        raise ValueError(
            f"{next(iter(build_ids))}: two-hop composition needs exactly 2 bricks and 1 block"
        )
    blocker = blockers[0]
    if blocker.get("marker_hint") not in base.ON_TABLE:
        raise ValueError(f"unsupported physical blocker {blocker.get('marker_hint')!r}")
    label = {
        "axis": blocker["axis"],
        "tier": blocker["tier"],
        "push_kind": blocker["push_kind"],
        "solve_rate": float(blocker["solve_rate"]),
        "tried": int(blocker["tried"]),
        "valid_1push": int(blocker["valid_1push"]),
        "valid_first_push": int(blocker["valid_first_push"]),
    }
    return DonorModule(next(iter(build_ids)), bricks, blocker, label)


def load_donors(csv_path: str | Path, profile: str) -> list[DonorModule]:
    """Load only two-brick modules carrying the existing label required by ``profile``."""
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {sorted(PROFILES)}")
    grouped: dict[str, list[dict]] = {}
    with Path(csv_path).open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            grouped.setdefault(row["build_id"], []).append(row)
    donors: list[DonorModule] = []
    for rows in grouped.values():
        try:
            donor = donor_from_rows(rows)
        except ValueError:
            continue
        label = donor.source_label
        if profile == "hard1-hard1":
            matches = (
                label["axis"] == "1push"
                and label["tier"] == "hard"
                and label["push_kind"] == "one_push"
                and label["valid_1push"] > 0
            )
        else:
            matches = (
                label["axis"] == "hmax2"
                and label["tier"] == "med"
                and label["push_kind"] == "needs_2_chain"
                and label["valid_1push"] == 0
                and label["valid_first_push"] > 0
            )
        if matches:
            donors.append(donor)
    return sorted(donors, key=lambda donor: donor.build_id)


def _connectivity(rects: Iterable[base.Rect], scene: dict, inflate: float) -> dict[str, bool]:
    blocked = base._blocked_mask(list(rects), inflate)
    cells = {
        "start": base._cell(scene["start"]),
        "middle": base._cell(scene["middle_anchor"]),
        "goal": base._cell(scene["goal"]),
    }

    def connected(left: str, right: str) -> bool:
        a, b = cells[left], cells[right]
        nx, ny = blocked.shape
        if not all(0 <= i < nx and 0 <= j < ny for i, j in (a, b)):
            return False
        if blocked[a] or blocked[b]:
            return False
        return bool(base._connected(blocked, a, b))

    return {
        "start_to_middle": connected("start", "middle"),
        "middle_to_goal": connected("middle", "goal"),
        "start_to_goal": connected("start", "goal"),
    }


def counterfactual_trace(scene: dict, margin_r: float) -> dict[str, dict[str, bool]]:
    """Return the four topology counterfactuals plus the both-open margin check."""
    k1, k2 = (rect for rect, _name in scene["blockers"])
    statics = scene["statics"]
    return {
        "initial": _connectivity(statics + [k1, k2], scene, base.INFLATE_R),
        "post_k1": _connectivity(statics + [k2], scene, base.INFLATE_R),
        "remove_k2_only": _connectivity(statics + [k1], scene, base.INFLATE_R),
        "both_removed": _connectivity(statics, scene, base.INFLATE_R),
        "margin_both_removed": _connectivity(statics, scene, margin_r),
    }


def _validate_scene(scene: dict) -> tuple[bool, str]:
    k1, k2 = (rect for rect, _name in scene["blockers"])
    if math.dist((k1.cx, k1.cy), (k2.cx, k2.cy)) < MIN_GATE_SEPARATION_M:
        return False, "gate_separation"
    if base.overlaps(k1, k2, pad=INDEPENDENCE_CLEARANCE_M):
        return False, "independence_clearance"
    all_rects = scene["statics"] + [k1, k2]
    for index, rect in enumerate(all_rects):
        if not base.inside_arena(rect):
            return False, f"outside_arena_{index}"
        for other in all_rects[index + 1 :]:
            if base.overlaps(rect, other, pad=0.004):
                return False, "physical_overlap"
    if not all(
        (
            base.start_is_placeable(scene["statics"] + [k2], k1, scene["start"]),
            base.start_is_placeable(scene["statics"] + [k1], k2, scene["start"]),
        )
    ):
        return False, "start_not_placeable"

    expected = {
        "initial": (False, False, False),
        "post_k1": (True, False, False),
        "remove_k2_only": (False, True, False),
        "both_removed": (True, True, True),
    }
    trace = counterfactual_trace(scene, scene["margin_r"])
    keys = ("start_to_middle", "middle_to_goal", "start_to_goal")
    for state, want in expected.items():
        if tuple(trace[state][key] for key in keys) != want:
            return False, f"topology_{state}"
    if not trace["margin_both_removed"]["start_to_goal"]:
        return False, "no_margin"
    return True, "ok"


def _row_rect(row: dict, name: str, dy: float, dx: float = 0.0) -> base.Rect:
    """Return one donor row translated as part of a rigid measured module."""
    is_block = row["item"] == "block"
    if is_block:
        object_name = row["marker_hint"]
        hx, hy, _hz = base.MOVABLES[object_name]
    else:
        hx = float(row["long_cm"]) / 200.0
        hy = float(row["short_cm"]) / 200.0
    yaw = math.radians(float(row["long_axis_bearing_deg"]) - (90.0 if is_block else 0.0))
    return base.Rect(
        float(row["centre_x_cm"]) / 100.0 + dx,
        float(row["centre_y_cm"]) / 100.0 + dy,
        hx,
        hy,
        yaw,
        name,
        row["marker_hint"],
    )


def _populate_gate_metrics(scene: dict) -> None:
    k1, k2 = (rect for rect, _name in scene["blockers"])
    statics = scene["statics"]
    margin_r = scene["margin_r"]
    scene["gate_metrics"] = [
        {
            "n_contacts": base.n_reachable_contacts(statics + [k2], k1, scene["start"]),
            "open_frac": base.open_frac(
                statics + [k2], k1, scene["start"], scene["middle_anchor"], margin_r
            ),
        },
        {
            "n_contacts": base.n_reachable_contacts(statics, k2, scene["middle_anchor"]),
            "open_frac": base.open_frac(
                statics, k2, scene["middle_anchor"], scene["goal"], margin_r
            ),
        },
    ]
    scene["n_contacts"] = scene["gate_metrics"][0]["n_contacts"]
    scene["open_frac"] = scene["gate_metrics"][0]["open_frac"]


def compose_donors(
    lower: DonorModule,
    upper: DonorModule,
    *,
    margin_cm: float,
    lower_gate_y: float = 0.250,
    upper_gate_y: float = 0.525,
    lower_shift_x: float = 0.0,
    upper_shift_x: float = 0.0,
    populate_metrics: bool = True,
) -> dict | None:
    """Translate two measured modules as rigid units into the two real-table gates."""
    if {lower.object_name, upper.object_name} != set(base.ON_TABLE):
        return None
    lower_dy = lower_gate_y - float(lower.blocker["centre_y_cm"]) / 100.0
    upper_dy = upper_gate_y - float(upper.blocker["centre_y_cm"]) / 100.0
    statics = [
        _row_rect(row, f"wall_inner_{index + 1}", lower_dy, lower_shift_x)
        for index, row in enumerate(lower.bricks)
    ]
    statics.extend(
        _row_rect(row, f"wall_inner_{index + 3}", upper_dy, upper_shift_x)
        for index, row in enumerate(upper.bricks)
    )
    k1 = _row_rect(lower.blocker, "obstacle_0_movable", lower_dy, lower_shift_x)
    k2 = _row_rect(upper.blocker, "obstacle_1_movable", upper_dy, upper_shift_x)
    margin_r = margin_cm / 200.0 + base.TIER1_MARGIN
    scene = {
        "statics": statics,
        "blocker": k1,
        "blocker_name": lower.object_name,
        "blockers": [(k1, lower.object_name), (k2, upper.object_name)],
        "start": (
            float(lower.blocker["robot_start_x_cm"]) / 100.0 + lower_shift_x,
            float(lower.blocker["robot_start_y_cm"]) / 100.0 + lower_dy,
        ),
        "middle_anchor": (
            float(upper.blocker["robot_start_x_cm"]) / 100.0 + upper_shift_x,
            float(upper.blocker["robot_start_y_cm"]) / 100.0 + upper_dy,
        ),
        "goal": (
            float(upper.blocker["goal_x_cm"]) / 100.0 + upper_shift_x,
            float(upper.blocker["goal_y_cm"]) / 100.0 + upper_dy,
        ),
        "margin_r": margin_r,
        "source_modules": [lower, upper],
        "gate_target_y": [lower_gate_y, upper_gate_y],
        "module_shift_x": [lower_shift_x, upper_shift_x],
    }
    passed, _reason = _validate_scene(scene)
    if not passed:
        return None
    if populate_metrics:
        _populate_gate_metrics(scene)
    return scene


def donor_compositions(
    donors: list[DonorModule],
    *,
    margin_cm: float,
    lower_build_id: str | None = None,
    upper_build_id: str | None = None,
    lower_gate_y_options: tuple[float, ...] = LOWER_GATE_Y_OPTIONS,
    upper_gate_y_options: tuple[float, ...] = UPPER_GATE_Y_OPTIONS,
    lower_shift_x_options: tuple[float, ...] = (0.0,),
    upper_shift_x_options: tuple[float, ...] = (0.0,),
) -> list[dict]:
    """Compose ordered donors over an explicit rigid-module geometry grid."""
    lower_donors = [
        donor
        for donor in donors
        if lower_build_id is None or donor.build_id == lower_build_id
    ]
    upper_donors = [
        donor
        for donor in donors
        if upper_build_id is None or donor.build_id == upper_build_id
    ]
    scenes = []
    for lower, upper in itertools.product(lower_donors, upper_donors):
        for lower_gate_y, upper_gate_y, lower_shift_x, upper_shift_x in itertools.product(
            lower_gate_y_options,
            upper_gate_y_options,
            lower_shift_x_options,
            upper_shift_x_options,
        ):
            scene = compose_donors(
                lower,
                upper,
                margin_cm=margin_cm,
                lower_gate_y=lower_gate_y,
                upper_gate_y=upper_gate_y,
                lower_shift_x=lower_shift_x,
                upper_shift_x=upper_shift_x,
                populate_metrics=False,
            )
            if scene is not None:
                scenes.append(scene)
    return scenes


def filter_donor_compositions(
    scenes: list[dict],
    *,
    lower_build_id: str | None,
    upper_build_id: str | None,
) -> list[dict]:
    """Keep only scenes composed from the requested ordered donor modules."""
    return [
        scene
        for scene in scenes
        if (
            lower_build_id is None
            or scene["source_modules"][0].build_id == lower_build_id
        )
        and (
            upper_build_id is None
            or scene["source_modules"][1].build_id == upper_build_id
        )
    ]


def _horizontal_support(object_name: str, yaw: float) -> float:
    hx, hy, _hz = base.MOVABLES[object_name]
    return abs(math.cos(yaw)) * hx + abs(math.sin(yaw)) * hy


def _blocking_yaw(object_name: str, rng: random.Random) -> float:
    """Aim the object's largest support across the table, with small candidate diversity."""
    hx, hy, _hz = base.MOVABLES[object_name]
    optimum = math.atan2(hy, hx)
    return optimum + math.radians(rng.uniform(-2.0, 2.0))


def _blocker(object_name: str, side: str, y: float, rng: random.Random, index: int) -> base.Rect:
    yaw = _blocking_yaw(object_name, rng)
    support_x = _horizontal_support(object_name, yaw)
    wall_end = 2.0 * base.BRICK_HALF[0]
    free_width = base.ARENA_W - wall_end - 2.0 * support_x
    if free_width <= 0.0:
        raise ValueError(f"{object_name} does not fit beside the divider wall")
    gap = free_width / 2.0
    cx = wall_end + gap + support_x if side == "left" else gap + support_x
    hx, hy, _hz = base.MOVABLES[object_name]
    return base.Rect(cx, y, hx, hy, yaw, f"obstacle_{index}_movable", object_name)


def _extra_divider_wall(gate: int, lower_y: float, upper_y: float) -> base.Rect:
    """Place the wall_9 clone along the table edge adjoining the requested divider."""
    half_long, half_short, _half_height = base.BRICK_HALF
    center_offset = half_short + EXTRA_WALL_JOINT_GAP_M + half_long
    if gate == 1:
        cx = half_short + EXTRA_WALL_EDGE_INSET_M
        cy = lower_y + center_offset
    elif gate == 2:
        cx = base.ARENA_W - half_short - EXTRA_WALL_EDGE_INSET_M
        cy = upper_y - center_offset
    else:
        raise ValueError(f"extra_wall_gate must be 1 or 2, got {gate}")
    return base.Rect(cx, cy, half_long, half_short, math.pi / 2.0, "wall_inner_3", "brick")


def _candidate(
    rng: random.Random,
    object_order: tuple[str, str],
    margin_r: float,
    extra_wall_gate: int,
) -> dict:
    y1 = rng.uniform(0.225, 0.255)
    y2 = rng.uniform(0.515, 0.545)
    statics = [
        base._brick(base.BRICK_HALF[0], y1, 0.0, 1),
        base._brick(base.ARENA_W - base.BRICK_HALF[0], y2, 0.0, 2),
        _extra_divider_wall(extra_wall_gate, y1, y2),
    ]
    k1 = _blocker(object_order[0], "left", y1, rng, 0)
    k2 = _blocker(object_order[1], "right", y2, rng, 1)
    middle_y = (y1 + y2) / 2.0
    scene = {
        "statics": statics,
        "blocker": k1,
        "blocker_name": object_order[0],
        "blockers": [(k1, object_order[0]), (k2, object_order[1])],
        "start": (rng.uniform(0.10, 0.18), rng.uniform(0.085, 0.115)),
        "middle_anchor": (rng.uniform(0.22, 0.30), middle_y),
        "goal": (rng.uniform(0.31, 0.39), rng.uniform(0.665, 0.705)),
        "margin_r": margin_r,
        "extra_wall_gate": extra_wall_gate,
    }
    _populate_gate_metrics(scene)
    return scene


def sample_scene(
    *,
    seed: int,
    object_order: tuple[str, str],
    extra_wall_gate: int = 1,
    margin_cm: float = 10.0,
    tries: int = 200,
) -> dict:
    """Return one statically certified candidate for an exact movable order."""
    if tuple(sorted(object_order)) != tuple(sorted(base.ON_TABLE)) or len(set(object_order)) != 2:
        raise ValueError(f"object_order must use obj_1 and obj_4 exactly once, got {object_order}")
    if extra_wall_gate not in (1, 2):
        raise ValueError(f"extra_wall_gate must be 1 or 2, got {extra_wall_gate}")
    margin_r = margin_cm / 200.0 + base.TIER1_MARGIN
    rng = random.Random(seed)
    reasons: dict[str, int] = {}
    for _attempt in range(tries):
        scene = _candidate(rng, object_order, margin_r, extra_wall_gate)
        ok, reason = _validate_scene(scene)
        if ok:
            return scene
        reasons[reason] = reasons.get(reason, 0) + 1
    raise RuntimeError(f"failed to sample a two-hop scene after {tries} tries: {reasons}")


def to_xml(scene: dict) -> str:
    return base.to_xml(scene)


def to_build_sheet(scene: dict, scene_id: str, *, profile: str, seed: int) -> dict:
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {sorted(PROFILES)}")
    sheet = base.to_build_sheet(scene, scene_id)
    object_order = [name for _rect, name in scene["blockers"]]
    blockers = [rect for rect, _name in scene["blockers"]]
    target = dict(PROFILES[profile])
    if len(scene["statics"]) == 4:
        wall_markers = WALL_MARKERS_4
    elif len(scene["statics"]) == 3:
        wall_markers = WALL_MARKERS_3
    elif len(scene["statics"]) == 2:
        wall_markers = WALL_MARKERS_2
    else:
        raise ValueError(f"unsupported wall count {len(scene['statics'])}; expected 2, 3, or 4")
    sheet["bricks"] = [
        {
            "marker_hint": marker,
            "center_cm": [round(rect.cx * 100.0, 1), round(rect.cy * 100.0, 1)],
            "long_axis_bearing_deg": base.long_axis_bearing_deg(rect.hx, rect.hy, rect.yaw),
            "yaw_deg": round(math.degrees(rect.yaw) % 180.0, 1),
            "long_cm": round(max(rect.hx, rect.hy) * 200.0, 1),
            "short_cm": round(min(rect.hx, rect.hy) * 200.0, 1),
            "height_cm": 10.0,
        }
        for marker, rect in zip(wall_markers, scene["statics"])
    ]
    source_modules = scene.get("source_modules") or []
    sheet.update(
        generator="gen_real_buildable_twohop.py",
        generator_seed=seed,
        object_order=object_order,
        requested_profile=profile,
        classification_status="needs_revalidation" if source_modules else "unlabelled",
        classification_note=(
            "Source-module labels steer composition but do not transfer automatically. Label K1 "
            "exhaustively on this XML; materialize the canonical post-K1 state; then label K2 "
            "exhaustively on that XML."
        ),
        topology_target={
            "hop_count": 2,
            "boundary_objects": [["obstacle_0_movable"], ["obstacle_1_movable"]],
        },
        middle_anchor_cm=[round(v * 100.0, 1) for v in scene["middle_anchor"]],
        gate_center_separation_cm=round(
            math.dist(
                (blockers[0].cx, blockers[0].cy),
                (blockers[1].cx, blockers[1].cy),
            )
            * 100.0,
            1,
        ),
        independence_tolerance={
            "non_target_position_mm": 2.0,
            "non_target_angle_deg": 1.0,
        },
        gates=[
            {
                "index": index + 1,
                "boundary_object": f"obstacle_{index}_movable",
                "physical_object": object_order[index],
                "target": dict(target),
                "proxy_open_frac": round(scene["gate_metrics"][index]["open_frac"], 4),
                "proxy_reachable_contacts": scene["gate_metrics"][index]["n_contacts"],
            }
            for index in range(2)
        ],
        counterfactual_trace=counterfactual_trace(scene, scene["margin_r"]),
    )
    if "extra_wall_gate" in scene:
        sheet["extra_wall_gate"] = scene["extra_wall_gate"]
        sheet["tag_convention"] = (
            "wall_9 is a wall_10 clone with the same 19.5 cm by 5.5 cm dimensions and tag "
            "mounting convention. Place every wall by long_axis_bearing_deg."
        )
    if source_modules:
        sheet["gate_target_y_cm"] = [round(value * 100.0, 1) for value in scene["gate_target_y"]]
        sheet["module_shift_x_cm"] = [
            round(value * 100.0, 1) for value in scene.get("module_shift_x", (0.0, 0.0))
        ]
        sheet["source_modules"] = [
            {
                "gate": index + 1,
                "build_id": module.build_id,
                "physical_object": module.object_name,
                "source_label": dict(module.source_label),
            }
            for index, module in enumerate(source_modules)
        ]
        sheet["tag_convention"] = (
            "four configured bars are assigned deterministically as wall_9, wall_10, wall_11, "
            "and wall_12. wall_9 is a wall_10 clone with the same 19.5 cm by 5.5 cm dimensions "
            "and tag mounting convention. Place every wall by long_axis_bearing_deg."
        )
    return sheet


def write_pool(
    out_dir: str | Path,
    *,
    num: int,
    seed: int,
    profile: str,
    margin_cm: float,
    donor_csv: str | Path | None = None,
    lower_build_id: str | None = None,
    upper_build_id: str | None = None,
) -> dict:
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {sorted(PROFILES)}")
    if num < 1:
        raise ValueError("num must be positive")
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    manifests: list[str] = []
    sheets: list[dict] = []
    if donor_csv is not None:
        donors = load_donors(donor_csv, profile)
        scenes = donor_compositions(
            donors,
            margin_cm=margin_cm,
            lower_build_id=lower_build_id,
            upper_build_id=upper_build_id,
        )
        scenes = filter_donor_compositions(
            scenes,
            lower_build_id=lower_build_id,
            upper_build_id=upper_build_id,
        )
        random.Random(seed).shuffle(scenes)
        if len(scenes) < num:
            raise RuntimeError(
                f"requested {num} scenes but only {len(scenes)} statically valid compositions "
                f"exist among {len(donors)} matching two-brick donors"
            )
        selected = [(seed, scene) for scene in scenes[:num]]
        candidate_source = "measured_v2_modules"
    else:
        selected = []
        for index in range(num):
            scene_seed = seed + index * 1009
            object_order = OBJECT_ORDERS[index % len(OBJECT_ORDERS)]
            extra_wall_gate = index % 2 + 1
            selected.append(
                (
                    scene_seed,
                    sample_scene(
                        seed=scene_seed,
                        object_order=object_order,
                        extra_wall_gate=extra_wall_gate,
                        margin_cm=margin_cm,
                    ),
                )
            )
        candidate_source = "procedural_topology_smoke"

    for index, (scene_seed, scene) in enumerate(selected):
        if "gate_metrics" not in scene:
            _populate_gate_metrics(scene)
        scene_id = f"twohop_{index:05d}"
        scene_dir = root / scene_id
        scene_dir.mkdir(parents=True, exist_ok=True)
        xml_path = scene_dir / "env.xml"
        xml_path.write_text(to_xml(scene), encoding="utf-8")
        manifests.append(str(xml_path.resolve()))
        sheets.append(to_build_sheet(scene, scene_id, profile=profile, seed=scene_seed))

    (root / "manifest.txt").write_text("".join(f"{path}\n" for path in manifests), encoding="utf-8")
    (root / "build_sheets.json").write_text(
        json.dumps(sheets, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    summary = {
        "emitted": len(manifests),
        "profile": profile,
        "seed": seed,
        "margin_cm": margin_cm,
        "object_orders": [list(order) for order in OBJECT_ORDERS],
        "extra_wall_gates": [1, 2] if donor_csv is None else None,
        "candidate_source": candidate_source,
        "donor_csv": str(Path(donor_csv).resolve()) if donor_csv is not None else None,
        "lower_build_id": lower_build_id,
        "upper_build_id": upper_build_id,
        "classification_status": "needs_revalidation" if donor_csv is not None else "unlabelled",
    }
    (root / "generation_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--num", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--profile", choices=sorted(PROFILES), required=True)
    parser.add_argument("--margin-cm", type=float, default=10.0)
    parser.add_argument(
        "--donor-csv",
        required=True,
        help="matching v2 build-sheet CSV (1push/hard or hmax2/med); only exact labelled "
        "two-brick modules are composed",
    )
    parser.add_argument(
        "--lower-build-id",
        help="optional ordered donor build ID to use only for K1",
    )
    parser.add_argument(
        "--upper-build-id",
        help="optional ordered donor build ID to use only for K2",
    )
    args = parser.parse_args()
    summary = write_pool(
        args.out_dir,
        num=args.num,
        seed=args.seed,
        profile=args.profile,
        margin_cm=args.margin_cm,
        donor_csv=args.donor_csv,
        lower_build_id=args.lower_build_id,
        upper_build_id=args.upper_build_id,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
