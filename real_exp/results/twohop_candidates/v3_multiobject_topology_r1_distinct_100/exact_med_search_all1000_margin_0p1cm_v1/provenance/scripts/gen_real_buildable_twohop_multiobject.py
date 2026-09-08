#!/usr/bin/env python3
"""Compose real-table two-hop scenes with one measured two-object gate.

Every scene uses four real bricks, one obj_1, and two distinct physical copies
of obj_4.  Donor geometry moves only as a rigid module; source difficulty is a
search hint and every composed scene remains explicitly marked for revalidation.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
import random
import shutil
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

import gen_real_buildable_scenes as base
import gen_real_buildable_twohop as legacy
import real_twohop_multiobject as multi


MIN_GATE_SEPARATION_M = 0.24
WALL_MARKERS = ("wall_9", "wall_10", "wall_11", "wall_12")
DEFAULT_PROTOCOL = {
    "composition": {
        "lower_gate_y": [0.22, 0.24, 0.26, 0.28],
        "upper_gate_y": [0.49, 0.51, 0.53, 0.55],
    }
}


def _clone_rect(
    rect: base.Rect,
    *,
    name: str,
    kind: str | None = None,
    mirror_x: bool = False,
) -> base.Rect:
    return base.Rect(
        base.ARENA_W - rect.cx if mirror_x else rect.cx,
        rect.cy,
        rect.hx,
        rect.hy,
        math.pi - rect.yaw if mirror_x else rect.yaw,
        name,
        kind or rect.kind,
    )


def _single_rect(
    row: Mapping[str, Any],
    name: str,
    target_y: float,
    opener_y: float,
) -> base.Rect:
    dy = target_y - opener_y
    return legacy._row_rect(dict(row), name, dy)


def _single_anchor(
    donor: legacy.DonorModule,
    key: str,
    target_y: float,
) -> tuple[float, float]:
    dy = target_y - float(donor.blocker["centre_y_cm"]) / 100.0
    return (
        float(donor.blocker[f"{key}_x_cm"]) / 100.0,
        float(donor.blocker[f"{key}_y_cm"]) / 100.0 + dy,
    )


def _mirror_point(point: tuple[float, float], enabled: bool) -> tuple[float, float]:
    return (base.ARENA_W - point[0], point[1]) if enabled else point


def _validate_three_object_geometry(scene: Mapping[str, Any]) -> tuple[bool, str]:
    statics = list(scene["statics"])
    blockers = [rect for rect, _shape in scene["blockers"]]
    if len(statics) != 4:
        return False, "wall_count"
    if len(blockers) != 3:
        return False, "movable_count"
    k1, k2, helper = blockers
    if math.dist((k1.cx, k1.cy), (k2.cx, k2.cy)) < MIN_GATE_SEPARATION_M:
        return False, "gate_separation"

    all_rects = statics + blockers
    for index, rect in enumerate(all_rects):
        if not base.inside_arena(rect):
            return False, f"outside_arena_{index}"
        for other in all_rects[index + 1 :]:
            if base.overlaps(rect, other, pad=0.004):
                return False, "physical_overlap"
    return True, "ok"


def validate_three_object_scene(
    scene: Mapping[str, Any],
    *,
    geometry_checked: bool = False,
) -> tuple[bool, str]:
    """Check inventory, placement, and the exact ordered topology contract."""
    statics = list(scene["statics"])
    blockers = [rect for rect, _shape in scene["blockers"]]
    if not geometry_checked:
        geometry_ok, geometry_reason = _validate_three_object_geometry(scene)
        if not geometry_ok:
            return False, geometry_reason
    all_rects = statics + blockers

    for anchor_name in ("start", "middle_anchor", "goal"):
        anchor = scene[anchor_name]
        if not base.inside_arena(
            base.Rect(
                anchor[0],
                anchor[1],
                base.ROBOT_HALF_X,
                base.ROBOT_HALF_Y,
                0.0,
                anchor_name,
                "robot",
            )
        ):
            return False, f"{anchor_name}_outside_arena"
        if any(base._surface_gap(anchor, rect) < base.ROBOT_CIRCUMSCRIBED_R for rect in all_rects):
            return False, f"{anchor_name}_not_placeable"

    expected = {
        "initial": (False, False, False),
        "post_k1": (True, False, False),
        "remove_auxiliary_only": (False, False, False),
        "remove_k2_only": (False, True, False),
        "both_openers_removed": (True, True, True),
    }
    trace = multi.three_object_counterfactual_trace(scene, float(scene["margin_r"]))
    for state, wanted in expected.items():
        if trace[state] != wanted:
            return False, f"topology_{state}"
    if not trace["margin_both_openers_removed"][2]:
        return False, "no_margin"
    return True, "ok"


def _anchor_candidates(
    source: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    lower_gate_y: float,
    upper_gate_y: float,
) -> Iterable[tuple[tuple[float, float], tuple[float, float], tuple[float, float]]]:
    """Yield donor anchors first, then a deterministic lattice inside the three rooms."""
    yield source
    x_options = (0.08, 0.16, base.ARENA_W / 2.0, 0.33, 0.41)
    start_y = max(0.075, lower_gate_y - 0.13)
    middle_y = 0.5 * (lower_gate_y + upper_gate_y)
    goal_y = min(base.ARENA_H - 0.075, upper_gate_y + 0.15)
    for start_x, middle_x, goal_x in itertools.product(x_options, repeat=3):
        yield (
            (start_x, start_y),
            (middle_x, middle_y),
            (goal_x, goal_y),
        )


def compose_three_object_scene(
    multi_donor: multi.MultiObjectDonor,
    single_donor: legacy.DonorModule,
    *,
    multi_gate: str,
    lower_gate_y: float,
    upper_gate_y: float,
    mirror_multi: bool,
    margin_cm: float,
) -> dict[str, Any] | None:
    """Rigidly compose one multi-object and one cloned-obj_4 gate."""
    if multi_gate not in {"k1", "k2"}:
        raise ValueError("multi_gate must be 'k1' or 'k2'")
    if single_donor.object_name != "obj_4":
        return None
    multi_target_y = lower_gate_y if multi_gate == "k1" else upper_gate_y
    single_target_y = upper_gate_y if multi_gate == "k1" else lower_gate_y
    single_opener_y = float(single_donor.blocker["centre_y_cm"]) / 100.0

    multi_walls = [
        multi.transform_rect(
            wall,
            target_y=multi_target_y,
            opener_y=multi_donor.target.cy,
            mirror_x=mirror_multi,
            name="pending",
            kind="brick",
        )
        for wall in multi_donor.walls
    ]
    single_walls = [
        _single_rect(row, "pending", single_target_y, single_opener_y)
        for row in single_donor.bricks
    ]
    lower_walls, upper_walls = (
        (multi_walls, single_walls) if multi_gate == "k1" else (single_walls, multi_walls)
    )
    statics = [
        _clone_rect(rect, name=f"wall_inner_{index + 1}")
        for index, rect in enumerate(lower_walls + upper_walls)
    ]

    multi_opener = multi.transform_rect(
        multi_donor.target,
        target_y=multi_target_y,
        opener_y=multi_donor.target.cy,
        mirror_x=mirror_multi,
        name="pending",
        kind=multi_donor.target.physical_shape,
    )
    helper = multi.transform_rect(
        multi_donor.auxiliary,
        target_y=multi_target_y,
        opener_y=multi_donor.target.cy,
        mirror_x=mirror_multi,
        name="obstacle_2_movable",
        kind=multi_donor.auxiliary.physical_shape,
    )
    single_opener = _single_rect(
        single_donor.blocker,
        "pending",
        single_target_y,
        single_opener_y,
    )
    if multi_gate == "k1":
        k1, k2 = multi_opener, single_opener
        gate_members = [
            ["obstacle_0_movable", "obstacle_2_movable"],
            ["obstacle_1_movable"],
        ]
    else:
        k1, k2 = single_opener, multi_opener
        gate_members = [
            ["obstacle_0_movable"],
            ["obstacle_1_movable", "obstacle_2_movable"],
        ]
    k1 = _clone_rect(k1, name="obstacle_0_movable")
    k2 = _clone_rect(k2, name="obstacle_1_movable")

    multi_start = multi.transform_rect(
        multi.SourceRect("start", *multi_donor.robot_start, 0.0, 0.0, 0.0),
        target_y=multi_target_y,
        opener_y=multi_donor.target.cy,
        mirror_x=mirror_multi,
    )
    multi_goal = multi.transform_rect(
        multi.SourceRect("goal", *multi_donor.goal, 0.0, 0.0, 0.0),
        target_y=multi_target_y,
        opener_y=multi_donor.target.cy,
        mirror_x=mirror_multi,
    )
    single_start = _single_anchor(single_donor, "robot_start", single_target_y)
    single_goal = _single_anchor(single_donor, "goal", single_target_y)
    if multi_gate == "k1":
        start = (multi_start.cx, multi_start.cy)
        middle = single_start
        goal = single_goal
    else:
        start = single_start
        middle = (multi_start.cx, multi_start.cy)
        goal = (multi_goal.cx, multi_goal.cy)

    shapes = (k1.kind, k2.kind, helper.kind)
    if sorted(shapes) != ["obj_1", "obj_4", "obj_4"]:
        return None
    physical_roles: dict[str, str] = {}
    for index, shape in enumerate(shapes):
        sim_id = f"obstacle_{index}_movable"
        if shape == "obj_1":
            physical_roles[sim_id] = "obj_1"
        elif index == 2 or (multi_gate == "k1" and index == 0) or (multi_gate == "k2" and index == 1):
            physical_roles[sim_id] = "obj_4a"
        else:
            physical_roles[sim_id] = "obj_4b"

    scene: dict[str, Any] = {
        "statics": statics,
        "blocker": k1,
        "blocker_name": shapes[0],
        "blockers": [(k1, shapes[0]), (k2, shapes[1]), (helper, shapes[2])],
        "start": start,
        "middle_anchor": middle,
        "goal": goal,
        "margin_r": margin_cm / 200.0 + base.TIER1_MARGIN,
        "gate_contract": {
            "boundary_objects": [["obstacle_0_movable"], ["obstacle_1_movable"]],
            "gate_members": gate_members,
        },
        "physical_roles": physical_roles,
        "multi_gate_index": 0 if multi_gate == "k1" else 1,
        "source_multiobject": multi.donor_to_json(multi_donor),
        "source_singleobject": {
            "build_id": single_donor.build_id,
            "physical_object": single_donor.object_name,
            "source_label": dict(single_donor.source_label),
        },
        "gate_target_y": [lower_gate_y, upper_gate_y],
        "mirror_multi": mirror_multi,
    }
    geometry_ok, _reason = _validate_three_object_geometry(scene)
    if not geometry_ok:
        return None
    source_anchors = (start, middle, goal)
    for candidate_start, candidate_middle, candidate_goal in _anchor_candidates(
        source_anchors,
        lower_gate_y,
        upper_gate_y,
    ):
        scene["start"] = candidate_start
        scene["middle_anchor"] = candidate_middle
        scene["goal"] = candidate_goal
        passed, _reason = validate_three_object_scene(scene, geometry_checked=True)
        if passed:
            return scene
    return None


def _q(value: float, resolution: float) -> int:
    return int(round(value / resolution))


def _angle_q(value: float) -> int:
    wrapped = math.atan2(math.sin(value), math.cos(value))
    return _q(math.degrees(wrapped), 0.1)


def _hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def exact_geometry_id(scene: Mapping[str, Any], profile: str) -> str:
    rows = [
        [rect.name, _q(rect.cx, 0.0001), _q(rect.cy, 0.0001), _q(rect.hx, 0.0001), _q(rect.hy, 0.0001), _angle_q(rect.yaw)]
        for rect in scene["statics"]
    ]
    rows.extend(
        [f"obstacle_{index}_movable", shape, _q(rect.cx, 0.0001), _q(rect.cy, 0.0001), _q(rect.hx, 0.0001), _q(rect.hy, 0.0001), _angle_q(rect.yaw)]
        for index, (rect, shape) in enumerate(scene["blockers"])
    )
    return _hash(
        {
            "profile": profile,
            "geometry": rows,
            "anchors": [[_q(x, 0.0001), _q(y, 0.0001)] for x, y in (scene["start"], scene["middle_anchor"], scene["goal"])],
            "gate_contract": scene["gate_contract"],
        }
    )


def normalized_layout_id(scene: Mapping[str, Any]) -> str:
    blockers = [rect for rect, _shape in scene["blockers"]]
    k1, k2, helper = blockers
    wall_rows = []
    for gate_index, (opener, walls) in enumerate(((k1, scene["statics"][:2]), (k2, scene["statics"][2:]))):
        wall_rows.append(
            [
                [_q(wall.cx - opener.cx, 0.0001), _q(wall.cy - opener.cy, 0.0001), _angle_q(wall.yaw)]
                for wall in walls
            ]
        )
    multi_opener = blockers[int(scene["multi_gate_index"])]
    return _hash(
        {
            "source_multi": [scene["source_multiobject"]["source_family"], scene["source_multiobject"]["source_scene_id"]],
            "source_single": scene["source_singleobject"]["build_id"],
            "multi_gate_index": scene["multi_gate_index"],
            "walls_relative": wall_rows,
            "helper_relative": [_q(helper.cx - multi_opener.cx, 0.0001), _q(helper.cy - multi_opener.cy, 0.0001), _angle_q(helper.yaw - multi_opener.yaw)],
            "gate_separation": [_q(k2.cx - k1.cx, 0.0001), _q(k2.cy - k1.cy, 0.0001)],
            "anchors_relative": [
                [_q(scene["start"][0] - k1.cx, 0.0001), _q(scene["start"][1] - k1.cy, 0.0001)],
                [_q(scene["middle_anchor"][0] - k1.cx, 0.0001), _q(scene["middle_anchor"][1] - k1.cy, 0.0001)],
                [_q(scene["goal"][0] - k2.cx, 0.0001), _q(scene["goal"][1] - k2.cy, 0.0001)],
            ],
        }
    )


def to_xml(scene: Mapping[str, Any]) -> str:
    return base.to_xml(scene)


def to_build_sheet(scene: Mapping[str, Any], scene_id: str, *, profile: str, seed: int) -> dict[str, Any]:
    cm = lambda value: round(value * 100.0, 1)
    movable_rows = []
    for index, (rect, shape) in enumerate(scene["blockers"]):
        sim_id = f"obstacle_{index}_movable"
        hx, hy, hz = base.MOVABLES[shape]
        movable_rows.append(
            {
                "sim_id": sim_id,
                "physical_role": scene["physical_roles"][sim_id],
                "source_shape": shape,
                "center_cm": [cm(rect.cx), cm(rect.cy)],
                "long_axis_bearing_deg": base.long_axis_bearing_deg(hx, hy, rect.yaw),
                "yaw_deg": round(math.degrees(rect.yaw) % 180.0, 1),
                "long_cm": round(max(hx, hy) * 200.0, 1),
                "short_cm": round(min(hx, hy) * 200.0, 1),
                "height_cm": round(hz * 200.0, 1),
            }
        )
    wall_rows = [
        {
            "sim_id": rect.name,
            "physical_id": marker,
            "center_cm": [cm(rect.cx), cm(rect.cy)],
            "long_axis_bearing_deg": base.long_axis_bearing_deg(rect.hx, rect.hy, rect.yaw),
            "yaw_deg": round(math.degrees(rect.yaw) % 180.0, 1),
            "long_cm": 19.5,
            "short_cm": 5.5,
            "height_cm": 10.0,
        }
        for marker, rect in zip(WALL_MARKERS, scene["statics"])
    ]
    return {
        "scene_id": scene_id,
        "generator": "gen_real_buildable_twohop_multiobject.py",
        "generator_seed": seed,
        "requested_profile": profile,
        "classification_status": "needs_revalidation",
        "classification_note": "Source labels steer composition only; measure K1 and K2 on composed states.",
        "arena_cm": [base.ARENA_W * 100.0, base.ARENA_H * 100.0],
        "robot_start_cm": [cm(value) for value in scene["start"]],
        "robot_start_bearing_deg": base.ROBOT_START_BEARING_DEG,
        "middle_anchor_cm": [cm(value) for value in scene["middle_anchor"]],
        "goal_cm": [cm(value) for value in scene["goal"]],
        "run_namo_goal_flag": f"--goal {cm(scene['goal'][0]):.1f} {cm(scene['goal'][1]):.1f}",
        "physical_roles": dict(scene["physical_roles"]),
        "movables": movable_rows,
        "gate_contract": scene["gate_contract"],
        "multi_gate_index": scene["multi_gate_index"],
        "gate_walls": wall_rows,
        "topology_target": {"hop_count": 2, "boundary_objects": scene["gate_contract"]["boundary_objects"]},
        "source_multiobject": scene["source_multiobject"],
        "source_singleobject": scene["source_singleobject"],
        "normalized_layout_id": normalized_layout_id(scene),
        "exact_geometry_id": exact_geometry_id(scene, profile),
        "counterfactual_trace": multi.three_object_counterfactual_trace(scene, scene["margin_r"]),
        "clone_readiness": "obj_4b requires a unique ArUco tag before real trials.",
    }


def _excluded_ids(manifests: Iterable[str | Path]) -> tuple[set[str], set[str]]:
    exact: set[str] = set()
    normalized: set[str] = set()
    for manifest in manifests:
        for line in Path(manifest).read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            sheet_path = Path(line.strip()).parent / "build_sheet.json"
            record = json.loads(sheet_path.read_text(encoding="utf-8"))
            exact.add(record["exact_geometry_id"])
            normalized.add(record["normalized_layout_id"])
    return exact, normalized


def write_pool(
    out_dir: str | Path,
    *,
    profile: str,
    multi_donor_manifest: str | Path,
    single_donor_csv: str | Path,
    capacity: int,
    seed: int,
    config: Mapping[str, Any] | None = None,
    exclude_manifests: Iterable[str | Path] = (),
) -> dict[str, Any]:
    if capacity < 1:
        raise ValueError("capacity must be positive")
    root = Path(out_dir)
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"refusing non-empty pool output: {root}")
    protocol = config or DEFAULT_PROTOCOL
    composition = protocol.get("composition", {})
    lower_options = tuple(composition.get("lower_gate_y", DEFAULT_PROTOCOL["composition"]["lower_gate_y"]))
    upper_options = tuple(composition.get("upper_gate_y", DEFAULT_PROTOCOL["composition"]["upper_gate_y"]))
    margin_cm = float(composition.get("margin_cm", 10.0))

    multi_rows = json.loads(Path(multi_donor_manifest).read_text(encoding="utf-8"))
    multi_donors = [multi.donor_from_json(row) for row in multi_rows]
    single_donors = [donor for donor in legacy.load_donors(single_donor_csv, profile) if donor.object_name == "obj_4"]
    pairs = list(itertools.product(multi_donors, single_donors))
    random.Random(seed).shuffle(pairs)
    excluded_exact, excluded_normalized = _excluded_ids(exclude_manifests)
    seen_exact = set(excluded_exact)
    seen_normalized = set(excluded_normalized)
    rejected: Counter[str] = Counter()
    selected: list[dict[str, Any]] = []
    variants = list(itertools.product(("k1", "k2"), (False, True), lower_options, upper_options))
    for donor, single in pairs:
        for multi_gate, mirror, lower_y, upper_y in variants:
            scene = compose_three_object_scene(
                donor,
                single,
                multi_gate=multi_gate,
                lower_gate_y=float(lower_y),
                upper_gate_y=float(upper_y),
                mirror_multi=mirror,
                margin_cm=margin_cm,
            )
            if scene is None:
                rejected["invalid_composition"] += 1
                continue
            exact_id = exact_geometry_id(scene, profile)
            normalized_id = normalized_layout_id(scene)
            if exact_id in excluded_exact or normalized_id in excluded_normalized:
                rejected["existing_geometry"] += 1
                continue
            if exact_id in seen_exact:
                rejected["exact_duplicate"] += 1
                continue
            if normalized_id in seen_normalized:
                rejected["translation_only_duplicate"] += 1
                continue
            seen_exact.add(exact_id)
            seen_normalized.add(normalized_id)
            selected.append(scene)
            break
        if len(selected) >= capacity:
            break

    temp = root.with_name(f".{root.name}.tmp-{os.getpid()}")
    if temp.exists():
        shutil.rmtree(temp)
    temp.mkdir(parents=True)
    sheets = []
    manifest_rows = []
    try:
        for index, scene in enumerate(selected):
            scene_id = f"twohop_multi_{index:05d}"
            scene_dir = temp / scene_id
            scene_dir.mkdir()
            (scene_dir / "env.xml").write_text(to_xml(scene), encoding="utf-8")
            sheet = to_build_sheet(scene, scene_id, profile=profile, seed=seed)
            (scene_dir / "build_sheet.json").write_text(json.dumps(sheet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            sheets.append(sheet)
            manifest_rows.append(str((root.resolve() / scene_id / "env.xml")))
        summary = {
            "profile": profile,
            "seed": seed,
            "capacity": capacity,
            "multi_donors": len(multi_donors),
            "source_hint_target_aligned": sum(
                donor.source_hint_target_aligned for donor in multi_donors
            ),
            "profile_hint_matched": sum(
                profile in donor.source_hint_profiles for donor in multi_donors
            ),
            "topology_only_for_profile": sum(
                profile not in donor.source_hint_profiles for donor in multi_donors
            ),
            "donor_pairs": len(pairs),
            "emitted": len(selected),
            "rejected": dict(sorted(rejected.items())),
            "classification_status": "needs_revalidation",
        }
        (temp / "manifest.txt").write_text("".join(f"{row}\n" for row in manifest_rows), encoding="utf-8")
        (temp / "build_sheets.json").write_text(json.dumps(sheets, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        (temp / "generation_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if root.exists():
            root.rmdir()
        os.replace(temp, root)
    except Exception:
        shutil.rmtree(temp, ignore_errors=True)
        raise
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--profile", required=True, choices=multi.PROFILES)
    parser.add_argument("--multi-donor-manifest", required=True, type=Path)
    parser.add_argument("--single-donor-csv", required=True, type=Path)
    parser.add_argument("--capacity", required=True, type=int)
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--exclude-manifest", action="append", default=[])
    args = parser.parse_args()
    protocol = yaml.safe_load(args.config.read_text(encoding="utf-8")) if args.config else None
    summary = write_pool(
        args.out_dir,
        profile=args.profile,
        multi_donor_manifest=args.multi_donor_manifest,
        single_donor_csv=args.single_donor_csv,
        capacity=args.capacity,
        seed=args.seed,
        config=protocol,
        exclude_manifests=args.exclude_manifest,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
