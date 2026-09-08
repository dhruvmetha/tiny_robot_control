#!/usr/bin/env python3
"""Choose translation- and mirror-aware diverse two-hop candidates for promotion."""

from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
import os
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml

from real_twohop_multiobject import normalize_gate_contract


DIMENSION_TOLERANCE_CM = 0.5
COMPUTE_SCRATCH_ROOT = Path("/scache/scratch")
LOGIN_SCRATCH_ROOT = Path("/scratch")
DEFAULT_DIVERSITY_THRESHOLDS = {
    "auxiliary_position_m": 0.05,
    "auxiliary_yaw_deg": 20.0,
    "wall_position_m": 0.05,
    "wall_yaw_deg": 15.0,
}
DISTINCTNESS_POLICY = {
    "categorical_topology_changes_are_distinct": True,
    "donor_ids": "soft_coverage_only",
    "gate_separation": "recorded_not_scored",
    "geometry_scope": "local_gate_geometry_only",
    "reflection": "one_global_left_right",
    "task_anchors": "recorded_not_scored",
    "wall_assignment": "best_permutation_within_gate",
}


def _angle_delta(left: float, right: float) -> float:
    difference = abs((left - right) % 180.0)
    return min(difference, 180.0 - difference)


def _dimensions(row: Mapping[str, Any]) -> list[float]:
    return [float(row[key]) for key in ("long_cm", "short_cm", "height_cm")]


def _relative_row(
    row: Mapping[str, Any],
    opener: Mapping[str, Any],
    *,
    category: str,
) -> dict[str, Any]:
    x, y = (float(value) for value in row["center_cm"])
    ox, oy = (float(value) for value in opener["center_cm"])
    yaw = float(row["yaw_deg"])
    opener_yaw = float(opener["yaw_deg"])
    return {
        "category": category,
        "dimensions_cm": _dimensions(row),
        "position_cm": [round(x - ox, 4), round(y - oy, 4)],
        "yaw_deg": round((yaw - opener_yaw) % 180.0, 4),
    }


def _source_value(source: Mapping[str, Any], plain: str, prefixed: str) -> str:
    value = source.get(plain, source.get(prefixed))
    if not isinstance(value, str) or not value:
        raise ValueError(f"source multi-object metadata lacks {prefixed}")
    return value


def layout_features(sheet: Mapping[str, Any]) -> dict[str, Any]:
    """Return provenance plus translation-free physical features for both gates."""
    contract = normalize_gate_contract(sheet.get("gate_contract"))
    multi_gate_index = int(sheet["multi_gate_index"])
    if multi_gate_index not in (0, 1):
        raise ValueError("multi_gate_index must be 0 or 1")
    movable_rows = sheet.get("movables")
    wall_rows = sheet.get("gate_walls")
    if not isinstance(movable_rows, list) or not isinstance(wall_rows, list):
        raise ValueError("build sheet lacks movable or gate-wall rows")
    by_sim_id = {row.get("sim_id"): row for row in movable_rows}
    if len(by_sim_id) != len(movable_rows):
        raise ValueError("movable rows require unique sim_id values")
    if wall_rows and isinstance(wall_rows[0], list):
        walls_by_gate = wall_rows
    else:
        if len(wall_rows) != 4:
            raise ValueError("flat gate_walls must contain exactly four rows")
        walls_by_gate = [wall_rows[:2], wall_rows[2:]]
    if len(walls_by_gate) != 2 or any(len(rows) != 2 for rows in walls_by_gate):
        raise ValueError("each gate must contain exactly two wall rows")

    gates = []
    openers = []
    for gate_index in range(2):
        opener_id = contract["boundary_objects"][gate_index][0]
        try:
            opener = by_sim_id[opener_id]
            members = [
                by_sim_id[object_id]
                for object_id in contract["gate_members"][gate_index]
                if object_id != opener_id
            ]
        except KeyError as exc:
            raise ValueError("gate contract references a missing movable row") from exc
        openers.append(opener)
        gates.append(
            {
                "opener": {
                    "category": str(opener.get("source_shape", "")),
                    "dimensions_cm": _dimensions(opener),
                    "yaw_deg": round(float(opener["yaw_deg"]) % 180.0, 4),
                },
                "auxiliaries": sorted(
                    (
                        _relative_row(
                            member,
                            opener,
                            category=str(member.get("source_shape", "")),
                        )
                        for member in members
                    ),
                    key=_feature_sort_key,
                ),
                "walls": sorted(
                    (
                        _relative_row(row, opener, category="wall")
                        for row in walls_by_gate[gate_index]
                    ),
                    key=_feature_sort_key,
                ),
            }
        )

    origin_x, origin_y = (float(value) for value in openers[0]["center_cm"])

    def relative_point(raw: Any, *, context: str) -> list[float]:
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            raise ValueError(f"build sheet lacks {context}")
        x, y = (float(value) for value in raw)
        return [round(x - origin_x, 4), round(y - origin_y, 4)]

    source = sheet.get("source_multiobject")
    single = sheet.get("source_singleobject")
    if not isinstance(source, Mapping) or not isinstance(single, Mapping):
        raise ValueError("build sheet lacks donor provenance")
    return {
        "source_family": _source_value(source, "family", "source_family"),
        "source_scene_id": _source_value(source, "scene_id", "source_scene_id"),
        "single_donor": str(single["build_id"]),
        "multi_gate_index": multi_gate_index,
        "gates": gates,
        "task_points": {
            "gate_1_opener": relative_point(
                openers[1]["center_cm"],
                context="second gate opener",
            ),
            "robot_start": relative_point(
                sheet.get("robot_start_cm"),
                context="robot_start_cm",
            ),
            "middle_anchor": relative_point(
                sheet.get("middle_anchor_cm"),
                context="middle_anchor_cm",
            ),
            "goal": relative_point(sheet.get("goal_cm"), context="goal_cm"),
        },
    }


def donor_pair_id(sheet: Mapping[str, Any]) -> str:
    features = layout_features(sheet)
    return "/".join(
        (
            features["source_family"],
            features["source_scene_id"],
            features["single_donor"],
        )
    )


def _feature_sort_key(feature: Mapping[str, Any]) -> str:
    return json.dumps(feature, sort_keys=True, separators=(",", ":"))


def _physical_features(features: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "multi_gate_index": features["multi_gate_index"],
        "gates": features["gates"],
    }


def _reflected(features: Mapping[str, Any]) -> dict[str, Any]:
    """Reflect one complete physical layout left/right in a single operation."""
    reflected = copy.deepcopy(features)
    for gate in reflected["gates"]:
        gate["opener"]["yaw_deg"] = (-gate["opener"]["yaw_deg"]) % 180.0
        for collection in (gate["auxiliaries"], gate["walls"]):
            for feature in collection:
                feature["position_cm"][0] = -feature["position_cm"][0]
                feature["yaw_deg"] = (-feature["yaw_deg"]) % 180.0
            collection.sort(key=_feature_sort_key)
    return reflected


def mirror_family_id(sheet: Mapping[str, Any]) -> str:
    features = _physical_features(layout_features(sheet))
    serializations = [
        json.dumps(value, sort_keys=True, separators=(",", ":"))
        for value in (features, _reflected(features))
    ]
    return hashlib.sha256(min(serializations).encode("utf-8")).hexdigest()


def _empty_difference() -> dict[str, float]:
    return {
        "categorical_mismatch": 0.0,
        "dimension_cm": 0.0,
        "auxiliary_position_cm": 0.0,
        "auxiliary_yaw_deg": 0.0,
        "wall_position_cm": 0.0,
        "wall_yaw_deg": 0.0,
    }


def _merge_max(target: dict[str, float], values: Mapping[str, float]) -> None:
    for key, value in values.items():
        target[key] = max(target[key], value)


def _row_difference(
    left: Mapping[str, Any],
    right: Mapping[str, Any],
    *,
    position_key: str,
    yaw_key: str,
) -> dict[str, float]:
    result = _empty_difference()
    result["categorical_mismatch"] = float(left["category"] != right["category"])
    result["dimension_cm"] = max(
        abs(a - b)
        for a, b in zip(left["dimensions_cm"], right["dimensions_cm"])
    )
    result[position_key] = math.dist(left["position_cm"], right["position_cm"])
    result[yaw_key] = _angle_delta(left["yaw_deg"], right["yaw_deg"])
    return result


def _collection_pairings(
    left: list[Mapping[str, Any]],
    right: list[Mapping[str, Any]],
) -> Iterable[tuple[tuple[Mapping[str, Any], Mapping[str, Any]], ...]]:
    if len(left) != len(right):
        return ()
    return (
        tuple(zip(left, permutation))
        for permutation in itertools.permutations(right)
    )


def _difference_options(
    left_sheet: Mapping[str, Any],
    right_sheet: Mapping[str, Any],
) -> list[dict[str, float]]:
    left = _physical_features(layout_features(left_sheet))
    right = _physical_features(layout_features(right_sheet))
    options = []
    for oriented_right in (right, _reflected(right)):
        base_difference = _empty_difference()
        base_difference["categorical_mismatch"] = float(
            left["multi_gate_index"] != oriented_right["multi_gate_index"]
        )
        gate_pairing_options: list[list[dict[str, float]]] = []
        for left_gate, right_gate in zip(left["gates"], oriented_right["gates"]):
            gate_difference = _empty_difference()
            gate_difference["categorical_mismatch"] = float(
                left_gate["opener"]["category"]
                != right_gate["opener"]["category"]
            )
            gate_difference["dimension_cm"] = max(
                abs(a - b)
                for a, b in zip(
                    left_gate["opener"]["dimensions_cm"],
                    right_gate["opener"]["dimensions_cm"],
                )
            )
            gate_difference["wall_yaw_deg"] = _angle_delta(
                left_gate["opener"]["yaw_deg"],
                right_gate["opener"]["yaw_deg"],
            )
            if len(left_gate["walls"]) != len(right_gate["walls"]) or len(
                left_gate["auxiliaries"]
            ) != len(right_gate["auxiliaries"]):
                gate_difference["categorical_mismatch"] = 1.0
                gate_pairing_options.append([gate_difference])
                continue

            gate_options = []
            wall_pairings = _collection_pairings(
                left_gate["walls"],
                right_gate["walls"],
            )
            auxiliary_pairings = _collection_pairings(
                left_gate["auxiliaries"],
                right_gate["auxiliaries"],
            )
            for wall_pairs, auxiliary_pairs in itertools.product(
                wall_pairings,
                auxiliary_pairings,
            ):
                difference = dict(gate_difference)
                for left_row, right_row in wall_pairs:
                    _merge_max(
                        difference,
                        _row_difference(
                            left_row,
                            right_row,
                            position_key="wall_position_cm",
                            yaw_key="wall_yaw_deg",
                        ),
                    )
                for left_row, right_row in auxiliary_pairs:
                    _merge_max(
                        difference,
                        _row_difference(
                            left_row,
                            right_row,
                            position_key="auxiliary_position_cm",
                            yaw_key="auxiliary_yaw_deg",
                        ),
                    )
                gate_options.append(difference)
            gate_pairing_options.append(gate_options)

        for gate_differences in itertools.product(*gate_pairing_options):
            difference = dict(base_difference)
            for gate_difference in gate_differences:
                _merge_max(difference, gate_difference)
            options.append(difference)
    return options


def _difference_score(
    difference: Mapping[str, float],
    thresholds: Mapping[str, Any],
) -> float:
    if difference["categorical_mismatch"]:
        return math.inf
    scales = {
        "dimension_cm": DIMENSION_TOLERANCE_CM,
        "auxiliary_position_cm": 100.0 * float(thresholds["auxiliary_position_m"]),
        "auxiliary_yaw_deg": float(thresholds["auxiliary_yaw_deg"]),
        "wall_position_cm": 100.0 * float(thresholds["wall_position_m"]),
        "wall_yaw_deg": float(thresholds["wall_yaw_deg"]),
    }
    return max(difference[key] / scale for key, scale in scales.items())


def difference_components(
    left_sheet: Mapping[str, Any],
    right_sheet: Mapping[str, Any],
    thresholds: Mapping[str, Any] = DEFAULT_DIVERSITY_THRESHOLDS,
) -> dict[str, float]:
    """Return the best whole-layout match across wall assignments and reflection."""
    return min(
        _difference_options(left_sheet, right_sheet),
        key=lambda difference: _difference_score(difference, thresholds),
    )


def is_meaningfully_distinct(
    left_sheet: Mapping[str, Any],
    right_sheet: Mapping[str, Any],
    thresholds: Mapping[str, Any],
) -> bool:
    """Return whether every valid wall/reflection match exceeds a threshold."""
    return min(
        _difference_score(difference, thresholds)
        for difference in _difference_options(left_sheet, right_sheet)
    ) > 1.0


def _normalized_distance(
    left_sheet: Mapping[str, Any],
    right_sheet: Mapping[str, Any],
    thresholds: Mapping[str, Any],
) -> float:
    return min(
        _difference_score(difference, thresholds)
        for difference in _difference_options(left_sheet, right_sheet)
    )


def _tie_key(row: Mapping[str, Any]) -> tuple[Any, ...]:
    features = layout_features(row["sheet"])
    return (
        int(row.get("simulator_calls", 0)),
        features["source_family"],
        features["source_scene_id"],
        str(row["sheet"].get("exact_geometry_id", "")),
        str(row["scene_id"]),
    )


def select_diverse(
    candidate_rows: Iterable[Mapping[str, Any]],
    *,
    limit: int,
    thresholds: Mapping[str, Any],
) -> list[Mapping[str, Any]]:
    """Select deterministic farthest-first rows with family and donor-pair coverage."""
    if limit < 1:
        raise ValueError("limit must be positive")
    remaining = sorted(list(candidate_rows), key=_tie_key)
    if not remaining:
        return []
    selected = [remaining.pop(0)]
    while remaining and len(selected) < limit:
        viable = [
            row
            for row in remaining
            if all(
                is_meaningfully_distinct(row["sheet"], chosen["sheet"], thresholds)
                for chosen in selected
            )
        ]
        if not viable:
            break
        used_families = {
            layout_features(row["sheet"])["source_family"] for row in selected
        }
        unused_family = [
            row
            for row in viable
            if layout_features(row["sheet"])["source_family"] not in used_families
        ]
        if unused_family:
            viable = unused_family
        else:
            used_pairs = {donor_pair_id(row["sheet"]) for row in selected}
            unused_pair = [
                row for row in viable if donor_pair_id(row["sheet"]) not in used_pairs
            ]
            if unused_pair:
                viable = unused_pair
        scored = [
            (
                min(
                    _normalized_distance(row["sheet"], chosen["sheet"], thresholds)
                    for chosen in selected
                ),
                row,
            )
            for row in viable
        ]
        best_distance = max(distance for distance, _row in scored)
        best = min(
            (row for distance, row in scored if math.isclose(distance, best_distance)),
            key=_tie_key,
        )
        selected.append(best)
        remaining.remove(best)
    return selected


def _shared_path(path: str | Path) -> Path:
    """Resolve a record path, translating Amarel's compute-node scratch alias."""
    candidate = Path(path).expanduser()
    if candidate.exists():
        return candidate.resolve()
    try:
        relative = candidate.relative_to(COMPUTE_SCRATCH_ROOT)
    except ValueError:
        return candidate.resolve()
    shared = LOGIN_SCRATCH_ROOT / relative
    return shared.resolve() if shared.exists() else candidate.resolve()


def _build_sheet_path(path: str | Path) -> Path:
    candidate = _shared_path(path)
    if candidate.is_dir():
        candidate = candidate / "build_sheet.json"
    elif candidate.name != "build_sheet.json":
        candidate = candidate.parent / "build_sheet.json"
    if not candidate.is_file():
        raise FileNotFoundError(f"missing prior build sheet: {candidate}")
    return candidate.resolve()


def _is_legacy_two_object_sheet(record: Mapping[str, Any]) -> bool:
    return bool(
        record.get("generator") == "gen_real_buildable_twohop.py"
        and isinstance(record.get("object_order"), list)
        and len(record["object_order"]) == 2
        and isinstance(record.get("gates"), list)
        and len(record["gates"]) == 2
    )


def load_excluded_sheets(
    inputs: Iterable[str | Path],
) -> tuple[list[dict[str, Any]], int]:
    """Load compatible prior sheets and count known legacy two-object sheets."""
    sheet_paths = []
    for raw_path in inputs:
        path = _shared_path(raw_path)
        if path.is_dir() or path.name in {"build_sheet.json", "env.xml"}:
            sheet_paths.append(_build_sheet_path(path))
            continue
        if not path.is_file():
            raise FileNotFoundError(f"missing prior selection input: {path}")
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                entry = Path(line.strip()).expanduser()
                if not entry.is_absolute():
                    entry = path.parent / entry
                sheet_paths.append(_build_sheet_path(entry))

    sheets = []
    incompatible = 0
    seen = set()
    for sheet_path in sheet_paths:
        if sheet_path in seen:
            continue
        seen.add(sheet_path)
        record = json.loads(sheet_path.read_text(encoding="utf-8"))
        if _is_legacy_two_object_sheet(record):
            incompatible += 1
            continue
        layout_features(record)
        sheets.append(record)
    return sheets, incompatible


def load_passing_candidates(records_root: str | Path) -> tuple[list[dict[str, Any]], dict[str, int]]:
    rows = []
    rejected: Counter[str] = Counter()
    exact_ids: set[str] = set()
    for record_path in sorted(Path(records_root).rglob("*.json")):
        try:
            record = json.loads(record_path.read_text(encoding="utf-8"))
            if record.get("status") != "passed":
                rejected["screen_not_passed"] += 1
                continue
            xml_path = _shared_path(record["xml_path"])
            sheet = json.loads((xml_path.parent / "build_sheet.json").read_text(encoding="utf-8"))
            exact_id = sheet["exact_geometry_id"]
            if exact_id in exact_ids:
                rejected["duplicate_geometry_id"] += 1
                continue
            exact_ids.add(exact_id)
            layout_features(sheet)
            rows.append(
                {
                    "scene_id": record["scene_id"],
                    "simulator_calls": int(record["simulator_calls"]),
                    "xml_path": str(xml_path),
                    "record_path": str(record_path.resolve()),
                    "sheet": sheet,
                }
            )
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
            rejected["malformed_candidate"] += 1
    return rows, dict(sorted(rejected.items()))


def write_selection(
    out_dir: str | Path,
    *,
    records_root: str | Path,
    limit: int,
    thresholds: Mapping[str, Any],
    exclusions: Iterable[str | Path] = (),
) -> dict[str, Any]:
    root = Path(out_dir)
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"refusing non-empty selection output: {root}")
    prior_sheets, prior_incompatible_topology = load_excluded_sheets(exclusions)
    candidates, rejected = load_passing_candidates(records_root)
    rejected_counts = Counter(rejected)
    retained = []
    for row in candidates:
        if any(
            not is_meaningfully_distinct(row["sheet"], prior, thresholds)
            for prior in prior_sheets
        ):
            rejected_counts["prior_near_duplicate"] += 1
        else:
            retained.append(row)
    candidates = retained
    rejected = dict(sorted(rejected_counts.items()))
    chosen = select_diverse(candidates, limit=limit, thresholds=thresholds)
    root.mkdir(parents=True, exist_ok=True)
    manifest = [str(row["xml_path"]) for row in chosen]
    audit = {
        "eligible": len(candidates),
        "selected": len(chosen),
        "limit": limit,
        "thresholds": dict(thresholds),
        "effective_thresholds": {
            **thresholds,
            "dimension_tolerance_cm": DIMENSION_TOLERANCE_CM,
        },
        "distinctness_policy": dict(DISTINCTNESS_POLICY),
        "prior_exclusions": len(prior_sheets) + prior_incompatible_topology,
        "prior_incompatible_topology": prior_incompatible_topology,
        "scenes": [
            {
                "scene_id": row["scene_id"],
                "xml_path": row["xml_path"],
                "record_path": row["record_path"],
                "source_family": layout_features(row["sheet"])["source_family"],
                "donor_pair_id": donor_pair_id(row["sheet"]),
                "mirror_family_id": mirror_family_id(row["sheet"]),
            }
            for row in chosen
        ],
    }
    temporary = root / ".selection.tmp"
    temporary.write_text(json.dumps(audit, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, root / "selection.json")
    (root / "promotion_manifest.txt").write_text(
        "".join(f"{path}\n" for path in manifest),
        encoding="utf-8",
    )
    (root / "rejected.json").write_text(
        json.dumps(rejected, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return audit


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--limit", required=True, type=int)
    parser.add_argument(
        "--exclude",
        "--exclude-manifest",
        action="append",
        default=[],
        help="prior build sheet, env directory/XML, or manifest; repeatable",
    )
    parser.add_argument(
        "--config",
        default=Path(__file__).resolve().parents[2] / "config" / "real_twohop_v3_multiobject.yaml",
        type=Path,
    )
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    summary = write_selection(
        args.out_dir,
        records_root=args.records,
        limit=args.limit,
        thresholds=config["diversity"],
        exclusions=args.exclude,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
