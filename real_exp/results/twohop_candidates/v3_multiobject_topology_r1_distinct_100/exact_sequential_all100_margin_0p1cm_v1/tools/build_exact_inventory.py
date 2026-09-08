#!/usr/bin/env python3
"""Build the run-specific 100-scene exact-label and navigation inventory."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import os
import sys
import uuid
from collections import Counter
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping


PROFILES = ("hard1-hard1", "med2-med2")
GATE_OBJECTS = ("obstacle_0_movable", "obstacle_1_movable")
GATE_LABEL_FIELDS = (
    "n_tried_1push",
    "n_valid_1push",
    "n_valid_first_push",
    "n_open_within_2push",
    "solve_rate_1push",
    "solve_rate_hmax2",
    "tier_1push",
    "tier_hmax2",
    "push_kind",
    "timed_out",
    "classification_complete",
)
NAVIGATION_DETAIL_FIELDS = (
    "any_navigation_succeeded",
    "ignore_status",
    "ignore_reached",
    "ignore_failure_cause",
    "ignore_distance_to_goal_cm",
    "penalise_status",
    "penalise_reached",
    "penalise_failure_cause",
    "penalise_distance_to_goal_cm",
)


class InventoryError(RuntimeError):
    """Raised when the inventory inputs cannot be joined unambiguously."""


def _canon(path: str | Path) -> str:
    return os.path.realpath(str(path).replace("/scache/scratch/", "/scratch/", 1))


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise InventoryError(f"missing JSONL input: {path}")
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise InventoryError(f"{path}:{line_number}: invalid JSON: {error}") from error
        if not isinstance(row, dict):
            raise InventoryError(f"{path}:{line_number}: expected a JSON object")
        rows.append(row)
    return rows


def _load_selector(selector_repo: Path, selector_module: str | Path) -> ModuleType:
    repo = selector_repo.resolve()
    module_path = Path(selector_module)
    if not module_path.is_absolute():
        module_path = repo / module_path
    module_path = module_path.resolve()
    if not module_path.is_file():
        raise InventoryError(f"missing selector module: {module_path}")
    module_name = f"exact_inventory_selector_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise InventoryError(f"cannot load selector module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    import_paths = [str(module_path.parent), str(repo / "scripts")]
    inserted: list[str] = []
    try:
        for import_path in reversed(import_paths):
            if import_path not in sys.path:
                sys.path.insert(0, import_path)
                inserted.append(import_path)
        spec.loader.exec_module(module)
    except Exception as error:
        raise InventoryError(f"failed to load selector module {module_path}: {error}") from error
    finally:
        for import_path in inserted:
            sys.path.remove(import_path)
    required = (
        "_valid_k1_row",
        "_valid_k2_row",
        "classify_episode",
        "matches_profile",
        "mechanical_independence",
        "normalize_gate_contract",
    )
    missing = [name for name in required if not callable(getattr(module, name, None))]
    if missing:
        raise InventoryError(f"selector module lacks authoritative function(s): {', '.join(missing)}")
    return module


def _load_sheets(path: Path, profile: str) -> tuple[dict[str, dict[str, Any]], dict[str, str]]:
    if not path.is_file():
        raise InventoryError(f"missing build sheets: {path}")
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, list):
        raise InventoryError(f"build sheets must be a JSON list: {path}")
    by_source: dict[str, dict[str, Any]] = {}
    by_scene: dict[str, str] = {}
    for index, sheet in enumerate(raw):
        if not isinstance(sheet, dict):
            raise InventoryError(f"{path}: sheet {index} is not an object")
        scene_id = sheet.get("scene_id")
        if not isinstance(scene_id, str) or not scene_id:
            raise InventoryError(f"{path}: sheet {index} has no scene_id")
        if sheet.get("requested_profile") != profile:
            raise InventoryError(
                f"{path}: scene {scene_id} requested_profile is {sheet.get('requested_profile')!r}, "
                f"expected {profile!r}"
            )
        source = _canon(path.parent / scene_id / "env.xml")
        if source in by_source:
            raise InventoryError(f"duplicate build-sheet source join: {source}")
        if scene_id in by_scene:
            raise InventoryError(f"duplicate build-sheet scene join: {(profile, scene_id)!r}")
        by_source[source] = sheet
        by_scene[scene_id] = source
    return by_source, by_scene


def _unique_rows_by_path(
    rows: list[dict[str, Any]],
    path_field: str,
    kind: str,
) -> dict[str, dict[str, Any]]:
    indexed: dict[str, dict[str, Any]] = {}
    for index, row in enumerate(rows):
        value = row.get(path_field)
        if not isinstance(value, str) or not value:
            raise InventoryError(f"{kind} row {index} has no {path_field}")
        key = _canon(value)
        if key in indexed:
            raise InventoryError(f"duplicate {kind} join key: {key}")
        indexed[key] = row
    return indexed


def _load_navigation(path: Path) -> dict[tuple[str, str], dict[str, str]]:
    if not path.is_file():
        raise InventoryError(f"missing navigation CSV: {path}")
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"profile", "scene_id", "navigation_category"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise InventoryError(f"navigation CSV lacks required column(s): {', '.join(sorted(missing))}")
        indexed: dict[tuple[str, str], dict[str, str]] = {}
        for line_number, row in enumerate(reader, 2):
            key = (row["profile"], row["scene_id"])
            if not all(key):
                raise InventoryError(f"navigation CSV line {line_number} has an empty join key")
            if key in indexed:
                raise InventoryError(f"duplicate navigation join key: {key!r}")
            indexed[key] = row
    return indexed


def _gate_label(selector: ModuleType, row: Mapping[str, Any] | None, object_id: str) -> dict[str, Any] | None:
    if row is None:
        return None
    record = (row.get("kh1_key") or {}).get(object_id)
    if not isinstance(record, dict):
        return None
    result = selector.classify_episode(record)
    if not isinstance(result, dict):
        raise InventoryError(f"authoritative classify_episode returned {type(result).__name__}")
    return result


def _primary_gate_metrics(label: Mapping[str, Any] | None) -> dict[str, Any]:
    if not label:
        return {"axis": "", "n_tried": "", "n_valid": "", "solve_rate": "", "tier": ""}
    one_push = label.get("push_kind") == "one_push"
    axis = "1push" if one_push else "hmax2"
    return {
        "axis": axis,
        "n_tried": label.get("n_tried_1push", ""),
        "n_valid": label.get("n_valid_1push" if one_push else "n_open_within_2push", ""),
        "solve_rate": label.get("solve_rate_1push" if one_push else "solve_rate_hmax2", ""),
        "tier": label.get("tier_1push" if one_push else "tier_hmax2", ""),
    }


def _put_gate_fields(output: dict[str, Any], gate: str, label: Mapping[str, Any] | None) -> None:
    primary = _primary_gate_metrics(label)
    for name, value in primary.items():
        output[f"{gate}_{name}"] = value
    for name in GATE_LABEL_FIELDS:
        output[f"{gate}_{name}"] = label.get(name, "") if label else ""


def _coordinate_fields(sheet: Mapping[str, Any]) -> dict[str, Any]:
    def pair(name: str) -> tuple[str, Any, Any]:
        value = sheet.get(name)
        if not isinstance(value, list) or len(value) != 2:
            return "", "", ""
        return json.dumps(value, separators=(",", ":")), value[0], value[1]

    goal, goal_x, goal_y = pair("goal_cm")
    start, start_x, start_y = pair("robot_start_cm")
    return {
        "goal_cm": goal,
        "goal_x_cm": goal_x,
        "goal_y_cm": goal_y,
        "robot_start_cm": start,
        "robot_start_x_cm": start_x,
        "robot_start_y_cm": start_y,
        "robot_start_bearing_deg": sheet.get("robot_start_bearing_deg", ""),
        "run_namo_goal_flag": sheet.get("run_namo_goal_flag", ""),
    }


def _joint_label(
    selector: ModuleType,
    k1_label: Mapping[str, Any] | None,
    k2_label: Mapping[str, Any] | None,
    transitions_valid: bool,
) -> str:
    labels_complete = bool(
        k1_label
        and k2_label
        and k1_label.get("classification_complete")
        and k2_label.get("classification_complete")
    )
    if not transitions_valid or not labels_complete:
        return "incomplete"
    matches = [
        profile
        for profile in PROFILES
        if selector.matches_profile(dict(k1_label), dict(k2_label), profile)
    ]
    if len(matches) > 1:
        raise InventoryError(f"authoritative profile predicates overlap: {matches!r}")
    return matches[0] if matches else "other"


def _counter(values: list[Any]) -> dict[str, int]:
    return dict(sorted(Counter(str(value) for value in values).items()))


def _atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(content, encoding="utf-8")
    os.replace(temporary, path)


def _csv_text(rows: list[dict[str, Any]], fieldnames: list[str]) -> str:
    import io

    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise", lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def build_inventory(
    *,
    exact_root: str | Path,
    build_sheets: Mapping[str, str | Path],
    selector_repo: str | Path,
    selector_module: str | Path,
    navigation_csv: str | Path,
    output_csv: str | Path,
    output_json: str | Path,
    expected_rows: int = 100,
) -> dict[str, Any]:
    """Join exact sequential labels and navigation outcomes without reordering K1 rows."""
    if set(build_sheets) != set(PROFILES):
        raise InventoryError(f"build_sheets must provide exactly {PROFILES!r}")
    if expected_rows < 1:
        raise InventoryError("expected_rows must be positive")
    root = Path(exact_root).resolve()
    selector = _load_selector(Path(selector_repo), selector_module)
    navigation = _load_navigation(Path(navigation_csv))
    inventory: list[dict[str, Any]] = []
    build_sheet_rows: dict[str, int] = {}

    for profile in PROFILES:
        sheet_path = Path(build_sheets[profile]).resolve()
        sheets_by_source, sheets_by_scene = _load_sheets(sheet_path, profile)
        build_sheet_rows[profile] = len(sheets_by_scene)
        k1_path = root / profile / "k1" / "merged_rows.jsonl"
        k2_path = root / profile / "k2" / "merged_rows.jsonl"
        k1_rows = _load_jsonl(k1_path)
        _unique_rows_by_path(k1_rows, "xml_path", "K1")
        k2_rows = _load_jsonl(k2_path)
        k2_by_input = _unique_rows_by_path(k2_rows, "xml_path", "K2")
        consumed_sources: set[str] = set()
        joined_k2_inputs: set[str] = set()

        for profile_index, k1_row in enumerate(k1_rows):
            source_key = _canon(k1_row["xml_path"])
            sheet = sheets_by_source.get(source_key)
            if sheet is None:
                raise InventoryError(f"K1 source has no build-sheet join: {source_key}")
            if source_key in consumed_sources:
                raise InventoryError(f"duplicate K1-to-build-sheet join: {source_key}")
            consumed_sources.add(source_key)
            scene_id = sheet["scene_id"]
            nav_key = (profile, scene_id)
            nav = navigation.get(nav_key)
            if nav is None:
                raise InventoryError(f"missing navigation join key: {nav_key!r}")

            k1_valid = bool(selector._valid_k1_row(k1_row))
            k1_out = k1_row.get("out_xml")
            k2_row = k2_by_input.get(_canon(k1_out)) if isinstance(k1_out, str) and k1_out else None
            if k2_row is not None:
                joined_k2_inputs.add(_canon(k2_row["xml_path"]))
            k2_valid = bool(k2_row is not None and selector._valid_k2_row(k2_row))
            transitions_valid = k1_valid and k2_valid
            k1_label = _gate_label(selector, k1_row, GATE_OBJECTS[0])
            k2_label = _gate_label(selector, k2_row, GATE_OBJECTS[1])

            independence_status = "not_computed"
            independence_failure = ""
            independence: dict[str, Any] = {}
            if transitions_valid:
                try:
                    contract = selector.normalize_gate_contract(
                        sheet.get("gate_contract"),
                        gate_objects=list(GATE_OBJECTS),
                    )
                    independence = selector.mechanical_independence(
                        k1_row["xml_path"],
                        k1_row["out_xml"],
                        k2_row["out_xml"],
                        gate_members=contract["gate_members"],
                    )
                    independence_status = str(independence.get("status", "unverifiable"))
                    independence_failure = str(independence.get("failure") or "")
                except Exception as error:
                    independence_status = "unverifiable"
                    independence_failure = f"{type(error).__name__}: {error}"

            row: dict[str, Any] = {
                "inventory_index": len(inventory),
                "profile_index": profile_index,
                "discovery_profile": profile,
                "scene_id": scene_id,
                "source_xml": k1_row["xml_path"],
                "post_k1_xml": k1_out or "",
                "post_k2_xml": k2_row.get("out_xml", "") if k2_row else "",
                "k1_materialization_status": k1_row.get("status", "missing"),
                "k1_transition_status": "valid" if k1_valid else "invalid",
                "k2_materialization_status": k2_row.get("status", "missing") if k2_row else "missing",
                "k2_transition_status": "valid" if k2_valid else ("invalid" if k2_row else "missing"),
                "sequential_transition_status": "valid" if transitions_valid else "incomplete",
                "exact_joint_label": _joint_label(selector, k1_label, k2_label, transitions_valid),
                "mechanical_independence_status": independence_status,
                "mechanical_independence_failure": independence_failure,
                "k1_to_k2_position_delta_mm": independence.get("k1_to_k2_position_delta_mm", ""),
                "k1_to_k2_angle_delta_deg": independence.get("k1_to_k2_angle_delta_deg", ""),
                "k2_to_k1_position_delta_mm": independence.get("k2_to_k1_position_delta_mm", ""),
                "k2_to_k1_angle_delta_deg": independence.get("k2_to_k1_angle_delta_deg", ""),
                "navigation_category": nav["navigation_category"],
                "nav_manifest_index": nav.get("manifest_index", ""),
            }
            row.update(_coordinate_fields(sheet))
            _put_gate_fields(row, "k1", k1_label)
            _put_gate_fields(row, "k2", k2_label)
            for name in NAVIGATION_DETAIL_FIELDS:
                row[f"nav_{name}"] = nav.get(name, "")
            inventory.append(row)

        orphan_k2 = sorted(set(k2_by_input) - joined_k2_inputs)
        if orphan_k2:
            raise InventoryError(f"{profile}: orphan K2 join key: {orphan_k2[0]}")

    if len(inventory) != expected_rows:
        raise InventoryError(f"expected {expected_rows} inventory rows, found {len(inventory)}")
    inventory_keys = {(row["discovery_profile"], row["scene_id"]) for row in inventory}
    if len(inventory_keys) != len(inventory):
        raise InventoryError("duplicate final inventory join key")
    extra_navigation = sorted(set(navigation) - inventory_keys)
    if extra_navigation:
        raise InventoryError(f"navigation CSV has unjoined key: {extra_navigation[0]!r}")

    output_csv_path = Path(output_csv).resolve()
    output_json_path = Path(output_json).resolve()
    fieldnames = list(inventory[0])
    requested_exact_candidates = {
        profile: [
            {
                "discovery_profile": row["discovery_profile"],
                "scene_id": row["scene_id"],
            }
            for row in inventory
            if row["exact_joint_label"] == profile
            and row["mechanical_independence_status"] == "passed"
        ]
        for profile in PROFILES
    }
    navigation_resistant_2push2push = [
        {
            "discovery_profile": row["discovery_profile"],
            "scene_id": row["scene_id"],
            "k1_tier": row["k1_tier"],
            "k1_solve_rate": row["k1_solve_rate"],
            "k2_tier": row["k2_tier"],
            "k2_solve_rate": row["k2_solve_rate"],
        }
        for row in inventory
        if row["sequential_transition_status"] == "valid"
        and row["mechanical_independence_status"] == "passed"
        and row["navigation_category"] == "both_failed"
        and row["k1_push_kind"] == "needs_2_chain"
        and row["k2_push_kind"] == "needs_2_chain"
    ]
    summary: dict[str, Any] = {
        "rows": len(inventory),
        "build_sheet_rows": build_sheet_rows,
        "profile_counts": _counter([row["discovery_profile"] for row in inventory]),
        "exact_joint_label_counts": _counter([row["exact_joint_label"] for row in inventory]),
        "sequential_transition_status_counts": _counter(
            [row["sequential_transition_status"] for row in inventory]
        ),
        "mechanical_independence_status_counts": _counter(
            [row["mechanical_independence_status"] for row in inventory]
        ),
        "navigation_category_counts": _counter([row["navigation_category"] for row in inventory]),
        "requested_exact_candidates": requested_exact_candidates,
        "navigation_resistant_2push2push_candidates": navigation_resistant_2push2push,
        "inputs": {
            "exact_root": str(root),
            "build_sheets": {
                profile: str(Path(build_sheets[profile]).resolve()) for profile in PROFILES
            },
            "selector_repo": str(Path(selector_repo).resolve()),
            "selector_module": str(selector_module),
            "navigation_csv": str(Path(navigation_csv).resolve()),
        },
        "outputs": {
            "csv": str(output_csv_path),
            "json": str(output_json_path),
        },
    }
    _atomic_write_text(output_csv_path, _csv_text(inventory, fieldnames))
    _atomic_write_text(output_json_path, json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exact-root", required=True)
    parser.add_argument("--hard-build-sheets", required=True)
    parser.add_argument("--med-build-sheets", required=True)
    parser.add_argument("--selector-repo", required=True)
    parser.add_argument(
        "--selector-module",
        default="scripts/pipeline/select_real_twohop_profiles.py",
        help="absolute path or a path relative to --selector-repo",
    )
    parser.add_argument("--navigation-csv", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--expected-rows", type=int, default=100)
    args = parser.parse_args()
    summary = build_inventory(
        exact_root=args.exact_root,
        build_sheets={
            "hard1-hard1": args.hard_build_sheets,
            "med2-med2": args.med_build_sheets,
        },
        selector_repo=args.selector_repo,
        selector_module=args.selector_module,
        navigation_csv=args.navigation_csv,
        output_csv=args.output_csv,
        output_json=args.output_json,
        expected_rows=args.expected_rows,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
