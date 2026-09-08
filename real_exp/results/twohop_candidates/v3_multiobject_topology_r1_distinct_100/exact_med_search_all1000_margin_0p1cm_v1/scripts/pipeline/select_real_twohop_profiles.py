#!/usr/bin/env python3
"""Select real two-hop scenes only after both local keyholes have measured labels.

K1 labels come from ``materialize_keyhole2.py`` on the original two-hop XML. That same pass fixes
one deterministic K1 opener and writes the post-K1 XML. K2 labels can come either from a second
``materialize_keyhole2.py`` pass on that XML or from the ordinary exhaustive depth-2 collection,
reduced with ``build_2push_validset.py``.

The supported paper profiles intentionally retain the ordered local labels:

* ``hard1-hard1``: both gates are canonical 1push/hard and have a one-push opener.
* ``med2-med2``: both gates are canonical hmax2/med, have no one-push opener, and do have a
  setup-plus-finish chain.

Geometric proxy fields from the generator are never consulted here.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))

from eval_common import bin_of  # noqa: E402
from real_twohop_multiobject import normalize_gate_contract  # noqa: E402


PROFILES = ("hard1-hard1", "med2-med2")
INDEPENDENT_POSITION_TOLERANCE_M = 0.002
INDEPENDENT_ANGLE_TOLERANCE_RAD = math.radians(1.0)


def _canon(path: str | Path) -> str:
    return os.path.realpath(str(path).replace("/scache/scratch/", "/scratch/", 1))


def _unique_count(value: Any) -> int:
    if value is None:
        return 0
    if isinstance(value, int):
        return value
    return len({tuple(cell) for cell in value})


def _cells(value: Any) -> set[tuple[int, int]] | None:
    if value is None or isinstance(value, int):
        return None
    return {tuple(cell) for cell in value}


def _angle_delta(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(right - left), math.cos(right - left)))


def _geom_pose(path: str | Path, object_id: str) -> tuple[float, float, float]:
    root = ET.parse(path).getroot()
    geom = root.find(f".//geom[@name='{object_id}']")
    if geom is None:
        raise ValueError(f"{path}: missing geom {object_id!r}")
    position = [float(value) for value in (geom.get("pos") or "").split()]
    if len(position) < 2:
        raise ValueError(f"{path}: geom {object_id!r} has no xy position")
    quaternion = [float(value) for value in (geom.get("quat") or "").split()]
    if quaternion:
        if len(quaternion) != 4:
            raise ValueError(f"{path}: geom {object_id!r} has invalid quaternion")
        w, x, y, z = quaternion
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    else:
        euler = [float(value) for value in (geom.get("euler") or "0 0 0").split()]
        if len(euler) != 3:
            raise ValueError(f"{path}: geom {object_id!r} has invalid Euler angles")
        compiler = root.find("compiler")
        unit = (compiler.get("angle") if compiler is not None else None) or "degree"
        yaw = math.radians(euler[2]) if unit == "degree" else euler[2]
    return position[0], position[1], yaw


def mechanical_independence(
    initial_xml: str | Path,
    post_k1_xml: str | Path,
    post_k2_xml: str | Path,
    gate_members: list[list[str]] | None = None,
) -> dict:
    """Measure cross-gate motion for every member; within-gate motion is allowed."""
    explicit_members = gate_members is not None
    groups = gate_members or [["obstacle_0_movable"], ["obstacle_1_movable"]]
    if len(groups) != 2 or any(not group for group in groups):
        raise ValueError("gate_members must contain two non-empty groups")

    def compare(before_xml: str | Path, after_xml: str | Path, object_ids: list[str]) -> dict:
        objects = {}
        for object_id in object_ids:
            before = _geom_pose(before_xml, object_id)
            after = _geom_pose(after_xml, object_id)
            position = math.dist(before[:2], after[:2])
            angle = _angle_delta(before[2], after[2])
            objects[object_id] = {
                "position_delta_mm": round(1000.0 * position, 4),
                "angle_delta_deg": round(math.degrees(angle), 4),
                "within_tolerance": bool(
                    position <= INDEPENDENT_POSITION_TOLERANCE_M
                    and angle <= INDEPENDENT_ANGLE_TOLERANCE_RAD
                ),
            }
        return objects

    k1_to_k2 = compare(initial_xml, post_k1_xml, groups[1])
    k2_to_k1 = compare(post_k1_xml, post_k2_xml, groups[0])
    failure = None
    if not all(row["within_tolerance"] for row in k1_to_k2.values()):
        failure = "k1_moved_k2_member" if explicit_members else "k1_moved_k2"
    elif not all(row["within_tolerance"] for row in k2_to_k1.values()):
        failure = "k2_moved_k1_member" if explicit_members else "k2_moved_k1"
    k2_opener = k1_to_k2[groups[1][0]]
    k1_opener = k2_to_k1[groups[0][0]]
    return {
        "status": "failed" if failure else "passed",
        "failure": failure,
        "position_tolerance_mm": 1000.0 * INDEPENDENT_POSITION_TOLERANCE_M,
        "angle_tolerance_deg": math.degrees(INDEPENDENT_ANGLE_TOLERANCE_RAD),
        "k1_to_k2_position_delta_mm": k2_opener["position_delta_mm"],
        "k1_to_k2_angle_delta_deg": k2_opener["angle_delta_deg"],
        "k2_to_k1_position_delta_mm": k1_opener["position_delta_mm"],
        "k2_to_k1_angle_delta_deg": k1_opener["angle_delta_deg"],
        "k1_to_k2": k1_to_k2,
        "k2_to_k1": k2_to_k1,
    }


def canonical_opener_matches(row: Mapping[str, Any], expected_object: str) -> bool:
    opener = row.get("canonical_opener")
    return bool(
        isinstance(opener, list)
        and len(opener) == 3
        and opener[1] == expected_object
    )


def classify_episode(record: dict) -> dict:
    """Normalize count-only materialization rows and set-valued exhaustive answer-key rows."""
    tried_value = record.get("tried_1push", record.get("tried"))
    valid_value = record.get("valid_1push", record.get("valid"))
    first_value = record.get("valid_first_push")
    tried = _unique_count(tried_value)
    valid = _unique_count(valid_value)
    valid_first = _unique_count(first_value)

    valid_cells = _cells(valid_value)
    first_cells = _cells(first_value)
    if valid_cells is not None and first_cells is not None:
        valid_hmax2 = len(valid_cells | first_cells)
    else:
        valid_hmax2 = valid + valid_first
    valid_hmax2 = min(valid_hmax2, tried) if tried else 0

    timed_out = bool(
        record.get("timed_out")
        or record.get("neighbour_timed_out")
        or record.get("depth2_censored")
    )
    rate_1push = valid / tried if tried else 0.0
    rate_hmax2 = valid_hmax2 / tried if tried else 0.0
    if valid:
        push_kind = "one_push"
    elif valid_first:
        push_kind = "needs_2_chain"
    else:
        push_kind = "unsolvable"
    return {
        "n_tried_1push": tried,
        "n_valid_1push": valid,
        "n_valid_first_push": valid_first,
        "n_open_within_2push": valid_hmax2,
        "solve_rate_1push": rate_1push,
        "solve_rate_hmax2": rate_hmax2,
        "tier_1push": bin_of(rate_1push) if valid else "unsolvable",
        "tier_hmax2": bin_of(rate_hmax2) if valid_hmax2 else "unsolvable",
        "push_kind": push_kind,
        "timed_out": timed_out,
        "classification_complete": bool(tried) and not timed_out,
    }


def matches_profile(k1: dict, k2: dict, profile: str) -> bool:
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {PROFILES}")
    if (
        not k1.get("classification_complete")
        or not k2.get("classification_complete")
        or k1.get("timed_out")
        or k2.get("timed_out")
    ):
        return False
    if profile == "hard1-hard1":
        return all(
            gate.get("tier_1push") == "hard" and gate.get("push_kind") == "one_push"
            for gate in (k1, k2)
        )
    return all(
        gate.get("tier_hmax2") == "med" and gate.get("push_kind") == "needs_2_chain"
        for gate in (k1, k2)
    )


def _load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _pick_k2_episode(episodes: list[dict]) -> dict | None:
    exact = [episode for episode in episodes if episode.get("object_id") == "obstacle_1_movable"]
    return exact[0] if len(exact) == 1 else None


def _valid_k1_row(row: dict) -> bool:
    return bool(
        row.get("status") == "ok"
        and row.get("hop_count") == 2
        and row.get("post_hop_count") == 1
        and row.get("kh1_boundary_objects") == ["obstacle_0_movable"]
        and row.get("next_boundary_objects") == ["obstacle_1_movable"]
        and row.get("post_next_boundary_objects") == ["obstacle_1_movable"]
        and row.get("next_boundary_matches") is True
        and not row.get("kh1_timed_out")
        and row.get("out_xml")
    )


def _valid_k2_row(row: dict) -> bool:
    """Require a measured terminal opening of the second, and only the second, blocker."""
    return bool(
        row.get("status") == "ok"
        and row.get("hop_count") == 1
        and row.get("post_hop_count") == 0
        and row.get("kh1_boundary_objects") == ["obstacle_1_movable"]
        and row.get("next_boundary_objects") == []
        and row.get("post_next_boundary_objects") == []
        and row.get("next_boundary_matches") is True
        and not row.get("kh1_timed_out")
        and row.get("out_xml")
    )


def _joined_candidates(
    sheets_path: Path,
    k1_rows_path: Path,
    profile: str,
    *,
    k2_key_path: Path | None = None,
    k2_rows_path: Path | None = None,
) -> tuple[list[dict], dict[str, int]]:
    if (k2_key_path is None) == (k2_rows_path is None):
        raise ValueError("provide exactly one of k2_key_path or k2_rows_path")
    sheet_root = sheets_path.parent
    sheets = json.loads(sheets_path.read_text(encoding="utf-8"))
    k1_rows = {_canon(row["xml_path"]): row for row in _load_jsonl(k1_rows_path)}
    k2_key: dict[str, list[dict]] = {}
    k2_rows: dict[str, dict] = {}
    if k2_key_path is not None:
        raw_k2 = json.loads(k2_key_path.read_text(encoding="utf-8"))
        k2_key = {_canon(path): episodes for path, episodes in raw_k2.items()}
    else:
        assert k2_rows_path is not None
        k2_rows = {_canon(row["xml_path"]): row for row in _load_jsonl(k2_rows_path)}
    reasons: dict[str, int] = {}
    candidates: list[dict] = []

    def reject(reason: str) -> None:
        reasons[reason] = reasons.get(reason, 0) + 1

    for sheet in sheets:
        if sheet.get("requested_profile") != profile:
            reject("wrong_requested_profile")
            continue
        try:
            contract = normalize_gate_contract(
                sheet.get("gate_contract"),
                gate_objects=["obstacle_0_movable", "obstacle_1_movable"],
            )
        except ValueError:
            reject("invalid_gate_contract")
            continue
        gate_openers = [group[0] for group in contract["boundary_objects"]]
        source_xml = (sheet_root / sheet["scene_id"] / "env.xml").resolve()
        if not source_xml.is_file():
            reject("missing_source_xml")
            continue
        k1_row = k1_rows.get(_canon(source_xml))
        if not k1_row or not _valid_k1_row(k1_row):
            reject("invalid_k1_materialization")
            continue
        if not canonical_opener_matches(k1_row, gate_openers[0]):
            reject("opener_object_mismatch")
            continue
        k1_record = (k1_row.get("kh1_key") or {}).get("obstacle_0_movable")
        if not k1_record:
            reject("missing_k1_label")
            continue
        k2_row = None
        if k2_rows_path is not None:
            k2_row = k2_rows.get(_canon(k1_row["out_xml"]))
            if not k2_row or not _valid_k2_row(k2_row):
                reject("invalid_k2_materialization")
                continue
            if not canonical_opener_matches(k2_row, gate_openers[1]):
                reject("opener_object_mismatch")
                continue
            k2_episode = (k2_row.get("kh1_key") or {}).get("obstacle_1_movable")
        else:
            k2_episode = _pick_k2_episode(k2_key.get(_canon(k1_row["out_xml"]), []))
        if not k2_episode:
            reject("missing_k2_label")
            continue
        k1_label = classify_episode(k1_record)
        k2_label = classify_episode(k2_episode)
        if not matches_profile(k1_label, k2_label, profile):
            reject("profile_mismatch")
            continue
        independence = None
        if k2_row is not None:
            try:
                independence = mechanical_independence(
                    source_xml,
                    k1_row["out_xml"],
                    k2_row["out_xml"],
                    gate_members=contract["gate_members"],
                )
            except (OSError, ValueError, ET.ParseError):
                reject("independence_unverifiable")
                continue
            if independence["status"] != "passed":
                reject("mechanical_coupling")
                continue
        candidates.append(
            {
                "scene_id": sheet["scene_id"],
                "source_xml": str(source_xml),
                "post_k1_xml": str(Path(k1_row["out_xml"]).resolve()),
                "sheet": sheet,
                "k1_row": k1_row,
                "k2_row": k2_row,
                "mechanical_independence": independence,
                "labels": [k1_label, k2_label],
            }
        )
    candidates.sort(key=lambda candidate: candidate["scene_id"])
    return candidates, reasons


def select_profiles(
    *,
    sheets_path: str | Path,
    k1_rows_path: str | Path,
    k2_key_path: str | Path | None = None,
    k2_rows_path: str | Path | None = None,
    profile: str,
    out_dir: str | Path,
    limit: int,
) -> dict:
    if limit < 1:
        raise ValueError("limit must be positive")
    candidates, rejects = _joined_candidates(
        Path(sheets_path),
        Path(k1_rows_path),
        profile,
        k2_key_path=Path(k2_key_path) if k2_key_path is not None else None,
        k2_rows_path=Path(k2_rows_path) if k2_rows_path is not None else None,
    )
    chosen = candidates[:limit]
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    manifest: list[str] = []
    selection_rows: list[dict] = []
    for candidate in chosen:
        scene_dir = root / candidate["scene_id"]
        scene_dir.mkdir(parents=True, exist_ok=True)
        selected_xml = scene_dir / "env.xml"
        post_k1_xml = scene_dir / "post_k1_env.xml"
        shutil.copy2(candidate["source_xml"], selected_xml)
        shutil.copy2(candidate["post_k1_xml"], post_k1_xml)
        k2_row = candidate["k2_row"]
        post_k2_xml = None
        if k2_row is not None:
            post_k2_xml = scene_dir / "post_k2_env.xml"
            shutil.copy2(k2_row["out_xml"], post_k2_xml)
        measured_sheet = dict(candidate["sheet"])
        measured_sheet.update(
            classification_status="measured",
            measured_profile=profile,
            measured_gates=candidate["labels"],
            k1_canonical_opener=candidate["k1_row"].get("canonical_opener"),
            post_k1_xml="post_k1_env.xml",
        )
        if k2_row is not None:
            measured_sheet.update(
                k2_canonical_opener=k2_row.get("canonical_opener"),
                post_k2_xml="post_k2_env.xml",
                mechanical_independence=candidate["mechanical_independence"],
            )
        (scene_dir / "build_sheet.json").write_text(
            json.dumps(measured_sheet, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        manifest.append(str(selected_xml.resolve()))
        selection_rows.append(
            {
                "scene_id": candidate["scene_id"],
                "xml": str(selected_xml.resolve()),
                "labels": candidate["labels"],
            }
        )

    (root / "manifest.txt").write_text("".join(f"{path}\n" for path in manifest), encoding="utf-8")
    summary = {
        "profile": profile,
        "eligible": len(candidates),
        "selected": len(chosen),
        "limit": limit,
        "rejects": rejects,
        "scenes": selection_rows,
    }
    (root / "selection.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sheets", required=True)
    parser.add_argument("--k1-rows", required=True)
    k2_source = parser.add_mutually_exclusive_group(required=True)
    k2_source.add_argument("--k2-key")
    k2_source.add_argument("--k2-rows")
    parser.add_argument("--profile", choices=PROFILES, required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--limit", type=int, default=1)
    args = parser.parse_args()
    summary = select_profiles(
        sheets_path=args.sheets,
        k1_rows_path=args.k1_rows,
        k2_key_path=args.k2_key,
        k2_rows_path=args.k2_rows,
        profile=args.profile,
        out_dir=args.out_dir,
        limit=args.limit,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
