#!/usr/bin/env python3
"""Validate and reduce one exact sequential K1/K2 materialization stage."""

from __future__ import annotations

import argparse
import json
import math
import os
from collections import Counter
from pathlib import Path
from typing import Any


class ReductionError(RuntimeError):
    """Raised when array output cannot be reduced without ambiguity."""


def _canon(path: str | Path) -> str:
    return os.path.realpath(str(path).replace("/scache/scratch/", "/scratch/", 1))


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    os.replace(temporary, path)


def _read_manifest(path: Path) -> list[str]:
    rows = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ReductionError(f"empty manifest: {path}")
    canonical = [_canon(row) for row in rows]
    if len(canonical) != len(set(canonical)):
        raise ReductionError(f"manifest contains duplicate inputs: {path}")
    missing = [row for row in rows if not Path(row).is_file()]
    if missing:
        raise ReductionError(f"manifest references {len(missing)} missing input XML(s)")
    return rows


def _load_shard_row(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ReductionError(f"missing shard row: {path}")
    lines = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(lines) != 1:
        raise ReductionError(f"expected exactly one JSON row in {path}, found {len(lines)}")
    try:
        row = json.loads(lines[0])
    except json.JSONDecodeError as error:
        raise ReductionError(f"invalid JSON in {path}: {error}") from error
    if not isinstance(row, dict):
        raise ReductionError(f"expected a JSON object in {path}")
    return row


def _opener_matches(row: dict[str, Any], expected_object: str) -> bool:
    opener = row.get("canonical_opener")
    return bool(
        isinstance(opener, list)
        and len(opener) == 3
        and isinstance(opener[0], int)
        and opener[0] >= 1
        and opener[1] == expected_object
        and isinstance(opener[2], list)
        and len(opener[2]) == opener[0]
    )


def _transition_reject_reason(
    row: dict[str, Any],
    stage: str,
) -> str | None:
    is_k1 = stage == "k1"
    expected_object = "obstacle_0_movable" if is_k1 else "obstacle_1_movable"
    expected_boundary = [expected_object]
    expected_next = ["obstacle_1_movable"] if is_k1 else []
    expected_hops = (2, 1) if is_k1 else (1, 0)
    checks = (
        (row.get("status") == "ok", "status_not_ok"),
        ((row.get("hop_count"), row.get("post_hop_count")) == expected_hops, "wrong_hop_transition"),
        (row.get("kh1_boundary_objects") == expected_boundary, "wrong_opened_boundary"),
        (row.get("next_boundary_objects") == expected_next, "wrong_input_next_boundary"),
        (row.get("post_next_boundary_objects") == expected_next, "wrong_output_next_boundary"),
        (row.get("next_boundary_matches") is True, "next_boundary_mismatch"),
        (not row.get("kh1_timed_out"), "classification_timed_out"),
        (bool(row.get("out_xml")), "missing_output_xml"),
    )
    for passed, reason in checks:
        if not passed:
            return reason
    return None


def _validate_success_output(
    row: dict[str, Any],
    profile: str,
    stage: str,
    task_root: Path,
) -> str:
    expected_protected = {
        ("hard1-hard1", "k1"): ["obstacle_1_movable", "obstacle_2_movable"],
        ("hard1-hard1", "k2"): ["obstacle_0_movable"],
        ("med2-med2", "k1"): ["obstacle_1_movable"],
        ("med2-med2", "k2"): ["obstacle_0_movable", "obstacle_2_movable"],
    }[(profile, stage)]
    round_trip_metrics = (
        row.get("max_dxy_mm"),
        row.get("max_dtheta_deg"),
        row.get("robot_dxy_mm"),
        row.get("robot_dtheta_deg"),
    )
    checks = (
        (bool(row.get("out_xml")), "missing_output_xml"),
        (row.get("post_goal_in_free_space") is True, "post_goal_not_free"),
        (row.get("protected_objects") == expected_protected, "wrong_protected_object_contract"),
        ((row.get("next_boundary_protection") or {}).get("status") == "passed", "next_boundary_moved"),
        ((row.get("protected_object_motion") or {}).get("status") == "passed", "protected_object_moved"),
        (row.get("missing_bodies") == [], "round_trip_missing_body"),
        (
            all(isinstance(value, (int, float)) and math.isfinite(value) for value in round_trip_metrics),
            "nonfinite_round_trip_metric",
        ),
    )
    for passed, reason in checks:
        if not passed:
            raise ReductionError(f"status ok row violates {reason}")
    output = Path(row["out_xml"])
    if not output.is_file():
        raise ReductionError("status ok row references a missing output XML")
    canonical_output = _canon(output)
    expected_output_root = _canon(task_root / "xmls") + os.sep
    if not canonical_output.startswith(expected_output_root):
        raise ReductionError("status ok row output is outside its task shard")
    return canonical_output


def reduce_stage(
    manifest_path: str | Path,
    stage_root: str | Path,
    profile: str,
    stage: str,
) -> dict[str, Any]:
    if profile not in {"hard1-hard1", "med2-med2"}:
        raise ReductionError(f"unsupported profile: {profile}")
    if stage not in {"k1", "k2"}:
        raise ReductionError(f"unsupported stage: {stage}")

    manifest_path = Path(manifest_path).resolve()
    stage_root = Path(stage_root).resolve()
    inputs = _read_manifest(manifest_path)
    shards_root = stage_root / "shards"
    expected_names = {f"task_{index:04d}" for index in range(len(inputs))}
    actual_names = {path.name for path in shards_root.glob("task_*") if path.is_dir()}
    extras = sorted(actual_names - expected_names)
    if extras:
        raise ReductionError(f"unexpected task shard(s): {', '.join(extras)}")

    rows: list[dict[str, Any]] = []
    valid_outputs: list[str] = []
    normal_outputs: list[str] = []
    nodecrement_outputs: list[str] = []
    reject_counts: Counter[str] = Counter()
    for index, expected_input in enumerate(inputs):
        task_root = shards_root / f"task_{index:04d}"
        row = _load_shard_row(task_root / "rows.jsonl")
        if not row.get("xml_path") or _canon(row["xml_path"]) != _canon(expected_input):
            raise ReductionError(f"task {index} row does not match manifest index {index}")
        normal_output = None
        if row.get("status") == "ok":
            try:
                normal_output = _validate_success_output(row, profile, stage, task_root)
            except ReductionError as error:
                raise ReductionError(f"task {index} {error}") from error
            if normal_output in normal_outputs:
                raise ReductionError(f"duplicate successful output XML: {normal_output}")
            normal_outputs.append(normal_output)
        elif row.get("out_xml"):
            raise ReductionError(f"task {index} non-success row owns a normal output XML")
        reason = _transition_reject_reason(row, stage)
        if reason is None:
            assert normal_output is not None
            output = normal_output
            if output in valid_outputs:
                raise ReductionError(f"duplicate validated output XML: {output}")
            valid_outputs.append(output)
        else:
            reject_counts[reason] += 1
        nodecrement_output = row.get("nodecrement_out_xml")
        if nodecrement_output:
            if row.get("status") != "no_opener_decrements_hop":
                raise ReductionError(f"task {index} has nodecrement XML with status {row.get('status')!r}")
            nodecrement = Path(nodecrement_output)
            if not nodecrement.is_file():
                raise ReductionError(f"task {index} references a missing nodecrement XML")
            expected_nodecrement_root = _canon(task_root / "xmls_nodecrement") + os.sep
            canonical_nodecrement = _canon(nodecrement)
            if not canonical_nodecrement.startswith(expected_nodecrement_root):
                raise ReductionError(f"task {index} nodecrement XML is outside its task shard")
            if canonical_nodecrement in nodecrement_outputs:
                raise ReductionError(f"duplicate nodecrement output XML: {canonical_nodecrement}")
            nodecrement_outputs.append(canonical_nodecrement)
        rows.append(row)

    row_inputs = [_canon(row["xml_path"]) for row in rows]
    if len(row_inputs) != len(set(row_inputs)):
        raise ReductionError("merged rows contain duplicate input XMLs")

    xml_artifacts = {
        _canon(path)
        for path in shards_root.glob("task_*/xmls/*.xml")
        if path.is_file()
    }
    unreferenced = sorted(xml_artifacts - set(normal_outputs))
    missing_artifacts = sorted(set(normal_outputs) - xml_artifacts)
    if unreferenced or missing_artifacts:
        raise ReductionError(
            "normal XML mismatch: "
            f"{len(missing_artifacts)} missing, {len(unreferenced)} unreferenced"
        )
    nodecrement_artifacts = {
        _canon(path)
        for path in shards_root.glob("task_*/xmls_nodecrement/*.xml")
        if path.is_file()
    }
    missing_nodecrement = sorted(set(nodecrement_outputs) - nodecrement_artifacts)
    orphan_nodecrement = sorted(nodecrement_artifacts - set(nodecrement_outputs))
    if missing_nodecrement or orphan_nodecrement:
        raise ReductionError(
            "nodecrement XML mismatch: "
            f"{len(missing_nodecrement)} missing, {len(orphan_nodecrement)} orphan"
        )

    status_counts = dict(sorted(Counter(str(row.get("status", "missing")) for row in rows).items()))
    valid_key = "validated_post_k1" if stage == "k1" else "validated_post_k2"
    summary: dict[str, Any] = {
        "profile": profile,
        "stage": stage,
        "manifest": str(manifest_path),
        "manifest_rows": len(inputs),
        "merged_rows": len(rows),
        "unique_inputs": len(set(row_inputs)),
        valid_key: len(valid_outputs),
        "status_counts": status_counts,
        "semantic_reject_counts": dict(sorted(reject_counts.items())),
        "unreferenced_xml_artifacts": unreferenced,
        "nodecrement_xml_artifacts": len(nodecrement_artifacts),
    }
    merged = "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows)
    _atomic_write(stage_root / "merged_rows.jsonl", merged)
    if stage == "k1":
        _atomic_write(stage_root / "post_k1_manifest.txt", "".join(f"{path}\n" for path in valid_outputs))
    _atomic_write(stage_root / "validation.json", json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--stage-root", required=True)
    parser.add_argument("--profile", required=True, choices=("hard1-hard1", "med2-med2"))
    parser.add_argument("--stage", required=True, choices=("k1", "k2"))
    args = parser.parse_args()
    summary = reduce_stage(args.manifest, args.stage_root, args.profile, args.stage)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
