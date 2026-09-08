#!/usr/bin/env python3
"""Audit and aggregate the 100-scene dynamic navigation-baseline run."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple


MODES = ("ignore", "penalise")
EXPECTED_SPEED = 0.4
EXPECTED_HZ = 30.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_csv(
    path: Path,
    rows: list[Dict[str, Any]],
    fieldnames: list[str],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def audit_trajectory(path: Path, result: Dict[str, Any]) -> Dict[str, Any]:
    rows = 0
    max_abs_command = 0.0
    max_abs_navigation_command = 0.0
    max_abs_retreat_command = 0.0
    final_action = None
    all_finite = True
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            rows += 1
            values = [
                row["simulation_time_s"],
                row["robot"]["x_cm"],
                row["robot"]["y_cm"],
                row["robot"]["theta_deg"],
            ]
            for pose in row["objects"].values():
                values.extend((pose["x_cm"], pose["y_cm"], pose["theta_deg"]))
            action = row["action"]
            if action is not None:
                values.extend((action["left_speed"], action["right_speed"]))
                action_magnitude = max(
                    abs(float(action["left_speed"])),
                    abs(float(action["right_speed"])),
                )
                max_abs_command = max(max_abs_command, action_magnitude)
                if row["controller_state"] == "RETREATING":
                    max_abs_retreat_command = max(
                        max_abs_retreat_command, action_magnitude
                    )
                else:
                    max_abs_navigation_command = max(
                        max_abs_navigation_command, action_magnitude
                    )
            final_action = action
            all_finite = all_finite and all(
                math.isfinite(float(value)) for value in values
            )

    return {
        "rows": rows,
        "rows_match_ticks": rows == int(result["control_ticks"]) + 1,
        "max_abs_command": max_abs_command,
        "max_abs_navigation_command": max_abs_navigation_command,
        "max_abs_retreat_command": max_abs_retreat_command,
        "commands_within_normalized_range": max_abs_command <= 1.0 + 1e-12,
        "navigation_commands_within_speed": (
            max_abs_navigation_command <= EXPECTED_SPEED + 1e-12
        ),
        "terminal_stop": final_action == {"left_speed": 0.0, "right_speed": 0.0},
        "all_finite": all_finite,
    }


def read_arm(scene_out: Path, mode: str) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    arm_dir = scene_out / f"nav_baseline_{mode}"
    result_path = arm_dir / "result.json"
    trajectory_path = arm_dir / "trajectory.jsonl"
    if not result_path.is_file() or not trajectory_path.is_file():
        raise FileNotFoundError(f"missing {mode} output under {scene_out}")
    result = json.loads(result_path.read_text(encoding="utf-8"))
    audit = audit_trajectory(trajectory_path, result)
    return result, audit


def arm_fields(result: Dict[str, Any], audit: Dict[str, Any]) -> Dict[str, Any]:
    outcome = result["outcome"]
    return {
        "status": result["status"],
        "reached": bool(outcome["reached"]),
        "reason": result["reason"],
        "failure_cause": outcome["failure_cause"],
        "distance_to_goal_cm": outcome["distance_to_goal_cm"],
        "objects_moved": outcome["objects_moved"],
        "stuck_retries": outcome["stuck_retries"],
        "stuck_causes": outcome["stuck_causes"],
        "control_ticks": result["control_ticks"],
        "simulation_time_s": result["simulation_time_s"],
        "wall_clock_elapsed_s": result["wall_clock_elapsed_s"],
        "planning_calls": result["planning_calls"],
        "planning_wall_clock_s": result["planning_wall_clock_s"],
        "n_sims": result["n_sims"],
        "max_abs_command": audit["max_abs_command"],
        "max_abs_navigation_command": audit["max_abs_navigation_command"],
        "max_abs_retreat_command": audit["max_abs_retreat_command"],
    }


def validate_arm(
    scene: str,
    mode: str,
    result: Dict[str, Any],
    audit: Dict[str, Any],
) -> Iterable[str]:
    prefix = f"{scene}:{mode}"
    if result["arm"] != f"nav_baseline_{mode}":
        yield f"{prefix}: wrong arm {result['arm']!r}"
    if not math.isclose(float(result["nav_speed"]), EXPECTED_SPEED):
        yield f"{prefix}: nav_speed={result['nav_speed']}"
    if not math.isclose(float(result["control_hz"]), EXPECTED_HZ):
        yield f"{prefix}: control_hz={result['control_hz']}"
    if int(result["n_sims"]) != 0:
        yield f"{prefix}: n_sims={result['n_sims']}"
    expected_stack = {
        "planner": "NavigationBaselinePlanner",
        "executor": "SubgoalExecutor",
        "navigation_controller": "NavigationController",
        "path_follower": "FollowPathController",
    }
    if result["controller_stack"] != expected_stack:
        yield f"{prefix}: wrong controller stack {result['controller_stack']!r}"
    for key in (
        "rows_match_ticks",
        "commands_within_normalized_range",
        "navigation_commands_within_speed",
        "terminal_stop",
        "all_finite",
    ):
        if not audit[key]:
            yield f"{prefix}: trajectory audit failed: {key}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--expected-count", type=int, default=100)
    args = parser.parse_args()

    root = args.root.expanduser().resolve()
    manifest = args.manifest.expanduser().resolve()
    scenes = [Path(line.strip()) for line in manifest.read_text().splitlines() if line.strip()]
    if len(scenes) != args.expected_count or len(set(scenes)) != len(scenes):
        raise ValueError(
            f"manifest has {len(scenes)} rows and {len(set(scenes))} unique scenes; "
            f"expected {args.expected_count}"
        )

    rows = []
    violations = []
    profile_counts: Dict[str, Counter[str]] = defaultdict(Counter)
    for index, scene_dir in enumerate(scenes):
        profile = scene_dir.parent.name
        scene_id = scene_dir.name
        scene_out = root / "scenes" / profile / scene_id
        results = {}
        row: Dict[str, Any] = {
            "manifest_index": index,
            "profile": profile,
            "scene_id": scene_id,
            "scene_dir": str(scene_dir),
        }
        for mode in MODES:
            result, audit = read_arm(scene_out, mode)
            results[mode] = result
            violations.extend(validate_arm(str(scene_dir), mode, result, audit))
            row.update(
                {f"{mode}_{key}": value for key, value in arm_fields(result, audit).items()}
            )

        ignore_reached = bool(results["ignore"]["outcome"]["reached"])
        penalise_reached = bool(results["penalise"]["outcome"]["reached"])
        if ignore_reached and penalise_reached:
            category = "both_succeeded"
        elif ignore_reached:
            category = "ignore_only_succeeded"
        elif penalise_reached:
            category = "penalise_only_succeeded"
        else:
            category = "both_failed"
        row["navigation_category"] = category
        row["any_navigation_succeeded"] = ignore_reached or penalise_reached
        profile_counts[profile][category] += 1
        rows.append(row)

    if violations:
        raise RuntimeError("\n".join(violations[:50]))

    csv_path = root / "navigation_realstack.csv"
    fieldnames = list(rows[0])
    write_csv(csv_path, rows, fieldnames)
    both_failed_path = root / "both_navigation_baselines_failed.csv"
    write_csv(
        both_failed_path,
        [row for row in rows if row["navigation_category"] == "both_failed"],
        fieldnames,
    )

    overall = Counter(row["navigation_category"] for row in rows)
    summary = {
        "schema_version": 1,
        "scene_count": len(rows),
        "arm_count": len(rows) * len(MODES),
        "manifest": str(manifest),
        "manifest_sha256": sha256(manifest),
        "expected_nav_speed": EXPECTED_SPEED,
        "expected_control_hz": EXPECTED_HZ,
        "command_audit": {
            "navigation_state_limit": EXPECTED_SPEED,
            "retreat_state_limit": 1.0,
            "note": (
                "The real controller uses the configured 0.4 navigation cap; "
                "its shared reverse-toward retreat helper independently clamps "
                "steering commands to the normalized [-1, 1] actuator range."
            ),
        },
        "controller_stack_audit_passed": True,
        "trajectory_audit_passed": True,
        "overall": dict(sorted(overall.items())),
        "by_profile": {
            profile: dict(sorted(counts.items()))
            for profile, counts in sorted(profile_counts.items())
        },
        "csv": str(csv_path),
        "csv_sha256": sha256(csv_path),
        "both_failed_csv": str(both_failed_path),
        "both_failed_csv_sha256": sha256(both_failed_path),
    }
    summary_path = root / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
