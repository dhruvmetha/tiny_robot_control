#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import re
import sys
import time
from pathlib import Path

import yaml

DEFAULT_REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_closed_loop_module(repo_root: Path):
    spec = importlib.util.spec_from_file_location(
        "closed_loop_session_mod",
        repo_root / "robot_control" / "scripts" / "closed_loop_session.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _env_sort_key(path: Path):
    m = re.fullmatch(r"env(\d+)", path.name)
    if m:
        return (0, int(m.group(1)), path.name)
    if path.name == "multi_contact":
        return (1, 0, path.name)
    return (2, 0, path.name)


def _session_dirs(closed_loop_root: Path, push_dir: str, hop_dir: str) -> list[Path]:
    base = closed_loop_root / push_dir / hop_dir
    out: list[Path] = []
    if not base.exists():
        return out
    for env_dir in sorted([p for p in base.iterdir() if p.is_dir()], key=_env_sort_key):
        sessions_dir = env_dir / "sessions"
        if not sessions_dir.exists():
            continue
        for session_dir in sorted([p for p in sessions_dir.iterdir() if p.is_dir()]):
            out.append(session_dir)
    return out


def _session_sort_key(session_dir: Path):
    try:
        env_dir = session_dir.parents[1]
        push_dir = session_dir.parents[3]
        hop_dir = session_dir.parents[2]
        return (push_dir.name, hop_dir.name, _env_sort_key(env_dir), session_dir.name)
    except Exception:
        return (str(session_dir),)


def _load_solution(sol_path: Path):
    if not sol_path.exists():
        return None
    return yaml.safe_load(sol_path.read_text()) or {}


def _dump_summary(path: Path, summary: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=DEFAULT_REPO_ROOT)
    parser.add_argument("--closed-loop-root", type=Path, required=True)
    parser.add_argument("--push-dir", type=str, default=None)
    parser.add_argument("--hop-dir", type=str, default=None)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--run-name", type=str, default=None)
    parser.add_argument("--summary-path", type=Path, required=True)
    parser.add_argument(
        "--exclude-session-substr",
        action="append",
        default=[],
        help="Skip any session whose path contains this substring. Repeatable.",
    )
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    closed_loop_root = args.closed_loop_root.resolve()
    summary_path = args.summary_path.resolve()
    mod = _load_closed_loop_module(repo_root)

    run_order = [
        "primitive_run1", "primitive_run2", "primitive_run3",
        "random_rollout_run1", "random_rollout_run2", "random_rollout_run3",
    ]
    if args.run_name is not None and args.run_name not in run_order:
        raise SystemExit(f"Unsupported --run-name: {args.run_name}")

    if args.session_dir is None and (not args.push_dir or not args.hop_dir):
        raise SystemExit("Need either --session-dir or both --push-dir and --hop-dir")

    summary = {
        "closed_loop_root": str(closed_loop_root),
        "push_dir": args.push_dir,
        "hop_dir": args.hop_dir,
        "session_dir": str(args.session_dir.resolve()) if args.session_dir is not None else None,
        "run_name": args.run_name,
        "started_at_epoch": time.time(),
        "runs": [],
    }
    _dump_summary(summary_path, summary)

    if args.session_dir is not None:
        target_sessions = [args.session_dir.resolve()]
    else:
        target_sessions = _session_dirs(closed_loop_root, args.push_dir, args.hop_dir)

    selected_runs = [args.run_name] if args.run_name is not None else run_order

    for session_dir in sorted(target_sessions, key=_session_sort_key):
        session_str = str(session_dir)
        if any(substr and substr in session_str for substr in args.exclude_session_substr):
            print(f"=== SKIP SESSION {session_dir} ===", flush=True)
            continue
        print(f"=== SESSION {session_dir} ===", flush=True)
        for run_name in selected_runs:
            run_dir = session_dir / run_name
            iter_dir = run_dir / "iter_001"
            sol_path = iter_dir / "sim_candidates" / "candidate1" / "solution.yaml"
            selected_trial = iter_dir / "selected_trial_spec.yaml"
            selected_plan = iter_dir / "selected_plan.json"
            entry = {
                "session_dir": str(session_dir),
                "run_name": run_name,
                "strategy": "primitive" if run_name.startswith("primitive_") else "random_rollout",
                "iteration": 1,
                "replanned": False,
                "prepared": False,
                "success": False,
                "sim_pushes_tried": None,
                "search_time_ms": None,
                "error": None,
            }
            try:
                existing = _load_solution(sol_path)
                existing_pushes = None
                if existing is not None:
                    existing_pushes = ((existing.get("search_stats") or {}).get("sim_pushes_tried"))
                already_prepared = selected_trial.exists() and selected_plan.exists()
                need_replan = existing is None or existing_pushes is None

                if need_replan:
                    print(f"[REPLAN] {run_name}", flush=True)
                    t0 = time.time()
                    result = mod.replan_iteration(session_dir, run_name, 1)
                    dt = time.time() - t0
                    entry["replanned"] = True
                    entry["replan_result"] = result
                    print(
                        f"[REPLAN DONE] {run_name} success={result.get('successful_plan')} "
                        f"strategy={result.get('strategy')} elapsed_s={dt:.1f}",
                        flush=True,
                    )
                    if not result.get("successful_plan"):
                        entry["error"] = "planning_failed"
                        summary["runs"].append(entry)
                        _dump_summary(summary_path, summary)
                        continue
                else:
                    print(f"[SKIP REPLAN] {run_name} existing sim_pushes_tried={existing_pushes}", flush=True)

                solution = _load_solution(sol_path)
                search_stats = (solution or {}).get("search_stats") or {}
                entry["sim_pushes_tried"] = search_stats.get("sim_pushes_tried")
                entry["search_time_ms"] = search_stats.get("search_time_ms")
                entry["success"] = bool((solution or {}).get("success"))
                if entry["sim_pushes_tried"] is None:
                    entry["error"] = "missing_sim_pushes_tried"
                    print(f"[WARN] {run_name} missing sim_pushes_tried after solution generation", flush=True)
                    summary["runs"].append(entry)
                    _dump_summary(summary_path, summary)
                    continue

                if not already_prepared or entry["replanned"]:
                    prep = mod.prepare_real_push(session_dir, run_name, 1)
                    entry["prepared"] = True
                    entry["prepare_result"] = prep
                    print(
                        f"[PREPARED] {run_name} obj={prep['selected_real_object_id']} "
                        f"edge={prep['edge_idx']} push_steps={prep['push_steps']} "
                        f"sim_pushes_tried={entry['sim_pushes_tried']}",
                        flush=True,
                    )
                else:
                    print(f"[SKIP PREPARE] {run_name} already prepared", flush=True)
                    entry["prepared"] = True

            except Exception as exc:
                entry["error"] = repr(exc)
                print(f"[ERROR] {run_name}: {exc!r}", flush=True)
            summary["runs"].append(entry)
            _dump_summary(summary_path, summary)

    summary["finished_at_epoch"] = time.time()
    _dump_summary(summary_path, summary)
    print(f"WROTE {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
