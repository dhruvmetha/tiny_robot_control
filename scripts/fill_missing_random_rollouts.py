#!/usr/bin/env python3
"""Backfill missing runN slots under real_test_envs/*/*/*/solution/random_rollout.

Each missing run is launched via ``scripts/run_namo.py --sim`` using the same
layout and arguments as the existing historical runs:

  * ``--sim-xml <env>/env.xml``
  * ``--sim-real-run-dir <env>``
  * ``--strategy random_rollout``
  * ``--rollout-samples-per-state 36000``
  * ``--diag-path <env>/solution/random_rollout``
  * ``--run-name runN``
  * ``--allow-overwrite``

Driver stdout/stderr is captured to ``<env>/solution/random_rollout_runN_driver.log``.

The script continues across non-zero run_namo exit codes because a missing plan
still produces the artifact directory the caller may want to keep.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


EXPECTED_RUNS = tuple(f"run{i}" for i in range(1, 6))


def iter_envs(real_test_envs_root: Path) -> list[Path]:
    """Return all <env> directories at depth real_test_envs/*/*/*."""
    return sorted(p for p in real_test_envs_root.glob("*/*/*") if p.is_dir())


def existing_runs(random_rollout_dir: Path) -> set[str]:
    """Return run directory names currently present under solution/random_rollout."""
    if not random_rollout_dir.is_dir():
        return set()
    return {p.name for p in random_rollout_dir.iterdir() if p.is_dir()}


def launch_missing_run(env_dir: Path, run_name: str, repo_root: Path) -> int:
    """Launch one missing run and mirror stdout/stderr into the driver log."""
    diag_path = env_dir / "solution" / "random_rollout"
    driver_log = env_dir / "solution" / f"random_rollout_{run_name}_driver.log"
    env_rel = env_dir.relative_to(repo_root)

    cmd = [
        sys.executable,
        "scripts/run_namo.py",
        "--sim",
        "--sim-xml",
        str(env_rel / "env.xml"),
        "--sim-real-run-dir",
        str(env_rel),
        "--strategy",
        "random_rollout",
        "--rollout-samples-per-state",
        "36000",
        "--diag-path",
        str(env_rel / "solution" / "random_rollout"),
        "--run-name",
        run_name,
        "--allow-overwrite",
    ]

    child_env = os.environ.copy()
    src_dir = repo_root / "src"
    existing_pythonpath = child_env.get("PYTHONPATH")
    child_env["PYTHONPATH"] = (
        f"{src_dir}{os.pathsep}{existing_pythonpath}"
        if existing_pythonpath
        else str(src_dir)
    )

    print(f"[fill] {env_rel} {run_name}")
    with open(driver_log, "w") as log_file:
        proc = subprocess.run(
            cmd,
            cwd=repo_root,
            env=child_env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    run_dir = diag_path / run_name
    print(
        f"[fill] exit={proc.returncode} "
        f"run_dir={'yes' if run_dir.is_dir() else 'no'} "
        f"log={driver_log.relative_to(repo_root)}"
    )
    return proc.returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run missing sim random_rollout backfills under real_test_envs."
    )
    parser.add_argument(
        "--real-test-envs-root",
        type=Path,
        default=Path("real_test_envs"),
        help="Path to real_test_envs root relative to repo root (default: real_test_envs).",
    )
    parser.add_argument(
        "--max-runs",
        type=int,
        default=None,
        help="Optional cap on the number of missing runs to launch.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print missing runs; do not launch run_namo.py.",
    )
    parser.add_argument(
        "--env-subpath",
        type=str,
        default=None,
        help=(
            "Optional env path relative to the repo root, e.g. "
            "'real_test_envs/1push/1hop/multi_contact'. When set, only that "
            "single env is considered."
        ),
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    real_test_envs_root = (
        args.real_test_envs_root
        if args.real_test_envs_root.is_absolute()
        else repo_root / args.real_test_envs_root
    )

    env_dirs = iter_envs(real_test_envs_root)
    if args.env_subpath is not None:
        target_env = repo_root / args.env_subpath
        env_dirs = [env_dir for env_dir in env_dirs if env_dir == target_env]
        if not env_dirs:
            print(f"[fill] target env not found: {args.env_subpath}", file=sys.stderr)
            return 2

    missing: list[tuple[Path, str]] = []
    for env_dir in env_dirs:
        rr_dir = env_dir / "solution" / "random_rollout"
        present = existing_runs(rr_dir)
        for run_name in EXPECTED_RUNS:
            if run_name not in present:
                missing.append((env_dir, run_name))

    print(f"[fill] envs={len(env_dirs)} missing_runs={len(missing)}")
    for env_dir, run_name in missing:
        print(f"[fill] missing {env_dir.relative_to(repo_root)} {run_name}")

    if args.dry_run or not missing:
        return 0

    launched = 0
    exit_codes: list[int] = []
    for env_dir, run_name in missing:
        if args.max_runs is not None and launched >= args.max_runs:
            break
        exit_codes.append(launch_missing_run(env_dir, run_name, repo_root))
        launched += 1

    nonzero = sum(1 for code in exit_codes if code != 0)
    print(
        f"[fill] launched={launched} nonzero_exit_codes={nonzero} "
        f"zero_exit_codes={launched - nonzero}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
