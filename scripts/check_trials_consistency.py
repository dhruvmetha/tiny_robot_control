#!/usr/bin/env python3
"""Every trial row must agree with the command that produced it.

`trials.csv` records which arm and which execution mode ran as parsed columns,
and also carries the full `command` string that ran. The columns are what the
analysis groups on; the command is the only complete record of what happened.
Two records of one fact drift, and the drift is silent: a row filed under the
wrong arm does not fail, it produces a number that goes in a table.

The matrix is 14 scenes x 2 arms x 2 modes, analysed paired within scene, so
losing one cell does not add noise, it breaks the sign test. This check turns a
transcription slip into a failure at the table rather than a discovery during
analysis.

Four rules:

  the vocabularies are closed        `arm` and `exec_mode` outside their fixed
                                     sets, including empty, are rejected
  the columns match the command      `--best-first-prior X` and `--exec-mode Y`
                                     in `command` must equal the columns. A
                                     command that names neither is not checked
                                     against, since the defaults are what ran
  a success names no cause           `planner_outcome` success with a
                                     `failure_cause` other than `none` is one of
                                     the two disagreeing
  a failure names one                `failure_cause` empty on any row, which
                                     reads as both "did not fail" and "nobody
                                     triaged this yet"

Exit 0 clean, 1 on any violation, and 0 with a note when the file does not exist
yet, so this can be wired into CI before the first matrix row is written.

  python scripts/check_trials_consistency.py [path/to/trials.csv]
"""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_control.planner.search_config import (  # noqa: E402
    BEST_FIRST_PRIOR_CHOICES,
    EXEC_MODE_CHOICES,
)

DEFAULT_TRIALS_PATH = Path("closed_loop_sessions/trials.csv")

# The success vocabulary for `failure_cause`. The seven failure values are
# real_robot's taxonomy and this check does not police them beyond "not empty":
# a person assigns the cause at verdict time and inventing a stricter rule here
# would reject their judgement.
NO_FAILURE = "none"

# Only these two flags are cross-checked. Both name a factor of the design, and
# both have a column. Everything else in `command` is provenance, not a grouping
# key.
_ARM_IN_COMMAND = re.compile(r"--best-first-prior[= ]+(\S+)")
_MODE_IN_COMMAND = re.compile(r"--exec-mode[= ]+(\S+)")

# What `planner_outcome` says when the trial worked. Kept as a set rather than
# one string because the column predates this check and may already carry more
# than one spelling.
_SUCCESS_OUTCOMES = {"success", "solved", "opened", "true"}


def _violations(row: Dict[str, str], line: int) -> List[str]:
    out: List[str] = []
    arm = (row.get("arm") or "").strip()
    mode = (row.get("exec_mode") or "").strip()
    cause = (row.get("failure_cause") or "").strip()
    command = row.get("command") or ""
    outcome = (row.get("planner_outcome") or "").strip().lower()

    if arm not in BEST_FIRST_PRIOR_CHOICES:
        out.append(f"arm={arm!r} is not one of {list(BEST_FIRST_PRIOR_CHOICES)}")
    if mode not in EXEC_MODE_CHOICES:
        out.append(f"exec_mode={mode!r} is not one of {list(EXEC_MODE_CHOICES)}")

    said_arm = _ARM_IN_COMMAND.search(command)
    if said_arm and said_arm.group(1) != arm:
        out.append(
            f"arm column says {arm!r} but the command ran "
            f"--best-first-prior {said_arm.group(1)!r}"
        )
    said_mode = _MODE_IN_COMMAND.search(command)
    if said_mode and said_mode.group(1) != mode:
        out.append(
            f"exec_mode column says {mode!r} but the command ran "
            f"--exec-mode {said_mode.group(1)!r}"
        )

    if not cause:
        out.append(
            "failure_cause is empty, which reads as both 'did not fail' and "
            f"'not triaged yet'; use {NO_FAILURE!r} for a success"
        )
    elif outcome in _SUCCESS_OUTCOMES and cause != NO_FAILURE:
        out.append(
            f"planner_outcome={outcome!r} but failure_cause={cause!r}; "
            "one of the two is wrong"
        )

    return [f"row {line}: {v}" for v in out]


def main(argv: List[str]) -> int:
    path = Path(argv[1]) if len(argv) > 1 else DEFAULT_TRIALS_PATH
    if not path.is_file():
        print(f"no trials file at {path}, nothing to check")
        return 0

    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))

    if not rows:
        print(f"{path}: no rows")
        return 0

    missing = {"arm", "exec_mode", "failure_cause"} - set(rows[0])
    if missing:
        print(f"{path}: header is missing {sorted(missing)} -- schema not landed yet")
        return 1

    problems: List[str] = []
    for offset, row in enumerate(rows, start=2):  # 1 is the header
        problems.extend(_violations(row, offset))

    if problems:
        print(f"{path}: {len(problems)} problem(s) across {len(rows)} row(s)")
        for p in problems:
            print(f"  {p}")
        return 1

    print(f"{path}: {len(rows)} row(s) consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
