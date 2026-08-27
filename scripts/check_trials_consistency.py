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

Exit 0 clean, 1 on any violation, and 0 with a note when a NAMED file does not
exist yet, so this can be wired in before the first matrix row is written.

The path is required and there is deliberately no default. An earlier version
defaulted to a guessed location, and run with no argument it printed "nothing to
check" and exited 0 having checked nothing. A green light from a check that never
read a file is worse than no check, and it is the same silent-drop shape this
script exists to catch. Naming the file is the caller's job.

Worth knowing before wiring this into CI: `trials.csv` lives under a gitignored
directory in robot_control, so it is not merely absent from a fresh checkout, it
is excluded by rule. CI cannot read it from the repo. Point this at the working
copy on the machine that owns the file, or at an artefact.

  python scripts/check_trials_consistency.py path/to/trials.csv
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

# `failure_cause` in full, from docs/ICRA_REAL_ROBOT_STUDY.md and
# docs/REAL_ROBOT_TRIALS.md. The value IS policed, and an earlier version of
# this check deliberately did not police it, on the reasoning that a stricter rule
# would reject the human's judgement at the table. That has it backwards. The
# vocabulary is fixed because the analysis groups on it, so a typo like `stal`
# or an invented value like `battery` does not read as judgement, it reads as a
# category of one that nobody notices until the table is drawn. `other` is in
# the list precisely so a cause that does not fit still has somewhere to go.
NO_FAILURE = "none"
FAILURE_CAUSES = (
    "corridor_too_tight",
    "marker_unreachable",
    "overshoot_onto_goal",
    "stall",
    "radio_dropout",
    "planning_failed",
    "other",
)
CAUSE_CHOICES = (NO_FAILURE,) + FAILURE_CAUSES

# Only these two flags are cross-checked. Both name a factor of the design, and
# both have a column. Everything else in `command` is provenance, not a grouping
# key.
_ARM_IN_COMMAND = re.compile(r"--best-first-prior[= ]+(\S+)")
_MODE_IN_COMMAND = re.compile(r"--exec-mode[= ]+(\S+)")

# Success is read off `user_verdict`, not `planner_outcome`. The runbook is
# explicit that the human's verdict is ground truth and the planner's exit code
# is not, and the outcome column is free prose: the pilot rows carry "push ok +
# region goal reached" and "goal reached 0.9cm 1 push", which no fixed set of
# words matches. An earlier version keyed off `planner_outcome` against
# {"success","solved","opened","true"} and so never fired on a real row.
# Verdicts are prose too, but they start with the word: "success" and "success
# (region semantics)" both count, "n/a" is neither success nor failure and
# constrains nothing beyond the vocabulary.
_SUCCESS_VERDICT_PREFIX = "success"
_NOT_A_TRIAL_VERDICT = "n/a"


def _violations(row: Dict[str, str], line: int) -> List[str]:
    out: List[str] = []
    arm = (row.get("arm") or "").strip()
    mode = (row.get("exec_mode") or "").strip()
    cause = (row.get("failure_cause") or "").strip()
    command = row.get("command") or ""
    verdict = (row.get("user_verdict") or "").strip().lower()

    if arm not in BEST_FIRST_PRIOR_CHOICES:
        out.append(f"arm={arm!r} is not one of {list(BEST_FIRST_PRIOR_CHOICES)}")
    if mode not in EXEC_MODE_CHOICES:
        out.append(f"exec_mode={mode!r} is not one of {list(EXEC_MODE_CHOICES)}")
    if cause not in CAUSE_CHOICES:
        out.append(f"failure_cause={cause!r} is not one of {list(CAUSE_CHOICES)}")

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
    if not said_arm and not said_mode:
        # Neither flag appears, so the cross-check compared nothing. The matrix
        # command names both, so this means the column holds a prose summary
        # instead of what ran. Saying "consistent" there claims a comparison
        # that did not happen.
        out.append(
            "command names neither --best-first-prior nor --exec-mode, so the "
            "columns were not checked against anything; record the command that "
            "actually ran"
        )

    if verdict.startswith(_SUCCESS_VERDICT_PREFIX) and cause != NO_FAILURE:
        out.append(
            f"user_verdict={verdict!r} but failure_cause={cause!r}; "
            "one of the two is wrong"
        )
    if (
        verdict
        and not verdict.startswith(_SUCCESS_VERDICT_PREFIX)
        and verdict != _NOT_A_TRIAL_VERDICT
        and cause == NO_FAILURE
    ):
        out.append(
            f"user_verdict={verdict!r} is not a success but failure_cause is "
            f"{NO_FAILURE!r}; a failed trial names its cause"
        )

    return [f"row {line}: {v}" for v in out]


def main(argv: List[str]) -> int:
    if len(argv) < 2:
        print(
            "usage: check_trials_consistency.py path/to/trials.csv\n"
            "  No default: a guessed path that does not exist would exit 0 having\n"
            "  read nothing, which reads as a pass."
        )
        return 1
    path = Path(argv[1])
    if not path.is_file():
        print(f"no trials file at {path}, nothing to check")
        return 0

    # Width first, because DictReader hides the failure this catches. An
    # unquoted comma in `notes` splits one field into two, every column after it
    # shifts left by one, and DictReader silently files the overflow under the
    # key None. Two of the four pilot rows were in exactly that state, with the
    # scene checksum sitting in `log_path`. Nothing read the file loudly enough
    # to notice for three days.
    with path.open(newline="") as fh:
        raw = list(csv.reader(fh))
    if raw:
        width = len(raw[0])
        bad_width = [
            f"row {i}: {len(r)} fields, header has {width}"
            + (" (an unquoted comma in a free-text column splits it)" if len(r) > width else "")
            for i, r in enumerate(raw[1:], start=2)
            if len(r) != width
        ]
        if bad_width:
            print(f"{path}: {len(bad_width)} row(s) are the wrong width")
            for b in bad_width:
                print(f"  {b}")
            return 1

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
