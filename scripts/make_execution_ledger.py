#!/usr/bin/env python3
"""Freeze the randomized four-cell execution order for every matrix scene.

Crossing execution mode with the prior arm gives each of the 14 scenes four
cells: {model,uniform} x {search,reactive}. The pre-registration requires the
cell order per scene to be randomized and FROZEN before the first row is
collected, so nobody can pick or re-randomize cells after seeing results.
Until this script, no manifest supplied that order; the runbook said so.

Deterministic by construction: the seed is fixed here, so re-running
reproduces the ledger byte for byte, which is what makes it pre-registerable.
The scene list is the pre-registered matrix from ICRA_REAL_ROBOT_STUDY.md,
in the doc's order; execution ORDER ACROSS scenes stays the operator's call
(hard pairs outrank everything), only the order WITHIN a scene is frozen.

  python scripts/make_execution_ledger.py            # writes the ledger
  python scripts/make_execution_ledger.py --check    # verifies the file on
                                                     # disk matches the seed
"""

from __future__ import annotations

import csv
import random
import sys
from pathlib import Path
from typing import List, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
LEDGER_PATH = REPO_ROOT / "real_trials/matrix_v2/execution_ledger.csv"

# Same seed the replacement draw was registered under, so the study has one
# pre-registration seed rather than two.
LEDGER_SEED = 20260826

# The pre-registered matrix, verbatim from docs/ICRA_REAL_ROBOT_STUDY.md.
MATRIX_SCENES: Tuple[Tuple[str, str], ...] = (
    ("1push", "hard_037"),
    ("1push", "hard_001"),
    ("1push", "hard_012"),
    ("1push", "hard_005"),
    ("1push", "hard_009"),
    ("hmax2", "hard_007"),
    ("hmax2", "hard_013"),
    ("hmax2", "hard_008"),
    ("1push", "med_085"),
    ("1push", "med_077"),
    ("1push", "med_093"),
    ("hmax2", "med_069"),
    ("1push", "easy_002"),
    ("1push", "easy_001"),
)

CELLS: Tuple[Tuple[str, str], ...] = (
    ("model", "search"),
    ("model", "reactive"),
    ("uniform", "search"),
    ("uniform", "reactive"),
)

HEADER = ["axis", "build_id", "cell_order", "arm", "exec_mode"]


def build_rows() -> List[List[str]]:
    rng = random.Random(LEDGER_SEED)
    rows: List[List[str]] = []
    for axis, build_id in MATRIX_SCENES:
        cells = list(CELLS)
        rng.shuffle(cells)
        for order, (arm, mode) in enumerate(cells, start=1):
            rows.append([axis, build_id, str(order), arm, mode])
    return rows


def main(argv: List[str]) -> int:
    rows = build_rows()
    if "--check" in argv:
        if not LEDGER_PATH.is_file():
            print(f"missing: {LEDGER_PATH}")
            return 1
        with LEDGER_PATH.open(newline="") as fh:
            on_disk = [row for row in csv.reader(fh)]
        if on_disk != [HEADER] + rows:
            print("LEDGER DOES NOT MATCH THE SEED. It was edited after being "
                  "frozen, or the scene list changed. Neither is allowed after "
                  "row 1; investigate before running anything.")
            return 1
        print(f"{LEDGER_PATH}: matches seed {LEDGER_SEED}, "
              f"{len(rows)} cells over {len(MATRIX_SCENES)} scenes")
        return 0

    LEDGER_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LEDGER_PATH.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(HEADER)
        writer.writerows(rows)
    print(f"wrote {LEDGER_PATH}: {len(rows)} cells, "
          f"seed {LEDGER_SEED}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
