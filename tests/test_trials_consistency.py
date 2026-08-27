"""A row that disagrees with its own command must fail here, not in the analysis.

`trials.csv` will record the arm and the execution mode as columns while also
keeping the `command` string that ran. Two records of one fact drift, and the
drift is silent: a row filed under the wrong arm produces a number rather than an
error, and that number goes in a table beside a number from the other arm.

With 14 scenes x 2 arms x 2 modes analysed paired within scene, losing one cell
does not add noise, it breaks the sign test.

Seven properties, each with the failure it catches:

  a clean file passes            the check cannot be so strict it rejects the
                                 rows it is meant to protect
  a mismatched arm fails         the transcription slip this exists for
  a mismatched mode fails        the same slip on the second crossed factor
  a bad vocabulary fails         "reactve" and "" are mistakes, not values
  an empty cause fails           it reads as both "did not fail" and "not
                                 triaged", and those resolve wrongly at 1am
  success with a cause fails     the outcome and the cause contradict
  a missing file is not an error  so this can be wired into CI before the first
                                 matrix row exists

Pure file parsing: no hardware, no binding, no checkpoint.

To verify:
  cd robot_control && pytest tests/test_trials_consistency.py -v
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from check_trials_consistency import main  # noqa: E402


# ─── Named constants ────────────────────────────────────────────────────

HEADER = [
    "trial_id", "build_id", "tier", "axis", "arm", "exec_mode",
    "started_at", "ended_at", "command", "planner_outcome", "failure_cause",
    "user_verdict", "notes", "scene_checksum", "log_path",
]

CLEAN = 0
VIOLATION = 1

BASE_COMMAND = (
    "python scripts/run_namo.py --algorithm full_namo --local-search best_first "
    "--best-first-prior model --exec-mode reactive --scorer-ckpt HY5U_s2"
)


def _write(tmp_path, **overrides):
    row = {
        "trial_id": "t1",
        "build_id": "v2",
        "tier": "easy",
        "axis": "offset",
        "arm": "model",
        "exec_mode": "reactive",
        "started_at": "2026-08-26T10:00:00",
        "ended_at": "2026-08-26T10:04:00",
        "command": BASE_COMMAND,
        "planner_outcome": "success",
        "failure_cause": "none",
        "user_verdict": "success",
        "notes": "",
        "scene_checksum": "abc123",
        "log_path": "runs/t1",
    }
    row.update(overrides)
    path = tmp_path / "trials.csv"
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=HEADER)
        writer.writeheader()
        writer.writerow(row)
    return path


def _check(path):
    return main(["check_trials_consistency.py", str(path)])


# ─── Tests ──────────────────────────────────────────────────────────────


def test_a_consistent_row_passes(tmp_path):
    """The check must protect the rows, not reject them."""
    assert _check(_write(tmp_path)) == CLEAN


def test_an_arm_that_contradicts_its_command_fails(tmp_path):
    """The slip this exists for: the column says one arm, the run was the other."""
    assert _check(_write(tmp_path, arm="uniform")) == VIOLATION


def test_a_mode_that_contradicts_its_command_fails(tmp_path):
    assert _check(_write(tmp_path, exec_mode="search")) == VIOLATION


@pytest.mark.parametrize("mode", ["reactve", "policy", "", "Reactive"])
def test_a_mode_outside_the_vocabulary_fails(tmp_path, mode):
    assert _check(_write(tmp_path, exec_mode=mode)) == VIOLATION


@pytest.mark.parametrize("arm", ["ml", "random", ""])
def test_an_arm_outside_the_vocabulary_fails(tmp_path, arm):
    assert _check(_write(tmp_path, arm=arm)) == VIOLATION


def test_an_empty_failure_cause_fails(tmp_path):
    """Empty means both 'did not fail' and 'not triaged yet'."""
    assert _check(_write(tmp_path, failure_cause="")) == VIOLATION


def test_a_success_that_names_a_cause_fails(tmp_path):
    assert _check(
        _write(tmp_path, planner_outcome="success", failure_cause="stall")
    ) == VIOLATION


def test_a_failure_that_names_a_cause_passes(tmp_path):
    assert _check(
        _write(
            tmp_path, planner_outcome="failure", failure_cause="corridor_too_tight"
        )
    ) == CLEAN


def test_a_command_that_names_no_flags_is_not_cross_checked(tmp_path):
    """The defaults are what ran, so there is nothing to disagree with."""
    assert _check(
        _write(
            tmp_path,
            command="python scripts/run_namo.py --algorithm full_namo",
            arm="model",
            exec_mode="search",
        )
    ) == CLEAN


def test_a_missing_file_is_not_a_failure(tmp_path):
    """So this can be wired into CI before the first matrix row is written."""
    assert _check(tmp_path / "does_not_exist.csv") == CLEAN


def test_the_old_header_is_reported_rather_than_silently_passing(tmp_path):
    """A file without the new columns has not had the schema landed yet."""
    path = tmp_path / "trials.csv"
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["trial_id", "command", "planner_outcome"])
        writer.writerow(["t1", BASE_COMMAND, "success"])

    assert _check(path) == VIOLATION


def test_running_with_no_path_refuses_rather_than_passing():
    """A guessed default that does not exist would exit 0 having read nothing.

    That is a green light from a check that never opened a file, which is the
    same silent-drop shape the script exists to catch. The earlier version of
    this script did exactly that against an invented path.
    """
    assert main(["check_trials_consistency.py"]) == VIOLATION
