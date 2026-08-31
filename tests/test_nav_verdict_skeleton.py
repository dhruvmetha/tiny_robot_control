"""The verdict file the operator fills in after watching the drive.

Scene qualification stopped being mechanical on 2026-08-31: not the exit
code, not a row formula, but a judgment call at the table
(ICRA_REAL_ROBOT_STUDY.md step 2). The run leaves the shape and the evidence
so the call can be made and recorded without opening a second file.

To verify:
  cd robot_control && python -m pytest tests/test_nav_verdict_skeleton.py -v
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_navigation_baseline_robot import write_verdict_skeleton  # noqa: E402

from robot_control.planner.navigation_baseline_planner import BaselineOutcome


def _outcome(**kw) -> BaselineOutcome:
    outcome = BaselineOutcome(mode="penalise")
    outcome.reached = kw.get("reached", False)
    outcome.distance_to_goal_cm = kw.get("distance", 33.6)
    outcome.stuck_retries = kw.get("retries", 0)
    outcome.objects_moved_cm = kw.get("moved", {})
    return outcome


def _write(tmp_path, outcome) -> dict:
    write_verdict_skeleton(tmp_path, outcome, Path("nav_baseline_penalise_1.json"))
    return json.loads((tmp_path / "nav_verdict.json").read_text())


def test_qualifies_is_left_unset(tmp_path):
    """Null, not a guess. A default of true or false would be the script
    making the call the doc says a person makes."""
    assert _write(tmp_path, _outcome())["qualifies"] is None


def test_it_carries_the_evidence_the_doc_says_to_read(tmp_path):
    written = _write(tmp_path, _outcome(reached=True, retries=3,
                                        moved={"obj_1": 9.4}))

    evidence = written["_evidence"]
    for field in ("reached", "distance_to_goal_cm", "objects_moved",
                  "objects_moved_cm", "stuck_retries", "stuck_causes",
                  "route_crosses", "failure_cause"):
        assert field in evidence
    # The bulldozing case: arrived, but only after shoving something three times.
    assert evidence["reached"] is True
    assert evidence["stuck_retries"] == 3
    assert evidence["objects_moved_cm"] == {"obj_1": 9.4}


def test_an_existing_verdict_is_never_overwritten(tmp_path):
    """A recorded verdict is a human decision. Rerunning the baseline is not
    grounds to throw it away."""
    (tmp_path / "nav_verdict.json").write_text(
        json.dumps({"qualifies": False, "reason": "drove straight there"})
    )

    write_verdict_skeleton(tmp_path, _outcome(), Path("row.json"))

    kept = json.loads((tmp_path / "nav_verdict.json").read_text())
    assert kept["qualifies"] is False
    assert kept["reason"] == "drove straight there"


def test_it_creates_the_scene_directory_if_missing(tmp_path):
    scene = tmp_path / "twohop_00013"

    write_verdict_skeleton(scene, _outcome(), Path("row.json"))

    assert (scene / "nav_verdict.json").exists()
