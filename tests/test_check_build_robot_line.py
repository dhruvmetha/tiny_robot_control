"""check_build shows the robot start as a live placement line.

The items had live dx/dy/dtheta guidance and the robot did not, so the one
pose the scene's reachability labels are measured from was the one placed
blind. The robot line compares against the sheet's start pose with the full
360-degree heading period and a looser heading band, since a car pivots in
place but a position error changes what the wavefront reaches.
"""

from importlib import import_module
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
check_build = import_module("check_build")

HEAD = {
    "robot_start_x_cm": "27.8",
    "robot_start_y_cm": "7.4",
    "robot_start_bearing_deg": "0.0",
}


def test_robot_on_pose_reads_ok():
    line = check_build.line_for_robot(HEAD, (27.9, 7.5, 5.0))
    assert "OK" in line


def test_heading_wraps_through_zero():
    # 350 deg against a 0 deg target is a -10 deg error, not +350.
    line = check_build.line_for_robot(HEAD, (27.8, 7.4, 350.0))
    assert "-10.0deg" in line
    assert "OK" in line


def test_position_error_is_not_ok():
    # 11 cm high, the exact miss from the 2026-08-31 table session.
    line = check_build.line_for_robot(HEAD, (27.3, 18.4, 0.0))
    assert "OK" not in line


def test_heading_error_beyond_band_is_not_ok():
    line = check_build.line_for_robot(HEAD, (27.8, 7.4, 101.0))
    assert "OK" not in line


def test_unseen_robot_reads_missing():
    line = check_build.line_for_robot(HEAD, None)
    assert "MISSING" in line


def test_gui_close_gate_requires_robot_in_place():
    # Objects can PASS their checksum while the robot is parked anywhere;
    # the auto-close gate must hold the window open until the robot is OK.
    assert not check_build.robot_placed(HEAD, {})
    assert not check_build.robot_placed(
        HEAD, {check_build.ROBOT_KEY: (27.3, 18.4, 101.0)})
    assert check_build.robot_placed(
        HEAD, {check_build.ROBOT_KEY: (27.9, 7.5, 350.0)})
    # A sheet with no start pose never blocks the close.
    assert check_build.robot_placed({}, {})


def test_simulated_observation_includes_robot():
    rows = [{
        "marker_hint": "wall_10", "item": "brick",
        "centre_x_cm": "10.0", "centre_y_cm": "20.0",
        "long_axis_bearing_deg": "0.0", "long_cm": "19.5", "short_cm": "5.5",
        **HEAD,
    }]
    observed = check_build.simulated_observation(rows, None, 0.0, 0.0)
    assert observed[check_build.ROBOT_KEY] == (27.8, 7.4, 0.0)
