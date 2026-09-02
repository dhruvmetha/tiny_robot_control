"""check_build must read a twohop scene's per-scene build_sheet.json.

The twohop generator writes one build_sheet.json per scene directory rather
than a row in a campaign CSV. Before this support, the operator setting up a
formal_v2 scene had no live placement guidance at all, just raw coordinates.
"""

from importlib import import_module
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
check_build = import_module("check_build")

SHEET = (Path(__file__).resolve().parents[1] / "real_exp" / "environments" /
         "twohop_selected" / "med2-med2" / "twohop_00001" / "build_sheet.json")


def test_twohop_sheet_yields_placement_rows():
    rows = check_build.load_sheet_rows(SHEET, "twohop_00001")

    hints = [r["marker_hint"] for r in rows]
    assert hints == ["wall_9", "wall_10", "wall_11", "wall_12", "obj_4", "obj_1"]
    assert {r["item"] for r in rows} == {"brick", "block"}
    # Spot-check one brick and one block against the sheet's numbers.
    wall_9 = rows[0]
    assert float(wall_9["centre_x_cm"]) == 10.1
    assert float(wall_9["long_cm"]) == 19.0
    obj_1 = rows[-1]
    assert float(obj_1["long_axis_bearing_deg"]) == 164.3
    # Shared fields every row carries.
    assert all(r["push_kind"] == "needs_2_chain" for r in rows)
    assert all(r["build_id"] == "twohop_00001" for r in rows)


def test_wrong_build_id_exits_loudly():
    with pytest.raises(SystemExit):
        check_build.load_sheet_rows(SHEET, "twohop_00099")


def test_multiobject_twohop_sheet_yields_physical_rows(tmp_path):
    sheet = {
        "scene_id": "twohop_multi_00015",
        "requested_profile": "hard1-hard1",
        "robot_start_cm": [8.0, 9.0],
        "robot_start_bearing_deg": 0.0,
        "goal_cm": [8.0, 64.0],
        "gate_walls": [
            {
                "physical_id": "wall_9",
                "center_cm": [38.9, 17.5],
                "long_axis_bearing_deg": 10.6,
                "long_cm": 19.5,
                "short_cm": 5.5,
            },
            {
                "physical_id": "wall_10",
                "center_cm": [13.9, 26.5],
                "long_axis_bearing_deg": 70.5,
                "long_cm": 19.5,
                "short_cm": 5.5,
            },
        ],
        "movables": [
            {
                "physical_role": "obj_4a",
                "center_cm": [29.4, 49.0],
                "long_axis_bearing_deg": 8.8,
                "long_cm": 12.0,
                "short_cm": 7.5,
            },
            {
                "physical_role": "obj_4b",
                "center_cm": [26.0, 22.0],
                "long_axis_bearing_deg": 22.6,
                "long_cm": 12.0,
                "short_cm": 7.5,
            },
        ],
    }
    path = tmp_path / "build_sheet.json"
    path.write_text(json.dumps(sheet))

    rows = check_build.load_sheet_rows(path, "twohop_multi_00015")

    assert [row["marker_hint"] for row in rows] == [
        "wall_9",
        "wall_10",
        "obj_4",
        "obj_4b",
    ]
    assert [row["item"] for row in rows] == ["brick", "brick", "block", "block"]
    assert all(row["n_bricks"] == "2" for row in rows)
    assert all(row["push_kind"] == "hard1-hard1" for row in rows)
