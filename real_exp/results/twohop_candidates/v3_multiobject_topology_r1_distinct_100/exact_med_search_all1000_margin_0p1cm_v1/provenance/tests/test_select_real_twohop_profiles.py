from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "pipeline"))

import select_real_twohop_profiles as selector  # noqa: E402


def _counts(*, tried=100, valid=0, valid_first=0):
    return {
        "tried": tried,
        "valid": valid,
        "valid_first_push": valid_first,
        "timed_out": False,
    }


def _sets(*, tried=100, valid=0, valid_first=0):
    cells = [[index, 0] for index in range(tried)]
    return {
        "object_id": "obstacle_1_movable",
        "tried_1push": cells,
        "valid_1push": cells[:valid],
        "valid_first_push": cells[valid : valid + valid_first],
        "depth2_censored": False,
    }


def _write_state_xml(path, obstacle_0, obstacle_1, obstacle_2=None):
    helper = (
        f"<geom name='obstacle_2_movable' pos='{obstacle_2[0]} {obstacle_2[1]} 0.02' "
        f"euler='0 0 {obstacle_2[2]}'/>"
        if obstacle_2 is not None
        else ""
    )
    path.write_text(
        "<mujoco><compiler angle='degree'/><worldbody><body>"
        f"<geom name='obstacle_0_movable' pos='{obstacle_0[0]} {obstacle_0[1]} 0.02' "
        f"euler='0 0 {obstacle_0[2]}'/>"
        f"<geom name='obstacle_1_movable' pos='{obstacle_1[0]} {obstacle_1[1]} 0.02' "
        f"euler='0 0 {obstacle_1[2]}'/>"
        f"{helper}"
        "</body></worldbody></mujoco>",
        encoding="utf-8",
    )


def test_classification_uses_canonical_axes_and_preserves_pure_two_push():
    hard_one = selector.classify_episode(_counts(valid=4))
    medium_two = selector.classify_episode(_counts(valid=0, valid_first=20))

    assert hard_one["tier_1push"] == "hard"
    assert hard_one["push_kind"] == "one_push"
    assert medium_two["tier_hmax2"] == "med"
    assert medium_two["push_kind"] == "needs_2_chain"
    assert medium_two["solve_rate_hmax2"] == 0.2


def test_profile_matching_requires_both_local_gates():
    hard = selector.classify_episode(_counts(valid=4))
    med2 = selector.classify_episode(_counts(valid_first=20))

    assert selector.matches_profile(hard, hard, "hard1-hard1")
    assert selector.matches_profile(med2, med2, "med2-med2")
    assert not selector.matches_profile(hard, med2, "hard1-hard1")
    assert not selector.matches_profile(med2, hard, "med2-med2")


def test_select_joins_k2_by_materialized_post_k1_xml_and_emits_measured_sheet(tmp_path):
    pool = tmp_path / "pool"
    scene_dir = pool / "twohop_00000"
    scene_dir.mkdir(parents=True)
    original_xml = scene_dir / "env.xml"
    _write_state_xml(original_xml, (0.10, 0.20, 0.0), (0.30, 0.40, 0.0))
    sheets = pool / "build_sheets.json"
    sheets.write_text(
        json.dumps(
            [
                {
                    "scene_id": "twohop_00000",
                    "requested_profile": "med2-med2",
                    "classification_status": "unlabelled",
                    "object_order": ["obj_1", "obj_4"],
                }
            ]
        ),
        encoding="utf-8",
    )

    post_xml = tmp_path / "round1" / "xmls" / "scene.xml"
    post_xml.parent.mkdir(parents=True)
    post_xml.write_text("<mujoco/>", encoding="utf-8")
    k1_rows = tmp_path / "k1_rows.jsonl"
    k1_rows.write_text(
        json.dumps(
            {
                "xml_path": str(original_xml),
                "out_xml": str(post_xml),
                "status": "ok",
                "hop_count": 2,
                "post_hop_count": 1,
                "kh1_boundary_objects": ["obstacle_0_movable"],
                "next_boundary_objects": ["obstacle_1_movable"],
                "post_next_boundary_objects": ["obstacle_1_movable"],
                "next_boundary_matches": True,
                "kh1_timed_out": False,
                "canonical_opener": [2, "obstacle_0_movable", [[3, 1], [8, 0]]],
                "kh1_key": {"obstacle_0_movable": _counts(valid_first=20)},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    k2_key = tmp_path / "k2_key.json"
    k2_key.write_text(json.dumps({str(post_xml.resolve()): [_sets(valid_first=20)]}), encoding="utf-8")

    out = tmp_path / "selected"
    summary = selector.select_profiles(
        sheets_path=sheets,
        k1_rows_path=k1_rows,
        k2_key_path=k2_key,
        profile="med2-med2",
        out_dir=out,
        limit=1,
    )

    selected_sheet = json.loads((out / "twohop_00000" / "build_sheet.json").read_text())
    assert summary["selected"] == 1
    assert selected_sheet["classification_status"] == "measured"
    assert [gate["tier_hmax2"] for gate in selected_sheet["measured_gates"]] == ["med", "med"]
    assert [gate["push_kind"] for gate in selected_sheet["measured_gates"]] == [
        "needs_2_chain",
        "needs_2_chain",
    ]
    assert selected_sheet["k1_canonical_opener"] == [
        2,
        "obstacle_0_movable",
        [[3, 1], [8, 0]],
    ]
    manifest = (out / "manifest.txt").read_text(encoding="utf-8").splitlines()
    assert manifest == [str((out / "twohop_00000" / "env.xml").resolve())]


def test_select_rejects_timed_out_or_incomplete_labels(tmp_path):
    k1 = selector.classify_episode(_counts(valid_first=20))
    k2 = selector.classify_episode(_sets(valid_first=20))

    k1["timed_out"] = True
    assert not selector.matches_profile(k1, k2, "med2-med2")
    assert selector.classify_episode(_counts(tried=0))["classification_complete"] is False


def test_select_accepts_terminal_k2_materialization_row(tmp_path):
    pool = tmp_path / "pool"
    scene_dir = pool / "twohop_00007"
    scene_dir.mkdir(parents=True)
    original_xml = scene_dir / "env.xml"
    _write_state_xml(original_xml, (0.10, 0.20, 0.0), (0.30, 0.40, 0.0))
    sheets = pool / "build_sheets.json"
    sheets.write_text(
        json.dumps(
            [
                {
                    "scene_id": "twohop_00007",
                    "requested_profile": "med2-med2",
                    "classification_status": "needs_revalidation",
                    "object_order": ["obj_1", "obj_4"],
                }
            ]
        ),
        encoding="utf-8",
    )

    round1_xml = tmp_path / "round1" / "xmls" / "twohop_00007.xml"
    round1_xml.parent.mkdir(parents=True)
    _write_state_xml(round1_xml, (0.16, 0.20, 5.0), (0.30, 0.40, 0.0))
    k1_rows = tmp_path / "round1" / "rows.jsonl"
    k1_rows.write_text(
        json.dumps(
            {
                "xml_path": str(original_xml),
                "out_xml": str(round1_xml),
                "status": "ok",
                "hop_count": 2,
                "post_hop_count": 1,
                "kh1_boundary_objects": ["obstacle_0_movable"],
                "next_boundary_objects": ["obstacle_1_movable"],
                "post_next_boundary_objects": ["obstacle_1_movable"],
                "next_boundary_matches": True,
                "kh1_timed_out": False,
                "canonical_opener": [2, "obstacle_0_movable", [[3, 1], [8, 0]]],
                "kh1_key": {"obstacle_0_movable": _counts(valid_first=20)},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    round2_xml = tmp_path / "round2" / "xmls" / "twohop_00007.xml"
    round2_xml.parent.mkdir(parents=True)
    _write_state_xml(round2_xml, (0.16, 0.20, 5.0), (0.24, 0.42, 8.0))
    k2_rows = tmp_path / "round2" / "rows.jsonl"
    k2_rows.write_text(
        json.dumps(
            {
                "xml_path": str(round1_xml),
                "out_xml": str(round2_xml),
                "status": "ok",
                "hop_count": 1,
                "post_hop_count": 0,
                "kh1_boundary_objects": ["obstacle_1_movable"],
                "next_boundary_objects": [],
                "post_next_boundary_objects": [],
                "next_boundary_matches": True,
                "kh1_timed_out": False,
                "canonical_opener": [2, "obstacle_1_movable", [[5, 2], [9, 1]]],
                "kh1_key": {"obstacle_1_movable": _counts(valid_first=20)},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    out = tmp_path / "selected"
    summary = selector.select_profiles(
        sheets_path=sheets,
        k1_rows_path=k1_rows,
        k2_rows_path=k2_rows,
        profile="med2-med2",
        out_dir=out,
        limit=1,
    )

    selected_sheet = json.loads((out / "twohop_00007" / "build_sheet.json").read_text())
    assert summary["selected"] == 1
    assert (out / "twohop_00007" / "post_k2_env.xml").is_file()
    assert selected_sheet["k2_canonical_opener"] == [
        2,
        "obstacle_1_movable",
        [[5, 2], [9, 1]],
    ]
    assert selected_sheet["post_k2_xml"] == "post_k2_env.xml"
    assert selected_sheet["mechanical_independence"]["status"] == "passed"


def test_mechanical_independence_detects_cross_gate_motion(tmp_path):
    initial = tmp_path / "initial.xml"
    post_k1 = tmp_path / "post_k1.xml"
    post_k2 = tmp_path / "post_k2.xml"
    _write_state_xml(initial, (0.10, 0.20, 0.0), (0.30, 0.40, 0.0))
    _write_state_xml(post_k1, (0.16, 0.20, 5.0), (0.305, 0.40, 0.0))
    _write_state_xml(post_k2, (0.16, 0.20, 5.0), (0.24, 0.42, 8.0))

    result = selector.mechanical_independence(initial, post_k1, post_k2)

    assert result["status"] == "failed"
    assert result["failure"] == "k1_moved_k2"
    assert result["k1_to_k2_position_delta_mm"] == 5.0


def test_mechanical_independence_checks_every_member_of_remote_gate(tmp_path):
    initial = tmp_path / "initial.xml"
    post_k1 = tmp_path / "post_k1.xml"
    post_k2 = tmp_path / "post_k2.xml"
    _write_state_xml(
        initial,
        (0.10, 0.20, 0.0),
        (0.30, 0.40, 0.0),
        (0.18, 0.25, 0.0),
    )
    _write_state_xml(
        post_k1,
        (0.16, 0.20, 5.0),
        (0.301, 0.40, 0.5),
        (0.22, 0.25, 12.0),
    )
    _write_state_xml(
        post_k2,
        (0.161, 0.20, 5.5),
        (0.24, 0.42, 8.0),
        (0.223, 0.25, 12.0),
    )

    result = selector.mechanical_independence(
        initial,
        post_k1,
        post_k2,
        gate_members=[
            ["obstacle_0_movable", "obstacle_2_movable"],
            ["obstacle_1_movable"],
        ],
    )

    assert result["status"] == "failed"
    assert result["failure"] == "k2_moved_k1_member"
    assert result["k2_to_k1"]["obstacle_2_movable"]["position_delta_mm"] == 3.0


def test_join_rejects_canonical_opener_for_the_wrong_object(tmp_path):
    pool = tmp_path / "pool"
    scene_dir = pool / "twohop_wrong_opener"
    scene_dir.mkdir(parents=True)
    original_xml = scene_dir / "env.xml"
    _write_state_xml(original_xml, (0.10, 0.20, 0.0), (0.30, 0.40, 0.0))
    sheets = pool / "build_sheets.json"
    sheets.write_text(
        json.dumps(
            [
                {
                    "scene_id": "twohop_wrong_opener",
                    "requested_profile": "hard1-hard1",
                    "gate_contract": {
                        "boundary_objects": [
                            ["obstacle_0_movable"],
                            ["obstacle_1_movable"],
                        ],
                        "gate_members": [
                            ["obstacle_0_movable", "obstacle_2_movable"],
                            ["obstacle_1_movable"],
                        ],
                    },
                }
            ]
        ),
        encoding="utf-8",
    )
    post_xml = tmp_path / "post.xml"
    post_xml.write_text("<mujoco/>", encoding="utf-8")
    k1_rows = tmp_path / "k1.jsonl"
    k1_rows.write_text(
        json.dumps(
            {
                "xml_path": str(original_xml),
                "out_xml": str(post_xml),
                "status": "ok",
                "hop_count": 2,
                "post_hop_count": 1,
                "kh1_boundary_objects": ["obstacle_0_movable"],
                "next_boundary_objects": ["obstacle_1_movable"],
                "post_next_boundary_objects": ["obstacle_1_movable"],
                "next_boundary_matches": True,
                "kh1_timed_out": False,
                "canonical_opener": [1, "obstacle_2_movable", [[3, 1]]],
                "kh1_key": {"obstacle_0_movable": _counts(valid=2)},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    k2_key = tmp_path / "k2.json"
    k2_key.write_text(
        json.dumps({str(post_xml.resolve()): [_sets(valid=2)]}),
        encoding="utf-8",
    )

    candidates, rejects = selector._joined_candidates(
        sheets,
        k1_rows,
        "hard1-hard1",
        k2_key_path=k2_key,
    )

    assert candidates == []
    assert rejects == {"opener_object_mismatch": 1}
