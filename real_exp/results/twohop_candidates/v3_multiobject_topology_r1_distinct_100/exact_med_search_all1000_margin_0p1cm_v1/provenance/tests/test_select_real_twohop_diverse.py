from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "pipeline"))

import select_real_twohop_diverse as selector  # noqa: E402


THRESHOLDS = {
    "auxiliary_position_m": 0.05,
    "auxiliary_yaw_deg": 20.0,
    "wall_position_m": 0.05,
    "wall_yaw_deg": 15.0,
}


def _sheet(
    scene_id="scene_a",
    family="hard_loose",
    source_scene="rb_00001",
    single="hard_001",
    helper=(18.0, 25.0, 10.0),
):
    def movable(sim_id, center, yaw, shape="obj_4", dimensions=(12.0, 7.5, 5.0)):
        return {
            "sim_id": sim_id,
            "center_cm": list(center),
            "yaw_deg": yaw,
            "source_shape": shape,
            "long_cm": dimensions[0],
            "short_cm": dimensions[1],
            "height_cm": dimensions[2],
        }

    def wall(sim_id, center, yaw):
        return {
            "sim_id": sim_id,
            "center_cm": list(center),
            "yaw_deg": yaw,
            "long_cm": 19.5,
            "short_cm": 5.5,
            "height_cm": 10.0,
        }

    return {
        "scene_id": scene_id,
        "multi_gate_index": 0,
        "source_multiobject": {
            "source_family": family,
            "source_scene_id": source_scene,
        },
        "source_singleobject": {"build_id": single},
        "gate_contract": {
            "boundary_objects": [["obstacle_0_movable"], ["obstacle_1_movable"]],
            "gate_members": [
                ["obstacle_0_movable", "obstacle_2_movable"],
                ["obstacle_1_movable"],
            ],
        },
        "movables": [
            movable("obstacle_0_movable", (12.0, 24.0), 30.0),
            movable("obstacle_1_movable", (30.0, 52.0), 75.0),
            movable(
                "obstacle_2_movable",
                helper[:2],
                helper[2],
                shape="obj_1",
                dimensions=(15.0, 7.0, 4.0),
            ),
        ],
        "gate_walls": [
            wall("wall_inner_1", (8.0, 20.0), 5.0),
            wall("wall_inner_2", (25.0, 28.0), 80.0),
            wall("wall_inner_3", (10.0, 50.0), 15.0),
            wall("wall_inner_4", (35.0, 55.0), 100.0),
        ],
        "robot_start_cm": [7.0, 8.0],
        "middle_anchor_cm": [24.0, 38.0],
        "goal_cm": [41.0, 67.0],
        "exact_geometry_id": f"exact_{scene_id}",
        "normalized_layout_id": f"normalized_{scene_id}",
    }


def _translate_sheet(sheet, dx, dy):
    moved = copy.deepcopy(sheet)
    for row in moved["movables"] + moved["gate_walls"]:
        row["center_cm"][0] += dx
        row["center_cm"][1] += dy
    for key in ("robot_start_cm", "middle_anchor_cm", "goal_cm"):
        moved[key][0] += dx
        moved[key][1] += dy
    return moved


def _mirror_sheet(sheet):
    mirrored = copy.deepcopy(sheet)
    for row in mirrored["movables"] + mirrored["gate_walls"]:
        row["center_cm"][0] = 49.0 - row["center_cm"][0]
        row["yaw_deg"] = (180.0 - row["yaw_deg"]) % 180.0
    for key in ("robot_start_cm", "middle_anchor_cm", "goal_cm"):
        mirrored[key][0] = 49.0 - mirrored[key][0]
    return mirrored


def test_layout_features_ignore_global_translation_and_group_mirrors():
    sheet = _sheet()
    translated = _translate_sheet(sheet, 1.2, -1.8)
    mirrored = _mirror_sheet(sheet)

    assert selector.layout_features(sheet) == selector.layout_features(translated)
    assert selector.mirror_family_id(sheet) == selector.mirror_family_id(mirrored)
    assert selector.is_meaningfully_distinct(sheet, mirrored, THRESHOLDS) is False


def test_meaningful_difference_rejects_small_wall_or_helper_perturbations():
    sheet = _sheet()
    small = _sheet(scene_id="small", source_scene="rb_00002", helper=(20.0, 25.0, 10.0))
    small["gate_walls"][0]["center_cm"][0] += 2.0
    large = _sheet(scene_id="large", source_scene="rb_00003", helper=(24.0, 25.0, 10.0))

    assert selector.is_meaningfully_distinct(sheet, small, THRESHOLDS) is False
    assert selector.is_meaningfully_distinct(sheet, large, THRESHOLDS) is True


def test_donor_ids_do_not_make_identical_geometry_distinct():
    sheet = _sheet()
    renamed = _sheet(
        scene_id="renamed",
        family="med",
        source_scene="rb_other",
        single="other_single",
    )

    assert selector.is_meaningfully_distinct(sheet, renamed, THRESHOLDS) is False
    assert selector.mirror_family_id(sheet) == selector.mirror_family_id(renamed)
    assert selector.difference_components(sheet, renamed)["wall_position_cm"] == 0.0


def test_exact_threshold_perturbations_remain_near_duplicates():
    sheet = _sheet()
    boundary = copy.deepcopy(sheet)
    boundary["source_multiobject"]["source_scene_id"] = "rb_other"
    boundary["movables"][2]["center_cm"][0] += 5.0
    boundary["movables"][2]["yaw_deg"] += 20.0
    boundary["gate_walls"][0]["center_cm"][1] += 5.0
    boundary["gate_walls"][1]["yaw_deg"] += 15.0

    assert selector.is_meaningfully_distinct(sheet, boundary, THRESHOLDS) is False


def test_material_single_gate_wall_change_counts_as_distinct():
    sheet = _sheet()
    changed = copy.deepcopy(sheet)
    changed["source_singleobject"]["build_id"] = "other_single"
    changed["gate_walls"][2]["center_cm"][0] += 5.1

    assert selector.is_meaningfully_distinct(sheet, changed, THRESHOLDS) is True


def test_wall_assignment_uses_best_permutation_within_each_gate():
    sheet = _sheet()
    reordered = copy.deepcopy(sheet)
    reordered["gate_walls"] = [
        reordered["gate_walls"][1],
        reordered["gate_walls"][0],
        reordered["gate_walls"][3],
        reordered["gate_walls"][2],
    ]

    assert selector.is_meaningfully_distinct(sheet, reordered, THRESHOLDS) is False
    assert selector.mirror_family_id(sheet) == selector.mirror_family_id(reordered)


def test_dimensions_affect_distinctness_only_beyond_tolerance():
    sheet = _sheet()
    near = copy.deepcopy(sheet)
    near["movables"][1]["long_cm"] += 0.5
    material_dimension = copy.deepcopy(sheet)
    material_dimension["movables"][1]["long_cm"] += 0.51

    assert selector.is_meaningfully_distinct(sheet, near, THRESHOLDS) is False
    assert selector.is_meaningfully_distinct(sheet, material_dimension, THRESHOLDS) is True


def test_task_anchors_and_gate_separation_do_not_establish_distinctness():
    sheet = _sheet()
    changed = copy.deepcopy(sheet)
    changed["robot_start_cm"] = [45.0, 2.0]
    changed["middle_anchor_cm"] = [2.0, 40.0]
    changed["goal_cm"] = [45.0, 68.0]
    changed["movables"][1]["center_cm"][1] += 8.0
    for wall in changed["gate_walls"][2:]:
        wall["center_cm"][1] += 8.0

    assert selector.is_meaningfully_distinct(sheet, changed, THRESHOLDS) is False
    assert selector.mirror_family_id(sheet) == selector.mirror_family_id(changed)


def test_material_topology_change_counts_as_distinct():
    sheet = _sheet()
    changed = copy.deepcopy(sheet)
    changed["movables"][1]["source_shape"] = "obj_1"

    assert selector.is_meaningfully_distinct(sheet, changed, THRESHOLDS) is True


def test_write_selection_excludes_prior_env_directory(tmp_path):
    prior_dir = tmp_path / "prior"
    prior_dir.mkdir()
    (prior_dir / "env.xml").write_text("<mujoco />\n", encoding="utf-8")
    (prior_dir / "build_sheet.json").write_text(
        json.dumps(_sheet("prior")),
        encoding="utf-8",
    )

    candidate_dir = tmp_path / "candidate"
    candidate_dir.mkdir()
    candidate_sheet = _sheet(
        "candidate",
        family="med",
        source_scene="rb_other",
        single="other_single",
    )
    candidate_xml = candidate_dir / "env.xml"
    candidate_xml.write_text("<mujoco />\n", encoding="utf-8")
    (candidate_dir / "build_sheet.json").write_text(
        json.dumps(candidate_sheet),
        encoding="utf-8",
    )
    records = tmp_path / "records"
    records.mkdir()
    (records / "candidate.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "scene_id": "candidate",
                "simulator_calls": 12,
                "xml_path": str(candidate_xml),
            }
        ),
        encoding="utf-8",
    )

    summary = selector.write_selection(
        tmp_path / "selection",
        records_root=records,
        limit=3,
        thresholds=THRESHOLDS,
        exclusions=[prior_dir],
    )

    assert summary["eligible"] == 0
    assert summary["selected"] == 0
    assert summary["prior_exclusions"] == 1
    assert summary["prior_incompatible_topology"] == 0
    assert summary["effective_thresholds"] == {
        **THRESHOLDS,
        "dimension_tolerance_cm": 0.5,
    }
    assert summary["distinctness_policy"] == {
        "categorical_topology_changes_are_distinct": True,
        "donor_ids": "soft_coverage_only",
        "gate_separation": "recorded_not_scored",
        "geometry_scope": "local_gate_geometry_only",
        "reflection": "one_global_left_right",
        "task_anchors": "recorded_not_scored",
        "wall_assignment": "best_permutation_within_gate",
    }
    assert json.loads((tmp_path / "selection" / "rejected.json").read_text()) == {
        "prior_near_duplicate": 1
    }
    assert json.loads((tmp_path / "selection" / "selection.json").read_text()) == summary


def test_load_excluded_sheets_accepts_repeatable_manifests_and_sheet_paths(tmp_path):
    first = tmp_path / "first"
    first.mkdir()
    first_sheet = first / "build_sheet.json"
    first_sheet.write_text(json.dumps(_sheet("first")), encoding="utf-8")
    manifest_root = tmp_path / "manifests"
    manifest_root.mkdir()
    relative_scene = manifest_root / "second"
    relative_scene.mkdir()
    relative_sheet = relative_scene / "build_sheet.json"
    relative_sheet.write_text(json.dumps(_sheet("second")), encoding="utf-8")
    manifest = manifest_root / "prior.txt"
    manifest.write_text("second/env.xml\n", encoding="utf-8")

    excluded, incompatible = selector.load_excluded_sheets([first_sheet, manifest])

    assert [sheet["scene_id"] for sheet in excluded] == ["first", "second"]
    assert incompatible == 0


def test_legacy_two_object_prior_is_reported_and_does_not_block_selection(tmp_path):
    legacy = tmp_path / "legacy"
    legacy.mkdir()
    (legacy / "build_sheet.json").write_text(
        json.dumps(
            {
                "scene_id": "legacy_two_object",
                "generator": "gen_real_buildable_twohop.py",
                "object_order": ["obj_1", "obj_4"],
                "gates": [{"index": 1}, {"index": 2}],
            }
        ),
        encoding="utf-8",
    )
    candidate_dir = tmp_path / "candidate"
    candidate_dir.mkdir()
    candidate_xml = candidate_dir / "env.xml"
    candidate_xml.write_text("<mujoco />\n", encoding="utf-8")
    (candidate_dir / "build_sheet.json").write_text(
        json.dumps(_sheet("candidate")),
        encoding="utf-8",
    )
    records = tmp_path / "records"
    records.mkdir()
    (records / "candidate.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "scene_id": "candidate",
                "simulator_calls": 12,
                "xml_path": str(candidate_xml),
            }
        ),
        encoding="utf-8",
    )

    summary = selector.write_selection(
        tmp_path / "selection",
        records_root=records,
        limit=1,
        thresholds=THRESHOLDS,
        exclusions=[legacy],
    )

    assert summary["eligible"] == 1
    assert summary["selected"] == 1
    assert summary["prior_exclusions"] == 1
    assert summary["prior_incompatible_topology"] == 1


def test_candidate_loading_maps_compute_scache_paths_to_shared_scratch(
    tmp_path,
    monkeypatch,
):
    shared_scratch = tmp_path / "scratch"
    candidate_dir = shared_scratch / "run" / "candidate"
    candidate_dir.mkdir(parents=True)
    candidate_xml = candidate_dir / "env.xml"
    candidate_xml.write_text("<mujoco />\n", encoding="utf-8")
    (candidate_dir / "build_sheet.json").write_text(
        json.dumps(_sheet("candidate")),
        encoding="utf-8",
    )
    records = tmp_path / "records"
    records.mkdir()
    (records / "candidate.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "scene_id": "candidate",
                "simulator_calls": 12,
                "xml_path": "/scache/scratch/run/candidate/env.xml",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(selector, "LOGIN_SCRATCH_ROOT", shared_scratch)

    rows, rejected = selector.load_passing_candidates(records)

    assert rejected == {}
    assert rows[0]["xml_path"] == str(candidate_xml.resolve())


def test_farthest_first_covers_families_and_donor_pairs_before_repeats():
    rows = [
        {
            "scene_id": "a",
            "simulator_calls": 1,
            "sheet": _sheet("a", "hard_loose", "rb_a", "single_a", (18.0, 25.0, 10.0)),
        },
        {
            "scene_id": "b",
            "simulator_calls": 9,
            "sheet": _sheet("b", "hard_zig", "rb_b", "single_b", (25.0, 25.0, 40.0)),
        },
        {
            "scene_id": "c",
            "simulator_calls": 5,
            "sheet": _sheet("c", "med", "rb_c", "single_c", (33.0, 25.0, 75.0)),
        },
        {
            "scene_id": "d",
            "simulator_calls": 2,
            "sheet": _sheet("d", "hard_loose", "rb_d", "single_d", (40.0, 25.0, 110.0)),
        },
    ]

    chosen = selector.select_diverse(rows, limit=4, thresholds=THRESHOLDS)

    assert chosen[0]["scene_id"] == "a"
    assert len({selector.layout_features(row["sheet"])["source_family"] for row in chosen[:3]}) == 3
    assert len({selector.donor_pair_id(row["sheet"]) for row in chosen}) == 4
