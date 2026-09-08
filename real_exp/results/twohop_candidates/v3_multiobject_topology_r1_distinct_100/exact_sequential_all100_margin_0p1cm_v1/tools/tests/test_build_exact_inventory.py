import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build_exact_inventory import InventoryError, build_inventory


FAKE_SELECTOR = '''
from selector_support import SENTINEL

def _valid_k1_row(row):
    return row.get("transition_valid") is True

def _valid_k2_row(row):
    return row.get("transition_valid") is True

def classify_episode(record):
    return dict(record)

def matches_profile(k1, k2, profile):
    if profile == "hard1-hard1":
        return all(g["tier_1push"] == "hard" and g["push_kind"] == "one_push" for g in (k1, k2))
    if profile == "med2-med2":
        return all(g["tier_hmax2"] == "med" and g["push_kind"] == "needs_2_chain" for g in (k1, k2))
    raise ValueError(profile)

def normalize_gate_contract(contract, gate_objects):
    return contract

def mechanical_independence(initial_xml, post_k1_xml, post_k2_xml, gate_members=None):
    if "must_not_measure" in str(initial_xml):
        raise AssertionError("mechanical independence called for an invalid transition")
    return {
        "status": "passed",
        "failure": None,
        "k1_to_k2_position_delta_mm": 0.1,
        "k1_to_k2_angle_delta_deg": 0.2,
        "k2_to_k1_position_delta_mm": 0.3,
        "k2_to_k1_angle_delta_deg": 0.4,
    }
'''


def _label(kind: str) -> dict:
    common = {
        "n_tried_1push": 100,
        "n_valid_1push": 0,
        "n_valid_first_push": 0,
        "n_open_within_2push": 0,
        "solve_rate_1push": 0.0,
        "solve_rate_hmax2": 0.0,
        "tier_1push": "unsolvable",
        "tier_hmax2": "unsolvable",
        "push_kind": "unsolvable",
        "timed_out": False,
        "classification_complete": True,
    }
    if kind == "hard":
        common.update(
            n_valid_1push=1,
            n_open_within_2push=1,
            solve_rate_1push=0.01,
            solve_rate_hmax2=0.01,
            tier_1push="hard",
            tier_hmax2="hard",
            push_kind="one_push",
        )
    elif kind == "med":
        common.update(
            n_valid_first_push=10,
            n_open_within_2push=10,
            solve_rate_hmax2=0.1,
            tier_hmax2="med",
            push_kind="needs_2_chain",
        )
    return common


class BuildExactInventoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.exact = self.root / "exact"
        self.selector_repo = self.root / "selector_repo"
        self.selector_repo.mkdir()
        (self.selector_repo / "selector.py").write_text(FAKE_SELECTOR, encoding="utf-8")
        (self.selector_repo / "selector_support.py").write_text("SENTINEL = True\n", encoding="utf-8")
        self.sheets: dict[str, Path] = {}

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _xml(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("<mujoco/>\n", encoding="utf-8")
        return path.resolve()

    def _write_profile(self, profile: str, scenes: list[tuple[str, str, bool]]) -> None:
        sheets_root = self.root / "sheets" / profile
        sheets = []
        k1_rows = []
        k2_rows = []
        for scene_id, label_kind, valid in scenes:
            source = self._xml(sheets_root / scene_id / "env.xml")
            post_k1 = self._xml(self.exact / profile / "artifacts" / scene_id / "post_k1.xml")
            post_k2 = self._xml(self.exact / profile / "artifacts" / scene_id / "post_k2.xml")
            sheets.append(
                {
                    "scene_id": scene_id,
                    "requested_profile": profile,
                    "goal_cm": [41.9, 60.8],
                    "robot_start_cm": [20.0, 10.0],
                    "robot_start_bearing_deg": 90.0,
                    "run_namo_goal_flag": "--goal 42 61",
                    "gate_contract": {
                        "boundary_objects": [["obstacle_0_movable"], ["obstacle_1_movable"]],
                        "gate_members": [["obstacle_0_movable"], ["obstacle_1_movable"]],
                    },
                }
            )
            k1_rows.append(
                {
                    "xml_path": str(source),
                    "out_xml": str(post_k1),
                    "status": "ok" if valid else "topology_mismatch",
                    "transition_valid": valid,
                    "kh1_key": {"obstacle_0_movable": _label(label_kind)},
                }
            )
            k2_rows.append(
                {
                    "xml_path": str(post_k1),
                    "out_xml": str(post_k2),
                    "status": "ok",
                    "transition_valid": valid,
                    "kh1_key": {"obstacle_1_movable": _label(label_kind)},
                }
            )
        sheets_root.mkdir(parents=True, exist_ok=True)
        sheets_path = sheets_root / "build_sheets.json"
        sheets_path.write_text(json.dumps(sheets), encoding="utf-8")
        self.sheets[profile] = sheets_path
        for stage, rows in (("k1", k1_rows), ("k2", k2_rows)):
            path = self.exact / profile / stage / "merged_rows.jsonl"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")

    def _write_navigation(self, rows: list[dict]) -> Path:
        path = self.root / "navigation_realstack.csv"
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "manifest_index",
                    "profile",
                    "scene_id",
                    "navigation_category",
                    "any_navigation_succeeded",
                    "ignore_status",
                    "ignore_reached",
                    "ignore_failure_cause",
                    "ignore_distance_to_goal_cm",
                    "penalise_status",
                    "penalise_reached",
                    "penalise_failure_cause",
                    "penalise_distance_to_goal_cm",
                ],
                extrasaction="ignore",
            )
            writer.writeheader()
            for index, row in enumerate(rows):
                writer.writerow({"manifest_index": index, **row})
        return path

    def _build(self, navigation: Path, expected_rows: int) -> tuple[list[dict], dict]:
        csv_path = self.root / "inventory.csv"
        json_path = self.root / "summary.json"
        summary = build_inventory(
            exact_root=self.exact,
            build_sheets=self.sheets,
            selector_repo=self.selector_repo,
            selector_module="selector.py",
            navigation_csv=navigation,
            output_csv=csv_path,
            output_json=json_path,
            expected_rows=expected_rows,
        )
        with csv_path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(json.loads(json_path.read_text(encoding="utf-8")), summary)
        return rows, summary

    def test_builds_joined_inventory_in_each_k1_file_order(self) -> None:
        self._write_profile(
            "hard1-hard1",
            [("twohop_multi_00002", "hard", True), ("twohop_multi_00001", "hard", True)],
        )
        self._write_profile("med2-med2", [("twohop_multi_00003", "med", True)])
        navigation = self._write_navigation(
            [
                {"profile": "med2-med2", "scene_id": "twohop_multi_00003", "navigation_category": "both_failed"},
                {"profile": "hard1-hard1", "scene_id": "twohop_multi_00001", "navigation_category": "ignore_only_succeeded"},
                {
                    "profile": "hard1-hard1",
                    "scene_id": "twohop_multi_00002",
                    "navigation_category": "both_succeeded",
                    "ignore_status": "success",
                    "ignore_reached": "true",
                    "ignore_distance_to_goal_cm": "0.7",
                },
            ]
        )

        rows, summary = self._build(navigation, expected_rows=3)

        self.assertEqual(
            [(row["discovery_profile"], row["scene_id"]) for row in rows],
            [
                ("hard1-hard1", "twohop_multi_00002"),
                ("hard1-hard1", "twohop_multi_00001"),
                ("med2-med2", "twohop_multi_00003"),
            ],
        )
        self.assertEqual([row["exact_joint_label"] for row in rows], ["hard1-hard1", "hard1-hard1", "med2-med2"])
        self.assertEqual(rows[0]["k1_n_tried_1push"], "100")
        self.assertEqual(rows[0]["k1_n_valid_1push"], "1")
        self.assertEqual(rows[2]["k2_n_valid_first_push"], "10")
        self.assertEqual(rows[2]["k2_solve_rate_hmax2"], "0.1")
        self.assertEqual(rows[0]["mechanical_independence_status"], "passed")
        self.assertEqual(rows[0]["navigation_category"], "both_succeeded")
        self.assertEqual(rows[0]["goal_cm"], "[41.9,60.8]")
        self.assertEqual(rows[0]["goal_x_cm"], "41.9")
        self.assertEqual(rows[0]["robot_start_cm"], "[20.0,10.0]")
        self.assertEqual(rows[0]["robot_start_bearing_deg"], "90.0")
        self.assertEqual(rows[0]["run_namo_goal_flag"], "--goal 42 61")
        self.assertEqual(rows[0]["nav_ignore_status"], "success")
        self.assertEqual(rows[0]["nav_ignore_distance_to_goal_cm"], "0.7")
        self.assertEqual(summary["rows"], 3)
        self.assertEqual(summary["exact_joint_label_counts"], {"hard1-hard1": 2, "med2-med2": 1})
        self.assertEqual(summary["navigation_category_counts"], {"both_failed": 1, "both_succeeded": 1, "ignore_only_succeeded": 1})
        self.assertEqual(
            summary["requested_exact_candidates"],
            {
                "hard1-hard1": [
                    {"discovery_profile": "hard1-hard1", "scene_id": "twohop_multi_00002"},
                    {"discovery_profile": "hard1-hard1", "scene_id": "twohop_multi_00001"},
                ],
                "med2-med2": [
                    {"discovery_profile": "med2-med2", "scene_id": "twohop_multi_00003"}
                ],
            },
        )
        self.assertEqual(
            summary["navigation_resistant_2push2push_candidates"],
            [
                {
                    "discovery_profile": "med2-med2",
                    "scene_id": "twohop_multi_00003",
                    "k1_tier": "med",
                    "k1_solve_rate": 0.1,
                    "k2_tier": "med",
                    "k2_solve_rate": 0.1,
                }
            ],
        )

    def test_marks_invalid_sequential_transition_incomplete_without_measuring_independence(self) -> None:
        self._write_profile("hard1-hard1", [("must_not_measure", "hard", False)])
        self._write_profile("med2-med2", [])
        navigation = self._write_navigation(
            [{"profile": "hard1-hard1", "scene_id": "must_not_measure", "navigation_category": "both_failed"}]
        )

        rows, summary = self._build(navigation, expected_rows=1)

        self.assertEqual(rows[0]["exact_joint_label"], "incomplete")
        self.assertEqual(rows[0]["k1_transition_status"], "invalid")
        self.assertEqual(rows[0]["sequential_transition_status"], "incomplete")
        self.assertEqual(rows[0]["mechanical_independence_status"], "not_computed")
        self.assertEqual(summary["exact_joint_label_counts"], {"incomplete": 1})

    def test_rejects_duplicate_k2_join_keys(self) -> None:
        self._write_profile("hard1-hard1", [("scene", "hard", True)])
        self._write_profile("med2-med2", [])
        k2_path = self.exact / "hard1-hard1" / "k2" / "merged_rows.jsonl"
        k2_path.write_text(k2_path.read_text(encoding="utf-8") * 2, encoding="utf-8")
        navigation = self._write_navigation(
            [{"profile": "hard1-hard1", "scene_id": "scene", "navigation_category": "both_failed"}]
        )

        with self.assertRaisesRegex(InventoryError, "duplicate K2"):
            self._build(navigation, expected_rows=1)

    def test_rejects_duplicate_navigation_join_keys(self) -> None:
        self._write_profile("hard1-hard1", [("scene", "hard", True)])
        self._write_profile("med2-med2", [])
        row = {"profile": "hard1-hard1", "scene_id": "scene", "navigation_category": "both_failed"}
        navigation = self._write_navigation([row, row])

        with self.assertRaisesRegex(InventoryError, "duplicate navigation"):
            self._build(navigation, expected_rows=1)

    def test_rejects_duplicate_build_sheet_source_joins(self) -> None:
        self._write_profile("hard1-hard1", [("scene", "hard", True)])
        self._write_profile("med2-med2", [])
        sheets_path = self.sheets["hard1-hard1"]
        sheets = json.loads(sheets_path.read_text(encoding="utf-8"))
        sheets_path.write_text(json.dumps(sheets * 2), encoding="utf-8")
        navigation = self._write_navigation(
            [{"profile": "hard1-hard1", "scene_id": "scene", "navigation_category": "both_failed"}]
        )

        with self.assertRaisesRegex(InventoryError, "duplicate build-sheet"):
            self._build(navigation, expected_rows=1)

    def test_allows_the_build_sheet_to_be_a_superset_of_promoted_k1_rows(self) -> None:
        self._write_profile("hard1-hard1", [("selected", "hard", True)])
        self._write_profile("med2-med2", [])
        sheets_path = self.sheets["hard1-hard1"]
        sheets = json.loads(sheets_path.read_text(encoding="utf-8"))
        self._xml(sheets_path.parent / "not_promoted" / "env.xml")
        sheets.append(
            {
                "scene_id": "not_promoted",
                "requested_profile": "hard1-hard1",
                "gate_contract": {
                    "boundary_objects": [["obstacle_0_movable"], ["obstacle_1_movable"]],
                    "gate_members": [["obstacle_0_movable"], ["obstacle_1_movable"]],
                },
            }
        )
        sheets_path.write_text(json.dumps(sheets), encoding="utf-8")
        navigation = self._write_navigation(
            [{"profile": "hard1-hard1", "scene_id": "selected", "navigation_category": "both_failed"}]
        )

        rows, summary = self._build(navigation, expected_rows=1)

        self.assertEqual([row["scene_id"] for row in rows], ["selected"])
        self.assertEqual(summary["build_sheet_rows"], {"hard1-hard1": 2, "med2-med2": 0})


if __name__ == "__main__":
    unittest.main()
