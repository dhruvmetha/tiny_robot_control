import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reduce_exact_materialization import ReductionError, reduce_stage


def _valid_row(xml_path: Path, out_xml: Path, stage: str, profile: str) -> dict:
    is_k1 = stage == "k1"
    protected = {
        ("hard1-hard1", "k1"): ["obstacle_1_movable", "obstacle_2_movable"],
        ("hard1-hard1", "k2"): ["obstacle_0_movable"],
        ("med2-med2", "k1"): ["obstacle_1_movable"],
        ("med2-med2", "k2"): ["obstacle_0_movable", "obstacle_2_movable"],
    }[(profile, stage)]
    return {
        "xml_path": str(xml_path.resolve()),
        "status": "ok",
        "hop_count": 2 if is_k1 else 1,
        "post_hop_count": 1 if is_k1 else 0,
        "kh1_boundary_objects": ["obstacle_0_movable"] if is_k1 else ["obstacle_1_movable"],
        "next_boundary_objects": ["obstacle_1_movable"] if is_k1 else [],
        "post_next_boundary_objects": ["obstacle_1_movable"] if is_k1 else [],
        "next_boundary_matches": True,
        "kh1_timed_out": False,
        "canonical_opener": [1, "obstacle_0_movable" if is_k1 else "obstacle_1_movable", [[2, 3]]],
        "canonical_is_lex_min": True,
        "post_goal_in_free_space": True,
        "protected_objects": protected,
        "next_boundary_protection": {"status": "passed"},
        "protected_object_motion": {"status": "passed"},
        "missing_bodies": [],
        "max_dxy_mm": 0.02,
        "max_dtheta_deg": 0.01,
        "robot_dxy_mm": 0.0,
        "robot_dtheta_deg": 0.0,
        "out_xml": str(out_xml.resolve()),
    }


class ReduceExactMaterializationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def _write_manifest(self, count: int) -> tuple[Path, list[Path]]:
        inputs = []
        for index in range(count):
            path = self.root / f"input_{index}.xml"
            path.write_text("<mujoco/>\n", encoding="utf-8")
            inputs.append(path)
        manifest = self.root / "manifest.txt"
        manifest.write_text("".join(f"{path.resolve()}\n" for path in inputs), encoding="utf-8")
        return manifest, inputs

    def _write_row(self, stage_root: Path, index: int, row: dict) -> None:
        shard = stage_root / "shards" / f"task_{index:04d}"
        shard.mkdir(parents=True, exist_ok=True)
        (shard / "rows.jsonl").write_text(json.dumps(row) + "\n", encoding="utf-8")

    def test_k1_merges_every_row_and_manifests_only_valid_transitions(self) -> None:
        manifest, inputs = self._write_manifest(2)
        stage_root = self.root / "k1"
        output = stage_root / "shards" / "task_0000" / "xmls" / "post.xml"
        output.parent.mkdir(parents=True)
        output.write_text("<mujoco/>\n", encoding="utf-8")
        valid = _valid_row(inputs[0], output, "k1", "hard1-hard1")
        # Earlier lexicographic openers may fail to reduce the hop count; the
        # materializer then correctly chooses the first later opener that does.
        valid["canonical_is_lex_min"] = False
        self._write_row(stage_root, 0, valid)
        nodecrement = stage_root / "shards" / "task_0001" / "xmls_nodecrement" / "post.xml"
        nodecrement.parent.mkdir(parents=True)
        nodecrement.write_text("<mujoco/>\n", encoding="utf-8")
        rejected = _valid_row(inputs[1], output, "k1", "hard1-hard1")
        rejected.update(
            status="no_opener_decrements_hop",
            out_xml=None,
            nodecrement_out_xml=str(nodecrement.resolve()),
        )
        self._write_row(stage_root, 1, rejected)

        summary = reduce_stage(manifest, stage_root, "hard1-hard1", "k1")

        self.assertEqual(summary["manifest_rows"], 2)
        self.assertEqual(summary["validated_post_k1"], 1)
        self.assertEqual(summary["status_counts"], {"no_opener_decrements_hop": 1, "ok": 1})
        self.assertEqual(summary["nodecrement_xml_artifacts"], 1)
        self.assertEqual((stage_root / "post_k1_manifest.txt").read_text(), f"{output.resolve()}\n")
        self.assertEqual(len((stage_root / "merged_rows.jsonl").read_text().splitlines()), 2)

    def test_k2_accepts_only_a_measured_terminal_second_gate_opening(self) -> None:
        manifest, inputs = self._write_manifest(1)
        stage_root = self.root / "k2"
        output = stage_root / "shards" / "task_0000" / "xmls" / "post.xml"
        output.parent.mkdir(parents=True)
        output.write_text("<mujoco/>\n", encoding="utf-8")
        self._write_row(stage_root, 0, _valid_row(inputs[0], output, "k2", "med2-med2"))

        summary = reduce_stage(manifest, stage_root, "med2-med2", "k2")

        self.assertEqual(summary["validated_post_k2"], 1)
        self.assertFalse((stage_root / "post_k1_manifest.txt").exists())

    def test_rejects_a_row_from_the_wrong_manifest_index(self) -> None:
        manifest, inputs = self._write_manifest(2)
        stage_root = self.root / "k1"
        output = self.root / "post.xml"
        output.write_text("<mujoco/>\n", encoding="utf-8")
        self._write_row(stage_root, 0, _valid_row(inputs[1], output, "k1", "hard1-hard1"))
        self._write_row(stage_root, 1, _valid_row(inputs[0], output, "k1", "hard1-hard1"))

        with self.assertRaisesRegex(ReductionError, "manifest index"):
            reduce_stage(manifest, stage_root, "hard1-hard1", "k1")

    def test_rejects_an_orphan_nodecrement_xml(self) -> None:
        manifest, inputs = self._write_manifest(1)
        stage_root = self.root / "k1"
        output = stage_root / "shards" / "task_0000" / "xmls" / "post.xml"
        output.parent.mkdir(parents=True)
        output.write_text("<mujoco/>\n", encoding="utf-8")
        orphan = stage_root / "shards" / "task_0000" / "xmls_nodecrement" / "orphan.xml"
        orphan.parent.mkdir()
        orphan.write_text("<mujoco/>\n", encoding="utf-8")
        self._write_row(stage_root, 0, _valid_row(inputs[0], output, "k1", "hard1-hard1"))

        with self.assertRaisesRegex(ReductionError, "nodecrement"):
            reduce_stage(manifest, stage_root, "hard1-hard1", "k1")

    def test_rejects_status_ok_with_the_wrong_protection_contract(self) -> None:
        manifest, inputs = self._write_manifest(1)
        stage_root = self.root / "k1"
        output = stage_root / "shards" / "task_0000" / "xmls" / "post.xml"
        output.parent.mkdir(parents=True)
        output.write_text("<mujoco/>\n", encoding="utf-8")
        row = _valid_row(inputs[0], output, "k1", "hard1-hard1")
        row["protected_objects"] = ["obstacle_1_movable"]
        self._write_row(stage_root, 0, row)

        with self.assertRaisesRegex(ReductionError, "wrong_protected_object_contract"):
            reduce_stage(manifest, stage_root, "hard1-hard1", "k1")

    def test_keeps_but_does_not_manifest_a_well_formed_topology_mismatch(self) -> None:
        manifest, inputs = self._write_manifest(1)
        stage_root = self.root / "k1"
        output = stage_root / "shards" / "task_0000" / "xmls" / "post.xml"
        output.parent.mkdir(parents=True)
        output.write_text("<mujoco/>\n", encoding="utf-8")
        row = _valid_row(inputs[0], output, "k1", "med2-med2")
        row["kh1_boundary_objects"] = ["obstacle_0_movable", "obstacle_2_movable"]
        self._write_row(stage_root, 0, row)

        summary = reduce_stage(manifest, stage_root, "med2-med2", "k1")

        self.assertEqual(summary["status_counts"], {"ok": 1})
        self.assertEqual(summary["validated_post_k1"], 0)
        self.assertEqual(summary["semantic_reject_counts"], {"wrong_opened_boundary": 1})
        self.assertEqual((stage_root / "post_k1_manifest.txt").read_text(), "")


if __name__ == "__main__":
    unittest.main()
