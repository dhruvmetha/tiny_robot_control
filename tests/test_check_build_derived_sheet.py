"""check_build must read a v3_b2 pool's build_sheet_derived.json.

That pool ships env.xml only, so the sim side derives a sheet with
schema "derived_from_env_xml" and, in the v3-b2-all bundle, no "scene"
key. The scene directory name carries the same identity, so the parser
falls back to it. Before this, every v3_b2 scene died at the build_id
check and the operator had no placement rows at all.
"""

from importlib import import_module
from pathlib import Path
import json
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
check_build = import_module("check_build")

SHEET = (Path(__file__).resolve().parents[1] / "real_exp" / "environments" /
         "real_2mov" / "v3_b2" / "hard_hc2" / "rb_00001" / "build_sheet_derived.json")


def _sheet_copy(tmp_path: Path, dirname: str, **extra) -> Path:
    sheet = json.loads(SHEET.read_text())
    sheet.update(extra)
    out = tmp_path / dirname / "build_sheet_derived.json"
    out.parent.mkdir()
    out.write_text(json.dumps(sheet))
    return out


def test_derived_sheet_without_scene_key_uses_directory_name():
    assert "scene" not in json.loads(SHEET.read_text())
    rows = check_build.load_sheet_rows(SHEET, "rb_00001")

    assert rows
    assert all(r["build_id"] == "rb_00001" for r in rows)
    assert {r["item"] for r in rows} <= {"brick", "block"}
    assert all(r["push_kind"] == "needs_2_chain" for r in rows)


def test_wrong_build_id_exits_loudly():
    with pytest.raises(SystemExit):
        check_build.load_sheet_rows(SHEET, "rb_00999")


def test_explicit_scene_key_wins_over_directory_name(tmp_path):
    # Scene key matches, directory does not: loads.
    good = _sheet_copy(tmp_path, "unrelated_dir", scene="pool/rb_00001")
    assert check_build.load_sheet_rows(good, "rb_00001")
    # Directory matches, scene key does not: still exits.
    bad = _sheet_copy(tmp_path, "rb_00001", scene="pool/rb_00777")
    with pytest.raises(SystemExit):
        check_build.load_sheet_rows(bad, "rb_00001")
