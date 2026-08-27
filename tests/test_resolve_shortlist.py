"""The shortlist resolver joins on xml only and refuses everything else.

Three id namespaces exist for overlapping scene sets (v1 shipped-600, the v2
sheets, the gallery's full-pool ids), so a bare id can name different
geometry depending on where it came from. The resolver must never guess: an
entry resolves through its xml path or it is reported unresolvable, and a
partly-bad shortlist must exit nonzero so it cannot read as clean.

Uses the real tracked sheets, which is deliberate: the join these tests pin
is the join the table will use.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from resolve_shortlist import MARKER_SHEET, main  # noqa: E402

CLEAN = 0
VIOLATION = 1


def _xml_of(build_id: str, axis: str) -> str:
    with MARKER_SHEET.open(newline="") as fh:
        for row in csv.DictReader(fh):
            if row["build_id"] == build_id and row["axis"] == axis:
                return row["xml"].strip()
    raise AssertionError(f"{axis}/{build_id} not in the marker sheet")


def _run(entries, tmp_path, capsys):
    path = tmp_path / "shortlist.json"
    path.write_text(json.dumps(entries))
    code = main(["resolve_shortlist.py", str(path)])
    return code, capsys.readouterr().out


def test_a_real_xml_resolves_to_its_v2_scene(tmp_path, capsys):
    code, out = _run(
        [{"xml": _xml_of("hard_037", "1push"), "gallery_id": "hard_612"}],
        tmp_path, capsys,
    )
    assert code == CLEAN
    assert "1push/hard_037" in out
    assert "hard_612" in out  # the gallery name stays visible for cross-checking


def test_an_unknown_xml_is_unresolved_and_fails_the_run(tmp_path, capsys):
    code, out = _run([{"xml": "/nope/fake.xml"}], tmp_path, capsys)
    assert code == VIOLATION
    assert "UNRESOLVED" in out


def test_a_bare_id_is_never_guessed(tmp_path, capsys):
    """easy_002 exists in the v2 sheets, and the resolver must still refuse.

    Resolving a bare id by name is exactly the cross-namespace mistake the
    xml join exists to prevent: the shipped-600 gallery's easy_002 is not
    the v2 sheets' easy_002.
    """
    code, out = _run(
        [{"gallery_id": "easy_002", "dataset": "shipped-600"}], tmp_path, capsys
    )
    assert code == VIOLATION
    assert "UNRESOLVED" in out
    assert "namespaces" in out


def test_one_bad_entry_fails_the_whole_run(tmp_path, capsys):
    """A shortlist that half-resolves must not exit clean."""
    code, out = _run(
        [
            {"xml": _xml_of("med_069", "hmax2")},
            {"gallery_id": "ghost"},
        ],
        tmp_path, capsys,
    )
    assert code == VIOLATION
    assert "hmax2/med_069" in out
    assert "1 of 2 resolved" in out


def test_whitespace_around_the_xml_still_joins(tmp_path, capsys):
    """A pasted path with a stray newline or CR must not miss the join.

    The sheets themselves ship with CRLF endings, so the raw last column
    carries a \\r when read naively; the resolver strips both sides.
    """
    xml = _xml_of("hard_037", "1push")
    code, out = _run([{"xml": f"  {xml}\r\n"}], tmp_path, capsys)
    assert code == CLEAN
    assert "1push/hard_037" in out


def test_a_car_envs_entry_is_refused_for_its_dataset(tmp_path, capsys):
    """car-envs legitimately has no xml; the message must blame the dataset.

    "missing field" would send someone hunting an export bug that is not
    there. Sim-only scenes are not buildable, full stop.
    """
    code, out = _run(
        [{"gallery_id": "pool3_017", "dataset": "car-envs"}], tmp_path, capsys
    )
    assert code == VIOLATION
    assert "sim-only" in out
    assert "xml" not in out.split("pool3_017")[1].split("\n")[0]


def test_a_stale_star_is_told_to_reload(tmp_path, capsys):
    """A real-table entry without xml means a copy from an unreloaded page."""
    code, out = _run(
        [{"gallery_id": "hard_612", "dataset": "full-exhaustive-pool"}],
        tmp_path, capsys,
    )
    assert code == VIOLATION
    assert "reload" in out


@pytest.mark.parametrize("payload", ["not json at all", '{"an": "object"}', "[]"])
def test_malformed_input_fails_loudly(payload, tmp_path, capsys):
    path = tmp_path / "shortlist.json"
    path.write_text(payload)
    assert main(["resolve_shortlist.py", str(path)]) == VIOLATION
