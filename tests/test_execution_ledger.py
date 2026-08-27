"""The execution ledger is deterministic, complete, and balanced.

The ledger is a pre-registration artifact: 14 scenes x 4 cells whose
within-scene order is frozen by a fixed seed before any row is collected.
These tests pin the properties that make it one. Determinism, because a
ledger that changes between runs is not frozen. Completeness, because a
missing cell breaks the pairing for both sign tests at once. And the check
mode, because an edited ledger must fail loudly rather than read as frozen.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from make_execution_ledger import (  # noqa: E402
    CELLS,
    HEADER,
    LEDGER_PATH,
    MATRIX_SCENES,
    build_rows,
    main,
)

N_SCENES = 14
N_CELLS_PER_SCENE = 4


def test_same_seed_same_ledger():
    assert build_rows() == build_rows()


def test_every_scene_has_every_cell_exactly_once():
    rows = build_rows()
    assert len(rows) == N_SCENES * N_CELLS_PER_SCENE
    for axis, build_id in MATRIX_SCENES:
        scene_rows = [r for r in rows if r[0] == axis and r[1] == build_id]
        assert len(scene_rows) == N_CELLS_PER_SCENE
        assert sorted(r[2] for r in scene_rows) == ["1", "2", "3", "4"]
        assert sorted((r[3], r[4]) for r in scene_rows) == sorted(CELLS)


def test_the_order_is_actually_randomized():
    """At least one scene must differ from the unshuffled cell order.

    A broken shuffle that left every scene in declaration order would pass
    the completeness test and defeat the point of the ledger.
    """
    rows = build_rows()
    declared = [list(c) for c in CELLS]
    per_scene = [
        [[r[3], r[4]] for r in rows if r[0] == axis and r[1] == build_id]
        for axis, build_id in MATRIX_SCENES
    ]
    assert any(order != declared for order in per_scene)


def test_check_mode_accepts_the_generated_file(tmp_path, monkeypatch):
    import make_execution_ledger as m

    monkeypatch.setattr(m, "LEDGER_PATH", tmp_path / "ledger.csv")
    assert m.main([]) == 0
    assert m.main(["--check"]) == 0


def test_check_mode_refuses_an_edited_ledger(tmp_path, monkeypatch, capsys):
    import make_execution_ledger as m

    monkeypatch.setattr(m, "LEDGER_PATH", tmp_path / "ledger.csv")
    assert m.main([]) == 0
    rows = list(csv.reader((tmp_path / "ledger.csv").open()))
    rows[1], rows[2] = rows[2], rows[1]  # somebody "just swapped two cells"
    with (tmp_path / "ledger.csv").open("w", newline="") as fh:
        csv.writer(fh).writerows(rows)
    assert m.main(["--check"]) == 1
    assert "DOES NOT MATCH" in capsys.readouterr().out


def test_check_mode_refuses_a_missing_ledger(tmp_path, monkeypatch):
    import make_execution_ledger as m

    monkeypatch.setattr(m, "LEDGER_PATH", tmp_path / "never_written.csv")
    assert m.main(["--check"]) == 1
