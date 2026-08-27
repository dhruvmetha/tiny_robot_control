#!/usr/bin/env python3
"""Resolve a gallery shortlist into v2 sheet scenes by xml path and horizon.

The scene gallery exports starred scenes as a JSON array. Its build ids are
NOT the v2 sheet ids this repo builds from: three id namespaces exist for
overlapping scene sets (v1 "shipped 600", the v2 sheets, and the gallery's
full-pool ids), the same bare name can mean different geometry in each, and
the xml path is the only cross-version identity that is safe. The exported
horizon then selects the `1push` or `hmax2` v2 axis because 52 physical scenes
appear on both. So this script refuses to resolve by id at all. An entry with
`xml` and `horizon` joins exactly; an older XML-only entry joins only when the
XML occurs on one v2 axis.

Input: a JSON array on stdin or as a file argument. Current gallery exports
provide `xml` and `horizon`; `gallery_id` and `dataset` are echoed when present
so a mixed paste is visible. Output: one line per entry with the v2
axis+build_id, tier, marker
verdict, and corridor numbers, ready to hand to make_build_cards /
check_build. Exit 1 if anything failed to resolve, so a partly-bad shortlist
cannot read as a clean one.

  python scripts/resolve_shortlist.py shortlist.json
  xclip -o | python scripts/resolve_shortlist.py
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

REPO_ROOT = Path(__file__).resolve().parents[1]

# One row per sheet entry, all 593, keyed by source xml plus axis.
# post_push_clearance carries the corridor numbers under the same pair.
MARKER_SHEET = REPO_ROOT / "real_trials/sheets_v2_a82a66a/marker_retarget.csv"
CLEARANCE_SHEET = REPO_ROOT / "real_trials/sheets_v2_a82a66a/post_push_clearance.csv"


# The gallery's horizon vocabulary against the sheets' axis vocabulary. Same
# concept, two spellings, mapped in exactly one place.
HORIZON_TO_AXIS = {"1push": "1push", "2push": "hmax2"}


VALID_AXES = frozenset(HORIZON_TO_AXIS.values())


def _index_by_xml_and_axis(path: Path) -> Dict[str, Dict[str, Dict[str, str]]]:
    """xml -> axis -> row. The xml alone is NOT unique: 52 of the 593 scenes
    were evaluated under both horizons and appear on both axes with the same
    xml, so a flat xml index silently keeps whichever row was read last.

    A duplicate (xml, axis) pair raises rather than overwriting, and an axis
    outside the known two raises rather than indexing. Either means the sheet
    is not the frozen input this join assumes, and last-row-wins is the exact
    bug this function exists to remove, one key deeper.
    """
    index: Dict[str, Dict[str, Dict[str, str]]] = {}
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            xml = (row.get("xml") or "").strip()
            if not xml:
                continue
            axis = row["axis"]
            if axis not in VALID_AXES:
                raise ValueError(f"{path.name}: unknown axis {axis!r} for {xml}")
            if axis in index.setdefault(xml, {}):
                raise ValueError(
                    f"{path.name}: duplicate ({xml}, {axis}); the sheet is "
                    f"not the frozen input this join assumes"
                )
            index[xml][axis] = row
    return index


def resolve(entries: List[Dict[str, Any]]) -> int:
    markers = _index_by_xml_and_axis(MARKER_SHEET)
    clearance = _index_by_xml_and_axis(CLEARANCE_SHEET)

    failures = 0
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict):
            failures += 1
            print(f"UNRESOLVED  entry {i}: not an object ({entry!r})")
            continue
        label = str(entry.get("gallery_id") or entry.get("scene") or f"entry {i}")
        dataset = entry.get("dataset")
        tag = f" [{dataset}]" if dataset else ""
        xml_raw = entry.get("xml")
        if xml_raw is not None and not isinstance(xml_raw, str):
            failures += 1
            print(f"UNRESOLVED  {label}{tag}: xml is not a string ({xml_raw!r})")
            continue
        xml = (xml_raw or "").strip() or None
        if dataset == "car-envs":
            # That gallery indexes sim-only pools and legitimately has no xml.
            # Refuse on the dataset, not the missing field, so the message
            # says what is wrong rather than what is absent.
            failures += 1
            print(
                f"UNRESOLVED  {label}{tag}: car-envs scenes are sim-only and "
                f"not buildable from the v2 sheets."
            )
            continue
        if not xml:
            failures += 1
            print(
                f"UNRESOLVED  {label}{tag}: no `xml` field. A bare id is "
                f"ambiguous across the three id namespaces. Stars saved "
                f"before the export fix lack xml until the page is reloaded, "
                f"so reload the gallery and copy the shortlist again."
            )
            continue
        by_axis = markers.get(xml)
        if not by_axis:
            failures += 1
            print(
                f"UNRESOLVED  {label}{tag}: xml not in the v2 pool "
                f"({xml}). car-envs and pre-v2 scenes are not buildable "
                f"from the current sheets."
            )
            continue
        # A PRESENT horizon must be a valid one. `entry.get(...) or ""` would
        # let null, false, 0 or "" fall through to the no-horizon path, and
        # for an axis-unique xml that resolves an entry whose horizon field is
        # explicitly garbage.
        if "horizon" in entry:
            horizon = entry["horizon"]
            if not isinstance(horizon, str) or not horizon.strip():
                failures += 1
                print(
                    f"UNRESOLVED  {label}{tag}: horizon is present but not a "
                    f"usable value ({horizon!r}); expected one of "
                    f"{sorted(HORIZON_TO_AXIS)}."
                )
                continue
            horizon = horizon.strip()
        else:
            horizon = None
        if horizon is not None:
            axis = HORIZON_TO_AXIS.get(horizon)
            if axis is None:
                failures += 1
                print(
                    f"UNRESOLVED  {label}{tag}: unknown horizon "
                    f"{horizon!r}; expected one of {sorted(HORIZON_TO_AXIS)}."
                )
                continue
            row = by_axis.get(axis)
            if row is None:
                failures += 1
                print(
                    f"UNRESOLVED  {label}{tag}: this xml was selected on "
                    f"horizon {horizon!r} but the v2 sheets only carry it on "
                    f"axis {sorted(by_axis)}; the starred card and the sheets "
                    f"disagree, resolve with the sim side."
                )
                continue
        elif len(by_axis) == 1:
            row = next(iter(by_axis.values()))
        else:
            # 52 scenes exist on both axes under one xml. Guessing here files
            # the build under the wrong horizon half the time, which corrupts
            # exactly the pairing the ledger exists to protect.
            failures += 1
            print(
                f"UNRESOLVED  {label}{tag}: this xml exists on both axes "
                f"({sorted(by_axis)}) and the entry carries no horizon field; "
                f"re-copy the shortlist from a gallery build that exports "
                f"horizon, or say which horizon was meant."
            )
            continue
        c = clearance.get(xml, {}).get(row["axis"], {})
        if c and c.get("build_id") != row.get("build_id"):
            failures += 1
            print(
                f"UNRESOLVED  {label}{tag}: marker sheet says "
                f"{row['axis']}/{row['build_id']} but the clearance sheet has "
                f"{c.get('build_id')!r} for the same (xml, axis); the two "
                f"sheets disagree and neither can be trusted for this scene."
            )
            continue
        print(
            f"{row['axis']}/{row['build_id']:10s} tier={row['tier']:5s} "
            f"marker={row['verdict']:8s} "
            f"corridor best={c.get('best_corridor_cm', '?'):>5s} "
            f"worst={c.get('worst_corridor_cm', '?'):>5s}"
            f"  <- {label}{tag}"
        )

    n = len(entries)
    print(f"\n{n - failures} of {n} resolved" + (", FIX THE REST BEFORE BUILDING" if failures else ""))
    return 1 if failures else 0


def main(argv: List[str]) -> int:
    from_file = len(argv) > 1 and argv[1] != "-"
    raw = Path(argv[1]).read_text() if from_file else sys.stdin.read()
    try:
        entries = json.loads(raw)
    except json.JSONDecodeError as exc:
        print(f"not JSON: {exc}")
        return 1
    if not isinstance(entries, list):
        print("expected a JSON array (the gallery's shortlist export)")
        return 1
    if not entries:
        print("empty shortlist")
        return 1
    return resolve(entries)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
