#!/usr/bin/env python3
"""Resolve a gallery shortlist into v2 sheet scenes, joining on the xml path.

The scene gallery exports starred scenes as a JSON array. Its build ids are
NOT the v2 sheet ids this repo builds from: three id namespaces exist for
overlapping scene sets (v1 "shipped 600", the v2 sheets, and the gallery's
full-pool ids), the same bare name can mean different geometry in each, and
the xml path is the only join that is safe across any pair. So this script
refuses to resolve by id at all. An entry either carries `xml` and joins, or
it is reported unresolvable with the reason.

Input: a JSON array on stdin or as a file argument. Each entry needs `xml`;
`gallery_id` and `dataset` are echoed when present so a mixed paste is
visible. Output: one line per entry with the v2 axis+build_id, tier, marker
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

# One row per scene, all 593, with the xml path each was generated from.
# post_push_clearance carries the corridor numbers under the same key.
MARKER_SHEET = REPO_ROOT / "real_trials/sheets_v2_a82a66a/marker_retarget.csv"
CLEARANCE_SHEET = REPO_ROOT / "real_trials/sheets_v2_a82a66a/post_push_clearance.csv"


def _index_by_xml(path: Path) -> Dict[str, Dict[str, str]]:
    with path.open(newline="") as fh:
        return {
            row["xml"].strip(): row
            for row in csv.DictReader(fh)
            if (row.get("xml") or "").strip()
        }


def resolve(entries: List[Dict[str, Any]]) -> int:
    markers = _index_by_xml(MARKER_SHEET)
    clearance = _index_by_xml(CLEARANCE_SHEET)

    failures = 0
    for i, entry in enumerate(entries):
        label = str(entry.get("gallery_id") or entry.get("scene") or f"entry {i}")
        dataset = entry.get("dataset")
        tag = f" [{dataset}]" if dataset else ""
        xml = (entry.get("xml") or "").strip() or None
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
        row = markers.get(xml)
        if row is None:
            failures += 1
            print(
                f"UNRESOLVED  {label}{tag}: xml not in the v2 pool "
                f"({xml}). car-envs and pre-v2 scenes are not buildable "
                f"from the current sheets."
            )
            continue
        c = clearance.get(xml, {})
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
