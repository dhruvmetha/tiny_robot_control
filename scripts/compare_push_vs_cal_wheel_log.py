"""Diff per-tick wheel commands between a run_namo push and a calibration trial.

Both sides write JSONL with the same schema (t_s, left_cmd, right_cmd, mode):
  - Calibration: <session_dir>/real/trial_NN/commands.jsonl
  - run_namo push: file pointed to by NAMO_PUSH_WHEEL_LOG env var when the
    push controller runs (set before launching run_namo, e.g.
    `NAMO_PUSH_WHEEL_LOG=/tmp/push_wheels.jsonl python scripts/run_namo.py ...`)

Reports: per-tick means / std / max | min for each stream, and a summary
suitable for spotting "production push uses systematically different wheel
commands than calibration measured".
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Dict

import numpy as np


def load_jsonl(path: Path) -> List[Dict]:
    out: List[Dict] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def summarize(records: List[Dict], label: str) -> Dict:
    if not records:
        print(f"[{label}] empty — nothing to summarize")
        return {}
    t = np.array([r["t_s"] for r in records])
    l = np.array([r["left_cmd"] for r in records])
    r = np.array([r["right_cmd"] for r in records])
    mag = np.maximum(np.abs(l), np.abs(r))
    avg = (l + r) / 2.0
    diff = r - l

    s = {
        "label": label,
        "n_ticks": len(records),
        "duration_s": float(t[-1] - t[0]),
        "tick_rate_hz": len(records) / max(float(t[-1] - t[0]), 1e-9),
        "left_mean": float(l.mean()),  "left_std":  float(l.std()),
        "right_mean": float(r.mean()), "right_std": float(r.std()),
        "avg_wheel_mean": float(avg.mean()),
        "avg_wheel_std":  float(avg.std()),
        "max_wheel_mean": float(mag.mean()),
        "max_wheel_max":  float(mag.max()),
        "wheel_diff_mean": float(diff.mean()),
        "wheel_diff_std":  float(diff.std()),
    }
    return s


def print_table(stats_list: List[Dict]) -> None:
    keys = [
        ("n_ticks",         "%5d"),
        ("duration_s",      "%7.2f"),
        ("tick_rate_hz",    "%6.2f Hz"),
        ("left_mean",       "%6.3f"),
        ("left_std",        "%6.3f"),
        ("right_mean",      "%6.3f"),
        ("right_std",       "%6.3f"),
        ("avg_wheel_mean",  "%6.3f"),
        ("avg_wheel_std",   "%6.3f"),
        ("max_wheel_mean",  "%6.3f"),
        ("max_wheel_max",   "%6.3f"),
        ("wheel_diff_mean", "%+6.3f"),
        ("wheel_diff_std",  "%6.3f"),
    ]
    header = "metric".ljust(22) + "  " + "  ".join(s["label"][:18].ljust(18) for s in stats_list)
    print(header)
    print("-" * len(header))
    for k, fmt in keys:
        row = k.ljust(22) + "  " + "  ".join((fmt % s[k]).ljust(18) for s in stats_list)
        print(row)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("paths", nargs="+",
                   help="JSONL files to summarize. First positional is treated as "
                        "the 'reference' for relative comparisons.")
    p.add_argument("--labels", nargs="+", default=None,
                   help="Label per file (default: filename stem).")
    args = p.parse_args()

    if args.labels and len(args.labels) != len(args.paths):
        print("--labels count must match paths count", file=sys.stderr)
        return 2

    stats = []
    for i, path_str in enumerate(args.paths):
        path = Path(path_str).expanduser()
        if not path.exists():
            print(f"missing: {path}", file=sys.stderr)
            return 2
        records = load_jsonl(path)
        label = args.labels[i] if args.labels else path.stem
        s = summarize(records, label)
        if s:
            stats.append(s)

    if not stats:
        return 1

    print_table(stats)

    if len(stats) >= 2:
        ref, cmp = stats[0], stats[1]
        print()
        print(f"=== {cmp['label']} vs {ref['label']} (relative) ===")
        for k in ("avg_wheel_mean", "max_wheel_mean", "wheel_diff_std",
                  "tick_rate_hz"):
            if ref[k] != 0:
                pct = 100.0 * (cmp[k] - ref[k]) / abs(ref[k])
                print(f"  {k:>20}: {cmp[k]:.4f} vs {ref[k]:.4f}  → {pct:+.1f}%")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
