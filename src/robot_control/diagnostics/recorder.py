"""Thread-safe diagnostics event recorder.

One DiagnosticsRecorder is created per run. When the underlying root directory
is None, every method is a no-op — call sites can invoke recorder methods
unconditionally without checking enabled state. When enabled, events are
appended to JSONL files line-by-line (line-buffered, flushed after each write
for crash safety) and the final summary is written atomically via temp+rename.
"""

from __future__ import annotations

import datetime as _dt
import json
import os
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional


def _now_pair() -> Dict[str, Any]:
    # Both fields together — machine-readable epoch and human-readable ISO.
    # Always UTC to eliminate timezone ambiguity in downstream tooling.
    now = time.time()
    iso = _dt.datetime.fromtimestamp(now, tz=_dt.timezone.utc).isoformat(
        timespec="milliseconds"
    ).replace("+00:00", "Z")
    return {"at_epoch": now, "at_utc_iso": iso}


class DiagnosticsRecorder:
    """Thread-safe sink for run-time diagnostics events.

    When root_dir is None, the recorder is disabled and every method returns
    immediately with a no-op. This lets call-sites invoke recorder methods
    unconditionally, keeping the production code free of `if recorder:` guards.

    Public methods:
      write_config(payload)
      save_scene(when, jpg_bytes, json_payload, xml_path_to_copy)
      log_capture_error(when, kind, reason)
      record_plan(payload) -> plan_id
      record_subgoal_start(payload) -> subgoal_id
      record_subgoal_end(subgoal_id, outcome)
      record_push(payload)
      record_connectivity(payload)
      write_summary(payload)
      close()

    Aggregated counters (read via .totals) and planning metrics (read via
    .planning) accumulate during the run; the Runtime is expected to fold
    them into the summary before calling write_summary().
    """

    JSONL_FILES = (
        "plans.jsonl",
        "subgoals.jsonl",
        "pushes.jsonl",
        "connectivity.jsonl",
    )

    def __init__(
        self,
        root_dir: Optional[Path] = None,
        allow_overwrite: bool = False,
        verbose: bool = True,
    ) -> None:
        self._root: Optional[Path] = None
        self._lock = threading.Lock()
        self._verbose = verbose

        # Counters surfaced through .totals — used by Runtime to populate
        # summary.json without re-parsing the JSONL files.
        self.totals: Dict[str, Any] = {
            "plan_calls": 0,
            "subgoals_dispatched": 0,
            "subgoals_succeeded": 0,
            "subgoals_failed": 0,
            "pushes_attempted": 0,
            "pushes_succeeded": 0,
            "pushes_stuck": 0,
            "connectivity_transitions": 0,
            "offline_during_plan_count": 0,
        }
        self.planning: Dict[str, Any] = {
            "wall_time_ms_total": 0.0,
            "wall_time_ms_fresh_search": 0.0,
            "wall_time_ms_reuse_verification": 0.0,
            "simulations_used_total": 0,
            "simulations_used_fresh_search": 0,
            "simulations_used_reuse_verification": 0,
            "model_warmup_ms": 0.0,
        }

        # Sequential IDs assigned as events come in.
        self._plan_id = 0
        self._subgoal_id = 0

        # Open subgoal records keyed by subgoal_id, waiting for their _end
        # event. Each holds the start payload so the JSONL record can be
        # written atomically with combined start+end fields.
        self._open_subgoals: Dict[int, Dict[str, Any]] = {}

        # File handles, opened lazily when enabled.
        self._files: Dict[str, Any] = {}
        self._capture_error_log: Optional[Any] = None

        if root_dir is None:
            return

        # Resolve + verify the target directory. Allow overwrite only when
        # explicitly opted in; otherwise refuse to clobber existing runs.
        root = Path(root_dir).expanduser().resolve()
        if root.exists():
            if not allow_overwrite:
                raise FileExistsError(
                    f"Diagnostics directory already exists: {root} "
                    f"(pass --allow-overwrite to replace it)"
                )
            # Clean out so we don't mingle records from a previous run.
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=False)

        self._root = root
        for name in self.JSONL_FILES:
            # Line-buffered so each appended record is durable.
            self._files[name] = open(root / name, "w", buffering=1)

        if self._verbose:
            print(f"[Diagnostics] Recording to {self._root}", flush=True)

    # ------------------------------------------------------------------ status

    @property
    def enabled(self) -> bool:
        return self._root is not None

    @property
    def root(self) -> Optional[Path]:
        return self._root

    # --------------------------------------------------------------- top-level

    def write_config(self, payload: Dict[str, Any]) -> None:
        if not self.enabled:
            return
        with self._lock:
            with open(self._root / "config.json", "w") as f:
                json.dump(payload, f, indent=2, default=_json_fallback)
                f.write("\n")
        if self._verbose:
            print("[Diagnostics] Wrote config.json", flush=True)

    def write_summary(self, payload: Dict[str, Any]) -> None:
        if not self.enabled:
            return
        # Atomic write: tmp file + rename. Readers never see a partial file.
        tmp = self._root / "summary.json.tmp"
        final = self._root / "summary.json"
        with self._lock:
            with open(tmp, "w") as f:
                json.dump(payload, f, indent=2, default=_json_fallback)
                f.write("\n")
            os.replace(tmp, final)
        if self._verbose:
            print(f"[Diagnostics] Wrote summary.json (outcome={payload.get('outcome')})", flush=True)

    # ------------------------------------------------------------ scene assets

    def save_scene(
        self,
        when: str,                                # "before" | "after"
        jpg_bytes: Optional[bytes],
        json_payload: Optional[Dict[str, Any]],
        xml_source_path: Optional[Path],
    ) -> Dict[str, Optional[str]]:
        """Save scene artifacts. Returns dict with the saved relative paths
        (None for any artifact that failed). Errors are appended to
        scene_capture_errors.log and bubbled up via the return value so the
        caller can list them in summary.json.
        """
        if not self.enabled:
            return {"jpg": None, "json": None, "xml": None}

        results: Dict[str, Optional[str]] = {"jpg": None, "json": None, "xml": None}

        # JPG
        if jpg_bytes is not None:
            try:
                rel = f"scene_{when}.jpg"
                with self._lock:
                    with open(self._root / rel, "wb") as f:
                        f.write(jpg_bytes)
                results["jpg"] = rel
            except Exception as exc:
                self.log_capture_error(when, "jpg", f"write failed: {exc!r}")
        else:
            self.log_capture_error(when, "jpg", "no image bytes captured")

        # JSON
        if json_payload is not None:
            try:
                rel = f"scene_{when}.json"
                with self._lock:
                    with open(self._root / rel, "w") as f:
                        json.dump(json_payload, f, indent=2, default=_json_fallback)
                        f.write("\n")
                results["json"] = rel
            except Exception as exc:
                self.log_capture_error(when, "json", f"write failed: {exc!r}")
        else:
            self.log_capture_error(when, "json", "no scene-state payload")

        # XML (copy from a source path the planner wrote to, then clean up
        # the transient source file — we don't want _scene_*_tmp.xml hanging
        # around in the run directory next to the canonical scene_*.xml).
        if xml_source_path is not None:
            try:
                src = Path(xml_source_path)
                if not src.exists():
                    self.log_capture_error(when, "xml", f"source missing: {src}")
                else:
                    rel = f"scene_{when}.xml"
                    with self._lock:
                        shutil.copyfile(src, self._root / rel)
                    results["xml"] = rel
                    # Best-effort cleanup of the transient source.
                    try:
                        if src.name.startswith("_scene_"):
                            src.unlink()
                    except Exception:
                        pass
            except Exception as exc:
                self.log_capture_error(when, "xml", f"copy failed: {exc!r}")
        else:
            self.log_capture_error(when, "xml", "no xml source path provided")

        return results

    def log_capture_error(self, when: str, kind: str, reason: str) -> None:
        if not self.enabled:
            return
        line = f"{_now_pair()['at_utc_iso']} scene_{when}.{kind}: {reason}\n"
        with self._lock:
            if self._capture_error_log is None:
                self._capture_error_log = open(
                    self._root / "scene_capture_errors.log", "a", buffering=1
                )
            self._capture_error_log.write(line)
        if self._verbose:
            print(f"[Diagnostics] ⚠️ scene_{when}.{kind} capture failed: {reason}", flush=True)

    # -------------------------------------------------------------- event sink

    def record_plan(self, payload: Dict[str, Any]) -> int:
        if not self.enabled:
            return 0
        with self._lock:
            self._plan_id += 1
            plan_id = self._plan_id
        full = {"plan_id": plan_id, **_now_pair(), **payload}
        self._write_jsonl("plans.jsonl", full)
        operation = payload.get("planning_operation")
        with self._lock:
            self.totals["plan_calls"] += 1
            warmup_ms = max(
                0.0, float(payload.get("model_warmup_ms", 0.0) or 0.0)
            )
            self.planning["model_warmup_ms"] += warmup_ms
            if operation in {"fresh_search", "reuse_verification"}:
                wall_time_ms = max(
                    0.0, float(payload.get("planning_wall_time_ms", 0.0) or 0.0)
                )
                simulations_used = max(
                    0, int(payload.get("simulations_used", 0) or 0)
                )
                self.planning["wall_time_ms_total"] += wall_time_ms
                self.planning["simulations_used_total"] += simulations_used
                self.planning[f"wall_time_ms_{operation}"] += wall_time_ms
                self.planning[f"simulations_used_{operation}"] += simulations_used
        if self._verbose:
            print(f"[DIAG] plan #{plan_id} recorded (success={payload.get('success')})", flush=True)
        return plan_id

    def record_subgoal_start(self, payload: Dict[str, Any]) -> int:
        if not self.enabled:
            return 0
        with self._lock:
            self._subgoal_id += 1
            subgoal_id = self._subgoal_id
            start_record = {
                "subgoal_id": subgoal_id,
                "dispatched": _now_pair(),
                **payload,
            }
            self._open_subgoals[subgoal_id] = start_record
        self.totals["subgoals_dispatched"] += 1
        if self._verbose:
            print(f"[DIAG] subgoal #{subgoal_id} start recorded", flush=True)
        return subgoal_id

    def record_subgoal_end(
        self, subgoal_id: int, outcome: Dict[str, Any]
    ) -> None:
        if not self.enabled:
            return
        with self._lock:
            start = self._open_subgoals.pop(subgoal_id, None)
        if start is None:
            # End without a matching start — emit as a standalone record so it
            # isn't silently dropped, but mark it so consumers know.
            full = {
                "subgoal_id": subgoal_id,
                "_warning": "end_without_start",
                "completed": _now_pair(),
                **outcome,
            }
        else:
            duration = _now_pair()["at_epoch"] - start["dispatched"]["at_epoch"]
            full = {
                **start,
                "completed": _now_pair(),
                "duration_sec": duration,
                **outcome,
            }
        self._write_jsonl("subgoals.jsonl", full)

        is_success = outcome.get("outcome") == "success"
        if is_success:
            self.totals["subgoals_succeeded"] += 1
        else:
            self.totals["subgoals_failed"] += 1
        if self._verbose:
            print(
                f"[DIAG] subgoal #{subgoal_id} end recorded "
                f"(outcome={outcome.get('outcome')})",
                flush=True,
            )

    def record_push(self, payload: Dict[str, Any]) -> None:
        if not self.enabled:
            return
        full = {**_now_pair(), **payload}
        self._write_jsonl("pushes.jsonl", full)
        self.totals["pushes_attempted"] += 1
        if payload.get("stuck"):
            self.totals["pushes_stuck"] += 1
        else:
            self.totals["pushes_succeeded"] += 1
        if self._verbose:
            mag = payload.get("delta_pos_magnitude_cm", 0.0)
            stuck = payload.get("stuck", False)
            print(
                f"[DIAG] push recorded "
                f"(obj={payload.get('object_id')}, Δ={mag:.2f}cm, stuck={stuck})",
                flush=True,
            )

    def record_connectivity(self, payload: Dict[str, Any]) -> None:
        if not self.enabled:
            return
        full = {**_now_pair(), **payload}
        self._write_jsonl("connectivity.jsonl", full)
        if payload.get("event") == "transition":
            self.totals["connectivity_transitions"] += 1
        elif payload.get("event") == "offline_during_plan":
            self.totals["offline_during_plan_count"] += 1

    # --------------------------------------------------------------- lifecycle

    def close(self) -> None:
        if not self.enabled:
            return
        with self._lock:
            for fh in self._files.values():
                try:
                    fh.flush()
                    fh.close()
                except Exception:
                    pass
            self._files.clear()
            if self._capture_error_log is not None:
                try:
                    self._capture_error_log.flush()
                    self._capture_error_log.close()
                except Exception:
                    pass
                self._capture_error_log = None

    # --------------------------------------------------------------- internals

    def _write_jsonl(self, name: str, record: Dict[str, Any]) -> None:
        fh = self._files.get(name)
        if fh is None:
            return
        line = json.dumps(record, default=_json_fallback)
        with self._lock:
            fh.write(line)
            fh.write("\n")
            # Line-buffered open() already flushes after newline; explicit
            # flush is a belt-and-braces guarantee for crash recovery.
            fh.flush()


def _json_fallback(obj: Any) -> Any:
    # Best-effort coercion for objects that aren't natively JSON-serializable.
    # Numpy scalars, Paths, datetimes, sets, dataclasses — convert here rather
    # than at every call-site. Anything still unsupported falls back to repr().
    try:
        import numpy as np  # type: ignore
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.generic):
            return obj.item()
    except ImportError:
        pass
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, _dt.datetime):
        return obj.isoformat()
    if isinstance(obj, set):
        return sorted(obj)
    # Dataclass instances expose __dataclass_fields__.
    if hasattr(obj, "__dataclass_fields__"):
        from dataclasses import asdict
        return asdict(obj)
    return repr(obj)
