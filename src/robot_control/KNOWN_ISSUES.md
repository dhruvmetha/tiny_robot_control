# Known issues

Living document of bugs that are known, reproducible, and worth a fix but
haven't been addressed yet. Each entry should explain **what's broken**,
**why it matters**, **how to reproduce**, **likely cause**, and **what the
fix would look like** so future contributors can pick it up cleanly.

---

## 1. Runtime auto-quit on `quit_on_complete=True` is racy

**Tags:** runtime, gui, qt, threading
**Files:**
- `robot_control/runtime.py` — control-loop close site (~line 845)
- `robot_control/gui/window.py` — `close_window()` (~line 399)

### Symptom

When a NAMO run completes successfully (e.g. `GOAL REACHED`), the runtime
prints `[Runtime] Plan complete - stopping robot` and `[Runtime] Shutting
down`, but the GUI window does **not** always close. The main thread stays
blocked inside `_window.run() → QApplication.exec()`, and the `run()`
method's `finally` block never executes.

Visible downstream consequences:

- `summary.json` is missing from the diagnostics directory (because the
  runtime never gets to call `_write_summary()`).
- `scene_after.{jpg,json,xml}` are missing.
- The Python process appears stuck — only force-quitting the window (or
  sending SIGKILL) frees it.
- It is **intermittent**: short runs (0 plans, just navigate-to-goal)
  usually succeed; longer runs (many plans, multiple pushes) more often
  hang.

### Why it matters

Affects every developer running NAMO with `quit_on_complete=True`, which
is the default in autonomous mode. For diagnostics it means the on-disk
summary may be missing — though the per-event JSONL files (`plans.jsonl`,
`subgoals.jsonl`, `pushes.jsonl`, `connectivity.jsonl`) are still complete
because they are written line-by-line as events occur. **The diagnostics
recorder itself is not at fault** — when `finally` does fire, every file
is written correctly. The bug is purely in the runtime's GUI auto-close
path.

### Reproduction

```bash
cd robot_control
python scripts/run_namo.py \
    --config config/real.yaml \
    --camera-service tcp://localhost:5556 \
    --goal 25 65 \
    --diag-path /tmp/namo_diag \
    --run-name "{strategy}_{timestamp}" \
    --capture-scene \
    --verbose
```

Run multiple times. In runs where the planner does several push/replan
iterations before reaching the goal, the window often won't close and
`/tmp/namo_diag/.../summary.json` will be absent. Compare to runs where
the goal is immediately reachable (0 plans) — those typically write
`summary.json` correctly.

### Likely cause

`runtime.py` calls `self._window.close_window()` from the **control
thread** (non-GUI). `close_window()` was originally implemented as
`self.close()`, which mutates a `QMainWindow` cross-thread — undefined
behaviour in Qt. We changed it to `self._app.quit()` (which Qt documents
as thread-safe), but the GUI thread may already be wedged on something
else, so the deferred quit event never gets processed.

Three suspects identified during debugging:

1. **`_window.update(obs)` / `set_status()` / `update_drawings()` calls
   from the control thread** (`runtime.py:836-838`, immediately before
   `close_window()`). These are direct GUI widget mutations from a
   non-GUI thread — a well-known cause of Qt hangs/deadlocks. The
   `quit()` we then call from the same thread may be sitting behind
   broken events that the GUI thread can't drain.
2. **The `Tee` wrapped around stdout/stderr** can stall the GUI loop if
   any Qt code path holds a lock while writing.
3. **`_capture_scene_snapshot("before")` on the main thread before
   `_window.run()`** — if the camera_service REQ/REP call blocks the
   2-second timeout, the GUI never starts cleanly. Less likely the root
   cause but worth ruling out.

### What the fix should look like

Most likely fix (in priority order):

1. **Stop touching the GUI from the control thread.** Drop the
   `_window.update(obs)` / `set_status()` / `update_drawings()` calls
   in the close branch — the window is about to close anyway. Then
   `close_window()` runs against a clean event queue.
2. **Use `QTimer.singleShot(0, lambda: self.close())`** instead of
   `_app.quit()`. The singleShot posts a close request onto the GUI
   thread's own event loop, where it's safe to mutate widgets.
3. **Install a SIGINT handler that calls `QApplication.quit()`** as a
   manual escape hatch — lets a developer Ctrl+C out of a hung run
   without needing to force-quit the window.

### Workarounds today

- Close the GUI window manually (click the X) when the run is done.
  This unblocks `_window.run()` and the `finally` block fires normally
  — summary.json + scene_after are written.
- The JSONL streams (`plans.jsonl`, `subgoals.jsonl`, `pushes.jsonl`,
  `connectivity.jsonl`) are crash-safe (line-buffered, flushed per
  write), so even if the window has to be force-quit the per-event
  data is intact.
