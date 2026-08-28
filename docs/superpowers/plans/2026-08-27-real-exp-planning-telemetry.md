# Real-experiment planning telemetry implementation plan

> Execute this plan in the isolated `fix/real-exp-telemetry-metrics` worktree.
> Do not start the camera service, serial runtime, or physical robot.

**Goal:** Make real-trial planning wall time and simulator usage complete,
uniformly scoped, summarized, and documented.

**Architecture:** `NAMOPlanner` owns per-operation measurement because it is
the common caller boundary for fresh search and chain verification.
`DiagnosticsRecorder` owns online aggregation because it already serializes
each plan line and supplies summary counters to `Runtime`. Existing component
timings and nested statistics remain intact for compatibility.

**Stack:** Python 3.12, pytest, JSON/JSONL diagnostics.

---

## Task 1: Prove and fix recorder aggregation

**Files:**

- Create: `tests/test_diagnostics_planning_metrics.py`
- Modify: `src/robot_control/diagnostics/recorder.py`

1. Write a test that creates an enabled `DiagnosticsRecorder`, records one
   `fresh_search`, one failed `reuse_verification`, and one `decision_only`
   event, then asserts split and combined wall-time/simulation totals.
2. Run the focused test and verify it fails because no planning aggregate
   exists.
3. Add a zero-initialized `planning` aggregate and update it from top-level
   `planning_operation`, `planning_wall_time_ms`, and `simulations_used` fields
   after a plan record is written.
4. Ensure `decision_only` does not enter fresh/reuse totals and metrics remain
   no-ops when the recorder is disabled.
5. Run the focused test and verify it passes.
6. Commit only the recorder and its test.

## Task 2: Prove and fix per-operation planner records

**Files:**

- Modify: `tests/test_namo_planner_diagnostics.py`
- Modify: `tests/test_namo_planner_chain_reuse.py`
- Modify: `src/robot_control/planner/namo_planner.py`

1. Write a direct diagnostics test showing a fresh plan record must expose
   `planning_operation=fresh_search`, outer `planning_wall_time_ms`, and
   top-level `simulations_used`, while preserving `search_time_ms` and nested
   `algorithm_stats`.
2. Write a chain-reuse test showing a failed suffix verification records
   `sim_pushes_tried` and the independently measured outer duration.
3. Run both focused tests and verify they fail for the current dropped fields.
4. Add explicit `planning_wall_time_ms` parameters to the shared fresh/held
   diagnostic method and `simulations_used` plus outer time to reuse recording.
5. Measure monotonic time immediately around `_bridge.plan()`,
   `advance_boundary()`, and `_bridge.verify_chain()`; do not include robot
   execution or observation wait time.
6. Add zero-valued `decision_only` metrics to goal-retarget records.
7. Normalize fresh simulation statistics once and reuse the normalized value
   at both top level and inside `algorithm_stats`.
8. Run the focused tests and all planner diagnostics/reuse tests.
9. Commit only planner telemetry and its tests.

## Task 3: Put the aggregate in runtime summaries

**Files:**

- Modify: `tests/test_runtime_terminal_outcomes.py`
- Modify: `src/robot_control/runtime.py`

1. Write a summary test with a temporary recorder that asserts the emitted
   `summary.json` contains the recorder's exact `planning` aggregate.
2. Run the test and verify it fails because `_write_summary()` omits that
   object.
3. Add a copied `planning` object to the summary payload so later recorder
   changes cannot mutate an already-built payload.
4. Run the focused runtime and recorder tests.
5. Commit the runtime summary change and test.

## Task 4: Document collection and legacy-trial interpretation

**Files:**

- Create: `real_exp/METRICS.md`
- Modify: `real_exp/README.md`
- Modify: `tests/test_runtime_terminal_outcomes.py` or create a focused
  documentation test if needed

1. Add a metrics reference defining the three per-plan fields, fresh/reuse
   timing boundaries, simulation semantics, aggregation formulas, and legacy
   `search_time_ms` limitation.
2. Document that the completed easy trial has 8 simulator transitions (7
   fresh + 1 reuse) and 6129.758 ms of legacy component-reported time, but no
   reconstructable uniform outer duration.
3. Link the metrics reference from the experiment README and remove stale text
   that identifies the archived incomplete trial as an active result.
4. Add or update a lightweight documentation assertion for the required
   metric names and definitions.
5. Run the focused documentation test and commit docs/tests together.

## Task 5: Verify, review, integrate, and push

1. Source `/home/dhruv/projects_dhruv/namo/namo_cpp/env.robotlearning.sh`, set
   `NAMO_REPO`, and run all focused telemetry tests.
2. Run `python -m pytest tests -q` with the pinned Python 3.12 interpreter.
3. Run `git diff --check`, inspect the complete branch diff from
   `ac76104`, and confirm the primary checkout's pre-existing dirty files are
   untouched.
4. Review the implementation against the design, checking failed operations,
   zero-simulation cases, summary copying, and backward-compatible fields.
5. Fast-forward the primary `real-robot` branch only after all checks pass;
   preserve all untracked experiment data and unrelated GUI/submodule edits.
6. Re-run focused checks from the integrated primary checkout and push
   `real-robot`.
