# Plan-Only Summary Outcome Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `--sim-xml` plan-only runs write an authoritative success or planning-failure summary instead of being mislabeled by the generic crash fallback.

**Architecture:** Preserve the explicit service outcome on `NAMOPlanBridge`, build one reusable plan-only solution payload, record the planning attempt through the existing diagnostics recorder, and write `summary.json` before returning. Leave `Runtime._write_summary()` and the generic fallback logic unchanged so real execution and genuine crash handling retain their existing contracts.

**Tech Stack:** Python 3.12, pytest, YAML/JSON diagnostics, existing `DiagnosticsRecorder` and `NAMOPlanBridge`.

**Engineering Standards:** Follow `plan-coding-standards`: keep the fix scoped to plan-only outcome propagation and recording, reuse the recorder's aggregation API, add docstrings for changed helper contracts, preserve repository naming and error taxonomy, introduce no environment-specific constants, and require red-green tests plus existing diagnostics/runtime regressions before committing.

---

## File map

- `src/robot_control/planner/namo_bridge.py` retains the explicit success flag and failure reason from the last whole-problem plan.
- `scripts/run_namo.py` builds the plan-only solution payload, records its metrics, writes its terminal summary, and returns the explicit outcome.
- `tests/test_plan_only_summary.py` pins success, failure, zero-push, metrics, and fallback non-overwrite behavior.
- `src/robot_control/KNOWN_ISSUES.md` marks the deterministic limitation resolved and names the regression coverage.

### Task 1: Pin the plan-only summary contract

**Files:**
- Create: `tests/test_plan_only_summary.py`

- [ ] **Step 1: Write failing helper-level tests**

Add tests that construct a real `DiagnosticsRecorder`, call a new
`_write_plan_only_summary()` helper with explicit success/failure solution
payloads, then call `_write_run_summary()` and assert:

```python
assert summary["outcome"] == "success"
assert summary["mode"] == "sim_plan_only"
assert summary["planning"]["simulations_used_total"] == 3
assert summary["planning"]["wall_time_ms_total"] == pytest.approx(125.0)
assert summary_after_fallback == summary
```

Add a failure case asserting `outcome == "planning_failed"` and the contextual
reason, plus an emitter case where explicit success and an empty push list still
produce `success: true`.

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```bash
PYTHONPATH="../namo_cpp/build_python:../namo_cpp/python:src" \
  /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest \
  tests/test_plan_only_summary.py -v
```

Expected: collection fails because `_write_plan_only_summary` does not exist and
the solution emitter does not accept an explicit success outcome.

### Task 2: Preserve and write the explicit outcome

**Files:**
- Modify: `src/robot_control/planner/namo_bridge.py`
- Modify: `scripts/run_namo.py`
- Test: `tests/test_plan_only_summary.py`

- [ ] **Step 1: Add bridge outcome telemetry**

Initialize and reset `last_plan_success: Optional[bool]` and
`last_plan_failure_reason: Optional[str]`. Immediately after `plan_from_xml`,
store `bool(result.success)` and a reason selected from `result.error_message`,
`algorithm_stats["failure_kind"]`, or `None` on success.

- [ ] **Step 2: Make the solution emitter explicit and reusable**

Change `_emit_plan_only_solution_yaml(...)` to accept
`success_override: Optional[bool]`, use it when present, and return the payload
it writes. Preserve the existing action-list inference only for legacy callers
that omit the override.

- [ ] **Step 3: Write the authoritative summary**

Add `_write_plan_only_summary(args, recorder, solution_payload,
algorithm_stats)` which calls `recorder.record_plan()` once with
`planning_operation="fresh_search"`, planner-reported time, simulations used,
explicit success, execution mode, and returned-push count. Then atomically write
a summary containing `success` or `planning_failed`, its reason,
`mode="sim_plan_only"`, goal/configuration fields, recorder totals/planning
aggregates, and solution/planner-scene artifact paths.

- [ ] **Step 4: Route `_run_plan_only_mode()` through the new contract**

For unheld planning, use `bridge.last_plan_success` and
`bridge.last_plan_failure_reason`. For held planning, retain the existing status
as the contextual outcome and treat a returned executable plan as success.
Write the solution and summary before rendering, and return 0 from explicit
success rather than from `bool(plan_subgoals)`.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run:

```bash
PYTHONPATH="../namo_cpp/build_python:../namo_cpp/python:src" \
  /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest \
  tests/test_plan_only_summary.py \
  tests/test_diagnostics_planning_metrics.py \
  tests/test_runtime_terminal_outcomes.py -v
```

Expected: all tests pass; the runtime summary tests prove the real execution
path is unchanged.

### Task 3: Verify the real plan-only path and document closure

**Files:**
- Modify: `src/robot_control/KNOWN_ISSUES.md`

- [ ] **Step 1: Run one successful model plan-only regression**

Use the existing `real_test_envs/1push/1hop/env1` scene with
`--exec-mode greedy_dfs`, diagnostics under a validated temporary directory,
and no real config/camera/serial flags. Assert `solution.yaml` and
`summary.json` both report success, `mode` is `sim_plan_only`, planning metrics
are non-zero, and the process opens no serial device.

- [ ] **Step 2: Update the known issue**

Mark “Plan-only fallback summary can misreport a successful plan” resolved,
identify the root cause and fix, and cite the new regression test and successful
simulation-only verification without claiming real-hardware validation.

- [ ] **Step 3: Run the full repository test suite**

Run:

```bash
cd /home/dhruv/projects_dhruv/namo/robot_control
set -a && . ../namo_cpp/env.robotlearning.sh && set +a
export NAMO_REPO=/home/dhruv/projects_dhruv/namo/namo_cpp
PYTHONPATH="$NAMO_REPO/build_python:$NAMO_REPO/python:src" \
  /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest tests -q
```

Expected: all repository-owned tests pass.

- [ ] **Step 4: Commit the coherent fix**

```bash
git add scripts/run_namo.py src/robot_control/planner/namo_bridge.py \
  src/robot_control/KNOWN_ISSUES.md tests/test_plan_only_summary.py \
  docs/superpowers/specs/2026-08-27-plan-only-summary-outcome-design.md \
  docs/superpowers/plans/2026-08-27-plan-only-summary-outcome.md
git commit -m "fix: record authoritative plan-only summaries"
```

