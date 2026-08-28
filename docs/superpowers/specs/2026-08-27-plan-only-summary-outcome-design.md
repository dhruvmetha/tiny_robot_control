# Plan-Only Summary Outcome Design

## Problem

`scripts/run_namo.py --sim-xml` deliberately bypasses `Runtime`, writes an
authoritative `solution.yaml`, and returns. The top-level `finally` block then
calls `_write_run_summary()`, whose only current contract is a crash fallback
for runs where `Runtime` failed to write `summary.json`. It therefore labels
every completed plan-only invocation as `outcome: crashed`, including successful
plans.

## Approved behavior

Plan-only owns its terminal summary because it owns the terminal outcome. After
planning, it will write `summary.json` before returning. A successful planner
result records `outcome: success`; an ordinary no-plan result records
`outcome: planning_failed` and the planner's contextual reason. The summary uses
`mode: sim_plan_only`, includes the goal, planner configuration, pushes returned,
planner-reported search time, simulations used, recorder totals, recorder
planning aggregates, and artifact paths.

`NAMOPlanBridge` will retain the explicit success flag and failure reason from
the most recent `plan_from_xml` result. Plan-only will use that flag rather than
infer success from a non-empty action list. This preserves the correct contract
for a successful zero-action result and prevents partial diagnostic actions from
being mistaken for a plan.

The existing fallback remains unchanged. Because the authoritative plan-only
summary exists by the time the top-level `finally` runs, the fallback's existing
"do not overwrite" guard becomes the isolation boundary: real-runtime summary
behavior and crash handling do not change.

## Data flow

1. `NAMOPlanBridge.plan()` resets and then records `last_plan_success` and
   `last_plan_failure_reason` from the service result.
2. `_run_plan_only_mode()` builds one structured solution payload from the
   explicit planner outcome and writes `solution.yaml`.
3. `_run_plan_only_mode()` records the planning attempt through
   `DiagnosticsRecorder.record_plan()` so totals and planning aggregates are
   populated.
4. `_run_plan_only_mode()` writes the authoritative `summary.json` and returns
   exit code 0 for success or 1 for planning failure.
5. `_write_run_summary()` observes the existing file and returns without
   overwriting it.

## Failure handling

If the bridge has no explicit result, plan-only records `planning_failed` with
`planner did not return a result`. An empty service error falls back to the
planner's `failure_kind`, then to `planner returned no plan`. Summary-writing
errors continue to be caught by the existing top-level diagnostics guard and do
not alter real robot execution.

## Testing

- A successful plan-only result writes `outcome: success` and survives the
  generic fallback unchanged.
- A failed result writes `outcome: planning_failed` with its reason.
- Explicit success with zero pushes remains success in both `solution.yaml` and
  `summary.json`.
- Planning time and simulation counts populate `plans.jsonl`, `totals`, and the
  summary's planning aggregate.
- Existing runtime terminal-outcome and diagnostics metric tests remain green.

