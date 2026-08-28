# Real-experiment planning telemetry design

**Date:** 2026-08-27

## Problem

The real-trial recorder does not currently provide one reliable answer to
either of these paper metrics:

1. wall-clock time spent in planning operations; and
2. total simulator transitions used by those operations.

Fresh search exposes `simulations_used` only inside `algorithm_stats`.
Prior-chain verification returns `sim_pushes_tried`, but
`NAMOPlanner._record_reuse_diagnostics()` drops it. `summary.json` only
contains event counters, so consumers must reconstruct planning metrics from
`plans.jsonl`. The legacy `search_time_ms` field also mixes scopes: fresh
search is timed inside the planning service, while reuse verification is timed
around bridge preprocessing and simulation.

The completed `hmax2/easy_020/model_search/trial1` demonstrates the loss. Its
fresh searches report 5 and 2 simulations, while the failed one-push suffix
verification tried one additional simulation that was not recorded. The true
total is therefore 8, not the 7 visible in nested fresh-search statistics.

## Goals

- Record a uniformly scoped outer wall-clock duration for every fresh search
  and reuse-verification operation.
- Record the simulator-transition count at the top level of every plan record.
- Put split and combined planning totals directly in `summary.json`.
- Define the fields and aggregation rules in the real-experiment runbook.
- Preserve existing fields so current analysis code does not break.

This change does not alter planning, suffix reuse, robot control, success
criteria, or scene setup. It must be verifiable without camera or robot motion.

## Per-operation schema

Every line in `plans.jsonl` will contain:

- `planning_operation`: one of `fresh_search`, `reuse_verification`, or
  `decision_only`;
- `planning_wall_time_ms`: elapsed monotonic wall time at the
  `NAMOPlanner` call boundary; and
- `simulations_used`: the number of simulator `env.step` transitions executed
  by that operation.

For whole-problem fresh search, the wall-clock interval starts immediately
before `NAMOPlanBridge.plan()` and ends immediately after it returns. For held
boundary search, it surrounds `advance_boundary()`, which is the corresponding
complete planning operation. For suffix/full-chain reuse, it surrounds
`NAMOPlanBridge.verify_chain()`. A goal-retarget record is a decision rather
than a planning operation and records zero for both metrics.

Fresh-search simulation counts come from the bridge's normalized algorithm
statistics. Reuse-verification counts come from
`ChainVerificationResult.sim_pushes_tried`, which increments exactly once per
simulated push. Missing counts are recorded as zero, never omitted.

Existing `search_time_ms`, `cumulative_ms`, `origin`, `plan_source`, and nested
`algorithm_stats` remain unchanged for compatibility. `search_time_ms` remains
a legacy component-reported duration and must not be used as the uniform
wall-clock metric.

## Summary schema

`summary.json` gains a top-level `planning` object:

```json
{
  "wall_time_ms_total": 0.0,
  "wall_time_ms_fresh_search": 0.0,
  "wall_time_ms_reuse_verification": 0.0,
  "simulations_used_total": 0,
  "simulations_used_fresh_search": 0,
  "simulations_used_reuse_verification": 0
}
```

`decision_only` records do not contribute to the split planning totals. The
combined total is the sum of the fresh-search and reuse-verification buckets.
The recorder accumulates these values as it writes each plan record, so runtime
shutdown does not need to reopen or reparse `plans.jsonl`.

## Failure behavior and compatibility

Metrics are recorded for successful and failed planning operations. A failed
verification that simulated one push therefore contributes one simulation.
Diagnostics remain best-effort: recording failures may be logged, but cannot
abort or redirect the robot planner.

Older trial directories remain valid and are not rewritten. They lack the new
uniform wall-clock field and cannot have that value reconstructed after the
fact. The current completed easy trial can, however, be documented as 8 total
simulations and 6129.758 ms of legacy component-reported planning time, split
as 7 fresh-search simulations plus 1 reuse-verification simulation.

## Verification

Tests will first demonstrate the current omissions, then verify:

- fresh, reuse, and decision-only plan lines have the new top-level fields;
- failed reuse verification contributes its simulated pushes;
- recorder totals split fresh search from reuse and exclude decision-only;
- runtime copies the aggregate into `summary.json`; and
- legacy plan fields remain present.

The focused tests and the full repository suite will run in the sourced
`namo_cpp` environment. No physical process will be launched.
