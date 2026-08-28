# Planning metrics for real experiments

This file defines the planning metrics recorded for paper trials. Use
`summary.json` for run-level values and `plans.jsonl` when per-operation detail
is needed. `summary.json.duration_sec` is the whole physical trial—including
robot motion and observation waits—and is not planning time.

## Per-operation records

Each `plans.jsonl` record contains these top-level fields:

- `planning_operation` classifies the record as `fresh_search`,
  `reuse_verification`, or `decision_only`.
- `planning_wall_time_ms` is elapsed monotonic wall-clock time at the
  `NAMOPlanner` call boundary.
- `model_warmup_ms` is the separately measured one-time checkpoint, renderer, device, and synthetic-forward initialization cost; it is positive only on the first model-prior planning record.
- `model_warmup_excluded_from_planning_time` states whether that record carries a warmup duration excluded from `planning_wall_time_ms`.
- `simulations_used` is the number of simulator `env.step` transitions made
  by that operation. Count transitions even when the operation fails.

The timing boundary depends only on the operation class:

- `fresh_search`: immediately before and after `NAMOPlanBridge.plan()` for
  whole-problem search, or around `advance_boundary()` for held-boundary
  search;
- `reuse_verification`: immediately before and after
  `NAMOPlanBridge.verify_chain()`; and
- `decision_only`: zero. A goal retarget does not run search or simulation.

For learned best-first search, warmup completes before the first operation clock starts. It is recorded for auditability but excluded from `planning_wall_time_ms`; actual-scene rendering, ranker inference, candidate generation, graph construction, and simulation remain inside the measured operation.

This outer timing includes scene conversion and the called planning or
verification operation. It excludes physical robot execution, waiting for a
new camera observation, video recording after the call, and the runtime's
final navigation to the goal.

`search_time_ms` and `cumulative_ms` remain in the record for compatibility.
Do not use them as the uniform paper wall-clock metric: historically fresh
search timed the planning service internally, while reuse timed a broader
bridge operation. Nested `algorithm_stats.simulations_used` also remains for
compatibility, but the top-level `simulations_used` is authoritative across
both fresh and reuse paths.

## Run summary

New trials put these fields under `summary.json.planning`:

- `wall_time_ms_fresh_search`: sum of `planning_wall_time_ms` for
  `fresh_search` records;
- `wall_time_ms_reuse_verification`: corresponding sum for
  `reuse_verification` records;
- `wall_time_ms_total`: the two wall-time sums above added together;
- `simulations_used_fresh_search`: sum of `simulations_used` for fresh search;
- `simulations_used_reuse_verification`: corresponding sum for reuse;
- `simulations_used_total`: the two simulation-count sums above added together; and
- `model_warmup_ms`: separately accumulated one-time model initialization; it is not included in `wall_time_ms_total`.

Successful and unsuccessful operations both contribute. `decision_only`
records contribute to the existing `totals.plan_calls` compatibility counter,
but not to any planning total.

For a paper table, report `wall_time_ms_total` and `simulations_used_total`.
Keep the split values when diagnosing why a run was expensive or comparing
fresh replanning with suffix reuse.

Report `model_warmup_ms` separately when describing system startup overhead. Never add it to the paper planning-time metric.

## Completed easy trial recorded before this schema

`results/hmax2/easy_020/model_search/trial1` is the accepted completed easy
trial. It reached the real goal in 49.608 s, used two successful physical
pushes, and finished 1.108 cm from the requested goal.

That trial predates the fields above. Its existing records support this
forensic count:

- plan 1: 5 fresh-search simulator transitions;
- plan 2: 1 reuse-verification simulator transition for the one-push suffix,
  followed by `goal_not_reachable_after_chain`; and
- plan 3: 2 fresh-search simulator transitions.

The trial therefore used **8 simulator transitions**: **7 fresh-search** plus
**1 reuse-verification**. The old `search_time_ms` values sum to
**6129.758 ms** (5847.774 ms fresh plus 281.983 ms reuse). This is retained as
a legacy component-reported timing only. The new uniformly scoped
`planning_wall_time_ms` was not captured and cannot be reconstructed after the
fact; do not substitute 6129.758 ms for it in a cross-trial wall-time table.
That legacy timing also predates explicit scorer warmup, so any one-time model initialization inside it cannot be separated after the fact.
This does not invalidate the physical trial: its successful outcome, physical
pushes, final navigation, and reconstructed simulator count remain valid. We
will rerun it later for directly comparable timing; that rerun adds a timed
replicate rather than replacing this result.

Do not edit the old JSONL or summary to make them look like new-schema output.
Their absence of the new fields is provenance that the run used the older
recorder. All subsequent paper trials should be run from a commit containing
this telemetry schema.
