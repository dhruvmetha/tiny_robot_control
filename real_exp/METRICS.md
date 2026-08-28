# Planning metrics for formal-v2 real experiments

Use `summary.json` for run-level values and `plans.jsonl` for per-operation
detail. `summary.json.duration_sec` is the complete physical trial, including
motion and observation waits; it is not planning time.

## Per-operation records

Each `plans.jsonl` record contains:

- `planning_operation`: `fresh_search`, `reuse_verification`, or
  `decision_only`;
- `planning_wall_time_ms`: monotonic wall time at the planner-call boundary;
- `simulations_used`: physics `env.step` transitions made by that operation;
- `model_warmup_ms`: separately measured one-time model initialization; and
- `model_warmup_excluded_from_planning_time`: whether that warmup is excluded
  from the operation wall time.

For learned best-first planning, warmup completes before the first operation
clock begins. Actual-scene graph construction, candidate generation, model-input
rendering, inference, and any requested simulations remain inside the measured
operation.

Planning wall time excludes physical robot execution, waiting for the next
camera observation, post-plan video recording, and final navigation. Retain
`search_time_ms` only for compatibility; use `planning_wall_time_ms` for formal
analysis.

## Run summary

Report these authoritative fields from `summary.json.planning`:

- `wall_time_ms_total`
- `simulations_used_total`

Keep these diagnostic splits:

- `wall_time_ms_fresh_search`
- `wall_time_ms_reuse_verification`
- `simulations_used_fresh_search`
- `simulations_used_reuse_verification`

Report `model_warmup_ms` separately as startup overhead. Never add it to the
paper planning-time metric. Successful and failed planning operations both
contribute to the totals.

## Arm accounting

### `model_search`

Full NAMO search uses physics rollouts to construct a complete plan. Every
attempted `env.step` counts, including actions rejected as no-ops, jams, or
unsuccessful opening attempts. Suffix verification also contributes its own
wall time and simulation transitions.

### `model_pure_policy`

The pure-policy arm uses `--exec-mode greedy_policy`. Every live decision
rebuilds the graph, generates candidates, renders model input, runs inference,
and returns the ranked arg-max without calling `env.step`.

Therefore:

- `simulations_used` is zero for each pure-policy decision;
- `simulations_used_total` is zero for a correctly configured pure-policy run;
- `planning_wall_time_ms` remains nonzero because graph construction, rendering,
  candidate ranking, and inference are measured; and
- the next camera observation starts a new `fresh_search` record rather than a
  suffix-reuse verification.

Physical push counts are separate. Use `totals.pushes_attempted`,
`totals.pushes_succeeded`, and `totals.pushes_stuck` to analyze what the robot
did; do not substitute those values for simulator transitions.

## Success and provenance

A returned plan or policy action is not trial success. A real trial succeeds
only after final camera-confirmed navigation to the goal tolerance.

Every formal-v2 `config.json` must contain:

- `repositories.robot_control.available: true`;
- `repositories.namo_cpp.available: true`; and
- the frozen backend commit documented in `README.md`.

Repository state is captured before the output directory is created, so the
run's own files do not make a clean source checkout appear dirty.
