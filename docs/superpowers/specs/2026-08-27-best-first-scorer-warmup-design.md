# Best-First Scorer Warmup and Real-Trial Seed Protocol

## Goal

Exclude one-time learned-ranker initialization from measured planning wall time while preserving it as separately recorded experimental overhead, and freeze the formal real-trial seeds to `0,1,2,3,4`.

## Experimental interpretation

The learned model is a search ranker: simulator transitions measure how efficiently it orders candidate pushes, while planning wall time includes repeated scene rendering, ranker inference, and simulator execution performed by search. One-time process startup work is not a property of an individual planning decision and must therefore be completed before the measured planning interval.

The warmup exclusion is narrow. It excludes checkpoint loading, renderer construction, device initialization, and repeated synthetic ranker forward passes. It does not exclude rendering or inference on the actual observed scene, candidate generation, graph construction, simulation, scene conversion, or suffix verification.

## Warmup ownership and flow

`namo_cpp` owns construction and inference for the best-first scorer, so `NAMOPlanningService` will expose a preload-and-warm method that resolves the same process-cached `LiveScorer` instance used by `BestFirstRegionOpeningPlanner`. `LiveScorer` will expose a small warmup method that performs repeated forward passes with correctly shaped synthetic context and contact tensors on its configured device.

`NAMOPlanBridge` will expose a robot-control-facing warmup method that calls the planning service and measures the complete one-time duration. `NAMOPlanner` will invoke that method once, immediately before the first model-prior best-first planning operation and before starting `planning_wall_time_ms`. Uniform-prior best-first and non-best-first planners will not warm the scorer.

The same guard applies to both whole-problem and held-boundary planning paths. The process-level scorer cache ensures that the warmed object is the object later used by planning. A warmup failure propagates as a planning failure before robot motion; the runtime must not silently continue with a cold measured inference.

## Recorded metrics

Every planning diagnostic record will carry `model_warmup_ms`, with a positive value only on the operation before which warmup first ran and zero otherwise. It will also carry `model_warmup_excluded_from_planning_time`, true for the warmed model operation and false otherwise.

`summary.json.planning.model_warmup_ms` will sum the separately recorded warmup duration. `planning.wall_time_ms_total` will continue to sum only measured fresh search and reuse verification and will never include warmup. Documentation will state both boundaries explicitly.

## Formal seed protocol

Each formal experiment cell uses exactly five physical replicates with explicit `--shuffle-seed 0`, `1`, `2`, `3`, and `4`. The same seed labels are recorded for every cell so physical replicates remain aligned.

Uniform best-first consumes the seed when assigning random candidate priorities. Learned-model best-first is deterministic for an identical observed scene and does not consume the random generator in candidate ranking; its five physical runs can still differ because each camera observation and physical setup is a new state. Existing runs with omitted `--shuffle-seed` resolved internally to seed 42 and remain valid pilot evidence, but they are outside the formal five-seed timing set.

## Documentation changes

`real_exp/README.md` will add the frozen seed schedule, the model-versus-uniform seed interpretation, the pilot status of existing seed-42 runs, the required command-line flag, and the warmup rule. `real_exp/METRICS.md` will define the new warmup fields and make explicit that warmup is excluded from reported planning wall time.

## Tests

Robot-control tests will prove that model-prior warmup runs exactly once, runs before the measured clock starts, is absent for uniform search, appears separately in per-plan diagnostics, and aggregates into the run summary without changing `wall_time_ms_total`.

NAMO tests will prove that the service resolves the process-cached scorer and calls its warmup method, and that `LiveScorer.warmup` performs the requested number of correctly shaped forward passes. Focused tests will run before repository-wide suites in both changed repositories.
