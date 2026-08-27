# Current gaps

This file retains its historical name for compatibility. It tracks verified
gaps in the current checkout; detailed, reproducible bugs belong in
[`KNOWN_ISSUES.md`](KNOWN_ISSUES.md).

## ICRA table campaign (deadline 2026-09-15)

The ordered work list for the study. Software is done and pushed through
74dcf01; everything below needs the table, except where marked.

1. [ ] Calibration ladder (~15 min). obj_1 alone, +y-face offsets 1.5/2.0/2.5
   cm x2 plus one 3.5 anchor, 5 steps, fresh battery, via
   `execute_real_push.py`. Deliverable is ONE NUMBER: the offset where the
   block stops re-seating and starts accumulating rotation. Pre-registered
   prediction 2.0-2.5 cm (the car's 7 cm pusher face). Write it into
   ICRA_REAL_ROBOT_STUDY.md; it is the key for reading matrix failures.
2. [ ] Corridor floor (~3 scenes). 1push/hard_099, hmax2/hard_054 (8.36 cm),
   hmax2/hard_010 (8.76), spare 1push/hard_025. Pins the car's real pass/fail
   gap width; honest expected range 8.4-11 cm against the sim's unverified
   8.0. No matrix scene is at risk anywhere in that range (best corridor
   >= 13.74), so this reads results, it does not gate them.
3. [ ] The matrix, 56 runs. 14 scenes x {model,uniform} x {search,reactive},
   rebuild + checksum between cells, randomize order within a scene, both
   flags per the runbook command. Cut whole scenes only, easy then medium;
   floor is 8 hard scenes fully crossed (32 runs). Row into trials.csv after
   every trial, then `check_trials_consistency.py` on the spot.
4. [ ] (no table) Fix the two-hop test checkpoint path
   (KNOWN_ISSUES.md#two-hop-tests-collect-on-one-machine-only). Before the
   ladder if convenient, else first thing after the matrix.
5. [x] (no table) namo_cpp sim experiment pipeline pushed 2026-08-27 as
   `630a3f4` (edab269 rebased over seven sim-side commits, no overlap).
6. [ ] (after matrix) Recorder dedup in namo_planner.py (ten keys built three
   times) with the drift note, and move `_require_paired_namo_cpp` into
   tests/conftest.py. Both deferred deliberately: they change the instrument.

## Dependency and integration blockers

- Restore `namo.services.NAMOPlanningService` in the sibling `namo_cpp`
  package, or migrate `NAMOPlanBridge` to a supported replacement API, before
  fresh/full-search entrypoints and fallbacks can run. Prior-chain verification
  and reuse use the canonical compiled binding directly and do not require this
  class. The current failure is a missing in-process Python API, not an
  unavailable network service.
- Publish the existing two-line RVG C++17 compatibility change
  (`[[nodiscard("...")]]` to `[[nodiscard]]` in `VisibilityGraph/Edge.h` and
  `VisibilityGraph/Vertex.h`) from an owned fork or an accepted upstream
  commit before updating the `controller/motion_planner/rvg` submodule pointer.

## Runtime gaps

- Revalidate and resolve the post-`570407b` intermittent GUI auto-quit and
  run-end finalization path. Keep the evidence and reproduction requirements
  in the [current issue entry](KNOWN_ISSUES.md#runtime-auto-quit-revalidation).
- The built-in `SimEnv` advances only the robot's differential-drive pose.
  Object poses remain the static values from `SimConfig`, and the environment
  does not implement collision or push dynamics.

## Test coverage gaps

- Automated coverage is limited to the current [`tests/`](../../tests) suite.
  It does not exercise the unavailable full-search service boundary or a
  real-hardware workflow end to end.
- Bare `pytest` also collects vendored RVG tests and fails when their Python
  bindings are unavailable. Use `python -m pytest tests` until pytest discovery
  is scoped to the repository's own test directory.
