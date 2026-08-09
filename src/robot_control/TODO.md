# Current gaps

This file retains its historical name for compatibility. It tracks verified
gaps in the current checkout; detailed, reproducible bugs belong in
[`KNOWN_ISSUES.md`](KNOWN_ISSUES.md).

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
