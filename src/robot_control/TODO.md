# Current gaps

This file retains its historical name for compatibility. It tracks verified
gaps in the current checkout; detailed, reproducible bugs belong in
[`KNOWN_ISSUES.md`](KNOWN_ISSUES.md).

## Dependency and integration blockers

- Restore `namo.services.NAMOPlanningService` in the sibling `namo_cpp`
  package, or migrate `NAMOPlanBridge` to a supported replacement API, before
  NAMO-backed entrypoints can run. The current failure is a missing in-process
  Python class, not an unavailable network service.
- Publish the existing two-line RVG C++17 compatibility change
  (`[[nodiscard("...")]]` to `[[nodiscard]]` in `VisibilityGraph/Edge.h` and
  `VisibilityGraph/Vertex.h`) from an owned fork or an accepted upstream
  commit before updating the `controller/motion_planner/rvg` submodule pointer.

## Runtime and controller gaps

- The runtime GUI can fail to auto-quit after autonomous completion. Keep the
  symptom, reproduction, and proposed repair in the
  [known-issue entry](KNOWN_ISSUES.md#1-runtime-auto-quit-on-quit_on_completetrue-is-racy)
  rather than duplicating its analysis here.
- The built-in `SimEnv` advances only the robot's differential-drive pose.
  Object poses remain the static values from `SimConfig`, and the environment
  does not implement collision or push dynamics.

## Test coverage gaps

- Automated coverage is limited to the current [`tests/`](../../tests) suite.
  It does not exercise the unavailable in-process NAMO API boundary or a
  real-hardware workflow end to end.
- Bare `pytest` also collects vendored RVG tests and fails when their Python
  bindings are unavailable. Use `python -m pytest tests` until pytest discovery
  is scoped to the repository's own test directory.
