# Wavefront unification follow-ups

This is a current-checkout follow-up record for the two wavefront
implementations. The trapped-start behavior is mirrored, but the engines and
their configuration are not a single ground truth.

## Current snapshot

| Engine | Source | Current caller boundary |
|---|---|---|
| C++ `WavefrontPlanner` | `namo_cpp/src/wavefront/wavefront_planner.cpp` | The sibling `namo_rl` binding exposes reachability summaries. `NAMOPlanner._is_goal_reachable` is wired through `NAMOPlanBridge.analyze_reachability`; with the current sibling checkout and its environment sourced, the adapter resolves the in-process `namo.services.NAMOPlanningService` class. |
| Python `WavefrontPlanner` | [`utils/wavefront.py`](../src/robot_control/utils/wavefront.py) | [`WavefrontPathPlanner.plan`](../src/robot_control/planner/wavefront_path_planner.py) produces paths consumed by `NavigationController`. |

The implementations use separate grids and search code. Python uses Dijkstra
with optional obstacle-proximity cost shaping and gradient descent. C++ uses
its own reachability BFS; `WavefrontPlanner::extract_path` exists on the C++
class but is not exposed as the shared reachability-plus-path result described
below.

## What is and is not aligned

Two policies are intentionally mirrored in code:

- Effective robot radius uses `max(hx, hy)`. The Python source of truth is
  [`effective_robot_radius_cm`](../src/robot_control/utils/robot_geometry.py);
  C++ mirrors it in
  `namo_cpp/include/wavefront/goal_tolerance_utils.hpp` at
  `compute_rotation_safe_robot_radius_m`.
- Trapped-start recovery checks the blocked start's neighbors, clears only the
  start when an exit exists, and clears a 5x5 block when fully trapped. C++
  force-enqueues the original start; Python makes that cell traversable and
  plans from the original coordinate. Python implements its side in
  `WavefrontPlanner.apply_trapped_start_recovery`; C++ mirrors the policy in
  `WavefrontPlanner::recompute_wavefront`.

Those mirrors rely on cross-references and tests, not shared executable code.
The tier-1 inflation margin is currently divergent:

| Checkout | Sidecar | Tier-1 base margin |
|---|---|---:|
| `robot_control` | [`config/wavefront_inflation.yaml`](../config/wavefront_inflation.yaml) | `0.002 m` |
| sibling `namo_cpp` | `namo_cpp/config/wavefront_inflation.yaml` | `0.005 m` |

These are separate files, and configuration discovery depends on the primary
config's location. Python's
[`wavefront_inflation_config.py`](../src/robot_control/utils/wavefront_inflation_config.py)
loads the `robot_control` sidecar directly. For a direct `namo_cpp` run whose
primary config remains inside the sibling checkout,
`ConfigManager::load_wavefront_inflation_config` can find the sibling sidecar
beside the primary config or by walking its ancestors.

The normal [`NAMOPlanBridge`](../src/robot_control/planner/namo_bridge.py) path
is different. `NAMOPlanBridge._build_effective_namo_config` copies the primary
YAML, with its runtime robot-size override, to
`/tmp/namo_config_runtime_*.yaml`. C++ sidecar discovery then starts beside
that temporary file and walks `/tmp`'s ancestors, so it does not reach
`namo_cpp/config/wavefront_inflation.yaml`. On this bridge path the C++
`PlanningConfig` default remains in effect; it is `0.005 m`, as is
`kDefaultWavefrontTier1MarginM`. The sibling sidecar currently has the same
number, but the bridge is not reading it.

The sidecars also differ at `push_approach.additional_margin_m` (`0.0 m` here,
`0.003 m` in `namo_cpp`). When C++ does discover a sidecar, its current loader
reads only the tier-1 field. That second difference is not a shared runtime
setting and does not alter the bridge's fallback behavior.

The verified runtime mismatch is therefore the Python path's configured
`0.002 m` versus the bridge C++ path's `0.005 m` default. Editing only the
sibling sidecar does not align bridge behavior.

## Trapped-start regression history

The 2026-05-19 failure had opposite outcomes for the same scene: C++ reported
the goal reachable while Python returned no navigation waypoints. Python had
selected a Euclidean-nearest free cell that could belong to a disconnected
pocket. The fix removed that fallback from the runtime path and mirrored the
C++ blocked-start recovery in Python.

Current anchors for that behavior are:

- [`WavefrontPlanner.apply_trapped_start_recovery`](../src/robot_control/utils/wavefront.py)
  and its unconditional call from `WavefrontPlanner.plan`;
- [`WavefrontPathPlanner.plan`](../src/robot_control/planner/wavefront_path_planner.py),
  which applies and reports recovery for a blocked start;
- `WavefrontPlanner::recompute_wavefront` in the sibling C++ source; and
- [`tests/test_wavefront_trapped_start.py`](../tests/test_wavefront_trapped_start.py).

The historical PNG previously cited by this note is not present in this
checkout, so it is not current evidence. Use the regression test and source
symbols above instead.

## Follow-ups

### Resolve the current configuration divergence

Choose the intended tier-1 margin, then align the relevant sidecars and C++
defaults and/or propagate the intended policy through the bridge's generated
config path. Editing only the sibling sidecar is insufficient. Exercise both
engines on the same fixtures after the bridge actually consumes the selected
value. Until then, a reachability comparison can differ because obstacle
inflation differs before either search algorithm starts.

### Move toward one engine boundary

The longer-term target is one call that returns reachability and a path from
the same C++ grid snapshot:

```text
NAMO reachability caller       NavigationController
             \                    /
              WavefrontEngine wrapper
                       |
              NAMOPlanBridge binding
                       |
              C++ WavefrontPlanner
              -> reachable + path + diagnostic
```

A reviewable sequence is:

1. Keep the trapped-start regression fixture and add cross-engine coverage for
   the selected inflation margin.
2. Expose a binding that recomputes one C++ snapshot and returns both the
   reachability result and `WavefrontPlanner::extract_path` output.
3. Add a bridge method and cut both NAMO reachability and navigation over to
   that result. Preserve or deliberately migrate the current
   `NAMOPlanningService` boundary while changing the adapter path.
4. Decide whether C++ must reproduce Python's obstacle-proximity cost shaping;
   without it, valid paths may hug inflation boundaries more closely.
5. Remove the duplicate Python runtime search only after checking every caller
   of [`utils/wavefront.py`](../src/robot_control/utils/wavefront.py), including
   non-runtime helpers and tests.

## Reproducible source checks

Run from the `robot_control` repository root in the NAMO workspace:

```bash
grep -n "base_inflation_margin_m" \
  config/wavefront_inflation.yaml \
  ../namo_cpp/config/wavefront_inflation.yaml
grep -n 'prefix="namo_config_runtime_"' \
  src/robot_control/planner/namo_bridge.py
grep -n "primary_path.parent_path" \
  ../namo_cpp/src/config/config_manager.cpp
grep -n "wavefront_tier1_inflation_margin = 0.005" \
  ../namo_cpp/include/config/config_manager.hpp

grep -n "def apply_trapped_start_recovery" \
  src/robot_control/utils/wavefront.py
grep -n "WavefrontPlanner::recompute_wavefront" \
  ../namo_cpp/src/wavefront/wavefront_planner.cpp
grep -n "WavefrontPlanner::extract_path" \
  ../namo_cpp/src/wavefront/wavefront_planner.cpp

PYTHONPATH=src python -m pytest tests/test_wavefront_trapped_start.py -q
```
