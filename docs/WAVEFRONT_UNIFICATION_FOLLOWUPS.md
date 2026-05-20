# Wavefront unification — follow-up notes

Status: **not priority #1**. Captured here so we don't lose the context
behind the 2026-05-19 trapped-start fix and the deferred bigger cleanup.

## Where we are today

There are still **two wavefront engines** in the runtime:

| Engine | File | Used by |
|---|---|---|
| C++ `WavefrontPlanner` | `namo_cpp/src/wavefront/wavefront_planner.cpp` | `NAMOPlanner._is_goal_reachable` (via `NAMOPlanBridge.analyze_reachability` → `namo_rl` pybind11 binding) |
| Python `WavefrontPlanner` | `robot_control/src/robot_control/utils/wavefront.py` | `WavefrontPathPlanner.plan` (path output consumed by `NavigationController.navigate_to`) |

They are independent implementations: separate grids, separate
BFS/Dijkstra, separate file. What's "unified" between them is a set
of **policies kept in sync by docstring cross-references**:

| Policy | Python | C++ | Sync mechanism |
|---|---|---|---|
| Effective robot inflation radius (`max(hx, hy)`) | `robot_geometry.py:effective_robot_radius_cm` | `goal_tolerance_utils.hpp:compute_rotation_safe_robot_radius_m` | Doc-comment in `robot_geometry.py:51-55` says "keep these in sync" |
| Tier-1 inflation margin default (0.005 m) | `wavefront_inflation_config.py` | `goal_tolerance_utils.hpp:kDefaultWavefrontTier1MarginM` | Both ends read from the same YAML at runtime |
| Trapped-start recovery (dilate + force-enqueue) | `wavefront.py:apply_trapped_start_recovery` *(added 2026-05-19)* | `wavefront_planner.cpp:recompute_wavefront:303-380` | Cross-reference comments in both files |

The 2026-05-19 incident added the third row. Each new row is a debt
payment toward the eventual "single ground truth" refactor.

## What was done on 2026-05-19

Bug: `NAMOPlanner._is_goal_reachable` (C++ engine) reported "Goal
REACHABLE" but `WavefrontPathPlanner.plan` (Python engine) returned
zero waypoints on the same scene. Root cause: when the robot's start
cell was an inflated obstacle, the Python side picked the
Euclidean-nearest free cell, which could land in a disconnected
pocket. C++ instead force-enqueued the blocked cell and let the BFS
escape through any free neighbor.

Fix: ported the C++ policy to Python (the third row in the table
above). See:
- `robot_control/src/robot_control/utils/wavefront.py` —
  `apply_trapped_start_recovery`. Called unconditionally at the top of
  `plan`, so the engine entry point matches `recompute_wavefront`.
- `robot_control/src/robot_control/planner/wavefront_path_planner.py`
  — removed the misleading `find_nearest_free` fallback; replaced the
  catch-all "both free but no path" message with proper branches.
- `namo_cpp/src/wavefront/wavefront_planner.cpp` — back-reference
  comment so a future C++ edit notices the Python mirror.
- `robot_control/tests/test_wavefront_trapped_start.py` — 5 tests
  including the regression case.

## Why we deferred the bigger refactor

The "right" fix is to make C++ the **single ground truth** for
reachability AND path extraction, and have Python's
`WavefrontPathPlanner` delegate via the existing `namo_rl` pybind11
binding. Then the Python `utils/wavefront.WavefrontPlanner` class
disappears for runtime use, and we stop accumulating cross-reference
rows.

We didn't do this on 2026-05-19 because:
1. The bug was a production blocker — needed a small, reviewable fix.
2. The refactor crosses two repos (`namo_cpp` bindings + `robot_control`
   wrapper) and changes runtime call patterns.
3. We didn't want to invalidate field test data on real hardware.

## Trigger for revisiting

Do the single-engine refactor when **any** of these happens:

- A fourth row is added to the policy-sync table (a new "keep these
  formulas matched" docstring shows up).
- A divergence-style bug recurs (different reachability answer from
  the two engines on the same input).
- We need a wavefront feature in Python that already exists in C++ —
  the temptation will be to copy more code over. Don't.

## Refactor sketch (so it's not lost)

Architecture target:

```
NAMOPlanner._is_goal_reachable      NavigationController.navigate_to
            \                                   /
             \                                 /
              \                               /
               \                             /
              WavefrontEngine  (Python wrapper)
              - plan(obs, goal) -> WavefrontPlanResult
                 (reachable, path_cm, diagnostic)
                              |
                              |
              NAMOPlanBridge.compute_wavefront_plan
                              |
                              | pybind11
                              |
              namo_rl.RLEnvironment::compute_wavefront_plan (new)
                              |
                              |
              C++ WavefrontPlanner — single ground truth
              - recompute_wavefront() runs trapped-start + BFS
              - extract_path() uses the same dynamic_grid_
```

Key idea: **one C++ call returns both `reachable: bool` AND
`path: List[(x,y)]` from the same snapshot.** That makes it
impossible for the two pieces of information to disagree, which is
exactly the failure mode we just fixed.

### Stages

1. **Regression test first** — replay the 2026-05-19 scenario via a
   captured XML. Must fail on the day-of-refactor `main` and pass
   after.
2. **Expose path extraction from C++ via the existing binding** —
   `WavefrontPlanner::extract_path` already exists
   (`namo_cpp/src/wavefront/wavefront_planner.cpp:557-569`); bind it
   on `RLEnvironment` as a single `compute_wavefront_plan(goal_x,
   goal_y) -> dict` returning `{reachable, path, diagnostic}`.
3. **Bridge method** — `NAMOPlanBridge.compute_wavefront_plan(obs,
   goal_cm)`. Reuses `_generate_xml`, mirrors `analyze_reachability`.
4. **`WavefrontEngine` Python class** — thin wrapper over the bridge.
   One class, one implementation, no Protocol. Docstring says "do not
   write a second implementation; see CLAUDE.md and these notes."
5. **Cut over callers** — `NAMOPlanner._is_goal_reachable` and
   `NavigationController.navigate_to` both consume the engine.
   `WavefrontPathPlanner` deleted.
6. **Delete duplicate** — remove the now-unused parts of
   `utils/wavefront.py`. `find_nearest_free` may still have other
   callers (e.g., `xml_generator.py:529`); grep before deletion.
7. **(Optional) cross-validation guard** — env-flag-gated debug mode
   that calls both engines and asserts they agree. CI-only for a
   release cycle to catch unexpected behavior drift.

### Known open questions for the refactor

1. **Cost shaping for navigation paths.** Today's Python planner adds
   a proximity cost that biases paths away from obstacle edges
   (`utils/wavefront.py:_build_cost_grid`). C++ does plain BFS, so its
   `extract_path` will hug inflation boundaries more tightly. If we
   want the existing aesthetic, the C++ engine needs a similar cost
   mode. Check real-hardware paths after cutover.
2. **Per-call XML serialization overhead.** Today's
   `analyze_reachability` writes a temp XML on every call. Path
   planning happens once per `navigate_to` (not per tick) so the
   cost is acceptable, but if the integration shows up as a hot path
   we should keep one `RLEnvironment` alive and mutate it in place
   between calls.
3. **Obstacles argument removal.** `NavigationController.navigate_to`
   today takes an explicit obstacle list. After cutover the engine
   derives obstacles from the observation. Grep for callers passing
   a curated/filtered obstacle list before removing the parameter.
4. **Debug image dumps.**
   `WavefrontPathPlanner._save_debug_image` writes the PNGs in
   `output_dump/wavefront/`. The refactored engine needs an
   equivalent — either a new binding method that dumps the C++ grid,
   or wire `NAMO_QPOS_DUMP` / `NAMO_NAV_LOG` through (those env vars
   already exist on the C++ side).
5. **Other `utils/wavefront.py` callers.** Keep the file for non-
   runtime helpers (`find_nearest_free` in `xml_generator.py`,
   visualization, tests) — just remove the `plan` / `Dijkstra` /
   gradient-descent code paths once nothing imports them.

## References

- Incident discussion / debug image: `output_dump/wavefront/wavefront_20260519_162214_002.png`
- Regression tests: `robot_control/tests/test_wavefront_trapped_start.py`
- C++ trapped-start policy: `namo_cpp/src/wavefront/wavefront_planner.cpp:303-380`
- Python trapped-start policy: `robot_control/src/robot_control/utils/wavefront.py:apply_trapped_start_recovery`
- Single-engine binding entry point (today): `namo_cpp/python/namo/cpp_bindings/bindings.cpp:88-113` (`get_reachability_summary`)
- C++ path extraction (ready to bind): `namo_cpp/src/wavefront/wavefront_planner.cpp:557-569`
