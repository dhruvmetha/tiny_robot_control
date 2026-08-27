# closed_loop_sessions

Session workspace for closed-loop real/sim evaluation runs.

> **Current checkout status:** Layout, status, migration, normalization, and
> real-push preparation helpers are available. `replan-reuse-only` and the
> reuse phase of `replan` verify a prior chain directly with the canonical
> compiled `namo_rl` binding; they do not require
> `namo.services.NAMOPlanningService`. If reuse fails, `replan` falls back to
> service-backed full search, while `replan-full-search-only` starts there.
> Those full-search paths work with the current sibling `../namo_cpp` checkout
> after sourcing its `env.robotlearning.sh`; this is current-environment
> compatibility, not a guarantee for arbitrary sibling revisions.
> Real push execution additionally requires configured camera and robot
> hardware services. Generated session contents are ignored experiment output;
> this README documents their workspace layout and is tracked as source.

This tree is intentionally separate from `real_test_envs/`:

- `real_test_envs/` stays as the canonical benchmark scene bank.
- `closed_loop_sessions/` stores per-evaluation working state, replans, and
  real execution artifacts across iterations.

The main entrypoint is:

```bash
python scripts/closed_loop_session.py --help
```

## Layout

Each env gets a mirrored path under this directory:

```text
closed_loop_sessions/
  1push/1hop/multi_contact/
    sessions/
      <session_name>/
        session_meta.json
        migration_from_bootstrap.json
        start_state/
          env.xml
          mid_obs.jsonl
          env.png
          scene.jpg
        primitive_run1/
          run_meta.json
          iter_001/
            scene_before/
            sim_candidates/
              candidate1/
            real_push/
            scene_after/
            selected_plan.json
            selected_trial_spec.yaml
            launch_real_push.sh
            status.json
          iter_002/
            ...
        random_rollout_run1/
          ...
```

### Mental model

- One env session corresponds to one benchmark env, for example
  `real_test_envs/1push/1hop/env1`.
- `start_state/` is the canonical initial snapshot for that env.
- `primitive_run1..3` are independent closed-loop experiments that start from
  `start_state/` and replan with strategy `primitive`.
- `random_rollout_run1..3` are independent closed-loop experiments seeded from
  the original successful sim runs in
  `real_test_envs/.../solution/random_rollout/` when available.
- `iter_001`, `iter_002`, ... are the successive real/sim replanning steps
  inside one seeded run.
- `iter_K/scene_after/` becomes `iter_{K+1}/scene_before/` when the goal was
  neither physically reached nor made planner-wavefront-reachable in reality.

### Region-opening control contract

The model-guided full NAMO integration must use two nested closed-loop
replanning levels. This is the required control contract, not the behavior of
the current planner bridge yet.

The outer full-NAMO loop uses the latest observed scene to build the region
connectivity graph, finds a path from the robot region to the final mission-goal
region, and selects only the immediate next region, `path[1]`. The rest of that
region path is advisory and must not be committed because moving the blocking
object can change the graph, its regions, and the best path.

Selecting `path[1]` starts one region-opening subproblem. The subproblem keeps
the following identity fixed until it is solved:

- sampled points from the selected target region
- the selected blocking object
- failure feedback accumulated while opening that boundary

The sampled points, rather than the region label, identify the target across
physical pushes. Region labels may change when the scene changes. The points
must be sampled once at the start of the subproblem and must not be resampled
after setup pushes.

The inner region-opening loop then repeats:

1. Rebuild the planning scene from the latest physical observation.
2. Solve the same region-opening subproblem with the fixed target samples and
   blocking object.
3. Require the solver to return its winning local push chain, the simulated
   post-solution state, search statistics, and failure status.
4. Execute only the first push of that local chain on the physical robot.
5. Observe the changed scene and test whether the fixed target samples are now
   reachable using the region-opening success criterion.

A one-push opening completes this inner loop immediately. For a two-push
opening, the first setup push does not return control to global region-path
selection: the next planning call remains conditioned on the same target
samples and blocker. It may produce a new local push chain from the observed
post-push scene; the unexecuted suffix of the previous simulated chain is not a
physical commitment.

Only after the target region has been opened does the system discard the active
subproblem, return to the outer loop, and recompute the entire region graph and
region path from the changed scene.

The current `NAMOPlanner` MPC behavior replans at the top level after every
physical push and does not persist this active region-opening state. Therefore,
returning the best-first search's internal `it["plan"]` is necessary but not
sufficient: the integration must also preserve the target samples and blocker
across inner-loop replans.

### Directory meanings

- `start_state/`
  Canonical initial scene for the env. This never changes after migration.
- `primitive_runN/`
  One independent closed-loop execution using replanning strategy `primitive`.
- `random_rollout_runN/`
  One independent closed-loop execution using replanning strategy
  `random_rollout`.
- `scene_before/`
  The exact scene snapshot used to plan the current iteration.
- `sim_candidates/`
  Sim-planning outputs for this iteration. `iter_001` imports the seed as
  `candidate1`; later iterations also plan exactly one `candidate1`.
- `real_push/`
  Diagnostics output from `execute_real_push.py` for the selected first push.
- `scene_after/`
  Regenerated scene bundle derived from `real_push/scene_after.json`.

## Typical workflow

### 1. One-time migration

The original bootstrap sessions used an older `iter_000` layout. Convert them
once with:

```bash
python scripts/closed_loop_session.py migrate-bootstrap --all-under-root
```

Or migrate a single env session:

```bash
python scripts/closed_loop_session.py migrate-bootstrap \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26
```

After migration:

- `start_state/` is created from the old `iter_000/scene_before/`
- each successful bootstrap seed is copied under its preserved source
  directory name at
  `<preserved-run-name>/iter_001/sim_candidates/candidate1/`
- the old `iter_000/` tree is removed

### 1b. Normalize runs by strategy

This separate step applies the strategy-based run names and converts a session
to the current six-run layout:

> **DANGER — destructive operation:** `normalize-run-layout` deletes every
> recognized run directory containing more than one `iter_*` directory,
> including already normalized `primitive_runN` and `random_rollout_runN`
> directories with execution progress. It also deletes every recognized run
> name outside the desired six. Make a backup and run normalization only
> immediately after migration, before any execution or progress. Do not run it
> on a progressed session. Do not use `--all-under-root` unless every targeted
> session is freshly migrated, unprogressed, and backed up.

```bash
python scripts/closed_loop_session.py normalize-run-layout --all-under-root
```

Or normalize a single env session:

```bash
python scripts/closed_loop_session.py normalize-run-layout \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26
```

After normalization:

- legacy `runN` folders are renamed to `random_rollout_runN` when they are
  still single-iteration seeds
- any legacy run with more than one iteration is removed
- missing `primitive_run1..3` and `random_rollout_run1..3` folders are created
- `primitive_run*` is created empty and waits for replanning
- `random_rollout_run*` is recreated from
  `real_test_envs/.../solution/random_rollout/runN` when that seed exists

### 2. Inspect a session

```bash
python scripts/closed_loop_session.py status \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26
```

To inspect one seeded run only:

```bash
python scripts/closed_loop_session.py status \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26 \
  --run random_rollout_run1
```

### 3. Prepare the first real push for one iteration

This reads `sim_candidates/candidate1/solution.yaml`, extracts the first push,
maps the sim object ID back to the real `obj_N`, and writes:

- `selected_plan.json`
- `selected_trial_spec.yaml`
- `launch_real_push.sh`

Example:

```bash
python scripts/closed_loop_session.py prepare-real-push \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26 \
  --run random_rollout_run1 \
  --iteration 1
```

### 4. Execute the prepared real push

The wrapper does not drive the robot automatically. It generates a launch
script so the operator can explicitly run the push:

> **WARNING — reruns replace diagnostics:** The generated launcher always
> passes `--allow-overwrite`. `execute_real_push.py` passes
> `<iteration-dir>/real_push/` to `DiagnosticsRecorder`, which recursively
> deletes that existing directory before recreating it. This happens during
> diagnostics bootstrap, before the trial spec is validated or robot execution
> begins. Copy or move any real-push run that must be retained before rerunning
> `launch_real_push.sh`; the flag does not merge with the prior run.

```bash
cd robot_control
bash closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26/random_rollout_run1/iter_001/launch_real_push.sh
```

`launch_real_push.sh` calls `execute_real_push.py` with:

- `--config <robot-control-root>/config/real.yaml`
- `--camera-service tcp://localhost:5556`
- `--trial-spec <iteration-dir>/selected_trial_spec.yaml`
- `--diag-path <iteration-dir>`
- `--run-name real_push`
- `--headless`
- `--capture-scene`
- `--record-video`
- `--nav-speed 0.4`
- `--push-speed 0.4`
- `--no-reset-check`
- `--allow-overwrite`

The two `0.4` overrides are phase-specific controller caps, not a global
wheel-command or hardware ceiling. `--nav-speed` applies to navigation path
following used during approach and navigation-based retreat; it does not
replace the navigation rotation speeds. `--push-speed` applies to the push
path follower during `PUSHING`; `ADVANCING` and reverse retreat use their own
push settings. See [Navigation and wheel commands](../docs/NAVIGATION_AND_WHEEL_COMMANDS.md)
and [Push duration calibration](../docs/PUSH_DURATION_CALIBRATION.md) for the
full command and tick semantics.

### 5. Advance the iteration from the real result

> **Scene integrity warning:** Do not move the physical robot, objects, or
> workspace after the push until both post-push artifacts are captured and
> `advance-iteration` completes. Recovery observes the current physical scene,
> not the scene at the time the push ended.

`execute_real_push.py` normally writes `real_push/scene_after.json` and
`real_push/scene_after.jpg`. To recover them explicitly, keep the configured
camera service running and the scene unchanged, then run:

```bash
python scripts/closed_loop_session.py recover-scene-after \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26 \
  --run random_rollout_run1 \
  --iteration 1 \
  --allow-overwrite
```

Recovery reads the camera-service address from the real-push diagnostics and
captures both artifacts from the live scene. Without `--allow-overwrite`, it
refuses to run if either artifact already exists; with the flag, it replaces
both.

If either artifact is missing when `advance-iteration` starts, the helper
automatically invokes this live recovery with overwrite enabled. It can
therefore replace the remaining artifact with data from the current physical
scene. Ensure the camera service is running and the scene is unchanged before
advancing or retrying.

After the artifacts are complete, advance the session:

```bash
python scripts/closed_loop_session.py advance-iteration \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26 \
  --run random_rollout_run1 \
  --iteration 1
```

This reads:

- `real_push/scene_after.json`
- `real_push/scene_after.jpg`

and regenerates:

- `scene_after/env.xml`
- `scene_after/env.png`
- `scene_after/mid_obs.jsonl`
- `scene_after/scene.jpg`

If the robot is still farther than `5.0 cm` from the mission goal and the
planner-side 1x car wavefront still cannot reach the goal from the robot's
post-push pose, the helper creates `iter_002/scene_before/` as a copy of
`iter_001/scene_after/`.

### 6. Replan the next iteration

For an iteration awaiting replanning, generate exactly one new sim candidate:

```bash
cd robot_control
python scripts/closed_loop_session.py replan \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26 \
  --run primitive_run1 \
  --iteration 2
```

`replan` is reuse-first. Given a prior `selected_plan.json` and the current
`scene_before/`, it uses `NAMOPlanBridge.verify_chain()` with the canonical
compiled `namo_rl` binding. It first verifies the remaining suffix of a
multi-action plan. If that suffix fails at its first step, it also checks the
full prior plan; a one-action plan is checked in full. Successful reuse writes
`sim_candidates/candidate1` without importing `NAMOPlanningService`.

If no prior plan can be reused, `replan` falls back to full search. The two
explicit modes are:

```bash
# Verify prior-plan reuse and stop; never enter full search.
python scripts/closed_loop_session.py replan-reuse-only \
  --session-dir <session-dir> --run <run-name> --iteration <N>

# Skip reuse and enter full search directly.
python scripts/closed_loop_session.py replan-full-search-only \
  --session-dir <session-dir> --run <run-name> --iteration <N>
```

`replan-reuse-only` requires a prior plan/scene and the canonical compiled
`namo_rl` binding, but not `NAMOPlanningService`. If reuse does not succeed, it
sets the iteration state to `awaiting_remote_replan`, reports
`needs_remote_search`, and stops.

The fallback used by `replan`, and the direct path used by
`replan-full-search-only`, invoke this command pattern:

```bash
python scripts/run_namo.py --sim \
  --sim-xml <iter>/scene_before/env.xml \
  --sim-real-run-dir <iter>/scene_before \
  --namo-config <car-1x-config> \
  --strategy <primitive|random_rollout> \
  --shuffle-seed <run-seed> \
  --diag-path <iter>/sim_candidates \
  --run-name candidate1 \
  --allow-overwrite
```

The strategy comes from the run name:

- `primitive_run*` -> `--strategy primitive`
- `random_rollout_run*` -> `--strategy random_rollout` and
  `--rollout-samples-per-state 36000`

Full search retries up to `MAX_REPLAN_ATTEMPTS = 8` and imports the in-process
Python class `namo.services.NAMOPlanningService`. With the current sibling
checkout and `env.robotlearning.sh` sourced, that class and the compiled
`namo_rl` binding resolve and the full-search path is available. The
2026-08-27 verification was simulation-only; real closed-loop execution still
requires its camera and robot services.

Closed-loop replans keep the sim candidate as-is, but if the first push has
`push_steps == 1`, `prepare-real-push` floors only the real execution spec to
`push_steps = 2`. The sim plan remains unchanged; the override is recorded in
`selected_plan.json` and `status.json`.

Then repeat:

1. `prepare-real-push`
2. run `launch_real_push.sh`
3. `advance-iteration`
4. `replan` if the goal was not reached

## Important behavior

- The wrapper is additive. It does not modify `run_namo.py`,
  `execute_real_push.py`, `runtime.py`, or `real_test_envs/`.
- Seed numbering is preserved for random-rollout imports. Original
  `random_rollout/run3` becomes closed-loop `random_rollout_run3`.
- Every normalized session has six run folders:
  - `primitive_run1..3`
  - `random_rollout_run1..3`
- `primitive_runN` starts from `start_state/` and waits for a `primitive`
  replan.
- `random_rollout_runN` is seeded from the matching successful
  `real_test_envs/.../solution/random_rollout/runN` when available; otherwise
  it is created empty and waits for replanning.
- `advance-iteration` intentionally does not use `real_push/scene_after.xml`.
  It regenerates `env.xml` from `real_push/scene_after.json`, because
  `execute_real_push.py` does not produce usable XML on this path.
- The canonical mission goal is stored once in `session_meta.json`, sourced
  from `start_state/mid_obs.jsonl` when available and otherwise from the
  `<site name="goal">` in `start_state/env.xml`.
- `advance-iteration` marks an iteration successful if either:
  - the robot is within `5.0 cm` of the canonical goal, or
  - the planner-side unified wavefront check (`RLEnvironment.is_robot_goal_reachable()`
    using `namo_config_complete_skill15_car_1x.yaml`) says the goal is
    reachable from the robot's post-push pose.
- If a real push fails with `Approach position unreachable`, the wrapper
  treats that as a same-iteration replan case:
  - it does not create `iter_{K+1}`
  - it leaves the run at `iter_K`
  - the next `replan` should be run against that same `iter_K/scene_before/`
- `launch_real_push.sh` uses `PYTHON_BIN` if set; otherwise it falls back to
  `python`.

## Useful paths

- Session root:
  `closed_loop_sessions/<push>/<hop>/<env>/sessions/bootstrap_from_real_test_envs_2026-05-26/`
- Shared start state:
  `.../start_state/`
- First primitive run:
  `.../primitive_run1/`
- First random-rollout run:
  `.../random_rollout_run1/`
- First iteration status:
  `.../random_rollout_run1/iter_001/status.json`
- Generated real-push launcher:
  `.../random_rollout_run1/iter_001/launch_real_push.sh`

## Notes

- Session data should be created under `.../sessions/<session_name>/` so
  multiple evaluations of the same env do not collide.
