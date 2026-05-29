# closed_loop_sessions

Session workspace for closed-loop real/sim evaluation runs.

This tree is intentionally separate from `real_test_envs/`:

- `real_test_envs/` stays as the canonical benchmark scene bank.
- `closed_loop_sessions/` stores per-evaluation working state, replans, and
  real execution artifacts across iterations.

The main entrypoint is:

```bash
python scripts/closed_loop_session.py --help
```

For a short copy/paste operator guide, see
[MANUAL_LOOP_RUNBOOK.md](./MANUAL_LOOP_RUNBOOK.md).

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
- successful bootstrap seeds are copied into
  `random_rollout_runN/iter_001/sim_candidates/candidate1/`
- the old `iter_000/` tree is removed

### 1b. Normalize runs by strategy

To convert a session to the current six-run layout:

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

```bash
cd robot_control
PYTHON_BIN=/home/dhruv/miniconda3/envs/namo312/bin/python \
  closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26/random_rollout_run1/iter_001/launch_real_push.sh
```

`launch_real_push.sh` calls `execute_real_push.py` with:

- `--trial-spec selected_trial_spec.yaml`
- `--diag-path <iter dir>`
- `--run-name real_push`
- `--capture-scene`
- `--record-video`
- `--no-reset-check`

### 5. Advance the iteration from the real result

After `execute_real_push.py` finishes, advance the session:

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

When `iter_002` exists, generate exactly one new sim candidate:

```bash
cd robot_control
PYTHON_BIN=/home/dhruv/miniconda3/envs/namo312/bin/python \
python scripts/closed_loop_session.py replan \
  --session-dir closed_loop_sessions/1push/1hop/env1/sessions/bootstrap_from_real_test_envs_2026-05-26 \
  --run primitive_run1 \
  --iteration 2
```

This shells out to:

```bash
python scripts/run_namo.py --sim \
  --sim-xml <iter>/scene_before/env.xml \
  --sim-real-run-dir <iter>/scene_before \
  --strategy random_rollout \
  --rollout-samples-per-state 36000 \
  --diag-path <iter>/sim_candidates \
  --run-name candidate1 \
  --allow-overwrite
```

The strategy depends on the run:

- `primitive_run*` -> `--strategy primitive`
- `random_rollout_run*` -> `--strategy random_rollout`

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
  `execute_real_push.py` does not currently produce usable XML on this path.
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

- `_session_template/` mirrors the new layout with a shared `start_state/`
  plus a per-seed `run_template/iter_001/` scaffold.
- Session data should be created under `.../sessions/<session_name>/` so
  multiple evaluations of the same env do not collide.
- The currently running `random_rollout` backfill under `real_test_envs/`
  is intentionally untouched by this directory layout.
- The pointer files under `real_test_envs/.../solution/closed_loop_session_pointer.txt`
  point from the old env folders into the new closed-loop session layout.
