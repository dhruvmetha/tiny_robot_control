# Manual Closed-Loop Runbook

Copy/paste guide for the common closed-loop session scenarios.

## Setup

Run from the repo root:

```bash
cd /home/dhruv/projects_dhruv/namo

export PY=/home/dhruv/miniconda3/envs/namo312/bin/python
export PYTHONPATH=/home/dhruv/projects_dhruv/namo/robot_control/src
export QT_QPA_PLATFORM=offscreen
```

Set the session and run you want:

```bash
export SESSION=/home/dhruv/projects_dhruv/namo/robot_control/closed_loop_sessions/1push/2hop/env2/sessions/bootstrap_from_real_test_envs_2026-05-26
export RUN=primitive_run1
```

Available run names per session:

- `primitive_run1`, `primitive_run2`, `primitive_run3`
- `random_rollout_run1`, `random_rollout_run2`, `random_rollout_run3`

## Quick Sanity Check

Before running any closed-loop command, verify that `SESSION` points at the
actual session root and is not empty:

```bash
printf '<%s>\n' "$SESSION"
test -f "$SESSION/session_meta.json" && echo SESSION_OK || echo SESSION_BAD
```

If `SESSION` is empty, `--session-dir "$SESSION"` resolves to the current
working directory and the helper will look for `session_meta.json` in the
wrong place.

Before trying to execute a run, also check whether the iteration is actually
prepared or still empty:

```bash
export ITER=1
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/status.json"
```

Interpretation:

- `state = "real_push_prepared"`: execute `launch_real_push.sh`
- `state = "planned"`: run `prepare-real-push` first
- `state = "awaiting_replan"`: run `replan` first

## Scenario 1: Rebuild All Runs After Wiping Them

This recreates the six run folders for every session:

```bash
$PY robot_control/scripts/closed_loop_session.py normalize-run-layout --all-under-root
```

Important behavior:

- `primitive_run*`
  - recreated empty
  - `iter_001/state = awaiting_replan`
  - no `candidate1`
- `random_rollout_run*`
  - recreated from `real_test_envs/.../solution/random_rollout/runN` when that seed exists
  - otherwise created empty and `awaiting_replan`

To rebuild just one session:

```bash
$PY robot_control/scripts/closed_loop_session.py normalize-run-layout \
  --session-dir "$SESSION"
```

## Scenario 1b: Redo One Run From Scratch

If one run has bad/old data and you want to start that run over from
`start_state`, delete just that run directory and recreate the session layout:

```bash
export RUN=primitive_run2
rm -rf "$SESSION/$RUN"

$PY robot_control/scripts/closed_loop_session.py normalize-run-layout \
  --session-dir "$SESSION"
```

Then inspect the recreated run:

```bash
cat "$SESSION/$RUN/iter_001/status.json"
```

Expected result for `primitive_run*`:

- `state = "awaiting_replan"`
- no selected plan yet
- no `real_push/` artifacts yet

Important:

- recreating a `primitive_run*` does **not** restore an old `solution.yaml`
- it comes back empty and you must `replan` unless you manually restore a
  compatible candidate from somewhere else

Then start again from:

1. `replan`
2. `prepare-real-push`
3. execute `launch_real_push.sh`

## Scenario 2: Start a Fresh Primitive Run

Primitive runs do not start with a seed candidate. First do a sim search:

```bash
$PY robot_control/scripts/closed_loop_session.py replan \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration 1
```

Then prepare the first real push:

```bash
$PY robot_control/scripts/closed_loop_session.py prepare-real-push \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration 1
```

Then execute it:

```bash
QT_QPA_PLATFORM=offscreen PYTHON_BIN="$PY" \
  bash "$SESSION/$RUN/iter_001/launch_real_push.sh"
```

The generated launch script now includes `--headless` by default. That avoids
the Runtime GUI auto-quit race and makes `scene_after`/`summary.json` much
more reliable.

## Scenario 3: Start a Seeded Random-Rollout Run

Random-rollout runs may already have `iter_001/sim_candidates/candidate1`.
If so, you can prepare the real push directly:

```bash
$PY robot_control/scripts/closed_loop_session.py prepare-real-push \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration 1
```

Then execute it:

```bash
QT_QPA_PLATFORM=offscreen PYTHON_BIN="$PY" \
  bash "$SESSION/$RUN/iter_001/launch_real_push.sh"
```

If the seeded candidate is missing, just use the primitive-run flow above:

1. `replan`
2. `prepare-real-push`
3. execute `launch_real_push.sh`

## Scenario 3b: Use a Plan That Is Already Prepared

If these files already exist:

- `selected_plan.json`
- `selected_trial_spec.yaml`
- `launch_real_push.sh`

then you do not need to run `prepare-real-push` again. You can execute the
prepared run directly:

```bash
QT_QPA_PLATFORM=offscreen PYTHON_BIN="$PY" \
  bash "$SESSION/$RUN/iter_001/launch_real_push.sh"
```

If you are constructing an `execute_real_push.py` command manually, include
`--headless` unless you specifically need the Runtime GUI window:

```bash
$PY robot_control/scripts/execute_real_push.py \
  --config robot_control/config/real.yaml \
  --camera-service tcp://localhost:5556 \
  --trial-spec "$SESSION/$RUN/iter_001/selected_trial_spec.yaml" \
  --diag-path "$SESSION/$RUN/iter_001" \
  --run-name real_push \
  --headless \
  --capture-scene \
  --record-video \
  --nav-speed 0.4 \
  --push-speed 0.4 \
  --no-reset-check \
  --allow-overwrite
```

Quick check:

```bash
ls "$SESSION/$RUN/iter_001"/selected_plan.json \
   "$SESSION/$RUN/iter_001"/selected_trial_spec.yaml \
   "$SESSION/$RUN/iter_001"/launch_real_push.sh
```

If that prepared run was created before the `--headless` update, regenerate
the launch script before executing:

```bash
$PY robot_control/scripts/closed_loop_session.py prepare-real-push \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration 1
```

Sanity-check that the generated script includes `--headless`:

```bash
rg -- '--headless' "$SESSION/$RUN/iter_001/launch_real_push.sh"
```

## Scenario 4: Continue After a Real Push

Set the current iteration number first:

```bash
export ITER=1
```

Recommended pattern:

- before every command block, set `ITER` explicitly
- do not rely on an old `NEXT_ITER` value still being correct from a previous shell step

After the push finishes, do not treat `[Runtime] Shutting down` as the end by
itself. Wait until the shell prompt comes back, then verify that the
post-push artifacts needed for `advance-iteration` landed:

```bash
ls "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/real_push/scene_after.json" \
   "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/real_push/scene_after.jpg"
```

Optional extra sanity check:

```bash
ls "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/real_push/summary.json"
```

`summary.json` is nice to have, but `advance-iteration` only requires
`scene_after.json` and `scene_after.jpg`.

If those files exist, advance normally:

After a real push for iteration `N` finishes:

```bash
$PY robot_control/scripts/closed_loop_session.py advance-iteration \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration "$ITER" \
  --allow-overwrite
```

If `real_push/scene_after.json` is missing, `advance-iteration` now tries to
recover `scene_after.{json,jpg}` automatically from the live
`camera_service`. This only works if:

- the physical scene has not been moved since the push
- `camera_service.py` is still running

If you want to recover explicitly first, run:

```bash
$PY robot_control/scripts/closed_loop_session.py recover-scene-after \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration "$ITER" \
  --allow-overwrite
```

Then rerun `advance-iteration`.

If the shell prompt returned but `scene_after` is still missing, the safest
recovery sequence is:

```bash
$PY robot_control/scripts/closed_loop_session.py recover-scene-after \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration "$ITER" \
  --allow-overwrite

$PY robot_control/scripts/closed_loop_session.py advance-iteration \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration "$ITER" \
  --allow-overwrite
```

Check whether that iteration terminated the run:

```bash
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/status.json"
```

If terminal, stop.

If not terminal, replan the next iteration:

```bash
export NEXT_ITER=$((ITER + 1))
```

Then:

```bash
$PY robot_control/scripts/closed_loop_session.py replan \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration "$NEXT_ITER"
```

Then prepare the next real push:

```bash
$PY robot_control/scripts/closed_loop_session.py prepare-real-push \
  --session-dir "$SESSION" \
  --run "$RUN" \
  --iteration "$NEXT_ITER"
```

Then execute it:

```bash
QT_QPA_PLATFORM=offscreen PYTHON_BIN="$PY" \
  bash "$SESSION/$RUN/iter_$(printf "%03d" "$NEXT_ITER")/launch_real_push.sh"
```

Standard loop:

1. execute `launch_real_push.sh`
2. set `ITER` to the iteration you just executed, then `advance-iteration`
3. if not terminal: set `NEXT_ITER=$((ITER + 1))`, then `replan`
4. `prepare-real-push`
5. execute next `launch_real_push.sh`

Important:

- Every helper command should start with `$PY robot_control/scripts/closed_loop_session.py ...`
- Do not type placeholder text like `...`
- Reset `ITER` after each real execution, and only set `NEXT_ITER` after you
  confirm the current iteration was not terminal
- If you are redoing a run from scratch, delete the run directory and use
  `normalize-run-layout --session-dir "$SESSION"` before replanning

## Scenario 5: Inspect What a Run Is Doing

Session-wide status:

```bash
$PY robot_control/scripts/closed_loop_session.py status \
  --session-dir "$SESSION"
```

One run only:

```bash
$PY robot_control/scripts/closed_loop_session.py status \
  --session-dir "$SESSION" \
  --run "$RUN"
```

Inspect one iteration:

```bash
export ITER=1
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/status.json"
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/sim_candidates/candidate1/solution.yaml"
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/selected_plan.json"
```

## What `replan` Does Now

The replan strategy depends on the run name:

- `primitive_run*` uses sim strategy `primitive`
- `random_rollout_run*` uses sim strategy `random_rollout`

For iteration `K`, `replan` first tries to reuse the previous committed chain
from the updated real post-push scene in `iter_K/scene_before`.

It tries, in order:

1. if the previous plan has more than one push, suffix reuse: `(a2, ..., an)`
2. if that suffix fails on step 1, full-chain retry: `(a1, ..., an)`
3. if the previous plan has only one push, direct full-chain retry: `(a1)`
4. if reuse fails, fallback to the run's base strategy

You can check which path was used in:

```bash
export ITER=2
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/status.json"
cat "$SESSION/$RUN/iter_$(printf "%03d" "$ITER")/sim_candidates/candidate1/solution.yaml"
```

Look for in `status.json`:

- `last_replan_strategy = "reuse_suffix"`
- `last_replan_strategy = "reuse_full"`
- `last_replan_strategy = "random_rollout"`
- `last_replan_strategy = "primitive"`

And in `solution.yaml`:

- `plan_origin.kind: reuse_suffix`
- `plan_origin.kind: reuse_full`

## Notes

- Closed-loop sim reuse keeps the original sim `push_steps`.
- Real execution still floors a first push of `1` step to `2` steps.
- New `solution.yaml` files now include:
  - `search_stats.search_time_ms`
  - `search_stats.pushes_in_plan`
  - `search_stats.sim_pushes_tried`
- Real pushes often succeed but the wrapper hangs during shutdown.
- The most reliable way to avoid that for closed-loop execution is to run
  `execute_real_push.py` with `--headless`, which generated launch scripts
  now do.
- If the push succeeded but `scene_after` is missing, do not move the scene.
  Run `recover-scene-after` or just retry `advance-iteration` while the live
  post-push scene is still visible to `camera_service`.
