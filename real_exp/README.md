# Real paper experiments: formal v2

This directory contains the frozen scene inputs and runbook for comparing two
model-prior Full NAMO arms on the real robot:

- `model_search`: construct a complete plan with simulated physics rollouts;
- `model_pure_policy`: rank the live actions, execute the arg-max physically,
  observe the camera, and decide again without a physics rollout.

Read [Planning metrics](METRICS.md) before analyzing a run.

## Frozen instrument

Formal-v2 uses the `namo_cpp` backend revision:

```text
1628d1ff195047246315aa81a7808ba5300bc379
```

Do not collect additional formal-v2 rows after changing that backend. Start a
new protocol version instead. The robot-control revision is the committed
revision used to launch the run; diagnostics records both repositories in
`config.json.repositories` before creating experiment output.

## Layout

- `shortlist.json` is the original gallery export.
- `shortlist/` preserves each exported gallery record.
- `environments/real_2mov/<pool>/<family>/<rb_id>/` holds the two-movable
  rooms delivered from the sim side (1598 rooms across pools v1, v2, v3,
  v3_b2, v3_snap, v4_rest, v4_snap). Each room has `env.xml`,
  `build_sheet_derived.json`, and `sweep_record.json`. The pool names are
  sweep batches, not generator recipes, and `rb_id` is a per-family counter,
  so `pool/family/rb_id` is the only scene key. Do not rename below
  `real_2mov/`; the sim side's cards, sweep records, and delivery scripts
  key on that path. Results for these rooms go to
  `results/real_2mov/<pool>/<family>/<rb_id>/<arm>/trialN/`.
- `environments/<axis>/<build_id>/build_sheet.csv` contains the physical build
  sheet for a shortlisted scene.
- `resolved_scenes.csv` records the selected build IDs, source XML provenance,
  and goals.
- `results/formal_v2/<axis>/<build_id>/<arm>/trialN/` receives new output.

The results directory starts empty. Do not recreate or reuse any removed
legacy, pilot, or replay result path.

## Five-trial protocol

Every arm uses exactly five physical replicates with this frozen mapping:

| Trial | Explicit seed |
| --- | ---: |
| `trial1` | `0` |
| `trial2` | `1` |
| `trial3` | `2` |
| `trial4` | `3` |
| `trial5` | `4` |

Always pass `--shuffle-seed`; never rely on a fallback seed. Freeze the order
of the search and pure-policy arms before collection so battery, lighting, and
temperature drift do not systematically favor one method.

Start each trial in a fresh planner process. Model-prior runs automatically
initialize the checkpoint, renderer, device, and synthetic ranker forwards
before measured planning. `model_warmup_ms` records this separately and it is
excluded from the planning wall-time fields.

## Preflight after a reboot

1. Log into the graphical desktop and connect the camera and powered robot.
2. Start the camera service and expose at least three fixed workspace markers
   during its 30-frame warmup.
3. Confirm `Warmup OK`, a live robot pose, stable object tags, and no stale
   owner of the robot serial port.
4. Confirm `namo_cpp` is exactly the frozen revision above and both repositories
   have only understood changes.
5. Run the placement checker and accept only `PASS` or an explicitly reviewed
   `MARGINAL` checksum.
6. Place the robot anywhere inside the allowed start region.

The RealSense `/dev/videoN` index can change after a reboot. Use the YUYV video
node reported by `v4l2-ctl`; do not start a second camera service.

## Scene-placement command

From the robot-control repository root, substitute the scene axis and build ID:

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet real_exp/environments/<axis>/<build_id>/build_sheet.csv \
  --build-id <build_id> --camera-service tcp://localhost:5556 \
  --gui --auto 10
```

## Shared launch setup

The examples below use `hmax2/hard_004`, whose goal is `(11.0, 67.6)` cm.
Use the goal in `resolved_scenes.csv` for another scene.

```bash
axis=hmax2
build_id=hard_004
trial_index=1
trial="trial${trial_index}"
seed=$((trial_index - 1))

cd ../namo_cpp
test "$(git rev-parse HEAD)" = \
  "1628d1ff195047246315aa81a7808ba5300bc379"
set -a
. env.robotlearning.sh
set +a
cd ../robot_control
export NAMO_REPO=/home/dhruv/projects_dhruv/namo/namo_cpp
```

After startup, inspect `config.json.repositories`: both `robot_control` and
`namo_cpp` must have `available: true` and the backend commit must match the
frozen revision.

## Arm A: complete model search

This arm uses Full NAMO best-first search to find a complete simulated plan.
The result path is distinct from pure policy:

```bash
arm=model_search
diag_path="real_exp/results/formal_v2/${axis}/${build_id}/${arm}"

NAMO_PUSH_WHEEL_LOG="${diag_path}/${trial}/push_phases.jsonl" \
PYTHONPATH="$NAMO_REPO/build_python:src" \
/home/dhruv/miniconda3/envs/namo312/bin/python -u scripts/run_namo.py \
  --config config/real.yaml --camera-service tcp://localhost:5556 \
  --robot-model car --algorithm full_namo \
  --local-search best_first --best-first-prior model \
  --scorer-ckpt /home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
  --best-first-hmax 2 --goal 11.0 67.6 \
  --no-shuffle-edges --max-chain-depth 2 \
  --max-planning-retries 1 --shuffle-seed "$seed" \
  --record-video --capture-scene \
  --diag-path "$diag_path" --run-name "$trial"
```

Do not pass an explicit `--exec-mode` for this unheld full-search arm. Physical
execution retains the normal suffix-verification and fresh-replan behavior.

## Arm B: model pure policy

The internal CLI token remains `greedy_policy`, but this formal arm is named
`model_pure_policy`. At each live observation it rebuilds the graph, ranks the
candidate pushes, and returns the arg-max without calling `env.step`. The robot
executes one push; its next camera observation becomes a fresh decision state.
There is no held boundary, simulated suffix, backtracking, or real-push cap.

```bash
arm=model_pure_policy
diag_path="real_exp/results/formal_v2/${axis}/${build_id}/${arm}"

NAMO_PUSH_WHEEL_LOG="${diag_path}/${trial}/push_phases.jsonl" \
PYTHONPATH="$NAMO_REPO/build_python:src" \
/home/dhruv/miniconda3/envs/namo312/bin/python -u scripts/run_namo.py \
  --config config/real.yaml --camera-service tcp://localhost:5556 \
  --robot-model car --algorithm full_namo \
  --local-search best_first --best-first-prior model \
  --scorer-ckpt /home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
  --exec-mode greedy_policy --best-first-hmax 2 \
  --goal 11.0 67.6 --no-shuffle-edges --max-chain-depth 2 \
  --max-planning-retries 1 --shuffle-seed "$seed" \
  --record-video --capture-scene \
  --diag-path "$diag_path" --run-name "$trial"
```

Pure policy reports zero physics-rollout simulations by definition. A physical
no-op or jam is detected by the live displacement threshold and fed into the
external edge blacklist. A real trial succeeds only after final navigation
places the observed robot within 5 cm of the goal or an explicit valid nearby
retarget.
## Execute one simulated keyhole at a time

`run_namo.py --planning-horizon first_keyhole` runs the existing Full NAMO best-first search until it finds a verified solution to the first keyhole, then returns that complete local push chain for physical execution. It supports both `--best-first-prior model` and `--best-first-prior uniform`. The default `full_goal` horizon still searches to the final goal before the first physical push.

The runtime executes one physical push at a time and verifies the remaining chain from the next camera observation against the opening's original target points and threshold. Once the observed scene opens that keyhole, it discards the old chain and selects the next keyhole from the observed scene. If reuse fails while the opening is still closed, Full NAMO searches again with the same blockers and frozen criterion. Goal-clearance tasks retain their occupied-goal-cell criterion. An opening is progress; final trial success still requires reaching the goal.

First-keyhole mode requires `--algorithm full_namo --local-search best_first --execution-mode mpc` and uses a full-problem simulation budget. Omit the legacy `--hold-region-target` / `--active-target` flags. `--exec-mode search` is optional here; policy and greedy-DFS modes have different semantics and are rejected with this horizon. The same mutable allowance covers every fresh search, physical retry, and suffix/full-chain verification in the trial. The default limit is the canonical 900 simulator pushes; `--keyhole-simulation-budget N` overrides the total trial limit in this mode. Camera checks without simulated pushes cost zero units. A new process or `NAMOPlanner.reset()` begins a new trial; restarting a process does not resume a budget.

The CLI flag overrides `namo.planning_horizon` in the real YAML configuration. Logs record `planning_horizon`, `plan_outcome`, the opening criterion, per-call `simulations_used`, and cumulative `trial_simulation_budget_used` / `trial_simulation_budget_remaining`. A plan-only local opening is reported as `keyhole_ready`, separately from full-problem success.

For paired worktrees, set `NAMO_CPP_WORKTREE` and `ROBOT_CONTROL_WORKTREE` to the two checkout roots, then prepare the environment below before using the usual trial command with `--planning-horizon first_keyhole`. Both worktrees are needed. Use the intended real object definitions and calibration when preparing a hardware trial; a code worktree contains its branch's tracked configuration.

```bash
cd "$NAMO_CPP_WORKTREE"
set -a
. env.robotlearning.sh
set +a
cd "$ROBOT_CONTROL_WORKTREE"
export PYTHONPATH="$PWD/src:$PYTHONPATH"
```
