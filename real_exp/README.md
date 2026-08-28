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
