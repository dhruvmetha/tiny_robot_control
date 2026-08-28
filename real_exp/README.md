# Real paper experiments

This directory freezes the three gallery environments selected for the paper
and keeps their physical-run artifacts separate from the earlier pilot and
matrix-v2 data.

See [Planning metrics](METRICS.md) before analyzing a run. It defines the
wall-clock and simulator-count fields, their exact measurement boundaries, and
the legacy interpretation of the first completed easy trial.

## Layout

- `shortlist.json` is the original gallery export.
- `shortlist/` preserves each exported gallery record under its original file
  name.
- `environments/<axis>/<build_id>/build_sheet.csv` is a local copy of the v2
  real-table sheet containing that build. Use the `build_id` in
  `resolved_scenes.csv` to select the three rows for the scene.
- `results/formal_v1/<axis>/<build_id>/` is the root for formal trial output.
- `results/<axis>/<build_id>/` contains legacy pilots and must not receive formal runs.

The gallery's source XML paths live on `/common/users/dm1487/...` and are not
mounted on `dhruv-linux`. Real trials do not consume those files directly:
the v2 build sheet defines physical placement and the runtime constructs its
planning scene from the live camera observation. The original XML path remains
recorded in both the shortlist and `resolved_scenes.csv` for provenance.

## Order

Start with `hmax2/easy_020`, then build the two hard scenes. A scene is not
ready for motion until `check_build.py` returns PASS or the human explicitly
accepts a MARGINAL checksum.

## Formal five-trial protocol

Each experiment cell uses exactly five physical replicates. The trial-to-seed mapping is frozen:

| Trial | Explicit seed |
| --- | ---: |
| `trial1` | `0` |
| `trial2` | `1` |
| `trial3` | `2` |
| `trial4` | `3` |
| `trial5` | `4` |

In short, the formal seeds are `0`, `1`, `2`, `3`, and `4`. Every formal command must pass `--shuffle-seed "$seed"`; do not rely on the planner fallback.

Uniform best-first consumes the seed when assigning random priorities to candidate pushes. For an identical observed scene, model-prior best-first is deterministic and does not consume the random generator in candidate ranking. The model arm still records the same five seeds because each physical setup and camera observation is a new scene state, and matching trial labels keeps the experiment balanced.

Existing runs that omitted `--shuffle-seed` resolved inside best-first to seed 42. They remain valid pilot evidence, but they are outside the formal five-seed timing set and must not be substituted for trials 1–5.

Every formal run belongs under `results/formal_v1`; this versioned subtree prevents a formal trial label from colliding with an unversioned pilot that already uses the same axis, build, method, and trial name.

For model-prior best-first, the process loads the checkpoint, constructs the renderer, initializes the configured device, and performs synthetic ranker forward passes once before measured planning. This one-time duration is recorded separately as `model_warmup_ms` and is excluded from the planning wall-time fields defined in [Planning metrics](METRICS.md).

## Setup command

From the `robot_control` repository root, substitute the row's axis and build
id from `resolved_scenes.csv`:

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet real_exp/environments/<axis>/<build_id>/build_sheet.csv \
  --build-id <build_id> --camera-service tcp://localhost:5556 \
  --gui --auto 10
```

## First trial command

The initial cell is the requested model-prior best-first search. Set the goal
from `resolved_scenes.csv`; the example below is `hmax2/easy_020`.

```bash
axis=hmax2
build_id=easy_020
trial_index=1
trial="trial${trial_index}"
seed=$((trial_index - 1))
diag_path="real_exp/results/formal_v1/${axis}/${build_id}/model_search"

cd ../namo_cpp
set -a
. env.robotlearning.sh
set +a
cd ../robot_control
export NAMO_REPO=/home/dhruv/projects_dhruv/namo/namo_cpp

NAMO_PUSH_WHEEL_LOG="${diag_path}/${trial}/push_phases.jsonl" \
PYTHONPATH="$NAMO_REPO/build_python:src" \
/home/dhruv/miniconda3/envs/namo312/bin/python -u scripts/run_namo.py \
  --config config/real.yaml --camera-service tcp://localhost:5556 \
  --robot-model car --algorithm full_namo \
  --local-search best_first --best-first-prior model \
  --scorer-ckpt /home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
  --goal 29.7 71.0 --no-shuffle-edges --max-chain-depth 2 \
  --shuffle-seed "$seed" --record-video --capture-scene \
  --diag-path "$diag_path" --run-name "$trial"
```

This is still model-prior best-first search: unheld `full_namo` performs the
whole-problem search directly. Do not add an explicit `--exec-mode search`
here: explicitly named `search` and `reactive` modes address the held-boundary
loop, and the CLI rejects them on this unheld path. Plan execution remains MPC
by default, so after each physical
push the runtime verifies the remaining suffix before falling back to a fresh
graph, boundary selection, and full replan.

## Greedy DFS model command

`greedy_dfs` is a separate unheld Full NAMO planning mode. It operates entirely
inside MuJoCo while constructing the plan: at the current simulator state it
rebuilds the full region graph, selects a boundary, scores that boundary's
candidate pushes with the same horizon-independent model ranker, and commits
the first highest-ranked candidate that actually moves the object. It then
rebuilds the graph from that committed child. It does not search sibling
children or backtrack to an earlier committed simulator state.

A no-op or jammed candidate is blacklisted at the unchanged state and still
counts as a simulation. A candidate that moves and then reports a jam is a real
child state, so it is committed. The default best-first horizon caps the plan at
two committed pushes; the command makes that cap explicit. This is not the
camera-reactive held-target mode, so do not pass `--hold-region-target` or
`--active-target`.

For a formal model-prior greedy-DFS trial, use a distinct result arm:

```bash
axis=hmax2
build_id=hard_004
trial_index=1
trial="trial${trial_index}"
seed=$((trial_index - 1))
diag_path="real_exp/results/formal_v1/${axis}/${build_id}/model_greedy_dfs"

cd ../namo_cpp
set -a
. env.robotlearning.sh
set +a
cd ../robot_control
export NAMO_REPO=/home/dhruv/projects_dhruv/namo/namo_cpp

NAMO_PUSH_WHEEL_LOG="${diag_path}/${trial}/push_phases.jsonl" \
PYTHONPATH="$NAMO_REPO/build_python:src" \
/home/dhruv/miniconda3/envs/namo312/bin/python -u scripts/run_namo.py \
  --config config/real.yaml --camera-service tcp://localhost:5556 \
  --robot-model car --algorithm full_namo \
  --local-search best_first --best-first-prior model \
  --scorer-ckpt /home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
  --exec-mode greedy_dfs --best-first-hmax 2 \
  --goal 11.0 67.6 --no-shuffle-edges --max-chain-depth 2 \
  --shuffle-seed "$seed" --record-video --capture-scene \
  --diag-path "$diag_path" --run-name "$trial"
```

Model warmup still happens before measured planning. `sim_pushes_tried` and the
planning wall-time fields include every greedy-DFS simulation after warmup,
including same-state candidates rejected as no-ops or jams.
After planning, physical execution uses the existing MPC suffix verification;
this mode changes how the simulator constructs the initial plan, not how a
returned push is executed or checked against the next camera observation.

Simulation reachability is a transition to the final physical navigation, not
a real-trial success condition. Once the live scene has an open path, the
runtime dispatches a `NavigateSubgoal`. A real trial succeeds only after the
camera observation places the robot within 5 cm of the goal (or of an explicit
nearby retarget when the exact goal point is geometrically covered).

`results/hmax2/easy_020/model_search/trial1` remains a valid pilot: it completed the easy scene with two successful physical pushes and final real navigation to within 1.108 cm of the requested goal. It predates the uniform telemetry schema, so use the explicit legacy accounting in [Planning metrics](METRICS.md) rather than interpreting its old `search_time_ms` fields as comparable wall time.

Before launch, confirm the camera service is healthy, `fuser /dev/ttyACM0`
shows no stale owner, the scene checksum is accepted, and the robot is on the
sheet's start circle.
