# Real paper experiments

This directory freezes the three gallery environments selected for the paper
and keeps their physical-run artifacts separate from the earlier pilot and
matrix-v2 data.

## Layout

- `shortlist.json` is the original gallery export.
- `shortlist/` preserves each exported gallery record under its original file
  name.
- `environments/<axis>/<build_id>/build_sheet.csv` is a local copy of the v2
  real-table sheet containing that build. Use the `build_id` in
  `resolved_scenes.csv` to select the three rows for the scene.
- `results/<axis>/<build_id>/` is the root for recorded trial output.

The gallery's source XML paths live on `/common/users/dm1487/...` and are not
mounted on `dhruv-linux`. Real trials do not consume those files directly:
the v2 build sheet defines physical placement and the runtime constructs its
planning scene from the live camera observation. The original XML path remains
recorded in both the shortlist and `resolved_scenes.csv` for provenance.

## Order

Start with `hmax2/easy_020`, then build the two hard scenes. A scene is not
ready for motion until `check_build.py` returns PASS or the human explicitly
accepts a MARGINAL checksum.

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
trial=trial1
diag_path="real_exp/results/${axis}/${build_id}/model_search"

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
  --record-video --diag-path "$diag_path" --run-name "$trial"
```

This is still model-prior best-first search: unheld `full_namo` performs the
whole-problem search directly. Do not add an explicit `--exec-mode` here;
that option addresses the held-boundary loop and the CLI rejects it on this
unheld path. Plan execution remains MPC by default, so after each physical
push the runtime verifies the remaining suffix before falling back to a fresh
graph, boundary selection, and full replan.

Before launch, confirm the camera service is healthy, `fuser /dev/ttyACM0`
shows no stale owner, the scene checksum is accepted, and the robot is on the
sheet's start circle.
