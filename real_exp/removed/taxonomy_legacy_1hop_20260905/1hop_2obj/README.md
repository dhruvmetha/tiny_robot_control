# One-hop, two-object real-robot environments

This active experiment catalog currently contains one selected environment:

| UID | Scene | Task | Real navigation | Real NAMO |
| --- | --- | --- | --- | --- |
| `2push__env__obstacle_0_movable__0489c6b2` | `v3/hard_loose/rb_00180` | hard, two-push; initial object `obstacle_0_movable` | penalise failed; ignore failed; qualifies | Trial 1 / seed 0 succeeded; next: trial 2 / seed 1 |

Use the full UID in every result path. `rb_00180` alone is not unique across
the environment pools. The physical mapping derived from object dimensions is
`wall_inner_1 -> wall_10`, `wall_inner_2 -> wall_11`,
`obstacle_0_movable -> obj_4`, and `obstacle_1_movable -> obj_1`.

The goal is `(35.6066, 68.2536)` cm. Robot placement is guidance only; it is
not graded as long as the robot begins in the allowed start region.

## Placement checker

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet real_exp/1hop_2obj/environment/rb_00180/build_sheet_derived.json \
  --build-id rb_00180 \
  --camera-service tcp://localhost:5556 \
  --gui --auto 5
```

## Model-policy trial 1

Run only after the reset has been reviewed. This uses Full NAMO's global
graph selection and executes one model-ranked moving push per live observation;
it is not restricted to the gallery card's initial object.

```bash
timeout 600 /home/dhruv/miniconda3/envs/namo312/bin/python -u \
  scripts/run_namo.py \
  --config config/real.yaml \
  --camera-service tcp://localhost:5556 \
  --robot-model car \
  --algorithm full_namo \
  --local-search best_first \
  --best-first-prior model \
  --scorer-ckpt /home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
  --exec-mode greedy_policy \
  --best-first-hmax 2 \
  --goal 35.6066 68.2536 \
  --no-shuffle-edges \
  --max-chain-depth 2 \
  --max-planning-retries 1 \
  --shuffle-seed 0 \
  --record-video --capture-scene \
  --diag-path real_exp/1hop_2obj/results/real/2push__env__obstacle_0_movable__0489c6b2/model_pure_policy \
  --run-name trial1
```

Model warmup is recorded separately and excluded from planning wall time.

## Model-policy results

| Trial | Seed | Outcome | Pushes | Planning wall time | Simulations | Warmup (excluded) | Final distance |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 0 | success | 4 | 580.47 ms | 0 | 4007.66 ms | 1.20 cm |
