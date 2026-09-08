# One-hop, two-object real-robot environments

This is the experiment register for scenes in the `1hop_2obj` study stratum.
It currently contains one selected environment:

| UID | Scene | Task | Navigation simulation | Real navigation | Real NAMO |
| --- | --- | --- | --- | --- | --- |
| `2push__env__obstacle_1_movable__f110e8c4` | `v3_snap/hard_loose/rb_00022` | hard, two-push; start with `obstacle_1_movable` | `ignore` failed; `penalise` failed | `ignore` failed; `penalise` failed; qualifies | Ready for model policy, trial 1 / seed 0 |

The staged XML contains exactly two movable objects,
`obstacle_0_movable` and `obstacle_1_movable`. The sweep record contains a
two-action solution that starts with `obstacle_1_movable` and finishes with
`obstacle_0_movable`, so both blocking movables must remain available to the
planner.

Use the full UID above in result paths. `rb_00022` alone is not unique across
the environment pools.

## Contents

- `environments.json` is the machine-readable register and provenance record.
- `environment/selection.json` preserves the selected gallery row supplied for
  this experiment. The original full gallery-card JSON was not present on
  `dhruv-linux` when this folder was created.
- `environment/rb_00022/env.xml` and `build_sheet_derived.json` are the
  physical-experiment version of the selected scene. On 2026-09-05 the
  experiment owner designated wall 11's live orientation as canonical; its
  target is now centered at (15.464805, 51.323587) cm with a 67.131053-degree
  long-axis bearing. Every other target is unchanged.
  `wall_11_orientation_adjustment.json` records the camera sample and
  conversion. `sweep_record.json` remains the original selection evidence.
  Keeping these files under the `rb_00022` parent is required by the placement
  checker's derived-sheet identity check.
- `results/navigation_sim/` preserves the result JSON for each previously run
  navigation baseline. Both failed in simulation.
- Future physical NAMO results belong under `results/real/`, separated by arm
  and trial. The next arm is `model_pure_policy`, starting with trial 1 and
  shuffle seed 0. Model warmup is recorded separately and excluded from the
  planning-time metric by `run_namo.py`.

## Model-policy run

Run from the `robot_control` repository after the reset checker passes. This
arm evaluates all blocking movables through Full NAMO; it is not restricted to
the gallery card's initial object.

```bash
python -u scripts/run_namo.py \
  --config config/real.yaml \
  --camera-service tcp://localhost:5556 \
  --robot-model car \
  --algorithm full_namo \
  --local-search best_first \
  --best-first-prior model \
  --scorer-ckpt /home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
  --exec-mode greedy_policy \
  --best-first-hmax 2 \
  --goal 24.4197 67.0011 \
  --no-shuffle-edges \
  --max-chain-depth 2 \
  --max-planning-retries 1 \
  --shuffle-seed 0 \
  --record-video \
  --capture-scene \
  --diag-path real_exp/1hop_2obj/results/real/2push__env__obstacle_1_movable__f110e8c4/model_pure_policy \
  --run-name trial1
```
