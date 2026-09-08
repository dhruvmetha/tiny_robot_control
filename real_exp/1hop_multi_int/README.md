# One-hop multi-interaction real-robot environments

Canonical catalog for one-hop scenes that require interacting with multiple
movable blockers. Selected environments are:

| UID | Scene | Task | Structural evidence | Next run |
| --- | --- | --- | --- | --- |
| `2push__env__obstacle_0_movable__febcb94c` | `v2/zig_solo0/rb_00034` | hard, two-push horizon | qualifies by majority: penalised navigation 2/5 reached, ignore 0/1 | none: all 15 trials collected (policy 5/5, model search 5/5, uniform search 1/5) |
| `2push__env__obstacle_0_movable__fbdce248` | `v2/dense_solo0/rb_00121` | medium, two-push horizon | complete 2026-09-08: policy 5/5, model search 5/5, uniform 3/5; nav qualifies by majority (2/5 reached) | none |

Real trials must run with the GUI visible. Do not pass `--headless` to
`scripts/run_namo.py`; policy decisions are still one action per replan, but
the live window must remain visible during execution.

Use the full UID in every result path. The physical reset mapping is
`wall_inner_1 -> wall_10`, `wall_inner_2 -> wall_11`,
`obstacle_0_movable -> obj_1`, and `obstacle_1_movable -> obj_4`.

Goal: `(38.4845, 65.4918)` cm. The active `rb_00034` directory is restored to
the original source layout. The changed layout is preserved under
`variants/rebased_wall10_obj4/rb_00034`; its wall 10 pose is
`(9.6841, 27.7871, 7.1996)` and its obj_4 pose is
`(20.2340, 44.8279, 14.6052)` in cm/degrees. Its navigation results are kept
under the matching result variant. Robot placement is guidance only as long
as it starts in the allowed robot region.

## Placement checker

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet real_exp/1hop_multi_int/environments/2push__env__obstacle_0_movable__febcb94c/rb_00034/build_sheet_derived.json \
  --build-id rb_00034 \
  --camera-service tcp://localhost:5556 \
  --gui --auto 5
```

For `fbdce248`, use its derived three-bar physical sheet:

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet real_exp/1hop_multi_int/environments/2push__env__obstacle_0_movable__fbdce248/rb_00121/build_sheet_derived.json \
  --build-id rb_00121 \
  --camera-service tcp://localhost:5556 \
  --gui --auto 5
```

Its mapping is `wall_inner_1..3 -> wall_10..12`, respectively, and
`obstacle_0_movable -> obj_4`, `obstacle_1_movable -> obj_1`. Its goal is
`(20.0793, 58.6272)` cm.

The root-level `../EXPERIMENT_STATUS.md` is the collection ledger for all
active environments and lists every completed and remaining trial.

## Original-layout ignore-navigation trial 1

```bash
timeout 120 /home/dhruv/miniconda3/envs/namo312/bin/python -u \
  scripts/run_navigation_baseline_robot.py \
  --config config/real.yaml \
  --mode ignore \
  --goal 38.4845 65.4918 \
  --camera-service tcp://localhost:5556 \
  --diag-path real_exp/1hop_multi_int/results/real/2push__env__obstacle_0_movable__febcb94c/variants/original \
  --run-name nav_baseline_ignore/trial1 \
  --capture-scene --record-video
```
