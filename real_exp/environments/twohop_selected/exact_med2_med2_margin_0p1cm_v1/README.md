# Exact medium-to-medium two-hop scenes

This is the small real-robot handoff bundle selected by exhaustive sequential
labeling of 1,000 generated scenes at the 0.1 cm tier-1 margin.

## Recommended paper scenes

| Scene | K1 hmax2 | K2 hmax2 | Pure navigation | Goal | Preview |
|---|---:|---:|---|---|---|
| `twohop_multi_00037` | 7/31 = 0.225806 | 4/38 = 0.105263 | both variants fail | `--goal 41.0 70.0` | [PNG](med2-med2/renders/twohop_multi_00037/env_regions.png) |
| `twohop_multi_00143` | 3/53 = 0.056604 | 7/91 = 0.076923 | both variants fail | `--goal 8.0 64.0` | [PNG](med2-med2/renders/twohop_multi_00143/env_regions.png) |

`twohop_multi_00710` is included as an alternate. Its K1 and K2 are also exact
medium two-push gates, but the penalising pure-navigation baseline succeeds,
so it is not recommended for the primary NAMO comparison.

The three layouts are pairwise distinct under the project's
translation/reflection-aware two-gate policy. Each scene directory contains
`env.xml`, `post_k1_env.xml`, `post_k2_env.xml`, `build_sheet.json`, the exact
K1/K2 rows, and its bounded-screen record. See
[candidates.csv](med2-med2/candidates.csv) and `audit/` for the compact data.

The navigation screen mirrors `run_navigation_baseline_robot.py`: it uses the
same wavefront planner, subgoal executor, navigation controller, and path
follower at speed `0.4`, for both `nav_baseline_ignore` and
`nav_baseline_penalise`. Only camera/base hardware I/O is replaced by MuJoCo
state sensing and velocity stepping; no separate simulated navigation policy
is used. Pure navigation records planning calls and wall-clock planning time,
and correctly records `n_sims = 0` because it never invokes the NAMO simulator.

The exact-label config used a `0.001 m` tier-1 base margin and `0.003 m`
push-approach addition. The navigation screen used the separately captured
effective config in `audit/effective_navigation_wavefront_inflation.yaml`:
the same `0.001 m` tier-1 base, zero navigation addition, and a zero
push-approach field that neither pure-navigation variant uses. Labeling and
navigation binding SHA-256 values were respectively `8827080e...` and
`d82e614d...`; the full hashes and all 1,000 input XML hashes are in the full
audit package cited below.

## Inventory prerequisite

Each layout uses four bars (`wall_9` through `wall_12`), `obj_1`, the existing
`obj_4` (sheet role `obj_4a`), and a same-size clone named `obj_4b`.

`obj_4b` must have its own ArUco marker and its own entry in
`config/objects.yaml` before live setup. The required shape is 12.0 x 7.5 x
5.0 cm. At bundle creation time that software entry was still absent; the
placement checker will otherwise report `obj_4b` as unseen.

## Live placement

From `/home/dhruv/projects_dhruv/namo/robot_control` on `dhruv-linux`:

```bash
scene=twohop_multi_00037   # or twohop_multi_00143

PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet "real_exp/environments/twohop_selected/exact_med2_med2_margin_0p1cm_v1/med2-med2/${scene}/build_sheet.json" \
  --build-id "$scene" --camera-service tcp://localhost:5556 \
  --gui --auto 5
```

The checker prints the robot start and goal along with live position/yaw
offsets for every tagged wall and object. The per-scene build sheet is the
source of truth for placement coordinates. These JSON sheets do not contain
the legacy contact-count checksum columns, so the final `no checksum columns`
message is informational; accept the setup from the per-item offsets.

## Full NAMO run flags

Use the normal real-robot launcher and model checkpoint with these core flags:

```text
--algorithm full_namo --local-search best_first --best-first-prior model \
--exec-mode search --best-first-hmax 2 --max-chain-depth 2
```

Append the goal shown above (or `run_namo_goal_flag` from the build sheet).
Write results under a unique hierarchy such as
`real_exp/results/matrix_v2/2hop/med2_med2/<scene>/<arm>/...`.

The complete 1,000-scene audit is under:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/exact_med_search_all1000_margin_0p1cm_v1
```
