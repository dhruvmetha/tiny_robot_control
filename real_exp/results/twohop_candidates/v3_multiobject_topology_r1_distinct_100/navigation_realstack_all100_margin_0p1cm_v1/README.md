# Real-stack navigation screen for 100 two-hop candidates

This directory contains the dynamic pure-navigation screen of all 100 scenes in `v3_multiobject_topology_r1_distinct_100`. It replaces a static path-existence check with closed-loop MuJoCo execution through the same navigation stack used on the real robot.

## Result

Two fresh simulations were run per scene at 30 Hz and normalized navigation speed 0.4:

- `nav_baseline_ignore`: movable cells have free-space cost.
- `nav_baseline_penalise`: movable cells are penalized but remain traversable.

Of 100 scenes, 47 defeated both baselines, 27 were solved by both, 19 only by `ignore`, and 7 only by `penalise`.

The reviewed final rerun is Amarel array `61233173`, reduced by job `61233174` after build gate `61233172`. All 100 array tasks completed and produced 200 result/trajectory pairs with no nonempty array or reducer error logs.

| Discovery profile | Both fail | Both succeed | Ignore only | Penalise only |
| --- | ---: | ---: | ---: | ---: |
| `hard1-hard1` | 14 | 18 | 15 | 3 |
| `med2-med2` | 33 | 9 | 4 | 4 |

The 47-row `both_navigation_baselines_failed.csv` is the primary candidate pool for NAMO experiments. `navigation_realstack.csv` contains all 100 scenes and both arms' status, terminal distance, moved objects, stuck retries, control/simulation/wall-clock timing, planning-call count, planning wall-clock time, and command maxima. These pure-navigation runs use no NAMO planning simulations, so `n_sims=0` by definition.

## Exact-label intersection

This same 100-scene pool already underwent exact sequential K1/K2 labeling at the older 0.5 cm tier-1 margin. That run produced no exact `hard1-hard1` scene and five exact `med2-med2` scenes. Their outcomes in this dynamic 0.1 cm navigation run are:

| Exact scene | Navigation outcome |
| --- | --- |
| `twohop_multi_00001` | ignore succeeds; penalise fails |
| `twohop_multi_00087` | ignore succeeds; penalise fails |
| `twohop_multi_00385` | both fail |
| `twohop_multi_00933` | both fail |
| `twohop_multi_00972` | both fail |

The strongest present paper candidates are therefore `twohop_multi_00385`, `twohop_multi_00933`, and `twohop_multi_00972`. Their ordered medium-medium evidence is exact at 0.5 cm, and both real-stack navigation arms fail at 0.1 cm. Before calling them final 0.1 cm medium-medium scenes, rerun the exact sequential labeler at 0.1 cm and then replay them on the real robot.

The five exact scene bundles, including initial/post-K1/post-K2 XMLs, build sheets, and renders, are at:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_candidates/exact_v2_100
```

## Broader inspection set

These candidates defeat both baselines through the controller's normal stuck/retry termination path in both arms, rather than relying on the outer harness limit. They also span different donor pairs within the already distinct geometry set.

- `hard1-hard1`: `twohop_multi_01071`, `twohop_multi_00331`, `twohop_multi_00003`, `twohop_multi_00466`, `twohop_multi_00531`.
- `med2-med2`: `twohop_multi_00430`, `twohop_multi_00217`, `twohop_multi_00140`, `twohop_multi_00540`, `twohop_multi_00673`.

For any CSV row, the matching environment on dhruv-linux is:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_candidates/v3_multiobject_topology_r1_distinct_100/<profile>/<scene_id>/env.xml
```

The `scene_dir` column intentionally retains the original Amarel path as provenance; use `<profile>/<scene_id>` for the portable mapping above.

## What was held equal to the real robot

The simulation invokes `NavigationBaselinePlanner`, `NavigationController`, `FollowPathController`, and `SubgoalExecutor` with the same lifecycle, wavefront configuration, stuck watchdog, retreat helper, retry cap, and controller YAML as `run_navigation_baseline_robot.py`. The MuJoCo adapter only converts m to cm, radians to degrees, and applies one normalized differential-wheel command for each 30 Hz controller tick. Each baseline arm constructs a fresh MuJoCo environment.

The requested tier-1 base margin is 0.001 m (0.1 cm). Robot radius remains 3.5 cm and all other inflation terms are unchanged, so total static-obstacle inflation is 3.6 cm. In particular, robot-control navigation/push additions remain 0.0 m, while namo_cpp's independent push-approach addition remains 0.003 m; XML collision resolution remains 0.005 m + 0.08 m. Both sidecar hashes and the effective navigation config are recorded in `run_config.json`.

The requested 0.4 cap applies to ordinary navigation states. The real shared retreat helper uses a 0.15 reverse base speed plus steering correction and clamps at the normalized actuator range of 1.0; the trajectory audit enforces those two real limits separately.

Amarel used its installed MuJoCo 3.8.1 to execute physics. The controller and adapter sources are hash-identical to the dhruv-linux parity branch; the exact revisions and composite hashes are in `run_config.json`.

## Runtime guard

Some blocked physical trajectories can keep an active subgoal alive beyond the planner's own timeout. The simulation therefore has a 120 s outer guard. Reaching it is a recorded experimental failure named `simulation_harness_tick_limit_reached`: the final stop is included inside the cap, so guarded runs end at exactly 3,600 control ticks and 120.0 simulated seconds. The final rerun had 30 such arms.

Each `result.json` preserves every field from the real robot's `BaselineOutcome.as_row()` at top level and also keeps the richer simulation metadata plus a nested `outcome` copy. Trajectory and result publication is ordered atomically with `result.json` last; an overwrite attempt first removes its owned old result, so a process error cannot make a stale successful row look current.

## Files

- `summary.json`: aggregate counts and audit hashes.
- `navigation_realstack.csv`: all 100 scene outcomes.
- `both_navigation_baselines_failed.csv`: the 47 scenes that neither arm solved.
- `run_config.json`: input hash, controller/physics parameters, source revisions, and Slurm job provenance.
- `scenes/<profile>/<scene_id>/<arm>/result.json`: one complete result per arm, with the real baseline row at top level and simulation metadata alongside it.
- `navigation_realstack_trajectories.tar.gz`: all 200 JSONL trajectories, compressed to save dhruv-linux disk space.
- `jobs/` and `logs/`: exact Slurm scripts, reducer, diagnostics, and job output.
- `portable_archives.sha256`: transfer-integrity hashes.

To materialize the trajectories in this directory:

```bash
tar -xzf navigation_realstack_trajectories.tar.gz
```

The uncompressed authoritative run remains at:

```text
/scratch/tdn39/real_exp/results/twohop_v3_multiobject/navigation_realstack_all100_margin_0p1cm_v1
```

## Reproduce one scene on dhruv-linux

The implementation is isolated in the following worktrees until it is merged:

```text
/home/dhruv/projects_dhruv/namo/robot_control/.worktrees/nav-baseline-sim
/home/dhruv/projects_dhruv/namo/namo_cpp/.worktrees/nav-baseline-sim
```

Example:

```bash
cd /home/dhruv/projects_dhruv/namo/robot_control/.worktrees/nav-baseline-sim
export NAMO_CPP_DIR=/home/dhruv/projects_dhruv/namo/namo_cpp/.worktrees/nav-baseline-sim
PYTHONPATH=src \
  /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/run_navigation_baseline_sim.py \
  --xml /home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_candidates/v3_multiobject_topology_r1_distinct_100/hard1-hard1/twohop_multi_01071/env.xml \
  --out /tmp/twohop_multi_01071_nav \
  --mode both \
  --speed 0.4 \
  --timeout 90 \
  --max-sim-time 120 \
  --namo-config "$NAMO_CPP_DIR/config/namo_config_complete_skill15_car_1x.yaml"
```

## Labeling caveat

`hard1-hard1` and `med2-med2` in the 100-scene pool are generator/discovery profiles. Exact sequential labeling at 0.5 cm has already shown why that distinction matters: none of the nominal hard-hard scenes remained hard-hard, while five nominal medium-medium scenes did. The requested 0.1 cm relabel must again solve K1 from the initial state, carry that terminal state into the middle room, and then solve K2. Real-robot replay is still required after that simulation label.
