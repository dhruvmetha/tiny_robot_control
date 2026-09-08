# Exact two-hop real-scene candidates

This bundle contains all five scenes from the 100-scene diversity pool that passed the original exact sequential `med2-med2` labeling. No candidate in the corresponding 50-scene hard-profile pool measured as `hard1-hard1`.

## Superseded label notice

The ordered all-100 relabel at the requested 0.1 cm tier-1 margin is complete.
It found zero exact `hard1-hard1` and zero exact `med2-med2` pairs. This
directory remains useful as historical 0.5 cm evidence, but its difficulty
labels are not paper labels at 0.1 cm. The authoritative audit, full inventory,
post-K1/post-K2 XMLs, and current inspection candidates are at:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/exact_sequential_all100_margin_0p1cm_v1
```

At 0.1 cm, `twohop_multi_00385` measured as two-push easy → easy and
`twohop_multi_00933` as two-push easy → medium. `twohop_multi_00001` and
`twohop_multi_00087` failed the exact K1 topology contract, and
`twohop_multi_00972` did not complete K2 before the scene timeout.

## Important margin status

The scene generation and exact K1/K2 labels in this bundle used the old tier-1 extra margin of **0.5 cm** (`TIER1_MARGIN = 0.005 m`). They were **not** labeled with the requested **0.1 cm** margin. The generator's separate 10 cm post-push corridor-width requirement was unchanged; changing the tier-1 margin alone changes the radius used by that corridor test from 5.5 cm to 5.1 cm.

The authoritative offline navigation screen described below did use the requested **0.1 cm** tier-1 margin. Its 3.5 cm robot radius and every other inflation term were left unchanged, for 3.6 cm total static-obstacle inflation. Therefore:

- The navigation-only conclusion below is valid at the requested 0.1 cm margin.
- The `med2-med2` difficulty label is a historical 0.5 cm result; the completed 0.1 cm relabel supersedes it.
- Treat these as historical candidates, not final 0.1 cm paper labels.

## Locations on dhruv-linux

Repository root:

```bash
ssh dhruv-linux
cd /home/dhruv/projects_dhruv/namo/robot_control
```

All five environment bundles:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_candidates/exact_v2_100
```

Offline navigation results and route images:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/exact_v2_100/navigation_grid_margin_0p1cm_v1
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/exact_v2_100/navigation_grid_margin_0p5cm_v1
```

The `0p1cm` directory is authoritative for the requested navigation screen. The `0p5cm` directory is retained only as a comparison with the old setting.

## Candidate inventory

| Scene | Robot start (cm) | Goal flag | K1 2-push solve rate | K2 2-push solve rate | Full NAMO screen |
|---|---:|---|---:|---:|---|
| `twohop_multi_00001` | 24.5, 9.0 | `--goal 41.0 66.0` | 25.37% | 23.40% | passed, 13 sims, 15.24 s |
| `twohop_multi_00087` | 24.5, 9.0 | `--goal 24.5 64.0` | 26.87% | 6.67% | passed, 16 sims, 12.25 s |
| `twohop_multi_00385` | 8.0, 9.0 | `--goal 24.5 68.0` | 21.15% | 18.18% | passed, 35 sims, 14.99 s |
| `twohop_multi_00933` | 8.0, 9.0 | `--goal 24.5 68.0` | 21.15% | 6.90% | passed, 36 sims, 75.22 s |
| `twohop_multi_00972` | 8.0, 9.0 | `--goal 8.0 64.0` | 21.15% | 8.33% | passed, 31 sims, 14.58 s |

Each scene has three movable objects. `obstacle_0_movable` and `obstacle_1_movable` are the ordered K1 and K2 boundary blockers. `obstacle_2_movable` is the auxiliary object. The physical roles are recorded in each `build_sheet.json`; the cloned `obj_4b` still needs a unique ArUco tag before a real trial.

Each scene directory contains:

- `env.xml`: initial real-trial scene
- `post_k1_env.xml`: exact state after the canonical K1 opener
- `post_k2_env.xml`: exact state after the canonical K2 opener
- `build_sheet.json`: physical placement, goal, and measured per-gate labels
- `screen.json`: bounded model-guided Full NAMO discovery replay
- `render/env_regions.png`: initial layout, wavefront regions, and region graph

The post-gate XMLs are evidence/debugging fixtures. Always begin a real trial from `env.xml`.

## Offline pure-navigation finding

Amarel jobs `61220855` (0.1 cm, authoritative) and `61220839` (0.5 cm, comparison) ran both existing pure-navigation variants:

- `ignore`: movable cells cost the same as open floor, producing the shortest route.
- `penalise`: movable cells cost 5, preferring a detour but still allowing a route through a movable. A sweep also tested costs 1, 1.25, 1.5, 2, 3, 5, 10, 25, and 100.

At the requested 0.1 cm margin, every scene returned `needs_a_push=True`. No penalty through 100 produced a route clear of all movables.

| Scene | Ignore crossed cells | Penalise crossed cells | Movables hit by penalised route | Clear route found |
|---|---:|---:|---|---|
| `twohop_multi_00001` | 71 | 56 | K1, K2 | no |
| `twohop_multi_00087` | 78 | 52 | K1, K2 | no |
| `twohop_multi_00385` | 63 | 50 | K1, K2 | no |
| `twohop_multi_00933` | 68 | 53 | K1, K2 | no |
| `twohop_multi_00972` | 68 | 43 | K1, K2 | no |

The CSV says `ignored_reached=True` and `penalised_reached=True` because these two planners deliberately allow traversal through movable cells. That does **not** mean the robot can drive around the obstacles. The route crosses both boundary blockers in every scene, so none is navigation-only in the geometric screen. The real robot may stall or bulldoze an object on such a route; the physical navigation trial is still the final qualification evidence.

## Dynamic real-stack navigation update

The static route diagnostic above has now been followed by closed-loop MuJoCo execution through the same `NavigationBaselinePlanner`, wavefront planner, `NavigationController`, `FollowPathController`, `SubgoalExecutor`, watchdog, retreat, and retry code used by the real robot. Both arms ran at normalized navigation speed 0.4 and 30 Hz with the requested 0.1 cm tier-1 margin; every arm used a fresh environment.

| Scene | Ignore | Penalise | Recommendation |
| --- | --- | --- | --- |
| `twohop_multi_00001` | success, 1.53 cm from goal | failed, 5.67 cm from goal | exclude if both navigation baselines must fail |
| `twohop_multi_00087` | success, 1.34 cm from goal | failed, 17.30 cm from goal | exclude if both navigation baselines must fail |
| `twohop_multi_00385` | failed, 47.47 cm from goal | failed, 14.03 cm from goal | strong candidate |
| `twohop_multi_00933` | failed, 47.47 cm from goal | failed, 7.97 cm from goal | strong candidate |
| `twohop_multi_00972` | failed, 44.45 cm from goal | failed, 27.01 cm from goal | strong candidate |

The complete 100-scene result and trajectory audit is at:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/navigation_realstack_all100_margin_0p1cm_v1
```

This dynamic screen produced a historical shortlist: `twohop_multi_00385`, `twohop_multi_00933`, and `twohop_multi_00972` are the intersection of exact ordered medium-medium evidence at 0.5 cm and failure of both dynamic navigation baselines at 0.1 cm. The completed 0.1 cm audit supersedes those medium-medium labels; consult the result path in the notice above.

## Build and inspect one scene

From the robot-control repository on dhruv-linux:

```bash
source ../namo_cpp/env.robotlearning.sh
SCENE=twohop_multi_00001
SCENE_ROOT=real_exp/environments/twohop_candidates/exact_v2_100/med2-med2

python scripts/check_build.py \
  --sheet "$SCENE_ROOT/$SCENE/build_sheet.json" \
  --build-id "$SCENE" \
  --camera-service tcp://localhost:5556 \
  --gui --auto 5
```

The checker prints the position and angle correction for every wall, movable, robot start, and goal. Place by `long_axis_bearing_deg`, not raw marker yaw.

To copy a scene visualization to a local machine:

```bash
scp dhruv-linux:/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_candidates/exact_v2_100/med2-med2/twohop_multi_00001/render/env_regions.png .
```

## Physical navigation trial

The paper protocol currently records the `penalise` arm. For `twohop_multi_00001`:

```bash
timeout 120 python scripts/run_navigation_baseline_robot.py \
  --config config/real.yaml \
  --mode penalise \
  --goal 41.0 66.0 \
  --camera-service tcp://localhost:5556 \
  --diag-path real_exp/results/formal_v2/twohop_multi_00001 \
  --run-name nav_penalise \
  --capture-scene --record-video --step-confirm
```

To physically test the second navigation variant, reset and recheck the scene first, then change `--mode penalise` to `--mode ignore` and use a distinct run name. A navigation attempt may move a block even though it never plans a push.

After the run, inspect the navigation row and fill in the generated `nav_verdict.json`. The physical observation and that explicit verdict—not the offline CSV alone—determine whether the scene enters the paper matrix.

## Provenance

- Exact scene-label source revision: `13320a0602843716e217d8cc88b23b293bb33887`
- Bundle/documentation revision recorded after labeling: `5fe27df4349471242b8d42f8a5caa8a3b5eae9dd`
- Navigation-baseline source revision: `fce48ef9df9db8379164fdda5fad6693da221a37`
- Dynamic real-stack navigation revisions: robot-control `d869420c3a6e488db93a0822c8214f3066f4e875`; NAMO bridge `3df35995f11c941ec07246cf9bfb0e862719ac9d` (Amarel equivalent `3e3fbec172b028b90bb877a6e3f49c957cf62bb3`).
- Authoritative 0.1 cm config SHA-256: `e4918a8ba61820d5fd69740b00e3ebd2799d432b16fd323a46d5451defa5e726`
- Amarel authoritative result: `/scratch/tdn39/real_exp/results/twohop_v3_multiobject/topology_r1/exact_labels_v2_100/navigation_grid_margin_0p1cm_v1`
- `run_metadata.json` pins the exact K1/K2 jobs, selected scene IDs, checkpoint, Full NAMO screen time, and simulator calls.
- `SHA256SUMS` covers every other file in this environment bundle.

Exact difficulty labels require ordered tests: K1 on the initial scene, then K2 on that exact post-K1 state. Do not assign K1 and K2 independently from two unrelated donor scenes.
