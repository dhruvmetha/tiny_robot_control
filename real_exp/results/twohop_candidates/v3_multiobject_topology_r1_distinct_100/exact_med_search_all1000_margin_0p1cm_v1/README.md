# Exhaustive medium-to-medium search at 0.1 cm

This is the complete sequential audit of all 1,000 generated `med2-med2`
discovery-profile scenes. It found three scenes whose measured K1 and K2 are
both exactly medium two-push gates. Two are recommended for the paper because
both real-stack pure-navigation baselines fail.

## Result

| Scene | Exact K1 | Exact K2 | Navigation | Goal | Preview |
|---|---:|---:|---|---|---|
| `twohop_multi_00037` | 7/31 = 0.225806 | 4/38 = 0.105263 | both fail | `--goal 41.0 70.0` | [PNG](med2-med2/selected/renders/twohop_multi_00037/env_regions.png) |
| `twohop_multi_00143` | 3/53 = 0.056604 | 7/91 = 0.076923 | both fail | `--goal 8.0 64.0` | [PNG](med2-med2/selected/renders/twohop_multi_00143/env_regions.png) |
| `twohop_multi_00710` | 9/34 = 0.264706 | 3/37 = 0.081081 | penalise succeeds | `--goal 8.0 66.0` | [PNG](med2-med2/selected/renders/twohop_multi_00710/env_regions.png) |

Use `twohop_multi_00037` and `twohop_multi_00143` as the primary paper
candidates. `twohop_multi_00710` is a valid exact medium-to-medium environment,
but it is a weaker method-comparison scene because the penalising navigation
baseline reaches the goal without explicit push planning.

[candidates.csv](med2-med2/selected/candidates.csv) is the compact, portable
inventory. Each selected scene directory contains the original XML, exact
post-K1 and post-K2 XMLs, build sheet, screening record, and exact K1/K2 rows.

## Meaning of the labels

The labels are measured sequentially, not copied from generator metadata:

1. K1 is evaluated on the original two-hop state. A valid materialization
   opens `obstacle_0_movable`, changes the region path from two hops to one,
   preserves the second boundary, and does not move its protected members.
2. K2 is evaluated on that exact post-K1 XML. A valid materialization opens
   `obstacle_1_movable`, changes one hop to zero, leaves the goal in free
   space, and does not move the protected K1/auxiliary members.
3. At each gate, an exact medium two-push label requires an uncensored result,
   zero valid one-push openers, at least one valid first step of a two-push
   chain, and `0.05 <= valid_first_push / tried < 0.30`.

All three retained scenes pass both transition contracts and both mechanical
protection checks. The repository's translation- and reflection-aware
distinctness implementation also marks all three pairs distinct. It compares
local geometry at both gates, tries both wall assignments within each gate,
and uses the established 5 cm / 15 degree wall, 5 cm / 20 degree auxiliary,
and 0.5 cm dimension thresholds. See
[pairwise_distinctness.json](med2-med2/selected/pairwise_distinctness.json).

## Full-search accounting

- K1: 1,000 unique inputs and 1,000 rows; 655 valid ordered transitions.
- Exact-medium K1 hits: 70. Every hit has exactly one K2 row.
- K2: 70 rows; 4 valid ordered transitions, 62 with no K2 opener, 2 where an
  opener did not reduce the hop count, and 2 scene timeouts.
- Of the four valid K2 transitions, three are exact medium-to-medium. The
  fourth (`twohop_multi_00067`) is medium at K1 and hard at K2.
- Scheduler coverage is complete. One K1 task (`00485`) was preempted and was
  rerun successfully as job `61236816`; its original preemption log is kept.

The authoritative data are
[all1000_k1_rows.jsonl](med2-med2/all1000_k1_rows.jsonl),
[exact_med_k1_candidates.tsv](med2-med2/exact_med_k1_candidates.tsv), and
[exact_med_k1_candidates_k2_rows.jsonl](med2-med2/exact_med_k1_candidates_k2_rows.jsonl).
Reducer validation is under `med2-med2/k1/` and each
`med2-med2/k2_exact/batch*/` directory.

## Margins and navigation baselines

For exact labeling, only the tier-1 base margin changed from `0.005 m` to
`0.001 m` (0.1 cm). Navigation contribution remained `0`, push-approach margin
remained `0.003 m`, XML minimum separation remained `0.005 m`, and table
inflation remained `0.08 m`.

Those are the labeling values captured in
`provenance/config/wavefront_inflation.yaml`. The pure-navigation screen used
its separately captured effective file,
`provenance/navigation/effective_wavefront_inflation.yaml`: tier-1 base
`0.001 m`, navigation addition `0`, XML minimum `0.005 m`, and XML/table
addition `0.08 m`. Its push-approach field is `0`, but that field is unused by
both pure-navigation variants because neither variant plans a push.

The navigation screen runs the same wavefront planner, subgoal executor,
navigation controller, and path follower as the real robot, at speed `0.4`.
It evaluates both `nav_baseline_ignore` and `nav_baseline_penalise`; neither arm
explicitly plans a push. All six arm trajectories and result records passed
the controller-stack, finite-value, command-bound, terminal-stop, and tick
count audits. Both arms record planning calls and planning wall-clock time;
`n_sims` is correctly zero for pure navigation. See
[navigation summary](med2-med2/navigation_realstack_selected_margin_0p1cm_v1/summary.json).

## Physical inventory and setup on dhruv-linux

Each scene uses `wall_9`, `wall_10`, `wall_11`, `wall_12`, `obj_1`, the
existing `obj_4` (called `obj_4a` in the sheet), and a second 12.0 x 7.5 x
5.0 cm clone called `obj_4b`.

Important: at packaging time, `config/objects.yaml` on `dhruv-linux` did not
yet contain `obj_4b`. Give the clone its own ArUco marker and add that marker
as `obj_4b` before using live placement or running the robot. Do not reuse
`obj_4`'s marker ID.

The promoted environment bundle is:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_selected/exact_med2_med2_margin_0p1cm_v1
```

From the robot-control repository, the live placement tool can guide either
recommended setup:

```bash
cd /home/dhruv/projects_dhruv/namo/robot_control
scene=twohop_multi_00037   # or twohop_multi_00143

PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python \
  scripts/check_build.py \
  --sheet "real_exp/environments/twohop_selected/exact_med2_med2_margin_0p1cm_v1/med2-med2/${scene}/build_sheet.json" \
  --build-id "$scene" --camera-service tcp://localhost:5556 \
  --gui --auto 5
```

The exact Full NAMO core flags are:

```text
--algorithm full_namo --local-search best_first --best-first-prior model \
--exec-mode search --best-first-hmax 2 --max-chain-depth 2
```

Append the scene's goal flag from the table or build sheet. Store new trial
outputs under an unambiguous hierarchy such as
`real_exp/results/matrix_v2/2hop/med2_med2/<scene>/<arm>/...`; do not reuse a
pilot output directory.

## Locations and provenance

Amarel source audit:

```text
/scratch/tdn39/real_exp/results/twohop_v3_multiobject/topology_r1/exact_med_search_all1000_margin_0p1cm_v1
```

Full `dhruv-linux` audit copy:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/exact_med_search_all1000_margin_0p1cm_v1
```

The run used labeling worktree commit
`0238730a56fb03cfe497a04b15299d764bffbbf1` with compatible binding SHA-256
`8827080ee877179aae170e676d44a1c11e999755cf31d09dc304346c22ef8968`.
The navigation source exports correspond to robot-control commit `d869420c3a6e`
and NAMO commit `3e3fbec172b028b90bb877a6e3f49c957cf62bb3`; navigation used its own
compiled binding with SHA-256
`d82e614ddaf75884bd0680df0d8444ebdbbfa355cf345c8d0b365996f0358a37`.

The package freezes the byte hashes of all 1,000 composed input XMLs in
`provenance/inputs/source_env_xml_sha256.txt`. It also bundles the exact
labeling algorithm YAML (including the 600-second per-neighbor timeout), the
generator and its local dependencies, donor-sheet CSVs, multi-object donor
manifest, run environment, activation script, and a machine-readable
`provenance/submission_contracts.json`. Single-object donors are rows keyed by
`build_id` in the bundled CSV rather than separate source XMLs; multi-object
donor records include their source XML hashes. See
`run_metadata.json`, `scheduler_accounting.txt`, `provenance/`, and
`SHA256SUMS` for the complete audit trail.

The selector/materializer copies are also installed in their original
`scripts/pipeline/` layout so the frozen tests are self-contained. In an
environment with `pytest`, rerun them from this directory with
`PYTHONDONTWRITEBYTECODE=1 python -m pytest -q -p no:cacheprovider provenance/tests tools/tests`.
