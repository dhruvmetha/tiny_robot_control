# 100 distinct multi-object two-hop candidates

This bundle contains 100 model-screened candidate scenes:

- `hard1-hard1/`: 50 candidates selected from 135 bounded-screen passes.
- `med2-med2/`: 50 candidates selected from 224 bounded-screen passes after excluding the 50 hard-profile selections.
- `manifest.relative.txt`: portable paths for all 100 scenes.
- `showcase_10_per_profile.relative.txt`: the first 10 candidates per profile; these have PNGs under `renders/`.
- `audit/`: immutable promotion manifests, rejection counts, effective thresholds, and selection metadata.

All 100 scenes have unique exact and mirror-normalized geometry IDs and span 26 multi-object source scenes. The audit found no near-duplicate violations in 4,950 pairwise comparisons and no collisions with the two previously retained multi-object scenes.

Distinctness compares local geometry at both gates, tries both wall assignments, and normalizes global translation and one left/right reflection. Donor names, gate spacing, and task-anchor movement cannot establish diversity. The thresholds are 5 cm / 15 degrees for walls, 5 cm / 20 degrees for auxiliary objects, and 0.5 cm for dimensions.

Important: `hard1-hard1` and `med2-med2` are discovery profiles. They describe how the generator found these scenes; they are not measured labels and must not be presented as paper difficulty labels.

## Exact sequential audit at 0.1 cm

The all-100 audit measures the two gates in their physical order. K1 is classified on the original two-hop XML. A valid K1 transition opens `obstacle_0_movable`, changes the topology from two hops to one, preserves the K2 boundary, and materializes `post_k1_xml`. K2 is then classified on that exact post-K1 state. A valid K2 transition opens `obstacle_1_movable`, changes the topology from one hop to zero, and materializes `post_k2_xml`. The K1 and K2 scores are therefore sequential measurements, not two independently inferred labels from the generator metadata.

Only the tier-1 base margin changed from `0.005 m` to `0.001 m` (0.1 cm) for this audit. The navigation contribution stayed at `0`, the push-approach margin stayed at `0.003 m`, and the XML collision terms stayed at a `0.005 m` minimum margin plus `0.08 m` table inflation. No other inflation term was changed.

The authoritative selector assigns the exact joint labels as follows:

- `hard1-hard1`: both gate classifications are complete and uncensored; each gate has at least one one-push opener and `0 < solve_rate_1push < 0.05` (`tier_1push=hard`).
- `med2-med2`: both gate classifications are complete and uncensored; each gate has zero one-push openers, at least one valid two-push first step, and `0.05 <= solve_rate_hmax2 < 0.30` (`tier_hmax2=med`).
- `other`: both sequential transitions and classifications are complete, but the pair matches neither exact profile above.
- `incomplete`: a required transition, gate label, or uncensored classification is missing or invalid.

The measured stage counts are:

- Hard discovery profile: 41 of 50 valid K1 transitions, followed by 38 valid K2 transitions.
- Medium discovery profile: 47 of 50 valid K1 transitions after topology validation, followed by 20 valid K2 transitions.
- Total: 58 complete sequential K1/K2 pairs. All 58 passed the all-member mechanical-independence check: opening one gate left every member of the other gate within 2 mm and 1 degree.

The result contains **zero exact `hard1-hard1` pairs and zero exact `med2-med2` pairs**. This is the central audit result: the discovery profile names did not survive exact sequential measurement at the 0.1 cm margin. Do not assign either requested paper label to these scenes yet.

The row-level join is [all100_exact_navigation_inventory.csv](exact_sequential_all100_margin_0p1cm_v1/all100_exact_navigation_inventory.csv), and aggregate counts and provenance are in [all100_exact_navigation_summary.json](exact_sequential_all100_margin_0p1cm_v1/all100_exact_navigation_summary.json). The CSV preserves each K1 `merged_rows.jsonl` order and contains the discovery profile separately from the measured `exact_joint_label`, both gate rates and tiers, transition/materialization status, mechanical independence, physical start/goal coordinates, and the real-stack navigation outcome.

## Exhaustive hard-profile follow-up

The follow-up audit expanded K1 labeling from the selected 50 hard-profile
scenes to all 1,136 generated hard-profile scenes. It found 400 valid K1
transitions but only four exact one-push-hard K1 gates. Their ordered K2 checks
produced one easy K2 and three scenes with no K2 opener, so the complete pool
still contains zero exact hard-to-hard pairs. The four hard-K1-only layouts are
pairwise distinct under the established diversity thresholds, but they are not
two-hop paper candidates.

The full shards, exact rows, renders, selector result, and scheduler provenance
are in [hard1_hard1_search_all1136_margin_0p1cm_v1](hard1_hard1_search_all1136_margin_0p1cm_v1/README.md). On dhruv-linux, the corresponding result is:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/hard1_hard1_search_all1136_margin_0p1cm_v1
```

## Exhaustive medium-profile follow-up

The follow-up audit expanded K1 labeling from the selected 50 medium-profile
scenes to all 1,000 generated medium-profile scenes. It found 70 exact-medium
K1 gates and evaluated K2 on every corresponding post-K1 state. Three scenes
are exact sequential medium-to-medium pairs: `twohop_multi_00037`,
`twohop_multi_00143`, and `twohop_multi_00710`. All three pass the ordered
two-hop transition and mechanical-protection contracts and are pairwise
distinct under the established translation/reflection-aware policy.

The same real-stack `ignore` and `penalise` navigation baselines fail on both
`twohop_multi_00037` and `twohop_multi_00143`, making those the recommended
paper candidates. The penalising baseline succeeds on `twohop_multi_00710`,
so that scene is retained as an alternate.

The complete audit is in
[exact_med_search_all1000_margin_0p1cm_v1](exact_med_search_all1000_margin_0p1cm_v1/README.md).
The compact real-robot handoff is in
[exact_med2_med2_selected_margin_0p1cm_v1](exact_med2_med2_selected_margin_0p1cm_v1/README.md).
On `dhruv-linux`, the corresponding locations are:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/exact_med_search_all1000_margin_0p1cm_v1
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/twohop_selected/exact_med2_med2_margin_0p1cm_v1
```

The second path contains the per-scene build sheets and the exact command for
the live placement checker. A separately tagged `obj_4b` entry is still needed
in `config/objects.yaml` before live use.

## Current 2push-to-2push alternatives

Four complete sequential pairs require a two-push chain at both gates and failed both real-stack pure-navigation variants. Their measured K1→K2 tiers and hmax2 solve rates are:

| Scene | Exact gate tiers | K1 rate | K2 rate | Render |
|---|---:|---:|---:|---|
| `twohop_multi_00284` | easy → medium | 0.7708333 | 0.2083333 | [image](exact_sequential_all100_margin_0p1cm_v1/recommended_2push2push_both_nav_failed/renders/twohop_multi_00284/env_regions.png) |
| `twohop_multi_00385` | easy → easy | 0.3846154 | 0.3030303 | [image](exact_sequential_all100_margin_0p1cm_v1/recommended_2push2push_both_nav_failed/renders/twohop_multi_00385/env_regions.png) |
| `twohop_multi_00933` | easy → medium | 0.3846154 | 0.2280702 | [image](exact_sequential_all100_margin_0p1cm_v1/recommended_2push2push_both_nav_failed/renders/twohop_multi_00933/env_regions.png) |
| `twohop_multi_00711` | easy → medium | 0.6379310 | 0.1315789 | [image](exact_sequential_all100_margin_0p1cm_v1/recommended_2push2push_both_nav_failed/renders/twohop_multi_00711/env_regions.png) |

These are alternatives for physical inspection, not approved paper labels. Review the corresponding XML, render, staging coordinates, and measured row before selecting a real-robot trial.

## Provenance and handoff locations

Amarel code/config source:

```text
/scratch/tdn39/real_exp/code/namo_cpp_twohop_exact_margin0p1cm_v1
```

Amarel exact-label result source:

```text
/scratch/tdn39/real_exp/results/twohop_v3_multiobject/topology_r1/exact_labels_v2_100_margin_0p1cm_v1
```

Amarel real-stack navigation source:

```text
/scratch/tdn39/real_exp/results/twohop_v3_multiobject/navigation_realstack_all100_margin_0p1cm_v1
```

The intended dhruv-linux destination for the combined exact audit, inventory, recommended candidates, and renders is:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/exact_sequential_all100_margin_0p1cm_v1
```

The real-stack navigation output remains at:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/navigation_realstack_all100_margin_0p1cm_v1
```

Original candidate-generation code revision: `a0e2111` on `feat/real-twohop-multiobject-diversity`.
