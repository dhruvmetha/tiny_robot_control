# Exhaustive hard-to-hard search at 0.1 cm

This audit labels K1 across all 1,136 generated `hard1-hard1` discovery-profile
scenes, then labels K2 only for scenes whose measured K1 is exactly one-push
hard. It uses the same ordered materialization contract and 0.1 cm tier-1 base
margin as the all-100 audit.

## Result

No exact hard-to-hard scene exists in this generated pool.

- 1,136 K1 inputs were processed successfully at the scheduler level.
- 400 produced a valid ordered K1 transition after topology validation.
- 4 of those 400 measured as exact one-push hard at K1.
- 1 of those 4 produced a valid K2 transition, but its K2 was easy.
- The other 3 had no K2 opener.
- The authoritative selector therefore returned 0 exact `hard1-hard1` scenes.

The four hard-K1-only scenes are pairwise distinct under the established
translation/reflection-aware policy and its 5 cm / 15 degree wall, 5 cm / 20
degree auxiliary-object, and 0.5 cm dimension thresholds. They are diagnostic
examples, not two-hop paper candidates:

| Scene | K1 one-push result | K2 result |
|---|---:|---|
| `twohop_multi_00013` | 2/129 = 0.0155039 (hard) | 11/35 = 0.3142857 (easy) |
| `twohop_multi_00022` | 1/47 = 0.0212766 (hard) | no K2 opener |
| `twohop_multi_00581` | 1/47 = 0.0212766 (hard) | no K2 opener |
| `twohop_multi_00605` | 1/47 = 0.0212766 (hard) | no K2 opener |

Their original XMLs, build sheets, exact K1/K2 rows, post-K1 XMLs, available
post-K2 XML, and renders are under `hard1-hard1/exact_hard_k1_only/`.

## K1 accounting

Across all 1,136 inputs, materialization status was:

- `ok`: 430
- `no_kh1_opener`: 470
- `no_opener_decrements_hop`: 182
- `no_region_path`: 54

Thirty `ok` rows failed the ordered topology contract, leaving 400 valid K1
transitions. The failures were 23 wrong opened boundaries, 6 wrong post-state
next boundaries, and 1 wrong hop transition. The reducer reconciled all normal
and no-decrement XML artifacts with no unreferenced files.

## Configuration

Only the tier-1 base margin was changed to `0.001 m` (0.1 cm). Navigation
contribution remained `0`, push-approach margin remained `0.003 m`, XML minimum
collision margin remained `0.005 m`, and table inflation remained `0.08 m`.
Both hard-profile stages used chain depth 1, seed 42, a 600-second per-neighbor
limit, and a 900-second per-scene cap.

The 1,136 inputs were split only to respect Amarel's maximum array index and
500-submitted-task policy. `run_metadata.json` records the exact original-index
mapping and every job ID. No scene was skipped or reordered.

## Locations

Amarel full audit:

```text
/scratch/tdn39/real_exp/results/twohop_v3_multiobject/topology_r1/exact_hard_search_all1136_margin_0p1cm_v1
```

dhruv-linux copy:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/twohop_candidates/v3_multiobject_topology_r1_distinct_100/hard1_hard1_search_all1136_margin_0p1cm_v1
```

The outcome suggests that generating more blind perturbations from this same
profile is low-yield. A subsequent generator should use measured sequential K1
and K2 feedback as an acceptance criterion rather than treating donor labels
as composed-scene labels.
