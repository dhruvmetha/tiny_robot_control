# ICRA real-robot study brief

The living record of what the real-robot section claims, how success is
scored, which scenes run, and how the numbers get analyzed. Decisions here
were locked BEFORE data collection (2026-08-25/26, jointly with the sim
side) and changing them mid-study invalidates the pre-registration. The
companion operations doc is [REAL_ROBOT_TRIALS.md](REAL_ROBOT_TRIALS.md);
this one says what counts, that one says how to run it.

Deadline: ICRA submission 2026-09-15.

## Claims

Primary: the method (full_namo, best-first local search, model prior,
HY5U_s2.ckpt as the single fixed seed for every real trial) transfers to
real keyhole/region-opening problems. Secondary, run opportunistically
after the primary is safe: a paired ablation against
`--best-first-prior uniform` on the same physical builds.

## Success semantics

The runtime succeeds at the goal marker within the 5.0 cm goal tolerance.
When the marker's wavefront cell is blocked but a free reachable cell
exists within GOAL_RETARGET_CAP_CM = 12.0 of the marker, the executor
retargets there and arrival counts as success-with-retarget (cap
derivation: half the largest movable's long side 7.5 + robot inflation 3.5
+ wavefront margin 0.5, rounded up; see `planner/namo_planner.py`). Beyond
the cap is a failure. Strict-marker and with-retarget numbers are reported
side by side; the gap counts plans that bury the marker under the pushed
block.

## Trial matrix

14 scenes paired (28 runs), hard-dominated, all v2 sheet ids, all
marker-strict per `marker_retarget.csv`, all post-push corridors >= 11 cm:

- 1push hard: hard_037, hard_001, hard_012, hard_005, hard_009
- hmax2 hard (all needs_2_chain): hard_007, hard_013, hard_008
- 1push med: med_085, med_077, med_093
- hmax2 med: med_069
- 1push easy: easy_002, easy_001

Pairing: rebuild the scene between arms, checksum-verify both builds with
`check_build.py`, randomize which arm runs first per scene. Cut order if
time slips: easy, then medium, never below 8 hard pairs (the sign test
rests on them). Interventions are tiered clean / recovered / invalid;
recovered counts in statistics and is excluded from the video reel.

v2 sheets and all analysis inputs live in
`real_trials/sheets_v2_a82a66a/` (from namo_cpp 96667f6, all 593 scenes
validated, zero refusals): the six sheet CSVs, `post_push_clearance.csv`,
`marker_retarget.csv`, `uniform_offset_baseline.csv`,
`uniform_offset_distributions.csv`. v1 and v2 build ids do NOT name the
same scenes; join on the `xml` column across versions.

## Analysis, pre-registered

- Success comparison: sign test over the paired per-scene outcomes.
- Ranker offset preference: PAIRED WITHIN SCENE, the model-arm push's
  percentile within its own scene's candidate offset distribution
  (distributions file above), never pooled centimetre means; uniform
  candidate-set means range 1.25-5.89 cm across scenes, so pooling drowns
  the effect in geometry. Sim-side reference point: the ranker's argmax
  sits near the 35th percentile over 200 scenes (Spearman -0.414 raw,
  -0.232 with simulated travel partialed out; about half the preference is
  a longer-push preference).
- Any comparison against the measured rotation coefficients (2.13, 2.42
  deg/cm) filters to face=long contacts first.
- Offset-vs-outcome ships with a stated offset-travel confound note; no
  partialing at n=28.
- Every failed trial records `failure_cause` from the fixed vocabulary:
  corridor_too_tight / marker_unreachable / overshoot_onto_goal / stall /
  radio_dropout / planning_failed / other. The human at the table assigns
  it at verdict time.

## Physics freeze and the measured divergence

Sim physics is frozen for the study (both humans' decision, 2026-08-26):
no fitting of sim mass or friction to hardware, because every label both
sides use came from the current physics. The divergence is a result:

- Translation is calibrated: ~4.4 cm of block travel per commanded step on
  both sides; first contact lands ~14 ticks (0.47 s) into a push, a fixed
  haircut on effective push length.
- Rotation is not: hardware accumulates ~0.70 deg/cm of travel per cm of
  contact offset from the face centre; sim self-squares the block at every
  offset (a missing corner-overhang effect, suspected to live in invented
  friction constants). Near-centre real contacts also self-square
  (rotate ~3 deg, then flat); corner contacts never do.
- Consequence: open-loop replay of push chains diverges in a
  systematically optimistic direction, and MPC re-observation between
  pushes is the measured mitigation. easy_002 (+0.7 deg, matched sim to
  1.3%) is the control case; hard_004's chain closed on the table while
  open-loop replay diverged.

## Calibration blocks (characterization, feeds no fit)

- Push ladder: obj_1 alone, +y-face offsets 1.5/2.0/2.5 cm x2 plus one
  3.5 anchor, 5 steps, fresh battery. Output per push is the re-seat flag
  from the tick stream. Pre-registered prediction: the re-seat/accumulate
  flip sits at offset 2.0-2.5 cm if set by the car's 7 cm pusher face.
- Corridor floor: 1push/hard_099 and hmax2/hard_054 (8.36 cm),
  hmax2/hard_010 (8.76). Measures the real pass/fail gap width; the sim's
  8.0 cm constant is unverified, honest expected range 8.4-11 cm.
- Optional: weigh all objects, tilt-test friction. Characterization table
  only.

## Status

- 2026-08-23: pilot session, 3 scenes (v1), 3 successes, runtime bugs
  found.
- 2026-08-25: runtime fixes landed (nav-failure replan, bounded retarget),
  matrix re-picked on v2 labels.
- 2026-08-26: analysis pre-registered, baselines received and verified.
  Matrix runs completed: 0 of 28.
