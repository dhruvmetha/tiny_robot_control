# ICRA real-robot study brief

The living record of what the real-robot section claims, how success is
scored, which scenes run, and how the numbers get analyzed. Decisions here
were locked BEFORE data collection (2026-08-25/26, jointly with the sim
side) and changing them mid-study invalidates the pre-registration. Read
[Amendments to the locked design](#amendments-to-the-locked-design) before
acting on any section below, because Amendment 1 changed the trial matrix.
The companion operations doc is [REAL_ROBOT_TRIALS.md](REAL_ROBOT_TRIALS.md);
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

14 scenes, hard-dominated, all v2 sheet ids, all
marker-strict per `marker_retarget.csv`, all post-push corridors >= 11 cm.
Amendment 1 crossed execution mode over this list, so the run count is 56
and not the 28 written below:

- 1push hard: hard_037, hard_001, hard_012, hard_005, hard_009
- hmax2 hard (all needs_2_chain): hard_007, hard_013, hard_008
- 1push med: med_085, med_077, med_093
- hmax2 med: med_069
- 1push easy: easy_002, easy_001

Pairing: rebuild the scene between every cell and checksum-verify every build
with `check_build.py`. Crossing creates four cells per scene, so freeze a
randomized four-cell execution order for all 14 scenes before row 1. No
existing script or manifest currently generates or supplies that order; the
operator must preserve it as the execution ledger and must not select or
re-randomize cells after seeing results. Cut order if time slips: easy, then
medium, never below 8 hard pairs (the sign test rests on them). Amendment 1
restates that floor as 8 hard scenes fully crossed, and forbids cutting single
cells. Interventions are tiered clean / recovered / invalid; recovered counts
in statistics and is excluded from the video reel.

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
  How to read the result against the matrix (measured from
  `post_push_clearance.csv`, 2026-08-26): every matrix scene has a best
  corridor of 13.74 cm or better and thirteen of the fourteen sit at the
  30.0 cm ceiling, so no matrix scene becomes an expected failure anywhere
  in that range. Best corridor is the best route, not the only one. Three
  scenes have a worst route near the floor, easy_002 at 9.95 cm and both
  easy_001 and med_077 at 11.2 cm. If the ladder lands above 9.95 and a run
  on one of those three fails, `failure_cause` has to separate
  corridor_too_tight from marker_unreachable, because the two look alike at
  the table. Separately, 51 of the 593 shipped scenes have a best corridor
  under 9.9 cm; none of them are in the matrix.
  Where the pool is thin, if a scene ever needs replacing: the damage is
  not spread evenly across the hard tier. At a 9.9 cm threshold 1push/hard
  loses 31 of 100 scenes and hmax2/hard loses 12 of 93, so the 1push/hard
  pool is the fragile one and the matrix draws 5 of its 8 hard scenes from
  it. The sim side's `corridor_risk.csv` (namo_cpp handoff, d4675f7) bands
  every scene by the threshold that kills it. A replacement does not have
  to inherit that exposure, because the corridor is known per scene before
  building: 40 of the 100 1push/hard scenes sit at the 30 cm measurement
  ceiling and 30 of those are also marker-strict, so a conditioned draw has
  no corridor exposure at all. Replacing a matrix scene mid-study is still
  a design change and needs its own amendment, so this note records where
  to look rather than granting permission to swap.
- Optional: weigh all objects, tilt-test friction. Characterization table
  only.

## Amendments to the locked design

Anything below changes a decision the sections above state as locked. Each
entry records what changed, the date, and how much matrix data existed at
that moment.

An amendment written before the first matrix row lands is an amendment. The
same words written afterwards are a revision against data, which is a
different and much weaker thing.

PROVENANCE, AND IT QUALIFIES ALL THREE. Every amendment here records a
decision that reached this session through another session relaying it,
never from Dhruv directly. One such relay was already wrong once on
2026-08-26, on a smaller matter, and only the data caught it. So each entry
below states a decision Dhruv has not confirmed to the author of this
document. That is why they sit in an uncommitted working tree rather than in
the repository history. Committing them is his act of confirmation. Until he
does, read every amendment below as proposed rather than settled.

### Amendment 1: crossed execution-mode arm

Dhruv decided this on 2026-08-26 and the `real` session relayed it. No
matrix data existed when it was written. Zero of the original 28 runs had
been collected. The only rows in `real_trials/trials.csv` are the four
pilot trials from 2026-08-23, which ran on v1 build ids that the matrix
does not use.

What changed. The matrix now crosses execution mode with the prior arm.
Every scene runs both search and reactive execution, and each mode runs
the model prior and the uniform prior. That is 14 scenes x 2 priors x 2
modes = 56 hardware runs, up from 28.

Why. The measured divergence attacks lookahead and nothing else.
Translation is calibrated, rotation is not, so the second push of a
depth-2 plan contacts a face that has rotated away from where the plan
assumed. Open-loop replay therefore fails in a systematically optimistic
direction. Reactive execution re-decides from the camera at every step and
never reads a predicted state, so the gap cannot corrupt it the same way.
Nobody knows which of the two transfers better, and only the hardware runs
can settle it.

What this does not change. The primary claim, the success semantics, the
12.0 cm retarget cap, the paired-within-scene percentile analysis, the
physics freeze, and the calibration blocks all stand exactly as locked.
Reactive adds a second sign test. It does not replace the first.

Cut order under crossing. Each scene now contributes a block of four
runs, and dropping a single cell breaks the pairing for both sign tests at
once. So cut whole scenes and never individual cells. Cut easy first, then
medium. The floor is 8 hard scenes fully crossed, which is 32 runs and
holds both sign tests at n=8.

Recording requirement. `real_trials/trials.csv` must carry `arm`,
`exec_mode`, and `failure_cause` as parsed columns with fixed vocabularies
before the first matrix run. Today the arm survives only as free text
inside the `command` string. Crossing turns two ways to mislabel a row
into four. The frozen randomized order of all four cells for each scene must
also exist before row 1 and be preserved as the execution ledger. No current
script or manifest supplies it. Each completed row is appended immediately;
deviations are recorded rather than silently re-randomizing the remaining
cells.

### Amendment 2: both arms hold the region target

Dhruv decided this on 2026-08-26 and the `real` session relayed it. No matrix
data existed when it was written.

What changed. Both arms add `--hold-region-target` to the matrix command. The
command in [REAL_ROBOT_TRIALS.md](REAL_ROBOT_TRIALS.md) passed neither that
flag nor `--active-target`, and the flag defaults to off.

Why, first reason. `--exec-mode` reaches only `solve_boundary_from_xml`, and
only the held loop calls that method. `namo_planner.py:1314` branches on the
flag, so the documented command took the other path entirely. Without this
change the search arm would run the plain planning path while the reactive arm
ran the held-boundary loop. The two arms would then differ by code path rather
than by execution mode, and the sign test would report a pipeline difference as
a mode effect. That is worse than a broken arm, because it produces numbers.

Why, second and stronger reason. Holding the target is a correctness fix for
the search arm on its own. `planner/region_target.py` states the failure in its
module docstring: the planner normally re-derives the region graph, next
region, blocking object and target points on every replan, and executing one
physical push at a time makes that wrong, because a setup push opens nothing by
definition and can leave the next replan aimed at a different boundary,
stranding the work just performed. Stranding damages setup-then-finish chains
specifically, and `hmax2/hard` is 92 chains out of 93. The unheld path is
weakest exactly where the study's power sits.

The honest limit. The 2026-08-23 pilot ran unheld and its one two-push chain,
hard_004, closed on the table. So this removes a documented risk rather than
fixing an observed failure.

What it costs. The search arm no longer runs the code path the pilot ran. The
pilot trials were already outside matrix statistics because they use v1 build
ids, so nothing is lost, but no pilot result may be quoted as evidence about
the matrix search arm.

### Amendment 3: pre-named conditioned replacements

Dhruv decided this on 2026-08-26 and the `real` session relayed it. No matrix
data existed when it was written.

What changed. The design had no rule for a scene that will not build to
checksum at the table, and crossing gives four cells per scene and so four
times the chances to hit one. The rule is now to substitute that scene's
pre-named replacement, never to choose one at the table and never to drop the
scene. Two replacements per scene cover all 14, in
`handoff/real_scene_build_sheets_v2/matrix_replacements.csv` at namo_cpp
baefe75, assigned under seed 20260826 so the choice is reproducible and
pre-registered rather than made after a failure. That file was redrawn twice on
2026-08-26 as review tightened it; c1e08f4 and 076fcd4 are superseded and must
not be used.

Conditioning. Same axis and tier, best route at the 30 cm measurement ceiling,
marker verdict strict, not already in the matrix, no bare id shared with a
matrix scene, and no bare id repeated anywhere in the table. Candidates are
taken roomiest worst-route first rather than drawn uniformly, so within a row
replacement_1 is at least as roomy as replacement_2. Prefer replacement_1.

Verified here against `sheets_v2_a82a66a/` on 2026-08-26. All 28 named scenes
pass every condition. All 16 hard-tier replacements sit at 30.0 cm on both the
best and the worst measured route. The narrowest worst route anywhere in the
table is 12.38 cm, on `1push/easy_074`, and nothing falls under 11.5 cm.

Why conditioned rather than random. An unconditioned `1push/hard` draw carries
the same 31 in 100 best-route corridor exposure as the scene it replaces. A
conditioned draw beats a random one, and the roomiest-first ordering also
clears the worst route, which the earlier drafts did not. Two earlier versions
left residual worst-route exposure in the hard tier; this one does not.

The worst-route floor started at the 30 cm ceiling, relaxed to 11.0 cm because
`1push/easy` offered a single candidate for two scenes, then rose again once
selection took the roomiest candidates first. The sim side disclosed each
change rather than leaving it to be found later.

A trap for the table, kept although this draw does not trigger it. v2 build ids
are per-axis, so one id can name two different scenes. This table has no
repeat and no collision with a matrix scene, but the next redraw could
reintroduce one. Read axis and build id together, never the id alone.

Recording. A substitution goes in the trial's `notes` with both build ids, and
the replaced scene keeps its row so the matrix still shows 14 scenes attempted.

## Status

- 2026-08-23: pilot session, 3 scenes (v1), 3 successes, runtime bugs
  found.
- 2026-08-25: runtime fixes landed (nav-failure replan, bounded retarget),
  matrix re-picked on v2 labels.
- 2026-08-26: analysis pre-registered, baselines received and verified.
  Amendment 1 crossed the execution-mode arm, taking the matrix to 56 runs.
  Amendment 2 put both arms on the held-boundary loop; Amendment 3 pre-named
  conditioned replacements. Matrix runs completed: 0 of 56.
