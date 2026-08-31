# ICRA real-robot study brief

The living record of what the real-robot section claims, how success is
scored, how each trial runs, and what gets recorded. The companion
operations doc is [REAL_ROBOT_TRIALS.md](REAL_ROBOT_TRIALS.md); this one
says what counts, that one says how to run it.

Deadline: ICRA submission 2026-09-15.

## Design history

This file previously held a different design: 14 scenes crossing prior
(model/uniform) with execution mode (search/reactive), 56 single runs,
locked 2026-08-25/26 with three amendments. Dhruv superseded it on
2026-08-31 with zero of its 56 runs collected, which makes this an
abandoned design rather than a revision against data. The full text is in
history: `git show cc98de3:docs/ICRA_REAL_ROBOT_STUDY.md`. Its success
semantics, failure vocabulary, and physics-divergence record carry
forward below unchanged; its trial matrix, execution-mode crossing, and
replacement tables do not.

## Claims

Primary: the method (full_namo, best-first local search, model prior,
HY5U_s2.ckpt as the single fixed checkpoint for every real trial)
transfers to real problems, measured against the uniform-prior ablation
on the same scenes with repeats. Each scene also runs a pure-navigation
baseline first, so every scene in the NAMO comparison is certified to
need a push at all.

## Protocol, per scene

Scenes live under `real_exp/environments/twohop_selected/`; results under
`real_exp/results/formal_v2/`. Source `../namo_cpp/env.robotlearning.sh`
and export `NAMO_REPO` before anything plans, or every plan fails in
~30 ms with NO SUBGOALS. Every table run gets `--capture-scene` and an
external `timeout`.

1. Build the scene. The first capture is the reference layout.
2. Nav-penalty baseline, once, on the table:

   ```
   timeout 120 python scripts/run_navigation_baseline_robot.py \
     --config config/real.yaml --mode penalise \
     --camera-service tcp://localhost:5556 \
     --diag-path real_exp/results/formal_v2/<scene> \
     --run-name nav_penalise --capture-scene
   ```

   Exit 0 means driving alone reached the goal; the scene leaves the NAMO
   comparison but its record stays in results. Exit 2 means it needs NAMO.
3. Ten NAMO trials, alternating arms M-R-M-R-M-R-M-R-M-R. Never 5+5
   blocks, so battery sag and session drift spread across both arms:

   ```
   timeout <N> python scripts/run_namo.py --config config/real.yaml \
     --camera-service tcp://localhost:5556 --headless \
     --local-search best_first --best-first-prior model \
     --scorer-ckpt <HY5U_s2 path> --shuffle-seed <k> \
     --diag-path real_exp/results/formal_v2/<scene> \
     --run-name model_r<i> --capture-scene
   ```

   The uniform arm is the same command with `--best-first-prior uniform`
   and no checkpoint. MPC execution is the default: one push per plan,
   then replan from a fresh observation. Pass `--shuffle-seed` explicitly
   on every trial; that is what lands it in `config.json`.
4. Reset between trials is by hand and verified by eye (Dhruv's call,
   2026-08-31, no tolerance gate). The per-run `--capture-scene` start
   capture stays on disk so a suspect trial's actual layout can be
   checked after the fact.
5. Recharge or swap the battery on a fixed schedule, not on symptoms; a
   low battery drops the radio under motor load.

## Success semantics

The runtime succeeds at the goal marker within the 5.0 cm goal tolerance.
When the marker's wavefront cell is blocked but a free reachable cell
exists within GOAL_RETARGET_CAP_CM = 12.0 of the marker, the executor
retargets there and arrival counts as success-with-retarget (cap
derivation: half the largest movable's long side 7.5 + robot inflation
3.5 + wavefront margin 0.5, rounded up; see `planner/namo_planner.py`).
Beyond the cap is a failure. Strict-marker and with-retarget numbers are
reported side by side; the gap counts plans that bury the marker under
the pushed block.

## Recording

Each run's diagnostics folder carries the verdict and the trace:
`summary.json` (outcome, outcome_reason, duration, final distance to
goal), `plans.jsonl` (per-replan planning wall time and simulation
counts), `config.json` (full command including the seed), and the
before/after scene captures. A run killed by `timeout` writes
`outcome_reason: wall_clock_timeout`; SIGTERM lands as an ordinary
shutdown as of commit d6c3545.

Every failed trial additionally records `failure_cause` from the fixed
vocabulary: corridor_too_tight / marker_unreachable / overshoot_onto_goal
/ stall / radio_dropout / planning_failed / other. The human at the table
assigns it at verdict time.

Interventions are tiered clean / recovered / invalid; recovered counts in
statistics and is excluded from the video reel.

## Physics freeze and the measured divergence

Sim physics is frozen for the study (both humans' decision, 2026-08-26):
no fitting of sim mass or friction to hardware, because every label both
sides use came from the current physics. The divergence is a result:

- Translation is calibrated: ~4.4 cm of block travel per commanded step
  on both sides; first contact lands ~14 ticks (0.47 s) into a push, a
  fixed haircut on effective push length.
- Rotation is not: hardware accumulates ~0.70 deg/cm of travel per cm of
  contact offset from the face centre; sim self-squares the block at
  every offset. Near-centre real contacts also self-square; corner
  contacts never do.
- Consequence: open-loop replay of push chains diverges in a
  systematically optimistic direction. MPC re-observation between pushes
  is the measured mitigation, and it is why this protocol executes one
  push per plan.

## Open items before trial 1

- Decide `--hold-region-target`. The superseded design's Amendment 2
  argued that without it, each replan re-derives the region boundary and
  a setup push that opens nothing can strand the work just done, which
  bites one-push-at-a-time execution on multi-push scenes specifically.
  The command above passes neither that flag nor `--active-target`.
  Decide, then write the decision here.
- Pick the per-trial `timeout` value from the longest healthy run plus
  margin.
- Fix the battery schedule (every how many runs).

## Status

- 2026-08-23: pilot session, 3 scenes (v1), 3 successes, runtime bugs
  found.
- 2026-08-25/26: crossed-matrix design locked and amended; later
  superseded with zero matrix runs collected.
- 2026-08-31: formal_v2 flow adopted (this document). Timeout summary fix
  landed (d6c3545). Trials completed under this design: 0.
