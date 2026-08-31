# ICRA real-robot study brief

The record of what the real-robot section claims, what counts as success,
how each trial runs, and what gets written down. The companion operations
doc is [REAL_ROBOT_TRIALS.md](REAL_ROBOT_TRIALS.md); this one says what
counts, that one says how to run it.

Deadline: ICRA submission 2026-09-15.

## Design history

This file previously held a different design: 14 scenes crossing prior
(model/uniform) with execution mode (search/reactive), 56 single runs,
locked 2026-08-25/26 with three amendments. Dhruv superseded it on
2026-08-31 with zero of its 56 runs collected, so it counts as an
abandoned design, not a revision against data. The full text is in
history at `git show cc98de3:docs/ICRA_REAL_ROBOT_STUDY.md`. Its success
semantics, failure vocabulary, and physics-divergence record carry
forward below unchanged. Its trial matrix, execution-mode crossing, and
replacement tables do not.

## Claims

Primary claim: the method transfers to real problems. The method is
full_namo with best-first local search and the model prior, HY5U_s2.ckpt
as the single fixed checkpoint for every real trial. The comparison is a
uniform-prior ablation on the same scenes, with repeats. Every scene runs
a pure-navigation baseline first, so each scene in the comparison is one
a robot could not simply drive.

## Protocol, per scene

Scenes live under `real_exp/environments/twohop_selected/`; results under
`real_exp/results/formal_v2/`. Source `../namo_cpp/env.robotlearning.sh`
and export `NAMO_REPO` before anything plans, or every plan fails in
~30 ms with NO SUBGOALS. Every table run gets `--capture-scene` and an
external `timeout`.

1. Build the scene with the live checker, never from raw coordinates.
   Every twohop scene directory carries a `build_sheet.json`; point
   `check_build.py` at it and it watches the overhead camera and prints a
   live offset line per item while you nudge things into place:

   ```
   python scripts/check_build.py \
     --sheet real_exp/environments/twohop_selected/<group>/<scene>/build_sheet.json \
     --build-id <scene> --gui --auto 5
   ```

   Place each bar and block until its line reads OK (within 0.5 cm and
   2 deg). Twohop sheets carry no contact checksum, so all-items-OK is
   the bar. The same screen shows the robot start pose and the goal
   marker position. The first `--capture-scene` capture of step 2 is the
   reference layout.
2. Nav-penalty baseline, once, on the table:

   ```
   timeout 120 python scripts/run_navigation_baseline_robot.py \
     --config config/real.yaml --mode penalise \
     --camera-service tcp://localhost:5556 \
     --diag-path real_exp/results/formal_v2/<scene> \
     --run-name nav_penalise --capture-scene
   ```

   Whether the scene qualifies for NAMO is Dhruv's judgment call at the
   table, made after watching the drive. He decided this on 2026-08-31,
   replacing two earlier mechanical rules. Neither the exit code nor any
   row formula decides.

   The outcome row, `nav_baseline_penalise_<epoch>.json`, is the evidence
   to read before calling it. `reached` and `distance_to_goal_cm` say
   whether and how close the robot got. `objects_moved` and
   `objects_moved_cm` say what it shoved; a reached goal with shoved
   blocks is bulldozing, not driving. `stuck_retries` counts
   retreat-and-replan cycles from the 2026-08-31 change, `stuck_causes`
   names each one as blocked or under_commanded, and `route_crosses`
   lists what the planned route drove through.

   Record the call next to the row as
   `real_exp/results/formal_v2/<scene>/nav_verdict.json` holding
   `{"qualifies": true|false, "reason": "<one line>"}`, so the paper can
   say how each scene entered or left the comparison. The baseline run
   writes a skeleton with `qualifies: null` and the row's evidence copied
   in (d0d6095); fill it at the table, and it never overwrites a recorded
   verdict. This file is the only authoritative record of the decision;
   the row is evidence. Any analysis must fail loudly on a scene whose
   `qualifies` is still null. Null means nobody decided, never false, and
   treating it as false would silently drop scenes the same way the old
   exit-code rule did. The nav record stays in results either way.
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
4. Reset between trials by hand, verified by eye. Dhruv chose no
   tolerance gate on 2026-08-31. The per-run `--capture-scene` start
   capture stays on disk, so a suspect trial's actual layout can still be
   checked after the fact.
5. Recharge or swap the battery on a fixed schedule, not on symptoms. A
   low battery drops the radio under motor load.

## Success semantics

The runtime succeeds at the goal marker within the 5.0 cm goal tolerance.
When the marker's wavefront cell is blocked but a free reachable cell
exists within GOAL_RETARGET_CAP_CM = 12.0 of the marker, the executor
retargets there and arrival counts as success-with-retarget. The cap is
half the largest movable's long side, 7.5, plus robot inflation 3.5 plus
wavefront margin 0.5, rounded up; see `planner/namo_planner.py`. Beyond
the cap is a failure. Report strict-marker and with-retarget numbers side
by side; the gap counts plans that bury the marker under the pushed
block.

## Recording

Each run's diagnostics folder carries the verdict and the trace.
`summary.json` holds outcome, outcome_reason, duration, and final
distance to goal. `plans.jsonl` holds per-replan planning wall time and
simulation counts. `config.json` holds the full command, seed included.
The before/after scene captures sit alongside. When `timeout` kills a
run, the summary still lands with `outcome_reason: wall_clock_timeout`;
SIGTERM has been an ordinary shutdown since commit d6c3545.

Every failed trial also records `failure_cause` from the fixed
vocabulary: corridor_too_tight / marker_unreachable / overshoot_onto_goal
/ stall / radio_dropout / planning_failed / other. The human at the table
assigns it at verdict time.

Interventions are tiered clean / recovered / invalid. A recovered trial
counts in statistics but stays out of the video reel.

## Physics freeze and the measured divergence

Both humans froze the sim physics for the study on 2026-08-26: no fitting
of sim mass or friction to hardware, because every label both sides use
came from the current physics. The divergence is a result:

- Translation is calibrated. Both sides move the block ~4.4 cm per
  commanded step, and first contact lands ~14 ticks, 0.47 s, into a push,
  a fixed haircut on effective push length.
- Rotation is not. Hardware accumulates ~0.70 deg/cm of travel per cm of
  contact offset from the face centre; sim self-squares the block at
  every offset. Near-centre real contacts also self-square. Corner
  contacts never do.
- Consequence: open-loop replay of push chains diverges in a
  systematically optimistic direction. MPC re-observation between pushes
  is the measured mitigation, and it is why this protocol executes one
  push per plan.

## Open items before trial 1

- Decide `--hold-region-target`. The superseded design's Amendment 2
  argued that without it, each replan re-derives the region boundary from
  scratch. A setup push opens nothing by definition, so the next replan
  can aim at a different boundary and strand the push just made. That
  bites one-push-at-a-time execution on multi-push scenes specifically,
  and the command above passes neither that flag nor `--active-target`.
  Decide, then write the decision here.
- Pick the per-trial `timeout` value from the longest healthy run plus
  margin.
- Fix the battery schedule: after how many runs the battery gets swapped
  or charged.

## Status

- 2026-08-23: pilot session, 3 scenes on v1 build ids, 3 successes,
  runtime bugs found.
- 2026-08-25/26: crossed-matrix design locked and amended; later
  superseded with zero matrix runs collected.
- 2026-08-31: formal_v2 flow adopted, this document. Timeout summary fix
  landed at d6c3545. Scene qualification moved from formula to operator
  judgment at db9871d. Trials completed under this design: 0.
