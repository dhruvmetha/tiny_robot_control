# Real-robot trial procedure

How to run a NAMO trial on the physical table, from empty table to recorded
verdict. Distilled from the 2026-08-23 pilot session (easy_002, med_002,
hard_004, all successful) and the protocol decisions locked with the sim
side on 2026-08-25/26. Follow it in order; every step exists because
skipping it cost a trial once.

## One-time context

Scenes come from `namo_cpp/handoff/real_scene_build_sheets_v2/` (v2, the
exhaustive labels). v1 sheets one directory up are superseded; v1 and v2
build ids do NOT refer to the same scenes, join on the `xml` column across
versions. A local verified copy plus the offset baselines lives in
`real_trials/sheets_v2_a82a66a/`.

Success semantics: the runtime succeeds at the goal marker within 5.0 cm,
or, when the marker's cell is wavefront-blocked but a free reachable cell
exists within 12.0 cm (GOAL_RETARGET_CAP_CM in
`src/robot_control/planner/namo_planner.py`, derivation in the constant's
comment), at that cell, logged as success-with-retarget. Report strict and
with-retarget side by side.

Sim physics is frozen for the study. Never propose fitting sim mass or
friction to hardware data; every tier label depends on the current physics.
The measured divergence (rotation coupling ~0.70 deg/cm per cm of contact
offset; sim self-squares at every offset) is a finding, not an error.

## Division of labor

Worked out during the pilot and it worked: the agent (Claude session)
drives ALL software, the human drives ALL matter. The human should never
need a terminal during a session.

Agent: launches and monitors the camera service check, the build checker
(GUI windows on the desktop display), and every run_namo trial; watches
logs and calls out placement moves in table language ("wall_10 left 19,
turn clockwise 83"); verifies the scene checksum before launch; kills
wedged or zombie processes; records the trials.csv row; asks the human
for the verdict and failure cause after every trial; streams calibration
data to the sim side; keeps the study brief's status current.

Human: places and nudges bricks until the checker windows go green; sets
the robot on its start circle; swaps or charges the battery; rescues a
physically stuck robot; renders the success/fail verdict and picks the
failure cause from the fixed list (the human is the only observer who can
tell a stall from a radio drop); decides accept-vs-nudge on MARGINAL
checksums and rerun-vs-move-on after failures.

The verdict is always the human's. The agent proposes, logs, and
launches; it never overrides a table call.

## Preflight, every session

1. Environment. Source from the namo_cpp ROOT, then export the repo
   override (sourcing from elsewhere sets NAMO_REPO wrong):

       cd ../namo_cpp && set -a && . env.robotlearning.sh && set +a && cd -
       export NAMO_REPO=$HOME/projects_dhruv/namo/namo_cpp

   Skipping this makes every planning attempt return NO SUBGOALS in ~30 ms
   with the real error hidden unless -v is passed. An instant planning
   failure is an environment problem, not a planner problem.

2. Camera service, one instance:

       python scripts/camera_service.py --config config/real.yaml

3. Serial port has ONE owner. Before any launch:

       fuser /dev/ttyACM0

   Two processes on the port silently garble the radio (alive_ids stays
   empty). Kill stale run_namo processes; they sometimes outlive their
   terminal.

4. Battery. A low battery drops the radio under motor load and stalls
   rotations (the drivetrain has a ~0.3 command deadband). Swap or charge
   before a session, not during a trial. Mid-run swap is recoverable (MPC
   replans from camera) but marks the trial `recovered`.

5. Bricks on hand: wall_9 (19.0 cm, odd tag mounting), wall_10, wall_11,
   obj_1, obj_4. wall_9 may substitute for one 19.5 cm bar per scene;
   checksum-verify the substitution with a variant sheet first (precedent:
   `real_trials/med_002_wall9.csv`).

## Build the scene

    PYTHONPATH=src python scripts/check_build.py \
        --sheet <sheet.csv> --build-id <id> --gui --auto 10

Two windows open: a schematic (red = current, green = target, arrows = the
move) and the raw feed. Nudge bricks until lines go green; the contact
checksum runs every 10 s and the window closes itself on PASS. MARGINAL at
1-3 contacts moved is normal for hand placement (builds land within 0.3 cm
and 2 deg); accept it and note it. Robot goes on the purple circle, any
yaw. Cards with a red hatch mark items with under 0.5 cm of slack; place
those last and carefully.

## Run the trial

    NAMO_PUSH_WHEEL_LOG=real_trials/<scene>/trialN/push_phases.jsonl \
    PYTHONPATH=$NAMO_REPO/build_python:src python -u scripts/run_namo.py \
        --config config/real.yaml --camera-service tcp://localhost:5556 \
        --robot-model car --algorithm full_namo \
        --local-search best_first --best-first-prior model \
        --scorer-ckpt ~/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt \
        --goal <goal_x goal_y from the sheet row> \
        --no-shuffle-edges --max-chain-depth 2 --record-video \
        --diag-path real_trials/<scene>/trialN --run-name "{time}_bestfirst_car"

HY5U_s2 is the single fixed seed for every real trial; the uniform arm
replaces `--best-first-prior model` with `uniform` and drops the ckpt.
Goals are per-scene from the sheet, not a constant.

## Record the verdict

One row per trial in `real_trials/trials.csv`. The human's verdict is
ground truth, not the planner's exit code. On failure, set `failure_cause`
from the fixed vocabulary: corridor_too_tight / marker_unreachable /
overshoot_onto_goal / stall / radio_dropout / planning_failed / other.
Interventions are tiered: clean / recovered (task unchanged, e.g. battery
swap) / invalid (task changed). Recovered trials count in statistics and
are excluded from the video reel.

Videos, per-subgoal meta, plans/pushes/subgoals jsonl land under
`real_trials/<scene>/trialN/` automatically.

## Matrix protocol (ICRA, due 2026-09-15)

14 scenes, both arms per scene, rebuild between arms, checksum-verify both
builds, randomize which arm runs first. Scene list, cut order, and the
locked analysis design (paired within scene, percentile statistic, no
pooled means) live in the session memory and in
`real_trials/sheets_v2_a82a66a/uniform_offset_baseline.csv`. Hard pairs
outrank everything; never drop below 8.

## Calibration blocks

Push ladder (~15 min): obj_1 alone, +y-face pushes at offsets 1.5/2.0/2.5
cm x2 plus one 3.5 cm anchor, all 5 steps, fresh battery, via
`scripts/execute_real_push.py`. The recorded outcome per push is the
re-seat flag (rotates ~3 deg then flat vs steady accumulation), from the
tick streams, not an end-to-end average.

Corridor floor (~3 scenes): 1push/hard_099, hmax2/hard_054 (8.36 cm),
hmax2/hard_010 (8.76). Measures the car's real pass/fail gap width; the
sim's 8.0 cm constant is unverified and the honest expected range is
8.4-11.

Optional (5 min): weigh all five objects, tilt-test obj_1 on the table
board (mu = tan of slide angle). For the paper's characterization table
only; nothing feeds a sim fit.

## Known failure modes, fastest diagnosis first

- Planning fails instantly (~30 ms): environment not sourced. See preflight 1.
- Robot connected but motionless, log stalls: second process on the serial
  port, or battery. `fuser /dev/ttyACM0`, then swap battery.
- Rotation stalls 10-20 deg short, then recovers or loops: command
  deadband, worse on low battery.
- "GOAL BLOCKED" after a push with the region visibly open: fixed in the
  2026-08-25 runtime commits (bounded retarget); if it recurs, it is a
  regression, not a scene problem.
- A stall in a narrow corridor is a hardware failure mode the sim cannot
  produce. Record it as `stall`, never report it to the sim side as label
  noise.
