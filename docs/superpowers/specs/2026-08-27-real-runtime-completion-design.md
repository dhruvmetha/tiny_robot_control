# Real Runtime Completion Design

**Date:** 2026-08-27

## Problem

The real `easy_020` retry exposed a terminal-semantics regression. After the
second push, the current camera scene was simulation-reachable, but the robot
was still 32.8 cm from the goal. `Runtime._autonomous_step()` calls
`NAMOPlanner.is_complete()` before requesting the next subgoal, and commit
`b4acc7f` changed `is_complete()` to return reachability directly. Runtime
therefore stopped before the planner's existing reachable-goal branch could
return a `NavigateSubgoal`.

The earlier implementation also had a different bug: `_planning_failed`
returned `True` from `is_complete()`, and an empty plan could be presented as
success. The correction must restore physical arrival without restoring either
false-completion path.

## Decision

NAMO runtime completion is physical arrival, in both real and simulated
runtime execution. Reachability is a transition condition that allows the
planner to dispatch navigation; it is not a terminal outcome.

The direct planning bridge remains unchanged. A planning-only call may report
that its simulated scene opens a path to the goal without executing a robot.
The distinction is between planning success and runtime task completion.

## Runtime Flow

1. Runtime finishes the active push and gives the latest observation to the
   planner.
2. `NAMOPlanner.is_complete()` returns true only when goal navigation has been
   dispatched and the observed robot is within 5 cm of the dispatched goal or
   approved retarget point.
3. If the goal is reachable but the robot has not arrived, `is_complete()` is
   false. Runtime calls `plan()`, which returns the existing `NavigateSubgoal`.
4. The existing navigation executor drives the real or simulated robot.
5. After successful navigation, runtime observes physical/simulated arrival,
   records success with reason `goal reached`, stops the robot, writes the
   summary, and shuts down.
6. If planning returns no action while the robot has not arrived, runtime
   records planning failure. `_planning_failed` never implies completion.

The existing nearby-goal retarget behavior remains supported. When the exact
goal point is geometrically covered, completion is measured against the
retarget point selected and dispatched by the planner; the diagnostic log must
continue to identify this as a retarget.

## Diagnostics and Experiment Records

Terminal console status and `summary.json` must use the same retained outcome.
Success is recorded as `goal reached`; reachability alone must never create a
successful summary. `final_robot_pose_cm` and `final_distance_to_goal_cm`
remain recorded so arrival can be audited.

The `real_exp` command must include `--capture-scene`, producing start/end JPG,
JSON, and XML snapshots in addition to plan, push, wheel-phase, connectivity,
and video artifacts. The accepted setup checksum and camera jitter are trial
provenance, but are not completion criteria.

`trial2` remains preserved as an incomplete attempt: it validly records two
successful pushes and a reachable opening, but it did not physically finish
the task and must not be used as a completed end-to-end trial.

## Testing

Regression tests will pin these behaviors:

- A reachable goal while the robot is 32.8 cm away is not complete.
- The next planner action in that state is a `NavigateSubgoal` to the goal.
- Arrival within 5 cm after successful goal navigation is complete.
- A planning failure remains incomplete and becomes a runtime failure.
- Runtime does not emit `Plan Complete` before navigation.
- Runtime summary success reason is `goal reached`.
- The `real_exp` launch command includes `--capture-scene` and continues to
  omit held-target flags.

Focused terminal, navigation, diagnostics, chain-reuse, and routing tests run
before the complete project suite. A hardware-free runtime regression using a
trial-like observation verifies the transition from reachable scene to
navigation dispatch without opening the serial port.

## Out of Scope

- Changing NAMO search, suffix reuse, or boundary matching.
- Reclassifying planning-only bridge results as physical trials.
- Appending a separate navigation process to `trial2`.
- Moving the robot during implementation or automated verification.

## Implementation Status

Implemented and verified on 2026-08-27.

TDD evidence:

- Planner RED: 2 failed, 5 passed. Both a reachable robot far from the goal
  and a navigated robot still 6 cm away were incorrectly complete.
- Planner GREEN: 12 passed across navigation-failure and goal-retarget tests.
- Runtime RED: 2 failed, 5 passed after pinning both the retained terminal
  outcome and legacy summary fallback to `goal reached`.
- Documentation RED: the real-exp command test failed because
  `--capture-scene` was absent.
- Focused GREEN: 29 passed across terminal outcomes, arrival/failure,
  retargeting, navigation failure, MPC suffix reuse, and simulation telemetry.
- Complete worktree suite: 429 passed in 51.22 seconds with the worktree
  source explicitly first on `PYTHONPATH`.

Hardware-free verification dispatched a fake `NavigateSubgoal` from a
reachable-but-distant observation and confirmed that no terminal outcome was
recorded before arrival. No `run_namo.py` or `check_build.py` process was
started, and `/dev/ttyACM0` had no owner after verification.
