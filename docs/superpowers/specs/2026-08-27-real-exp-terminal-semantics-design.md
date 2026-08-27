# Real Experiment Terminal Semantics — Design

Status: approved behavior for implementation.

## Objective

Make the `real_exp` trials terminate according to NAMO reachability rather
than treating an empty plan as success. Keep MPC suffix reuse, and run these
trials without cross-snapshot held-region targeting.

## Scope

This change has two parts:

1. Remove `--hold-region-target` from the command documented in
   `real_exp/README.md`. The held-target implementation remains available for
   other workflows and is not redesigned here.
2. Correct autonomous terminal-state handling so that simulated goal
   reachability is success and planning exhaustion is failure.

Suffix/full-chain verification and reuse remain unchanged.

## Success semantics

At each idle planning boundary, the planner evaluates the latest observation
with the existing C++ wavefront reachability analysis. The observation is
converted to the planning simulation in the same way as a normal NAMO plan.

- If the goal is collision-free reachable without another push, the NAMO task
  is complete. The runtime stops; physical navigation to the goal is not
  required for these experiments.
- If the goal is not reachable and a push subgoal is available, execution
  continues.
- If the goal is not reachable and planning returns no subgoal, planning has
  failed. The runtime stops with a failure status and the summary records a
  failure, never success.

An empty plan is therefore not a terminal-success signal. Reachability is.

## Runtime flow

After a subgoal finishes, or on the initial planning tick:

1. If a verified push suffix is pending, the existing MPC reuse path remains
   eligible when planning is requested.
2. Ask the planner whether the goal is reachable in the current simulated
   scene.
3. If reachable, return `Plan Complete` and stop.
4. Otherwise request the next subgoal.
5. If no subgoal is returned, return `Planning Failed` and stop.

The runtime must handle `Planning Failed` as a terminal status so the GUI,
video recorder, summary writer, and serial shutdown all complete normally.
The outcome is `failure`; `Plan Complete` is reserved for reachability.

## Error handling

Planning failure remains distinguishable from operator abort and process
shutdown. Existing structured plan diagnostics retain planner-specific
failure details such as `ambiguous_boundary`; the run-level terminal reason
must at minimum state that planning returned no subgoal while the goal was
unreachable.

## Tests

Regression coverage must demonstrate:

- `_planning_failed` is not reported as planner completion.
- simulated goal reachability is reported as completion even when the robot
  has not physically arrived at the goal;
