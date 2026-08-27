# Real Experiment Terminal Semantics — Design

Status: revised after review; awaiting implementation approval.

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

## Confirmed defects

The failed `easy_020` run violates both the old physical-arrival criterion and
the new reachability criterion. Its false `Plan Complete` message is not
caused by choosing the wrong success criterion. It follows from multiple
inconsistent terminal paths:

1. `NAMOPlanner.is_complete()` returns true when `_planning_failed` is true,
   conflating failure with success.
2. `Runtime._autonomous_step()` unconditionally returns `Plan Complete` when
   `planner.plan()` returns no subgoal, without checking reachability or
   failure.
3. The held-boundary path can return no subgoal without setting
   `_planning_failed`. That is the path this run took after
   `ambiguous_boundary` exhausted boundary advancement.
4. The control loop has a terminal-success branch but no corresponding
   terminal-planning-failure branch.
5. Console status and structured outcome use separate rules. In this run the
   console said `Plan complete`, while `summary.json` said `aborted` because
   the robot was still far from the goal and `_planning_failed` was false.

The fix must remove every false-success route. In particular:

- planning failure is never planner completion;
- no returned subgoal is never sufficient evidence of success;
- a planner that returns no subgoal while the goal is unreachable is marked
  failed even if its internal branch forgot to set `_planning_failed`;
- the console, shutdown trigger, and structured summary consume the same
  terminal decision.

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

A single runtime terminal-result value must be retained once either outcome is
reached and reused when writing `summary.json`; shutdown must not independently
reinterpret the scene using different rules.

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
- an unreachable scene plus no returned subgoal yields `Planning Failed`, not
  `Plan Complete`;
- the terminal failure is written as a failure outcome;
- existing chain/suffix reuse tests remain green;
- `real_exp/README.md` no longer enables held-region targeting.

A replay of the captured post-push `easy_020` XML must be run without hardware
motion to confirm that fresh boundary selection reaches the full-search path.
The replay must also confirm that an unreachable no-plan result is terminal
failure and cannot emit `Plan Complete`, even if the planner's internal
`_planning_failed` flag was not set.
