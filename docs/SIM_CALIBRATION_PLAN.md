# Simulation calibration guide

The filename is retained for incoming links; this document describes the
implemented real-to-MuJoCo calibration workflow and the current checked-in
configuration. It is not an implementation plan.

The goal is to reconstruct the recorded scene for a real push, choose an
explicit simulated robot initialization, compare real and simulated object
displacement, and tune the MuJoCo push duration only when the collected
evidence supports that change.

## Current configuration snapshot

These values were verified in this checkout and its sibling `namo_cpp`
checkout. Treat them as a snapshot: inspect both YAML files again before a new
calibration session.

| Layer | Configuration | Current value |
|---|---|---:|
| Real `PUSHING` path-follower cap | `config/controller.yaml` -> `push.max_speed` | `0.4` normalized |
| Real ticks per NAMO unit | `config/controller.yaml` -> `push.push_steps` | `30` runtime ticks |
| Real direction mode | `config/controller.yaml` -> `push.dynamic_direction` | `false` |
| Real edge sampling | `config/controller.yaml` -> `push.points_per_face` | `15` per face |
| Sim ticks per NAMO unit | `../namo_cpp/config/namo_config_complete_skill15_car_1x.yaml` -> `skill.control_steps_per_push` | `550` MuJoCo ticks |
| Sim direction mode | sibling YAML -> `skill.dynamic_direction` | `false` |
| Sim edge sampling | sibling YAML -> `skill.points_per_face` | `15` per face |

[`execute_sim_push.py`](../scripts/execute_sim_push.py) selects that sibling
car configuration directly. It does not use `--push-speed` or `--nav-speed` in
C++ replay mode; those retained options are deprecated no-ops there.

## Keep the two clocks separate

The real controller and C++ simulator count different events:

```text
robot_control PUSHING ticks
  = PushSubgoal.push_steps * controller.yaml push.push_steps

MuJoCo physics ticks
  = (Action.depth + 1) * namo_cpp skill.control_steps_per_push
```

The general `robot_control` runtime targets 30 control iterations per second.
The generated car XML and checked-in nominal car scenes use a MuJoCo timestep
of 0.002 seconds. With the snapshot above, one simulated push unit therefore
requests 550 physics steps, or 1.10 seconds of push-loop simulation before
settling overhead. This conversion applies to the current car XML; always use
the actual model timestep when inspecting a different XML.

The counters should not be copied from one layer to the other. Compare final
poses and timestamps, then tune the layer responsible for the measured gap.
The complete real-controller multiplication and collection procedure is in
[Push Duration Calibration](./PUSH_DURATION_CALIBRATION.md).

## Implemented tools

All examples assume the repository root as the working directory and
`PYTHONPATH=src` in the active robotics environment.

### Collect real pushes

[`execute_real_push.py`](../scripts/execute_real_push.py) consumes a YAML trial
list and runs the same `Runtime`, approach controller, push controller, and
diagnostics path used for real execution. Important inputs are:

- `--config` and optional `--objects-path`;
- `--camera-service`;
- `--trial-spec` with `edge_idx` and `push_steps` or `depth`;
- `--diag-path` and `--run-name`;
- optional `--push-speed` and `--nav-speed` path-follower overrides (not
  global phase or hardware ceilings); and
- `--no-reset-check`, `--marker-lock`, `--capture-scene`, `--record-video`,
  and `--headless` when their documented behavior is required.

The run writes `pushes.jsonl`, `subgoals.jsonl`, `mid_obs.jsonl`,
`wheel_commands.jsonl`, per-push `tier2_push_trials/`, configuration metadata,
and logs. `pushes.jsonl` contains the compact before/after object and robot
poses used by the comparison tool; `mid_obs.jsonl` provides the full recorded
scene needed to reconstruct a push-start state.

### Replay in MuJoCo

[`execute_sim_push.py`](../scripts/execute_sim_push.py) dispatches through the
C++ `NAMOPushController` and records corresponding simulation artifacts. The
recommended calibration input is:

```bash
python scripts/execute_sim_push.py \
  --from-real-run calibration/push_calibration/obj_1/edge14/depth2/trial1 \
  --push-index 0 \
  --starting-pose-mode edge \
  --diag-path calibration/push_calibration/obj_1/edge14/depth2 \
  --run-name sim \
  --no-video
```

`--from-real-run` uses the selected record's edge, depth, object pose, and
recorded scene objects. It requires the source run's `pushes.jsonl`,
`subgoals.jsonl`, and `mid_obs.jsonl`. The recorded robot pose is available,
but it is not the default simulation start:

- `--starting-pose-mode edge` is the default. After reconstructing the scene,
  the script pre-teleports the car to the computed pose for the selected push
  edge.
- `--starting-pose-mode recorded` pre-teleports the car to the recorded robot
  pose instead.

The command above keeps `edge` explicit because this guide's duration workflow
calibrates the simulated primitive from a standardized contact pose and
matches `tune_control_steps.py`, which currently always uses the default edge
mode. For an apples-to-apples robot-start diagnostic, run a separate replay
with `recorded`; do not mix that result into a tree the tuner will overwrite.
`--no-video` skips MP4 encoding without skipping physics or calibration
artifact extraction.

The script also supports a direct `--mujoco-xml`, `--edge-idx`, and `--depth`
invocation, plus a continuous `--trial-spec` mode with `--real-run-dir`. Trial
spec mode reconstructs its initial scene from that real run's first
`mid_obs.jsonl` frame. `edge` starts the car at the first trial's computed edge
pose; `recorded` starts it at the robot pose in that frame. Subsequent pushes
share one continuously evolving physics state.

### Compare a tree

[`diff_real_vs_sim.py`](../scripts/diff_real_vs_sim.py) expects real repeats in
`edge<E>/depth<D>/trialN/` and one sim result in the sibling `sim/` directory:

```bash
python scripts/diff_real_vs_sim.py \
  --root calibration/push_calibration/obj_1
```

For each real repeat it computes:

- Euclidean gap between real and sim object XY displacement;
- wrapped absolute gap between real and sim object rotation;
- robot XY and rotation gaps for diagnosis; and
- differences in recorded controller settings.

The per-leaf loss averages the real repeats, and the headline averages leaf
losses using `object_xy_gap + theta_weight * object_theta_gap`. The default
angle weight is 0.5 centimeters per degree and can be changed with
`--theta-weight`. Robot gaps do not enter that loss.

Read outputs in this order:

1. `_unmatched.json` for missing real or sim leaves;
2. the controller-setting drift section of `comparison.md`;
3. `_per_trial.csv` for variance and outliers;
4. `_per_leaf.csv` for edge/depth structure; and
5. `_aggregate.json` and the headline only after those checks pass.

### Tune simulated duration

[`tune_control_steps.py`](../scripts/tune_control_steps.py) automates C++
replay and comparison while searching `skill.control_steps_per_push`:

```bash
python scripts/tune_control_steps.py \
  --root calibration/push_calibration/obj_1 \
  --yaml ../namo_cpp/config/namo_config_complete_skill15_car_1x.yaml \
  --skip-final-video
```

The tuner temporarily patches the supplied YAML, recreates each leaf's
`sim/` output, runs the diff, and steers its bracket using signed displacement
along the push direction. It restores the YAML on normal exit and handled
termination, and records the search history under the root's
`tune_control_steps/` directory. Preserve an independent clean Git state and
verify the restored YAML after any forced process termination.

## Recommended session sequence

1. Inspect the two configuration files and record their Git revisions in the
   session notes.
2. Choose representative object, edge, and depth leaves. Use multiple real
   repeats per leaf with unchanged surface, battery condition, object, start
   pose policy, `PUSHING` path-follower cap, other phase speeds, direction
   mode, and edge sampling.
3. Store new artifacts below `calibration/`, `recordings/`, or
   `chassis_calibration/`; these roots are ignored by Git.
4. Reconstruct one recorded push-start scene into `sim/` for every leaf and
   record whether the car was initialized in `edge` or `recorded` mode. Use
   `edge` for the tuner-compatible duration workflow above.
5. Run the diff and resolve unmatched leaves or controller-setting drift.
6. Inspect translation and rotation separately. Do not use duration to mask a
   contact-geometry or heading problem.
7. Run the tuner only when displacement along the push direction changes
   consistently with duration and real-trial variance is low enough to guide
   the search.
8. Re-run the full comparison after selecting a candidate, including any
   intentionally excluded leaves, and retain the exact config snapshots with
   the session artifacts.

Fresh experiment outputs are ignored artifacts by default. The checked-in
[object-1 comparison](../push_calibration/obj_1/diff/comparison.md) is a
historical dataset retained for reference. Its report contains a
controller-setting drift warning, so it demonstrates the output schema but
does not validate the current `30`-tick real or `550`-tick simulation
settings.

For the control path that produces normalized wheel commands, see
[Navigation and Wheel Commands](./NAVIGATION_AND_WHEEL_COMMANDS.md). For edge
numbering and contact construction, see [Push Geometry](./PUSH_GEOMETRY.md).
