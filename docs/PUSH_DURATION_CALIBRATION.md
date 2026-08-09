# Push duration calibration

This is the operator guide for measuring and tuning the duration of a push
primitive in the current checkout. It covers the real `robot_control` tick
budget and the separate MuJoCo tick budget used by `namo_cpp`.

Run all commands from the `robot_control` repository root in an environment
that provides the repository's robotics dependencies. Real collection also
requires the camera service and robot serial hardware; simulation requires the
built sibling `namo_cpp` bindings and MuJoCo assets.

## The two meanings of push steps

The names are similar, but the counters belong to different control loops.

### Real `robot_control` execution

A planner or trial spec emits a
[`PushSubgoal`](../src/robot_control/core/types.py) with `push_steps`, meaning
the number of NAMO push units. During `PUSHING`,
[`PushController`](../src/robot_control/controller/push.py) computes:

```text
real push ticks = PushSubgoal.push_steps * push.push_steps
```

The second term is the per-unit setting in
[`config/controller.yaml`](../config/controller.yaml). The checked-in runtime
snapshot is:

| Field | Current value | Meaning |
|---|---:|---|
| `push.max_speed` | `0.4` | Normalized path-follower cap during `PUSHING` |
| `push.push_steps` | `30` | `robot_control` ticks per NAMO push unit |
| `push.dynamic_direction` | `false` | Construct the push direction at the first tick and keep that path |
| `push.points_per_face` | `15` | 60 indexed edge points across four faces |
| `RuntimeConfig.frequency` | `30` Hz | Default runtime control-loop target |

For example, a trial entry with `push_steps: 3` requests `3 * 30 = 90`
`PUSHING` ticks. At the default target frequency that is nominally three
seconds. It is a configured tick budget, not a measured wall-clock guarantee:
camera delivery, process scheduling, and the real sender can affect observed
timing. Use timestamps in the generated logs when elapsed time matters.

`--push-speed` changes only the push path follower used during `PUSHING`.
`APPROACHING` uses `NavigationController`; `ADVANCING` uses
`push.advance_speed`; and `RETREATING` uses its retreat settings or the
navigation controller for a forward target. Likewise, `--nav-speed` changes
the navigation path follower used during approach or navigation-based retreat,
but not the navigation controller's independent rotation speeds. Lowering
`--push-speed` and `--nav-speed` therefore does not establish a global
real-robot wheel-command or hardware ceiling. Neither override changes the
per-unit `push.push_steps` value. Recollect real trials after changing any of
these settings.

With `dynamic_direction: false`, object movement does not cause the controller
to regenerate the push path on every tick. See
[Push Geometry](./PUSH_GEOMETRY.md) for edge generation and push direction.

### MuJoCo execution

`execute_sim_push.py` converts `depth` to `push_steps = depth + 1`. The C++
push controller then uses:

```text
MuJoCo push ticks = push_steps * skill.control_steps_per_push
```

Those are MuJoCo physics steps, not `robot_control` runtime ticks. The current
value and simulation workflow are recorded in the
[simulation calibration guide](./SIM_CALIBRATION_PLAN.md). Do not make the two
per-unit counters numerically equal: tune each against observed outcomes in
its own loop.

## What the tools record

[`scripts/execute_real_push.py`](../scripts/execute_real_push.py) runs the
normal real `Runtime` and `PushController` with a trial-list planner. A trial
spec contains `edge_idx` plus either `push_steps` or `depth`; an optional
`object_id` pins the target. Each run writes, among other diagnostics:

- `pushes.jsonl`: before/after object and robot poses, deltas, edge, push
  units, and controller settings;
- `subgoals.jsonl`: dispatch and completion outcomes;
- `mid_obs.jsonl`: timestamped scene observations;
- `wheel_commands.jsonl`: normalized per-tick commands and controller phase;
- `tier2_push_trials/`: per-push pose, command, path, metadata, and metrics
  bundles; and
- `config.json` and `run.log`: invocation and repository state.

[`scripts/execute_sim_push.py`](../scripts/execute_sim_push.py) replays one
recorded push through the C++ `NAMOPushController`. In `--from-real-run` mode,
it requires the real run's recorded scene stream and reconstructs the object
and scene state at the push selected by `--push-index`. Robot initialization
is a separate choice: `--starting-pose-mode edge` is the default and
pre-teleports the car to the computed contact edge, while
`--starting-pose-mode recorded` uses the recorded robot pose. It writes the
corresponding sim diagnostics and optional MP4.

The same option applies to continuous `--trial-spec` replay. In that mode the
scene comes from the first `mid_obs.jsonl` frame under `--real-run-dir`;
`edge` initializes the car at the first trial's computed edge pose and
`recorded` uses the robot pose from that frame. Physics then evolves
continuously between the remaining trials.

[`scripts/diff_real_vs_sim.py`](../scripts/diff_real_vs_sim.py) reads a
calibration tree's `pushes.jsonl` files. Its headline loss is the mean across
leaves of:

```text
object xy gap + theta_weight * object angle gap
```

Object pose drives the loss. Robot position and angle gaps are reported for
diagnosis but do not contribute. The report also flags controller-setting
drift; resolve any such drift before treating the loss as calibration
evidence.

[`scripts/tune_control_steps.py`](../scripts/tune_control_steps.py) repeatedly
replays the leaves, runs the diff, and searches the sibling YAML's
`control_steps_per_push`. Its signed signal projects sim and real displacement
onto each push direction: positive means sim moved farther, while negative
means real moved farther. The script saves and restores the YAML around the
search and writes its history under `tune_control_steps/`.

## Reproducible collection and comparison workflow

Use an ignored root such as `calibration/`; `.gitignore` also excludes the
standard `recordings/`, `chassis_calibration/`, and `output_dump/` roots. Do
not add a fresh run to Git unless it has been intentionally curated as a
small, durable fixture.

The diff tool expects this organization:

```text
calibration/push_calibration/<object>/
  edge<E>/depth<D>/
    trial1/pushes.jsonl
    trial2/pushes.jsonl
    ...
    sim/pushes.jsonl
```

The following example uses the checked-in one-push trial spec for edge 14 and
depth 2 (`push_steps: 3`). Export `PYTHONPATH` once in the shell:

```bash
export PYTHONPATH=src
```

Replace the `obj_1` directory label if the scene's single auto-detected movable
object has a different ID.

1. Collect a real repeat. Change `--run-name` to `trial2`, `trial3`, and so on
   for independent repeats. Omitting `--no-reset-check` enforces the target in
   the trial spec; retaining it records a push from the observed start pose.

```bash
python scripts/execute_real_push.py \
  --config config/real.yaml \
  --camera-service tcp://localhost:5556 \
  --trial-spec config/calibration/edge14_depth2.yaml \
  --diag-path calibration/push_calibration/obj_1/edge14/depth2 \
  --run-name trial1 \
  --no-reset-check \
  --headless
```

2. Replay a representative real repeat into the leaf's `sim/` directory. The
   real run contains one push, so its index is zero. This duration-calibration
   workflow chooses `edge` explicitly: it standardizes the simulated primitive
   at its intended contact pose and matches the fixed behavior of
   `tune_control_steps.py`. It reconstructs the recorded object and surrounding
   scene, but intentionally does not use the recorded robot pose.

```bash
python scripts/execute_sim_push.py \
  --from-real-run calibration/push_calibration/obj_1/edge14/depth2/trial1 \
  --push-index 0 \
  --starting-pose-mode edge \
  --diag-path calibration/push_calibration/obj_1/edge14/depth2 \
  --run-name sim \
  --no-video
```

3. After adding every intended edge/depth leaf, generate the comparison.

```bash
python scripts/diff_real_vs_sim.py \
  --root calibration/push_calibration/obj_1
```

The default `diff/` output contains `_per_trial.csv`, `_per_leaf.csv`,
`_aggregate.json`, `_unmatched.json`, `comparison.md`, and optional plots.
Inspect unmatched leaves and controller drift before the headline loss.

For an apples-to-apples diagnostic that also starts the car at its recorded
pose, replace `edge` with `recorded`. Keep that result separate from the
duration-tuning tree: the current tuner invokes `execute_sim_push.py` without
a pose-mode override and will overwrite each `sim/` leaf using the default
`edge` mode.

4. To search the MuJoCo duration against the fixed real dataset, point the
   tuner at the sibling configuration. The tuner overwrites each leaf's
   `sim/` result during the search and restores the YAML when it exits.

```bash
python scripts/tune_control_steps.py \
  --root calibration/push_calibration/obj_1 \
  --yaml ../namo_cpp/config/namo_config_complete_skill15_car_1x.yaml \
  --skip-final-video
```

Use `--exclude-edges` only with a recorded, defensible reason; excluded leaves
remain in the diff but do not steer the search. Use `--epsilon` only when a
fixed tolerance is justified; otherwise the tool derives one from real-trial
variance.

## Interpreting a result

Keep the object, surface, start pose, edge, controller settings, and battery
condition controlled across real repeats. Compare translation and rotation
separately before reducing them to one weighted loss:

- a consistent translation gap can indicate duration or speed mismatch;
- excess rotation often indicates contact placement, edge geometry, or
  heading error rather than duration; and
- large trial variance means the data do not support a precise duration
  choice.

The checked-in [object-1 comparison](../push_calibration/obj_1/diff/comparison.md)
is retained historical evidence of the report format and an earlier dataset.
It explicitly reports controller-setting drift, so it is not proof that the
current configuration is calibrated and must not be quoted as a current
outcome.
