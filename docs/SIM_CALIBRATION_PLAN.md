# Sim Calibration Plan

In-depth plan for the sim-vs-real push calibration workflow on the diff-drive
("car") robot. The output of this work is a calibration table that compares
real-robot push outcomes against the C++ NAMO push skill running in MuJoCo,
per `(object, edge, depth)` target. This table drives tuning of
`config/controller.yaml: push.max_speed` (wheel speed) and `push.push_steps`
(push duration in ticks per NAMO unit).

This document is the canonical reference for the implementation. The reader
is **future me, in another conversation**, who has the diff list but no prior
context for the calibration session.

Implementation status as of 2026-05-25:

- Real data collection is done with `scripts/execute_real_push.py`, not the
  older `collect_real_primitives.py` name.
- Sim replay is done with `scripts/execute_sim_push.py --from-real-run
  <real_run_dir> --push-index N`.
- `--from-real-run` is now **strict**: it requires `mid_obs.jsonl` from the
  real run and does **not** fall back to live camera state.
- The replay target is the recorded **push-start scene** for each push index.
  That means the pushed object pose and other recorded scene objects come from
  `mid_obs.jsonl`, while the compact before/after summary comes from
  `pushes.jsonl`.
- `execute_sim_push` still drives the C++ `NAMOPushController` path through
  `env.step(action)`, so the push itself is exercised through the same sim
  replay code path we trust elsewhere.
- Tier 1 (`push_tracker_max_speed`) was calibrated against PWM-0.4 straight-
  line pure-pursuit data via `scripts/find_sim_fraction.py` and the recommended
  value (≈0.054) is now baked into the canonical car YAML.
- `kCarWheelMaxSpeedMs` is **locked at 1.0** and is no longer a tuning knob.
  It's a multiplicative scale redundant with `push_tracker_max_speed`; the
  former Stage-2 sweep has been retired to avoid overparameterization.
- Real-vs-sim diff is `scripts/diff_real_vs_sim.py`, which reads matched
  `tier2_push_trials/<tag>/` bundles from both sides and computes the
  four-component pose-gap loss (object xy + θ, robot xy + heading θ).
- The deleted `MujocoSimEnv` Python path is gone — sim now dispatches through
  the C++ executor exclusively (matches what the planner uses internally).

---

## 1. Goal

For each recorded real push, run the **same primitive** in sim against the
**same recorded push-start scene**, then record sim's outcome and the gap.
The operational loop is:

```
1. Run one execute_real_push session with a trial spec.
2. That produces one real run directory containing pushes.jsonl, subgoals.jsonl,
   mid_obs.jsonl, wheel_commands.jsonl, and tier2_push_trials/.
3. For each desired push record N in that run:
     execute_sim_push --from-real-run <real_run_dir> --push-index N ...
4. Compare the real and sim outputs for that push index.
```

If you collect, for example, 3 pushes for each of 4 edges and 2 depths, that
is 24 real push records and therefore 24 separate sim replays.

---

## 2. Background — what already exists

### Existing real-side recording (`scripts/execute_real_push.py`)

Per real session, records to disk:

```
<diag-path>/<run-name>/
├── config.json              (args + git state)
├── pushes.jsonl             (per-push before/after summary)
├── subgoals.jsonl           (per-subgoal dispatch + outcome)
├── mid_obs.jsonl            (full recorded scene stream)
├── wheel_commands.jsonl     (per-tick push wheel commands)
├── tier2_push_trials/       (exported per-push chassis-calibration bundles)
└── run.log                  (tee'd stdout/stderr)
```

`pushes.jsonl` schema (one record per push):

```json
{
  "at_epoch": 1779385374.5397532,
  "at_utc_iso": "2026-05-21T17:42:54.539Z",
  "object_id": "obj_1",
  "expected_edge": 44,
  "expected_push_steps": 10,
  "object_pose_before": [19.69, 16.12, 268.76],
  "object_pose_after":  [20.24, 43.39, 265.61],
  "delta_pos_cm":           [0.55, 27.27],
  "delta_pos_magnitude_cm": 27.28,
  "delta_theta_deg":        -3.15,
  "stuck": false,
  "stuck_threshold_cm": 2.0,
  "stuck_threshold_deg": 15.0,
  "subgoal_id": 1
}
```

Current runs also include:

- `robot_pose_before_cm_deg` / `robot_pose_after_cm_deg`
- `push_start_obs_timestamp` / `push_end_obs_timestamp`
- `push_path_cm`, `push_controller_max_speed`, and related push metadata

`mid_obs.jsonl` is the important addition for replay. It records the full scene
state over time and is now required by `execute_sim_push --from-real-run`.

### Existing sim replay path (`scripts/execute_sim_push.py`)

`execute_sim_push.py` now wraps the replay path we actually use:

- it rebuilds a rewind XML from the recorded real run
- it calls `render_chain_to_mp4(...)`
- the subprocess calls `env.step(action)` through the C++ NAMO push skill
- the subprocess writes not just MP4/video output, but also
  `mid_obs.jsonl`, `pushes.jsonl`, `subgoals.jsonl`, `wheel_commands.jsonl`,
  `qpos_dump_full.txt`, and `tier2_push_trials/`

So the calibration path is no longer a hypothetical in-process tool; it is the
implemented `execute_sim_push --from-real-run --push-index N` flow.

### Existing namo_cpp/data scene templates

`namo_cpp/data/nominal_primitive_scene_{square,wide,tall}_1x_car.xml` —
pre-tuned scenes used for primitive generation. As of commit `e7df838`
("data: use MuJoCo default friction in 1x car scenes + regenerate
primitive DB"), each contains:

- **Floor**: `friction="0.5 0.005 0.001"`, `condim=4`, type=plane.
- **Movable obstacle** (`obstacle_1_movable`): **`friction="1 0.005 0.0001"`**
  (MuJoCo defaults — was `"0 0.1 0.1"` before `e7df838`; the frictionless
  version produced spinning-puck artifacts where corner pushes sent the
  obstacle 60 cm in the wrong direction), `mass="0.1"`, `condim=4`,
  height 0.05 m.
- **Walls** (`<body name="walls">`): four sides at ±0.8333 m, `condim=4`,
  no explicit friction (inherits MuJoCo defaults).
- **Top-level `<include>` of `little_car.xml`** at the **relative path**
  `../test_xml/little-car-modeling-package/assets/mjcf/little_car.xml`.
  Important gotcha: relative paths resolve from the XML's directory, so
  writing an edited copy to `/tmp/` will fail to find `little_car.xml`.
  Substitute an absolute path before writing (see §4.3).
- **`<option>` block**: `cone="elliptic"`, `integrator="implicitfast"`,
  `iterations=100`, `timestep=0.002`. Required for stable car contact —
  do not change.

`little_car.xml` itself defines:
- Chassis body mass 0.35 kg, wheels 0.025 kg each, casters 0.05 kg each,
  `condim=4`.
- Chassis and drive-wheel geoms have **no** explicit `friction` attribute
  — they inherit MuJoCo's defaults `(1, 0.005, 0.0001)`.
- Caster wheels have `friction="0 0 0"` (intentionally frictionless balls).
- Wheel velocity actuators: `ctrlrange="-25 25"` rad/s, `kv=0.75`,
  `forcerange="-0.5 0.5"` Nm.

These XMLs are the friction/physics baseline we want to preserve. The
**only** modifications per trial are: obstacle size (to match real object
dims), obstacle name (to match real's `object_id`), obstacle initial pose,
wall locations (moved very far away), and the `<include>` path (made
absolute). All in-memory; the source files are never written to.

### Motion primitive `.dat` files — not used by our pipeline

`namo_cpp/data/car_motion_primitives_15_{square,wide,tall}.dat` are the
regenerated primitive databases (last touched in `e7df838`). They are
read by the Python high-level planner's `PrimitiveGoalStrategy` for cost
and goal-position lookups. **They are not consumed during `env.step`.**
`NAMOPushController::execute_push_primitive` computes the push fresh at
runtime in MuJoCo. So sim_calibrate's results depend on the live physics
plus configured knobs, not on the cached `.dat` files.

### Existing C++ API surface (relevant bits)

From `namo_cpp/python/namo/cpp_bindings/`:

- `RLEnvironment(xml_path, config_path, visualize=False, skip_warmup=False)`
- `env.warm_up()` — public since the recent commit.
- `env.set_robot_pose(x_m, y_m, theta_rad)` — teleports the robot (used
  here only if needed; the push skill drives its own approach so we may
  not need it).
- `env.get_full_state() -> RLState` with `qpos`, `qvel` arrays.
- `env.set_full_state(state)` — restore a snapshot.
- `env.step(action) -> StepResult` where `action` has `object_id`,
  `edge_idx`, `depth`, and `(x, y, theta)` (the latter unused for direct
  primitive execution).
- `env.get_object_info()` returns a **cached** map populated at
  construction. **Not live.** Don't use this for post-step pose; use
  `get_full_state().qpos`.

### How `action.object_id` is consumed

Confirmed by reading `rl_env.cpp:238-242`:

```cpp
std::map<std::string, SkillParameterValue> params = {
    {"object_name", action.object_id},
    {"target_pose", SE2State(action.x, action.y, action.theta)},
    {"edge_idx", action.edge_idx},
    {"depth", action.depth}
};
```

The skill resolves the obstacle **by body name** in the MuJoCo model.
Therefore: name the obstacle body in our in-memory XML the same as
`real's object_id`, then `action.object_id` is identical to what real
recorded — no name translation needed.

---

## 3. Recommended working layout

One practical way to organize a calibration session is:

```
robot_control/calibration/
└── 2026-05-23_push_calibration/
    ├── real/
    │   └── <execute_real_push_run>/
    │       ├── pushes.jsonl
    │       ├── subgoals.jsonl
    │       ├── mid_obs.jsonl
    │       ├── wheel_commands.jsonl
    │       └── ...
    └── sim/
        ├── push_000/
        ├── push_001/
        ├── ...
        └── push_023/
```

The important mapping is:

- one `execute_real_push` run directory can contain many pushes
- `push_index` selects which recorded push to replay
- one `execute_sim_push` run should be created per `push_index`

For the example session:

- 4 edges
- 2 depths
- 3 repeats each

that is `4 * 2 * 3 = 24` push records, so you should expect 24 sim replays.

### Example commands

Collect one real session:

```bash
python scripts/execute_real_push.py \
  --config config/real.yaml \
  --camera-service tcp://localhost:5556 \
  --diag-path robot_control/calibration/2026-05-23_push_calibration/real \
  --run-name edge_depth_sweep \
  --trial-spec config/real_primitive_trials_example.yaml \
  --record-video
```

Replay one recorded push in sim:

```bash
python scripts/execute_sim_push.py \
  --from-real-run robot_control/calibration/2026-05-23_push_calibration/real/edge_depth_sweep \
  --push-index 7 \
  --diag-path robot_control/calibration/2026-05-23_push_calibration/sim \
  --run-name push_007
```

Loop over every recorded push:

```bash
for idx in $(seq 0 23); do
  python scripts/execute_sim_push.py \
    --from-real-run robot_control/calibration/2026-05-23_push_calibration/real/edge_depth_sweep \
    --push-index "$idx" \
    --diag-path robot_control/calibration/2026-05-23_push_calibration/sim \
    --run-name "push_$(printf '%03d' "$idx")"
done
```

## 4. Per-push replay pipeline (current implemented flow)

```
1. Run `execute_real_push.py` over a trial spec.

2. For replay of one recorded push, choose a `push_index` N.

3. `execute_sim_push.py --from-real-run <real_run_dir> --push-index N` loads:
     - `pushes.jsonl`
     - `subgoals.jsonl`
     - `mid_obs.jsonl`

4. The replay script selects the target push record and finds the nearest
   recorded scene frame for that push in `mid_obs.jsonl`.

5. The rewind XML is built from the recorded scene only:
     - pushed object pose from `pushes.jsonl.object_pose_before`
     - robot pose from `pushes.jsonl.robot_pose_before_cm_deg`
     - other scene objects from the selected `mid_obs.jsonl` frame
     - workspace bounds from `config/real.yaml`
   There is no live-camera fallback anymore.

6. `execute_sim_push.py` then dispatches the replay through the existing C++
   replay path:
     - `render_chain_to_mp4(...)`
     - subprocess `robot_control.diagnostics.sim_replay_subprocess`
     - `env.step(action)` using the C++ `NAMOPushController`

7. The sim run writes:
     - `pushes.jsonl`
     - `subgoals.jsonl`
     - `mid_obs.jsonl`
     - `wheel_commands.jsonl`
     - `qpos_dump_full.txt`
     - `tier2_push_trials/`
     - `sim_push.mp4`

8. Compare the real push record at index N against the sim output for that run.
```

Important caveat:

For `--from-real-run`, the script reconstructs the recorded scene, but the
current C++ replay path may pre-teleport the robot to the selected edge point
before `env.step(action)` to satisfy the reachability gate in the C++ skill.
So this pipeline is the right tool for **push calibration** against the same
recorded scene, not for reproducing the earlier real navigation trajectory.

Historical note:

Sections below may still mention `sim_calibrate.py` from the earlier design.
Operationally, replace that with looping
`execute_sim_push --from-real-run <real_run_dir> --push-index N` or a thin
wrapper around that command.

---

## 5. Output schema — `trial/sim/pushes.jsonl`

One record per file (matching real's one-record convention):

```json
{
  "at_epoch": <unix-time-of-sim-run>,
  "at_utc_iso": "2026-05-21T...",
  "object_id": "obj_1",
  "expected_edge": 44,
  "expected_push_steps": 10,
  "object_pose_before": [x_cm, y_cm, theta_deg],
  "object_pose_after":  [x_cm, y_cm, theta_deg],
  "delta_pos_cm":           [dx, dy],
  "delta_pos_magnitude_cm": 22.5,
  "delta_theta_deg":        -0.4,
  "stuck": false,
  "stuck_threshold_cm": 2.0,
  "stuck_threshold_deg": 15.0,
  "subgoal_id": 1,

  "_sim": {
    "namo_config":  "namo_config_complete_skill15_car_1x.yaml",
    "template_xml": "namo_cpp/data/nominal_primitive_scene_wide_1x_car.xml",
    "robot_model":  "car",
    "obstacle_dims_cm": [depth, width, height],
    "wall_extent_m":   2.0,
    "starting_state_check_cm": [drift_x, drift_y, drift_theta_deg],

    "knobs": {
      "push_tracker_max_speed":  0.3,          /* YAML, fraction; defaults
                                                  to 0.3 when absent from
                                                  skill15_car_1x.yaml */
      "control_steps_per_push":  375,          /* YAML, ticks; 375 × 0.002 s
                                                  = 0.75 s per NAMO unit */
      "k_car_wheel_max_speed_ms": 1.0,         /* C++ constexpr in
                                                  namo_push_controller.cpp:22;
                                                  read by sim_calibrate from
                                                  the source as ground-truth
                                                  record. Requires recompile
                                                  to change. */
      "settle_steps_pre_push":   100,          /* C++ default in
                                                  namo_push_controller.cpp; can
                                                  be overridden at runtime via
                                                  NAMOPushController::
                                                  set_settle_steps */
      "settle_steps_post_push":  100
    }
  }
}
```

`starting_state_check_cm` is the residual between (a) `object_pose_before`
from the real record and (b) the sim's post-warm-up obstacle pose. Should be
near zero. Large values indicate a bug in the XML edit step.

The `knobs` block records every parameter that affects sim's push outcome,
so the diff layer can correlate calibration gaps with the parameter setting
used for that trial. Critical for the tuning workflow in §12.

---

## 6. `trial/diff/diff.json` schema

Pure pose math. No sim or env work — reads real and sim pushes.jsonl, writes:

```json
{
  "object_id":      "obj_1",
  "edge_idx":       44,
  "push_steps":     10,

  "real": {
    "object_pose_before":     [...],
    "object_pose_after":      [...],
    "delta_pos_cm":           [dx, dy],
    "delta_pos_magnitude_cm": 22.68,
    "delta_theta_deg":        -0.17,
    "delta_per_unit_cm":      2.268
  },
  "sim": {
    "object_pose_before":     [...],
    "object_pose_after":      [...],
    "delta_pos_cm":           [dx, dy],
    "delta_pos_magnitude_cm": ...,
    "delta_theta_deg":        ...,
    "delta_per_unit_cm":      ...
  },
  "gap": {
    "delta_pos_cm":            [real_dx - sim_dx, real_dy - sim_dy],
    "delta_pos_magnitude_cm":  real_mag - sim_mag,
    "delta_pos_magnitude_pct": 100 * (real_mag - sim_mag) / sim_mag,
    "delta_theta_deg":         wrap_signed_deg(real_dtheta - sim_dtheta),
    "delta_per_unit_cm":       real_per_unit - sim_per_unit
  },
  "ratios": {
    "real_over_sim_pos_magnitude": real_mag / sim_mag,
    "real_over_sim_per_unit":      real_per_unit / sim_per_unit
  },
  "starting_state_drift_cm": [
    real_pose_before[0] - sim_pose_before[0],
    real_pose_before[1] - sim_pose_before[1],
    wrap_signed_deg(real_pose_before[2] - sim_pose_before[2])
  ]
}
```

### Why each field

- `delta_per_unit_cm` + `ratios.real_over_sim_per_unit` —
  direct knob for **push duration per unit**
  (`controller.yaml: push.push_steps`). If `real_over_sim_per_unit > 1`,
  real moves more per unit than sim → either lengthen sim's tick budget
  or shorten real's, depending on which side is the ground truth for
  tuning.
- `gap.delta_pos_magnitude_pct` —
  single % number suitable for **wheel speed** decisions
  (`push.max_speed`). Positive means real overshoots; negative means
  undershoots.
- `gap.delta_theta_deg` and per-axis `gap.delta_pos_cm` —
  diagnose **contact-geometry / friction** mismatch. If sim corner
  pushes rotate cleanly but real rotates more, the gap shows up here
  long before friction-tuning is the priority.
- `starting_state_drift_cm` —
  sanity. The XML editor should make this `(0, 0, 0)` within a fraction
  of a cm / degree. Anything else points at a bug in step 3.

---

## 7. Aggregate `calibration/_summary.csv`

One row per `(obj, edge, depth)`:

```
object,edge,depth,n_trials,
real_mag_mean,real_mag_std,
sim_mag_mean,sim_mag_std,
gap_mag_mean,gap_mag_std,gap_pct_mean,
real_per_unit_mean,sim_per_unit_mean,ratio_per_unit_mean,
real_dtheta_mean,sim_dtheta_mean,gap_dtheta_mean,gap_dtheta_std
```

Read this CSV to answer: "where should I change `push_steps` and
`max_speed`?"

For corner edges, `gap_dtheta_*` matters more than `gap_mag_*`.
For center edges, `gap_mag_pct_mean` is the headline.

---

## 8. Scripts

The pipeline is three top-level scripts in `scripts/`. The older
`sim_calibrate.py` / `compute_diff.py` design was retired in favor of
this leaner triad.

### `execute_real_push.py`

Collect one real push session (N pushes). Writes the diagnostic tree
(`pushes.jsonl`, `subgoals.jsonl`, `mid_obs.jsonl`,
`wheel_commands.jsonl`, `config.json`, `run.log`) plus a per-push
`tier2_push_trials/<tag>/` bundle that the sim side mirrors.

```
python scripts/execute_real_push.py \
    --config config/real.yaml \
    --camera-service tcp://localhost:5556 \
    --trial-spec <path/to/trial_spec.yaml> \
    --diag-path <where/to/write> \
    --run-name <name> \
    --no-reset-check --push-speed 0.4
```

### `execute_sim_push.py`

One push per invocation. With `--from-real-run`, loads the recorded
push-start scene from the real run's `mid_obs.jsonl` and replays that
exact push through the C++ `NAMOPushController` in MuJoCo. Produces
the same artifact shape as the real side, including its own
`tier2_push_trials/` bundle.

```
python scripts/execute_sim_push.py \
    --from-real-run <real_run_dir> \
    --push-index N \
    --diag-path <sim_parent_dir> \
    --run-name push_N
```

To match a real session of N pushes, invoke N times with
`--push-index 0..N-1` into a common `<sim_parent_dir>`.

### `diff_real_vs_sim.py`

Compares a real session against the matched sim runs. Joins trials
on `(object_id, expected_edge, expected_push_steps)` with timestamp
tie-break, computes the four-component pose gap (object xy + θ, robot
xy + heading θ), aggregates into a headline `L`, writes a report
under `<real_run>/diff_vs_<sim_basename>/` (CSV + JSON + markdown +
optional matplotlib plots).

```
python scripts/diff_real_vs_sim.py \
    --real <real_run_dir> \
    --sim <sim_parent_dir> \
    [--out-dir <dir>] [--theta-weight 0.5]
```

See §12 for the loss formula and how to use the report.

### `tune_forcerange.py` (not yet written)

The auto-tuner that wraps the §12 sweep. Patches `little_car.xml`,
runs `execute_sim_push` for every push at each candidate `forcerange`,
calls `diff_real_vs_sim`, reads `headline_loss_cm`, picks the winner.
Restores the XML on exit. See §12.7.

---

## 9. Implementation notes & gotchas

1. **Quaternion convention.** MuJoCo uses `[qw, qx, qy, qz]` (scalar
   first). For a pure Z-axis rotation by θ:
   ```
   qw = cos(θ/2)
   qx = 0
   qy = 0
   qz = sin(θ/2)
   ```
   To recover θ from qpos:
   ```
   θ = 2 * atan2(qz, qw)     # in radians, signed, in (-π, π]
   ```

2. **Degree wrap.** Real records theta in `[0, 360)` from the camera
   (per the test data observed: ~268°, ~180°, ~358°). Sim's `2·atan2(qz,
   qw)` returns `(-π, π]` → `(-180, 180]` degrees. The diff layer should
   convert to a consistent range (signed `[-180, 180]`) before computing
   gaps so wraparound near 360°/0° doesn't manifest as a 360° gap.

3. **Object pose conventions in XML.** The MuJoCo body's `pos`/`quat`
   attributes set the initial qpos of the freejoint. Geom `pos`/`euler`
   attributes are relative to the parent body. To teleport the obstacle:
   modify the **body's** attributes, leave the geom's at default.

4. **MuJoCo size units.** `<geom type="box" size="hx hy hz">` is
   **half-extents** in meters. Conversion: a real object with full
   dimensions `(depth_cm, width_cm, height_cm)` → MuJoCo size
   `(depth_cm/200, width_cm/200, height_cm/200)`.

5. **No `set_collision_checking(False)` needed.** Walls are at ±2 m,
   floor is at z=0 with the obstacle/robot sitting above. The push
   primitive has no walls or other obstacles to abort on (it does
   already ignore the floor naturally).

6. **`get_full_state()` returns a custom struct.** Field is `qpos`. We
   need to verify whether it's a numpy array, a Python list, or
   something opaque. Whatever it is, indexing with `[slot:slot+7]` should
   work. (TODO during implementation: confirm in REPL.)

7. **Caching the obstacle's qpos slot.** Doing it via "find the 7-tuple
   that matches our written values" works because the obstacle's pose is
   unique within the model. Robot's freejoint is at a different
   `(x, y)`. If the search fails (no slot matches), the XML edit didn't
   take — abort the trial with a clear error.

8. **NAMO config to use.** The car path needs
   `namo_config_complete_skill15_car_1x.yaml` (in `namo_cpp/config/`).
   Same config that `find_namo_config(robot_model="car")` returns in
   `scripts/run_namo.py:56-78`. Using it keeps sim_calibrate on the
   same code path as the production planning pipeline. There's a
   second car config (`namo_config_car.yaml`) for the primitive
   generator with different `control_steps_per_push` (50 vs 375) —
   **don't use it for calibration**.

9. **`push_velocity` is dead code for the car** (commit `4797df1`).
   `namo_config_complete_skill15_car_1x.yaml:85` still has
   `push_velocity: 0.2` for backward compatibility, but the diff-drive
   push path ignores it entirely. The car's chassis-velocity knob is
   `push_tracker_max_speed` (a fraction; defaults to 0.3 in
   `ConfigManager` at `config_manager.hpp:313` when absent from YAML).
   `complete_skill15_car_1x.yaml` does not set `push_tracker_max_speed`
   — sim runs with the 0.3 default unless we patch it.

10. **`kCarWheelMaxSpeedMs` is a compiled constant.** Defined as
    `constexpr double kCarWheelMaxSpeedMs = 1.0;` in
    `namo_push_controller.cpp:22`. Multiplies `push_tracker_max_speed`
    (the YAML fraction) to produce wheel-velocity command in m/s. The
    commit message (`4797df1`) calls it the "firmware-max-equivalent
    calibration constant." Recalibrating it requires a recompile;
    that's why the tuning workflow stages it after the YAML knobs
    (see §12).

11. **Working directory.** Even though `.dat` files aren't read by
    `env.step` (motion primitives are computed fresh by
    `NAMOPushController::execute_push_primitive`), the namo config
    may reference other resources relatively. Best practice:
    `os.chdir(namo_cpp_dir)` before constructing the env, then chdir
    back. `sim_replay` does this in its subprocess.

12. **Determinism.** A given starting state in sim produces a
    deterministic push outcome. Different real trials at the same target
    have slightly different starting poses (camera jitter); each is fed
    to sim_calibrate independently, so sim outputs are not identical
    across them — they capture starting-pose sensitivity. Good.

13. **What if `env.step` returns failure?** The `StepResult` has a
    `failure_reason` string when the skill aborts (e.g., bad edge
    index, object missing). Record the failure in `<trial>/sim/error.json`
    and skip the diff for that trial. The summary CSV reports n_trials
    as the number of *successful* sim runs.

14. **Walls at ±2 m vs anywhere else.** Picked 2 m as a round number
    ~4× the real workspace half-width (real walls at ±0.245 m world
    bounds since arena is 49 cm wide). The wavefront grid is fine at
    this scale based on existing test scenes (templates use ±0.83 m
    out of the box). Default is a CLI flag (`--wall-extent-m`).

15. **Absolute `<include>` path required for /tmp/ XMLs.** The scene
    template uses a relative include
    (`../test_xml/little-car-modeling-package/assets/mjcf/little_car.xml`)
    that resolves against the template's directory. When we write the
    edited XML to `/tmp/`, MuJoCo will look in `/test_xml/...` and fail.
    The XML editor in §4.3 step (e) replaces the relative path with the
    absolute path before write.

16. **Path A is the only push path now** (commit `266d8ed`). A depth-N
    push is **one continuous physics window** (`push_steps ×
    control_steps_per_push` ticks), not N separate 1-step pushes. This
    matches what real does and means our recorded Δ should be more
    consistent with real than the old Path B behavior would have
    suggested. No code change needed on our side.

17. **`set_se2` preserves z** (commit `6b4e449`). Earlier we worried
    that teleporting the robot or obstacle might reset z and cause a
    settle bounce. The DiffDriveAdapter now reads current z from qpos
    rather than init_pos_[2], so teleports preserve height. No special
    handling needed.

18. **Settle steps default 100 in C++.** Hard-coded fallback in
    `namo_push_controller.cpp:504` (`kSettleSteps = settle_steps_override_
    > 0 ? settle_steps_override_ : 100`). Override via
    `NAMOPushController::set_settle_steps(n)`. Not exposed via the
    `RLEnvironment` binding today. If we ever need shorter settles
    (e.g., for sim runtime), expose `set_settle_steps` through pybind.

19. **`get_full_state()` returns a custom struct.** Field is `qpos`. We
    need to verify in REPL whether it's a numpy array, a Python list,
    or something opaque. Whatever it is, indexing with `[slot:slot+7]`
    should work. (TODO during implementation: confirm.)

20. **Motion primitive `.dat` files are NOT used by `env.step`.**
    They are read by the Python high-level planner's
    `PrimitiveGoalStrategy` for cost and goal-position lookups. The
    push primitive itself is computed fresh by
    `NAMOPushController::execute_push_primitive` at runtime. So
    calibration depends on live physics + configured knobs, not on the
    cached `.dat` files. Regenerating those files isn't part of the
    calibration loop.

---

## 10. Suggested build order

1. **qpos-reader helper** — pure Python, no env.
   - Given an `xml_str`, identify which body has a free joint named
     `<object_id>` and predict its qpos slot from the order of joints
     in the model. (Or do this empirically by snapshotting after warm-up
     and searching for the 7-tuple matching what we wrote — simpler.)
   - Smoke-test by constructing an env from `wide_1x_car.xml`,
     calling get_full_state(), printing qpos to inspect layout.

2. **XML editor helper** — pure Python with ElementTree.
   - Function: `prepare_calibration_xml(template_path, object_id,
     dims_cm, pose_cm_deg, wall_extent_m) -> str`.
   - Output: a fully edited XML string ready to write to a tempfile.
   - Smoke-test by writing the result and visually opening it in
     `python -m mujoco.viewer` once.

3. **Single-trial sim runner** — minimal version, no looping.
   - Pick `calibration/obj_1/edge44_depth9/<some trial>/real/`.
   - Run steps 1–11 of §4 end to end.
   - Eyeball Δ_sim: should be positive Y (matches the real direction),
     magnitude in the ballpark of real (probably 20–30 cm).
   - **Verification gate**: confirm `starting_state_drift_cm` ≈ 0 and
     the obstacle moved in the expected direction. If yes, scale up.

4. **Loop over a target** — for example, all 10 trials of
   `obj_1/edge44_depth9/*/`. Confirm the layout matches expectations
   and there are no surprises (e.g., failed steps).

5. **`compute_diff.py`** — standalone, reads paired files, computes
   diffs + emits `_summary.csv`. Run on the same target to see the
   first calibration numbers come out.

6. **Full tree run** — 93 trials. Inspect `_summary.csv`. Decide
   whether the gap structure justifies tuning `push.push_steps` or
   `push.max_speed` first.

---

## 11. Open questions / future work

- **Edge cases in starting pose**: what if real's `object_pose_before`
  places the obstacle so close to a wall that even at ±2 m, the push
  saturates? Real had no such cases (verified during data collection),
  so currently a non-issue. If it ever becomes one, bail the trial.

- **Should sim be allowed to fail?** Real had no `stuck=true` trials.
  But sim could fail in different ways (e.g., physics divergence at
  high contact forces). Current plan: write `error.json`, skip diff,
  do not abort the whole sweep. Aggregate summary should report the
  per-target success rate alongside the magnitudes.

- **What about depth=0 (`push_steps=1`)?** Not in the current data set
  (smallest depth tested was 2 → `push_steps=3`). If/when added, no
  change to this plan.

- **Friction-tuning hooks**: `diff.json` already includes per-axis
  `gap.delta_pos_cm` and the Δθ gap, which are the right inputs for
  future friction tuning. No additional schema work needed.

- **Sphere comparison**: deferred. The car model is what real is, so
  sphere isn't on the table.

- **Per-trial vs per-target replay**: per-trial is the right call (each
  real trial has a unique starting state). Per-target would lose
  starting-state variance info.

- *(Resolved)* **Which car config?** — `namo_config_complete_skill15_car_1x.yaml`,
  same as robot_control's `--robot-model car` path.

- *(Resolved)* **What is sim's actual push duration per unit?** —
  `control_steps_per_push: 375` × `timestep: 0.002 s` = **0.75 s/unit**.
  Real is `push.push_steps: 26` / 30 Hz = **~0.87 s/unit**. So real
  currently pushes ~16% longer per unit than sim at nominally the same
  commanded velocity — built-in calibration miss before tuning.

- *(Resolved)* **What is sim's commanded chassis velocity?** —
  `push_tracker_max_speed: 0.3` (defaulted, not in YAML) × `kCarWheelMaxSpeedMs:
  1.0` = **0.3 m/s** chassis velocity. Real-side equivalent is
  `controller.yaml: push.max_speed: 0.2` (PWM scale), which the firmware
  maps to some real m/s value — measure on real hardware to verify
  whether sim's 0.3 m/s assumption matches.

---

## 12. Tuning algorithm — single-knob Tier 2 sweep

**Direction**: real is the fixed ground truth; sim is what we tune to
match. The pipeline is `execute_real_push` (once) →
`execute_sim_push --from-real-run` (per push) → `diff_real_vs_sim`
(per candidate parameter value).

### 12.1 The knobs, current status

| # | Knob | Type | Status | Current value | Effect |
|---|------|------|--------|---------------|--------|
| 1 | `skill.push_tracker_max_speed` | YAML | **Locked (Tier 1 done)** | ≈0.054 (was 0.3) | Scales the wheel-velocity command. Calibrated against PWM-0.4 straight-line pure-pursuit real data via `find_sim_fraction.py`. Don't sweep again unless you re-collect free-drive data. |
| 2 | `kCarWheelMaxSpeedMs` | C++ `constexpr` | **Locked at 1.0** | 1.0 m/s (`namo_push_controller.cpp:22`) | PWM-to-chassis-m/s mapping. Multiplicatively redundant with knob 1; pinning it eliminates one degree of freedom. Don't tune. |
| 3 | Wheel `forcerange` | XML (`little_car.xml:65-71`) | **Tier 2 — the tuning target** | ±0.5 Nm (symmetric) | Motor saturation torque. The single physically meaningful knob for *push-under-load* dynamics. Wheel saturates here when contact forces spike — exactly the regime Tier 1 doesn't see. |
| 4 | Wheel `kv` | XML | Secondary | 0.75 | Velocity-tracking gain. Affects transients (how fast the wheel reaches the setpoint), not steady-state push. Only sweep if `forcerange` doesn't move the loss. |
| 5 | Floor/obstacle friction | XML (per-scene) | Last resort | floor 0.5/0.005/0.001, obstacle 1/0.005/0.0001 | Contact dynamics. Two parameters (floor + obstacle), so not a clean single-knob sweep. Only reach for it if knobs 3 + 4 both fail. |

### 12.2 Tier 2 — 1-D sweep over `forcerange`

**Setup:**
1. Collect one real run with N pushes via `execute_real_push.py` on a
   set of representative `(object, edge, depth)` trials.
2. The Tier 2 sweep varies wheel `forcerange` symmetrically; the sweep
   variable is the half-range, e.g. `f ∈ {0.25, 0.35, 0.50, 0.65, 0.80}`
   Nm. Edit `little_car.xml:65-71` to set both wheels (left + right)
   to `forcerange="-f f"`.
3. For each candidate `f`:
    - Patch the XML.
    - For each push index `0..N-1`: run
      `execute_sim_push.py --from-real-run <real_run> --push-index N
      --diag-path <sim_parent>/f_<f> --run-name push_<N>`.
    - Run `diff_real_vs_sim.py --real <real_run> --sim <sim_parent>/f_<f>`.
    - Read `_aggregate.json["headline_loss_cm"]` from the diff output.
4. Pick the `f` minimizing the headline loss.

Each candidate evaluation = N sim pushes + one diff. With ~20 pushes
per real run and ~5 candidate values, one full sweep is ~100 sim
pushes, which the C++ executor runs in single-digit minutes.

### 12.3 Loss function (`diff_real_vs_sim.py`)

Per matched trial, four pose-gap components:

```
gap_object_xy_cm     = ‖Δobject_pos_real − Δobject_pos_sim‖        (Euclidean cm)
gap_object_theta_deg = |Δobject_theta_real − Δobject_theta_sim|    (deg, wrapped ±180)
gap_robot_xy_cm      = ‖Δrobot_pos_real − Δrobot_pos_sim‖
gap_robot_theta_deg  = |Δrobot_heading_real − Δrobot_heading_sim|
```

Aggregated:

```
L = mean(gap_object_xy_cm) + w · mean(gap_object_theta_deg)
  + mean(gap_robot_xy_cm)  + w · mean(gap_robot_theta_deg)
```

with `w = 0.5` cm/deg by default (heading-error weight; one degree
heading gap is worth 0.5 cm of position gap). Configurable via
`--theta-weight`. The four-component breakdown is reported alongside
the headline so a structural mismatch (e.g. only `gap_object_theta`
diverges) is immediately visible.

**Pairing:** trials are joined on `(object_id, expected_edge,
expected_push_steps)`, with `push_start_obs_timestamp` order as a
tiebreaker for repeated triples. Subgoal IDs are **not** used — the
unit of analysis for Tier 2 is the push, not the subgoal lifecycle.
Sim always writes `subgoal_id=1` by construction (one-push-per-
invocation), so a subgoal-id join would never work anyway.

### 12.4 Reading the report

`diff_real_vs_sim.py` writes:

```
<real_run>/diff_vs_<sim_basename>/
├── _per_trial.csv      one row per matched trial × gap field
├── _aggregate.json     per-field stats + L breakdown
├── _unmatched.json     trials present on only one side
├── comparison.md       human-readable headline + worst-5 trials
└── _plots/             scatter sim Δ vs real Δ for each pose component (matplotlib-gated)
```

The headline in `comparison.md` is the loss `L`, plus the
four-component breakdown. The "worst-5 trials" table flags individual
outliers — useful when one bad push is pulling the mean up
disproportionately.

**Decision rule from the breakdown:**

| Dominant component | Likely cause | Knob to reach for |
|---|---|---|
| `object_xy_cm` | Translation under push: contact friction or wheel saturation | `forcerange` (Tier 2), then obstacle friction |
| `object_theta_deg` | Spin under contact: contact-point geometry, slip | obstacle friction; structural — won't yield to `forcerange` alone |
| `robot_xy_cm` | Robot trajectory diverges during the push: wheel slippage or contact reaction | floor friction; wheel `kv` |
| `robot_theta_deg` | Robot heading drift: differential wheel response | wheel `kv`; structural |

### 12.5 When Tier 2 stops helping

If sweeping `forcerange` across [0.25, 0.80] Nm doesn't change `L` by
more than ~10% of its current value, the wheel actuator isn't
saturating during these pushes — the gap is somewhere else. Common
candidates, in increasing order of "willing to touch":
1. Friction (floor and/or obstacle) — directly affects how an object
   slides under contact.
2. Wheel `kv` — affects how fast the velocity setpoint is reached;
   matters for short pushes where the transient is a non-trivial
   fraction of the trial.
3. Contact geometry — the box approximation may not match the real
   object's effective shape. Last resort; requires geometric
   re-modeling.

Use the per-component breakdown above to pick which one to try.

### 12.6 Why a 1-D sweep, not Bayesian optimization or grid

We're searching one continuous parameter with cheap evaluations
(single-digit minutes). A linear sweep of 5–7 values gives the whole
loss curve, not just the extremum. The curve shape tells you whether
the loss is smooth (a clean minimum to bracket) or noisy (Tier 2
isn't the right knob and you should stop sweeping and read the
component breakdown). That diagnostic value is more important than
the speedup from Bayesian optimization.

If a finer grid is needed, refine with Nelder-Mead around the best
grid point (~10 extra evaluations). Don't bother with `scipy.minimize`
machinery before the grid sweep shows the curve has a clean shape.

### 12.7 Tooling — `tune_forcerange.py` (future)

The auto-tuner script that wraps the above sweep is not yet written.
The intended interface:

```
python scripts/tune_forcerange.py \
    --real <real_run_dir> \
    --xml little_car.xml \
    --grid 0.25,0.35,0.50,0.65,0.80 \
    [--out-dir tuning/]
```

For each `f` in the grid:
1. Patch `little_car.xml` to `forcerange="-f f"` on both wheel actuators.
2. Re-run `execute_sim_push --from-real-run` for every push in `<real_run_dir>`.
3. Run `diff_real_vs_sim` and read `headline_loss_cm`.
4. Restore the XML at exit.

Output: `tuning/_grid_loss.csv` (f, L, per-component breakdown),
`tuning/_grid_loss.png` (loss curve), and a final recommendation line
naming the best `f`. Implement when needed.

---

## 13. Glossary

- **NAMO unit**: one logical push step in the planner. Maps to
  `controller.yaml: push.push_steps` ticks (currently 26) × 0.033 s
  (30 Hz) = ~0.87 s per unit. Real uses `push_steps` to mean
  count-of-units; `controller.yaml`'s `push_steps` means ticks-per-unit.
  These are different numbers despite sharing a name (see
  `docs/PUSH_DURATION_CALIBRATION.md`).

- **depth**: planner concept = recursion depth in search. Maps 1:1 to
  `push_steps - 1` in the recorded trial schema (so depth-9 = 10 units).

- **edge_idx**: contact point on the object's perimeter. Range
  `[0, 4 × points_per_face)` = `[0, 60)` at the production setting of
  `points_per_face=15`. Even indices in `[0..29]` are top-face samples,
  odd are bottom-face samples (paired by `i XOR 1`). For `[30..59]`,
  even within = right face, odd within = left face. Sample index =
  `idx // 2` for `[0..29]`, `(idx-30) // 2` for `[30..59]`. Push
  direction is edge → object center.

- **shape rule** (5% tolerance): pick a primitive variant
  (square/wide/tall) based on `width / depth` ratio. Mirrors C++ side.

- **Δ_real / Δ_sim**: the (x, y, θ) change of the obstacle from before
  to after a single primitive execution. Always in cm + degrees in this
  workflow, even though sim's native units are meters + radians.

- **gap**: real minus sim, signed. Positive `gap.delta_pos_magnitude_cm`
  means real moved farther than sim for the same primitive.
