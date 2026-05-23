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

---

## 1. Goal

For each row of recorded real-robot data, run the **same primitive** in sim
with the **same starting object pose**, then record sim's outcome and the
gap. The result is:

```
For each (object_id, edge_idx, push_steps) trial in calibration/:
    Δ_real  — already captured in trial/real/pushes.jsonl
    Δ_sim   — to be computed by sim_calibrate.py → trial/sim/pushes.jsonl
    gap     — to be computed by compute_diff.py  → trial/diff/diff.json
```

Then aggregate per `(object, edge, depth)` into a top-level
`calibration/_summary.csv` for at-a-glance review.

---

## 2. Background — what already exists

### Existing real-side recording (`scripts/collect_real_primitives.py`)

Per real trial, records to disk:

```
calibration/<obj>/<edge_depth>/<trial>/real/
├── config.json         (args + git state)
├── connectivity.jsonl  (robot online/offline events)
├── plans.jsonl         (empty for direct trial-spec runs)
├── pushes.jsonl        (the calibration signal)
├── run.log             (tee'd stdout)
└── subgoals.jsonl      (per-subgoal dispatch + outcome)
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

All three fields `object_pose_before/after` are `[x_cm, y_cm, theta_deg]`.

### Existing sim_replay (`src/robot_control/diagnostics/sim_replay.py`)

Takes a starting XML + namo_config + chain of `(object_id, edge_idx,
push_steps)`, spawns a subprocess (clean `NAMO_QPOS_DUMP` static state),
runs `env.step` per chain entry, dumps per-tick qpos to a temp file, then
renders an MP4. **Outputs pixels, not pose data.** Useful for video
inspection, not for calibration math.

### Why sim_replay isn't reused as-is

- Renders MP4 (we don't need video).
- Output is per-tick qpos, not per-trial Δ.
- Subprocess is required there because of `NAMO_QPOS_DUMP` static-init.
  For calibration we don't dump qpos; we read `env.get_full_state()` directly,
  so in-process is fine and faster.

Calibration needs a different tool. It can borrow ideas (template XML, env
construction, action dispatch) but writes a new script.

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

## 3. File system layout

### Input (already in place)

```
robot_control/calibration/
├── obj_1/
│   ├── edge14_depth2/
│   │   ├── ticks26_trial1_<ts>/
│   │   │   ├── real/
│   │   │   │   ├── config.json
│   │   │   │   ├── connectivity.jsonl
│   │   │   │   ├── plans.jsonl
│   │   │   │   ├── pushes.jsonl   ← the input we read
│   │   │   │   ├── run.log
│   │   │   │   └── subgoals.jsonl
│   │   │   ├── sim/    (empty, to be filled by sim_calibrate.py)
│   │   │   └── diff/   (empty, to be filled by compute_diff.py)
│   │   ├── ticks26_trial2_<ts>/  (same shape)
│   │   └── ticks26_trial3_<ts>/
│   ├── edge14_depth6/   (similar)
│   ├── ...
│   └── edge44_depth9/   (10 trials each)
└── obj_4/
    ├── edge1_depth2/    (3 trials each)
    └── ...
```

Current counts as of the plan: **24 targets, 93 trial dirs**.
`config/objects.yaml` defines the obstacle dims (obj_1: 15×7×4, obj_4:
12×7.5×5; widths in cm).

### Output (to be filled)

```
trial/sim/
├── pushes.jsonl        ← same schema as real, plus sim-specific fields
└── config.json         ← sim invocation metadata (template path, namo_config,
                           dims used, robot start pose, etc.)

trial/diff/
└── diff.json           ← paired comparison + gap stats

calibration/
└── _summary.csv        ← per-(obj, edge, depth) aggregate (mean/std real,
                           mean/std sim, mean gap, %gap, n_trials)
```

---

## 4. Per-trial sim pipeline (in-process)

```
1. Read trial/real/pushes.jsonl
     → record: (object_id, edge_idx, push_steps, object_pose_before)
     → object_pose_before is [x_cm, y_cm, theta_deg]

2. Look up object dims from config/objects.yaml
     → (width_cm, depth_cm, height_cm)
     → shape rule:
        ratio = width / depth   (Y / X in object frame)
        if abs(ratio - 1.0) <= 0.05  → "square"
        elif ratio > 1.0             → "wide"  (Y > X)   ← obj_1, obj_4
        else                         → "tall"  (X > Y)
     → pick template:
        namo_cpp/data/nominal_primitive_scene_<shape>_1x_car.xml

3. Build in-memory XML (function: _prepare_calibration_xml):
     a. Read template file as string.
     b. Parse with xml.etree.ElementTree.
     c. Find the <body> whose name is "obstacle_1_movable":
        - Set body's pos="<x_m> <y_m> 0.05"
        - Set body's quat="<qw> 0 0 <qz>"  (Z-axis rotation only)
          qw = cos(theta_rad / 2)
          qz = sin(theta_rad / 2)
        - Rename body to <object_id> (e.g., "obj_1")
        - Find the body's child <geom>:
            * Update geom name to <object_id>
            * Update geom size to "<depth_cm/200> <width_cm/200> <height_cm/200>"
              (MuJoCo box size is half-extents in meters → cm/2/100 = cm/200)
     d. Find <body name="walls"> and modify it:
        - Update each wall geom's pos so all four walls are at ±2.0 m
          (4× the real arena half-width). This keeps the wavefront grid
          large enough to be meaningful but ensures no real contact.
     e. Find the top-level <include> element and rewrite its file=
        attribute from the relative path
        "../test_xml/little-car-modeling-package/assets/mjcf/little_car.xml"
        to the absolute path
        "<namo_cpp>/test_xml/little-car-modeling-package/assets/mjcf/little_car.xml".
        MuJoCo resolves relative includes from the XML file's directory; the
        edited XML will live in /tmp/ where the relative path doesn't exist.
     f. Serialize back to string.
     g. Write to a NamedTemporaryFile (suffix=".xml"). Keep the path; we'll
        unlink it after env teardown.

4. Construct env:
     namo_config_car_path = (
        "<namo_cpp>/config/namo_config_complete_skill15_car_1x.yaml"
     )
     # Note: this is the SAME config robot_control's --robot-model car
     # path loads (find_namo_config in run_namo.py:56-78). Keeps sim_calibrate
     # exercising the same code paths as the production planning pipeline.
     env = namo_rl.RLEnvironment(
        temp_xml_path,
        namo_config_car_path,
        visualize=False,
        skip_warmup=True,
     )
     env.warm_up()
     # Optionally: env.set_collision_checking(False) if you want the push
     # to be unconstrained by any residual wall contact. Walls are 2 m
     # away in our edited XML, so this is belt-and-suspenders.

5. Snapshot pre-state:
     state = env.get_full_state()
     qpos = state.qpos   # list/array of floats

     Find the obstacle's qpos slot (function: _find_object_qpos_slot):
     - Each free joint contributes 7 consecutive qpos elements:
       [x, y, z, qw, qx, qy, qz]
     - The car has its own freejoint + 2 wheel joints (1 qpos each)
     - We're looking for the [x, y] pair that matches our written values
       (the obstacle pose we set in the XML).
     - Strategy: iterate over qpos in steps of 1, find the consecutive
       7-element window where x and y are close to our written values
       and the z is close to 0.05 (the obstacle's z).
     - Slot index is cached for the lifetime of this env (one per trial).
     - Sanity-check: convert back to cm and assert the recovered pose
       matches the real `object_pose_before` to within 0.5 cm / 1°. If
       not, abort the trial.

6. Build action and step:
     action = namo_rl.Action()
     action.object_id = real's object_id          # matches XML body name
     action.edge_idx  = real's edge_idx
     action.depth     = real's push_steps - 1
     action.x = action.y = action.theta = 0.0     # unused for primitive exec
     env.step(action)

7. Snapshot post-state:
     state = env.get_full_state()
     Read same 7-element slot.
     object_pose_after_sim:
        x_cm = qpos[slot+0] * 100
        y_cm = qpos[slot+1] * 100
        theta_rad = 2 * atan2(qpos[slot+6], qpos[slot+3])   # 2·atan2(qz, qw)
        theta_deg = theta_rad * 180/pi  (wrap into a standard range)

8. Compute Δ (in cm + deg, matching the real schema's units):
     delta_pos_cm = [
       object_pose_after_sim[0] - object_pose_before_sim[0],
       object_pose_after_sim[1] - object_pose_before_sim[1],
     ]
     delta_pos_magnitude_cm = hypot(*delta_pos_cm)
     delta_theta_deg = wrap_signed_deg(after_theta - before_theta)
     # wrap_signed_deg keeps the value in [-180, 180]

9. Write trial/sim/pushes.jsonl with the full record (schema below).
   Write trial/sim/config.json with the invocation metadata.

10. unlink the temp XML.
    del env  (drop the RLEnvironment so the next trial gets a fresh state).
```

### Why a fresh env per trial

`NAMO_QPOS_DUMP` is not used (we're reading get_full_state directly), so
the static-state constraint that forces sim_replay into a subprocess does
not apply. But each trial needs a different starting object pose; the
cleanest way is to construct a new env per trial. The cost is acceptable
(~hundreds of milliseconds per env build).

Alternative explored: keep one env and call `set_full_state` between
trials. Rejected because the obstacle's qpos slot depends on the XML
(specifically, on which free joints are present and their order). Per-trial
XML construction simplifies the pipeline.

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

Two scripts under `src/robot_control/diagnostics/`:

### `sim_calibrate.py`

Heavy. Walks `<calibration_root>`, finds every
`<trial>/real/pushes.jsonl`, runs the per-trial sim flow (§4), writes
`<trial>/sim/`.

```
python -m robot_control.diagnostics.sim_calibrate <calibration_root>
    [--targets <glob>]               # e.g. "obj_1/edge44_*"
    [--namo-config <path>]           # default: the car config
    [--templates-dir <path>]         # default: namo_cpp/data/
    [--objects-yaml <path>]          # default: config/objects.yaml
    [--wall-extent-m 2.0]            # walls relocated to ±this distance
    [--force]                        # overwrite existing sim/
    [--dry-run]                      # list trials, do not execute
    [--max-parallel 1]               # currently sequential; can parallelize later
```

Idempotent: by default skips trials whose `<trial>/sim/pushes.jsonl`
already exists. Pass `--force` to overwrite.

### `compute_diff.py`

Light. Walks `<calibration_root>`, pairs each
`<trial>/real/pushes.jsonl` with `<trial>/sim/pushes.jsonl`, writes
`<trial>/diff/diff.json`, then emits `_summary.csv`.

```
python -m robot_control.diagnostics.compute_diff <calibration_root>
    [--targets <glob>]
    [--summary-out calibration/_summary.csv]
    [--force]                        # overwrite existing diff/
```

Re-runnable: pure pose math, no env.

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

## 12. Tuning algorithm — knob hierarchy + optimization strategy

**Direction**: real is the fixed ground truth; sim is what we tune to
match. Sim evaluations are cheap (~10 min per full sweep over 93 trials).

### 13.1 The knobs, ranked by tuning cost

| # | Knob | Type | Default | Cost to change | Effect |
|---|------|------|---------|----------------|--------|
| 1 | `skill.push_tracker_max_speed` | YAML (fraction) | 0.3 (in `ConfigManager`) | Free — YAML edit | Scales chassis velocity |
| 2 | `skill.control_steps_per_push` | YAML (int ticks) | 375 (in `skill15_car_1x.yaml`) | Free — YAML edit | Sets push duration per NAMO unit |
| 3 | `kCarWheelMaxSpeedMs` | C++ `constexpr` | 1.0 m/s (`namo_push_controller.cpp:22`) | **Recompile required** | Sets the PWM-to-chassis-m/s mapping. Real's firmware does this mapping; sim's compiled constant must match for sim-real velocity parity. |
| 4 | Wheel `kv`, `forcerange`, `ctrlrange` | XML (`little_car.xml:62-67`) | 0.75 / ±0.5 Nm / ±25 rad/s | Free — XML edit | Motor responsiveness and saturation |
| 5 | Floor/obstacle friction | XML (`nominal_primitive_scene_*_1x_car.xml`) | floor 0.5/0.005/0.001, obstacle 1/0.005/0.0001 | Free — XML edit, but changes primitive DB physics too | Contact dynamics |

**Stage 1** tunes knobs 1 and 2 only. **Stage 2** brings in knob 3 if
needed. **Stage 3** (only if stages 1–2 hit a wall) considers knobs 4
and 5.

### 13.2 Stage 1 — 2D grid sweep over the YAML knobs

```
push_tracker_max_speed       ∈ [0.15, 0.20, 0.25, 0.30, 0.40, 0.50]
control_steps_per_push       ∈ [200,  275,  375,  475,  575]
                                   (each maps to a duration per unit at 0.002 s/tick)
                                   200→0.40s, 275→0.55s, 375→0.75s, 475→0.95s, 575→1.15s
```

6 × 5 = 30 grid points. Each evaluation = full sweep of 93 trials
through sim_calibrate ≈ 10 min ⇒ **~5 hours unattended sim time**.

For each grid point, write a copy of `skill15_car_1x.yaml` to a temp
file with the patched values, run `sim_calibrate.py --namo-config
<temp_yaml>`, then run `compute_diff.py` to get the aggregate loss.

**Aggregate loss** (one number per grid point):
```
L = mean over trials of  |delta_pos_magnitude_pct|  +  w * |delta_theta_deg|
```
with `w ≈ 0.5 cm/degree` so the two components are commensurate. Use
`|gap_pct|` not raw `|gap_cm|` so deep pushes don't dominate.

### 13.3 Visualize the landscape

After the grid sweep, plot:
1. A **5×6 heatmap of L** over `(v, t)`. Find the minimum cell.
2. **Per-target heatmaps** (one per `(obj, edge, depth)`). If they all
   point to the same minimum, the calibration is consistent. If they
   disagree, the gap isn't a uniform calibration miss — it's a
   structural sim/real mismatch (probably friction or contact
   geometry), and no setting of `(v, t)` will close all gaps.
3. A **scatter of `L` vs `v × t`**. If the loss correlates almost
   perfectly with the product, the two knobs are redundant for our
   data (the multiplicative-dominance trap). Pick along the
   `v × t = const` curve by a secondary criterion (e.g., closest to
   current defaults, or shorter `t` for faster sim runtime).

### 13.4 Refine — Nelder-Mead near the best grid cell

```python
from scipy.optimize import minimize

def loss_at(v, t):
    write_temp_yaml(v, t)
    run_sim_calibrate_over_full_tree(temp_yaml)
    return aggregate_loss(read_all_diff_jsons())

result = minimize(
    lambda x: loss_at(x[0], int(round(x[1]))),
    x0=[best_v, best_t],
    method='Nelder-Mead',
    bounds=[(0.05, 0.6), (50, 700)],
    options={'xatol': 0.01, 'fatol': 0.01},
)
```

~15 evaluations after the grid. Bounds enforced via penalty (Nelder-
Mead doesn't natively respect bounds; wrap with `scipy.optimize.minimize`
+ `method='L-BFGS-B'` if you want hard bounds).

### 13.5 Stage 2 — recompile `kCarWheelMaxSpeedMs`

Only enter Stage 2 if Stage 1's minimum L is above tolerance (>~10%
aggregate, per the existing `PUSH_DURATION_CALIBRATION.md` standard)
**and** the YAML knobs were exercised near their extremes.

In Stage 2, `kCarWheelMaxSpeedMs` becomes a 1-D search. At each value:
- Edit `namo_push_controller.cpp:22` to the new value
- `./build_python_bindings.sh` (~30–60 s rebuild)
- Run sim_calibrate over the full tree at the Stage 1 optimal
  `(v, t)`
- Score the gap

Try 3–5 values (e.g., 0.7, 0.85, 1.0, 1.15, 1.3 m/s). Total ~1 hour
including rebuilds. The optimal value is the one where sim's commanded
chassis velocity matches the real robot's effective velocity at the
calibrated PWM setting.

### 13.6 Stage 3 — friction / wheel parameters (only if needed)

If Stage 2 still leaves a gap, the issue is contact dynamics, not
velocity calibration. Candidates:
- Obstacle friction `friction="1 0.005 0.0001"` — try defaults from
  more realistic materials, e.g. wood `(0.4, 0.005, 0.0001)`.
- Wheel `kv` — drop from 0.75 to 0.5 (slower motor response → larger
  acceleration phase → less per-unit displacement at short pushes).
- Wheel `forcerange` — narrow if real motor stalls earlier than sim.

These are XML edits, no recompile. But they affect more than the
calibration target (they're physically meaningful changes that may
break other downstream assumptions). Document any change carefully.

### 13.7 Why not Bayesian optimization?

Bayesian opt is overkill for cheap evaluations + 2D continuous
parameters. The GP surrogate's value is in expensive-eval regimes
(robot trials, deep learning training). Here we can afford a coarse
grid that gives us the **whole landscape**, not just an extremum. The
landscape view is what tells us whether the gap is calibration-shaped
or structural — and that's the more important decision than where
exactly the extremum sits.

### 13.8 Tooling — `sim_tune.py`

A third script in `src/robot_control/diagnostics/`:

```
python -m robot_control.diagnostics.sim_tune <calibration_root>
    --grid v=0.15,0.20,0.25,0.30,0.40,0.50
    --grid t=200,275,375,475,575
    [--out-dir tuning/]
    [--config-template <namo_cpp/config/namo_config_complete_skill15_car_1x.yaml>]
```

For each grid point: patches a temp YAML, invokes `sim_calibrate.py`
with `--namo-config <temp>`, reads the resulting summary, records the
aggregate loss. Output:

```
tuning/
├── v=0.20_t=375/_summary.csv       (per-target gap at this grid point)
├── v=0.20_t=475/_summary.csv
├── ...
├── _grid_loss.csv                  (aggregate loss per grid point)
├── _grid_loss.png                  (heatmap)
└── _per_target_heatmaps.png        (small multiples)
```

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
