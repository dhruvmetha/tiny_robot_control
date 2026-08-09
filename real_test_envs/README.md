# real_test_envs

Captured NAMO scenes for planning experiments and sim replay. Each leaf
directory is a self-contained scene with the common core files below.

> **Current checkout status:** Scene inspection and recapture instructions
> below describe available tooling. `scripts/run_namo.py` remains this
> checkout's intended planning entrypoint, but its NAMO-backed planning path
> currently stops when it imports the absent in-process Python class
> `namo.services.NAMOPlanningService` from the sibling `namo_cpp` checkout.
> This is a missing API, not a camera or network service outage; the planning
> command below is not operational until that dependency is restored or
> replaced. Recapture separately requires the configured camera service.

## Common scene layout

```
<env>/
├── env.xml          ← THE scene file (1×-scale car scene)
├── env.png          ← wavefront grid visualization
├── scene.jpg        ← annotated camera frame at capture time
└── mid_obs.jsonl    ← robot start pose source (first parseable record)
```

Some scenes contain additional tracked diagnostic artifacts; those are not
part of the common planning-input contract. Planner runs can also create
ignored output under `<env>/solution/random_rollout/runN/`.

Current scene inventory:

```
real_test_envs/
├── 1push/1hop/{env1, env2, env3, multi_contact, multi_contact3}/
├── 1push/2hop/{env1, env2, multi_contact}/
├── 2push/1hop/{env1, env2, env3, env4, multi_contact}/
└── 2push/2hop/{env1, multi_contact}/
```

All listed leaves have the four common files and use the planning command
below. Scene-specific diagnostic and generated-output subtrees vary.

## The scene file: `env.xml`

Every env has one canonical scene file: **`env.xml`**.

- 1× scale (MuJoCo meters; real-world workspace 49 × 77.5 cm).
- Car robot via `<include file=".../little_car.xml"/>`.
- Same MuJoCo options across all envs (`cone=elliptic`,
  `integrator=implicitfast`, `timestep=0.002`, `iterations=100`).
- 1 or 2 movable obstacles depending on the scene; format is identical
  either way.

## Where the robot pose lives

`env.xml` does **not** encode the robot's starting pose. The car body
is pulled in via:

```xml
<include file="/.../little-car-modeling-package/assets/mjcf/little_car.xml"/>
```

`little_car.xml` hard-codes the freejoint spawn at `(0, 0, 0.01)` and
MuJoCo `<include>` can't override a child body's pose. So the start
pose lives in **`mid_obs.jsonl`** — the planner reads the first
parseable record's `robot_pose_cm` and teleports the car there via
`RLEnvironment.set_robot_pose()` before search begins.

Schema (one JSON object per line, only the first parseable record with
non-empty `objects` is consumed — see
`scripts/run_namo.py:_robot_pose_from_real_run`):

```json
{
  "robot_pose_cm": [x_cm, y_cm, theta_deg],
  "objects": { "obj_1": {...}, "wall_9": {...} },
  ...
}
```

Every current env keeps `mid_obs.jsonl` at its root.

## Plan against an env (canonical command)

Run this from the repository root:

```bash
python scripts/run_namo.py \
    --sim \
    --sim-xml          real_test_envs/1push/1hop/env1/env.xml \
    --sim-real-run-dir real_test_envs/1push/1hop/env1 \
    --strategy         random_rollout \
    --rollout-samples-per-state 36000 \
    --diag-path        real_test_envs/1push/1hop/env1/solution/random_rollout \
    --run-name         runN \
    --allow-overwrite
```

`--sim-xml` dispatches to `run_namo.py`'s `_run_plan_only_mode`. The planner
verifies every push primitive through the C++ MuJoCo controller, so a returned
plan is a sim-success result. A successful run writes generated output such as
`solution.yaml`, `sim_push.mp4`, `config.json`, and diagnostic JSONL files
under the selected `--diag-path/--run-name`. Random-rollout outputs below an
env's `solution/` directory are ignored rather than tracked scene inputs.

### What changes per env

| Argument                | Source                                                |
| ----------------------- | ----------------------------------------------------- |
| `--sim-xml`             | `<env>/env.xml`                                       |
| `--sim-real-run-dir`    | `<env>` (env root; `mid_obs.jsonl` lives there)       |
| `--diag-path/--run-name` | wherever you want plan outputs                       |
| `--goal`                | optional override; otherwise uses `goal_cm` from the scene record when present, then the XML goal site |

## Re-capturing a scene

If the bricks/robot get bumped, refresh `env.xml`, `env.png`, and
`scene.jpg`. Requires `camera_service` already running on
`tcp://localhost:5556` + `5557`.

```bash
# env.xml + env.png
python scripts/capture_to_xml.py \
    --camera-service tcp://localhost:5556 \
    --scale-factor 1.0 --robot-model car --goal 37 67 \
    --output real_test_envs/1push/1hop/env1/env.xml

# scene.jpg
python -c "import zmq; s=zmq.Context().socket(zmq.REQ); \
    s.connect('tcp://localhost:5557'); s.send(b'vis'); \
    open('real_test_envs/1push/1hop/env1/scene.jpg','wb').write(s.recv())"
```

`capture_to_xml.py` does **not** emit `mid_obs.jsonl`. Update the first
parseable JSON record in the existing file from the script's captured-
configuration output: set `robot_pose_cm` to the printed robot
`[x_cm, y_cm, theta_deg]` and update the `objects` entries to the printed
object poses and dimensions. Keep `objects` non-empty or the planner will skip
the record. This keeps `env.xml` and `mid_obs.jsonl` tied to the same capture.

## Coordinate sanity

- Bottom-left origin, +X right, +Y up.
- θ in radians inside MuJoCo XML; degrees in `mid_obs.jsonl` and
  printouts.
- Every checked-in `env.xml` currently has a goal site at `(37, 67) cm`.
  Override it for planning with `--goal X Y` if needed.
- "Trapped-start recovery" messages from the wavefront planner mean the
  robot capture pose landed inside an inflated-obstacle margin near a
  wall; local cells are cleared so BFS still works. Expected, not a bug.
