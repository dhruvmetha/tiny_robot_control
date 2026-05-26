# real_test_envs

Captured NAMO scenes for planning experiments and sim replay. Each
sub-directory is a self-contained scene with a uniform layout.

## Layout — every env directory looks the same

```
<env>/
├── env.xml          ← THE scene file (1×-scale car scene)
├── env.png          ← wavefront grid visualization
├── scene.jpg        ← annotated camera frame at capture time
├── mid_obs.jsonl    ← robot start pose source (first parseable record)
└── solution/        ← present iff a real-robot run has happened here
    ├── sim_push_execution/run/
    │   └── initial_scene.xml   ← byte-identical historical copy of env.xml
    ├── real_push_execution/
    │   └── mid_obs.jsonl       ← original (now mirrored at env root)
    └── random_rollout/run{N}/  ← plan + sim_push.mp4 + diagnostics
```

Full tree:

```
real_test_envs/
├── 1push/1hop/{env1, env2, env3, multi_contact}/
├── 1push/2hop/{env1, env2, multi_contact}/
├── 2push/1hop/{env1, env2, env3, multi_contact}/
└── 2push/2hop/{env1, multi_contact}/
```

All sub-directories — `envN` and `multi_contact` alike — use the same
file format and the same planning command. The only thing that varies
is whether the `solution/` subtree has been populated yet (i.e. whether
a real-robot run has executed there).

## The scene file: `env.xml`

Every env has one canonical scene file: **`env.xml`**.

- 1× scale (MuJoCo meters; real-world workspace 49 × 77.5 cm).
- Car robot via `<include file=".../little_car.xml"/>`.
- Same MuJoCo options across all envs (`cone=elliptic`,
  `integrator=implicitfast`, `timestep=0.002`, `iterations=100`).
- 1 or 2 movable obstacles depending on the scene; format is identical
  either way.

`solution/sim_push_execution/run/initial_scene.xml`, where present, is
a byte-identical historical copy from when the scene was first
captured. Prefer `env.xml`.

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
`set_robot_se2()` before search begins.

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

Every env keeps a copy of `mid_obs.jsonl` at its root. Envs that have
completed a real-robot run also have it at
`solution/real_push_execution/mid_obs.jsonl` (written by
`execute_real_push`); the env-root copy mirrors that.

## Plan against an env (canonical command)

```bash
cd /home/dhruv/projects_dhruv/namo/robot_control

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

`--sim-xml` automatically triggers plan-only mode
(`run_namo.py:1852`) — the planner verifies every push primitive
through the C++ MuJoCo controller, so a returned plan **is** a
sim-success result. Outputs land in the diag dir: `solution.yaml`,
`sim_push.mp4`, `config.json`, plus C++ diagnostics jsonls.

Reference invocation per env lives at
`<env>/solution/random_rollout/run1/config.json` → `command_line`
(some historical entries reference older paths like
`solution/sim_push_execution/run/initial_scene.xml` for `--sim-xml`
and `solution/real_push_execution` for `--sim-real-run-dir`; both
still work because the new env-root files mirror the old locations,
but new runs should use the env-root paths shown above).

### What changes per env

| Argument                | Source                                                |
| ----------------------- | ----------------------------------------------------- |
| `--sim-xml`             | `<env>/env.xml`                                       |
| `--sim-real-run-dir`    | `<env>` (env root; `mid_obs.jsonl` lives there)       |
| `--diag-path/--run-name` | wherever you want plan outputs                       |
| `--goal`                | optional override; defaults to the `<site name='goal'>` in the XML (production goal is `(37, 67) cm`) |

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

`capture_to_xml.py` does **not** emit `mid_obs.jsonl`. Easiest path:
edit the existing `mid_obs.jsonl` and replace `robot_pose_cm` with the
pose `capture_to_xml.py` prints ("Robot: (X, Y) cm @ Θ°"); keep
`objects` non-empty or the planner will skip the record. For an atomic
capture pattern (env.xml + env.png + mid_obs.jsonl from one frame),
re-use `_capture_from_camera_service()` from `scripts/capture_to_xml.py`
and write the jsonl record alongside the generator call — see this
README's git history for the recipe.

## Coordinate sanity

- Bottom-left origin, +X right, +Y up.
- θ in radians inside MuJoCo XML; degrees in `mid_obs.jsonl` and
  printouts.
- Production goal `(37, 67) cm` is baked into every `env.xml`
  (see `feedback/eps initial_scene.xml goal`). Override with `--goal X Y`
  if needed.
- "Trapped-start recovery" messages from the wavefront planner mean the
  robot capture pose landed inside an inflated-obstacle margin near a
  wall; local cells are cleared so BFS still works. Expected, not a bug.
