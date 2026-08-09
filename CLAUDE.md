# CLAUDE.md - robot_control contributor guide

## Start here

Read the repository [README](README.md) for setup, entrypoints, and the
documentation index. Read the [architecture guide](src/robot_control/ARCHITECTURE.md)
before changing component boundaries or runtime flow.

These documents describe only the current checkout. Do not infer support from
workspace files, sibling-repository history, or generated experiment output.

## Current blocker

`NAMOPlanBridge` and its scene/object/subgoal conversion code exist, but the
current sibling `namo_cpp` checkout does not provide the in-process Python class
`namo.services.NAMOPlanningService` that the bridge imports lazily.

This is a missing Python API, not an external or network service outage. Do not
treat `scripts/run_namo.py` NAMO-backed modes, or commands that delegate to
them, as operational until the API is restored or the bridge is migrated to a
supported replacement.

## Repository commands

The normal workspace layout places this repository and `namo-cpp.yml` next to
each other. From the `robot_control` root, create the pinned environment and
install the package with:

```bash
git submodule update --init --recursive
conda env create -f ../namo-cpp.yml
conda activate namo-cpp
python -m pip install -e ".[dev,gui,zmq]"
```

These CLI import/help checks are available in the current checkout:

```bash
python scripts/test_control.py --help
python scripts/camera_service.py --help
python scripts/capture_to_xml.py --help
python scripts/execute_real_push.py --help
python scripts/execute_sim_push.py --help
python scripts/closed_loop_session.py --help
```

Run the repository-owned test suite explicitly; bare `pytest` also collects
vendored RVG tests:

```bash
python -m pytest tests
```

Help output verifies argument parsing and imports, not camera, MuJoCo, serial,
or real-robot operation end to end.

## Package map

| Location | Responsibility |
| --- | --- |
| `src/robot_control/core/` | Shared types, topics, world state, serialization, and object definitions. |
| `src/robot_control/environment/` | Simulation and real environments plus the real action-sender boundary. |
| `src/robot_control/nodes/` | Simulation, local-camera, and remote observation nodes. |
| `src/robot_control/planner/` | Planner interfaces, path planners, and the currently blocked NAMO adapter. |
| `src/robot_control/controller/` | Keyboard, navigation, path-following, push, and contact-geometry control. |
| `src/robot_control/camera/` | ArUco observation and workspace coordinates. |
| `src/robot_control/diagnostics/` | Structured recording, scene capture, rendering, and replay. |
| `src/robot_control/gui/` | PySide6 presentation. |
| `src/robot_control/runtime.py` | Component setup and the control loop. |
| `src/robot_control/coordinator.py` | Interactive controller routing. |
| `src/robot_control/executor.py` | Autonomous subgoal dispatch. |

## Engineering notes

- The robot-control observation and controller boundary uses centimeters for
  positions and degrees for headings. Workspace coordinates start at the
  bottom-left, with +X right and +Y up. MuJoCo-facing code converts positions
  to meters and headings to radians at the boundary. Wheel commands are
  normalized to `[-1, 1]`.
- Edge generation yields `4 * points_per_face` indices. Consecutive even/odd
  indices are opposite-face mates at the same sample, and the current
  `config/controller.yaml` value is `points_per_face: 15`. Keep the Python
  layout aligned with the planner-side convention; use
  [Push geometry](docs/PUSH_GEOMETRY.md) for the detailed construction.
- The NAMO conversion sets `PushSubgoal.push_steps` to planner `depth + 1`.
  `PushController` executes `PushSubgoal.push_steps * push.push_steps` control
  ticks, so the similarly named field in `config/controller.yaml` is the
  per-unit tick count, not the whole planned duration. See
  [Push-duration calibration](docs/PUSH_DURATION_CALIBRATION.md) before changing
  this relationship.
- Generated `closed_loop_sessions*` output is ignored; keep the tracked
  `closed_loop_sessions/README.md` as documentation rather than treating
  session artifacts as source.
- Preserve unrelated dirty submodule changes. Inspect submodule status before
  updating, resetting, staging, or committing a gitlink.

## Focused documentation

Use the root README's [documentation index](README.md#documentation-index) for
operator guides, controller details, calibration records, known issues, and
current gaps. Link to those focused documents instead of duplicating them here.
