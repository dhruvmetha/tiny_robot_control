# robot_control

`robot_control` is the single-robot runtime and execution layer in the NAMO
workspace. It provides simulation and real environments, camera observation,
planners, navigation and push controllers, diagnostics, scene capture, and
simulation/real calibration tools.

## Current status

| Area | Status |
| --- | --- |
| Runtime, simulation, controllers, camera, diagnostics, and calibration | Implemented |
| Captured-scene and closed-loop workspace helpers | Implemented; planning subcommands depend on the blocker below |
| NAMO-backed planning through `NAMOPlanBridge` | Blocked: the sibling `namo_cpp` checkout does not provide the in-process Python API `namo.services.NAMOPlanningService` |

## Architecture at a glance

```text
Observation source -> WorldState -> Planner -> SubgoalExecutor -> Controller -> Action -> Environment
```

Simulation nodes or camera/real nodes supply observations, and diagnostics
records the lifecycle. The `NAMOPlanBridge` is present, but its required
in-process `NAMOPlanningService` API is unavailable in this checkout.

## Setup

Use the pinned workspace environment, which assumes the usual NAMO layout in
which this repository and `namo-cpp.yml` are siblings. From this repository
root:

```bash
git submodule update --init --recursive
conda env create -f ../namo-cpp.yml
conda activate namo-cpp
python -m pip install -e ".[dev,gui,zmq]"
```

`../namo-cpp.yml` supplies the pinned workspace robotics dependencies; the
[project metadata](pyproject.toml) declares only this package and its extras.
For a test-only editable install, use `python -m pip install -e ".[dev]"`.
Users of another environment need equivalent robotics dependencies for camera,
MuJoCo, and real-robot workflows.

Review [real settings](config/real.yaml), [object definitions](config/objects.yaml),
and [controller settings](config/controller.yaml). Hardware workflows also
require locally configured camera and robot transport.

## Available entrypoints

```bash
python scripts/test_control.py --controller keyboard
python scripts/camera_service.py --help
python scripts/capture_to_xml.py --help
python scripts/execute_real_push.py --help
python scripts/execute_sim_push.py --help
python scripts/closed_loop_session.py status --help
python -m pytest tests
```

The `--help` commands check CLI and import availability; they do not validate
camera, hardware, or MuJoCo operation end to end. Use the repository test
command shown above for the package test suite.

### Currently blocked

As summarized in [Current status](#current-status), `NAMOPlanBridge` requires
the in-process Python class `namo.services.NAMOPlanningService`, which is
missing from the sibling `namo_cpp` checkout. This non-success probe currently
raises `ImportError`:

```bash
python -c "from namo.services import NAMOPlanningService"
```

No compatible `namo_cpp` revision is selected or supported by this checkout;
recovery requires restoring or replacing that API. Do not treat `run_namo` or
closed-loop planning subcommands as operational.

## Repository map

| Location | Contents |
| --- | --- |
| [src/robot_control](src/robot_control) | Runtime, environments, core state, planners, controllers, camera, and diagnostics. |
| [scripts](scripts) | Operator and development entrypoints. |
| [config](config) | Real-world, object, and controller configuration. |
| [tests](tests) | Package test suite. |
| [real_test_envs](real_test_envs) | Captured benchmark scenes and replay inputs. |
| [closed_loop_sessions](closed_loop_sessions) | Closed-loop workspace helpers and session layout. |
| [docs](docs) | Controller, geometry, and calibration notes. |
| [push_calibration](push_calibration) | Calibration results and comparisons. |

`closed_loop_sessions*` artifacts are generated and ignored.

## Documentation index

### Architecture and maintenance

- [Architecture](src/robot_control/ARCHITECTURE.md) — detailed runtime and component design.
- [Known issues](src/robot_control/KNOWN_ISSUES.md) — reproducible problems and their status.
- [Current gaps](src/robot_control/TODO.md) — outstanding capability gaps and follow-up work.
- [Contributor guidance](CLAUDE.md) — repository conventions and development context.

### Operations and scenes

- [Closed-loop sessions](closed_loop_sessions/README.md) — session workspace layout and operator workflow.
- [Real test environments](real_test_envs/README.md) — captured-scene format and replay inputs.

### Controllers, planning, and geometry

- [Navigation and wheel commands](docs/NAVIGATION_AND_WHEEL_COMMANDS.md) — navigation controller and wheel-command behavior.
- [Push geometry](docs/PUSH_GEOMETRY.md) — geometric conventions for push execution.
- [Wavefront follow-ups](docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md) — remaining wavefront-planning integration work.

### Calibration

- [Push-duration calibration](docs/PUSH_DURATION_CALIBRATION.md) — calibration procedure and interpretation.
- [Simulation calibration plan](docs/SIM_CALIBRATION_PLAN.md) — simulation calibration scope and method.
- [Object 1 comparison](push_calibration/obj_1/diff/comparison.md) — recorded comparison for object 1.

## Current limitations

- NAMO-backed planning is unavailable; see [Current status](#current-status).
- Ownership and durability of the two-line RVG C++17/GCC9 compatibility patch remain unresolved.
- GUI auto-quit remains unreliable after runtime completion.

See the [architecture](src/robot_control/ARCHITECTURE.md), [known issues](src/robot_control/KNOWN_ISSUES.md),
and [current gaps](src/robot_control/TODO.md) for detail.
