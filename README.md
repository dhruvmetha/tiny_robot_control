# robot_control

`robot_control` is the single-robot runtime and execution layer in the NAMO
workspace. It provides simulation and real environments, camera observation,
planners, navigation and push controllers, diagnostics, scene capture, and
simulation/real calibration tools.

## Current status

| Area | Status |
| --- | --- |
| Runtime, simulation, controllers, camera, diagnostics, and calibration | Implemented |
| Captured-scene and closed-loop workspace helpers | Implemented; `replan-reuse-only` can verify a prior chain in simulation when the current scene, prior plan, and canonical compiled `namo_rl` binding are available |
| NAMO-backed full search through `NAMOPlanBridge` | Working on hardware since 2026-08-23 (full_namo, best-first, model prior); requires sourcing `namo_cpp/env.<machine>.sh` first, see [Real-robot trials](docs/REAL_ROBOT_TRIALS.md) |

## Architecture at a glance

```text
Observation source -> WorldState -> Planner -> SubgoalExecutor -> Controller -> Action -> Environment
```

Simulation nodes or camera/real nodes supply observations, and diagnostics
records the lifecycle. `NAMOPlanBridge` can verify an existing chain through
the canonical `namo_rl` binding, but full search requires the unavailable
in-process `NAMOPlanningService` API.

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

As summarized in [Current status](#current-status), `NAMOPlanBridge`'s
NAMO-backed full-search path requires the in-process Python class
`namo.services.NAMOPlanningService`, which is missing from the sibling
`namo_cpp` checkout. This non-success probe currently raises `ImportError`:

```bash
python -c "from namo.services import NAMOPlanningService"
```

No compatible `namo_cpp` revision is selected or supported by this checkout;
recovery requires restoring or replacing that API. Consequently, `run_namo`
full search and `replan-full-search-only` are blocked. The `replan` command
tries prior-chain reuse first, but reaches the same blocker if reuse fails and
it falls back to full search.

`replan-reuse-only` does not use `NAMOPlanningService`: it verifies the prior
iteration's plan against the current scene through `NAMOPlanBridge.verify_chain()`.
It requires an existing scene and prior selected plan/chain plus the canonical
compiled `namo_rl` binding. This path is source-verified here, not validated as
a complete closed-loop or hardware workflow.

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

Status tags, borrowed from namo_cpp's convention: `[ROUTER]` entry/schema ·
`[HUB]` links many others · `[LIVE]` actively updated · `[REF]` stable
reference · `[SNAPSHOT]` dated results, not maintained.

### Architecture and maintenance

- [Architecture](src/robot_control/ARCHITECTURE.md) — detailed runtime and component design. `[REF]`
- [Known issues and status](src/robot_control/KNOWN_ISSUES.md) — current issue status, repair history, and revalidation evidence. `[LIVE]`
- [Current gaps](src/robot_control/TODO.md) — outstanding capability gaps and follow-up work. `[LIVE]`
- [Contributor guidance](CLAUDE.md) — repository conventions and development context. `[ROUTER]`

### Operations and scenes

- [ICRA real-robot study](docs/ICRA_REAL_ROBOT_STUDY.md) — the study brief: claims, pre-registered success semantics and analysis, the 14-scene matrix, the physics freeze, and current status. What counts and why. `[HUB]` `[LIVE]`
- [Real-robot trials](docs/REAL_ROBOT_TRIALS.md) — the table procedure: preflight, scene build with the checksum checker, trial commands, verdict recording, calibration blocks, and known hardware failure modes. Start here for any physical session. `[LIVE]`
- [Closed-loop sessions](closed_loop_sessions/README.md) — session workspace layout and operator workflow. `[REF]`
- [Real test environments](real_test_envs/README.md) — captured-scene format and replay inputs. `[REF]`

### Controllers, planning, and geometry

- [Navigation and wheel commands](docs/NAVIGATION_AND_WHEEL_COMMANDS.md) — navigation controller and wheel-command behavior. `[REF]`
- [Push geometry](docs/PUSH_GEOMETRY.md) — geometric conventions for push execution. `[REF]`
- [Wavefront follow-ups](docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md) — remaining wavefront-planning integration work. `[REF]`

### Calibration

- [Push-duration calibration](docs/PUSH_DURATION_CALIBRATION.md) — operator guide for tuning push-primitive duration. `[REF]`
- [Simulation calibration guide](docs/SIM_CALIBRATION_PLAN.md) — the implemented real-to-MuJoCo calibration workflow (filename kept for links; it is a guide, not a plan). `[REF]`
- [Object 1 comparison](push_calibration/obj_1/diff/comparison.md) — recorded comparison for object 1. `[SNAPSHOT]`

## Current limitations

- NAMO-backed fresh/full search is unavailable; prior-chain verification and
  reuse remain available under the conditions in [Current status](#current-status).
- Ownership and durability of the two-line RVG C++17/GCC9 compatibility patch remain unresolved.
- GUI auto-quit and run-end finalization remain intermittently unreliable after
  autonomous completion; see the [current issue status](src/robot_control/KNOWN_ISSUES.md#runtime-auto-quit-revalidation).

See the [architecture](src/robot_control/ARCHITECTURE.md), [known issues](src/robot_control/KNOWN_ISSUES.md),
and [current gaps](src/robot_control/TODO.md) for detail.
