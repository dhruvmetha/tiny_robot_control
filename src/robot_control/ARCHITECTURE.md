# robot_control architecture

## Scope and current status

`robot_control` owns the single-robot orchestration and execution layer in the
NAMO workspace. It assembles observations, state, planners, controllers,
simulation or hardware environments, an optional GUI, and diagnostics into one
runtime.

The NAMO boundary supports scene conversion, object mapping, canonical binding
loading, exact-chain simulation verification, prior-plan reuse, and
service-backed fresh/full search. Verification and reuse do not use
`NAMOPlanningService`; full search lazily imports the in-process class from the
sibling `namo_cpp` checkout. With the current sibling checkout and its
`env.robotlearning.sh` sourced, the service and compiled binding import and the
full-search path is operational. That is a compatibility result for this
checkout/environment, not a version-independent API guarantee.

## Runtime data flow

[`Runtime`](runtime.py) creates the environment and observation source for the
selected mode, a [`WorldState`](core/world_state.py), the standard controller
set, a [`ControlCoordinator`](coordinator.py), and the optional GUI. When a
planner is configured, it also creates a [`SubgoalExecutor`](executor.py).

The autonomous sense-plan-act path is:

```text
Observation source -> WorldState -> Planner -> SubgoalExecutor -> Controller -> Action -> Environment
                                      |              |                |
                                      +------ Runtime + DiagnosticsRecorder ------+
```

Sensors publish `Observation` values through the topics in
[`core/topics.py`](core/topics.py). `WorldState` caches and republishes the
latest observation. At the configured frequency, `Runtime` asks the planner for
a subgoal when needed, lets `SubgoalExecutor` dispatch that subgoal to the
navigation or push controller, and applies the resulting differential-drive
`Action` to the environment. The runtime attaches
[`DiagnosticsRecorder`](diagnostics/recorder.py) to planning and execution
lifecycle events and can capture scene and replay artifacts.

Without a planner, the interactive path replaces `Planner -> SubgoalExecutor`
with `ControlCoordinator`, which routes GUI input to the currently selected
controller.

## Components

### Core state and messages

- [`core/types.py`](core/types.py) defines observations, object poses, wheel
  actions, navigation and push subgoals, and workspace geometry.
- [`core/topics.py`](core/topics.py) names the in-process PyPubSub channels.
- [`core/world_state.py`](core/world_state.py) aggregates the latest sensor
  observation.
- [`core/serialization.py`](core/serialization.py) encodes observations for the
  ZeroMQ camera boundary.
- [`core/object_defs.py`](core/object_defs.py) loads physical object definitions
  from YAML.

### Environments and hardware transport

- [`environment/base.py`](environment/base.py) defines the environment
  interface.
- [`environment/sim.py`](environment/sim.py) provides `SimEnv`, a
  differential-drive simulation used by the runtime.
- [`environment/real.py`](environment/real.py) provides `RealEnv` for a single
  physical robot.
- [`environment/action_sender.py`](environment/action_sender.py) defines the
  sender protocol expected by `RealEnv`.
- [`environment/micromvp_adapter.py`](environment/micromvp_adapter.py) adapts
  MicroMVP's serial sender to that protocol; the serial implementation remains
  outside `robot_control`.

### Observation nodes

- [`nodes/sim_sensor.py`](nodes/sim_sensor.py) polls `SimEnv` and publishes
  simulation observations.
- [`nodes/camera_sensor.py`](nodes/camera_sensor.py) captures local camera
  frames; [`camera/observer.py`](camera/observer.py) turns marker detections
  into observations.
- [`nodes/remote_observer.py`](nodes/remote_observer.py) receives serialized
  observations from `camera_service` over ZeroMQ.
- [`nodes/remote_record_client.py`](nodes/remote_record_client.py) controls
  recording on that remote camera service, while
  [`utils/camera_recorder.py`](utils/camera_recorder.py) supports local camera
  recording.

### Planning

- [`planner/base.py`](planner/base.py) defines the task-planner interface, and
  [`planner/sequence_planner.py`](planner/sequence_planner.py) implements
  predefined navigation sequences.
- [`planner/rvg_planner.py`](planner/rvg_planner.py) and
  [`planner/wavefront_path_planner.py`](planner/wavefront_path_planner.py)
  provide obstacle-aware path planners used by navigation.
- [`planner/namo_planner.py`](planner/namo_planner.py) adapts NAMO results to the
  planner interface and maintains the push-subgoal execution state.
- [`planner/namo_bridge.py`](planner/namo_bridge.py) owns scene conversion,
  object-name mapping, exact-chain verification through the canonical binding,
  full-search calls through the service API, and conversion to `PushSubgoal`
  values.
- [`planner/namo_binding_loader.py`](planner/namo_binding_loader.py) resolves
  and verifies the canonical `namo_cpp/build_python` binding.

### Execution and controllers

[`SubgoalExecutor`](executor.py) dispatches `NavigateSubgoal` values to
`NavigationController` and `PushSubgoal` values to `PushController`.
[`ControlCoordinator`](coordinator.py) performs the corresponding interactive
controller selection and GUI event routing.

The concrete controllers are:

- [`controller/keyboard.py`](controller/keyboard.py) for direct wheel control.
- [`controller/navigation.py`](controller/navigation.py) for planned movement
  to a target pose.
- [`controller/follow_path.py`](controller/follow_path.py) for path tracking.
- [`controller/push.py`](controller/push.py) for approach, alignment, and push
  execution.
- [`controller/edge_points.py`](controller/edge_points.py) for object-face and
  push-contact geometry.

For controller equations, units, and calibration details, use
[Navigation and wheel commands](../../docs/NAVIGATION_AND_WHEEL_COMMANDS.md),
[Push geometry](../../docs/PUSH_GEOMETRY.md), and
[Push-duration calibration](../../docs/PUSH_DURATION_CALIBRATION.md).

### Presentation and recording

[`gui/`](gui) contains the PySide6 window, canvas, sidebar, and settings panel.
[`diagnostics/`](diagnostics) contains structured event recording, scene
capture, rendering, and simulation replay helpers. These helpers write runtime
artifacts; they are not part of the control data model.

## Simulation flow

`RuntimeConfig(mode="sim")` selects `SimEnv` and `SimSensorNode`. `SimEnv` runs
its differential-drive kinematics in a background thread, while
`SimSensorNode` publishes observations to `WorldState` at the configured
frequency.

When `RuntimeConfig.planner` is set, the runtime enters autonomous mode and
uses `SubgoalExecutor` to execute each returned navigation or push subgoal.
Without a planner, `ControlCoordinator` lets the selected keyboard, navigation,
or follow-path controller drive the simulation interactively.

## Real-hardware flow

`RuntimeConfig(mode="real")` uses [`config/real.yaml`](../../config/real.yaml)
and [`config/objects.yaml`](../../config/objects.yaml) for camera, workspace,
robot, serial, and object definitions. A local `CameraSensorNode` plus
`ArucoObserver`, or a `RemoteObserverNode` connected to `camera_service`,
provides observations to the same `WorldState` interface used in simulation.

`RealEnv` sends normalized left/right wheel actions through the configured
`ActionSender`. Its default `MicromvpAdapter` delegates to MicroMVP's serial
sender. Dry-run mode leaves sensing and orchestration active without sending
wheel commands.

## NAMO boundary

The repository contains the boundary code needed to:

1. generate a MuJoCo scene XML from an observation;
2. map real object names to simulator object names and back;
3. locate and validate the canonical `namo_rl` binding; and
4. simulate an exact push chain with `NAMOPlanBridge.verify_chain()` using the
   compiled binding directly;
5. convert planned pushes into `PushSubgoal` values for the runtime.

`verify_chain()` does not call the bridge's service loader. Consequently,
`replan-reuse-only` and the reuse phase of `replan` can validate a prior plan
when the current scene, prior plan, and canonical compiled binding are
available. A successful verification produces the next simulation candidate;
failed reuse-only reports `needs_remote_search` and stops.

Fresh/full search calls the service loader, which lazily imports
`namo.services.NAMOPlanningService`. The current sibling checkout exports that
class from `python/namo/services/planning_service.py`, while its compiled
`namo_rl` is loaded from `build_python`, after `env.robotlearning.sh` is
sourced. On 2026-08-27 the pinned environment passed all 383 repository tests
and simulation-only model/uniform prior x search/reactive execution probes all
returned successful plans. Therefore `replan-full-search-only`, fresh
`run_namo` search, and `replan`'s fallback after failed reuse are available in
that current paired environment. Re-check both imports when either checkout or
environment changes; the result does not validate real hardware end to end.
See the [closed-loop guide](../../closed_loop_sessions/README.md) for operator
flow.

## External dependencies

- The sibling `namo_cpp` repository supplies NAMO Python code, the compiled
  `namo_rl` binding, planner configuration/data, and MuJoCo-backed planning.
- Local or remote camera operation depends on OpenCV, marker calibration, and
  the repository's `camera_service` protocol; real wheel transport depends on
  MicroMVP and the configured robot/serial service.
- MuJoCo and its rendering/video dependencies are required by simulated push,
  verification, and replay tools.
- `controller/motion_planner/rvg` is a Git submodule that supplies the RVG
  dependency used by `RVGPlanner`.

## Repository structure

```text
robot_control/
├── README.md
├── config/                         runtime, controller, object, and calibration YAML
├── scripts/                        operator, diagnostics, calibration, and test entrypoints
├── src/robot_control/
│   ├── camera/                     local camera observation and workspace geometry
│   ├── controller/                 interactive, navigation, path, and push controllers
│   ├── core/                       shared types, topics, state, and serialization
│   ├── diagnostics/                event logs, scene capture, render, and replay
│   ├── environment/                simulation, real environment, and sender adapter
│   ├── gui/                        PySide6 presentation layer
│   ├── nodes/                      simulation, camera, and remote observation nodes
│   ├── planner/                    sequence, path, and NAMO planning adapters
│   ├── utils/                      geometry, wavefront, camera, and XML helpers
│   ├── coordinator.py              interactive controller routing
│   ├── executor.py                 autonomous subgoal dispatch
│   └── runtime.py                  component setup and control loop
├── tests/                          automated package tests
├── docs/                           focused controller and calibration notes
├── closed_loop_sessions/           closed-loop workspace helpers
├── real_test_envs/                 captured scene fixtures
└── push_calibration/               recorded calibration comparisons
```

The top-level [README](../../README.md) indexes the focused operator and
calibration documentation. Wavefront-specific open questions are tracked in
[Wavefront unification follow-ups](../../docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md)
rather than duplicated here.
