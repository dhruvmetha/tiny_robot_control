# Navigation and wheel commands

This guide describes the navigation path that the current checkout actually
uses: path planning, path following, normalized wheel actions, and the
simulation or hardware boundary. For the wider runtime data flow, see the
[architecture guide](../src/robot_control/ARCHITECTURE.md).

## Configuration precedence

Production runtime construction loads
[`config/controller.yaml`](../config/controller.yaml) through
[`controller/config.py`](../src/robot_control/controller/config.py). The
checked-in navigation settings are currently:

| Field | Current value | Effect |
|---|---:|---|
| `navigation.max_speed` | `0.4` | Normalized cap for the navigation path follower |
| `navigation.planner` | `wavefront` | Selects the path planner |
| `navigation.goal_tolerance_ratio` | `0.2` | Goal tolerance as a fraction of effective robot size |
| `navigation.pre_rotation_skip_angle` | `45` degrees | Skips pre-rotation when already aligned |
| `navigation.rotation_tolerance_deg` | `2.5` degrees | Rotation completion band |
| `navigation.rotation_stable_time` | `0.5` seconds | Required time inside that band |
| `navigation.rotation_speed_min` / `max` | `0.15` / `0.25` | Normalized in-place rotation range |
| `navigation.wheel_deadband` | `0.05` | Deadband for the navigation controller's in-place rotation |

These are runtime values, not the fallback constructor values in the Python
dataclasses. The fallbacks are used only when the YAML file or a field is
missing. `RuntimeConfig.nav_speed_override` (exposed by operator scripts as
`--nav-speed`) takes precedence over the YAML path-following cap at
construction time.

After construction, GUI speed presets behave differently by runtime mode:

- In non-autonomous interactive mode, `ControlCoordinator` applies the speed
  controls only to keyboard and standalone path-following controllers.
  The navigation and push path followers retain their YAML or CLI caps.
- In autonomous GUI mode, `Runtime` registers its own `+`/`-` key and speed
  button callbacks. Those callbacks call `NavigationController.set_speed()`
  and therefore replace the navigation path-following cap with the selected
  preset. They do not change the push controller's separate path-follower cap.

`NavigationController.set_speed()` updates its `FollowPathController`. It does
not change pre-rotation, post-rotation, or standalone `ROTATING`, which use
`navigation.rotation_speed_min` and `navigation.rotation_speed_max` instead.
The navigation override is therefore not a global wheel-command or hardware
ceiling.

The runtime prints the effective navigation and push path-follower caps when it
creates the controllers. Check that line before a hardware run when using
overrides.

## Runtime path

The navigation path is:

```text
goal + current observation
    -> WavefrontPathPlanner or RVGPlanner
    -> NavigationController
    -> FollowPathController
    -> Action(left_speed, right_speed)
    -> SimEnv or RealEnv
```

[`Runtime._create_controllers`](../src/robot_control/runtime.py) selects the
planner and injects the same `NavigationController` into the `PushController`
for approach and forward-retreat motion.

### Path planner selection

`navigation.planner` supports the checked-in values `wavefront` and `rvg`:

- `wavefront` constructs
  [`WavefrontPathPlanner`](../src/robot_control/planner/wavefront_path_planner.py).
  It builds a 0.5 cm grid, inflates obstacles for the current robot footprint,
  applies the configured 2 cm proximity-cost distance and weight `5.0`, then
  runs Dijkstra and gradient descent. With the current
  `navigation.wavefront_debug_dir`, images go to the ignored
  `output_dump/wavefront/` directory.
- `rvg` constructs
  [`RVGPlanner`](../src/robot_control/planner/rvg_planner.py), which uses the
  RVG binding and `navigation.robot_geometry_scale`. That binding must be
  available for this selection to return a path.

Both planners implement the same `plan(start, goal, obstacles)` interface and
return waypoints in workspace centimeters. An empty or trivial path leaves
navigation idle.

### Navigation state machine

[`NavigationController`](../src/robot_control/controller/navigation.py) follows
this sequence for a navigation request:

```text
IDLE -> PRE_ROTATING -> FOLLOWING -> POST_ROTATING -> FINISHED
```

Pre-rotation may be skipped when the robot already faces the path.
Post-rotation runs only when the goal includes an orientation. Rotation uses
opposite wheel signs, varies between the configured minimum and maximum, and
must remain within the tolerance for the stable-hold interval before it is
complete.

A standalone `rotate_to()` request does not traverse the navigation sequence:

```text
IDLE -> ROTATING -> FINISHED
```

Calling `rotate_to()` sets `ROTATING` directly; canceling any request returns
the controller to `IDLE`.

During `FOLLOWING`, the controller delegates to
[`FollowPathController`](../src/robot_control/controller/follow_path.py). The
follower:

1. resamples the planner path and limits how far its target search can advance;
2. combines pure-pursuit curvature with a gated cross-track-error PD term;
3. rotates in place when heading error is large;
4. schedules speed down for heading error and curvature;
5. converts curvature to differential wheel commands;
6. preserves the wheel ratio while limiting the larger magnitude to the
   effective path-follower cap; and
7. boosts a nonzero command below the deadband, then clamps each output to
   `[-1, 1]`.

The follower currently has its own `0.05` class constant for deadband scaling;
it does not read `navigation.wheel_deadband`. The configured field controls the
pre-, post-, and standalone rotation actions calculated by
`NavigationController`.

The emitted values are normalized commands, not centimeters per second,
radians per second, or raw PWM counts.

## Environment boundary

[`Action`](../src/robot_control/core/types.py) carries only normalized
`left_speed` and `right_speed`, each in `[-1, 1]`.

### Lightweight simulation

[`SimEnv`](../src/robot_control/environment/sim.py) multiplies each normalized
wheel value by `SimConfig.max_wheel_speed` and advances its differential-drive
ICC kinematics at the configured physics rate. This is the lightweight
`robot_control` simulation used by the general runtime; it is distinct from
the MuJoCo C++ push replay described in the
[simulation calibration guide](./SIM_CALIBRATION_PLAN.md).

### Real robot

[`RealEnv`](../src/robot_control/environment/real.py) passes the normalized
values and configured robot ID to the `ActionSender` interface. The default
[`MicromvpAdapter`](../src/robot_control/environment/micromvp_adapter.py):

1. wraps the values in `micromvp.core.models.Action`;
2. hands that action to micromvp's `SerialActionSender`;
3. uses `config/real.yaml` for the serial port, baud rate, send rate, and
   optional right-wheel inversion.

`robot_control` does not duplicate micromvp's serial protocol or firmware
scaling. The adapter is the ownership boundary: protocol framing, access-point
transport, and motor-side interpretation belong to micromvp and the robot
firmware. Emergency stops use the sender's immediate path; normal actions use
its rate-limited path.

## Diagnosing wheel behavior

Before changing gains, identify the layer that differs from expectation:

- Confirm the runtime's printed effective `nav` path-follower cap and whether
  `--nav-speed` was supplied.
- Confirm `navigation.planner` and inspect ignored wavefront images when a path
  is absent or unexpectedly close to obstacles.
- Inspect the controller state and follower mode (`ALIGN`, `ROTATE_IN_PLACE`,
  `TRACK`, or `ACQUIRE`) when the wheel pair is slower or has opposite signs.
  `ALIGN` is the follower's hysteretic large-heading mode;
  `ROTATE_IN_PLACE` is its far-from-path, wrong-way recovery mode.
- Compare the normalized action at the environment boundary. A difference
  after that boundary belongs to `SimConfig`, micromvp, serial configuration,
  or firmware rather than the path follower.

Push contact geometry and edge numbering are documented in
[Push Geometry](./PUSH_GEOMETRY.md). Push tick semantics and the real/sim
measurement workflow are documented in
[Push Duration Calibration](./PUSH_DURATION_CALIBRATION.md).
