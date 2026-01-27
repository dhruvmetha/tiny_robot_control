# CLAUDE.md - robot_control

This file provides guidance to Claude Code when working with the robot_control framework.

## Overview

**robot_control** is the real-robot execution bridge that sits between NAMO planning (namo_cpp) and physical hardware (micromvp). It provides a clean Sense-Plan-Act abstraction for executing push manipulation tasks on real MicroMVP robots.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         NAMO PLANNING (namo_cpp)                             │
│                                                                             │
│   StandardIDFS, TreeIDFS, MCTS → Plan: [(obj_id, edge_idx, depth), ...]    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ (future) SIM2REAL bridge
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         robot_control (this package)                         │
│                                                                             │
│   NAMOPlanner: Converts NAMO plan → sequence of Subgoals                   │
│   Controllers: Execute Subgoals (NavigateSubgoal, PushSubgoal)             │
│   Runtime: Sense-Plan-Act loop at 30 Hz                                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ wraps
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         micromvp (hardware abstraction)                      │
│                                                                             │
│   SimEnv, RealPushEnv, NavigationController, MVPWindow, RVG                │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Quick Commands

```bash
cd robot_control && pip install -e .

# Simulation with keyboard control
python scripts/test_control.py --controller keyboard

# Click-to-navigate with RVG path planning
PYTHONPATH=src/robot_control/controller/motion_planner/rvg:$PYTHONPATH \
    python scripts/test_navigation.py

# Camera calibration and ArUco detection
python scripts/test_camera.py

# Generate ArUco markers for printing
python scripts/generate_aruco_markers.py

# Capture simulation state to MuJoCo XML
python scripts/capture_to_xml.py --output env.xml
```

## Directory Structure

```
robot_control/
├── src/robot_control/
│   ├── core/                    # Data types
│   │   ├── types.py             # Observation, Action, Subgoal, WorkspaceConfig
│   │   ├── topics.py            # Pub/sub topic names
│   │   └── world_state.py       # Aggregated world state
│   │
│   ├── environment/             # Environment abstraction
│   │   ├── base.py              # Environment ABC
│   │   ├── sim.py               # SimEnvironment (wraps micromvp.SimEnv)
│   │   ├── real.py              # RealEnvironment (camera + UDP)
│   │   └── micromvp_adapter.py  # Only place that imports micromvp
│   │
│   ├── controller/              # Motion controllers
│   │   ├── base.py              # Controller ABC: step(obs, subgoal) → Action
│   │   ├── navigation.py        # Pure Pursuit + CTE-PD path following
│   │   ├── push.py              # Push execution (approach → align → push)
│   │   └── motion_planner/rvg/  # RVG library for obstacle-avoiding paths
│   │
│   ├── planner/                 # Task planners
│   │   └── base.py              # Planner ABC: plan(obs) → Subgoal
│   │
│   ├── camera/                  # Vision system
│   │   └── observer.py          # ArucoObserver: marker detection → poses
│   │
│   ├── nodes/                   # Pub/sub sensor nodes
│   │   ├── sim_sensor.py        # Publishes simulation observations
│   │   └── camera_sensor.py     # Publishes camera frames
│   │
│   ├── gui/                     # PyQt6/PySide6 visualization
│   │   └── window.py            # Uses micromvp's MVPWindow
│   │
│   └── runtime.py               # Main Sense-Plan-Act loop
│
├── config/                      # YAML configuration
│   ├── real.yaml                # Camera, markers, serial port
│   ├── controller.yaml          # Speed presets, pure pursuit params
│   └── objects.yaml             # Object marker definitions
│
└── scripts/                     # Test and example scripts
```

## Core Abstractions

### Runtime Loop (30 Hz)

```python
while running:
    observation = env.observe()           # SENSE
    if planner.is_complete(obs):
        break
    if controller.is_done(obs, subgoal):
        subgoal = planner.plan(obs)       # PLAN
        controller.reset()
    action = controller.step(obs, subgoal) # ACT
    env.apply(action)
```

### Data Types (`core/types.py`)

| Type | Purpose |
|------|---------|
| `Observation` | Robot pose (x, y, θ) + object poses + goal position |
| `Action` | Differential drive: `left_speed`, `right_speed` ∈ [-1, 1] |
| `Subgoal` | Abstract target (NavigateSubgoal, PushSubgoal) |
| `ObjectPose` | Object position, orientation, dimensions, is_static flag |
| `WorkspaceConfig` | Workspace and robot geometry |

### Controller Interface

```python
class Controller(ABC):
    def step(self, obs: Observation, subgoal: Subgoal) -> Action: ...
    def is_done(self, obs: Observation, subgoal: Subgoal) -> bool: ...
    def reset(self) -> None: ...
```

### Planner Interface

```python
class Planner(ABC):
    def plan(self, obs: Observation) -> Optional[Subgoal]: ...
    def is_complete(self, obs: Observation) -> bool: ...
    def reset(self) -> None: ...
```

## Integration with micromvp

robot_control **wraps** micromvp rather than extending it:

| robot_control | micromvp (wrapped) |
|---------------|-------------------|
| Single-robot `observe() → Observation` | Multi-robot `observe() → Dict[int, RobotObs]` |
| Subgoal-based control | Internal state machines |
| Task-level planning | Reactive path following |

**Single integration point**: `environment/micromvp_adapter.py`

## Integration with namo_cpp (SIM2REAL)

The integration path for executing NAMO plans on real robots:

```
namo_cpp plan output: [(object_id, edge_idx, depth), ...]
                           │
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                    NAMOPlanner (to implement)                 │
│                                                              │
│  1. Parse plan output from namo_cpp                          │
│  2. For each (object_id, edge_idx, depth):                   │
│     - Generate NavigateSubgoal to approach position          │
│     - Generate PushSubgoal with direction/distance           │
│  3. Track progress through plan                              │
└──────────────────────────────────────────────────────────────┘
                           │
                           ▼
              NavigateSubgoal / PushSubgoal
                           │
                           ▼
              NavigationController / PushController
```

### Key Translation: edge_idx → push direction

namo_cpp uses `edge_idx` (0-3) to specify which face of the object to push:
- edge_idx 0: Push from +X direction
- edge_idx 1: Push from +Y direction
- edge_idx 2: Push from -X direction
- edge_idx 3: Push from -Y direction

`push_steps = depth + 1` (depth is the recursion depth in search)

### State Correspondence

| namo_cpp (simulation) | robot_control (real) |
|-----------------------|---------------------|
| MuJoCo qpos | Observation.robot_x/y/theta |
| MuJoCo object positions | Observation.objects dict |
| RLEnvironment.is_robot_goal_reachable() | Planner.is_complete() |

## Integration with sage_learning (Future)

sage_learning provides ML models for goal prediction that could enhance NAMO planning:

```
┌──────────────────────────────────────────────────────────────┐
│                    sage_learning models                       │
│                                                              │
│  Flow Matching / Diffusion → Predicted push goal positions   │
└──────────────────────────────────────────────────────────────┘
                           │
                           │ inference
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                    NAMOPlanner                                │
│                                                              │
│  Use ML predictions to:                                      │
│  1. Select which object to push (learned heuristic)          │
│  2. Predict goal positions for objects (goal inference)      │
│  3. Guide search in namo_cpp (informed search)               │
└──────────────────────────────────────────────────────────────┘
```

## RVG Motion Planning

The **Rotation-stacked Visibility Graph** library (`controller/motion_planner/rvg/`) provides SE(2) path planning with obstacle avoidance:

```python
from rvg import RVG

planner = RVG(workspace_polygon, obstacles, robot_radius)
path = planner.plan(start=(x, y, θ), goal=(x, y, θ))
```

Used by NavigationController for collision-free navigation between push operations.

## Vision System (Real Robot)

ArUco marker-based pose estimation:

| Marker Type | Dictionary | Purpose |
|------------|------------|---------|
| Workspace corners | 5×5 | Defines coordinate frame |
| Robot | 4×4 | Robot pose tracking |
| Objects | 6×6 | Object pose tracking |
| Goal | 4×4 (ID 0) | Goal position |

**Calibration workflow:**
1. Camera intrinsics: `camera/calibrate_camera.py`
2. Workspace detection: `camera/test_workspace_detection.py`
3. Robot detection: `camera/test_aruco_code_detection.py`

## Key Design Patterns

1. **Wrapper Pattern**: Wraps micromvp without modifying it
2. **Composition**: NavigationController uses FollowPathController internally
3. **Protocol-based**: ActionSender is a Protocol, not ABC
4. **Pub/sub decoupling**: Sensor nodes publish independently
5. **Thread-safe state**: Locks protect shared state in SimEnv, ArucoObserver

## Configuration

All tuning parameters in YAML config files:

```yaml
# config/controller.yaml
speed_presets:
  slow: 0.3
  normal: 0.5
  fast: 0.8

pure_pursuit:
  lookahead_distance: 10.0  # cm
  min_lookahead: 5.0

rotation:
  tolerance: 5.0  # degrees
  speed: 0.3
```

## Coordinate System

- **Origin**: Bottom-left corner of workspace
- **X-axis**: Positive rightward
- **Y-axis**: Positive upward
- **Theta**: Counter-clockwise from +X axis (radians)
- **Units**: Centimeters (cm) for positions

## Development Notes

- Environment setup: `pip install -e .` from `robot_control/`
- GUI requires: `pip install -e ".[gui]"` for PySide6
- Tests: `pytest` (requires `pip install -e ".[dev]"`)
- Single-robot focus: All APIs designed for one robot
