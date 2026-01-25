# robot_control Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              USER / NAMO_TASK                               │
│                                                                             │
│  python -m namo_task.run --sim --goal 50 30 --visualize                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                                 RUNTIME                                      │
│                         (robot_control/runtime.py)                          │
│                                                                             │
│  Orchestrates the Sense-Plan-Act loop at fixed frequency (30 Hz)           │
│  Manages GUI visualization (optional, uses micromvp.gui.MVPWindow)         │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         MAIN LOOP                                    │   │
│  │                                                                      │   │
│  │  while running:                                                      │   │
│  │      observation = environment.observe()      # SENSE               │   │
│  │      state = estimator.update(observation)                          │   │
│  │                                                                      │   │
│  │      if controller.is_done(state, subgoal):                         │   │
│  │          subgoal = planner.plan(state)        # PLAN                │   │
│  │                                                                      │   │
│  │      action = controller.step(state, subgoal) # ACT                 │   │
│  │      environment.apply_action(action)                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
          │              │                │                    │
          ▼              ▼                ▼                    ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐
│ ENVIRONMENT  │ │  ESTIMATOR   │ │   PLANNER    │ │     CONTROLLERS      │
│   (wrapper)  │ │              │ │              │ │      (wrappers)      │
└──────────────┘ └──────────────┘ └──────────────┘ └──────────────────────┘
       │                                                     │
       │                                                     │
       ▼                                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              micromvp                                        │
│                    (Hardware Abstraction Layer)                              │
│                                                                             │
│   SimEnv, RealPushEnv, NavigationController, MVPWindow, RVG                │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Wrapper Pattern

robot_control wraps micromvp to provide a single-robot Sense-Plan-Act interface:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ADAPTER PATTERN                                     │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                      micromvp (Multi-Robot)                          │  │
│   │                                                                      │  │
│   │   observe() → Dict[int, RobotObservation]                           │  │
│   │   apply_actions(Dict[int, Action])                                   │  │
│   │                                                                      │  │
│   │   Controllers manage their own state:                                │  │
│   │     - set_path(path)                                                │  │
│   │     - rotate_to(theta)                                              │  │
│   │     - step(observation) → action                                    │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    │ WRAPPED BY                             │
│                                    ▼                                        │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                    robot_control (Single-Robot)                      │  │
│   │                                                                      │  │
│   │   observe() → Observation                                            │  │
│   │   apply_action(Action)                                               │  │
│   │                                                                      │  │
│   │   Controllers receive Subgoals from Planner:                         │  │
│   │     - step(state, subgoal) → action                                 │  │
│   │     - is_done(state, subgoal) → bool                                │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Details

### 1. Environment (Wrapper)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    robot_control/environment/                               │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      Environment (ABC)                               │   │
│  │                        base.py                                       │   │
│  │                                                                      │   │
│  │  observe() → Observation      # Single robot                        │   │
│  │  apply_action(Action)         # Single robot                        │   │
│  │  start() → bool                                                      │   │
│  │  close()                                                             │   │
│  │  workspace_config → WorkspaceConfig                                  │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                          ▲                    ▲                             │
│                          │                    │                             │
│         ┌────────────────┴──┐          ┌──────┴────────────────┐           │
│         │                   │          │                       │           │
│  ┌──────────────────┐  ┌──────────────────┐                                │
│  │  SimEnvironment  │  │ RealEnvironment  │                                │
│  │     sim.py       │  │     real.py      │                                │
│  │                  │  │                  │                                │
│  │  Wraps SimEnv    │  │ Wraps RealPushEnv│                                │
│  └──────────────────┘  └──────────────────┘                                │
│           │                     │                                           │
│           └──────────┬──────────┘                                           │
│                      ▼                                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         micromvp                                     │   │
│  │                                                                      │   │
│  │   SimEnv (sim_env.py)          RealPushEnv (real_push_env.py)       │   │
│  │   - Physics simulation         - ArUco marker detection             │   │
│  │   - DDR kinematics             - UDP robot communication            │   │
│  │   - Multi-robot Dict API       - Camera + obstacle tracking         │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2. Controllers

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     robot_control/controller/                               │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                       Controller (ABC)                               │   │
│  │                          base.py                                     │   │
│  │                                                                      │   │
│  │  step(state: State, subgoal: Subgoal) → Action                      │   │
│  │  is_done(state: State, subgoal: Subgoal) → bool                     │   │
│  │  reset()                                                             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                          ▲                    ▲                             │
│                          │                    │                             │
│         ┌────────────────┴──┐          ┌──────┴────────────────┐           │
│         │                   │          │                       │           │
│  ┌────────────────────┐  ┌────────────────────┐                            │
│  │NavigationController│  │  PushController    │                            │
│  │   navigation.py    │  │     push.py        │                            │
│  │                    │  │                    │                            │
│  │  WRAPPER around    │  │  Approach → Align  │                            │
│  │  micromvp's        │  │  → Push → Done     │                            │
│  │  NavigationCtrl    │  │                    │                            │
│  └────────────────────┘  └────────────────────┘                            │
│           │                                                                 │
│           │ Wraps                                                           │
│           ▼                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              micromvp/controller/navigation_controller/              │   │
│  │                                                                      │   │
│  │   NavigationController                                               │   │
│  │   - Pure Pursuit + CTE-PD path following                            │   │
│  │   - In-place rotation (rotate_to)                                   │   │
│  │   - State machine (IDLE, FOLLOWING, ROTATING, FINISHED)             │   │
│  │   - Path resampling, no-skip gating                                 │   │
│  │   - set_path(), clear_path(), set_speed()                           │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3. Planner

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        Planner (ABC)                                 │   │
│  │                robot_control/planner/base.py                         │   │
│  │                                                                      │   │
│  │  plan(state: State) → Optional[Subgoal]                             │   │
│  │  is_complete(state: State) → bool                                    │   │
│  │  reset()                                                             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    ▲                                        │
│                                    │                                        │
│                     ┌──────────────┴──────────────┐                        │
│                     │                             │                        │
│              ┌──────────────┐             ┌──────────────┐                 │
│              │ NAMOPlanner  │             │ (Future...)  │                 │
│              │  (stub)      │             │ PatrolPlanner│                 │
│              │              │             │ etc.         │                 │
│              │ namo_task/   │             │              │                 │
│              │ planner.py   │             │              │                 │
│              └──────────────┘             └──────────────┘                 │
│                     │                                                       │
│                     │ (future integration)                                  │
│                     ▼                                                       │
│              ┌──────────────┐                                              │
│              │  namo_cpp    │                                              │
│              │              │                                              │
│              │ - MuJoCo sim │                                              │
│              │ - Wavefront  │                                              │
│              │ - NAMO search│                                              │
│              └──────────────┘                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow

```
                    robot_control/core/types.py
                    ───────────────────────────

┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│   Observation    │     │      State       │     │      Action      │
│                  │     │                  │     │                  │
│  robot_x         │     │  robot_x         │     │  left_speed      │
│  robot_y         │ ──▶ │  robot_y         │ ──▶ │  right_speed     │
│  robot_theta     │     │  robot_theta     │     │                  │
│  objects: Dict   │     │  robot_vx, vy    │     │  (0.0 to 1.0)    │
│  timestamp       │     │  robot_omega     │     │                  │
│                  │     │  objects: Dict   │     │                  │
│  (from env)      │     │  (+ velocities)  │     │  (to env)        │
└──────────────────┘     └──────────────────┘     └──────────────────┘
        │                         │                        ▲
        │    StateEstimator       │                        │
        └─────────────────────────┘                        │
                                                           │
                    robot_control/core/subgoals.py         │
                    ──────────────────────────────         │
                                                           │
┌──────────────────┐     ┌──────────────────┐             │
│  NavigateSubgoal │     │   PushSubgoal    │             │
│                  │     │                  │             │
│  target_x        │     │  object_id       │             │
│  target_y        │     │  push_direction  │  Controller │
│  path (optional) │     │  push_distance   │ ────────────┘
│  tolerance       │     │                  │
│                  │     │                  │
│  (from planner)  │     │  (from planner)  │
└──────────────────┘     └──────────────────┘
```

---

## GUI Integration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         VISUALIZATION (Optional)                             │
│                                                                             │
│   Runtime (visualize=True)                                                  │
│       │                                                                     │
│       ├── Creates MVPWindow (from micromvp.gui)                            │
│       │                                                                     │
│       ├── Control loop in background thread                                │
│       │       └── _step() at 30 Hz                                         │
│       │                                                                     │
│       ├── GUI update timer on Qt main thread                               │
│       │       └── _update_gui() at 30 FPS                                  │
│       │             ├── Update car state                                   │
│       │             ├── Draw goal marker                                   │
│       │             ├── Draw path                                          │
│       │             └── Draw target point                                  │
│       │                                                                     │
│       └── Callbacks                                                         │
│             ├── on_key_press (ESC to quit)                                 │
│             └── on_curve_drawn (user draws path → robot follows)           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## File Structure

```
namo/
├── robot_control/                    # GENERAL FRAMEWORK
│   ├── __init__.py
│   ├── ARCHITECTURE.md              # This file
│   ├── runtime.py                   # Main Sense-Plan-Act loop
│   ├── qt_compat.py                 # PySide6/PyQt6 compatibility
│   │
│   ├── core/
│   │   ├── __init__.py
│   │   ├── types.py                 # Observation, State, Action
│   │   └── subgoals.py              # SubgoalType, NavigateSubgoal, PushSubgoal
│   │
│   ├── environment/
│   │   ├── __init__.py
│   │   ├── base.py                  # Environment ABC
│   │   ├── sim.py                   # SimEnvironment (wraps micromvp.SimEnv)
│   │   └── real.py                  # RealEnvironment (wraps micromvp.RealPushEnv)
│   │
│   ├── estimator/
│   │   ├── __init__.py
│   │   ├── base.py                  # StateEstimator ABC
│   │   └── simple.py                # SimpleStateEstimator (finite differences)
│   │
│   ├── controller/
│   │   ├── __init__.py
│   │   ├── base.py                  # Controller ABC
│   │   ├── navigation.py            # NavigationController (WRAPS micromvp)
│   │   └── push.py                  # PushController
│   │
│   └── planner/
│       ├── __init__.py
│       └── base.py                  # Planner ABC
│
├── namo_task/                        # NAMO-SPECIFIC
│   ├── __init__.py
│   ├── subgoals.py                  # NAMOPushSubgoal (edge_idx → direction)
│   ├── planner.py                   # NAMOPlanner (stub for namo_cpp)
│   └── run.py                       # Entry point
│
└── micromvp_push_dhruv/micromvp_v4/  # HARDWARE ABSTRACTION (external)
    └── src/micromvp/
        ├── core/                     # WorkspaceConfig, Action, RobotObservation
        ├── env/                      # SimEnv, RealPushEnv
        ├── controller/               # NavigationController (Pure Pursuit + CTE-PD)
        ├── coordinator/              # Multi-robot orchestration (not used)
        ├── gui/                      # MVPWindow (PyQt6/PySide6)
        └── rvg/                      # Motion planning (Rotation-stacked Visibility Graph)
```

---

## What robot_control Provides vs micromvp

| Component | micromvp | robot_control |
|-----------|----------|---------------|
| **Environments** | SimEnv, RealPushEnv (multi-robot Dict API) | Wrappers for single-robot |
| **Controllers** | NavigationController, WASDController, etc. | Wraps NavigationController, adds PushController |
| **Orchestration** | Coordinator (reactive, GUI-driven) | Runtime + Planner (autonomous, subgoal-based) |
| **State Estimation** | Inside controllers | Separate StateEstimator |
| **GUI** | MVPWindow | Reuses MVPWindow |
| **Motion Planning** | RVG library | Uses micromvp's RVG via NavigationController |

---

## Usage

```bash
# Simulation with visualization
python -m namo_task.run --sim --goal 50 30 --visualize

# Simulation headless
python -m namo_task.run --sim --goal 50 30

# Real robot
python -m namo_task.run --robot-id 1 --goal 50 30 --visualize

# With verbose logging
python -m namo_task.run --sim --goal 50 30 -v
```

---

## Key Design Decisions

1. **Wrapper Pattern**: robot_control wraps micromvp rather than modifying it
2. **Single-Robot Focus**: Adapts multi-robot API for single-robot NAMO task
3. **Subgoal-Based Control**: Planner decides WHAT (Navigate/Push), Controller does HOW
4. **Optional Visualization**: Same code works headless or with GUI
5. **No Code Duplication**: NavigationController wraps micromvp's implementation
6. **Separation of Concerns**:
   - micromvp: Hardware abstraction, path following, GUI
   - robot_control: Task orchestration, state estimation, subgoal management
   - namo_task: NAMO-specific planning logic
