# robot_control - TODO / Missing Features

## SimEnv

### Not Yet Implemented

| Feature | Description | Priority |
|---------|-------------|----------|
| Dynamic objects | Objects stored in SimEnv, updated at runtime | Medium |
| Object physics | Collision detection, push simulation | Low |
| Multi-robot | Multiple robots in same env | Low |

### Current Limitation

- `objects` in `SimConfig` are static (pose set at init, never updated)
- No collision detection (robot passes through objects/walls)
- Single robot only

---

## Framework

### Not Yet Implemented

| Component | Description | Priority |
|-----------|-------------|----------|
| Concrete Controllers | NavigationController, PushController | High |
| Concrete Planners | Task-specific planners | High |
| Concrete Subgoals | NavigateSubgoal, PushSubgoal | High |
| RealEnv | Hardware interface (camera + UDP) | Medium |
| StateEstimator | Sensor fusion, filtering | Medium |
| GUI | Visualization | Low |

---

## Notes

- SimEnv physics identical to micromvp (DDR with ICC model)
- Wall-clock time drives simulation (not fixed timestep)
- `speed_scale` allows pause/fast-forward for testing
