# `rb_00182_mirror_x` penalise navigation simulation

This run reconstructs the nominal real build sheet for `rb_00182_mirror_x` and runs `nav_baseline_penalise` through the same `NavigationBaselinePlanner`, `NavigationController`, `FollowPathController`, and `SubgoalExecutor` stack used by the real baseline. MuJoCo supplies state and applies the wheel commands at the real 30 Hz control rate.

- Corrected goal: `(18.8652, 66.1089)` cm
- Robot start: `(25.1804, 12.5340)` cm at 180 degrees
- Navigation speed: `0.3`, matching the successful real trial
- Result: success, 41.47 simulated seconds, 1.86 cm from the goal
- Recovery: one blocked detection, retreat, and replan; success on attempt 2
- Contact outcome: `obstacle_1_movable` moved 8.4 cm; the other movable shifted about 0.6 cm
- Amarel navigation job: `61237318`
- Final corrected render job: `61237550`

The MP4 is rendered from all 1,245 rows of `trajectory.jsonl`. The renderer checks each generated MuJoCo frame against the logged robot and movable-object SE(2) poses; the accepted render has sub-numerical-precision position and angle error.

The scene is the nominal build-sheet geometry, not a reconstruction of the small pose deviations measured by the camera during the physical run. The real penalise trial needed two failed attempts and succeeded on attempt 3; this nominal simulation needed one failed attempt and succeeded on attempt 2.
