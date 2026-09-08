# `v3_snap/hard_loose/rb_00022` navigation simulation

Candidate file: `2push__env__obstacle_1_movable__f110e8c4.json`

The environment is the self-contained XML staged on `dhruv-linux` at `real_exp/environments/v3_snap/hard_loose/rb_00022/env.xml`. Its sweep record matches the requested `obstacle_1_movable` episode: `2push`, hard, 5/125 successful cells, and 2/5 successful contact cells.

Both pure-navigation variants used the real planner/controller/executor stack at the paper's speed of 0.3 and the real 30 Hz control rate.

- `nav_baseline_ignore`: failure after 36.90 simulated seconds, 17.09 cm from the goal. It displaced `obstacle_0_movable` 26.61 cm and `obstacle_1_movable` 28.99 cm, then exhausted five blocked retries.
- `nav_baseline_penalise`: failure after 83.63 simulated seconds, 27.36 cm from the goal. It displaced the two movables 2.17 cm and 2.90 cm, then exhausted five under-commanded retries.
- Amarel navigation job: `61248748`
- Amarel video array: `61248760`

Each mode directory contains `result.json` and the complete 30 Hz `trajectory.jsonl`. The two MP4 files render every trajectory row and numerically verify the rendered robot/object poses against the logs.
