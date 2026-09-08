# `hard_019` navigation simulation

Canonical scene UID: `1push__hard_019__ce25d680`

Canonical source: `/common/users/dm1487/scratch_namo/real_buildable/pool2/hard_202/rb_00027/env.xml`

The iLab `/common` filesystem was not reachable through the expired Arrakis master session. The staged self-contained `env.xml` was therefore materialized from `canonical_card.json`, the UID-matched gallery card on `dhruv-linux`. Before submission, an exact-value audit matched every card-defined robot, goal, static-wall, and movable-object pose and dimension to the XML.

Both pure-navigation variants used the real planner/controller/executor stack at the paper's speed of 0.3 and the real 30 Hz control rate.

- `nav_baseline_ignore`: success in 13.33 simulated seconds, 1.59 cm from the goal. It crossed the movable and bulldozed it 40.87 cm.
- `nav_baseline_penalise`: failure after 34.03 simulated seconds, 30.83 cm from the goal. It moved the movable 5.89 cm, performed five recorded stuck retries, and terminated with `stuck_retry_limit_reached`.
- Amarel job: `61248720`

Each mode directory contains `result.json` and the complete 30 Hz `trajectory.jsonl`.
