# Real-robot results

Generated from the accepted trial directories under `1hop_simple/results/real/` and
`1hop_multi_int/results/real/`. Runs under any `invalid/` path are excluded and are not
counted anywhere here. [EXPERIMENT_STATUS.md](EXPERIMENT_STATUS.md) is the collection
ledger and says what is still owed; this file reports what the collected trials measured.

The protocol is five physical trials per arm per environment, `trialN` on explicit seed
N-1. The three arms share one planner and differ only in how the next push is chosen:
model policy takes the ranker arg-max with zero physics rollouts, model search runs
best-first with the ranker as its prior, and uniform search runs the same search with a
uniform prior and no checkpoint. Navigation baselines qualify an environment and do not
count toward the 15.

## Where collection stands

43 of 75 protocol trials are collected across 5 environments.

| environment | scene | tier | model policy | model search | uniform search | complete |
| --- | --- | --- | --- | --- | --- | --- |
| `hmax2__easy_020__5f0639e3` | `hmax2/easy_020` | easy | 5/5 | 5/5 | 5/5 | yes |
| `1push__hard_021__bf3e3cdf` | `1push/hard_021` | hard | 0/0 | 5/5 | 5/5 | no |
| `2push__env__obstacle_0_movable__0489c6b2` | `v3/hard_loose/rb_00180` | hard | 1/1 | 0/0 | 0/0 | no |
| `2push__env__obstacle_0_movable__febcb94c` | `v2/zig_solo0/rb_00034` | hard | 5/5 | 5/5 | 1/5 | yes |
| `2push__env__obstacle_0_movable__fbdce248` | `v2/dense_solo0/rb_00121` | medium | 1/1 | 0/0 | 1/1 | no |

Counts read successes over trials collected, not over the five the protocol asks for.

## `hmax2__easy_020__5f0639e3`

Scene `hmax2/easy_020`, tier easy, catalog `1hop_simple`.

**model policy.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 4 | 1 | 4 | 0 | 0.4 s |
| trial2 | success | 3 | 0 | 4 | 0 | 0.3 s |
| trial3 | success | 2 | 0 | 2 | 0 | 0.2 s |
| trial4 | success | 3 | 0 | 4 | 0 | 0.3 s |
| trial5 | success | 3 | 0 | 3 | 0 | 0.3 s |

**model search.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 3 | 0 | 5 | 6 | 1.9 s |
| trial2 | success | 3 | 0 | 4 | 8 | 2.4 s |
| trial3 | success | 3 | 0 | 4 | 7 | 2.1 s |
| trial4 | success | 3 | 0 | 4 | 7 | 2.1 s |
| trial5 | success | 3 | 0 | 6 | 13 | 3.4 s |

**uniform search.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 4 | 1 | 5 | 39 | 5.8 s |
| trial2 | success | 5 | 1 | 6 | 413 | 50.1 s |
| trial3 | success | 2 | 0 | 2 | 40 | 5.0 s |
| trial4 | success | 4 | 0 | 6 | 256 | 33.4 s |
| trial5 | success | 3 | 0 | 5 | 120 | 17.8 s |

## `1push__hard_021__bf3e3cdf`

Scene `1push/hard_021`, tier hard, catalog `1hop_simple`.

**model policy.** Not collected.

**model search.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| model_r1 | success | 2 | 0 | 4 | 8 | 1.6 s |
| model_r2 | success | 2 | 0 | 2 | 12 | 2.0 s |
| model_r3 | success | 2 | 0 | 2 | 3 | 1.0 s |
| model_r4 | success | 2 | 0 | 2 | 3 | 1.0 s |
| model_r5 | success | 2 | 0 | 2 | 3 | 1.0 s |

**uniform search.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| random_r1 | success | 6 | 2 | 9 | 41 | 5.1 s |
| random_r2 | success | 2 | 0 | 2 | 6 | 1.0 s |
| random_r3 | success | 2 | 0 | 4 | 9 | 1.4 s |
| random_r4 | success | 5 | 2 | 8 | 122 | 15.5 s |
| random_r5 | success | 2 | 0 | 2 | 3 | 0.8 s |

## `2push__env__obstacle_0_movable__0489c6b2`

Scene `v3/hard_loose/rb_00180`, tier hard, catalog `1hop_simple`.

**model policy.** 1 of 1 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 4 | 0 | 4 | 0 | 0.6 s |

**model search.** Not collected.

**uniform search.** Not collected.

## `2push__env__obstacle_0_movable__febcb94c`

Scene `v2/zig_solo0/rb_00034`, tier hard, catalog `1hop_multi_int`.

**model policy.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 2 | 0 | 2 | 0 | 0.2 s |
| trial2 | success | 4 | 0 | 4 | 0 | 0.4 s |
| trial3 | success | 3 | 0 | 3 | 0 | 0.3 s |
| trial4 | success | 4 | 0 | 4 | 0 | 0.4 s |
| trial5 | success | 4 | 0 | 4 | 0 | 0.4 s |

**model search.** 5 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 3 | 0 | 5 | 104 | 34.5 s |
| trial2 | success | 7 | 2 | 11 | 156 | 37.3 s |
| trial3 | success | 4 | 0 | 9 | 116 | 30.2 s |
| trial4 | success | 4 | 0 | 8 | 115 | 30.3 s |
| trial5 | success | 4 | 0 | 6 | 56 | 19.8 s |

**uniform search.** 1 of 5 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | failure | 0 | 0 | 1 | 900 | 172.2 s |
| trial2 | success | 7 | 1 | 9 | 791 | 149.8 s |
| trial3 | failure | 0 | 0 | 1 | 900 | 177.3 s |
| trial4 | failure | 0 | 0 | 1 | 900 | 171.3 s |
| trial5 | failure | 0 | 0 | 1 | 900 | 172.2 s |

## `2push__env__obstacle_0_movable__fbdce248`

Scene `v2/dense_solo0/rb_00121`, tier medium, catalog `1hop_multi_int`.

**model policy.** 1 of 1 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 4 | 1 | 4 | 0 | 0.4 s |

**model search.** Not collected.

**uniform search.** 1 of 1 reached the goal.

| trial | outcome | pushes | stuck | plans | sims | planning |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| trial1 | success | 8 | 3 | 10 | 644 | 146.5 s |
