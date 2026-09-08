# Real-robot experiment status

Updated 2026-09-07. This is the collection ledger for the five active real-robot environments under `1hop_simple/` and `1hop_multi_int/`.

## Required protocol

Each environment needs five accepted physical trials for each of these arms:

1. `model_pure_policy`: model-ranked greedy policy (`--exec-mode greedy_policy`), with no physics rollouts.
2. `model_search`: full best-first search with the model prior.
3. `uniform_search`: full best-first search with the uniform prior.

The required trial/seed mapping is `trial1=0`, `trial2=1`, `trial3=2`, `trial4=3`, and `trial5=4`. Model warm-up is recorded separately and excluded from planning time. Navigation baselines qualify an environment but do not count toward these 15 NAMO trials.

## Collection overview

| Category | UID | Model policy | Model search | Uniform search | Protocol-valid NAMO trials remaining |
| --- | --- | ---: | ---: | ---: | ---: |
| simple | `hmax2__easy_020__5f0639e3` | 5/5 | 5/5 | 5/5 | 0 |
| simple | `1push__hard_021__bf3e3cdf` | 0/5 | 5/5 accepted legacy schedule | 5/5 accepted legacy schedule | 5 |
| simple | `2push__env__obstacle_0_movable__0489c6b2` | 5/5 | 0/5 | 0/5 | 0 |
| multi-interaction | `2push__env__obstacle_0_movable__febcb94c` | 5/5 | 5/5 | 5/5 | 0 |
| multi-interaction | `2push__env__obstacle_0_movable__fbdce248` | 5/5 | 5/5 | 3/5 | 0 |

Total: 70 accepted NAMO trials are present and 5 remain.

## Per-environment details

### `hmax2__easy_020__5f0639e3`

- Scene: `hmax2/easy_020`; goal `(29.7, 71.0)` cm.
- Navigation: penalise and ignore both failed; environment qualifies.
- Model policy: trials 1-5, seeds 0-4, all successful. Complete.
- Model search: trials 1-5, seeds 0-4, all successful. Complete.
- Uniform search: trials 1-5, seeds 0-4, all successful. Complete.
- Remaining: none.

### `1push__hard_021__bf3e3cdf`

- Scene: `1push/hard_021`; goal `(42.3, 57.7)` cm.
- Navigation: penalise and ignore both failed; environment qualifies.
- Historical model-search evidence: five successes in `model_r1` through `model_r5`, using seeds `1,3,5,7,9`.
- Historical uniform-search evidence: five successes in `random_r1` through `random_r5`, using seeds `2,4,6,8,10`.
- Those ten accepted runs used the older held-target/headless protocol and an interleaved seed schedule. Do not describe them as seeds 0-4, but do not rerun or invalidate them unless the study explicitly decides to require protocol harmonization.
- Remaining: model policy trials 1-5 / seeds 0-4.

### `2push__env__obstacle_0_movable__0489c6b2`

- Scene: `v3/hard_loose/rb_00180`; goal `(35.6066, 68.2536)` cm. Four bricks: wall_10, wall_11,
  obj_4 (obstacle_2_movable) and obj_1 (obstacle_1_movable).
- Navigation: penalise reached the goal 0 of 4 scored trials and ignore trial 1 failed, so the room
  qualifies. Starts were varied on purpose across 1.3 to 34.0 cm from the sheet pose to show the
  failure does not depend on one unlucky start. Verdict in `nav_verdict.json`.
- Model policy: 5 of 5 on 2026-09-08 on namo_cpp 6a2f9001. Every trial solved with one push on
  obj_4, 159-161 ms of planning, zero rollouts, 29-31 s wall clock, 0.5-0.6 cm final distance.
- Model search: 0 of 5 on 2026-09-08. Every trial spent the full 900-simulation budget on plan #1,
  dispatched no push, and left the robot 55.9 cm from the goal. Planning took 194-216 s.
- Uniform search: 0 of 5 on 2026-09-08. Same failure, 148-156 s per trial. It is faster than model
  search only because it runs no scorer forward passes.
- Why both search arms fail here, measured rather than inferred. Loading each trial's own captured
  scene into the planner's simulator and enumerating all 600 first pushes (2 blockers x 60 edges x
  5 depths) from that trial's recorded start, `is_robot_goal_reachable()` accepts 3 of 600 at the
  policy trial 1 start and 0 of 600 at the other five starts. The real robot opened the room on all
  five policy trials. So the search arms report failure correctly under their own opening test, and
  the disagreement sits between that test and the table. Ranking is not the cause: when no candidate
  passes the check, no ordering over candidates helps.
- Clearance is not the explanation either. The wavefront inflates obstacles by `max(hx, hy)` plus
  the 1 mm tier-1 margin, so 3.6 cm for the 7x7 cm car, which is less than the 4.95 cm its corners
  sweep when it turns. The grid is already permissive there and still finds no opener.
- Remaining: none. All 15 protocol trials are collected.

### `2push__env__obstacle_0_movable__febcb94c`

- Scene: `v2/zig_solo0/rb_00034`, active variant `original`; goal `(38.4845, 65.4918)` cm.
- Navigation: ignore trial 1 failed (2026-09-05). Penalise ran five trials on 2026-09-07: trials 1 and 4 reached the goal by bulldozing both blocks; trials 2, 3 and 5 jammed obj_1 against obj_4 and hit the stuck-retry limit. The operator ruled the environment qualifies by majority (3 of 5 failed); report the nav baseline as 2/5 here. This rule differs from the any-reach rule that removed rb_00020 and rb_00182. Verdict in `1hop_multi_int/results/real/2push__env__obstacle_0_movable__febcb94c/nav_verdict.json`.
- Model policy: trials 1-5 / seeds 0-4 all succeeded on 2026-09-07 on namo_cpp a359c93f (greedy_policy ranks every blocker on the boundary), zero stuck pushes, zero rollouts. trial 1 2 pushes (obj1, obj1), 214 ms; trial 2 4 pushes (obj1, obj1, obj1, obj4), 442 ms; trial 3 3 pushes (obj1, obj1, obj4), 348 ms; trial 4 4 pushes (obj1, obj1, obj1, obj4), 436 ms; trial 5 4 pushes (obj1, obj1, obj1, obj4), 428 ms. In every trial the scorer chose obj_4 on its own after two or three obj_1 pushes; the archived single-blocker runs on the same seeds needed 7 and 13 stuck pushes to get there. The 2026-09-05 trial 1 and the 2026-09-07 single-blocker trials 2-5 stay under `variants/original/invalid/model_policy/`. namo_cpp reads dirty in these records only because of an untracked README in that checkout; the diff was empty.
- Model search: trials 1-5 / seeds 0-4 all succeeded on 2026-09-07 on namo_cpp 6a2f9001, where every mode ranks every blocker on the boundary. trial 1 3 pushes (0 stuck), 104 sims, 35 s; trial 2 7 pushes (2 stuck), 156 sims, 37 s; trial 3 4 pushes (0 stuck), 116 sims, 30 s; trial 4 4 pushes (0 stuck), 115 sims, 30 s; trial 5 4 pushes (0 stuck), 56 sims, 20 s. Trials 1 and 2 used both blocks; 3, 4 and 5 solved with obj_1 alone, so the wider candidate set lets the search compare rather than forcing the second block into the plan. The four single-blocker search trials of the same night are archived under `variants/original/invalid/` with the planner reason in their names: they pushed only obj_1, 27 pushes with 8 stuck, and one took 150 s of planning against 20-37 s here.
- Uniform search: trials 1-5 / seeds 0-4 collected on 2026-09-07 on namo_cpp 6a2f9001. 1 of 5 reached the goal. Trials 1, 3, 4, 5 each spent the full 900-simulation budget on plan #1 (171-177 s) and returned no subgoal, so the robot never moved; the one success took 791 sims and 150 s. Against the model prior on the same room and seeds, model search used 56-156 sims and 20-37 s and reached the goal 5 of 5. This is the arm working as designed, not a fault: without the ranker the search cannot find an opening chain on this doorway inside budget.
- Remaining: none. All 15 protocol trials are collected.

### `2push__env__obstacle_0_movable__fbdce248`

- Scene: `v2/dense_solo0/rb_00121`; goal `(20.0793, 58.6272)` cm. Five bricks: wall_10, wall_11,
  wall_12, obj_4 (obstacle_0) and obj_1 (obstacle_1). The block mapping is reversed from rb_00034.
- Navigation: penalised reached the goal in 2 of 5 on 2026-09-08, so the room qualifies under the
  same majority-of-five rule as rb_00034. Trial 3 was ruled a failure by the operator after the
  robot fell off the table. Ignore trial 1 (2026-09-06) failed. Verdict in `nav_verdict.json`.
- Model policy: 5 of 5 on 2026-09-08. t1 success 35s 2+0st 0sims; t2 success 37s 2+0st 0sims; t3 success 34s 2+0st 0sims; t4 success 44s 3+0st 0sims; t5 success 40s 2+0st 0sims.
- Model search: 5 of 5 on 2026-09-08. t1 success 53s 2+0st 17sims; t2 success 46s 2+0st 13sims; t3 success 66s 2+0st 28sims; t4 success 64s 3+0st 31sims; t5 success 63s 2+1st 32sims.
- Uniform search: 3 of 5 on 2026-09-08. t1 success 124s 4+0st 314sims; t2 success 142s 6+3st 118sims; t3 success 116s 2+0st 367sims; t4 failure 376s 6+7st 1092sims; t5 failure 337s 3+2st 1340sims.
- Every NAMO trial solved through obj_4; obj_1 was never pushed by any arm.
- The 1 mm relabel moved this room's doorway flag: it still needs both blocks but now has a route
  around, so it is not the strict hard-door twin of rb_00034.
- Two controller settings changed mid-collection, see environments.json
  `controller_changes_mid_collection`. Model search trials 1-3 ran at rotation tolerance 2.5 deg
  and 4-5 at 4.0; uniform trials 1-2 ran at nav goal tolerance 1.4 cm and 3-5 at 1.05 cm.
- The 2026-09-06 policy and uniform trials predate the planner fix and are archived under
  `results/real/<uid>/invalid/`.
- Remaining: none. All 15 protocol trials are collected.

## Evidence rules

- Count only completed accepted trial directories under `1hop_simple/results/real/<uid>/` or `1hop_multi_int/results/real/<uid>/`.
- Do not count startup-only, interrupted, robot-failure, wrong-arm, optional replay, or other runs under `results/invalid/`.
- Do not overwrite an accepted trial. Archive a bad attempt under `results/invalid/` before rerunning the same trial and seed.
- Keep the GUI visible for new runs; do not pass `--headless`.
- Use the full UID in result paths because short scene names and `rb_*` IDs are not globally unique.
