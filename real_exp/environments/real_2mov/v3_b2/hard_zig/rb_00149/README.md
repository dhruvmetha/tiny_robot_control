# v3_b2/hard_zig/rb_00149, staged for the table

Target path on the robot machine:
`/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/v3_b2/hard_zig/rb_00149/`

Copy `v3_b2/hard_zig/rb_00149/env.xml` there. Everything else in this directory is provenance.

## What the label means

The card says **2push / medium** for `obstacle_1_movable`. Read it as follows.

**Solver.** `scripts/pipeline/exhaustive_hmax2.py`, not a search. It enumerates every reachable (object, edge, depth) at the start state, executes each one, and for any push that does not open the goal region it then tries EVERY reachable push from the resulting state until one opens or all fail. No frontier, no beam, no budget. So a cell marked `dead` means every follow-up was actually simulated, not that something ran out. 9571 simulated pushes went into this one scene.

**Success bar.** At least 20 of the 100 poses sampled inside the goal region become reachable, and were not before. Sampled once at the root, seed 42. This is the region test, NOT the XML goal marker; `is_robot_goal_reachable()` disagrees with it and is not used. `goal_open_at_start` is false here, so the goal region genuinely starts shut.

**2push** means no single push on `obstacle_1_movable` opens the region (`solve_rate_1push` is 0.0) but 3 first pushes have a follow-up that does. **medium** is the difficulty tier, from 3 working setups out of 60 reachable pushes, a 5.0% solve rate, against the standing cuts of hard below 5%, medium below 30%, easy at or above. It sits right on the hard/medium boundary.

The three working first pushes, as (edge, depth): (0, 4), (2, 3), (2, 4).

## The recorded solution

Both pushes, re-simulated from a clean start:

1. `obstacle_1_movable`, edge 0, depth 4. Does not open the region.
2. `obstacle_0_movable`, edge 17, depth 3. Opens it.

Note the second push lands on the OTHER block. That is normal in this pool and it is why the scene qualifies: all 3 of its working setups make the two movables collide (`contact_pct` 100).

## Two caveats worth having before the run

**Edge indices are not comparable across the boundary without canonicalising yaw.** Your runtime plans on a camera capture, not on this file. If a block is placed 180 degrees from the sheet, the tag reads 180 off, the box occupies the same space, and both sides are correct, but `contact_points` emits `(u, +hy)` then `(u, -hy)` alternating, so a 180 flip swaps face parity. To map your logged index onto ours, canonicalise block yaw mod 180 and flip index parity (`i XOR 1`) within each 30-block when the yaws differ by ~180. Execution on the table is self-consistent either way.

**This scene is not one of the two-block-doorway cases.** `door_needs_both_blocks` is false and `has_route_around` is true, so it is an ordinary two-movable scene that happens to need a setup push. If you wanted one of the 240 hard ones where the only route needs both blocks moved, say so and I will stage one of those instead.

## Files

- `v3_b2/hard_zig/rb_00149/env.xml` — the scene. This is all the pool ships, there are no sidecars.
- `build_sheet_derived.json` — poses read back out of the XML, plus the robot start read from the simulator after reset since the XML carries no robot body. Marked `derived_from_env_xml` so it is not mistaken for a generator sheet. The v3_b2 pool has no build sheet and no tier CSV; this pool was never part of a handoff campaign.
- `sweep_record_hard_zig__rb_00149.json` — the raw label. Every enumerated cell with its kind, plus `n_goal_points`, `bar`, `goal_open_at_start`, `n_sims`.
- `gallery_card.json` — what the scene-card gallery shows.
- `gallery_replay.json` — the two-push solution with per-step geometry and region decomposition.
