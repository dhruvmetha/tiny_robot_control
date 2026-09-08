# One-hop simple real-robot environments

Canonical catalog for one-hop scenes without a required coupled interaction
between multiple movable blockers. A scene may contain one or two movable
objects; `simple` describes the interaction structure, not the raw object
count.

| UID | Scene | Task | Navigation | NAMO results |
| --- | --- | --- | --- | --- |
| `hmax2__easy_020__5f0639e3` | `hmax2/easy_020` | easy, two-push horizon, one movable | both variants failed; qualifies | model policy 5/5; model search 5/5; uniform search 5/5 |
| `1push__hard_021__bf3e3cdf` | `1push/hard_021` | hard, one-push horizon, one movable | both variants failed; qualifies | model search 5/5; uniform search 5/5 |
| `2push__env__obstacle_0_movable__0489c6b2` | `v3/hard_loose/rb_00180` | hard, two-push horizon, two movables; no required coupled interaction | complete 2026-09-08: policy 5/5, model search 0/5, uniform 0/5; nav qualifies (0/4 reached). Sim accepts 0 of 600 first pushes from 5 of 6 trial starts while the robot opens the room every time | none |

Use the full UID as the environment and result-directory identity. Build IDs
such as `easy_020`, `hard_021`, and `rb_00180` are not globally unique.

## Layout

- `environments.json` is the authoritative catalog and collection status.
- `cards/<uid>.json` preserves each shortlist/gallery selection.
- `environments/<uid>/` contains the physical reset sheet and available source
  scene files.
- `results/real/<uid>/` is the canonical location for accepted real trials.
- `results/invalid/<uid>/` preserves invalid or startup-only attempts.

The two historical `real_exp/results/formal_v2/...` paths are compatibility
symlinks into this folder so existing paper analysis continues to resolve. Raw
historical `config.json` and `run.log` files retain the path used when they were
recorded; they are evidence and are not rewritten during migration.
