# Ranking-Guided Region Opening — Design

Status: design spec. Describes required behavior and implementation work.
It does not claim the current code implements the design.

Audited 2026-08-18 at:

| Component | Branch | Head |
|---|---|---|
| `robot_control` | `real-robot` | `582f1d0` (+ uncommitted `closed_loop_sessions/README.md`, dirty `rvg`) |
| `namo_cpp` | `feat/horizon-q-redesign` | `dafe42f` (rebased + pushed 2026-08-18) |
| `sage_learning` | `feat/horizon-q` | `db75913` |
| ranker bundle | — | `/home/dhruv/projects_dhruv/namo/ranking/` (BNG s1–s3 + θ₀) |

## Objective

Make the learned-ranker best-first search the local search that `robot_control`
runs on the physical robot, replacing the legacy breadth-first region-opening
search, and correct the defects that currently make that switch unsafe.

The motivating result is not capability. It is cost:

- At simulation budget 4000, model and uniform-random ranking **converge**
  (2-push solve rate 100.0 vs 97.2). Registry: *"the contribution is cost,
  not capability."*
- Under the wall-clock protocol, the model's hard 2-push median instance is
  **10.9× faster**, and slower on 5.9% of instances.

For offline evaluation, spending fewer simulator calls is a convenience. For a
robot that must replan between physical pushes, it is the difference between a
usable closed loop and an unusable one.

## Audience

- Developers wiring `robot_control` to the `namo_cpp` planning service.
- Anyone deciding which checkpoint and which search protocol to deploy.
- Anyone reading a real-robot run log and needing to know which search
  produced a push.

## Background — what the ranker is

A 4.4 M-parameter `EdgeCrossAttn` network. Not a dynamics model, not a
classifier of success. **A ranker.** It orders candidate pushes; the simulator
verifies every one of them. What matters is the order, not calibration.

### Input

Two tensors, both derived from live simulator state.

1. **Context** — `(5, 64, 64) float32`. A top-down binary raster of a **0.5 m
   square crop centered on the object being pushed**, i.e. 7.8 mm/px. Channels,
   in order (`live_scorer.py:47-50`):

   | # | channel | encodes |
   |---|---|---|
   | 0 | `static` | walls and immovable geometry |
   | 1 | `movable` | all movable objects at current pose |
   | 2 | `target_object` | just the object being pushed |
   | 3 | `robot_region` | free-space component containing the robot |
   | 4 | `goal_sample_region` | free-space component containing the goal |

   Channels 3 and 4 are what make this a *region* model rather than a geometry
   model. They come from the wavefront exporter, so the inflation matches what
   the planner itself saw. There is no mean/std normalization — values are
   area-fractions in [0,1] produced by two `INTER_AREA` downsamples.

   The model does **not** see the robot, goal, or goal-sample circles that the
   renderer also draws.

2. **Contact points** — 60 pixel coordinates in the same crop frame, pure
   geometry, no learning (`eval_scorer.py:45-56`). **60 = 4 object faces × 15
   sample points per face**, interleaved so even/odd indices are opposite
   faces. This is the same indexing `robot_control/controller/edge_points.py`
   mirrors.

### Output

A `(60, 5)` grid: **60 contact points × 5 push depths = 300 candidate pushes
for one object at one state.**

Each cell is produced as a 51-bin HL-Gauss distribution and reduced to a scalar
by **bin expectation** — `softmax(logits) · bin_centers`, not argmax. The
bundle README is explicit that argmax produces "plausible-looking garbage".

The scalar's meaning comes from the training labels, whose alphabet on trained
cells is `{0, 0.9, 1}` — γ^k with γ = 0.9:

| value | meaning |
|---|---|
| 1 | this push opens the region now (opener / finisher) |
| 0.9 | valid setup push, one step from an opening |
| 0 | searched to budget, no solution found through it |
| −1 | geometrically unreachable (stored, masked out of loss) |
| MASK | reachable but never tried — **no gradient** |

The MASK rule is load-bearing. Forcing untried cells to 0 is the recorded
"C15 poison": false negatives suppress valid pushes.

### Two score scales

- **raw** = E[bin] ∈ [0,1]. The honest scale, default in the search.
- **sigmoid-on-top** = `1/(1+e^-E[bin])` ∈ [0.5, 0.731]. Monotone, so rankings
  are identical, but magnitudes are compressed.

`ScorerGoalStrategy` still uses the sigmoid path (`raw=False`). This is inert
under `combine=q` and **not** inert under `blend`, `product`, or
`--discount conf`.

### Cost

Warm, CPU, with `fast_scorer` enabled:

| step | time |
|---|---|
| render context | ~101 ms (was 2019 ms before the sage render-speedup merge) |
| network forward | ~36 ms |
| **one `score_state` call** | **~140 ms** |
| one simulator push | ~160 ms |

Scoring is no longer the bottleneck. Two properties matter for deployment
planning: **batch size is 1**, and one call covers **one object**. An
N-reachable-object state costs N × 140 ms. Batching the forward across objects
is an untaken win, though the render is ~3/4 of the cost.

## Background — how the search works

`scripts/sandbox/eval_bestfirst.py::solve_scene`. It is **a single global
priority queue over unsimulated pushes**, not a tree search.

A queue item is one candidate action `(object, edge, depth)` plus the state it
starts from. Nothing is simulated until popped.

```
while heap and sims < sim_budget:
    pop highest-priority candidate
    lazy-reinsert if its board weight decreased      # no sim
    skip if jam-pruned                               # no sim
    env.set_full_state(item.from)
    env.step(action);  sims += 1                     # the ONLY budget cost
    if is_open(env): return                          # free read
    if nothing moved: continue                       # no expansion
    if ndone < hmax:
        score the reached state, push all its children onto the SAME heap
```

Three design properties are the whole point:

1. **Cross-depth comparison.** No depth layering, no path-cost term. A depth-1
   child with high score outranks a fresh depth-0 root with lower score,
   because the objective is minimize *simulations*, not minimize plan length.

2. **The remaining budget is an input to the model.** `h = hmax - ndone` is fed
   to the forward pass, so a candidate is scored as "does this open the region
   within the pushes I have left." That is what makes scores comparable across
   depths.

3. **Nothing is pruned by score.** No beam, no top-k. Every candidate goes on
   the heap and the queue does selection globally. Only two structural prunes
   exist, both argued lossless:
   - **no-op dedup** — if neither object nor robot moved (1 µm tol), the child
     board would be a byte-identical duplicate. 27.4% of all sims, zero solves
     ever came from such a child.
   - **jam-depth pruning** — `push_steps = depth+1` is one continuous push, so
     if depth *k* jams, every deeper *k'* hits the same obstruction. Upward
     only. Validated 1214/1215 pairs.

Cost accounting: **exactly one simulation per surviving pop**, and **zero or
one scorer calls** (a leaf at `hmax` scores nothing). `sim_budget` is the single
reactive↔search dial — tiny budget degenerates to a reactive policy, large
budget is real search.

**Opening test.** `is_open(env)` is evaluated on the post-push state and costs
no simulation. The canonical bar is **≥20% of 100 goal-region points sampled
once at s0**. Sampling once is deliberate: after a push opens the path, the
goal region merges with the robot region and gets relabeled, so resampling
post-push samples the wrong region (93% vs 20% label agreement).

**Return.** `(solved, sims, plan_len, boards, end)` with
`end ∈ {solved, solved_mined, budget, exhausted}`. The winning plan and the
post-push simulator state are captured **only if the caller passes
`solution_out`**.

## Background — how it is wired today

```
plan_from_xml(**kwargs)
  → algorithm_params.update(kwargs)              # planning_service.py:283
  → PlannerFactory.create_planner("full_namo")
  → FullNAMOPlanner._make_region_opener()        # full_namo_planner.py:112-115
      full_namo_local_search == "best_first"
  → BestFirstRegionOpeningPlanner                # not registered, not exported
  → scripts/sandbox/eval_bestfirst.solve_scene   # sys.path insert at runtime
```

The outer loop (`FullNAMOPlanner.search`) rebuilds the region connectivity
graph **every iteration**, finds a path to the goal region, and takes only
`path[1]`. That single boundary is a **keyhole**. The outer loop hands the
opener a region *label*; the opener re-derives the blocking objects itself and
passes them to the search as `restrict_obj`.

`BestFirstRegionOpeningPlanner` is reachable **only** through
`algorithm="full_namo"` plus `full_namo_local_search="best_first"`. It is not
in `PlannerFactory` and not exported from its package `__init__`.

## Confirmed defects

These are verified against the audited heads. Each must be resolved or
explicitly waived before real-robot deployment.

### D1 — `external_edge_blacklist` is silently ignored (blocking) — RESOLVED

Fixed in namo_cpp `d67af7c`. Filtered at enumeration by wrapping the goal
enumerator, leaving `eval_bestfirst.solve_scene` untouched so the evaluated
protocol is unchanged. Covered by `test_best_first_edge_blacklist.py`.

`region_bfs` honors it at candidate-generation time
(`region_opening.py:443-449`). `robot_control` injects it in simulator naming
(`namo_bridge.py:665-674`) precisely so a push that failed on the physical
robot is not proposed again.

`BestFirstRegionOpeningPlanner` never reads it. Under `best_first`, the robot
will re-propose pushes it has already physically failed, and the replan loop
can cycle.

This is the single most dangerous defect for real-robot use.

### D2 — the two arms are graded on different opening bars (blocking) — RESOLVED

Fixed in namo_cpp `9f58f24`. The fraction is declared once as
`CANONICAL_MIN_REACHABLE_FRACTION` and read by both openers, so the two
cannot drift again. Covered by `test_opening_bar_parity.py`.

| arm | `region_min_reachable_fraction` | effective bar |
|---|---|---|
| `region_bfs` | 0.2 | ≥20 of 100 points |
| `best_first` | **0.0** (`best_first_region_opening.py:58`) | ≥1 of 100 points |

Commit `b572570` canonicalized the 20% bar and did not touch the best-first
file. `solvability_runner.build_full_namo_planner_config` never sets the key.
Any A/B between the arms is currently invalid, and the robot would accept a
boundary as "open" on a single reachable point.

### D3 — service defaults silently produce a different search than every eval — RESOLVED

Fixed in namo_cpp `66daea0`. `best_first_hmax` no longer inherits
`region_max_chain_depth`; both it and the keyhole budget are named with their
registry provenance. `budget_scope` deliberately left at `full_problem` —
changing it would alter region_bfs runs too. Covered by
`test_best_first_protocol_defaults.py`.

| parameter | value via `plan_from_xml` | value in every registered eval |
|---|---|---|
| `best_first_hmax` | **1** (falls back to `region_max_chain_depth`, default 1) | 2 |
| simulation budget | **100** (`full_namo_keyhole_simulation_budget` default) | 300–900 **per keyhole** |
| `budget_scope` | `full_problem` — the 100 drains across *all* keyholes | per-keyhole reset |
| `goals_per_region` | 10 | 100 (`PlannerConfig` canonical) |

A caller who passes only `full_namo_local_search="best_first"` gets a depth-1
search with a 100-simulation whole-scene budget graded against a 10-point
region sample. None of the published numbers apply to it.

### D4 — no CLI surface in `robot_control` — RESOLVED

Fixed in robot_control `65687d5`. `--local-search`, `--best-first-prior`,
`--scorer-ckpt`, `--best-first-hmax`, `--keyhole-simulation-budget`, mapped in
one place (`planner/search_config.py`) and forwarded from both the runtime
planner and plan-only mode. Defaults forward nothing, so unchanged runs are
unaffected. hmax and budget are omitted unless set so namo_cpp keeps ownership
of the canonical protocol. Covered by `tests/test_local_search_config.py`.

`scripts/run_namo.py` populates `extra_kwargs` with only
`rollout_samples_per_state`, `shuffle_seed`, `region_success_min_reachable`,
and the `ml_*` keys. There is no flag for `full_namo_local_search`,
`scorer_ckpt`, `best_first_hmax`, or the budget keys.

The kwargs road exists end to end; nothing drives on it.

### D5 — checkpoint generation skew

The `ranking/` bundle pins **BNG** (best 2p-hard@5 32.1) and records
`SAGE_COMMIT = 6f90dc6` on branch `exp/bootstrap-value-loop`. The bundle README
warns that this commit is *not* on `feat/horizon-q` — the currently checked-out
sage branch.

Since then the registry has named **HY5U** *"best deployable model to date"*
(complete 2026-08-12, wins every column on the common episode set, zero
train/test leakage), and the production planner is named for it
(`19e9462 full-namo: add best-first HY5U keyhole search`).

`robot_control`'s local handoff doc still pins BNG. It is a generation behind.

Related traps recorded in the bundle README, all of which must survive into any
deployment tooling:

- **`val_loss` does not rank these models.** BNG 1.87 vs θ₀ 1.70 — BNG looks
  worse and deploys ~10 points better. Different label sets.
- **s1/s2/s3 are independent seeds, not an ensemble.** Deployment must select
  one explicitly. Do not average weights.
- **The architecture is not in the checkpoint.** `load_scorer` re-derives it by
  sniffing `state_dict` key shapes. A sage checkout whose `EdgeCrossAttn`
  signature has drifted fails as a wrong-shaped load or, worse, loads and
  scores quietly wrong.

### D6 — `scripts/sandbox` is a runtime dependency of production code

Two live edges: `best_first_region_opening.py:24-32` imports `solve_scene` and
`make_action`; `scorer_goal_strategy.py:27` imports `LiveScorer`. The
transitive module-level import graph pulls in `scorer_beam`, `eval_m3`,
`live_scorer`, `scripts/eval_scorer.py`, `scripts/viz/trace_schema.py`, torch
and cv2.

Failure mode differs by prior: with `prior="model"` the import happens at
**planner construction** (scorer load), so it surfaces as a per-scene error
row; with `prior="uniform"` it survives construction and dies on the first
non-trivial `search()`.

### D7 — region conditioning may not reach the model — RETRACTED, NOT A DEFECT

This was asserted from reading `wavefront_snapshot.py:488-496` ("Always use XML
goal for consistency across snapshots") and inferring that `region_samples`
could not reach the `goal_sample_region` channel. **Tested, and it is wrong.**

On a scene with two genuine regions, rendering with samples drawn from one
region versus the other produces completely different channel 4:

```
A = samples in region_3      -> goal_sample_region sum = 3424.9
B = samples in robot's own   -> goal_sample_region sum =    3.2
max |A - B| = 1.0
```

Region conditioning reaches the model. No fix required, and no test is owed.

Two earlier readings during that investigation were inconclusive rather than
confirmatory, and should not be cited: a single-region scene (where any sample
set labels the same region, so identical output proves nothing) and a scene
where both region channels came back empty for an unrelated reason.

**One incidental finding worth keeping.** For an object far from the robot,
*both* `robot_region` and `goal_sample_region` came back entirely blank inside
the 0.5 m tight crop — the ranker scores that push on geometry alone, with no
region signal at all. How often that happens across a real corpus is unknown.
It is a measurement question, not a defect, and worth a sweep before drawing
conclusions about multi-hop performance.

### D8 — the winning result is thinner than the search knows

`BestFirstRegionOpeningPlanner` destructures `solve_scene`'s return and
discards `boards` — the per-board lifetime records (`tries` = (try#, q, opened)
triples, `k_failed`, `w`, candidate pools, model grids). It also never requests
`timing`, so `t_sim` / `t_score` / `t_wall` are unavailable, and never sets
`trace_out`.

For offline eval this is fine. For real-robot debugging — "why did it pick
that push" — it is the entire evidence trail, and it is thrown away.

### D9 — no test exercises the class

`test_full_namo_budget_and_config.py` pins config plumbing only. The
`already_accessible` regression tests drive a hand-rolled fake opener, not the
real class. Untested: constructor param parsing, `_boundary_objects`,
`_minimum_needed` (and therefore D2), `PlannerResult` population, the
`finally: set_full_state(baseline)` contract, budget accounting, and the
sandbox import.

### D10 — `robot_control` replans at the wrong level — RESOLVED

Both ways of running the robot can now hold one region boundary until it opens
or is exhausted, instead of re-deriving the whole problem every replan.

**namo_cpp** — `select_boundary_from_xml` (`7c42575`) chooses the next boundary
and samples its points once, reusing `full_namo`'s own `path[1]` rule via a
shared `find_region_path`. `solve_boundary_from_xml` + `region_target_points`
(Phase 1) then grade every later call against those frozen points.

**robot_control** — `RegionOpeningTarget` (`5d45df4`) is the durable record:
points as identity rather than a region label (labels are ordinal and renumber
after a push), and blockers in real naming rather than simulator naming
(simulator ids are a rank over the movables in one observation). `NAMOPlanner`
holds one (`60ca1be`); `run_namo --active-target` honours it on both entry
points (`a2c7545`); `closed_loop_session` hands it to the replan subprocess
(`7f89a08`), at run level because iteration creation rewrites `status.json`
and regenerates `scene_after/` wholesale.

The select/solve/release step is `advance_boundary`, called by both paths —
they share no other code, and a second copy would drift the way the two
chain-reuse ladders already have.

Also fixed on the way: the failed-push blacklist was cleared on *every*
successful push, including for objects that had not moved (`564d96a`). The
stated reason for clearing was right for the pushed object — `edge_idx` is in
its body frame — and wrong for everything else.

**Off by default.** `hold_region_target=False`, and the session path opts in via
`run_meta.json`. A run that does not ask for a held boundary is unchanged.

**Known gap.** Chain reuse is skipped while a target is held, because
`verify_chain` asks whether the remaining chain makes the *final goal*
reachable — a chain can satisfy that while abandoning the boundary being
opened. A target-conditioned reuse check would restore the optimisation; there
are two copies of that ladder and both would need it.

### R1 — persist the active region-opening target

Introduce a serializable target that survives across physical pushes and across
processes (the closed-loop session runs each replan as a separate command).

Identity is the **sampled points**, not the region label — labels change when
the scene changes. Sample once at subproblem start; never resample after setup
pushes.

Fields at minimum: target id, target sample points (metres), blocker identity
in both real and simulator naming, the iteration that selected it, the source
region path, sample seed, and the open fraction that defines success.

### R2 — one canonical opening bar

`region_min_reachable_fraction` defaults to 0.2 in both arms. The value is
named once and read by both `region_bfs` and `best_first`. Resolves D2.

### R3 — honor the blacklist in best-first

`external_edge_blacklist` filters candidates at enumeration time under
`best_first`, in the same simulator naming the bridge already produces.
Resolves D1.

Enumeration is the right layer: filtering after the fact would still charge the
simulation.

### R4 — explicit deployment parameters, no silent fallbacks

`best_first_hmax` and the simulation budget are required inputs on the
deployment path, not fallbacks off unrelated keys. A missing value is an error,
not a default. Budget scope is per-keyhole. Resolves D3.

### R5 — a `robot_control` CLI surface

`run_namo.py` gains flags for local search selection, checkpoint path, hmax,
per-keyhole budget, and device, forwarded through the existing `extra_kwargs`
path. Resolves D4.

The startup banner must print the resolved search identity — checkpoint
basename, prior, hmax, budget, opening bar — because a run log that does not
record which search produced a push is not diagnosable after the fact.

### R6 — retain the search evidence trail

The planner requests `timing` and retains enough of `boards` to answer "why
this push": at minimum the winning candidate's score, its rank within its
board's pool, the number of candidates tried before it, and the per-episode
`t_sim` / `t_score` / `n_score`. Written to the run artifacts alongside the
existing `pushes.jsonl` / `subgoals.jsonl`. Resolves D8.

### R7 — pin the model bundle

Add checksums and an architecture manifest next to the existing commit SHAs, so
`load_scorer`'s key-sniffing is not the only thing reconstructing the network.
Record which seed is deployed. Resolves the reproducibility half of D5.

### R8 — promote the sandbox dependencies

`solve_scene`, `LiveScorer`, and the enumeration helpers move into the
production package with a stable contract. Resolves D6, and is a precondition
for D9 being fixable.

## Verification

Layered, per the workspace standard. Each layer must show output, not a claim.

**Unit**

- Opening bar: a state with exactly 19 and exactly 20 of 100 points reachable
  grades as closed and open respectively, identically under both arms. Fails
  on current code (D2).
- Blacklist: a blacklisted `(object, edge)` never appears in the candidate pool
  under `best_first`. Fails on current code (D1).
- Parameter resolution: a missing hmax or budget raises rather than defaulting.
- Score reduction: bin expectation, not argmax, on a synthetic 51-bin logit
  vector.
- Target persistence: round-trip serialize/deserialize preserves sample points
  bit-exactly.

**Integration**

- `plan_from_xml` with `full_namo_local_search="best_first"` on a captured
  `real_test_envs` scene returns the same plan as `solvability_runner` driving
  the same checkpoint and protocol directly. This is the test that would have
  caught D3.
- Region conditioning: rendering with two different `region_samples` sets on
  one scene produces two different channel-4 rasters. This is the test that
  settles D7. **Expected to fail on current code.**
- Deleting `scripts/sandbox` from the import path produces a clear error at a
  defined point, not a silent wrong-scores path.

**End-to-end (simulation)**

- The nine `real_test_envs` scenes, both arms, same checkpoint, same budget and
  hmax, canonical opening bar. Report solve rate and simulations-to-solve per
  scene. The ranking arm must not be worse on solve rate and should show the
  cost reduction the registry predicts.
- Blacklist loop: force a push to fail, confirm the next plan does not
  re-propose it, and confirm the active target survives the replan.
- Multi-push subproblem: a scene needing a setup push, confirming the target
  region and blocker are unchanged across the two physical pushes.

**Protocol note.** Any number compared against the registry must match its
recorded protocol exactly — checkpoint, hmax, budget, `combine`, discount
policy, no-op dedupe, jam pruning. A mismatch is a different experiment, not a
cached control. Absolute simulation counts from before `5daaed5` are not
comparable to anything measured now.

## Non-goals

- Replacing `region_bfs`. Both arms remain selectable; `region_bfs` stays the
  default until the ranking arm passes the verification above.
- Retraining, or any change to the ranker architecture or labels.
- Batching the scorer forward across objects. Real win, separate change; the
  render dominates.
- The diffusion goal-inference path. It is broken independently — the service
  passes `namo_config_path` to a `GoalInferenceModel` signature that only
  exists on sage's `real-robot` branch — and it is not on the ranking path.
- Multi-robot, and any change to the physical push controller.

## References

- `namo_cpp/docs/experiments/horizon_q_model_registry.md` — canonical protocols
  and per-model results. The only place ranking numbers come from.
- `namo_cpp/docs/experiments/exit_loop_algorithm.md` — label alphabet and the
  collection loop.
- `ranking/README.md` — model card, deployment traps, pinned commits.
- `robot_control/MODEL_GUIDED_RO_INTEGRATION.local.md` — the untracked handoff
  this spec supersedes on the search side; its nested-loop control contract
  still stands.
