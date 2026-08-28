# Pure-Policy Formal-v2 Experiment Design

## Context

The existing `real_exp/results` tree mixes pilots, formal trials, simulated
replays, and generated labeling output collected under several planner
protocols. In particular, the recorded `hard_004/model_greedy_policy/trial1`
used the former simulation-filtered policy: each camera decision simulated
ranked candidates until one moved. The current backend at `1628d1f` defines a
pure policy instead: rank the current candidates, return the arg-max without a
physics rollout, execute it physically, observe the camera, and decide again.

The next collection block compares full simulated search with this pure policy.
Old records must not be pooled with the new block.

## Approved destructive reset

Delete every file and directory below `real_exp/results/` permanently. This
includes formal runs, pilots, replays, labeling output, JSON/JSONL telemetry,
scene captures, and videos. Do not archive or move these records elsewhere.

Preserve everything outside that boundary, including:

- `real_exp/environments/`
- `real_exp/shortlist/` and `real_exp/shortlist.json`
- `real_exp/resolved_scenes.csv`
- `real_exp/README.md` and `real_exp/METRICS.md`

Recreate `real_exp/results/` as an empty tracked directory after deletion.

## Formal-v2 result layout

New records live below `real_exp/results/formal_v2/`. Method directories state
the experimental arm rather than merely repeating an internal CLI token:

```text
real_exp/results/formal_v2/<axis>/<scene>/model_search/trialN/
real_exp/results/formal_v2/<axis>/<scene>/model_pure_policy/trialN/
```

The five-trial seed mapping remains frozen: `trial1` through `trial5` use seeds
0 through 4 respectively. The old `formal_v1` and `model_greedy_policy` result
paths must not be reused.

## Arm semantics

`model_search` runs Full NAMO search. It uses physics rollouts to construct a
complete simulated plan before physical execution. Its simulator-transition
count and planning wall time include those rollouts according to
`real_exp/METRICS.md`.

`model_pure_policy` invokes the existing CLI mode `--exec-mode greedy_policy`.
At every live camera state it:

1. rebuilds the current graph;
2. generates and model-ranks candidate pushes;
3. returns the arg-max without calling `env.step`;
4. executes that one push physically;
5. observes the camera and repeats.

The pure-policy arm therefore records zero physics-rollout simulations by
definition. Its measured planning wall time still includes graph construction,
candidate generation, model-input rendering, and inference. Physical failures
feed the existing external edge blacklist. Trial success still requires final
camera-confirmed navigation to the goal.

The model scorer must continue refusing the legacy-BFS render fallback. A
fallback is a planning failure, not permission to score masks produced with a
potentially wrong robot size.

## Frozen backend

The pure-policy collection block uses `namo_cpp` revision
`1628d1ff195047246315aa81a7808ba5300bc379`. Do not collect formal-v2 rows after
changing that repository without deliberately starting a new protocol version.

The robot-control revision is the committed revision containing this protocol,
the provenance recorder, and its documentation. Every run records the exact
runtime revisions, so no self-referential SHA is stored in this design file.

## Paired repository provenance

Diagnostics must snapshot repository state before creating the run directory.
Otherwise the new output itself makes a clean repository appear dirty.

Each new `config.json` retains the existing top-level `git` object for
compatibility and adds a `repositories` object containing separate
`robot_control` and `namo_cpp` snapshots. Each snapshot records commit, branch,
and dirtiness. The backend path is resolved from the existing `NAMO_REPO`
environment variable; no workstation path is hardcoded in source.

If backend provenance cannot be resolved, diagnostics records an explicit
unavailable value and warning rather than inventing a revision. The formal-v2
runbook requires a matching backend SHA before launch.

## Documentation

Update `real_exp/README.md` to:

- define the full-search and pure-policy arms;
- use `formal_v2` and `model_pure_policy` output paths;
- pin backend revision `1628d1f` for the collection block;
- retain automatic model warmup before measured planning; and
- require paired repository provenance in the preflight check.

Update `real_exp/METRICS.md` to state that pure policy performs zero physics
rollouts while its planning wall time still covers ranking and inference.
Remove descriptions of the deleted legacy trials.

## Minimal verification

Add one integration-level diagnostics test that creates clean temporary Git
repositories for robot control and NAMO, places the diagnostics output inside
the robot repository, and verifies that the resulting `config.json`:

- records both exact commits;
- records both repositories as clean; and
- proves provenance was captured before the output directory was created.

Do not add tests for Markdown wording or deleted data. Run the existing focused
diagnostics and policy-routing tests, followed by the repository's normal test
suite before pushing.

## Non-goals

- Do not alter Full NAMO search or pure-policy planner behavior.
- Do not rename the internal `greedy_policy` CLI token.
- Do not modify camera, robot motion, model weights, or scene definitions.
- Do not preserve any record currently under `real_exp/results/`.
