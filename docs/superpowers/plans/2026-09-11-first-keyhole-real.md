# First-keyhole real execution implementation plan

**Goal:** Return Full NAMO's first verified opening for physical execution, then continue from observed reality, under one trial simulation allowance.

**Architecture:** Keep Full NAMO's existing route selection and local search. Return a serializable opening criterion with its local push chain; use that criterion for real chain verification and any continuation search. The real planner owns a shared `PushAttemptBudget` that is passed to both fresh search and exact-chain verification.

**Engineering Standards:** Apply writing-plans and plan-coding-standards: focused changes, existing naming and reusable budget/verification machinery, behavioral tests before implementation, contextual logs, and one coherent commit per repository. The user approved implementation in separate worktrees; neither live checkout is an implementation or deployment target. No physical robot run is part of validation.

## Stage 1: NAMO search horizon

- [x] Create paired ignored worktrees on `feat/first-keyhole-real-20260911` from the deployed repository heads; preserve live uncommitted changes.
- [x] Run existing Full NAMO, service, and push-budget tests (34 passed) and robot planning/routing/verification tests (95 passed).
- [x] Add and run `python/tests/test_first_keyhole_horizon.py`: default completes two openings; first-keyhole returns the entire first chain without searching the next; preserve frozen points and blocker identity; distinguish local readiness from final-goal reachability.
- [ ] Add `full_namo_planning_horizon` (`full_goal` default, `first_keyhole` opt-in) and `full_namo_active_keyhole` continuation metadata in `python/namo/planners/full_namo/full_namo_planner.py`.
- [ ] Share region, final-goal, and goal-clearance predicates through `python/namo/planners/full_namo/keyhole_target.py`; preserve the opener's actual samples and threshold. Restrict continuation to the selected blockers and frozen criterion even if labels change.
- [ ] Run `python -m pytest python/tests/test_first_keyhole_horizon.py python/tests/test_full_namo_strict_bfs.py python/tests/test_full_namo_budget_and_config.py python/tests/test_full_namo_greedy_dfs.py python/tests/test_full_namo_goal_clearance.py python/tests/test_planning_service.py python/tests/test_push_budget.py -q`.
- [ ] Commit: `feat(planner): return the first verified keyhole for real execution`.

## Stage 2: Real runtime handoff

- [ ] Add behavioral tests for local suffix verification, an opening already complete in the new observation, continuation from observed poses, exhausted budgets, and final-goal success remaining distinct from keyhole readiness.
- [ ] Add `planning_horizon` to `LocalSearchConfig`, expose `--planning-horizon`, and read the YAML `namo.planning_horizon` default before diagnostics capture. Reject incompatible held, policy, non-best-first, and open-loop combinations before runtime starts.
- [ ] In `NAMOPlanBridge`, translate target object identities on every scene conversion, pass the target through `plan_from_xml`, and verify exact chains against that target. Charge each verification `env.step` to the injected allowance; zero remaining permits observation-only checks, never another simulated push.
- [ ] In `NAMOPlanner`, retain the active keyhole until the observed state satisfies its criterion. Clear a stale chain before selecting the next keyhole. Own one lazy canonical 900-call budget in first-keyhole mode and pass the same object through all searches, retries, and verification; `reset()` begins a new trial.
- [ ] Record per-call simulation deltas, cumulative trial usage/remaining, horizon, local outcome, and target metadata. Preserve the existing default full-goal path.
- [ ] Document the option and paired-worktree launch environment in `real_exp/README.md`; environment paths stay in shell environment configuration, not source code.
- [ ] Run the repository-owned `python -m pytest tests -q` suite and CLI/config checks, then commit: `feat(real): execute one simulated keyhole at a time with a shared trial budget`.

## Stage 3: Paired validation

- [ ] Exercise both paired worktrees with a captured two-keyhole scene through the actual compiled simulator and uniform search; check that only the first opening is returned while the final goal remains blocked.
- [ ] Exercise continuation and target verification on simulated observations, checking exact simulator-call counts and the shared limit. No wheel commands, camera restart, or trial-result writes.
- [ ] Review the diff and fresh validation output; leave the implementation committed in both worktrees for review and later deployment.
