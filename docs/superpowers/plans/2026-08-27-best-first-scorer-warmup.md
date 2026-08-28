# Best-First Scorer Warmup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Warm the learned best-first scorer once before measured planning, record warmup separately, and freeze formal real-trial seeds to 0–4.

**Architecture:** `namo_cpp` owns scorer construction and exposes a service-level preload-and-warm operation that uses the same process cache as search. `robot_control` invokes it once before its planning timer, attaches the excluded duration to the first plan record, aggregates it separately, and documents the formal protocol.

**Tech Stack:** Python 3.12, PyTorch, NumPy, pytest, JSONL diagnostics, Markdown.

**Engineering Standards:** Follow `plan-coding-standards`: keep warmup allocation one-time and reuse the cached scorer; document all new public methods; use repository naming; keep the service, bridge, planner, recorder, and documentation responsibilities separate; use named warmup constants; propagate contextual failures; log the exclusion boundary explicitly; test behavior and failure-sensitive timing before each scoped commit; add no environment-specific paths or secrets.

---

### Task 1: Backend scorer warmup

**Files:**
- Modify: `/home/dhruv/projects_dhruv/namo/namo_cpp/scripts/sandbox/live_scorer.py`
- Modify: `/home/dhruv/projects_dhruv/namo/namo_cpp/python/namo/services/planning_service.py`
- Test: `/home/dhruv/projects_dhruv/namo/namo_cpp/python/tests/test_live_scorer_warmup.py`
- Test: `/home/dhruv/projects_dhruv/namo/namo_cpp/python/tests/test_planning_service.py`

- [ ] **Step 1: Write failing scorer and service tests**

Add a `LiveScorer.__new__` unit test whose fake `score_ctx` records three calls and asserts each call receives context shape `(5, 64, 64)`, contact shape `(60, 2)`, `h=2`, and `raw=True`. Add a planning-service test that monkeypatches `namo.strategies.scorer_goal_strategy._get_scorer`, invokes `preload_best_first_scorer`, and asserts the service supplies the checkpoint, its own config path, the requested device, and calls `warmup(repeats=3)`.

- [ ] **Step 2: Run the tests and verify RED**

Run: `cd /home/dhruv/projects_dhruv/namo/namo_cpp && set -a && . ./env.robotlearning.sh && set +a && PYTHONPATH="$PWD/build_python:$PWD/python" /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest python/tests/test_live_scorer_warmup.py python/tests/test_planning_service.py -q`

Expected: FAIL because `LiveScorer.warmup` and `NAMOPlanningService.preload_best_first_scorer` do not exist.

- [ ] **Step 3: Implement the minimal backend warmup**

Add named constants for channel count, output size, contact count, planning horizon, and default repeats near the existing scorer constants. Add `LiveScorer.warmup(self, repeats=DEFAULT_WARMUP_REPEATS)` with a public docstring; allocate zero-valued NumPy inputs once and call `score_ctx(..., h=WARMUP_HORIZON, raw=True)` exactly `repeats` times. Add `NAMOPlanningService.preload_best_first_scorer(self, checkpoint, device="cpu", repeats=3)` with a public docstring; lazily import `_get_scorer`, resolve the same cache key `(checkpoint, self._config_path, device)`, and call `scorer.warmup(repeats=repeats)`.

- [ ] **Step 4: Run focused and backend suites**

Run the RED command again and expect PASS. Then run: `PYTHONPATH="$PWD/build_python:$PWD/python" /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest python/tests -q` and expect all backend Python tests to pass.

- [ ] **Step 5: Commit the backend stage**

Run: `git add scripts/sandbox/live_scorer.py python/namo/services/planning_service.py python/tests/test_live_scorer_warmup.py python/tests/test_planning_service.py && git commit -m "feat: warm best-first scorer before planning"`

### Task 2: Invoke warmup outside robot-control planning time

**Files:**
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/src/robot_control/planner/namo_bridge.py`
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/src/robot_control/planner/namo_planner.py`
- Test: `/home/dhruv/projects_dhruv/namo/robot_control/tests/test_namo_planner_chain_reuse.py`

- [ ] **Step 1: Write failing timing-boundary tests**

Extend the fake bridge with `warmup_best_first_scorer` call recording and a fixed returned duration. Add a model-prior test using `LocalSearchConfig(local_search="best_first", best_first_prior="model", scorer_ckpt="model.ckpt", ml_device="cuda")`; assert warmup is called once across two plans, the first diagnostic records the warmup duration, the second records zero, and the mocked `perf_counter` values still produce only the bridge-plan interval as `planning_wall_time_ms`. Add a uniform-prior test asserting no warmup call and zero excluded duration.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cd /home/dhruv/projects_dhruv/namo/robot_control && PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest tests/test_namo_planner_chain_reuse.py -q`

Expected: FAIL because the bridge and planner have no ranker-warmup API or diagnostic fields.

- [ ] **Step 3: Implement bridge and planner behavior**

Add `NAMOPlanBridge.warmup_best_first_scorer(checkpoint, device) -> float` with a public docstring. It obtains the cached planning service, calls `preload_best_first_scorer`, measures the complete initialization with `perf_counter`, logs the checkpoint/device and that the duration is excluded, and returns milliseconds.

Add one `_model_ranker_warmed` boolean to `NAMOPlanner` and a focused `_warmup_model_ranker_once() -> float` helper. The helper returns zero unless `self._local_search.uses_ranker`, returns zero after success, otherwise calls the bridge using the configured checkpoint/device and marks completion only after the call succeeds.

Call the helper once at the top of `_generate_plan`, before routing to held or whole-problem planning and before either `planning_wall_start`. Pass the returned duration into the first held or fresh diagnostic record as `model_warmup_ms` and set `model_warmup_excluded_from_planning_time` from whether a warmup ran. Emit zero/false on subsequent fresh, reuse, and decision-only records so the schema is explicit.

- [ ] **Step 4: Run focused and repository suites**

Run the RED command again and expect PASS. Then source `../namo_cpp/env.robotlearning.sh`, export `NAMO_REPO=/home/dhruv/projects_dhruv/namo/namo_cpp`, and run `PYTHONPATH="$NAMO_REPO/build_python:src" /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest tests -q`; expect the repository-owned suite to pass.

- [ ] **Step 5: Commit the robot-control warmup stage**

Run: `git add src/robot_control/planner/namo_bridge.py src/robot_control/planner/namo_planner.py tests/test_namo_planner_chain_reuse.py && git commit -m "feat: exclude ranker warmup from planning time"`

### Task 3: Aggregate and document warmup and seeds

**Files:**
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/src/robot_control/diagnostics/recorder.py`
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/tests/test_diagnostics_planning_metrics.py`
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/tests/test_runtime_terminal_outcomes.py`
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/tests/test_real_exp_metrics_docs.py`
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/real_exp/README.md`
- Modify: `/home/dhruv/projects_dhruv/namo/robot_control/real_exp/METRICS.md`

- [ ] **Step 1: Write failing aggregation and documentation tests**

Update diagnostics expectations to require `model_warmup_ms`, feed a positive warmup value on the first fresh-search record, and assert it is not added to `wall_time_ms_total`. Update the runtime-summary expectation accordingly. Extend the documentation contract test to require seeds `0`, `1`, `2`, `3`, `4`, `--shuffle-seed`, deterministic model wording, uniform seed consumption, seed-42 pilot wording, `model_warmup_ms`, and explicit exclusion from planning wall time.

- [ ] **Step 2: Run the tests and verify RED**

Run: `PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest tests/test_diagnostics_planning_metrics.py tests/test_runtime_terminal_outcomes.py tests/test_real_exp_metrics_docs.py -q`

Expected: FAIL because the aggregate and documentation do not yet define the new fields or seed protocol.

- [ ] **Step 3: Implement aggregation and documentation**

Initialize `DiagnosticsRecorder.planning["model_warmup_ms"]` to zero and add each non-negative per-record `model_warmup_ms` independently of the operation-specific wall-time sums. Do not change the calculation of `wall_time_ms_total`.

Add a formal-trials section to `real_exp/README.md` mapping trial 1–5 to seeds 0–4, requiring `--shuffle-seed "$seed"`, explaining that uniform consumes the seed while model ranking is deterministic for an identical scene, and retaining existing seed-42 artifacts as valid pilots outside the formal timing set. Update the example command to calculate the explicit seed. Add warmup boundary and `summary.json.planning.model_warmup_ms` definitions to `real_exp/METRICS.md`, using one source line per prose paragraph.

- [ ] **Step 4: Run focused and full robot-control suites**

Run the RED command again and expect PASS. Then run the fully sourced repository suite from Task 2 and expect all tests to pass.

- [ ] **Step 5: Commit the metrics and protocol stage**

Run: `git add src/robot_control/diagnostics/recorder.py tests/test_diagnostics_planning_metrics.py tests/test_runtime_terminal_outcomes.py tests/test_real_exp_metrics_docs.py real_exp/README.md real_exp/METRICS.md && git commit -m "docs: freeze warmup and seed protocol"`

### Task 4: End-to-end verification and delivery

**Files:**
- Verify only; no planned source changes.

- [ ] **Step 1: Run formatting and diff checks**

Run `git diff --check HEAD~2..HEAD` in `robot_control` and `git diff --check HEAD~1..HEAD` in `namo_cpp`. Inspect `git status --short` in both repositories and confirm only pre-existing unrelated user changes remain.

- [ ] **Step 2: Run a simulation-only warmup timing probe**

Run model-prior planning twice against `real_exp/results/hmax2/easy_020/uniform_search/trial1/scene_before.json` without connecting to serial. Confirm the startup log reports one excluded warmup, both plans return the same two-push chain for an identical scene, the first plan record has positive `model_warmup_ms`, the second has zero, and both `planning_wall_time_ms` values exclude the one-time warmup.

- [ ] **Step 3: Re-run both complete test suites**

Run the complete backend and robot-control commands from Tasks 1 and 2 immediately before claiming completion.

- [ ] **Step 4: Push coherent commits**

Push the current `namo_cpp` branch and then the current `robot_control` branch only after all checks pass. Do not stage, modify, or commit the pre-existing RVG submodule change, GUI shutdown work, generated environments, or recorded trial artifacts.
