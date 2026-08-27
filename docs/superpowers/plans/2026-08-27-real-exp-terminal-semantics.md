# Real Experiment Terminal Semantics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make autonomous NAMO runs report success only when the latest planning simulation says the goal is reachable, report an unreachable empty plan as failure, and run `real_exp` without held-region targeting while preserving MPC suffix reuse.

**Architecture:** `NAMOPlanner.is_complete()` becomes a reachability predicate instead of a combined success/failure sentinel. `Runtime` retains one terminal `(outcome, reason)` tuple, maps planner completion and empty-plan failure to distinct statuses, and reuses that tuple for shutdown and `summary.json`. The `real_exp` command omits both held targeting and the held-loop-only explicit execution-mode flag; unheld `full_namo` still performs best-first search through its normal whole-problem path.

**Tech Stack:** Python 3.12, pytest, existing `NAMOPlanBridge`/compiled `namo_rl` reachability analysis, MuJoCo plan replay.

**Engineering Standards:** Follow `plan-coding-standards`. Use test-first red/green changes, repository naming conventions, explicit terminal constants instead of repeated magic strings, focused private helpers with docstrings, actionable logs, scoped commits, and the pinned `namo312` environment. Do not modify the dirty RVG submodule or the unrelated Qt shutdown changes.

---

## File map

- Modify `src/robot_control/planner/namo_planner.py`: define completion exclusively by simulated goal reachability.
- Modify `src/robot_control/runtime.py`: distinguish success from planning failure, retain the terminal outcome, and shut down both terminal states consistently.
- Modify `tests/test_navigate_to_goal_failure_propagates.py`: update old assertions that intentionally treated exhausted failure as completion and add reachability-completion regressions.
- Create `tests/test_runtime_terminal_outcomes.py`: cover runtime status, shutdown, and summary-outcome consistency.
- Modify `real_exp/README.md`: remove held-target and explicitly named execution-mode flags from the experiment command while documenting that suffix reuse remains enabled.
- Modify `docs/superpowers/specs/2026-08-27-real-exp-terminal-semantics-design.md`: record the completed verification only after every code and replay check passes.

### Task 1: Make planner completion mean simulated reachability

**Files:**
- Modify: `tests/test_navigate_to_goal_failure_propagates.py`
- Modify: `src/robot_control/planner/namo_planner.py:956-1005`

- [ ] **Step 1: Write failing planner tests**

Add tests that isolate the two required decisions:

```python
def test_planning_failure_alone_is_not_completion(monkeypatch):
    planner = _make_planner(monkeypatch)
    obs = _obs()
    planner._planning_failed = True
    planner._is_goal_reachable = lambda _obs: False

    assert planner.is_complete(obs) is False


def test_simulated_reachability_is_completion_before_physical_arrival(monkeypatch):
    planner = _make_planner(monkeypatch)
    obs = _obs()  # 42.4 cm from the configured goal
    planner._planning_failed = False
    planner._is_goal_reachable = lambda _obs: True

    assert planner.is_complete(obs) is True
```

Update `test_repeated_navigate_failures_give_up_instead_of_freezing` so it asserts `_planning_failed is True` but no longer claims that failure itself is completion.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
python -m pytest tests/test_navigate_to_goal_failure_propagates.py -v
```

Expected: the new failure-alone test sees `True`, and the distant-but-reachable test sees `False` under the old implementation.

- [ ] **Step 3: Implement the minimal planner behavior**

Replace the failure/distance implementation of `is_complete()` with the existing simulation predicate:

```python
def is_complete(self, obs: Observation) -> bool:
    """Return whether the latest planning scene has a path to the goal.

    Planning exhaustion is a failure state, not completion. Physical arrival
    is not required by the real_exp manipulation-success criterion.
    """
    reachable = self._is_goal_reachable(obs)
    if reachable:
        print(
            f"[NAMOPlanner] GOAL REACHABLE in simulation | "
            f"Total planning: {self._plan_count} calls, "
            f"{self._total_planning_ms:.0f}ms cumulative"
        )
    return reachable
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the same pytest command. Expected: all tests in the file pass.

- [ ] **Step 5: Commit the coherent planner change**

```bash
git add src/robot_control/planner/namo_planner.py \
  tests/test_navigate_to_goal_failure_propagates.py
git commit -m "fix: define NAMO completion by reachability"
```

### Task 2: Give the runtime explicit success and failure terminals

**Files:**
- Create: `tests/test_runtime_terminal_outcomes.py`
- Modify: `src/robot_control/runtime.py:1-240, 970-1090, 1447-1469`

- [ ] **Step 1: Write failing runtime tests**

Use `Runtime.__new__` plus small fake planner/executor/config objects to avoid GUI, serial, camera, or MuJoCo dependencies. Cover:

```python
def test_no_subgoal_while_unreachable_is_planning_failure():
    runtime = _runtime(planner=_Planner(complete=False, subgoal=None))

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Planning Failed"
    assert runtime._terminal_outcome == (
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )


def test_reachability_is_plan_complete_without_requesting_a_subgoal():
    planner = _Planner(complete=True, subgoal=None)
    runtime = _runtime(planner=planner)

    _action, _drawings, status = runtime._autonomous_step(_obs())

    assert status == "Plan Complete"
    assert planner.plan_calls == 0
    assert runtime._terminal_outcome == (
        "success",
        "goal reachable in simulation",
    )


def test_retained_terminal_outcome_drives_summary_classification():
    runtime = Runtime.__new__(Runtime)
    runtime._terminal_outcome = (
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )

    assert runtime._determine_outcome() == runtime._terminal_outcome
```

Also test that the terminal-status helper stops/closes a configured run for `Planning Failed`, just as it already does for `Plan Complete`.

- [ ] **Step 2: Run the new test and verify RED**

Run:

```bash
python -m pytest tests/test_runtime_terminal_outcomes.py -v
```

Expected: failure status/constants/retained outcome are absent, and the current empty-plan branch returns `Plan Complete`.

- [ ] **Step 3: Implement explicit terminal state**

Add module constants:

```python
PLAN_COMPLETE_STATUS = "Plan Complete"
PLANNING_FAILED_STATUS = "Planning Failed"
AUTONOMOUS_TERMINAL_STATUSES = frozenset(
    {PLAN_COMPLETE_STATUS, PLANNING_FAILED_STATUS}
)
```

Initialize readable state in `Runtime.__init__`:

```python
self._terminal_announced = False
self._terminal_outcome: Optional[Tuple[str, str]] = None
```

Add one idempotent recorder:

```python
def _record_terminal_outcome(self, outcome: str, reason: str) -> None:
    """Retain the first autonomous terminal decision for summary writing."""
    if self._terminal_outcome is None:
        self._terminal_outcome = (outcome, reason)
```

In `_autonomous_step`, record reachability success before returning `Plan Complete`. If `plan()` returns `None`, recheck `is_complete()` once for generic planners that transition during `plan()`; record success if true, otherwise record failure and return `Planning Failed`.

Extract the existing terminal stop/close block into a private helper that accepts both terminal statuses, prints either `Plan complete` or `Planning failed`, stops the real robot once, and honors `quit_on_complete` for both. Keep the existing GUI-thread-safe `close_window()` call unchanged.

At the start of `_determine_outcome`, return `_terminal_outcome` when present. Retain the existing best-effort fallbacks for operator aborts or legacy callers that never reached `_autonomous_step`, but update success wording from `goal reached` to `goal reachable in simulation`.

- [ ] **Step 4: Run runtime and planner tests and verify GREEN**

```bash
python -m pytest tests/test_runtime_terminal_outcomes.py \
  tests/test_navigate_to_goal_failure_propagates.py -v
```

Expected: all tests pass with no camera, robot, or GUI process started.

- [ ] **Step 5: Commit the runtime change**

```bash
git add src/robot_control/runtime.py tests/test_runtime_terminal_outcomes.py
git commit -m "fix: distinguish planning failure from completion"
```

### Task 3: Make the real_exp command use fresh whole-problem planning

**Files:**
- Modify: `real_exp/README.md:52-75`

- [ ] **Step 1: Add a command regression assertion**

Add a focused text assertion to `tests/test_runtime_terminal_outcomes.py`:

```python
def test_real_exp_command_does_not_enable_held_targeting():
    readme = (Path(__file__).resolve().parents[1] / "real_exp" / "README.md").read_text()
    command = readme.split("## First trial command", 1)[1]
    assert "--hold-region-target" not in command
    assert "--exec-mode" not in command
```

The explicit `--exec-mode search` must be omitted together with held targeting: the CLI correctly rejects a named execution mode on the unheld whole-problem path. `full_namo` plus `best_first`/`model` still performs search; only the held-loop-specific routing flag disappears.

- [ ] **Step 2: Run the assertion and verify RED**

```bash
python -m pytest \
  tests/test_runtime_terminal_outcomes.py::test_real_exp_command_does_not_enable_held_targeting -v
```

Expected: the README still contains both flags.

- [ ] **Step 3: Update the experiment command and explanation**

Change the command fragment to:

```bash
--robot-model car --algorithm full_namo \
--local-search best_first --best-first-prior model \
```

Add a short note: MPC suffix reuse remains enabled by the default `--execution-mode mpc`; after invalid reuse, unheld `full_namo` rebuilds the graph and selects a fresh boundary. Do not add a new option or YAML key.

- [ ] **Step 4: Run the docs assertion and verify GREEN**

Run the focused assertion again. Expected: pass.

- [ ] **Step 5: Commit focused experiment documentation**

```bash
git add real_exp/README.md \
  tests/test_runtime_terminal_outcomes.py
git commit -m "docs: run real experiments without held targets"
```

### Task 4: Verify regressions and replay easy_020 without hardware

**Files:**
- Modify after verification: `docs/superpowers/specs/2026-08-27-real-exp-terminal-semantics-design.md`

- [ ] **Step 1: Run focused terminal and suffix-reuse suites**

```bash
python -m pytest \
  tests/test_runtime_terminal_outcomes.py \
  tests/test_navigate_to_goal_failure_propagates.py \
  tests/test_namo_planner_chain_reuse.py \
  tests/test_reuse_not_vacuous.py \
  tests/test_exec_mode_routing.py -v
```

Expected: all tests pass; suffix and full-chain reuse expectations are unchanged.

- [ ] **Step 2: Run the full repository suite**

```bash
python -m pytest tests
```

Expected: zero failures. Preserve and report the exact pass count.

- [ ] **Step 3: Replay the captured post-push scene through unheld full_namo**

Source `../namo_cpp/env.robotlearning.sh`, set `NAMO_REPO`, and run a read-only Python diagnostic that loads
`real_exp/results/hmax2/easy_020/model_search/trial1/scene_after.json` into an `Observation`, calls `NAMOPlanBridge.analyze_reachability()`, then calls `NAMOPlanBridge.plan()` with:

```python
algorithm="full_namo"
goal_strategy="primitive"
local_search="best_first"
best_first_prior="model"
scorer_ckpt="/home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt"
max_chain_depth=2
shuffle_edges=False
robot_goal_cm=(29.7, 71.0)
```

Expected: initial reachability is false; fresh whole-problem selection chooses the goal route and enters model best-first simulation (`simulations_used > 0`). No camera, serial port, `RealEnv`, or wheel command is opened.

- [ ] **Step 4: Record verified implementation status**

Only after Steps 1-3 pass, change the design status from awaiting approval to implemented and append the exact focused/full/replay commands and results. Then commit only that documentation:

```bash
git add docs/superpowers/specs/2026-08-27-real-exp-terminal-semantics-design.md
git commit -m "docs: record terminal semantics verification"
```

- [ ] **Step 5: Inspect final diff and repository state**

```bash
git diff --check HEAD~4..HEAD
git status --short --branch
```

Expected: only scoped committed changes in the implementation worktree. In the primary checkout, preserve the pre-existing dirty RVG submodule and Qt shutdown files.

- [ ] **Step 6: Integrate and report**

Integrate the scoped commits into `real-robot`, rerun the full suite in the primary checkout where the Qt shutdown fix is also present, and report the exact tests/replay result and commit IDs. Do not launch another physical trial until the user explicitly requests it.
