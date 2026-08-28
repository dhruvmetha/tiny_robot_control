# Real Runtime Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Make NAMO runtime success require the real or simulated robot to finish navigation to the goal, while preserving planning failure as a distinct failed outcome.

**Architecture:** Restore NAMOPlanner.is_complete() as the arrival predicate and keep reachability inside plan() as the transition that returns the existing NavigateSubgoal. Runtime remains planner-agnostic: it records success only when the planner reports arrival, and treats an empty incomplete plan as failure. Experiment documentation enables scene snapshots and labels the already-recorded trial2 as incomplete rather than rewriting its historical summary.

**Tech Stack:** Python 3.12, pytest, existing NAMOPlanner, Runtime, navigation executor, diagnostics JSON/JSONL, Markdown runbooks.

**Engineering Standards:** Follow plan-coding-standards. Use small behavior-specific changes, retain existing abstractions, update materially changed docstrings and terminal logs, avoid new configuration or environment-specific source values, use contextual outcome reasons, run behavior-focused and full-suite verification, and finish every source/documentation stage with a coherent commit.

---

## File Map

- src/robot_control/planner/namo_planner.py owns NAMO task-completion semantics and goal/retarget arrival tolerance.
- src/robot_control/runtime.py maps planner completion and no-action states to retained terminal outcomes.
- tests/test_navigate_to_goal_failure_propagates.py pins planner navigation, arrival, and failure behavior without hardware.
- tests/test_runtime_terminal_outcomes.py pins runtime dispatch and terminal summary behavior with fakes.
- real_exp/README.md defines the paper-trial launch command and artifact expectations.
- docs/superpowers/specs/2026-08-27-real-exp-terminal-semantics-design.md marks the reachability-as-runtime-success portion as superseded.
- docs/superpowers/specs/2026-08-27-real-runtime-completion-design.md records implementation and verification evidence.
- real_exp/results/hmax2/easy_020/model_search/trial2/VALIDITY.json is an untracked sidecar preserving trial2 while declaring it incomplete.

### Task 1: Restore arrival-based planner completion

**Files:**
- Modify: tests/test_navigate_to_goal_failure_propagates.py
- Modify: src/robot_control/planner/namo_planner.py:976

- [ ] **Step 1: Replace the reachability-completion regression with arrival regressions**

Replace test_simulated_reachability_is_completion_before_physical_arrival and extend success coverage with:

    def test_reachable_goal_is_not_complete_before_physical_arrival(monkeypatch):
        planner = _make_planner(monkeypatch)
        obs = _obs()
        planner._planning_failed = False
        planner._is_goal_reachable = lambda _obs: True

        assert planner.is_complete(obs) is False
        subgoal = planner.plan(obs)
        assert isinstance(subgoal, NavigateSubgoal)
        assert (subgoal.x, subgoal.y) == planner._robot_goal_cm


    def test_successful_navigation_completes_only_inside_goal_tolerance(monkeypatch):
        planner = _make_planner(monkeypatch)
        planner.plan(_obs())
        planner.notify_subgoal_done(_obs(), failed=False)

        far = Observation(
            robot_x=34.0, robot_y=40.0, robot_theta=90.0,
            objects={}, timestamp=1.0,
        )
        arrived = Observation(
            robot_x=36.0, robot_y=40.0, robot_theta=90.0,
            objects={}, timestamp=2.0,
        )

        assert planner.is_complete(far) is False
        assert planner.is_complete(arrived) is True

Retain test_planning_failure_alone_is_not_completion unchanged so the old false-success bug remains pinned.

- [ ] **Step 2: Run the planner tests and verify RED**

Run from the sourced NAMO environment:

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_navigate_to_goal_failure_propagates.py -v

Expected: the reachable-but-distant assertion fails because current is_complete() returns True; the arrival test fails because current completion ignores navigation state and distance.

- [ ] **Step 3: Restore the minimal arrival predicate**

Replace NAMOPlanner.is_complete() with:

    def is_complete(self, obs: Observation) -> bool:
        """Return whether goal navigation has physically reached its target.

        Reachability causes plan() to dispatch a NavigateSubgoal.
        Planning exhaustion is a failure state, not completion.
        """
        if self._planning_failed or not self._navigating_to_goal:
            return False

        target = self._retarget_point_cm or self._robot_goal_cm
        dist = math.hypot(obs.robot_x - target[0], obs.robot_y - target[1])
        if dist >= self._goal_tolerance:
            return False

        if self._retarget_point_cm is not None:
            print(
                f"[NAMOPlanner] GOAL REACHED (success-with-retarget)! "
                f"Distance to retarget point: {dist:.1f}cm | "
                f"Total planning: {self._plan_count} calls, "
                f"{self._total_planning_ms:.0f}ms cumulative"
            )
        else:
            print(
                f"[NAMOPlanner] GOAL REACHED! Distance: {dist:.1f}cm | "
                f"Total planning: {self._plan_count} calls, "
                f"{self._total_planning_ms:.0f}ms cumulative"
            )
        return True

This restores only arrival. It deliberately does not restore the former planning-failed-to-True branch.

- [ ] **Step 4: Run planner and goal-retarget tests and verify GREEN**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_navigate_to_goal_failure_propagates.py       tests/test_goal_retarget.py -v

Expected: all selected tests pass; failed navigation remains bounded, and retarget arrival remains supported.

- [ ] **Step 5: Commit the planner behavior**

    git add src/robot_control/planner/namo_planner.py       tests/test_navigate_to_goal_failure_propagates.py
    git commit -m "fix: require arrival for NAMO completion"

### Task 2: Make runtime success describe physical completion

**Files:**
- Modify: tests/test_runtime_terminal_outcomes.py
- Modify: src/robot_control/runtime.py:1088-1138

- [ ] **Step 1: Add a runtime regression for navigation dispatch and success reason**

Import NavigateSubgoal, let the fake executor retain dispatched subgoals, and replace the reachability-as-complete test with:

    def test_reachable_but_not_arrived_dispatches_navigation():
        navigate = NavigateSubgoal(x=29.7, y=71.0, theta=None)
        planner = _Planner(complete=False, subgoal=navigate)
        runtime = _runtime(planner)

        _action, _drawings, status = runtime._autonomous_step(_obs())

        assert status == "Autonomous: navigating"
        assert runtime._executor.subgoal == navigate
        assert runtime._terminal_outcome is None


    def test_arrival_records_goal_reached_success_without_planning_again():
        planner = _Planner(complete=True, subgoal=None)
        runtime = _runtime(planner)

        _action, _drawings, status = runtime._autonomous_step(_obs())

        assert status == "Plan Complete"
        assert planner.plan_calls == 0
        assert runtime._terminal_outcome == ("success", "goal reached")

Extend _Executor with the real interface used by the tested branch:

    def __init__(self):
        self.subgoal = None

    def set_subgoal(self, subgoal, _obs):
        self.subgoal = subgoal

    def step(self, _obs):
        from robot_control.core.types import Action
        return Action.stop()

    def get_drawings(self):
        return []

    def get_status(self):
        return "navigating"

Initialize _subgoal_start_time and stub diagnostics/video dispatch hooks in _runtime() so this remains hardware-free.

- [ ] **Step 2: Run runtime tests and verify RED**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_runtime_terminal_outcomes.py -v

Expected: navigation dispatch executes after completing the fake, while the arrival assertion fails because runtime records goal reachable in simulation.

- [ ] **Step 3: Change both runtime completion reasons**

In both completion branches of Runtime._autonomous_step(), use:

    self._record_terminal_outcome("success", "goal reached")

Do not change the incomplete empty-plan branch:

    self._record_terminal_outcome(
        "failure",
        "planner returned no subgoal while goal was unreachable",
    )

- [ ] **Step 4: Run terminal, shutdown, and navigation tests and verify GREEN**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_runtime_terminal_outcomes.py       tests/test_navigate_to_goal_failure_propagates.py       tests/test_navigation_failure_is_terminal.py       tests/test_runtime_qt_shutdown.py -v

Expected: all selected tests pass with physical arrival separated from planning failure and normal shutdown unchanged.

- [ ] **Step 5: Commit runtime terminal semantics**

    git add src/robot_control/runtime.py tests/test_runtime_terminal_outcomes.py
    git commit -m "fix: finish real NAMO trials at the goal"

### Task 3: Correct the experiment command and historical semantics

**Files:**
- Modify: tests/test_runtime_terminal_outcomes.py
- Modify: real_exp/README.md
- Modify: docs/superpowers/specs/2026-08-27-real-exp-terminal-semantics-design.md

- [ ] **Step 1: Add the missing scene-capture command assertion**

Extend test_real_exp_command_does_not_enable_held_targeting with:

    assert "--capture-scene" in command

- [ ] **Step 2: Run the documentation command test and verify RED**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_runtime_terminal_outcomes.py::test_real_exp_command_does_not_enable_held_targeting -v

Expected: fail because the current launch command does not contain --capture-scene.

- [ ] **Step 3: Update the real experiment runbook**

Add --capture-scene beside --record-video in the first-trial command. State that reachability dispatches physical navigation, real success requires observed arrival within 5 cm, and trial2 is an incomplete opening-success attempt. The next clean paper trial must use a fresh directory after resetting the scene.

- [ ] **Step 4: Mark the prior reachability-success design as superseded**

Add this note to 2026-08-27-real-exp-terminal-semantics-design.md:

    > **Superseded terminal criterion:** This document's reachability-as-runtime-
    > success decision was superseded by
    > 2026-08-27-real-runtime-completion-design.md. Planning failure remains a
    > failure, but runtime success now requires goal navigation and arrival.

- [ ] **Step 5: Run documentation tests and diff validation**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_runtime_terminal_outcomes.py -v
    git diff --check

Expected: runtime documentation tests pass and git diff --check emits no output.

- [ ] **Step 6: Commit runbook corrections**

    git add real_exp/README.md       docs/superpowers/specs/2026-08-27-real-exp-terminal-semantics-design.md       tests/test_runtime_terminal_outcomes.py
    git commit -m "docs: require physical completion in real experiments"

### Task 4: Preserve trial2 as an incomplete experiment artifact

**Files:**
- Create outside the tracked branch: real_exp/results/hmax2/easy_020/model_search/trial2/VALIDITY.json

- [ ] **Step 1: Create a non-destructive validity sidecar**

Create the sidecar in the primary checkout without editing summary.json:

    {
      "paper_trial_valid": false,
      "classification": "incomplete_end_to_end_trial",
      "reason": "runtime stopped when the post-push scene became simulation-reachable; the robot remained 32.786885274486984 cm from the goal and did not execute the final NavigateSubgoal",
      "preserved_evidence": "two successful physical pushes and an opened simulated reachability path",
      "superseded_runtime_commit": "f236e037f278bc707e9158fbd3be410ba84085b2"
    }

Do not delete or rewrite the original summary, JSONL streams, XML, or videos.

- [ ] **Step 2: Validate the sidecar and preserved artifact counts**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m json.tool       real_exp/results/hmax2/easy_020/model_search/trial2/VALIDITY.json
    find real_exp/results/hmax2/easy_020/model_search/trial2 -type f | sort

Expected: valid JSON; both MP4 files and all original metrics remain present.

### Task 5: Verify, document, and integrate

**Files:**
- Modify: docs/superpowers/specs/2026-08-27-real-runtime-completion-design.md

- [ ] **Step 1: Run focused behavior and regression coverage**

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest       tests/test_runtime_terminal_outcomes.py       tests/test_navigate_to_goal_failure_propagates.py       tests/test_goal_retarget.py       tests/test_navigation_failure_is_terminal.py       tests/test_runtime_qt_shutdown.py       tests/test_namo_planner_chain_reuse.py       tests/test_namo_planner_diagnostics.py -v

Expected: all selected tests pass. This is the hardware-free runtime replay: a reachable distant observation dispatches navigation and does not produce terminal success.

- [ ] **Step 2: Run the complete project suite**

Source env.robotlearning.sh, set NAMO_REPO, then run:

    /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest -q tests

Expected: every project test passes with zero failures.

- [ ] **Step 3: Record exact verification evidence in the design**

Append an Implementation Status section containing focused and complete-suite pass counts, the RED failures observed before each fix, and a statement that no serial port or robot process was opened during verification.

- [ ] **Step 4: Commit verification evidence**

    git add docs/superpowers/specs/2026-08-27-real-runtime-completion-design.md
    git commit -m "docs: record real completion verification"

- [ ] **Step 5: Review branch scope and integrate**

    git status --short --branch
    git log --oneline real-robot..HEAD
    git diff --check real-robot...HEAD
    git diff --stat real-robot...HEAD

Expected: only planned source, test, runbook, spec, and plan changes appear. After verification, fast-forward real-robot, rerun the complete suite in the primary checkout so the preserved GUI shutdown test is included, and push origin/real-robot. Never overwrite or include the dirty RVG submodule, GUI file, untracked GUI test, or experiment data.
