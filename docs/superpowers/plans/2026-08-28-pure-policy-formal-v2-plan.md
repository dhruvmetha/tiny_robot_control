# Pure-Policy Formal-v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reset the real-experiment results tree for the pure-policy protocol and make every new run record clean, paired robot/backend Git provenance.

**Architecture:** Diagnostics will snapshot both repositories before constructing the output directory, retain the legacy top-level `git` object, and add a `repositories` map for explicit pairing. The experiment runbook will use a fresh `formal_v2` namespace with separate `model_search` and `model_pure_policy` arms, while all existing results are permanently deleted within the user-approved boundary.

**Tech Stack:** Python 3.12, pytest, Git, Markdown, existing `DiagnosticsRecorder` pipeline.

**Engineering Standards:** Follow `plan-coding-standards`. Keep the recorder helper focused and reusable, derive the backend path from `NAMO_REPO`, preserve compatibility, emit actionable unavailable-provenance warnings, use one behavior-level integration test, and end each stage in a coherent tested commit. No unrelated refactoring or extra tests.

---

### Task 1: Capture paired provenance before output creation

**Files:**
- Create: `tests/test_diag_setup_provenance.py`
- Modify: `scripts/_diag_setup.py`

- [ ] **Step 1: Write the failing integration test**

Create one test that initializes two clean temporary Git repositories, patches the robot repository root and `NAMO_REPO`, invokes `bootstrap_diagnostics()` with an output directory inside the robot repository, and restores `sys.stdout`/`sys.stderr` in `finally`. Assert that `config.json` records both exact short commits as clean under `repositories`, and that the compatibility `git` object matches `robot_control`.

```python
def test_bootstrap_snapshots_both_repositories_before_creating_output(
    tmp_path, monkeypatch,
):
    robot_repo = _clean_git_repo(tmp_path / "robot")
    namo_repo = _clean_git_repo(tmp_path / "namo")
    monkeypatch.setattr(diag, "ROBOT_CONTROL_REPO", robot_repo)
    monkeypatch.setenv("NAMO_REPO", str(namo_repo))
    args = argparse.Namespace(
        diag_path=str(robot_repo / "real_exp" / "results" / "formal_v2"),
        run_name="trial1",
        allow_overwrite=False,
        goal=[11.0, 67.6],
        sim=False,
        config=None,
        objects=None,
        strategy="primitive",
        algorithm="full_namo",
        capture_scene=False,
    )

    old_out, old_err = sys.stdout, sys.stderr
    recorder = log_file = None
    try:
        recorder, log_file = diag.bootstrap_diagnostics(args)
    finally:
        sys.stdout, sys.stderr = old_out, old_err
        if log_file is not None:
            log_file.close()

    payload = json.loads((recorder.root_dir / "config.json").read_text())
    assert payload["repositories"]["robot_control"]["commit"] == _head(robot_repo)
    assert payload["repositories"]["namo_cpp"]["commit"] == _head(namo_repo)
    assert payload["repositories"]["robot_control"]["dirty"] is False
    assert payload["repositories"]["namo_cpp"]["dirty"] is False
    assert payload["git"] == payload["repositories"]["robot_control"]
```

- [ ] **Step 2: Run the new test and verify RED**

Run:

```bash
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest \
  tests/test_diag_setup_provenance.py -q
```

Expected: fail because `scripts._diag_setup` does not expose paired `repositories` provenance and snapshots Git after output creation.

- [ ] **Step 3: Implement the minimal provenance helper**

In `scripts/_diag_setup.py`, define the repository root from the script location, replace the tuple-only Git query with a reusable snapshot, and collect both repositories before constructing `DiagnosticsRecorder`:

```python
ROBOT_CONTROL_REPO = Path(__file__).resolve().parent.parent


def _git_snapshot(repo: Optional[Path]) -> Dict[str, Any]:
    unavailable = {
        "commit": "nogit",
        "dirty": False,
        "branch": "unknown",
        "available": False,
    }
    if repo is None:
        return unavailable
    try:
        def git(*args: str) -> str:
            return subprocess.run(
                ["git", "-C", str(repo), *args],
                capture_output=True,
                text=True,
                check=True,
                timeout=2.0,
            ).stdout.strip()

        return {
            "commit": git("rev-parse", "--short", "HEAD"),
            "dirty": git("status", "--porcelain") != "",
            "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "available": True,
        }
    except (OSError, subprocess.SubprocessError):
        return unavailable


def _repository_snapshots() -> Dict[str, Dict[str, Any]]:
    backend = os.environ.get("NAMO_REPO")
    return {
        "robot_control": _git_snapshot(ROBOT_CONTROL_REPO),
        "namo_cpp": _git_snapshot(Path(backend).expanduser()) if backend else _git_snapshot(None),
    }
```

At the beginning of `bootstrap_diagnostics()`, after validating flags but before creating `run_dir`, call `_repository_snapshots()`. Pass that immutable snapshot to `_build_config_payload()`. Keep the legacy top-level `git` object by copying the robot snapshot, add `repositories`, and print a logged warning when `namo_cpp.available` is false.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run:

```bash
PYTHONPATH=src /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest \
  tests/test_diag_setup_provenance.py \
  tests/test_diagnostics_planning_metrics.py \
  tests/test_namo_planner_diagnostics.py -q
```

Expected: all pass with no failures.

- [ ] **Step 5: Commit paired provenance**

```bash
git add scripts/_diag_setup.py tests/test_diag_setup_provenance.py
git commit -m "feat: record paired experiment revisions"
```

### Task 2: Permanently reset results and document formal-v2

**Files:**
- Delete: every existing path below `real_exp/results/`
- Create: `real_exp/results/.gitkeep`
- Modify: `real_exp/README.md`
- Modify: `real_exp/METRICS.md`

- [ ] **Step 1: Resolve and validate the destructive boundary**

From the robot-control root, verify that the target resolves exactly to:

```text
/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results
```

List the immediate children, then permanently delete everything below that directory without touching `real_exp/environments`, shortlist files, or documentation.

- [ ] **Step 2: Recreate the tracked empty root**

Create only `real_exp/results/.gitkeep`. Verify with `find real_exp/results -mindepth 1` that `.gitkeep` is the sole remaining path.

- [ ] **Step 3: Update the runbook**

Edit `real_exp/README.md` to define only the formal `model_search` and `model_pure_policy` arms, route commands to `results/formal_v2`, pin backend `1628d1f`, retain seeds 0–4 and automatic model warmup, and require both repository snapshots during preflight. Remove legacy-trial and simulation-filtered-policy descriptions.

- [ ] **Step 4: Update metric semantics**

Edit `real_exp/METRICS.md` so `model_search` counts all physics transitions while `model_pure_policy` records zero physics-rollout simulations and still times graph construction, rendering, candidate ranking, and inference. Remove deleted legacy-trial accounting.

- [ ] **Step 5: Validate documentation and deletion scope**

Run:

```bash
test "$(find real_exp/results -mindepth 1 -printf '%P\n')" = ".gitkeep"
grep -RInE "formal_v1|model_greedy_policy|simulation-filtered|Completed easy trial" \
  real_exp/README.md real_exp/METRICS.md
git diff --check
```

Expected: the results assertion passes, the grep produces no matches, and `git diff --check` exits zero.

- [ ] **Step 6: Commit the reset and protocol documentation**

```bash
git add -A real_exp/results real_exp/README.md real_exp/METRICS.md
git commit -m "docs: reset results for pure-policy formal v2"
```

### Task 3: Verify the frozen instrument and push

**Files:**
- Verify only; no planned source changes.

- [ ] **Step 1: Verify backend revision and cleanliness**

Run in `namo_cpp`:

```bash
test "$(git rev-parse HEAD)" = "1628d1ff195047246315aa81a7808ba5300bc379"
git status --short
```

Expected: exact revision match and no output from status.

- [ ] **Step 2: Run the complete robot-control project suite**

Source `namo_cpp/env.robotlearning.sh`, then run from robot control:

```bash
PYTHONPATH="$NAMO_REPO/build_python:src" \
  /home/dhruv/miniconda3/envs/namo312/bin/python -m pytest tests -q
```

Expected: all project tests pass.

- [ ] **Step 3: Audit final Git state and protocol requirements**

Verify the only remaining uncommitted path is the previously known RVG submodule compatibility edit, inspect all commits and diffs since `9054404`, confirm the camera service is still healthy, and confirm no robot/planner process owns the serial port.

- [ ] **Step 4: Push the coherent commits**

```bash
git push origin real-robot
```

Expected: the remote branch advances through the provenance and formal-v2 reset commits. Do not push or modify `namo_cpp`; its frozen revision already matches its remote.
