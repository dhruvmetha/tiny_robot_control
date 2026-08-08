# Robot Control Documentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the current `robot_control` checkout understandable and operable from a new top-level README, with accurate architecture, limitation, contributor, and feature documentation beneath it.

**Architecture:** Use a curated documentation hierarchy. `README.md` is the concise project entrypoint and index, `src/robot_control/ARCHITECTURE.md` owns detailed component/data-flow documentation, focused guides retain task-specific detail, and limitation documents describe only verified gaps in this checkout. Explicitly distinguish code that exists from integrations that are operational.

**Tech Stack:** Markdown, Python CLI entrypoints, Git, shell-based link/path validation.

**Engineering Standards:** Follow `plan-coding-standards`. Keep each document focused, reuse links instead of duplicating long explanations, use repository naming and relative paths, avoid environment-specific absolute paths, make blocker messages actionable, validate all commands and links, and end each coherent documentation stage with a scoped commit. Runtime efficiency, logging, YAML option design, and application security are unchanged because this is documentation-only work.

---

## File map

- Create `README.md`: repository overview, current status, setup, quick starts, repository map, and documentation index.
- Rewrite `src/robot_control/ARCHITECTURE.md`: implemented package boundaries and runtime data flow.
- Rewrite `src/robot_control/TODO.md`: verified current-checkout gaps only.
- Rewrite `CLAUDE.md`: concise contributor guidance that defers general documentation to `README.md`.
- Modify `closed_loop_sessions/README.md`: mark service-dependent replanning commands as currently blocked.
- Modify `real_test_envs/README.md`: mark the canonical NAMO planning command as currently blocked while retaining the scene/capture guide.
- Validate and leave unchanged; the initial checkout audit found their focused
  content and referenced entrypoints present:
  - `docs/NAVIGATION_AND_WHEEL_COMMANDS.md`
  - `docs/PUSH_GEOMETRY.md`
  - `docs/PUSH_DURATION_CALIBRATION.md`
  - `docs/SIM_CALIBRATION_PLAN.md`
  - `docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md`
  - `src/robot_control/KNOWN_ISSUES.md`
  - `push_calibration/obj_1/diff/comparison.md`
- Preserve without staging `src/robot_control/controller/motion_planner/rvg`: unrelated dirty submodule compatibility edits.

The selected structure keeps the README scannable while giving architecture and operator workflows stable, focused homes. A single monolithic README would duplicate controller and calibration detail; a link-only README would leave known contradictions unresolved.

### Task 1: Add the repository entrypoint

**Files:**
- Create: `README.md`

- [ ] **Step 1: Confirm the preconditions and capture the current blocker**

Run:

```bash
cd /home/dhruv/projects_dhruv/namo/robot_control
test ! -e README.md
/home/dhruv/miniconda3/envs/namo312/bin/python - <<'PY'
try:
    from namo.services import NAMOPlanningService
except ImportError as exc:
    print(f"expected blocker: {exc}")
else:
    raise SystemExit("NAMOPlanningService unexpectedly exists; re-audit the status text")
PY
```

Expected: `README.md` is absent and the Python check prints an import blocker for `NAMOPlanningService`.

- [ ] **Step 2: Create `README.md` with the current-checkout contract**

Write a concise README with this exact section structure and content requirements:

```markdown
# robot_control

`robot_control` is the single-robot runtime and execution layer for the NAMO
workspace. It provides simulation and real-hardware environments, camera-based
observation, task planners, navigation and push controllers, diagnostics, scene
capture, and sim/real calibration tools.

## Current status

| Area | Status |
|---|---|
| Runtime, simulation, controllers, camera tooling, diagnostics, and calibration utilities | Implemented in this checkout |
| Captured-scene and closed-loop workspace utilities | Implemented; planning subcommands depend on the NAMO blocker below |
| NAMO-backed planning through `NAMOPlanBridge` | Blocked: `robot_control` imports `namo.services.NAMOPlanningService`, which is absent from the checked-out `namo_cpp` branch |

Do not imply that NAMO-backed `run_namo.py` or closed-loop `replan` commands
currently complete successfully. Point readers to the limitations section.

## Architecture at a glance

Show this implemented flow:

Observation source -> WorldState -> Planner -> SubgoalExecutor -> Controller
-> Action -> Environment

Explain that simulation or camera/real nodes supply observations, the runtime
owns orchestration, diagnostics record lifecycle events, and the NAMO bridge is
present but its external service dependency is unavailable.

## Setup

Document:

```bash
git submodule update --init --recursive
python -m pip install -e ".[dev]"
# Add GUI and remote-camera dependencies when needed:
python -m pip install -e ".[dev,gui,zmq]"
```

List `config/real.yaml`, `config/objects.yaml`, and `config/controller.yaml`.
State that real-hardware workflows additionally require the camera service and
robot transport configured for the local installation.

## Verified entrypoints

Include commands that are available in this checkout:

```bash
python scripts/test_control.py --controller keyboard
python scripts/camera_service.py --help
python scripts/capture_to_xml.py --help
python scripts/execute_real_push.py --help
python scripts/execute_sim_push.py --help
python scripts/closed_loop_session.py status --help
python -m pytest tests
```

Place NAMO planning commands in a separate "Currently blocked" paragraph and
link to the status explanation instead of presenting them as runnable.

## Repository map

Describe `src/robot_control`, `scripts`, `config`, `tests`, `real_test_envs`,
`closed_loop_sessions`, `docs`, and `push_calibration`. State that
`closed_loop_sessions*` run artifacts are generated and ignored.

## Documentation

Group and link every retained focused document:

- Architecture and maintenance:
  - `src/robot_control/ARCHITECTURE.md`
  - `src/robot_control/KNOWN_ISSUES.md`
  - `src/robot_control/TODO.md`
  - `CLAUDE.md`
- Operating workflows and scenes:
  - `closed_loop_sessions/README.md`
  - `real_test_envs/README.md`
- Controllers and planning geometry:
  - `docs/NAVIGATION_AND_WHEEL_COMMANDS.md`
  - `docs/PUSH_GEOMETRY.md`
  - `docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md`
- Calibration:
  - `docs/PUSH_DURATION_CALIBRATION.md`
  - `docs/SIM_CALIBRATION_PLAN.md`
  - `push_calibration/obj_1/diff/comparison.md`

Give every link a one-sentence description.

## Current limitations

Summarize the unavailable NAMO service, the unresolved `rvg` C++17/GCC 9
compatibility patch ownership problem, and the GUI auto-quit issue. Link to the
detailed architecture, known-issues, and current-gaps documents.
```

Use relative Markdown links, not backticked bare paths, in the final document.
Do not mention the proposed model-guided region-opening pipeline.

- [ ] **Step 3: Validate README links, paths, and advertised entrypoints**

Run:

```bash
python scripts/test_control.py --help >/dev/null
python scripts/camera_service.py --help >/dev/null
python scripts/capture_to_xml.py --help >/dev/null
python scripts/execute_real_push.py --help >/dev/null
python scripts/execute_sim_push.py --help >/dev/null
python scripts/closed_loop_session.py status --help >/dev/null
python -m pytest tests -q

for path in \
  src/robot_control/ARCHITECTURE.md \
  src/robot_control/KNOWN_ISSUES.md \
  src/robot_control/TODO.md \
  CLAUDE.md \
  closed_loop_sessions/README.md \
  real_test_envs/README.md \
  docs/NAVIGATION_AND_WHEEL_COMMANDS.md \
  docs/PUSH_GEOMETRY.md \
  docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md \
  docs/PUSH_DURATION_CALIBRATION.md \
  docs/SIM_CALIBRATION_PLAN.md \
  push_calibration/obj_1/diff/comparison.md; do
  test -e "$path" || exit 1
done

git diff --check -- README.md
```

Expected: every command exits zero, every indexed path exists, and `git diff --check` prints nothing.

- [ ] **Step 4: Commit the repository entrypoint**

```bash
git add README.md
git commit -m "docs: add robot control repository guide"
```

Expected: the commit contains only `README.md`.

### Task 2: Replace stale architecture and gap documentation

**Files:**
- Modify: `src/robot_control/ARCHITECTURE.md`
- Modify: `src/robot_control/TODO.md`

- [ ] **Step 1: Record stale identifiers and verified current components**

Run:

```bash
grep -nE "namo_task|src/robot_control/estimator|Concrete Controllers|Concrete Planners" \
  src/robot_control/ARCHITECTURE.md src/robot_control/TODO.md

for path in \
  src/robot_control/runtime.py \
  src/robot_control/executor.py \
  src/robot_control/environment/sim.py \
  src/robot_control/environment/real.py \
  src/robot_control/planner/namo_bridge.py \
  src/robot_control/planner/namo_planner.py \
  src/robot_control/controller/navigation.py \
  src/robot_control/controller/push.py \
  src/robot_control/nodes/remote_observer.py \
  src/robot_control/diagnostics/recorder.py; do
  test -f "$path" || exit 1
done
```

Expected: the grep finds stale claims and all implemented-component paths exist.

- [ ] **Step 2: Rewrite `ARCHITECTURE.md` around the actual package tree**

Replace the document with these sections and verified claims:

```markdown
# robot_control architecture

## Scope and current status
- Explain that `robot_control` owns single-robot orchestration and execution.
- State that the NAMO adapter code exists but is blocked by the missing service.

## Runtime data flow
- Document Runtime setup and the sense-plan-act loop.
- Show Observation source -> WorldState -> Planner -> SubgoalExecutor ->
  Controller -> Action -> Environment, with DiagnosticsRecorder alongside.

## Components
- Core: types, topics, world state, serialization, object definitions.
- Environments: base, simulation, real, action sender, MicroMVP adapter.
- Observation nodes: simulation sensor, camera sensor, remote observer/recorder.
- Planning: base, sequence, RVG, wavefront, NAMO planner/bridge/binding loader.
- Execution: SubgoalExecutor and controller dispatch.
- Controllers: keyboard, navigation, follow-path, push, edge geometry.
- Presentation/recording: GUI and diagnostics.

## Simulation flow
- RuntimeConfig(mode="sim") selects SimEnv and SimSensorNode.
- A configured planner enables autonomous subgoal execution; otherwise the
  selected controller drives interaction.

## Real-hardware flow
- RuntimeConfig(mode="real") loads real.yaml and objects.yaml.
- Observations come from local camera nodes or RemoteObserverNode.
- RealEnv sends wheel actions through its configured sender/adapter.

## NAMO boundary
- Describe XML generation, object mapping, binding loading, and subgoal
  conversion as implemented code.
- State that the current namo_cpp checkout does not provide
  NAMOPlanningService and therefore the boundary is not operational.

## External dependencies
- namo_cpp, camera/robot services, MuJoCo-related packages, and rvg submodule.

## Repository structure
- Show only directories and modules present in this checkout.
```

Use links to focused documents for controller equations and calibration rather
than duplicating them. Do not retain the old `namo_task`, estimator, or
micromvp-owned-RVG tree diagrams.

- [ ] **Step 3: Rewrite `TODO.md` as verified current gaps**

Keep the filename for compatibility, but replace its content with:

```markdown
# Current gaps

This file tracks verified gaps in the current checkout. Reproducible bugs with
detailed investigations live in `KNOWN_ISSUES.md`.

## Dependency and integration blockers
- Restore or replace NAMOPlanningService before NAMO-backed entrypoints can run.
- Make the two-line rvg C++17 compatibility change available from an owned fork
  or accepted upstream commit before updating the submodule pointer.

## Runtime and controller gaps
- Link the GUI auto-quit race to KNOWN_ISSUES.md; do not duplicate its analysis.
- Record only additional limitations confirmed directly from current source.

## Test coverage gaps
- State that automated coverage currently consists of the tests present under
  tests/ and does not exercise the unavailable NAMO service boundary or real
  hardware end to end.
- Record that bare `pytest` also collects vendored test suites inside the `rvg`
  submodule and fails on their unavailable bindings; the repository test command
  is `python -m pytest tests` until pytest discovery is scoped in configuration.
```

Do not list implemented controllers, planners, subgoals, GUI, or real
environment support as missing. Do not add speculative model work.

- [ ] **Step 4: Validate architecture paths and stale-claim removal**

Run:

```bash
! grep -nE "python -m namo_task|src/robot_control/estimator|Concrete Controllers.*Not Yet Implemented|Concrete Planners.*Not Yet Implemented" \
  src/robot_control/ARCHITECTURE.md src/robot_control/TODO.md

grep -q "NAMOPlanningService" src/robot_control/ARCHITECTURE.md
grep -q "not operational\|blocked\|absent" src/robot_control/ARCHITECTURE.md
grep -q "NAMOPlanningService" src/robot_control/TODO.md
grep -q "rvg" src/robot_control/TODO.md
git diff --check -- src/robot_control/ARCHITECTURE.md src/robot_control/TODO.md
```

Expected: stale-claim grep returns no matches, blocker checks succeed, and diff checking is clean.

- [ ] **Step 5: Commit the architecture correction**

```bash
git add src/robot_control/ARCHITECTURE.md src/robot_control/TODO.md
git commit -m "docs: align architecture with current checkout"
```

Expected: the commit contains exactly the two rewritten documents.

### Task 3: Align contributor and operator guides, then validate the hierarchy

**Files:**
- Modify: `CLAUDE.md`
- Modify: `closed_loop_sessions/README.md`
- Modify: `real_test_envs/README.md`
- Review: focused documents listed in the file map

- [ ] **Step 1: Rewrite `CLAUDE.md` as contributor guidance**

Use these sections:

```markdown
# CLAUDE.md - robot_control contributor guide

## Start here
- Link README.md and ARCHITECTURE.md.
- State that documentation describes only the current checkout.

## Current blocker
- State that NAMOPlanBridge exists but NAMOPlanningService is absent from the
  current namo_cpp branch.
- Direct contributors not to treat run_namo NAMO modes as operational.

## Repository commands
- Editable install and optional dependency groups.
- Verified --help commands and `python -m pytest tests`.

## Package map
- Concise list of actual source directories and responsibilities.

## Engineering notes
- Coordinate/unit conventions.
- Edge indexing and push_steps relationship as implemented.
- Generated closed_loop_sessions* output is ignored.
- Preserve unrelated dirty submodule changes.

## Focused documentation
- Link README.md's documentation index instead of duplicating it.
```

Remove the external `ROBOT_CONTROL_NAMO_INTEGRATION.md` dependency, the claim
that `NAMOPlanningService` is live, the future `sage_learning` architecture,
and any nonexistent package paths.

- [ ] **Step 2: Add operational-status notices to the scene and closed-loop guides**

Near the beginning of `real_test_envs/README.md`, add a notice that:

- Scene inspection and recapture instructions describe available tooling.
- The documented `run_namo.py` planning command is retained as the intended
  entrypoint for this checkout but currently stops at the absent
  `NAMOPlanningService` dependency.

Near the beginning of `closed_loop_sessions/README.md`, add a notice that:

- Layout/status/migration/preparation helpers exist.
- `replan`, `replan-reuse-only`, and `replan-full-search-only` ultimately need
  the unavailable NAMO service when they enter full planning.
- Real execution commands additionally require configured camera and robot
  hardware services.

Do not remove historical workspace-layout explanations or present generated
session data as version-controlled input.

- [ ] **Step 3: Validate the retained focused documents**

Verify that every retained document exists. The initial audit established that
the calibration CLIs expose `--help`, the navigation/push source paths exist,
the wavefront document is explicitly a follow-up record, the GUI auto-quit
issue still maps to direct window calls in `runtime.py`, and the comparison file
is historical experiment output. No content edit is planned for these files.

Run this inventory first:

```bash
for doc in \
  docs/NAVIGATION_AND_WHEEL_COMMANDS.md \
  docs/PUSH_GEOMETRY.md \
  docs/PUSH_DURATION_CALIBRATION.md \
  docs/SIM_CALIBRATION_PLAN.md \
  docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md \
  src/robot_control/KNOWN_ISSUES.md \
  push_calibration/obj_1/diff/comparison.md; do
  test -f "$doc" || exit 1
done

grep -RIn --include='*.md' -E "python -m namo_task|src/robot_control/estimator" \
  README.md CLAUDE.md docs closed_loop_sessions/README.md real_test_envs/README.md src/robot_control || true

grep -nE "close_window|_window.update|_window.set_status|_window.update_drawings" \
  src/robot_control/runtime.py
```

Expected: all documents exist, stale identifiers produce no current-architecture
matches, and the runtime grep confirms the known-issue code path still exists.

- [ ] **Step 4: Validate every repository Markdown link**

Run this read-only Python validator from the repository root:

```bash
python - <<'PY'
from pathlib import Path
import re
import sys

root = Path.cwd()
errors = []
for doc in sorted(root.rglob("*.md")):
    if ".git" in doc.parts or "src/robot_control/controller/motion_planner/rvg" in doc.as_posix():
        continue
    text = doc.read_text(encoding="utf-8")
    for match in re.finditer(r"\[[^\]]+\]\(([^)]+)\)", text):
        target = match.group(1).strip()
        if target.startswith(("http://", "https://", "mailto:", "#")):
            continue
        path_part = target.split("#", 1)[0]
        if not path_part:
            continue
        resolved = (doc.parent / path_part).resolve()
        if not resolved.exists():
            errors.append(f"{doc.relative_to(root)} -> {target}")
if errors:
    print("Broken Markdown links:", *errors, sep="\n")
    sys.exit(1)
print("All relative Markdown links resolve")
PY
```

Expected: `All relative Markdown links resolve`.

- [ ] **Step 5: Run final checkout-accuracy validation**

Run:

```bash
python scripts/test_control.py --help >/dev/null
python scripts/camera_service.py --help >/dev/null
python scripts/capture_to_xml.py --help >/dev/null
python scripts/execute_real_push.py --help >/dev/null
python scripts/execute_sim_push.py --help >/dev/null
python scripts/closed_loop_session.py --help >/dev/null

! grep -RIn --include='*.md' -E "python -m namo_task|src/robot_control/estimator" \
  README.md CLAUDE.md docs closed_loop_sessions/README.md real_test_envs/README.md src/robot_control

grep -q "NAMOPlanningService" README.md
grep -q "blocked\|absent\|unavailable" README.md
git diff --check

test "$(git status --porcelain | grep -v '^ m src/robot_control/controller/motion_planner/rvg$' | wc -l)" -ge 1
```

Expected: CLI checks and grep assertions pass, diff checking prints nothing,
and the only pre-existing non-documentation change remains the lowercase
submodule status for `rvg`.

- [ ] **Step 6: Commit contributor and operator-guide corrections**

Stage only documentation files changed in this task. Inspect the staged list
before committing:

```bash
git add CLAUDE.md closed_loop_sessions/README.md real_test_envs/README.md
git diff --cached --name-status
git diff --cached --check
git commit -m "docs: make guides reflect current checkout"
```

Expected: no source code, generated session data, or `rvg` submodule state is
staged.

- [ ] **Step 7: Verify the final branch state**

Run:

```bash
git status --short --branch
git log -4 --oneline --decorate
git diff --cached --quiet
test "$(git status --porcelain | grep -v '^ m src/robot_control/controller/motion_planner/rvg$' | wc -l)" -eq 0
```

Expected: the documentation commits are present, the staging area is empty,
and `rvg` is the only remaining dirty path.
