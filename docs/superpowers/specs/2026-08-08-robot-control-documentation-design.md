# Robot Control Documentation Design

## Objective

Create a reliable documentation entrypoint for `robot_control` that explains
the current checkout at a high level and routes readers to focused documents
for operation, captured scenes, controller internals, calibration, and known
issues.

The documentation must describe only code and workflows present in the current
checkout. It must not present the proposed model-guided region-opening pipeline
or any other future architecture as implemented.

## Audience

The primary readers are:

- Developers trying to understand the repository and its component boundaries.
- Operators trying to run simulation, camera, calibration, captured-scene, or
  closed-loop utilities.
- Contributors diagnosing the boundary between `robot_control`, `namo_cpp`,
  hardware services, and the `rvg` submodule.

## Source-of-truth hierarchy

The documentation will use a curated hierarchy rather than duplicating all
details in one file:

1. `README.md` is the repository entrypoint and documentation index.
2. `src/robot_control/ARCHITECTURE.md` is the detailed description of the
   implemented package architecture and runtime data flow.
3. Feature documents retain focused implementation and operator detail.
4. `src/robot_control/KNOWN_ISSUES.md` records reproducible defects.
5. `src/robot_control/TODO.md` records only verified gaps in the current
   checkout.
6. `CLAUDE.md` contains concise contributor guidance and defers general project
   documentation to `README.md`.

The workspace-level `ROBOT_CONTROL_NAMO_INTEGRATION.md` is outside this Git
repository and was last verified against an older planner-service interface.
The repository documentation will not depend on it as a current source of
truth.

## Root README

Add `README.md` with the following sections:

1. **Purpose** — describe `robot_control` as the single-robot execution,
   simulation, observation, controller, diagnostics, and NAMO-adapter layer
   between planning code and physical MicroMVP hardware.
2. **Current status** — distinguish available components from unavailable
   integration. In particular, state that the bridge code imports
   `namo.services.NAMOPlanningService`, which is absent from the checked-out
   `namo_cpp` branch, so NAMO-backed modes do not currently start.
3. **Architecture at a glance** — show the implemented sense-plan-act flow and
   identify the environment, world state, planner, executor, controllers, and
   diagnostics boundaries.
4. **Setup** — document the editable install, optional dependency groups,
   configuration files, `rvg` submodule requirement, and external runtime
   services used by real-hardware commands.
5. **Verified entrypoints** — include only commands whose files, arguments, and
   import paths can be validated in this checkout. Commands blocked by the
   missing NAMO service must be labeled as blocked rather than runnable.
6. **Repository map** — summarize the main packages, scripts, configuration,
   tests, captured scenes, and generated output roots.
7. **Documentation index** — group focused documents by task and give each link
   a one-sentence description.
8. **Current limitations** — summarize important blockers and link to
   `KNOWN_ISSUES.md` and `TODO.md` for details.

The README will stay concise enough to scan. Controller equations, calibration
schemas, and long operator procedures remain in focused documents.

## Existing-document changes

### `src/robot_control/ARCHITECTURE.md`

Rewrite it against the actual package tree. Remove references to nonexistent
`namo_task` and `src/robot_control/estimator` packages. Cover:

- `Runtime` orchestration and observation flow.
- Environment implementations and hardware adapter boundary.
- Planner interface and the present NAMO bridge code.
- `SubgoalExecutor`, navigation, and push controllers.
- Camera/remote-observer nodes, GUI, and diagnostics.
- Simulation versus real-hardware execution.
- Current external dependencies, including the blocked NAMO service boundary.

### `src/robot_control/TODO.md`

Replace obsolete claims that controllers, planners, subgoals, real hardware,
and GUI are unimplemented. Retain only gaps verified from the checkout and
separate them into:

- Dependency and integration blockers.
- Runtime/controller gaps.
- Documentation or test coverage gaps.

Do not include speculative model work.

### `CLAUDE.md`

Make `README.md` the general documentation entrypoint. Remove the claim that
`NAMOPlanningService` integration is live and remove future-model architecture
from the description of current behavior. Retain concise repository commands,
engineering conventions, and links to the rewritten architecture and focused
documents.

### Focused documents

Retain and index these documents:

- `closed_loop_sessions/README.md`
- `real_test_envs/README.md`
- `docs/NAVIGATION_AND_WHEEL_COMMANDS.md`
- `docs/PUSH_GEOMETRY.md`
- `docs/PUSH_DURATION_CALIBRATION.md`
- `docs/SIM_CALIBRATION_PLAN.md`
- `docs/WAVEFRONT_UNIFICATION_FOLLOWUPS.md`
- `src/robot_control/KNOWN_ISSUES.md`
- `push_calibration/obj_1/diff/comparison.md`

Audit them for claims contradicted by the checkout. Correct factual drift that
affects usage, but do not broaden the task into rewriting historical experiment
analysis. Generated experiment directories remain ignored.

## Accuracy rules

- Describe an integration as available only when its imports and entrypoint can
  run in the configured environment.
- Distinguish code that exists from workflows that are operational.
- Prefer relative repository links and paths.
- Do not cite nonexistent packages, scripts, configuration files, or external
  workspace documents as required reading.
- Do not document planned model inputs, priority queues, regional-opening
  composition, or production behavior that has not been implemented here.
- Preserve the unrelated dirty `rvg` compatibility edits without staging or
  changing them.

## Verification

The documentation change is complete when all of the following checks pass:

1. Every relative Markdown link resolves inside the repository.
2. Every referenced source, script, configuration, and documentation path
   exists.
3. Commands presented as runnable expose the documented arguments through
   `--help` or an equivalent non-mutating check.
4. Repository documentation no longer presents `namo_task`, the removed
   estimator package, or `NAMOPlanningService` as working current components.
5. The root README links every retained focused document and clearly labels its
   purpose.
6. `git diff --check` passes.
7. Only documentation files are included in the implementation commit; the
   dirty `rvg` submodule remains untouched.

## Non-goals

- Implementing or restoring `NAMOPlanningService`.
- Implementing the new model-guided planning pipeline.
- Changing runtime, controller, planner, or calibration behavior.
- Reorganizing source packages.
- Rewriting Git history or restoring removed experiment data.
