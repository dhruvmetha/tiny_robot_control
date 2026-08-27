# Known issues and status

This document tracks reproducible or source-supported problems that still need
resolution. Each entry records current evidence and the data needed to confirm
or close it.

## Ignore rules miss a symlink of the same name

**Status:** Fixed 2026-08-27. Kept as a record because the rule still reads as
though the slash were harmless, and because verifying it needs a specific
command.

A `.gitignore` rule ending in `/` matches a directory and not a symlink of the
same name. Reproduced in this checkout: a rule `foo/` ignores a real `foo`
directory and leaves a `foo` symlink untracked and visible, so `git add -A`
stages it. In the sibling repository that put an absolute machine-local path
into a commit, because a git worktree has no build output of its own and
symlinking the main tree's build is the obvious thing to do.

Eleven rules here end in `/`, including `recordings/`, `calibration/`,
`chassis_calibration/`, `output_dump/`, `build_cards/` and `real_trials/`. Each
covers a directory that a person might reasonably relocate to another disk and
symlink back. `real_trials/` is the one that matters most, because it holds
trial videos, the raw push archive, and the sheets the study reads.

There is also no rule of any kind for `build_python/`, which the trial command
references as `$NAMO_REPO/build_python`. A symlink or directory by that name in
this repository would be untracked and stageable today.

The fix: every rule naming a relocatable directory now appears twice, with and
without the trailing slash, and `build_python` gained a rule of both kinds.
Confirmed by creating a symlink named `build_python` pointing at the sibling
repository's build and watching `git status` stay silent about it.

`real_trials` is deliberately excluded from that treatment, and the exclusion
costs nothing. A bare rule would match the directory, and git cannot re-include
a path underneath an excluded directory, so it would kill the negations that
track `trials.csv` and the sheets.

The reason that is safe is the same tracking. Measured on 2026-08-27 by
replacing the directory with a symlink: git reports the 12 tracked files as
deleted and the symlink as untracked, and `git add -A` stages twelve removals.
That is a dozen deletions in the diff, not a quiet extra file. The symlink bug
this section is about is dangerous precisely where nothing is tracked, because
then the symlink's contents arrive as new material nobody looks at. Here the
failure runs the other way and announces itself.

Verify a change here with `git status --porcelain --untracked-files=all`, never
by reading the rule and never with `git check-ignore -q`, which exits 0 on a
negation match as well as on an ignore and will tell you a tracked file is
ignored.

## Two-hop tests collect on one machine only

**Status:** Open; found 2026-08-26, fix deferred until after the clearance
ladder so the suite does not change while it is the instrument checking
hardware work.

[`tests/test_region_opening_two_hop.py`](../../tests/test_region_opening_two_hop.py)
hardcodes the ranker checkpoint at line 59:

    SCORER_CKPT = "/home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt"

and skips the whole module with `allow_module_level=True` when that file is
absent. It is the only absolute home-directory path in the suite.

Four test functions each carry `@pytest.mark.parametrize("scene", SCENES)`, and
`SCENES` names `1push/2hop/env1` and `2push/2hop/env1`, so the module holds
eight tests. All eight disappear on any machine without that exact path, with
no failure and no warning. A second session collecting the same commit measured
316 tests where this checkout measures 324, and the whole difference is this
module.

Both `SCENES` entries are two-hop, so what goes missing is the
setup-then-finish chain coverage the hmax2 arm depends on. A one-push
regression would surface in other modules. This one would not.

Intended fix, not yet applied: resolve the checkpoint from an environment
variable with the current path as fallback, and keep the module-level skip for
a genuinely absent checkpoint. The module has a second module-level skip above
it, at line 78, for a missing canonical namo_cpp build. Keep the two messages
distinct and have the checkpoint skip name the variable it read, so the output
says which of the two things is missing and where it looked.

## Runtime auto-quit revalidation

**Status:** Open; intermittent recurrence after the intended fix is documented,
and a current end-to-end reproduction is still required.

The original symptom occurred after autonomous completion: the runtime printed
its shutdown messages, but the GUI could remain blocked in
`QApplication.exec()`. The run-end `finally` block then did not execute, so
cumulative summary, final scene, and success-replay artifacts could remain
unwritten.

Commit `570407b` landed two intended repairs:

- [`runtime.py`](runtime.py) stopped posting the extra `update`, `set_status`,
  and `update_drawings` calls in the autonomous completion close branch.
- [`gui/window.py`](gui/window.py) changed `close_window()` to schedule the
  normal close path with `QTimer.singleShot(0, self.close)` instead of directly
  quitting the application.

Commit `351bf0f`, two days later and after `570407b` in this branch's history,
records that the PyQt viewer event loop sometimes still did not quit cleanly.
It added per-plan simulation-success capture so XML, chain JSON, and replay MP4
artifacts can land during planning even when run-end finalization does not.
Current [`runtime.py`](runtime.py) retains the corresponding comment that the
event loop may stay parked on close. No later GUI/window fix appears in the
relevant path history, so `570407b` is a mitigation attempt rather than evidence
of closure; the recurrence remains open.

### Current mitigation and operational workaround

- With simulation-success capture enabled, use the inline
  `sim_replays/plan_NNN_*` artifacts written per plan; do not rely solely on the
  cumulative artifacts emitted from the run-end `finally` block.
- If the viewer remains parked after completion, preserve the diagnostics and
  console output before manually closing or interrupting the process.

The 2026-08-27 current-environment verification exercised fresh full search in
simulation-only probes, not the real-hardware or GUI event-loop path. A new
post-`570407b` reproduction should record the current commit, exact command and
run mode, viewer and capture flags, plan/push count, console log and diagnostics
path, which per-plan and cumulative artifacts landed, and whether the GUI event
loop returned and the runtime `finally` block completed.

## Plan-only fallback summary can misreport a successful plan

**Status:** Open diagnostics limitation; observed 2026-08-27 in simulation-only
`--sim-xml` plan-only runs.

A successful `scripts/run_namo.py --sim --sim-xml ...` plan-only invocation can
write `solution.yaml` with `success: true` and then write `summary.json` with
`outcome: crashed`. Plan-only returns after writing its solution and bypasses
`Runtime`, whose `finally` path normally writes the authoritative runtime
summary. The top-level fallback summary writer sees no Runtime-written summary
and emits its generic crash record.

For this plan-only case, the fallback `summary.json` does not mean planning
crashed. Read `solution.yaml` and the plan-only log lines for the planning
result, and treat the contradictory fallback summary as a diagnostics defect.
This deterministic explanation is scoped to the observed simulation-only
plan-only path; it does not establish that the real-hardware Runtime path has
the same behavior.
