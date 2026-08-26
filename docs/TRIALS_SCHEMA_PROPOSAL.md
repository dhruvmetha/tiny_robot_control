# Proposed trials.csv header

Status: proposal. Dhruv approves the header, `real_robot` lands it. Nothing here is applied.

Written from the namo side while adding `--exec-mode`, because that flag creates the second crossed factor and this file is where a crossed design either survives or quietly loses a cell.

## The problem, stated once

Three things a row needs are not columns today.

Which ranker arm ran lives inside the free-text `command` string, as the substring `model HY5U_s2`.
Which execution mode ran has nowhere to live at all, since the flag is new.
Why a trial failed lives in prose in two documents, in no code and no file, and has never been written down for a real trial because all four existing rows are successes.

The matrix is 14 scenes x 2 arms x 2 modes = 56 runs, analysed paired within scene. Two crossed factors means four ways to mislabel a row rather than two. A row whose arm has to be recovered by grepping a command string is a row that can be recovered wrong, and cutting even one cell breaks the sign test. Only the four v1 pilot rows exist, so the cost of re-cutting the header is near zero today and rises the moment the matrix starts.

## Proposed header

```
trial_id,build_id,tier,axis,arm,exec_mode,started_at,ended_at,command,planner_outcome,failure_cause,user_verdict,notes,scene_checksum,log_path
```

Three columns added, nothing removed, nothing renamed.

`arm` and `exec_mode` sit next to `tier` and `axis` rather than at the end, because all four are design factors and a person reading a row should find the grouping keys together.
`failure_cause` sits next to `planner_outcome`, because it only qualifies that.

## Vocabularies

**`arm`** is `model` or `uniform`.

Which prior ordered the candidate pool. Matches `--best-first-prior` exactly.

**`exec_mode`** is `search` or `reactive`.

Which decision rule ran per replan. `search` expands a priority queue and can back up; `reactive` takes the argmax at the state in front of the robot and cannot.

These two strings are `--exec-mode` verbatim, and `--exec-mode` is namo_cpp's `BOUNDARY_MODES` verbatim. One vocabulary across the CLI, both repositories and this file, with `test_exec_mode_routing.py::test_both_repositories_spell_the_modes_the_same_way` pinning that they stay equal. A translation table anywhere along that chain would be one more place to write the wrong arm.

The token was `policy` in the first draft of this proposal and is now `reactive`, decided by namo-a1 on the grounds that `policy` versus `search` names the wrong difference: the search runs the same ranker, ordering its queue by exactly the quantity the reactive rule takes the max of, so calling one arm "policy" implies the other has none. `reactive` versus `search` names the real difference, which is whether it looks ahead, and it matches `eval_reactive_argmax.py`. Renamed on both sides while zero matrix rows exist.

**`failure_cause`** is eight values, seven from `real_robot` plus `none`.

```
none                 the trial succeeded
corridor_too_tight
marker_unreachable
overshoot_onto_goal
stall
radio_dropout
planning_failed
other
```

The seven are `real_robot`'s taxonomy verbatim, from `docs/ICRA_REAL_ROBOT_STUDY.md:71` and `docs/REAL_ROBOT_TRIALS.md:119`. The human at the table assigns it at verdict time, and the verdict is ground truth over the planner's exit code.

`none` is the piece I argued for and it was accepted. A successful trial needs an explicit value, because an empty cell reads as both "did not fail" and "nobody triaged this yet", and after a long table session those are very different. That ambiguity resolves itself wrongly at 1am.

The first two are load-bearing sooner than the rest. Three of the 14 matrix scenes have a worst-case route near the threshold, `easy_002` at 9.95 cm and `easy_001` and `med_077` at 11.2. If the calibration ladder lands above 9.95 and one of those fails, this column is the only thing separating `corridor_too_tight` from `marker_unreachable`, because at the table they look identical.

## The pilot rows

All four are v1, pre-matrix, all successes. Backfill rather than leave blank:

`arm=model`, from the `HY5U_s2` already in their `command` string.
`exec_mode=search`, because reactive did not exist on 2026-08-23. That is a fact about the date, not a guess.
`failure_cause=none`.

## Keep `command`, and cross-check against it

`command` should stay. It is the only full record of what actually ran, and the parsed columns are a summary of it.

But the fix for "the arm lives in free text" is not to delete the free text, it is to stop the two from being able to disagree. A small check over the file, asserting that every row's `arm` and `exec_mode` match what its `command` string says, turns a transcription slip into a failure at the table instead of a discovered-in-analysis one. `real_robot` asked for it here, so it is written: `scripts/check_trials_consistency.py`, with `tests/test_trials_consistency.py` covering it. It also enforces the two vocabularies and the `none`-on-success rule, and it runs against a file that does not exist yet without complaining.

The console already helps: `describe_effective_search` now prints `exec mode: reactive  |  local search: best_first/model` before anything moves, on every path including the default. A person filling a row can read both factors off one line rather than reconstructing them.

## The open problem this proposal does not solve

`real_robot` found it while reading the flag: the pre-registered matrix command passes neither `--hold-region-target` nor `--active-target`, and only the held-boundary loop reaches `solve_boundary_from_xml`, which is the only method that reads the mode.

So as written the two arms would not differ by execution mode. They would differ by code path, search on the whole-problem planning path and reactive on the held-boundary loop, with different verification semantics and a different boundary-selection loop between them. The sign test would compare two pipelines and report it as a mode effect.

Two ways out, both amending a pre-registered command and both Dhruv's: either both arms gain `--hold-region-target` so they share the held loop and differ only in mode, or exec mode learns to work on the non-held path. `real_robot` leans to the first and so do I, since it is one flag on both arms and keeps the refusal honest, but it does change the search arm's code path from what the pilot ran.

Nothing in this file depends on which way that goes. The columns are right either way.

## What I am not proposing

The trial protocol, the randomisation, or where the file lives. Those are `real_robot`'s and Dhruv's.
