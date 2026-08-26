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

**`exec_mode`** is `search` or `policy`.

Which decision rule ran per replan. `search` expands a priority queue and can back up; `policy` takes the argmax at the state in front of the robot and cannot.

These two strings are `--exec-mode` verbatim, and `--exec-mode` is namo_cpp's `BOUNDARY_MODES` verbatim. One vocabulary across the CLI, both repositories and this file, with `test_exec_mode_routing.py::test_both_repositories_spell_the_modes_the_same_way` pinning that they stay equal. A translation table anywhere along that chain would be one more place to write the wrong arm.

The prose calls this mode "reactive" and the token is `policy`. Say the word, log the token. If `reactive` is the better token it is a one-line change on both sides and I would rather make it now than after 56 rows.

**`failure_cause`** needs `real_robot` to fill in.

The taxonomy is theirs and it lives in prose at `docs/ICRA_REAL_ROBOT_STUDY.md:71` and `docs/REAL_ROBOT_TRIALS.md:119`, neither of which is at my base commit. I am not going to invent it from a summary.

Two values are already load-bearing and should be in it: `corridor_too_tight` and `marker_unreachable`. Three of the 14 matrix scenes have a worst-case route near the threshold, `easy_002` at 9.95 cm and `easy_001` and `med_077` at 11.2. If the calibration ladder lands above 9.95 and one of those fails, that column is the only thing separating the two, because at the table they look identical.

One thing that is a schema question rather than a taxonomy one: a successful trial needs an explicit value, not an empty cell. Empty reads as both "did not fail" and "nobody triaged it yet", and after a long table session those are very different. Proposed `none` for a success and `unset` for a failure not yet triaged, so a row that still needs a person is greppable.

## The pilot rows

All four are v1, pre-matrix, all successes. Backfill rather than leave blank:

`arm=model`, from the `HY5U_s2` already in their `command` string.
`exec_mode=search`, because reactive did not exist on 2026-08-23. That is a fact about the date, not a guess.
`failure_cause=none`.

## Keep `command`, and cross-check against it

`command` should stay. It is the only full record of what actually ran, and the parsed columns are a summary of it.

But the fix for "the arm lives in free text" is not to delete the free text, it is to stop the two from being able to disagree. A small check over the file, asserting that every row's `arm` and `exec_mode` match what its `command` string says, turns a transcription slip into a failure at the table instead of a discovered-in-analysis one. That check is cheap and I will write it if `real_robot` wants it here rather than in their own harness.

The console already helps: `describe_effective_search` now prints `exec mode: policy | local search: best_first/model` before anything moves, on every path including the default. A person filling a row can read both factors off one line rather than reconstructing them.

## What I am not proposing

Nothing about the failure taxonomy's contents, the trial protocol, the randomisation, or where the file lives. Those are `real_robot`'s and Dhruv's.
