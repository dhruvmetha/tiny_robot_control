# Known issues and status

This document tracks reproducible or source-supported problems that still need
resolution. Each entry records current evidence and the data needed to confirm
or close it.

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

This documentation work could not perform fresh full-search or real-hardware
end-to-end revalidation because those paths require the unavailable
full-search Python API or configured hardware. A new post-`570407b`
reproduction should record the current commit, exact command and run mode,
viewer and capture flags, plan/push count, console log and diagnostics path,
which per-plan and cumulative artifacts landed, and whether the GUI event loop
returned and the runtime `finally` block completed.
