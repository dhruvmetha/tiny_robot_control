"""A run killed from outside must still classify its summary.

Formal trials cap run_namo with `timeout N ...` because a wedged robot never
finishes on its own. `timeout` sends SIGTERM, whose default disposition kills
the interpreter without running finally blocks, so before this fix the runs
that most needed a record wrote no summary.json at all. The fix routes
SIGTERM through Runtime.abort(), which pins the outcome and lets run()'s
normal summary path fire.
"""

from importlib import import_module
from pathlib import Path
import signal
import sys

from robot_control.runtime import Runtime

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
run_namo = import_module("run_namo")


def _bare_runtime() -> Runtime:
    runtime = Runtime.__new__(Runtime)
    runtime._terminal_outcome = None
    runtime._running = False  # stop() becomes a no-op shortcut
    return runtime


def test_abort_records_reason_then_stops():
    runtime = _bare_runtime()
    calls = []
    runtime.stop = lambda: calls.append("stop")

    runtime.abort("wall_clock_timeout")

    assert runtime._terminal_outcome == ("aborted", "wall_clock_timeout")
    assert calls == ["stop"]


def test_abort_does_not_overwrite_a_real_terminal_outcome():
    runtime = _bare_runtime()
    runtime._terminal_outcome = ("success", "goal reached")

    runtime.abort("wall_clock_timeout")

    # The robot reached the goal before the supervisor's grace ran out; the
    # summary must say success, not timeout.
    assert runtime._determine_outcome() == ("success", "goal reached")


def test_abort_reason_drives_summary_classification():
    runtime = _bare_runtime()
    runtime.abort(run_namo.WALL_CLOCK_TIMEOUT_REASON)

    assert runtime._determine_outcome() == (
        "aborted",
        run_namo.WALL_CLOCK_TIMEOUT_REASON,
    )


def test_sigterm_handler_aborts_with_the_timeout_reason():
    aborted = []

    class _FakeRuntime:
        def abort(self, reason):
            aborted.append(reason)

    handler = run_namo._sigterm_handler_for(_FakeRuntime())
    handler(signal.SIGTERM, None)

    assert aborted == [run_namo.WALL_CLOCK_TIMEOUT_REASON]
