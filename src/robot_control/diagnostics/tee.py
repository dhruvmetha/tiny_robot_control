"""Stream-duplicator used to mirror stdout/stderr into a run.log file.

The Tee class wraps an original stream and a destination file. Every write
goes to both — the user still sees terminal output, and the diagnostics
directory ends up with a complete copy. File should be opened line-buffered
(open(path, "w", buffering=1)) so partial buffers survive a crash.
"""

from __future__ import annotations

from typing import IO


class Tee:
    """Wrap a stream so writes go to both the original and a destination file.

    Implements the minimal interface Python expects of sys.stdout/sys.stderr:
    write(), flush(), isatty(), fileno(). Anything else (e.g. encoding-aware
    methods) is delegated to the original stream via __getattr__ so libraries
    that introspect sys.stdout keep working.
    """

    def __init__(self, original: IO[str], destination: IO[str]) -> None:
        self._original = original
        self._destination = destination

    def write(self, data: str) -> int:
        # Always write to the original first so the terminal stays
        # responsive even if the destination file write blocks.
        n = self._original.write(data)
        try:
            self._destination.write(data)
        except Exception:
            # Diagnostics must never break the program. Swallow file-write
            # errors silently; the original stream still received the data.
            pass
        return n

    def flush(self) -> None:
        try:
            self._original.flush()
        except Exception:
            pass
        try:
            self._destination.flush()
        except Exception:
            pass

    def isatty(self) -> bool:
        try:
            return self._original.isatty()
        except Exception:
            return False

    def fileno(self) -> int:
        # Required for libraries that bypass write() (e.g. subprocess stdout
        # redirection). They get the original fd; the destination won't see
        # those writes, but that's an accepted tradeoff.
        return self._original.fileno()

    def __getattr__(self, name: str):
        # Forward any other attribute access to the original stream.
        return getattr(self._original, name)
