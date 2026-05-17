"""Diagnostics recording for NAMO runs.

This package implements the structured-output diagnostics pipeline used by
run_namo.py when --diag-path is supplied. The public surface is intentionally
small:

  DiagnosticsRecorder  — thread-safe event sink, writes JSONL files + summary
  Tee                  — duplicates a stream (typically stdout/stderr) into a file
  render_top_down      — synthetic top-down PNG renderer for sim/real scenes
  build_scene_state    — structured-JSON snapshot of the current scene
  request_camera_frame — fetches a JPEG frame from a running camera_service

All higher-level call-sites should depend on DiagnosticsRecorder. Each of its
methods is a no-op when recorder.enabled is False, so call-sites never need
`if recorder:` guards.
"""

from robot_control.diagnostics.recorder import DiagnosticsRecorder
from robot_control.diagnostics.tee import Tee

__all__ = [
    "DiagnosticsRecorder",
    "Tee",
]
