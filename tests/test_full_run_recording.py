"""One continuous video per run, with a burned-in elapsed timer.

The recorder used to write one MP4 per subgoal, so a run became 3-6 clips
and the seconds between subgoals were never on film. Now the runtime starts
full_run.mp4 once per session and the per-subgoal meta JSONs carry
wall-clock epochs that index into it. The elapsed timer is stamped on
recorded frames only, on a copy, so the shared live frame stays clean.
"""

from __future__ import annotations

import json

import numpy as np

from robot_control.core.types import NavigateSubgoal, Observation
from robot_control.runtime import FULL_RUN_VIDEO_STEM, Runtime
from robot_control.utils.camera_recorder import CameraRecorder, format_elapsed


def test_format_elapsed():
    assert format_elapsed(0.0) == "00:00.0"
    assert format_elapsed(75.34) == "01:15.3"
    assert format_elapsed(600.0) == "10:00.0"


def test_recorder_stamps_a_copy_not_the_shared_frame(tmp_path):
    rec = CameraRecorder(output_dir=str(tmp_path), fps=10.0)
    rec._is_recording = True
    rec._output_path = tmp_path / "clip.mp4"

    frame = np.zeros((64, 64, 3), dtype=np.uint8)
    rec._on_frame(frame, timestamp=100.0)
    rec._on_frame(frame, timestamp=100.5)
    saved = rec.stop()

    assert saved is not None and (tmp_path / "clip.mp4").exists()
    assert rec.frame_count == 0  # reset by stop
    # The publisher's frame is shared with the GUI; the stamp must not
    # have touched it.
    assert not frame.any()


class _FakeRemoteRecorder:
    def __init__(self):
        self.start_calls = []
        self.stop_calls = []

    def start(self, session_dir, filename=None):
        self.start_calls.append(filename)
        return f"{session_dir}/{filename}.mp4"

    def stop(self):
        self.stop_calls.append(True)


def _obs() -> Observation:
    return Observation(
        robot_x=10.0, robot_y=10.0, robot_theta=90.0, objects={}, timestamp=0.0
    )


def _recording_runtime(tmp_path) -> tuple:
    runtime = Runtime.__new__(Runtime)
    runtime._record_session_dir = str(tmp_path)
    runtime._record_subgoal_index = 0
    runtime._record_active = True
    runtime._remote_recorder = _FakeRemoteRecorder()
    runtime._planner = None
    return runtime, runtime._remote_recorder


def test_subgoal_bracket_never_touches_the_recorder(tmp_path):
    runtime, fake = _recording_runtime(tmp_path)
    sub = NavigateSubgoal(x=20.0, y=30.0)

    runtime._video_subgoal_start(sub, _obs())
    runtime._video_subgoal_done(_obs(), failed=False)

    # The film runs for the whole session; brackets only write meta.
    assert fake.start_calls == []
    assert fake.stop_calls == []


def test_subgoal_meta_indexes_into_the_full_run_film(tmp_path):
    runtime, _ = _recording_runtime(tmp_path)
    sub = NavigateSubgoal(x=20.0, y=30.0)

    runtime._video_subgoal_start(sub, _obs())
    runtime._video_subgoal_done(_obs(), failed=False)

    meta = json.loads((tmp_path / "subgoal_001_navigate.json").read_text())
    assert meta["video_path"] == f"{FULL_RUN_VIDEO_STEM}.mp4"
    assert meta["video_epoch_at_dispatch"] > 0
    assert meta["video_epoch_at_done"] >= meta["video_epoch_at_dispatch"]
    assert meta["outcome"] == "success"
