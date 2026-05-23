"""Camera frame recorder - subscribes to camera topic and saves to video."""

from __future__ import annotations

import threading
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from pubsub import pub

from robot_control.core.topics import Topics


class CameraRecorder:
    """
    Records camera frames to a video file.

    Subscribes to Topics.CAMERA_BGR_RAW and records when active.

    Usage:
        recorder = CameraRecorder(output_dir="recordings")
        recorder.subscribe()  # Start listening to camera frames

        # Toggle recording on/off
        recorder.start()  # Begin recording
        # ... frames are automatically captured
        recorder.stop()   # Finalize video

        # Or use toggle:
        recorder.toggle()  # Start/stop

        # Cleanup
        recorder.unsubscribe()
    """

    def __init__(
        self,
        output_dir: str = "recordings",
        fps: float = 30.0,
        codec: str = "mp4v",
    ):
        """
        Initialize camera recorder.

        Args:
            output_dir: Directory to save recordings
            fps: Frames per second for output video
            codec: FourCC codec (default: mp4v — software MPEG-4 encoder,
                works without hardware acceleration. Use 'avc1' if your
                machine has a working H.264 encoder for smaller files.)
        """
        self._output_dir = Path(output_dir)
        self._fps = fps
        self._codec = codec

        self._writer: Optional[cv2.VideoWriter] = None
        self._output_path: Optional[Path] = None
        self._frame_count = 0
        self._is_recording = False
        self._subscribed = False
        # Wall-clock anchor for the current recording — set from the first
        # frame's timestamp in _on_frame. Used to slot incoming frames into
        # fixed 1/_fps bins so the output container's declared fps matches
        # real elapsed time regardless of the camera publisher's actual
        # delivery rate (which is ~15/s in practice, not the configured 60).
        self._start_timestamp: Optional[float] = None

        # Frames arrive on the pubsub publisher thread; start/stop are driven
        # from the camera_service REP thread. cv2.VideoWriter is not
        # thread-safe — write() concurrent with release() segfaults the
        # underlying encoder. This lock serialises every touch of _writer.
        self._lock = threading.Lock()

    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._is_recording

    @property
    def output_path(self) -> Optional[Path]:
        """Get current output file path (None if not recording)."""
        return self._output_path

    @property
    def frame_count(self) -> int:
        """Get number of frames recorded in current session."""
        return self._frame_count

    def subscribe(self) -> None:
        """Subscribe to camera frames."""
        if not self._subscribed:
            pub.subscribe(self._on_frame, Topics.CAMERA_BGR_RAW)
            self._subscribed = True
            print("[CameraRecorder] Subscribed to camera frames")

    def unsubscribe(self) -> None:
        """Unsubscribe from camera frames."""
        if self._subscribed:
            # Unsubscribe FIRST so no further _on_frame deliveries are in
            # flight, then stop the (now-quiet) writer. The previous order
            # (stop, then unsubscribe) left a small window where a frame
            # delivery could re-enter _on_frame after release().
            pub.unsubscribe(self._on_frame, Topics.CAMERA_BGR_RAW)
            self._subscribed = False
            if self._is_recording:
                self.stop()
            print("[CameraRecorder] Unsubscribed")

    def _on_frame(self, frame: np.ndarray, timestamp: float) -> None:
        """Handle incoming camera frame."""
        if frame is None:
            return

        # Hold the lock for the whole write — stop() must not be able to
        # call release() between our None-check and write(frame).
        with self._lock:
            if not self._is_recording:
                return

            # Initialize writer on first frame
            if self._writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*self._codec)
                self._writer = cv2.VideoWriter(
                    str(self._output_path),
                    fourcc,
                    self._fps,
                    (w, h),
                )

                if not self._writer.isOpened():
                    print(f"[CameraRecorder] Failed to open video writer: {self._output_path}")
                    self._is_recording = False
                    return

            # Real-time pacing: slot this frame into the 1/_fps grid anchored
            # on the first frame's timestamp. When the source is slower than
            # _fps the while-loop duplicates `frame` to fill the gap; when
            # the source is faster than _fps, slot < _frame_count and the
            # loop body never runs (frame is dropped). Net effect: the MP4's
            # declared _fps matches real elapsed time.
            if self._start_timestamp is None:
                self._start_timestamp = timestamp
            slot = int((timestamp - self._start_timestamp) * self._fps)
            while self._frame_count <= slot:
                self._writer.write(frame)
                self._frame_count += 1

    def start(self, filename: Optional[str] = None) -> str:
        """
        Start recording.

        Args:
            filename: Optional custom filename (without extension).
                     If None, uses timestamp: YYYY-MM-DD_HH-MM-SS.mp4

        Returns:
            Path to the output file
        """
        # mkdir doesn't need the lock and may do IO; do it outside.
        self._output_dir.mkdir(parents=True, exist_ok=True)

        if filename is None:
            filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        ext = ".avi" if self._codec.upper() == "XVID" else ".mp4"
        new_path = self._output_dir / f"{filename}{ext}"

        with self._lock:
            if self._is_recording:
                # Inline the stop logic under the same lock so callers don't
                # observe a "not recording → recording" gap during restart.
                print("[CameraRecorder] Already recording, stopping previous...")
                self._stop_locked()

            self._output_path = new_path
            self._frame_count = 0
            self._start_timestamp = None
            self._is_recording = True
            # Writer initialised lazily on first frame (need frame size).
            self._writer = None

        print(f"[CameraRecorder] Recording started: {new_path}")
        return str(new_path)

    def stop(self) -> Optional[str]:
        """
        Stop recording and finalize video file.

        Returns:
            Path to the saved video file, or None if not recording
        """
        with self._lock:
            return self._stop_locked()

    def _stop_locked(self) -> Optional[str]:
        # Must be called with self._lock held. Releases the writer and
        # resets recording state. The lock is what makes this safe against
        # _on_frame mid-write — release() and write() cannot interleave.
        if not self._is_recording:
            return None

        output_path = self._output_path

        if self._writer is not None:
            self._writer.release()
            self._writer = None

        self._is_recording = False

        if self._frame_count > 0:
            duration = self._frame_count / self._fps
            print(f"[CameraRecorder] Recording saved: {output_path}")
            print(f"  Frames: {self._frame_count}, Duration: {duration:.1f}s")
        else:
            if output_path and output_path.exists():
                output_path.unlink()
            print("[CameraRecorder] Recording stopped (no frames captured)")
            output_path = None

        self._output_path = None
        self._frame_count = 0

        return str(output_path) if output_path else None

    def toggle(self) -> bool:
        """
        Toggle recording on/off.

        Returns:
            True if now recording, False if stopped
        """
        if self._is_recording:
            self.stop()
            return False
        else:
            self.start()
            return True
