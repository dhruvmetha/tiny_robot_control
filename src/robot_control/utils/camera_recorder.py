"""Camera frame recorder - subscribes to camera topic and saves to video."""

from __future__ import annotations

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

    Subscribes to Topics.CAMERA_FRAME and records when active.

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
        codec: str = "avc1",
    ):
        """
        Initialize camera recorder.

        Args:
            output_dir: Directory to save recordings
            fps: Frames per second for output video
            codec: FourCC codec (default: avc1 for H.264 MP4)
        """
        self._output_dir = Path(output_dir)
        self._fps = fps
        self._codec = codec

        self._writer: Optional[cv2.VideoWriter] = None
        self._output_path: Optional[Path] = None
        self._frame_count = 0
        self._is_recording = False
        self._subscribed = False

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
            pub.subscribe(self._on_frame, Topics.CAMERA_FRAME)
            self._subscribed = True
            print("[CameraRecorder] Subscribed to camera frames")

    def unsubscribe(self) -> None:
        """Unsubscribe from camera frames."""
        if self._subscribed:
            # Stop recording if active
            if self._is_recording:
                self.stop()
            pub.unsubscribe(self._on_frame, Topics.CAMERA_FRAME)
            self._subscribed = False
            print("[CameraRecorder] Unsubscribed")

    def _on_frame(self, frame: np.ndarray, timestamp: float) -> None:
        """Handle incoming camera frame."""
        if not self._is_recording:
            return

        if frame is None:
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

        # Write frame
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
        if self._is_recording:
            print("[CameraRecorder] Already recording, stopping previous...")
            self.stop()

        # Create output directory
        self._output_dir.mkdir(parents=True, exist_ok=True)

        # Generate filename
        if filename is None:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = timestamp

        # Use .avi for XVID, .mp4 for H.264/other codecs
        ext = ".avi" if self._codec.upper() == "XVID" else ".mp4"
        self._output_path = self._output_dir / f"{filename}{ext}"
        self._frame_count = 0
        self._is_recording = True

        # Writer will be initialized on first frame (need frame size)
        self._writer = None

        print(f"[CameraRecorder] Recording started: {self._output_path}")
        return str(self._output_path)

    def stop(self) -> Optional[str]:
        """
        Stop recording and finalize video file.

        Returns:
            Path to the saved video file, or None if not recording
        """
        if not self._is_recording:
            return None

        output_path = self._output_path

        # Release writer
        if self._writer is not None:
            self._writer.release()
            self._writer = None

        self._is_recording = False

        if self._frame_count > 0:
            duration = self._frame_count / self._fps
            print(f"[CameraRecorder] Recording saved: {output_path}")
            print(f"  Frames: {self._frame_count}, Duration: {duration:.1f}s")
        else:
            # Remove empty file
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
