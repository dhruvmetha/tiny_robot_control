"""Camera sensor node that captures and publishes frames.

Threading model:
- start(): Call from main thread
- _capture_loop(): Background thread (capture + publish)
- stop(): Call from main thread

Publishes to two topics:
- Topics.CAMERA_FRAME_RAW: Raw RGB frames (no processing)
- Topics.CAMERA_FRAME: Undistorted RGB frames (if calibration provided)
"""

from __future__ import annotations

import platform
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml
from pubsub import pub

from robot_control.core.topics import Topics


@dataclass
class CameraConfig:
    """Configuration for CameraSensorNode."""

    camera_device: int = 0
    resolution: str = "720p"  # 480p, 720p, 1080p
    fps: int = 60
    exposure: Optional[int] = -6  # Camera exposure (None = auto)
    calibration_file: str = ""  # Path to calibration YAML (for undistortion)


class CameraSensorNode:
    """
    Camera sensor node that captures frames and publishes them.

    Publishes to two topics:
        - Topics.CAMERA_FRAME_RAW: Raw RGB frames
        - Topics.CAMERA_FRAME: Undistorted RGB frames (same as raw if no calibration)

    Frame format:
        - frame: np.ndarray (RGB image, not BGR)
        - timestamp: float

    Usage:
        node = CameraSensorNode(config)
        node.start()
        # ... frames are published automatically
        node.stop()
    """

    def __init__(self, config: CameraConfig) -> None:
        self._config = config
        self._running = False

        # Camera
        self._cap: Optional[cv2.VideoCapture] = None

        # Calibration for undistortion
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None

        # Latest frame (thread-safe access)
        self._frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()

        # Thread
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """Start capturing. Returns True if successful."""
        if self._running:
            return True

        # Load calibration if provided
        if self._config.calibration_file:
            try:
                self._K, self._D = self._load_calibration(self._config.calibration_file)
                print(f"[CameraSensorNode] Loaded calibration from {self._config.calibration_file}")
            except Exception as e:
                print(f"[CameraSensorNode] Failed to load calibration: {e}")
                print("[CameraSensorNode] Continuing without undistortion")

        self._cap = self._open_camera()
        if self._cap is None or not self._cap.isOpened():
            print("[CameraSensorNode] Failed to open camera")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        undistort_status = "enabled" if self._K is not None else "disabled"
        print(f"[CameraSensorNode] Started (device={self._config.camera_device}, undistort={undistort_status})")
        return True

    def _load_calibration(self, file_path: str) -> Tuple[np.ndarray, np.ndarray]:
        """Load camera calibration from YAML file."""
        with open(file_path, "r") as f:
            calib = yaml.safe_load(f)
        K = np.array(calib["camera_matrix"], dtype=np.float32)
        D = np.array(calib["distortion_coefficients"], dtype=np.float32)
        return K, D

    def stop(self) -> None:
        """Stop capturing."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def get_frame(self) -> Optional[np.ndarray]:
        """Get latest frame (thread-safe)."""
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        """Open camera with configured settings."""
        resolutions = {
            "480p": (640, 480),
            "720p": (1280, 720),
            "1080p": (1920, 1080),
        }
        width, height = resolutions.get(self._config.resolution, (1280, 720))

        system = platform.system()
        if system == "Linux":
            cap = cv2.VideoCapture(self._config.camera_device, cv2.CAP_V4L2)
        elif system == "Windows":
            cap = cv2.VideoCapture(self._config.camera_device, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(self._config.camera_device)

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, self._config.fps)

        if self._config.exposure is not None:
            cap.set(cv2.CAP_PROP_EXPOSURE, self._config.exposure)

        return cap

    def _capture_loop(self) -> None:
        """Main capture loop (runs in background thread)."""
        while self._running:
            ret, frame_bgr_raw = self._cap.read()
            if not ret:
                time.sleep(0.005)
                continue

            timestamp = time.time()

            # Convert BGR to RGB
            frame_rgb_raw = cv2.cvtColor(frame_bgr_raw, cv2.COLOR_BGR2RGB)

            # Undistort BGR if calibration available
            if self._K is not None and self._D is not None:
                frame_bgr_processed = cv2.undistort(frame_bgr_raw, self._K, self._D)
            else:
                frame_bgr_processed = frame_bgr_raw

            # Store latest processed frame
            with self._frame_lock:
                self._frame = frame_bgr_processed

            # Publish all three topics
            pub.sendMessage(Topics.CAMERA_BGR_RAW, frame=frame_bgr_raw, timestamp=timestamp)
            pub.sendMessage(Topics.CAMERA_RGB_RAW, frame=frame_rgb_raw, timestamp=timestamp)
            pub.sendMessage(Topics.CAMERA_BGR_PROCESSED, frame=frame_bgr_processed, timestamp=timestamp)

    @property
    def is_running(self) -> bool:
        """Check if capture is running."""
        return self._running
