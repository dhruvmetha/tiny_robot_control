"""ArUco-based pose observer that processes camera frames.

Subscribes to camera frames from CameraSensorNode and estimates poses.
Publishes Observation to Topics.SENSOR_VISION.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml
from pubsub import pub

from robot_control.camera.workspace import (
    MARKER_SIZE_CM,
    WORKSPACE_HEIGHT_CM,
    WORKSPACE_MARKERS,
    WORKSPACE_WIDTH_CM,
)
from robot_control.core.topics import Topics
from robot_control.core.types import ObjectPose, Observation


@dataclass
class ObserverConfig:
    """Configuration for ArucoObserver."""

    # Calibration file (camera intrinsics)
    calibration_file: str = ""
    undistort: bool = False

    # Robot marker (4x4 ArUco)
    robot_marker_id: int = 1
    robot_marker_size_mm: float = 36.0
    marker_to_wheel_offset_cm: Tuple[float, float] = (0.0, 0.0)

    # Object markers (6x6 ArUco)
    # Maps object_name -> (marker_id, width_mm, height_mm)
    objects: Dict[str, Tuple[int, float, float]] = field(default_factory=dict)
    object_marker_size_mm: float = 30.0

    # Workspace pose estimation
    warmup_frames: int = 30
    min_workspace_inliers: int = 12
    reproj_error_px: float = 5.0

    # Visualization
    draw_detections: bool = True
    draw_axis: bool = True
    axis_length_m: float = 0.03


class ArucoObserver:
    """
    ArUco-based pose observer.

    Subscribes to camera frames and estimates robot/object poses.
    Publishes Observation to Topics.SENSOR_VISION.
    Also publishes annotated frames to Topics.CAMERA_FRAME (with detections drawn).

    Usage:
        observer = ArucoObserver(config)
        observer.start()  # Subscribes to Topics.CAMERA_FRAME
        # ... frames are processed automatically
        observer.stop()
    """

    def __init__(self, config: ObserverConfig) -> None:
        self._config = config
        self._running = False

        # Calibration
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None

        # ArUco detectors
        self._ws_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._robot_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._object_detector: Optional[cv2.aruco.ArucoDetector] = None

        # Marker geometry
        self._robot_marker_len_m = config.robot_marker_size_mm / 1000.0
        self._object_marker_len_m = config.object_marker_size_mm / 1000.0
        self._robot_marker_pts_m: Optional[np.ndarray] = None
        self._object_marker_pts_m: Optional[np.ndarray] = None

        # Workspace pose (camera -> workspace transform)
        self._rvec_ws: Optional[np.ndarray] = None
        self._tvec_ws: Optional[np.ndarray] = None
        self._R_ws_cam: Optional[np.ndarray] = None
        self._t_ws_cam: Optional[np.ndarray] = None
        self._ws_fixed = False

        # Warmup state
        self._warmup_count = 0
        self._warmup_best_inliers = -1
        self._warmup_done = False

        # Latest data (thread-safe)
        self._observation: Optional[Observation] = None
        self._vis_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()

        # Build reverse lookup: marker_id -> object_name
        self._marker_to_object: Dict[int, str] = {}
        for name, (marker_id, _, _) in config.objects.items():
            self._marker_to_object[marker_id] = name

    def start(self) -> bool:
        """Start the observer. Subscribes to camera frames."""
        if self._running:
            return True

        # Load calibration
        if self._config.calibration_file:
            try:
                self._K, self._D = self._load_calibration(self._config.calibration_file)
            except Exception as e:
                print(f"[ArucoObserver] Failed to load calibration: {e}")
                return False
        else:
            # Default calibration for 720p
            self._K = np.array([
                [800, 0, 640],
                [0, 800, 360],
                [0, 0, 1]
            ], dtype=np.float32)
            self._D = np.zeros(5, dtype=np.float32)

        # Setup detectors
        self._setup_detectors()
        self._robot_marker_pts_m = self._marker_corners_local(self._robot_marker_len_m)
        self._object_marker_pts_m = self._marker_corners_local(self._object_marker_len_m)

        # Reset warmup
        self._warmup_count = 0
        self._warmup_best_inliers = -1
        self._warmup_done = self._config.warmup_frames <= 0

        # Subscribe to processed BGR frames
        self._running = True
        pub.subscribe(self._on_frame, Topics.CAMERA_BGR_PROCESSED)

        print("[ArucoObserver] Started")
        return True

    def stop(self) -> None:
        """Stop the observer."""
        self._running = False
        try:
            pub.unsubscribe(self._on_frame, Topics.CAMERA_BGR_PROCESSED)
        except Exception:
            pass

    def get(self) -> Optional[Observation]:
        """Get latest observation (thread-safe)."""
        with self._lock:
            return self._observation

    def get_vis_frame(self) -> Optional[np.ndarray]:
        """Get latest visualization frame with detections drawn."""
        with self._lock:
            return self._vis_frame.copy() if self._vis_frame is not None else None

    def is_workspace_ready(self) -> bool:
        """Check if workspace pose is available."""
        return self._rvec_ws is not None

    @property
    def is_running(self) -> bool:
        return self._running

    # -------------------------
    # Frame Processing
    # -------------------------

    def _on_frame(self, frame: np.ndarray, timestamp: float) -> None:
        """Handle incoming camera frame (BGR, already undistorted)."""
        if not self._running:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Warmup phase: find best workspace pose
        if not self._warmup_done:
            self._do_warmup_step(gray)
            return

        # Update workspace pose if not fixed
        if not self._ws_fixed:
            rvec, tvec, inliers = self._estimate_workspace_pose(gray)
            if rvec is not None and inliers >= self._config.min_workspace_inliers:
                self._rvec_ws = rvec
                self._tvec_ws = tvec
                self._R_ws_cam, self._t_ws_cam = self._invert_rt(rvec, tvec)

        ws_ok = self._rvec_ws is not None

        # Prepare visualization frame (already BGR)
        vis = frame.copy() if self._config.draw_detections else None

        if ws_ok:
            # Draw workspace boundary
            if vis is not None:
                self._draw_workspace_boundary(vis)

            # Detect robot and objects
            robot_pose = self._detect_robot(gray, vis)
            objects = self._detect_objects(gray, vis)

            if robot_pose is not None:
                x_cm, y_cm, yaw_deg = robot_pose
                obs = Observation(
                    robot_x=x_cm,
                    robot_y=y_cm,
                    robot_theta=yaw_deg,
                    objects=objects,
                    timestamp=timestamp,
                )
                with self._lock:
                    self._observation = obs
                # Publish observation
                pub.sendMessage(Topics.SENSOR_VISION, obs=obs)

            # Add status text
            if vis is not None:
                with self._lock:
                    obs = self._observation
                if obs is not None:
                    status = f"Robot: OK | Objects: {len(obs.objects)}"
                else:
                    status = "Robot: N/A"
                self._put_text(vis, status, (10, 30))
        else:
            if vis is not None:
                self._put_text(vis, "Workspace not ready", (10, 30))

        # Store visualization frame
        if vis is not None:
            with self._lock:
                self._vis_frame = vis

    def _do_warmup_step(self, gray: np.ndarray) -> None:
        """Process one warmup frame."""
        rvec, tvec, inliers = self._estimate_workspace_pose(gray)
        if rvec is not None and inliers > self._warmup_best_inliers:
            self._warmup_best_inliers = inliers
            self._rvec_ws = rvec
            self._tvec_ws = tvec
            self._R_ws_cam, self._t_ws_cam = self._invert_rt(rvec, tvec)

        self._warmup_count += 1
        if self._warmup_count >= self._config.warmup_frames:
            self._warmup_done = True
            if self._rvec_ws is not None:
                self._ws_fixed = True
                print(f"[ArucoObserver] Warmup OK (inliers={self._warmup_best_inliers})")
            else:
                print("[ArucoObserver] Warmup failed, using dynamic mode")

    # -------------------------
    # Calibration
    # -------------------------

    def _load_calibration(self, file_path: str) -> Tuple[np.ndarray, np.ndarray]:
        with open(file_path, "r") as f:
            calib = yaml.safe_load(f)
        K = np.array(calib["camera_matrix"], dtype=np.float32)
        D = np.array(calib["distortion_coefficients"], dtype=np.float32)
        return K, D

    # -------------------------
    # ArUco Setup
    # -------------------------

    def _setup_detectors(self) -> None:
        aruco = cv2.aruco
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self._ws_detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_5X5_50), params
        )
        self._robot_detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50), params
        )
        self._object_detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_6X6_50), params
        )

    def _marker_corners_local(self, marker_len_m: float) -> np.ndarray:
        h = marker_len_m / 2.0
        return np.array([
            [-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]
        ], dtype=np.float32)

    # -------------------------
    # Workspace Pose
    # -------------------------

    def _marker_workspace_corners_cm(self, marker_id: int) -> Optional[np.ndarray]:
        if marker_id not in WORKSPACE_MARKERS:
            return None
        x, y = WORKSPACE_MARKERS[marker_id]
        s = MARKER_SIZE_CM
        return np.array([
            [x, y], [x, y + s], [x + s, y + s], [x + s, y]
        ], dtype=np.float32)

    def _build_workspace_correspondences(
        self, corners: List, ids: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        obj_pts, img_pts = [], []
        if ids is None:
            return np.zeros((0, 3), np.float32), np.zeros((0, 2), np.float32)

        for i, mid in enumerate(ids.flatten().tolist()):
            ws_corners_cm = self._marker_workspace_corners_cm(mid)
            if ws_corners_cm is None:
                continue
            ws_corners_m = np.hstack([ws_corners_cm / 100.0, np.zeros((4, 1), np.float32)])
            img_corners = corners[i].reshape(4, 2).astype(np.float32)
            obj_pts.append(ws_corners_m)
            img_pts.append(img_corners)

        if not obj_pts:
            return np.zeros((0, 3), np.float32), np.zeros((0, 2), np.float32)
        return np.vstack(obj_pts).astype(np.float32), np.vstack(img_pts).astype(np.float32)

    def _solve_workspace_pose(
        self, obj_pts: np.ndarray, img_pts: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        if obj_pts.shape[0] < 8:
            return None, None, 0

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            objectPoints=obj_pts,
            imagePoints=img_pts,
            cameraMatrix=self._K,
            distCoeffs=self._D,
            reprojectionError=self._config.reproj_error_px,
            confidence=0.999,
            iterationsCount=200,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or rvec is None or inliers is None:
            return None, None, 0
        return rvec.astype(np.float32), tvec.astype(np.float32), int(inliers.shape[0])

    def _invert_rt(self, rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        R, _ = cv2.Rodrigues(rvec)
        R_inv = R.T
        t_inv = -R_inv @ tvec.reshape(3, 1)
        return R_inv.astype(np.float32), t_inv.astype(np.float32)

    def _estimate_workspace_pose(self, gray: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        ws_corners, ws_ids, _ = self._ws_detector.detectMarkers(gray)
        if ws_ids is None or len(ws_ids) == 0:
            return None, None, 0
        obj_pts, img_pts = self._build_workspace_correspondences(ws_corners, ws_ids)
        return self._solve_workspace_pose(obj_pts, img_pts)

    # -------------------------
    # Detection
    # -------------------------

    def _transform_marker_to_workspace(
        self, rvec: np.ndarray, tvec: np.ndarray, marker_pts_m: np.ndarray
    ) -> np.ndarray:
        R_marker, _ = cv2.Rodrigues(rvec)
        corners_cam = (R_marker @ marker_pts_m.T + tvec.reshape(3, 1)).T
        corners_ws_m = (self._R_ws_cam @ corners_cam.T + self._t_ws_cam).T
        return (corners_ws_m * 100.0)[:, :2]

    def _compute_pose_from_corners(self, corners_cm: np.ndarray) -> Tuple[float, float, float]:
        center = corners_cm.mean(axis=0)
        v = 0.5 * (corners_cm[0] + corners_cm[1]) - 0.5 * (corners_cm[2] + corners_cm[3])
        yaw = float(np.degrees(np.arctan2(v[1], v[0])))
        return float(center[0]), float(center[1]), (yaw + 360.0) % 360.0

    def _detect_robot(self, gray: np.ndarray, vis: Optional[np.ndarray]) -> Optional[Tuple[float, float, float]]:
        corners, ids, _ = self._robot_detector.detectMarkers(gray)
        if ids is None:
            return None

        for i, mid in enumerate(ids.flatten().tolist()):
            if mid != self._config.robot_marker_id:
                continue

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], self._robot_marker_len_m, self._K, self._D
            )
            rvec, tvec = rvecs[0, 0, :], tvecs[0, 0, :]

            corners_cm = self._transform_marker_to_workspace(rvec, tvec, self._robot_marker_pts_m)
            x_cm, y_cm, yaw_deg = self._compute_pose_from_corners(corners_cm)

            # Apply offset
            offset_x, offset_y = self._config.marker_to_wheel_offset_cm
            if offset_x != 0.0 or offset_y != 0.0:
                yaw_rad = np.radians(yaw_deg)
                x_cm += offset_x * np.cos(yaw_rad) - offset_y * np.sin(yaw_rad)
                y_cm += offset_x * np.sin(yaw_rad) + offset_y * np.cos(yaw_rad)

            if vis is not None:
                if self._config.draw_axis:
                    cv2.drawFrameAxes(vis, self._K, self._D, rvec, tvec, self._config.axis_length_m)
                cv2.aruco.drawDetectedMarkers(vis, [corners[i]], np.array([[mid]]))
                center_px = corners[i].reshape(4, 2).mean(axis=0)
                self._put_text(vis, f"R: {x_cm:.0f},{y_cm:.0f}", (int(center_px[0]), int(center_px[1]) + 20))

            return (x_cm, y_cm, yaw_deg)
        return None

    def _detect_objects(self, gray: np.ndarray, vis: Optional[np.ndarray]) -> Dict[str, ObjectPose]:
        if not self._marker_to_object:
            return {}

        corners, ids, _ = self._object_detector.detectMarkers(gray)
        if ids is None:
            return {}

        objects: Dict[str, ObjectPose] = {}
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._object_marker_len_m, self._K, self._D
        )

        for i, mid in enumerate(ids.flatten().tolist()):
            if mid not in self._marker_to_object:
                continue

            name = self._marker_to_object[mid]
            rvec, tvec = rvecs[i, 0, :], tvecs[i, 0, :]

            corners_cm = self._transform_marker_to_workspace(rvec, tvec, self._object_marker_pts_m)
            x_cm, y_cm, yaw_deg = self._compute_pose_from_corners(corners_cm)

            _, width_mm, height_mm = self._config.objects[name]
            width_cm, height_cm = width_mm / 10.0, height_mm / 10.0

            if vis is not None:
                if self._config.draw_axis:
                    cv2.drawFrameAxes(vis, self._K, self._D, rvec, tvec, self._config.axis_length_m)
                cv2.aruco.drawDetectedMarkers(vis, [corners[i]], np.array([[mid]]))
                center_px = corners[i].reshape(4, 2).mean(axis=0)
                self._put_text(vis, f"{name}: {x_cm:.0f},{y_cm:.0f}", (int(center_px[0]), int(center_px[1]) + 20))

            objects[name] = ObjectPose(x=x_cm, y=y_cm, theta=yaw_deg, width=width_cm, height=height_cm)

        return objects

    # -------------------------
    # Drawing
    # -------------------------

    def _draw_workspace_boundary(self, frame: np.ndarray) -> None:
        boundary_cm = np.array([
            [0, 0, 0],
            [0, WORKSPACE_HEIGHT_CM, 0],
            [WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM, 0],
            [WORKSPACE_WIDTH_CM, 0, 0]
        ], dtype=np.float32)
        img_pts, _ = cv2.projectPoints(
            boundary_cm / 100.0, self._rvec_ws, self._tvec_ws, self._K, self._D
        )
        cv2.polylines(frame, [img_pts.astype(np.int32)], True, (0, 255, 255), 3)

    def _put_text(self, img: np.ndarray, text: str, org: Tuple[int, int], scale: float = 0.6) -> None:
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 2)
