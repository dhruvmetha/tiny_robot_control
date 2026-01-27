"""Test camera detection with ArUco markers.

This script tests the camera module without robot actuation.
Uses the new architecture:
  CameraSensorNode -> ArucoObserver -> WorldState

Usage:
    python scripts/test_camera.py
    python scripts/test_camera.py --config config/real.yaml

Controls:
    Q / Escape : Quit
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Dict, Tuple

import cv2
import yaml

from robot_control.camera import ArucoObserver, ObserverConfig
from robot_control.core.world_state import WorldState
from robot_control.nodes import CameraSensorNode, CameraConfig


def load_config(config_path: str) -> Tuple[CameraConfig, ObserverConfig, Dict]:
    """Load configuration from YAML file."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Build objects dict: name -> (marker_id, width, height)
    objects = {}
    for name, obj_config in config.get("objects", {}).items():
        marker_id = obj_config["marker_id"]
        size = obj_config.get("size_mm", [40, 40])
        objects[name] = (marker_id, size[0], size[1])

    camera_cfg = config.get("camera", {})
    robot_cfg = config.get("robot", {})
    workspace_cfg = config.get("workspace", {})

    # Parse marker offset
    marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
    if isinstance(marker_offset, list) and len(marker_offset) == 2:
        marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
    else:
        marker_offset_tuple = (0.0, 0.0)

    # Camera sensor config
    camera_config = CameraConfig(
        camera_device=camera_cfg.get("device", 0),
        resolution=camera_cfg.get("resolution", "720p"),
        fps=camera_cfg.get("fps", 60),
        exposure=camera_cfg.get("exposure", -6),
    )

    # ArUco observer config
    observer_config = ObserverConfig(
        calibration_file=camera_cfg.get("calibration_file", ""),
        undistort=camera_cfg.get("undistort", False),
        robot_marker_id=robot_cfg.get("marker_id", 1),
        robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
        marker_to_wheel_offset_cm=marker_offset_tuple,
        objects=objects,
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    return camera_config, observer_config, config


def main():
    parser = argparse.ArgumentParser(description="Test camera detection")
    parser.add_argument(
        "--config", "-c",
        type=str,
        default="config/real.yaml",
        help="Path to config file (default: config/real.yaml)"
    )
    args = parser.parse_args()

    # Load config
    config_path = Path(args.config)
    if config_path.exists():
        print(f"Loading config from {config_path}")
        camera_config, observer_config, raw_config = load_config(str(config_path))
    else:
        print(f"Config file not found: {config_path}, using defaults")
        camera_config = CameraConfig()
        observer_config = ObserverConfig()
        raw_config = {}

    # Create world state (subscribes to SENSOR_VISION)
    world = WorldState()

    # Create camera sensor node (captures frames, publishes to CAMERA_FRAME)
    print("\nStarting CameraSensorNode...")
    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("Failed to start camera!")
        return

    # Create ArUco observer (subscribes to CAMERA_FRAME, publishes to SENSOR_VISION)
    print("Starting ArucoObserver...")
    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("Failed to start observer!")
        camera.stop()
        return

    print("\n" + "=" * 50)
    print("Camera Detection Test")
    print("=" * 50)
    print(f"Robot marker ID: {observer_config.robot_marker_id}")
    print(f"Objects: {list(observer_config.objects.keys())}")
    print("=" * 50)
    print("\nPress 'q' or Escape to quit\n")

    # Create preview window
    window_name = "Camera Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            # Get visualization frame from observer
            vis_frame = observer.get_vis_frame()
            if vis_frame is not None:
                cv2.imshow(window_name, vis_frame)

            # Get observation from world state
            obs = world.get()

            # Print every frame
            if obs is None:
                print("\rNo observation available", end="", flush=True)
            else:
                print(f"\rRobot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ {obs.robot_theta:.1f}°  ", end="")
                if obs.objects:
                    obj_str = " | ".join([
                        f"{name}: ({o.x:.1f}, {o.y:.1f}, {o.theta:.1f}°)"
                        for name, o in obs.objects.items()
                    ])
                    print(f"Objects: {obj_str}  ", end="")
                print("", flush=True)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        observer.stop()
        camera.stop()
        world.unsubscribe()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
