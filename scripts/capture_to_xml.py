"""Capture ArUco markers and generate NAMO XML.

Captures current robot and object positions from camera,
then generates a MuJoCo XML for namo_cpp planning.

Usage:
    python scripts/capture_to_xml.py
    python scripts/capture_to_xml.py --output /tmp/env.xml
    python scripts/capture_to_xml.py --goal 30 50  # goal at (30cm, 50cm)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Dict

import cv2
import yaml

from robot_control.camera import ArucoObserver, ObserverConfig
from robot_control.camera.observer import ObjectDefinition
from robot_control.camera.workspace import WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM
from robot_control.nodes import CameraSensorNode, CameraConfig
from robot_control.utils import NAMOXMLGenerator


def load_objects_config(objects_path: str) -> Dict[str, ObjectDefinition]:
    """Load object definitions from objects.yaml."""
    with open(objects_path, "r") as f:
        config = yaml.safe_load(f)

    object_defs = {}
    for name, obj_cfg in config.get("objects", {}).items():
        obj_type = obj_cfg.get("type", "movable")
        shape = obj_cfg.get("shape", {})
        offset = obj_cfg.get("marker_offset", {})
        object_defs[name] = ObjectDefinition(
            marker_id=obj_cfg["marker_id"],
            is_static=(obj_type == "static"),
            is_goal=(obj_type == "goal"),
            width_cm=shape.get("width", 0.0),
            depth_cm=shape.get("depth", 0.0),
            height_cm=shape.get("height", 0.0),
            marker_offset_x_cm=offset.get("x", 0.0),
            marker_offset_y_cm=offset.get("y", 0.0),
        )
    return object_defs


def load_config(config_path: str, objects_path: str = None):
    """Load configuration from YAML files."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    camera_cfg = config.get("camera", {})
    robot_cfg = config.get("robot", {})
    workspace_cfg = config.get("workspace", {})

    # Parse marker offset
    marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
    if isinstance(marker_offset, list) and len(marker_offset) == 2:
        marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
    else:
        marker_offset_tuple = (0.0, 0.0)

    # Load object definitions
    object_defs = {}
    if objects_path and Path(objects_path).exists():
        object_defs = load_objects_config(objects_path)
    else:
        # Fallback to legacy format in real.yaml
        for name, obj_config in config.get("objects", {}).items():
            marker_id = obj_config["marker_id"]
            size = obj_config.get("size_mm", [50, 50])
            object_defs[name] = ObjectDefinition(
                marker_id=marker_id,
                is_static=False,
                width_cm=size[0] / 10.0,
                depth_cm=size[1] / 10.0,
                height_cm=5.0,
            )

    camera_config = CameraConfig(
        camera_device=camera_cfg.get("device", 0),
        resolution=camera_cfg.get("resolution", "720p"),
        fps=camera_cfg.get("fps", 60),
        exposure=camera_cfg.get("exposure", -6),
        calibration_file=camera_cfg.get("calibration_file", ""),
    )

    observer_config = ObserverConfig(
        calibration_file=camera_cfg.get("calibration_file", ""),
        robot_marker_id=robot_cfg.get("marker_id", 1),
        robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
        marker_to_wheel_offset_cm=marker_offset_tuple,
        object_defs=object_defs,
        object_marker_size_mm=robot_cfg.get("object_marker_size_mm", 30.0),
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    return camera_config, observer_config


def main():
    parser = argparse.ArgumentParser(description="Capture ArUco markers to NAMO XML")
    parser.add_argument(
        "--config", "-c",
        type=str,
        default="config/real.yaml",
        help="Path to config file"
    )
    parser.add_argument(
        "--objects",
        type=str,
        default="config/objects.yaml",
        help="Path to objects definition file"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="env_from_camera.xml",
        help="Output XML file path"
    )
    parser.add_argument(
        "--goal", "-g",
        type=float,
        nargs=2,
        default=None,
        help="Goal position in cm: --goal X Y"
    )
    parser.add_argument(
        "--stable-frames",
        type=int,
        default=10,
        help="Number of stable frames before capture"
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Auto-capture after stable frames (no GUI interaction)"
    )
    parser.add_argument(
        "--scale-factor", "-s",
        type=float,
        default=6.0,
        help="Scale factor for simulation (6.0 = real 2.5cm robot -> sim 15cm robot)"
    )
    args = parser.parse_args()

    # Load config
    config_path = Path(args.config)
    objects_path = Path(args.objects)
    if config_path.exists():
        print(f"Loading config from {config_path}")
        if objects_path.exists():
            print(f"Loading objects from {objects_path}")
        camera_config, observer_config = load_config(str(config_path), str(objects_path))
    else:
        print(f"Config not found: {config_path}, using defaults")
        camera_config = CameraConfig()
        observer_config = ObserverConfig()

    # Start camera
    print("\nStarting camera...")
    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("Failed to start camera!")
        return

    # Start observer
    print("Starting ArUco observer...")
    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("Failed to start observer!")
        camera.stop()
        return

    print("\n" + "=" * 50)
    print("Waiting for stable detection...")
    print("Press 'c' to capture, 'q' to quit")
    print("=" * 50)

    cv2.namedWindow("Capture", cv2.WINDOW_NORMAL)

    stable_count = 0
    last_obs = None
    captured = False

    try:
        while not captured:
            vis = observer.get_vis_frame()
            if vis is not None:
                # Add instructions
                cv2.putText(vis, "Press 'c' to capture, 'q' to quit",
                           (10, vis.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Capture", vis)

            obs = observer.get()
            if obs is not None:
                # Check stability
                if last_obs is not None:
                    dx = abs(obs.robot_x - last_obs.robot_x)
                    dy = abs(obs.robot_y - last_obs.robot_y)
                    if dx < 1.0 and dy < 1.0:  # Less than 1cm movement
                        stable_count += 1
                    else:
                        stable_count = 0
                last_obs = obs

                goal_str = f"({obs.goal_x:.1f}, {obs.goal_y:.1f})" if obs.goal_x is not None else "N/A"
                print(f"\rRobot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ {obs.robot_theta:.1f}° | "
                      f"Objects: {len(obs.objects)} | Goal: {goal_str} | Stable: {stable_count}/{args.stable_frames}  ",
                      end="", flush=True)

            # Auto-capture mode
            if args.auto and stable_count >= args.stable_frames:
                captured = True
                print("\n\nAuto-capturing...")
                break

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                print("\n\nCancelled.")
                break
            elif key == ord('c') and obs is not None:
                captured = True
                print("\n\nCapturing...")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nInterrupted.")

    finally:
        cv2.destroyAllWindows()
        observer.stop()
        camera.stop()

    if not captured or last_obs is None:
        print("No valid observation captured.")
        return

    # Generate XML
    print("\nGenerating XML...")

    # Determine goal (priority: CLI arg > detected marker > auto)
    if args.goal:
        goal_x, goal_y = args.goal
        print(f"Goal from CLI: ({goal_x:.1f}, {goal_y:.1f}) cm")
    elif last_obs.goal_x is not None and last_obs.goal_y is not None:
        goal_x, goal_y = last_obs.goal_x, last_obs.goal_y
        print(f"Goal from marker 0: ({goal_x:.1f}, {goal_y:.1f}) cm")
    else:
        # Default: opposite corner from robot
        if last_obs.robot_x < WORKSPACE_WIDTH_CM / 2:
            goal_x = WORKSPACE_WIDTH_CM - 5
        else:
            goal_x = 5
        if last_obs.robot_y < WORKSPACE_HEIGHT_CM / 2:
            goal_y = WORKSPACE_HEIGHT_CM - 5
        else:
            goal_y = 5
        print(f"Auto goal (no marker 0 detected): ({goal_x:.1f}, {goal_y:.1f}) cm")

    # Build objects dict for generator
    objects = {}
    for name, obj in last_obs.objects.items():
        # (x_cm, y_cm, theta_deg, width_cm, depth_cm, height_cm, is_static)
        objects[name] = (obj.x, obj.y, obj.theta, obj.width, obj.depth, obj.height, obj.is_static)

    generator = NAMOXMLGenerator(scale_factor=args.scale_factor)
    xml_str = generator.from_observation(
        robot_x_cm=last_obs.robot_x,
        robot_y_cm=last_obs.robot_y,
        objects=objects,
        goal_x_cm=goal_x,
        goal_y_cm=goal_y,
        workspace_bounds_cm=(0.0, WORKSPACE_WIDTH_CM, 0.0, WORKSPACE_HEIGHT_CM),
    )

    if args.scale_factor != 1.0:
        print(f"\nScale factor: {args.scale_factor}x (real -> simulation)")
        print(f"  Robot radius: {generator.ROBOT_RADIUS_BASE*100:.1f}cm -> {generator.ROBOT_RADIUS*100:.1f}cm")

    # Save XML
    output_path = Path(args.output)
    generator.save(xml_str, str(output_path))

    # Save wavefront grid (use resolved robot position after collision resolution)
    wavefront_path = output_path.with_suffix(".png")
    resolved_pos = generator.get_resolved_robot_pos()
    if resolved_pos is not None:
        robot_pos_m = resolved_pos  # Already in scaled meters
    else:
        robot_pos_m = (last_obs.robot_x / 100.0 * args.scale_factor,
                       last_obs.robot_y / 100.0 * args.scale_factor)
    generator.save_wavefront(str(wavefront_path), robot_pos_m)

    print(f"\nSaved: {output_path}")
    print("\n" + "=" * 50)
    print("Captured configuration:")
    print("=" * 50)
    print(f"Robot: ({last_obs.robot_x:.2f}, {last_obs.robot_y:.2f}) cm @ {last_obs.robot_theta:.1f}°")
    goal_source = "marker 0" if (last_obs.goal_x is not None and not args.goal) else ("CLI" if args.goal else "auto")
    print(f"Goal:  ({goal_x:.2f}, {goal_y:.2f}) cm [from {goal_source}]")
    print(f"Objects ({len(objects)}):")
    for name, (x, y, theta, w, d, h, is_static) in objects.items():
        type_str = "[STATIC]" if is_static else "[MOVABLE]"
        print(f"  {type_str} {name}: ({x:.2f}, {y:.2f}) cm @ {theta:.1f}°, size: {w:.1f}x{d:.1f}x{h:.1f} cm")
    print("=" * 50)


if __name__ == "__main__":
    main()
