#!/usr/bin/env python3
"""Persistent camera service that publishes Observations over ZeroMQ.

Start once, leave running. Clients (e.g. run_namo.py --camera-service)
connect via ZMQ SUB and receive Observation objects instantly with no warmup.

Usage:
    # Basic
    python scripts/camera_service.py --config config/real.yaml

    # With local OpenCV monitoring window
    python scripts/camera_service.py --config config/real.yaml --show

    # Custom port
    python scripts/camera_service.py --config config/real.yaml --port 5557
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import zmq
from pubsub import pub

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robot_control.camera import ArucoObserver, ObserverConfig
from robot_control.camera.observer import ObjectDefinition
from robot_control.core.serialization import obs_to_bytes
from robot_control.core.topics import Topics
from robot_control.core.types import Observation
from robot_control.nodes import CameraConfig, CameraSensorNode


def load_camera_config(config_path: str, objects_path: str):
    """Load camera and observer configuration from YAML files."""
    import yaml

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
    objects_file = Path(objects_path)
    if objects_file.exists():
        with open(objects_file, "r") as f:
            obj_config = yaml.safe_load(f)
        for name, obj_cfg in obj_config.get("objects", {}).items():
            obj_type = obj_cfg.get("type", "movable")
            shape = obj_cfg.get("shape", {})
            offset = obj_cfg.get("marker_offset", {})
            # Camera only needs marker mapping + offsets, not physical sizes
            object_defs[name] = ObjectDefinition(
                marker_id=obj_cfg["marker_id"],
                is_goal=(obj_type == "goal"),
                marker_offset_x_cm=offset.get("x", 0.0),
                marker_offset_y_cm=offset.get("y", 0.0),
            )

    object_marker_size = config.get("object_marker_size_mm") or robot_cfg.get(
        "object_marker_size_mm", 30.0
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
        object_marker_size_mm=object_marker_size,
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    return camera_config, observer_config


def main():
    parser = argparse.ArgumentParser(
        description="Persistent camera service publishing Observations over ZeroMQ",
    )
    parser.add_argument(
        "--config", "-c", type=str, default="config/real.yaml",
        help="Path to real robot config YAML",
    )
    parser.add_argument(
        "--objects", type=str, default="config/objects.yaml",
        help="Path to objects definition file",
    )
    parser.add_argument(
        "--port", type=int, default=5556,
        help="ZMQ PUB port (default: 5556)",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Show local OpenCV monitoring window",
    )
    args = parser.parse_args()

    # Load config
    camera_config, observer_config = load_camera_config(args.config, args.objects)

    # Setup ZMQ PUB socket
    ctx = zmq.Context()
    socket = ctx.socket(zmq.PUB)
    bind_addr = f"tcp://*:{args.port}"
    socket.bind(bind_addr)
    print(f"[CameraService] ZMQ PUB bound to {bind_addr}")

    # Setup ZMQ REP socket for on-demand frame requests (diagnostics).
    # Clients send b"vis" or b"raw"; we respond with JPEG-encoded bytes.
    frame_socket = ctx.socket(zmq.REP)
    frame_bind_addr = f"tcp://*:{args.port + 1}"
    frame_socket.bind(frame_bind_addr)
    print(f"[CameraService] ZMQ REP (frame requests) bound to {frame_bind_addr}")

    # Create camera + observer (standard in-process PyPubSub pipeline)
    camera = CameraSensorNode(camera_config)
    observer = ArucoObserver(observer_config)

    # Stats
    obs_count = 0
    start_time = time.time()
    last_print_time = start_time
    last_print_count = 0

    # Subscribe to SENSOR_VISION and forward over ZMQ
    def on_observation(obs: Observation):
        nonlocal obs_count, last_print_time, last_print_count
        data = obs_to_bytes(obs)
        socket.send_multipart([b"obs", data])
        obs_count += 1

        # Print stats every 5 seconds
        now = time.time()
        dt = now - last_print_time
        if dt >= 5.0:
            rate = (obs_count - last_print_count) / dt
            n_objects = len(obs.objects)
            goal_str = (
                f"({obs.goal_x:.1f}, {obs.goal_y:.1f})"
                if obs.goal_x is not None
                else "N/A"
            )
            print(
                f"[CameraService] {obs_count} obs | {rate:.1f} Hz | "
                f"objects: {n_objects} | goal: {goal_str} | "
                f"robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f})"
            )
            last_print_time = now
            last_print_count = obs_count

    pub.subscribe(on_observation, Topics.SENSOR_VISION)

    # Graceful shutdown
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\n[CameraService] Shutting down...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start camera + observer
    print("[CameraService] Starting camera...")
    if not camera.start():
        print("[CameraService] ERROR: Failed to start camera!")
        socket.close(linger=0)
        ctx.term()
        return 1

    print("[CameraService] Starting ArUco observer...")
    if not observer.start():
        print("[CameraService] ERROR: Failed to start observer!")
        camera.stop()
        socket.close(linger=0)
        ctx.term()
        return 1

    # Background thread that serves on-demand frame requests over the REP
    # socket. We poll with a short timeout instead of blocking forever, so
    # signal-driven shutdown still works promptly.
    import cv2 as _cv2

    def _frame_request_loop() -> None:
        poller = zmq.Poller()
        poller.register(frame_socket, zmq.POLLIN)
        while running:
            try:
                events = dict(poller.poll(timeout=200))  # ms
            except zmq.ZMQError:
                # Context terminated under us during shutdown.
                break
            if frame_socket not in events:
                continue
            try:
                kind = frame_socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            except zmq.ZMQError:
                break
            try:
                kind_str = kind.decode("utf-8", errors="ignore").strip().lower() or "vis"
                if kind_str == "raw":
                    frame = camera.get_frame()
                else:
                    frame = observer.get_vis_frame()
                    if frame is None:
                        frame = camera.get_frame()
                if frame is None:
                    frame_socket.send(b"")  # Empty reply signals "no frame available"
                    continue
                ok, buf = _cv2.imencode(".jpg", frame,
                                        [int(_cv2.IMWRITE_JPEG_QUALITY), 92])
                payload = buf.tobytes() if ok else b""
                frame_socket.send(payload)
            except Exception as exc:
                # Ensure REP socket state machine stays balanced (must always
                # reply after a recv). Send empty bytes on any failure.
                try:
                    frame_socket.send(b"")
                except Exception:
                    pass
                print(f"[CameraService] frame request error: {exc!r}", flush=True)

    frame_thread = threading.Thread(
        target=_frame_request_loop, name="CameraService-FrameReq", daemon=True
    )
    frame_thread.start()

    elapsed_startup = time.time() - start_time
    print(f"[CameraService] Ready in {elapsed_startup:.1f}s")
    print(f"[CameraService] Clients connect to tcp://localhost:{args.port}")
    print(f"[CameraService] Frame requests: tcp://localhost:{args.port + 1} (REQ→REP)")
    if args.show:
        print("[CameraService] Showing monitoring window (press 'q' to quit)")

    # Main loop
    try:
        if args.show:
            import cv2

            cv2.namedWindow("Camera Service", cv2.WINDOW_NORMAL)
            while running:
                vis = observer.get_vis_frame()
                if vis is not None:
                    cv2.imshow("Camera Service", vis)
                key = cv2.waitKey(30) & 0xFF
                if key == ord("q") or key == 27:
                    running = False
            cv2.destroyAllWindows()
        else:
            while running:
                time.sleep(0.1)
    finally:
        # Cleanup
        pub.unsubscribe(on_observation, Topics.SENSOR_VISION)
        observer.stop()
        camera.stop()
        # Give the frame-request thread a moment to notice `running=False`
        # before tearing down its socket and the context.
        if frame_thread.is_alive():
            frame_thread.join(timeout=1.0)
        try:
            frame_socket.close(linger=0)
        except Exception:
            pass
        socket.close(linger=0)
        ctx.term()
        elapsed_total = time.time() - start_time
        rate = obs_count / elapsed_total if elapsed_total > 0 else 0
        print(
            f"[CameraService] Stopped. {obs_count} observations in "
            f"{elapsed_total:.1f}s ({rate:.1f} Hz avg)"
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
