#!/usr/bin/env python3
"""Render a recorded navigation trajectory as an annotated MuJoCo MP4."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import mujoco
import numpy as np


WIDTH = 720
HEIGHT = 1080
FPS = 30


def yaw_quaternion(yaw_rad: float) -> np.ndarray:
    return np.array(
        [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)],
        dtype=float,
    )


def quaternion_yaw(quaternion: np.ndarray) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_error_deg(actual: float, expected: float) -> float:
    return abs((actual - expected + 180.0) % 360.0 - 180.0)


def load_model(xml_path: Path) -> mujoco.MjModel:
    root = ET.fromstring(xml_path.read_text(encoding="utf-8"))
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", str(WIDTH))
    global_visual.set("offheight", str(HEIGHT))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def body_qpos_address(model: mujoco.MjModel, body_name: str) -> tuple[int, int]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"missing body {body_name!r}")
    joint_id = int(model.body_jntadr[body_id])
    if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError(f"body {body_name!r} does not have a free joint")
    return body_id, int(model.jnt_qposadr[joint_id])


def set_free_body_pose(
    qpos: np.ndarray,
    address: int,
    x_m: float,
    y_m: float,
    z_m: float,
    yaw_rad: float,
) -> None:
    qpos[address : address + 3] = (x_m, y_m, z_m)
    qpos[address + 3 : address + 7] = yaw_quaternion(yaw_rad)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rows = [
        json.loads(line)
        for line in args.trajectory.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not rows:
        raise ValueError("trajectory contains no frames")

    model = load_model(args.xml)
    data = mujoco.MjData(model)
    initial_qpos = model.qpos0.copy()
    car_body_id, car_qpos_address = body_qpos_address(model, "car")
    car_z_m = float(initial_qpos[car_qpos_address + 2])

    first_objects = rows[0]["objects"]
    movable_names = sorted(
        name for name, pose in first_objects.items() if not pose["is_static"]
    )
    movable = {}
    for name in movable_names:
        body_id, qpos_address = body_qpos_address(model, name)
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise ValueError(f"missing geom {name!r}")
        movable[name] = {
            "body_id": body_id,
            "qpos_address": qpos_address,
            "geom_id": geom_id,
            "local_position": model.geom_pos[geom_id].copy(),
            "local_yaw": quaternion_yaw(model.geom_quat[geom_id]),
        }

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.245, 0.3875, 0.0]
    camera.distance = 1.02
    camera.azimuth = 90.0
    camera.elevation = -90.0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    intermediate = args.output.with_name(f"{args.output.stem}.mp4v{args.output.suffix}")
    writer = cv2.VideoWriter(
        str(intermediate), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer for {intermediate}")

    initial_positions = {
        name: np.array(
            [first_objects[name]["x_cm"], first_objects[name]["y_cm"]], dtype=float
        )
        for name in movable_names
    }
    max_position_error_cm = 0.0
    max_angle_error_deg = 0.0

    try:
        for row in rows:
            data.qpos[:] = initial_qpos
            robot = row["robot"]
            set_free_body_pose(
                data.qpos,
                car_qpos_address,
                robot["x_cm"] / 100.0,
                robot["y_cm"] / 100.0,
                car_z_m,
                math.radians(robot["theta_deg"]),
            )

            for name, info in movable.items():
                pose = row["objects"][name]
                target_yaw = math.radians(pose["theta_deg"])
                body_yaw = target_yaw - info["local_yaw"]
                local = info["local_position"]
                cosine, sine = math.cos(body_yaw), math.sin(body_yaw)
                rotated_x = cosine * local[0] - sine * local[1]
                rotated_y = sine * local[0] + cosine * local[1]
                body_x = pose["x_cm"] / 100.0 - rotated_x
                body_y = pose["y_cm"] / 100.0 - rotated_y
                body_z = -float(local[2]) + float(local[2])
                set_free_body_pose(
                    data.qpos,
                    info["qpos_address"],
                    body_x,
                    body_y,
                    body_z,
                    body_yaw,
                )

            mujoco.mj_forward(model, data)

            actual_car = data.xpos[car_body_id]
            max_position_error_cm = max(
                max_position_error_cm,
                100.0
                * math.hypot(
                    float(actual_car[0]) - robot["x_cm"] / 100.0,
                    float(actual_car[1]) - robot["y_cm"] / 100.0,
                ),
            )
            car_matrix = data.xmat[car_body_id].reshape(3, 3)
            max_angle_error_deg = max(
                max_angle_error_deg,
                angle_error_deg(
                    math.degrees(math.atan2(car_matrix[1, 0], car_matrix[0, 0])),
                    robot["theta_deg"],
                ),
            )

            for name, info in movable.items():
                pose = row["objects"][name]
                actual = data.geom_xpos[info["geom_id"]]
                max_position_error_cm = max(
                    max_position_error_cm,
                    100.0
                    * math.hypot(
                        float(actual[0]) - pose["x_cm"] / 100.0,
                        float(actual[1]) - pose["y_cm"] / 100.0,
                    ),
                )
                matrix = data.geom_xmat[info["geom_id"]].reshape(3, 3)
                max_angle_error_deg = max(
                    max_angle_error_deg,
                    angle_error_deg(
                        math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])),
                        pose["theta_deg"],
                    ),
                )

            renderer.update_scene(data, camera)
            frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (WIDTH, 100), (0, 0, 0), thickness=-1)
            cv2.addWeighted(overlay, 0.62, frame, 0.38, 0.0, frame)
            cv2.putText(
                frame,
                "rb_00022 hard_loose  |  nav_baseline_penalise  |  speed 0.3",
                (18, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"sim {row['simulation_time_s']:5.1f}s  |  {row['controller_state']}",
                (18, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            moved = []
            for name in movable_names:
                pose = row["objects"][name]
                current = np.array([pose["x_cm"], pose["y_cm"]], dtype=float)
                label = name.removeprefix("obstacle_").removesuffix("_movable")
                moved.append(
                    f"obj{label}:{np.linalg.norm(current - initial_positions[name]):.1f}cm"
                )
            cv2.putText(
                frame,
                "object displacement  " + "  ".join(moved),
                (18, 86),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (150, 230, 255),
                1,
                cv2.LINE_AA,
            )
            writer.write(frame)
    finally:
        writer.release()
        renderer.close()

    if max_position_error_cm > 1e-6 or max_angle_error_deg > 1e-6:
        raise RuntimeError(
            "rendered poses diverged from trajectory: "
            f"position={max_position_error_cm:.9g}cm angle={max_angle_error_deg:.9g}deg"
        )

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        subprocess.run(
            [
                ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-i",
                str(intermediate),
                "-c:v",
                "libx264",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(args.output),
            ],
            check=True,
        )
        intermediate.unlink()
    else:
        os.replace(intermediate, args.output)

    print(
        json.dumps(
            {
                "frames": len(rows),
                "fps": FPS,
                "duration_s": len(rows) / FPS,
                "max_pose_position_error_cm": max_position_error_cm,
                "max_pose_angle_error_deg": max_angle_error_deg,
                "output": str(args.output.resolve()),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
