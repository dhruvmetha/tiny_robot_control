"""MuJoCo-backed simulation environment.

Drop-in replacement for ``SimEnv`` (which uses micromvp's diff-drive kinematics
only — no contact physics, so the robot drives through obstacles). This one
runs full MuJoCo physics per tick, so pushes actually move objects.

Why this exists: ``RLEnvironment.step()`` in the namo_rl C++ binding is atomic
— it runs the full C++ ``NAMOPushController`` whose push phase is open-loop
motion-primitive replay. To get *pure-pursuit* push behaviour (matching
``run_namo.py`` and ``collect_real_primitives.py``) plus *MuJoCo contact
physics*, we need a Python loop that:

    1. Sets wheel velocities from Action.left_speed/right_speed
    2. Steps physics one tick
    3. Reads the resulting robot + object poses

The C++ bindings don't expose the low-level pieces (no apply_wheel_control or
mj_step), so we open the same XML directly via the mujoco python package and
do this loop ourselves.

XML expectations (car model, as produced by ``capture_to_xml.py --robot-model
car``):
    - actuators: ``left_wheel_drive`` + ``right_wheel_drive`` (velocity)
    - robot freejoint: ``car_freejoint``
    - each movable obstacle is a body with a freejoint and one geom carrying
      its dimensions
    - workspace walls are inside a body whose name starts with ``walls`` and
      have no freejoint (treated as static)
"""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Lock, Thread
from typing import Dict, Optional

import mujoco
import numpy as np

from robot_control.core.types import Action, ObjectPose, Observation
from robot_control.environment.base import Environment


# Tunable knobs ---------------------------------------------------------------

# Maximum chassis linear speed at PWM = 1.0, in m/s. Matches
# ``kCarWheelMaxSpeedMs`` in namo_push_controller.cpp so PWM units map 1:1
# between sim and real.
_PWM_TO_LINEAR_M_S = 1.0

# Recording defaults. Match sim_replay_subprocess so the output looks the
# same as the existing replay MP4s.
_VIDEO_WIDTH = 1280
_VIDEO_HEIGHT = 720
_VIDEO_FPS = 30
_CAMERA_DISTANCE_FACTOR = 1.6


def _quat_to_yaw_deg(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract yaw (rotation around +Z) from a MuJoCo unit quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


class MujocoSimEnv(Environment):
    """Single-robot MuJoCo simulation matching the SimEnv API.

    Threading: physics runs on its own thread at the rate set by the loaded
    XML's ``<option timestep=...>``. ``apply()`` stashes the latest Action
    (thread-safe). ``observe()`` snapshots state (thread-safe).
    """

    def __init__(
        self,
        xml_path: str,
        *,
        viewer: bool = False,
        speed_scale: float = 1.0,
        starting_robot_pose_cm: Optional[tuple] = None,
        record_video_path: Optional[str] = None,
    ) -> None:
        """
        Args:
            xml_path: Path to the MuJoCo XML (car-format from capture_to_xml).
            viewer: If True, spawn a passive ``mujoco.viewer`` window.
            speed_scale: Multiplier on wheel angular velocity (1.0 = nominal).
            starting_robot_pose_cm: ``(x_cm, y_cm, theta_deg)``. Teleports the
                car to this pose before any physics runs. Required for car
                XMLs because ``little_car.xml`` bakes a spawn pose of (0,0)
                that wouldn't match the captured scene.
            record_video_path: If set, write an MP4 of the run to this path.
                Frames are captured from the physics thread at 30 FPS using
                the same top-down camera as sim_replay_subprocess so the
                video looks comparable. None = no recording.
        """
        self._xml_path = xml_path
        self._speed_scale = float(speed_scale)
        self._want_viewer = viewer
        self._record_path = record_video_path

        # Recording requires the model to allow offscreen rendering at the
        # video dimensions. The XML's <visual><global offwidth=.. offheight=..>
        # controls this; sim_replay_subprocess injects it on the fly and we
        # do the same. Done before MjModel.from_xml_string so the model knows
        # the buffer size at load time.
        xml_str = Path(xml_path).read_text()
        if self._record_path:
            xml_str = _inject_offscreen_size(xml_str, _VIDEO_WIDTH, _VIDEO_HEIGHT)
            self._model = mujoco.MjModel.from_xml_string(xml_str)
        else:
            self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._data = mujoco.MjData(self._model)

        # Resolve actuator + joint indices once. mj_name2id returns -1 if the
        # name is missing; we fail loud so users notice the XML doesn't match
        # the car convention rather than getting silently zero motion.
        self._left_act = self._must_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_drive")
        self._right_act = self._must_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_drive")
        self._car_jntadr = self._jnt_qposadr_by_name("car_freejoint")

        # Wheel radius — pulled from the left wheel cylinder geom so PWM →
        # rad/s uses the model's actual wheel size rather than a hard-coded
        # value (which would silently desync if the XML wheel radius changes).
        left_wheel_geom = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_collision"
        )
        if left_wheel_geom < 0:
            self._wheel_radius = 0.015  # fallback matches DiffDriveAdapter.cpp default
        else:
            self._wheel_radius = float(self._model.geom_size[left_wheel_geom, 0])

        # Movable obstacles: every body with a freejoint other than the car.
        # We record per-body: body_id, primary geom_id (for world-pose
        # readout — capture_to_xml.py puts the obstacle's pose on the geom,
        # not the body, so body_xpos stays at the origin and we have to use
        # geom_xpos instead), and width/depth/height in cm.
        self._movables: Dict[str, Dict] = {}
        self._scan_bodies()

        # Car body id — we read body_xpos[car_body] for the robot pose
        # because little_car.xml puts the chassis pose on <body name="car">
        # (qpos is the freejoint delta on top of that).
        self._car_body_id = self._must_id(mujoco.mjtObj.mjOBJ_BODY, "car")

        # Action shared between control thread (writer) and physics thread
        # (reader). Lock-guarded; the values are tiny so contention is fine.
        self._action_lock = Lock()
        self._action = Action(left_speed=0.0, right_speed=0.0)

        # State snapshot lock for observe() consistency.
        self._state_lock = Lock()

        self._running = False
        self._thread: Optional[Thread] = None
        self._viewer = None
        self._physics_dt = float(self._model.opt.timestep)

        # Recording state. We DO NOT render frames in the physics thread —
        # mujoco.Renderer needs an OpenGL context tied to a single thread
        # and rendering from a background thread either yields black frames
        # or crashes (depends on the backend). Mirror sim_replay_subprocess:
        # capture per-tick qpos here; render the MP4 from the main thread
        # in stop() once the physics thread has joined.
        self._video_qpos_frames: list = []
        self._video_stride = 1
        self._video_tick_count = 0

        # Teleport the car to the captured pose if one was supplied. We write
        # directly into qpos for the car_freejoint and let mj_forward
        # propagate it through xpos/xmat. Done BEFORE mj_forward so the
        # initial observe() returns the live pose, not (0, 0, 0).
        if starting_robot_pose_cm is not None:
            self.set_robot_pose(*starting_robot_pose_cm)

        # mj_forward populates xpos/xmat from qpos so observe() works before
        # the physics thread has fired even once.
        mujoco.mj_forward(self._model, self._data)

    def set_robot_pose(self, x_cm: float, y_cm: float, theta_deg: float) -> None:
        """Teleport the car to the given pose. Safe to call any time after
        construction (the state lock protects against the physics thread)."""
        a = self._car_jntadr
        theta_rad = math.radians(float(theta_deg))
        car_jnt_id = self._must_id(mujoco.mjtObj.mjOBJ_JOINT, "car_freejoint")
        jntdofadr = int(self._model.jnt_dofadr[car_jnt_id])
        with self._state_lock:
            # Preserve z so the chassis stays on its baked spawn height.
            z = float(self._data.qpos[a + 2])
            self._data.qpos[a + 0] = float(x_cm) / 100.0
            self._data.qpos[a + 1] = float(y_cm) / 100.0
            self._data.qpos[a + 2] = z
            self._data.qpos[a + 3] = math.cos(theta_rad / 2.0)
            self._data.qpos[a + 4] = 0.0
            self._data.qpos[a + 5] = 0.0
            self._data.qpos[a + 6] = math.sin(theta_rad / 2.0)
            # Zero the freejoint velocity so the teleport doesn't inject a
            # transient.
            for i in range(6):
                self._data.qvel[jntdofadr + i] = 0.0
            mujoco.mj_forward(self._model, self._data)

    # ------------------------------------------------------------ resolution

    def _must_id(self, objtype, name: str) -> int:
        idx = mujoco.mj_name2id(self._model, objtype, name)
        if idx < 0:
            raise ValueError(
                f"MujocoSimEnv: required name {name!r} not found in {self._xml_path}"
            )
        return idx

    def _jnt_qposadr_by_name(self, name: str) -> int:
        jnt = self._must_id(mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self._model.jnt_qposadr[jnt])

    def _scan_bodies(self) -> None:
        """Walk all bodies, categorise into movable obstacles and statics.

        A body is movable iff its first joint is a freejoint and it isn't the
        car. We index by body name; the freejoint's qpos address is what we
        read every observe() tick.
        """
        for bid in range(self._model.nbody):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if not name or name == "car":
                continue
            jntadr = int(self._model.body_jntadr[bid])
            jntnum = int(self._model.body_jntnum[bid])
            if jntnum == 0:
                # No joint = welded to world. Walls live as sub-geoms of a
                # static parent and we don't need to report them as objects.
                continue
            first_jnt_type = int(self._model.jnt_type[jntadr])
            if first_jnt_type != mujoco.mjtJoint.mjJNT_FREE:
                continue
            # Primary geom for dimensions AND world-pose: first geom attached
            # to this body. NAMOXMLGenerator emits one geom per obstacle body
            # with the world pose written on the geom (not the body), so
            # geom_xpos/geom_xmat after mj_forward give the obstacle's world
            # pose.
            geomadr = int(self._model.body_geomadr[bid])
            geomnum = int(self._model.body_geomnum[bid])
            if geomnum <= 0:
                continue
            # Half-extents in metres (box: [hx, hy, hz]). robot_control's
            # ObjectPose convention is depth=X-extent, width=Y-extent (see
            # controller/edge_points.py docstring) — opposite of what an
            # uninspected ``size[0]→width`` would give. Get this wrong and
            # the edge-point math lands the approach in a wall.
            half_extents_m = self._model.geom_size[geomadr]
            depth_cm = float(half_extents_m[0]) * 2.0 * 100.0
            width_cm = float(half_extents_m[1]) * 2.0 * 100.0
            height_cm = float(half_extents_m[2]) * 2.0 * 100.0

            self._movables[name] = {
                "body_id": bid,
                "geom_id": geomadr,
                "width": width_cm,
                "depth": depth_cm,
                "height": height_cm,
            }

    # ----------------------------------------------------- thread lifecycle

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        if self._want_viewer:
            # Passive viewer = no internal sim loop, we drive it via
            # viewer.sync() from the physics thread.
            self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
        if self._record_path:
            self._setup_recording()
        self._thread = Thread(target=self._physics_loop, daemon=True)
        self._thread.start()

    def _setup_recording(self) -> None:
        """Pre-compute frame stride; the actual render pass runs in stop()
        on the main thread (see _render_video_from_qpos for why)."""
        self._video_stride = max(1, int(round((1.0 / _VIDEO_FPS) / self._physics_dt)))
        self._video_tick_count = 0
        self._video_qpos_frames = []
        print(f"[MujocoSimEnv] capturing qpos for video → {self._record_path} "
              f"(stride={self._video_stride}, fps={_VIDEO_FPS})",
              flush=True)

    def _compute_world_bounds_m(self, data=None) -> tuple:
        """Return (x_min, x_max, y_min, y_max) in metres covering all geoms.

        Pass an explicit ``data`` to read bounds from a non-live MjData (e.g.
        the fresh one we build for offline video render). Defaults to self._data.
        """
        d = data if data is not None else self._data
        xs = []
        ys = []
        for gid in range(self._model.ngeom):
            pos = d.geom_xpos[gid]
            xs.append(float(pos[0]))
            ys.append(float(pos[1]))
        if not xs:
            return (-0.5, 0.5, -0.5, 0.5)
        return (min(xs), max(xs), min(ys), max(ys))

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None
        # Render the video on this (main) thread now that physics has stopped.
        # Rendering from the physics thread produced black frames because
        # mujoco.Renderer's GL context binds to its creating thread.
        if self._record_path and self._video_qpos_frames:
            self._render_video_from_qpos()

    def stop_robot(self) -> None:
        """Zero wheel commands. Runtime calls this on shutdown."""
        self.apply(Action(left_speed=0.0, right_speed=0.0))

    # --------------------------------------------------------- Environment

    def apply(self, action: Action) -> None:
        with self._action_lock:
            self._action = action

    def observe(self) -> Observation:
        # Snapshot positions/orientations under the lock so they're internally
        # consistent (no mixing of pre/post mj_step values).
        with self._state_lock:
            car_xyz = self._data.xpos[self._car_body_id].copy()
            car_quat = self._data.xquat[self._car_body_id].copy()
            obstacles_snapshot = [
                (
                    name,
                    self._data.geom_xpos[meta["geom_id"]].copy(),
                    self._data.geom_xmat[meta["geom_id"]].copy(),  # 9-elem flat
                    meta,
                )
                for name, meta in self._movables.items()
            ]

        rx_m, ry_m = float(car_xyz[0]), float(car_xyz[1])
        theta_deg = _quat_to_yaw_deg(*(float(v) for v in car_quat))

        objects: Dict[str, ObjectPose] = {}
        for name, geom_xpos, geom_xmat, meta in obstacles_snapshot:
            ox_m, oy_m = float(geom_xpos[0]), float(geom_xpos[1])
            # Yaw from rotation matrix: atan2(R[1,0], R[0,0]). geom_xmat
            # is a row-major 3x3 flattened to 9 elements.
            otheta_deg = math.degrees(math.atan2(float(geom_xmat[3]), float(geom_xmat[0])))
            pose = ObjectPose(
                x=ox_m * 100.0,
                y=oy_m * 100.0,
                theta=otheta_deg,
                width=meta["width"],
                depth=meta["depth"],
                height=meta["height"],
                is_static=False,
            )
            # Emit each object under exactly one key so downstream auto-detect
            # ("exactly one movable") doesn't double-count. Prefer the short
            # ``obj_N`` form because that's what real-side ArUco produces; if
            # the body name doesn't match the ``obstacle_<digits>_movable``
            # convention, fall back to the body name verbatim.
            key = name
            if name.startswith("obstacle_") and name.endswith("_movable"):
                short = name[len("obstacle_"):-len("_movable")]
                if short.isdigit():
                    key = f"obj_{short}"
            objects[key] = pose

        return Observation(
            robot_x=rx_m * 100.0,
            robot_y=ry_m * 100.0,
            robot_theta=theta_deg,
            objects=objects,
            timestamp=time.time(),
        )

    def close(self) -> None:
        self.stop()

    # ------------------------------------------------------------ internals

    def _physics_loop(self) -> None:
        """Step MuJoCo at the model's native timestep, applying the latest
        Action each tick. We don't try to honour wall-clock; the controller
        loop runs at 30 Hz and physics runs much faster — letting MuJoCo
        run at its own pace keeps integration stable and matches how the
        C++ side runs it.
        """
        max_wheel_omega = _PWM_TO_LINEAR_M_S / self._wheel_radius

        while self._running:
            t0 = time.perf_counter()

            with self._action_lock:
                left_pwm = float(self._action.left_speed)
                right_pwm = float(self._action.right_speed)
            left_omega = left_pwm * max_wheel_omega * self._speed_scale
            right_omega = right_pwm * max_wheel_omega * self._speed_scale

            with self._state_lock:
                self._data.ctrl[self._left_act] = left_omega
                self._data.ctrl[self._right_act] = right_omega
                mujoco.mj_step(self._model, self._data)

            if self._viewer is not None:
                try:
                    self._viewer.sync()
                except Exception:
                    self._viewer = None  # viewer closed by user

            # Frame capture for the MP4 — just snapshot qpos here, no render.
            # The render pass runs in stop() on the main thread.
            if self._record_path:
                self._video_tick_count += 1
                if self._video_tick_count % self._video_stride == 0:
                    self._video_qpos_frames.append(self._data.qpos.copy())

            # Sleep so wall-clock doesn't run too far ahead of sim time.
            elapsed = time.perf_counter() - t0
            sleep_for = self._physics_dt - elapsed
            if sleep_for > 0.0:
                time.sleep(sleep_for)

    # -------------------------------------------------------------- helpers

    def _render_video_from_qpos(self) -> None:
        """Replay captured qpos snapshots into an MP4 on the main thread.

        Same approach as sim_replay_subprocess: rebuild MjData fresh, walk
        each saved qpos, mj_forward + renderer.render + cv2.write. Runs
        once at stop() after the physics thread has joined, so the
        renderer's GL context lives entirely on the main thread.
        """
        import cv2  # local import: only needed when recording is on
        n = len(self._video_qpos_frames)
        if n == 0:
            return
        try:
            renderer = mujoco.Renderer(
                self._model, height=_VIDEO_HEIGHT, width=_VIDEO_WIDTH
            )
        except Exception as exc:
            print(f"[MujocoSimEnv] renderer init failed; video not written: "
                  f"{exc!r}", flush=True)
            return

        # Free top-down camera framing the workspace. Bounds computed from
        # the FIRST captured frame (geom_xpos at t=0 covers the full scene).
        data_render = mujoco.MjData(self._model)
        data_render.qpos[:] = self._video_qpos_frames[0][: self._model.nq]
        mujoco.mj_forward(self._model, data_render)
        bounds = self._compute_world_bounds_m(data_render)
        cx = 0.5 * (bounds[0] + bounds[1])
        cy = 0.5 * (bounds[2] + bounds[3])
        extent = max(bounds[1] - bounds[0], bounds[3] - bounds[2])
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [cx, cy, 0.0]
        cam.distance = max(extent, 0.5) * _CAMERA_DISTANCE_FACTOR
        cam.azimuth = 90.0
        cam.elevation = -90.0

        Path(self._record_path).parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            self._record_path, fourcc, _VIDEO_FPS, (_VIDEO_WIDTH, _VIDEO_HEIGHT)
        )
        if not writer.isOpened():
            print(f"[MujocoSimEnv] VideoWriter open failed at {self._record_path}; "
                  f"video not written", flush=True)
            try:
                renderer.close()
            except Exception:
                pass
            return

        for q in self._video_qpos_frames:
            nq = min(len(q), self._model.nq)
            if nq > 0:
                data_render.qpos[:nq] = q[:nq]
            mujoco.mj_forward(self._model, data_render)
            renderer.update_scene(data_render, cam)
            rgb = renderer.render()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            writer.write(bgr)
        writer.release()
        try:
            renderer.close()
        except Exception:
            pass
        print(f"[MujocoSimEnv] wrote video → {self._record_path} "
              f"({n} frames @ {_VIDEO_FPS} fps)", flush=True)

    def get_status(self) -> dict:
        """Stub for Runtime — sim has no connectivity status."""
        return {"online": True, "rssi": 0}


# Module-level helpers --------------------------------------------------------

def _inject_offscreen_size(xml_str: str, width: int, height: int) -> str:
    """Inject ``<visual><global offwidth=.. offheight=../></visual>`` so the
    model supports offscreen rendering at the video resolution. Same trick
    sim_replay_subprocess uses — without it ``mujoco.Renderer`` falls back to
    the default 640x480 buffer regardless of the requested size."""
    root = ET.fromstring(xml_str)
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    glob = visual.find("global")
    if glob is None:
        glob = ET.SubElement(visual, "global")
    glob.set("offwidth", str(width))
    glob.set("offheight", str(height))
    return ET.tostring(root, encoding="unicode")
