"""Binary-search for the sim ``push_tracker_max_speed`` that makes sim's
chassis cruise velocity match real's at PWM 0.4 (or whatever the real
session was run at).

Reads ``session_summary.json`` written by ``collect_pp_trial_real.py``,
then iteratively runs sim PP trials at midpoint fractions inside
``[L, U]``, narrowing the bracket each iteration until the velocity
difference drops below ``ε = eps_multiplier × SE_real``.

The bracket bounds reflect the controller's effective range:
    - L (lower) defaults to 0.05 = FollowPathController.WHEEL_DEADBAND.
      Below this, _enforce_deadband_scale clamps wheel commands UP to
      0.05, so v_sim is constant in [0, 0.05) — searching there is
      meaningless. If real's measured v is below sim(0.05), search
      converges at the floor and signals "deadband is the bottleneck."
    - U (upper) defaults to 0.375 — the *saturation point*. Above 0.375
      the wheel actuator clips (ctrl > 25 rad/s) and v_sim no longer
      grows with the fraction, so a search above 0.375 is meaningless.

Sim is deterministic, so one trial per iteration is enough.

Usage:
    python scripts/find_sim_fraction.py \\
        --session-dir chassis_calibration/<timestamp>_pp_session
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional

# Make in-tree imports work whether or not the package is pip-install -e'd.
HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import mujoco

from robot_control.camera import make_real_workspace_config
from robot_control.controller.follow_path import FollowPathController
from robot_control.core.types import NavigateSubgoal, Observation
from robot_control.utils.robot_geometry import effective_robot_size_cm
from robot_control.diagnostics.chassis_motion import (
    CommandSample,
    PoseSample,
    TrialMeta,
    cruise_forward_velocity,
    pose_dedupe,
    quat_z_to_theta_deg,
    write_command_log,
    write_trial,
)


# ─────────────────────────────────────────────────────────────────────────
# Sim constants — see CLAUDE.md "no magic constants"
# ─────────────────────────────────────────────────────────────────────────

# Mirrors namo_cpp/src/planning/namo_push_controller.cpp:22
# (constexpr double kCarWheelMaxSpeedMs = 1.0). The C++ push tracker uses
# this to convert wheel fraction → wheel ang vel. We mirror it exactly so
# the sim chassis we measure here matches the chassis the C++ push
# primitive will produce in production.
K_CAR_WHEEL_MAX_SPEED_MS = 1.0

# Wheel radius from namo_cpp/test_xml/little-car-modeling-package/assets/
#   mjcf/little_car.xml — geom name="left_wheel_collision" size="0.015 ..."
WHEEL_RADIUS_M = 0.015

# Actuator ctrlrange ceiling from the same XML (ctrlrange="-25 25"). The
# upper bound on the binary search is set just below this in fraction
# space: U_max = 25 * 0.015 / 1.0 = 0.375.
WHEEL_CTRL_SATURATION_RADS = 25.0
SAT_FRACTION = WHEEL_CTRL_SATURATION_RADS * WHEEL_RADIUS_M / K_CAR_WHEEL_MAX_SPEED_MS  # = 0.375

# Default scene used by the C++ primitive generator. Same friction model;
# the binary search would be biased if we used a scene with different
# floor friction.
DEFAULT_XML = (
    Path(__file__).resolve().parent.parent.parent
    / "namo_cpp" / "data" / "nominal_primitive_scene_wide_1x_car.xml"
)

# Default robot dimensions (cm). Mirrors PushPathFollower::Params defaults
# in namo_cpp/include/navigation/push_path_follower.hpp:15-17 so the sim
# tracker we run here is parameter-identical to the one production drives
# during primitive generation (generate_motion_primitives_db.cpp).
SIM_CAR_WIDTH_CM = 7.0
SIM_CAR_HEIGHT_CM = 7.0

# Sim wheel base. C++ production uses 7.5 cm (default in
# PushPathFollower::Params.wheel_base_m = 0.075). For a straight-line
# path the diff-drive term κ × wheel_base/2 ≈ 0, so this is harmless to
# get wrong, but we set it correctly for principle.
SIM_WHEEL_BASE_CM = 7.5

# Lookahead + goal-tolerance ratios. We use NAVIGATION-style settings
# (lookahead = 0.5 × car_size, goal_tolerance = 0.2 × car_size) rather
# than push-tracker C++ defaults (both 0.3 ×). Why:
#   - The C++ PushPathFollower's push-tracker settings (0.3 ×) are tuned
#     for the contact phase, where the object damps oscillation.
#   - Real-side calibration is free-running (no object contact). Short
#     lookahead + no contact → visible wiggle.
#   - To keep sim/real apples-to-apples, sim mirrors real's settings.
# The wheel-to-chassis-velocity mapping is independent of these PP
# tuning knobs in steady-state straight cruise, so the resulting
# calibration still applies cleanly to push_tracker_max_speed in
# production.
SIM_LOOKAHEAD_RATIO = 0.5
SIM_GOAL_TOLERANCE_RATIO = 0.2

# Pre-push settle ticks — matches namo_push_controller.cpp:504
# (kSettleSteps = settle_steps_override > 0 ? override : 100). With the
# car's 0.002 s timestep that's 0.2 s of damping, enough for the chassis
# to land on its wheels after the qpos write and for any residual joint
# velocities to bleed off.
SIM_PRE_SETTLE_TICKS = 100

# Pose-sampling rate in sim — matches the camera service's 30 Hz to keep
# cruise-velocity computation comparable across sim and real.
SIM_POSE_SAMPLE_HZ = 30.0

# Per-iteration sim timeout. At the lowest practical search fraction
# (~0.01 → ~1 cm/s) we still reach 50 cm in ~50 s. 60 s is the safety
# margin; if the trial doesn't make it to 50 cm we use whatever it
# actually traveled.
SIM_TIMEOUT_S = 60.0

# Park-obstacle position: the primitive scene contains one obstacle near
# (0, 0); we shove it out of the way so the car's straight-line path is
# clear. 5 m × 5 m is well outside any sane workspace.
PARK_OBSTACLE_POS = (5.0, 5.0, 0.05)

# Binary-search safety: stop also if the bracket width in fraction space
# falls below this, even if we haven't hit the velocity-epsilon. Prevents
# infinite loops if the velocity-fraction curve has a flat spot.
MIN_BRACKET_FRACTION = 1e-4

# Sim path length (cm). Same as the real path so the cruise window
# computations are directly comparable.
PATH_LENGTH_CM = 50.0

# Video recording constants — match the production sim-replay subprocess
# (diagnostics/sim_replay_subprocess.py) so the visual style is familiar.
RECORD_WIDTH = 1280
RECORD_HEIGHT = 720
RECORD_FPS = 30
RECORD_CAMERA_DISTANCE_FACTOR = 1.6   # extent × this = camera distance


# ─────────────────────────────────────────────────────────────────────────
# Sim model helpers
# ─────────────────────────────────────────────────────────────────────────


class SimModel:
    """Wraps the MuJoCo model + addresses so we don't re-look-up names per call."""

    def __init__(self, xml_path: Path) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        # Actuator ids
        self.left_act = self.model.actuator("left_wheel_drive").id
        self.right_act = self.model.actuator("right_wheel_drive").id

        # Car freejoint (chassis pose lives in qpos[adr : adr+7])
        car_joint = self.model.joint("car_freejoint")
        self.car_qpos_adr = int(car_joint.qposadr[0])
        self.car_qvel_adr = int(car_joint.dofadr[0])

        # Obstacle freejoint. In the primitive-generation templates the
        # joint itself is unnamed (<joint type="free"/>), so we have to
        # look it up by body name and walk through body_jntadr → jnt_qposadr.
        # The car and obstacle overlap at (0,0,0.05) in the template — if
        # we don't park the obstacle, contact at t=0 shoves the car and
        # the cruise velocity reads garbage.
        self.obstacle_qpos_adr: Optional[int] = None
        try:
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, "obstacle_1_movable"
            )
            if body_id >= 0 and self.model.body_jntnum[body_id] >= 1:
                jnt_idx = int(self.model.body_jntadr[body_id])
                if int(self.model.jnt_type[jnt_idx]) == mujoco.mjtJoint.mjJNT_FREE:
                    self.obstacle_qpos_adr = int(self.model.jnt_qposadr[jnt_idx])
        except Exception:
            self.obstacle_qpos_adr = None
        if self.obstacle_qpos_adr is None:
            print(
                "[SimModel] WARNING: no obstacle freejoint found at body "
                "'obstacle_1_movable' — if the scene actually contains an "
                "obstacle that overlaps the car start pose, sim trials will "
                "be contaminated by start-time contact."
            )

        # Initial chassis Z above the floor. The freejoint settles via
        # mj_forward, but we need a positive starting height so the
        # wheel geoms don't penetrate the floor at t=0. 1 cm matches the
        # wheel radius (geoms in little_car.xml).
        self.car_z_init = 0.015

    def reset(self) -> None:
        """Reset to a known state: robot at origin facing +x, obstacle parked,
        then run SIM_PRE_SETTLE_TICKS of zero-ctrl physics to let the
        chassis land on its wheels (matches namo_push_controller.cpp:519).
        """
        mujoco.mj_resetData(self.model, self.data)
        if self.obstacle_qpos_adr is not None:
            self.data.qpos[self.obstacle_qpos_adr : self.obstacle_qpos_adr + 3] = list(
                PARK_OBSTACLE_POS
            )
        a = self.car_qpos_adr
        self.data.qpos[a : a + 3] = [0.0, 0.0, self.car_z_init]
        self.data.qpos[a + 3 : a + 7] = [1.0, 0.0, 0.0, 0.0]   # identity quat → faces +x
        b = self.car_qvel_adr
        self.data.qvel[b : b + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

        # Pre-push settle — production does this before any push command.
        # Zero ctrl, let physics integrate, lets wheels' commanded ω damp
        # to zero and chassis settle. Sim.data.time advances by
        # SIM_PRE_SETTLE_TICKS × timestep after this; the trial loop's
        # sample timestamps will be relative to that post-settle time.
        self.data.ctrl[self.left_act] = 0.0
        self.data.ctrl[self.right_act] = 0.0
        for _ in range(SIM_PRE_SETTLE_TICKS):
            mujoco.mj_step(self.model, self.data)
        # Reset sim time so the trial logs start at t=0 (the cruise-window
        # metric assumes the log starts when commanded motion begins).
        self.data.time = 0.0

    def read_pose_cm_deg(self) -> tuple:
        """Return (x_cm, y_cm, theta_deg) for the car chassis."""
        a = self.car_qpos_adr
        x_m = float(self.data.qpos[a])
        y_m = float(self.data.qpos[a + 1])
        qw = float(self.data.qpos[a + 3])
        qz = float(self.data.qpos[a + 6])
        theta_deg = quat_z_to_theta_deg(qw, qz)
        return x_m * 100.0, y_m * 100.0, theta_deg

    def set_wheel_fractions(self, left_frac: float, right_frac: float) -> None:
        """Convert wheel fractions → actuator ctrl (rad/s)."""
        left_rads = left_frac * K_CAR_WHEEL_MAX_SPEED_MS / WHEEL_RADIUS_M
        right_rads = right_frac * K_CAR_WHEEL_MAX_SPEED_MS / WHEEL_RADIUS_M
        self.data.ctrl[self.left_act] = left_rads
        self.data.ctrl[self.right_act] = right_rads


# ─────────────────────────────────────────────────────────────────────────
# Single sim trial — drives PP for one max_speed, returns cruise vel
# ─────────────────────────────────────────────────────────────────────────


def run_sim_trial(
    sim: SimModel,
    workspace_config,
    max_speed: float,
    trial_out_dir: Path,
) -> dict:
    """Run one sim PP trial at ``max_speed``. Saves trial dir. Returns metrics."""
    sim.reset()

    # Starting pose & path
    x0, y0, theta0 = sim.read_pose_cm_deg()
    theta0_rad = math.radians(theta0)
    ux, uy = math.cos(theta0_rad), math.sin(theta0_rad)
    goal_x = x0 + PATH_LENGTH_CM * ux
    goal_y = y0 + PATH_LENGTH_CM * uy
    path = [(x0, y0), (goal_x, goal_y)]

    # Construct FollowPathController with parameter values that mirror
    # the C++ PushPathFollower used by primitive generation
    # (namo_push_controller.cpp:61-73). Lookahead and goal_tolerance use
    # the production ratios 0.3 × car_size; FollowPathController defaults
    # are 0.5 × car_size and 0.2 × car_size respectively. For straight
    # paths the wheel-command output is identical regardless, but matching
    # production ratios keeps the comparison principled. Use
    # effective_robot_size_cm here so the formula matches whatever real
    # production uses — single source of truth.
    car_size_cm = effective_robot_size_cm(workspace_config.car_width, workspace_config.car_height)
    ctl = FollowPathController(
        workspace_config,
        max_speed=max_speed,
        lookahead_distance=SIM_LOOKAHEAD_RATIO * car_size_cm,
        goal_tolerance=SIM_GOAL_TOLERANCE_RATIO * car_size_cm,
    )
    ctl.set_path(path)

    sample_interval = 1.0 / SIM_POSE_SAMPLE_HZ
    next_sample_t = 0.0
    dummy_subgoal = NavigateSubgoal(x=goal_x, y=goal_y)

    poses: List[PoseSample] = []
    commands: List[CommandSample] = []
    goal_reached = False
    time_to_goal_s: Optional[float] = None

    while sim.data.time < SIM_TIMEOUT_S:
        x, y, theta = sim.read_pose_cm_deg()
        sim_t = float(sim.data.time)

        obs = Observation(
            robot_x=x, robot_y=y, robot_theta=theta,
            objects={}, timestamp=sim_t,
        )
        action = ctl.step(obs, dummy_subgoal)

        # Use the controller's own status as the goal-reach signal so the
        # trial honors whatever goal_tolerance we configured (here 0.3 ×
        # car_size to match production). A separate outer dist check
        # would either fire too early (loses arrival samples) or too late
        # (controller has already stopped wheels, robot never gets closer
        # → outer never fires → trial waits until SIM_TIMEOUT_S).
        if ctl.get_status() == "FINISHED":
            goal_reached = True
            time_to_goal_s = sim_t
            break

        sim.set_wheel_fractions(action.left_speed, action.right_speed)
        mujoco.mj_step(sim.model, sim.data)

        if sim_t >= next_sample_t:
            poses.append(PoseSample(t_s=sim_t, x_cm=x, y_cm=y, theta_deg=theta))
            commands.append(
                CommandSample(
                    t_s=sim_t,
                    left_cmd=float(action.left_speed),
                    right_cmd=float(action.right_speed),
                    mode=str(ctl.metadata.get("mode", "")),
                )
            )
            next_sample_t = sim_t + sample_interval

    # Make sure the final pose makes it into the log
    x, y, theta = sim.read_pose_cm_deg()
    if not poses or poses[-1].t_s < sim.data.time:
        poses.append(PoseSample(t_s=float(sim.data.time), x_cm=x, y_cm=y, theta_deg=theta))

    poses = pose_dedupe(poses)

    # Cruise velocity
    cruise = cruise_forward_velocity(
        poses,
        path_start_cm=(x0, y0),
        path_unit_vec=(ux, uy),
    )
    cruise.goal_reached = goal_reached
    cruise.time_to_goal_s = time_to_goal_s

    # Save trial dir
    trial_out_dir.mkdir(parents=True, exist_ok=True)
    meta = TrialMeta(
        trial_kind="straight",
        source="sim",
        left_cmd=float(max_speed),
        right_cmd=float(max_speed),
        cmd_units="fraction",
        duration_s=float(poses[-1].t_s) if poses else 0.0,
        started_at_utc_iso=(
            dt.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        ),
        n_samples=len(poses),
        sim_meta={
            "controller": "FollowPathController",
            "max_speed": float(max_speed),
            "k_car_wheel_max_speed_ms": K_CAR_WHEEL_MAX_SPEED_MS,
            "wheel_radius_m": WHEEL_RADIUS_M,
            "saturation_fraction": SAT_FRACTION,
            "path_start_cm": [x0, y0],
            "path_end_cm": [goal_x, goal_y],
            "path_unit_vec": [ux, uy],
            "goal_reached": bool(goal_reached),
            "time_to_goal_s": time_to_goal_s,
            "forward_cruise_cm_s": cruise.forward_cruise_cm_s,
            "arc_length_cm": cruise.arc_length_cm,
        },
    )
    write_trial(trial_out_dir, meta, poses)
    write_command_log(trial_out_dir / "commands.jsonl", commands)
    (trial_out_dir / "path.json").write_text(
        json.dumps({"path_cm": [list(pt) for pt in path]}, indent=2)
    )

    return {
        "max_speed": float(max_speed),
        "forward_cruise_cm_s": cruise.forward_cruise_cm_s,
        "arc_length_cm": cruise.arc_length_cm,
        "goal_reached": goal_reached,
        "time_to_goal_s": time_to_goal_s,
        "n_cruise_samples": cruise.n_cruise_samples,
    }


# ─────────────────────────────────────────────────────────────────────────
# Video recording — re-runs the trial at the converged X with rendering
# ─────────────────────────────────────────────────────────────────────────


def _inject_offscreen_size(xml_path: Path, width: int, height: int) -> str:
    """Return the XML text with <visual><global offwidth=.. offheight=../></visual>
    injected. Mirrors diagnostics/sim_replay_subprocess.py:_inject_offscreen_size
    so MuJoCo's offscreen renderer doesn't hit default-size caps.
    """
    root = ET.fromstring(xml_path.read_text())
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    glob = visual.find("global")
    if glob is None:
        glob = ET.SubElement(visual, "global")
    glob.set("offwidth", str(width))
    glob.set("offheight", str(height))
    return ET.tostring(root, encoding="unicode")


def record_sim_trial(
    xml_path: Path,
    workspace_config,
    max_speed: float,
    output_mp4: Path,
) -> dict:
    """Run one sim PP trial at ``max_speed`` and write an MP4 of the run.

    Uses a fresh MuJoCo model loaded from XML with offscreen-render size
    injected (so the renderer can allocate the framebuffer). Same control
    loop as run_sim_trial — same wheel-cmd → ctrl conversion, same
    pre-settle, same FollowPathController params — but renders a frame
    per sim-time slot of 1/RECORD_FPS.

    Returns the same metrics dict as run_sim_trial so the caller can
    compare against the binary-search iteration's measured v_sim.
    """
    import cv2
    import mujoco

    print(f"[record] loading sim model w/ offscreen size {RECORD_WIDTH}×{RECORD_HEIGHT}")
    # Write the size-injected XML next to the original so any <include>
    # paths (e.g., ../test_xml/.../little_car.xml) still resolve. Loading
    # via from_xml_string would use the CWD as the include base and break.
    xml_text = _inject_offscreen_size(xml_path, RECORD_WIDTH, RECORD_HEIGHT)
    sibling_xml = xml_path.parent / f".__record_{xml_path.stem}.xml"
    sibling_xml.write_text(xml_text)
    try:
        model = mujoco.MjModel.from_xml_path(str(sibling_xml))
    finally:
        try:
            sibling_xml.unlink()
        except OSError:
            pass
    data = mujoco.MjData(model)

    # Same address lookups as SimModel — duplicated here so this function
    # is self-contained (avoids passing a fresh SimModel through the args).
    left_act = model.actuator("left_wheel_drive").id
    right_act = model.actuator("right_wheel_drive").id
    car_qpos_adr = int(model.joint("car_freejoint").qposadr[0])
    car_qvel_adr = int(model.joint("car_freejoint").dofadr[0])
    obstacle_qpos_adr: Optional[int] = None
    try:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "obstacle_1_movable")
        if body_id >= 0 and model.body_jntnum[body_id] >= 1:
            jnt_idx = int(model.body_jntadr[body_id])
            if int(model.jnt_type[jnt_idx]) == mujoco.mjtJoint.mjJNT_FREE:
                obstacle_qpos_adr = int(model.jnt_qposadr[jnt_idx])
    except Exception:
        pass

    # Reset + park obstacle + pre-settle (mirrors SimModel.reset).
    mujoco.mj_resetData(model, data)
    if obstacle_qpos_adr is not None:
        data.qpos[obstacle_qpos_adr : obstacle_qpos_adr + 3] = list(PARK_OBSTACLE_POS)
    data.qpos[car_qpos_adr : car_qpos_adr + 3] = [0.0, 0.0, 0.015]
    data.qpos[car_qpos_adr + 3 : car_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[car_qvel_adr : car_qvel_adr + 6] = 0.0
    mujoco.mj_forward(model, data)
    data.ctrl[left_act] = 0.0
    data.ctrl[right_act] = 0.0
    for _ in range(SIM_PRE_SETTLE_TICKS):
        mujoco.mj_step(model, data)
    data.time = 0.0

    # Read post-settle pose for path construction.
    a = car_qpos_adr
    x_m, y_m = float(data.qpos[a]), float(data.qpos[a + 1])
    qw, qz = float(data.qpos[a + 3]), float(data.qpos[a + 6])
    theta_deg = quat_z_to_theta_deg(qw, qz)
    x0, y0 = x_m * 100.0, y_m * 100.0
    theta0_rad = math.radians(theta_deg)
    ux, uy = math.cos(theta0_rad), math.sin(theta0_rad)
    goal_x = x0 + PATH_LENGTH_CM * ux
    goal_y = y0 + PATH_LENGTH_CM * uy
    path = [(x0, y0), (goal_x, goal_y)]

    car_size_cm = effective_robot_size_cm(workspace_config.car_width, workspace_config.car_height)
    ctl = FollowPathController(
        workspace_config,
        max_speed=max_speed,
        lookahead_distance=SIM_LOOKAHEAD_RATIO * car_size_cm,
        goal_tolerance=SIM_GOAL_TOLERANCE_RATIO * car_size_cm,
    )
    ctl.set_path(path)
    dummy_subgoal = NavigateSubgoal(x=goal_x, y=goal_y)

    # Top-down camera centered on the start pose.
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [x_m, y_m, 0.0]
    # Path is 0.5 m long; use that as the extent so the camera frames the run.
    extent_m = max(0.5, PATH_LENGTH_CM / 100.0) * 1.2
    camera.distance = extent_m * RECORD_CAMERA_DISTANCE_FACTOR
    camera.azimuth = 90.0
    camera.elevation = -90.0

    renderer = mujoco.Renderer(model, height=RECORD_HEIGHT, width=RECORD_WIDTH)

    output_mp4.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_mp4), fourcc, RECORD_FPS, (RECORD_WIDTH, RECORD_HEIGHT))
    if not writer.isOpened():
        print(f"[record] VideoWriter open failed at {output_mp4}", file=sys.stderr)
        renderer.close()
        return {"error": "video_writer_failed"}

    # Trial loop. PP every mj_step (matches run_sim_trial); render at fixed
    # video FPS (~30) by advancing sim time and snapshotting when it passes
    # next frame deadline.
    frame_period_s = 1.0 / RECORD_FPS
    next_frame_t = 0.0
    poses: List[PoseSample] = []
    sample_interval = 1.0 / SIM_POSE_SAMPLE_HZ
    next_sample_t = 0.0
    goal_reached = False
    time_to_goal_s: Optional[float] = None

    while data.time < SIM_TIMEOUT_S:
        sim_t = float(data.time)
        # Pose
        a = car_qpos_adr
        x_m, y_m = float(data.qpos[a]), float(data.qpos[a + 1])
        qw, qz = float(data.qpos[a + 3]), float(data.qpos[a + 6])
        theta_deg = quat_z_to_theta_deg(qw, qz)
        x_cm, y_cm = x_m * 100.0, y_m * 100.0

        obs = Observation(
            robot_x=x_cm, robot_y=y_cm, robot_theta=theta_deg,
            objects={}, timestamp=sim_t,
        )
        action = ctl.step(obs, dummy_subgoal)
        if ctl.get_status() == "FINISHED":
            goal_reached = True
            time_to_goal_s = sim_t
            break

        # Apply + step physics
        data.ctrl[left_act] = action.left_speed * K_CAR_WHEEL_MAX_SPEED_MS / WHEEL_RADIUS_M
        data.ctrl[right_act] = action.right_speed * K_CAR_WHEEL_MAX_SPEED_MS / WHEEL_RADIUS_M
        mujoco.mj_step(model, data)

        # Pose log
        if sim_t >= next_sample_t:
            poses.append(PoseSample(t_s=sim_t, x_cm=x_cm, y_cm=y_cm, theta_deg=theta_deg))
            next_sample_t = sim_t + sample_interval

        # Video frame
        if sim_t >= next_frame_t:
            renderer.update_scene(data, camera)
            rgb = renderer.render()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            writer.write(bgr)
            next_frame_t = sim_t + frame_period_s

    # Final frame so the video shows the terminal pose
    renderer.update_scene(data, camera)
    bgr = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
    writer.write(bgr)

    writer.release()
    renderer.close()

    # Cruise metric for sanity-check against the binary-search iteration.
    try:
        cruise = cruise_forward_velocity(
            poses, path_start_cm=(x0, y0), path_unit_vec=(ux, uy)
        )
        forward = cruise.forward_cruise_cm_s
        arc = cruise.arc_length_cm
    except Exception as exc:
        forward = float("nan")
        arc = float("nan")
        print(f"[record] cruise metric failed: {exc!r}")

    print(
        f"[record] wrote {output_mp4} | "
        f"v_sim={forward:.3f} cm/s | arc={arc:.1f} cm | "
        f"goal={'✓' if goal_reached else '✗'} | "
        f"sim_t={data.time:.2f}s"
    )

    return {
        "max_speed": float(max_speed),
        "forward_cruise_cm_s": forward,
        "arc_length_cm": arc,
        "goal_reached": goal_reached,
        "time_to_goal_s": time_to_goal_s,
        "output_mp4": str(output_mp4),
    }


# ─────────────────────────────────────────────────────────────────────────
# Binary search
# ─────────────────────────────────────────────────────────────────────────


def _best_iteration(iterations: List[dict]) -> Optional[dict]:
    """Return the iteration with the smallest |delta_cm_s|, or None.

    Used when the search hits max_iters or a too-narrow bracket without
    converging, so we report an X we actually tested instead of an
    untested midpoint.
    """
    have_delta = [it for it in iterations if "delta_cm_s" in it]
    if not have_delta:
        return None
    return min(have_delta, key=lambda it: abs(it["delta_cm_s"]))


def binary_search(
    sim: SimModel,
    workspace_config,
    sim_out_dir: Path,
    v_real_median: float,
    epsilon: float,
    L0: float,
    U0: float,
    max_iters: int,
) -> dict:
    """Bisect [L0, U0] until |v_sim(X) − v_real_median| ≤ epsilon."""
    L, U = float(L0), float(U0)
    iterations: List[dict] = []
    converged = False
    final_X: Optional[float] = None

    print(
        f"[search] target v_real={v_real_median:.3f} cm/s, ε={epsilon:.3f} cm/s, "
        f"bracket=[{L:.4f}, {U:.4f}], max_iters={max_iters}"
    )

    for i in range(1, max_iters + 1):
        X = (L + U) / 2.0
        trial_dir = sim_out_dir / f"iter_{i:02d}_X{X:0.5f}".replace(".", "p")
        t_start = time.time()
        try:
            res = run_sim_trial(sim, workspace_config, X, trial_dir)
        except Exception as exc:
            print(f"[iter {i:02d}] FAILED: {exc!r}")
            iterations.append({
                "iter": i, "X": X, "L_before": L, "U_before": U,
                "error": repr(exc),
            })
            # Bias upward to escape: if a tiny X failed (no progress), raise L
            L = X
            continue
        wall_s = time.time() - t_start

        v_sim = res["forward_cruise_cm_s"]
        delta = v_sim - v_real_median
        within = abs(delta) <= epsilon

        print(
            f"[iter {i:02d}] X={X:.5f}  v_sim={v_sim:6.3f} cm/s  "
            f"Δ={delta:+.3f}  ε={epsilon:.3f}  "
            f"goal={'✓' if res['goal_reached'] else '✗'}  "
            f"wall={wall_s:.1f}s"
        )

        iterations.append({
            "iter": i,
            "X": X,
            "L_before": L,
            "U_before": U,
            "v_sim_cm_s": v_sim,
            "delta_cm_s": delta,
            "within_epsilon": within,
            "goal_reached": res["goal_reached"],
            "time_to_goal_s": res["time_to_goal_s"],
            "arc_length_cm": res["arc_length_cm"],
            "n_cruise_samples": res["n_cruise_samples"],
            "wall_s": wall_s,
            "trial_dir": str(trial_dir.resolve()),
        })

        if within:
            converged = True
            final_X = X
            print(f"[search] ✓ converged at iter {i}: X={X:.5f}")
            break

        # Tighten bracket. v_sim is monotonic in fraction below saturation.
        if delta > 0:
            U = X    # sim too fast → reduce upper
        else:
            L = X    # sim too slow → raise lower

        if (U - L) < MIN_BRACKET_FRACTION:
            # Pick the best-tested X (smallest |delta|) rather than the
            # untested midpoint, so the recommendation comes with a
            # measured v_sim attached.
            best = _best_iteration(iterations)
            final_X = best["X"] if best else (L + U) / 2.0
            print(
                f"[search] bracket below {MIN_BRACKET_FRACTION} "
                f"({U - L:.6f}) — stopping; reporting best-tested "
                f"X={final_X:.5f} (|Δ|={abs(best['delta_cm_s']):.3f} cm/s) "
                f"as best estimate." if best else
                f"[search] bracket below {MIN_BRACKET_FRACTION} — "
                f"no successful iterations; reporting midpoint {final_X:.5f}."
            )
            break

    if final_X is None:
        best = _best_iteration(iterations)
        final_X = best["X"] if best else (L + U) / 2.0
        print(
            f"[search] max_iters ({max_iters}) hit without convergence; "
            f"reporting best-tested X={final_X:.5f} "
            f"(|Δ|={abs(best['delta_cm_s']):.3f} cm/s)." if best else
            f"[search] max_iters hit and no iteration produced a v_sim "
            f"reading; reporting midpoint {final_X:.5f} (uninformative)."
        )

    return {
        "converged": converged,
        "recommended_push_tracker_max_speed": float(final_X),
        "final_bracket": [float(L), float(U)],
        "n_iterations": len(iterations),
        "iterations": iterations,
    }


# ─────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--session-dir",
        required=True,
        help="Calibration session dir (must contain real/session_summary.json).",
    )
    p.add_argument(
        "--xml-file",
        default=str(DEFAULT_XML),
        help=f"MuJoCo car XML (default: {DEFAULT_XML.name}).",
    )
    p.add_argument(
        "--max-iters",
        type=int,
        default=12,
        help="Max binary-search iterations (default: 12).",
    )
    p.add_argument(
        "--bracket-low",
        type=float,
        default=0.05,
        help="Lower bracket bound in fraction (default: 0.05 = "
        "FollowPathController.WHEEL_DEADBAND; below this the controller "
        "scales wheels UP to 0.05 so v_sim is flat and search is meaningless).",
    )
    p.add_argument(
        "--bracket-high",
        type=float,
        default=SAT_FRACTION,
        help=f"Upper bracket bound (default: {SAT_FRACTION:.3f} = saturation).",
    )
    p.add_argument(
        "--eps-multiplier",
        type=float,
        default=1.0,
        help="Convergence tolerance = eps_multiplier × SE(median) from real. "
        "Default: 1.0 (converge to within one real-side SE).",
    )
    p.add_argument(
        "--record-mp4",
        action="store_true",
        help="After the binary search converges, re-run one sim trial at the "
        "recommended X with offscreen MuJoCo rendering and write a top-down "
        "MP4 to <session_dir>/sim/replay_X<value>.mp4. Adds a few seconds.",
    )
    p.add_argument(
        "--record-only",
        type=float,
        default=None,
        help="Skip the search; just record a video at this fraction. Useful "
        "for replaying a previously-computed X. Requires --record-mp4.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    session_dir = Path(args.session_dir).resolve()
    real_summary_path = session_dir / "real" / "session_summary.json"
    if not real_summary_path.exists():
        print(
            f"[main] real summary not found at {real_summary_path}. "
            "Run collect_pp_trial_real.py first.",
            file=sys.stderr,
        )
        return 2

    real_summary = json.loads(real_summary_path.read_text())
    if "median_forward_cruise_cm_s" not in real_summary:
        print(
            "[main] real summary has no median (no valid trials?). "
            "Re-run real collection.",
            file=sys.stderr,
        )
        return 2

    v_real = float(real_summary["median_forward_cruise_cm_s"])
    se_real = float(real_summary["se_median_cm_s"])
    real_max_speed = float(real_summary["max_speed_used"])
    epsilon = args.eps_multiplier * se_real
    if epsilon <= 0:
        # If all real trials gave identical velocity (MAD = 0), fall back
        # to a fixed 5% tolerance so we don't search forever.
        epsilon = 0.05 * v_real
        print(
            f"[main] real SE was 0 (MAD=0); falling back to ε = 5% × v_real "
            f"= {epsilon:.3f} cm/s."
        )

    print(f"[main] real PWM used: {real_max_speed}")
    print(f"[main] real v_median = {v_real:.3f} cm/s")
    print(f"[main] real SE(median) = {se_real:.3f} cm/s")
    print(f"[main] convergence ε = {epsilon:.3f} cm/s")

    # WorkspaceConfig — mirrors C++ production sim values from
    # PushPathFollower::Params (push_path_follower.hpp:15-17):
    # robot_width = 0.07 m, robot_height = 0.07 m, wheel_base = 0.075 m.
    workspace_config = make_real_workspace_config(
        car_width=SIM_CAR_WIDTH_CM,
        car_height=SIM_CAR_HEIGHT_CM,
        wheel_base=SIM_WHEEL_BASE_CM,
    )

    sim_out_dir = session_dir / "sim"
    sim_out_dir.mkdir(parents=True, exist_ok=True)

    # If --record-only is set, skip search and just record at that fraction.
    if args.record_only is not None:
        if not args.record_mp4:
            print(
                "[main] --record-only requires --record-mp4. Adding it.",
                file=sys.stderr,
            )
            args.record_mp4 = True
        record_X = float(args.record_only)
        print(f"[main] --record-only: skipping search, recording at X={record_X}")
        mp4_path = sim_out_dir / f"replay_X{record_X:.5f}".replace(".", "p")
        mp4_path = mp4_path.with_suffix(".mp4")
        info = record_sim_trial(
            xml_path=Path(args.xml_file).resolve(),
            workspace_config=workspace_config,
            max_speed=record_X,
            output_mp4=mp4_path,
        )
        print(f"\n[main] MP4 → {mp4_path}")
        return 0

    # Load sim model once; reset between iterations.
    print(f"[main] loading sim model: {args.xml_file}")
    sim = SimModel(Path(args.xml_file).resolve())

    result = binary_search(
        sim=sim,
        workspace_config=workspace_config,
        sim_out_dir=sim_out_dir,
        v_real_median=v_real,
        epsilon=epsilon,
        L0=args.bracket_low,
        U0=args.bracket_high,
        max_iters=args.max_iters,
    )

    out = {
        "session_dir": str(session_dir),
        "real_pwm_used": real_max_speed,
        "real_v_median_cm_s": v_real,
        "real_se_median_cm_s": se_real,
        "epsilon_cm_s": epsilon,
        "bracket_initial": [args.bracket_low, args.bracket_high],
        "xml_file": str(Path(args.xml_file).resolve()),
        **result,
    }

    out_path = sim_out_dir / "search_result.json"
    out_path.write_text(json.dumps(out, indent=2))
    print(f"\n[main] result → {out_path}")

    X = result["recommended_push_tracker_max_speed"]
    print(f"\n{'=' * 60}")
    print(f"  Recommended push_tracker_max_speed = {X:.5f}")
    print(f"  (set in namo_cpp/config/namo_config_complete_skill15_car_1x.yaml)")
    print(
        f"  Converged: {result['converged']}, "
        f"iterations: {result['n_iterations']}"
    )
    print(f"{'=' * 60}")

    # Optional MP4 of the converged trial — visual sanity check that the
    # sim chassis behavior at X matches expectations.
    if args.record_mp4:
        mp4_path = sim_out_dir / f"replay_X{X:.5f}".replace(".", "p")
        mp4_path = mp4_path.with_suffix(".mp4")
        record_info = record_sim_trial(
            xml_path=Path(args.xml_file).resolve(),
            workspace_config=workspace_config,
            max_speed=X,
            output_mp4=mp4_path,
        )
        out["record"] = record_info
        out_path.write_text(json.dumps(out, indent=2))
        print(f"\n[main] MP4 → {mp4_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
