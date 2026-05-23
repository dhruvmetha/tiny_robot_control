"""Drive the diff-drive car in bare MuJoCo at a fixed wheel-actuator
command for N sim-seconds, log every Nth-tick pose to disk.

Uses ``mujoco.mj_step`` directly. Does NOT go through ``namo_rl.RLEnvironment``
— that would invoke the full push pipeline. Here we want raw chassis
dynamics: set the wheel velocity actuator commands, integrate physics,
record pose.

The scene is one of the friction-tuned car primitive scenes
(``namo_cpp/data/nominal_primitive_scene_wide_1x_car.xml``). We don't
modify it: we just exercise its car body and read the car's freejoint
qpos. The obstacle sits at the origin, untouched by the moving car.

Usage:
    # Straight line at the default fraction (mirrors real's push.max_speed
    # so a straight comparison is meaningful):
    python scripts/collect_chassis_motion_sim.py \\
        --out-dir chassis_calibration/<session>/sim/straight_cmd0p20

    # Override the fraction:
    python scripts/collect_chassis_motion_sim.py \\
        --left-frac 0.3 --right-frac 0.3 \\
        --out-dir chassis_calibration/<session>/sim/straight_cmd0p30

    # Turn in place (CCW):
    python scripts/collect_chassis_motion_sim.py \\
        --left-frac -0.2 --right-frac 0.2 \\
        --out-dir chassis_calibration/<session>/sim/turn_cmd0p20_ccw

The default value for --left-frac / --right-frac comes from
``config/controller.yaml: push.max_speed`` so the sim runs at numerically
the same wheel-command value as the real robot. With ``kCarWheelMaxSpeedMs
= 1.0`` (in ``namo_push_controller.cpp:22``), a sim fraction reads
directly as commanded chassis m/s.
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import sys
from pathlib import Path
from typing import Optional

# Make in-tree imports work whether or not the package is pip-install -e'd.
HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import mujoco

from robot_control.controller.config import load_controller_configs
from robot_control.diagnostics.chassis_motion import (
    PoseSample,
    TrialMeta,
    infer_trial_kind,
    quat_z_to_theta_deg,
    write_trial,
)


# ─────────────────────────────────────────────────────────────────────────
# Defaults — kept as module constants so a human reader can find them.
# ─────────────────────────────────────────────────────────────────────────

# Same scene the C++ push primitive generator uses. Friction is MuJoCo
# defaults on the obstacle (commit e7df838) and 0.5/0.005/0.001 on the
# floor. Editing this file would alter the primitive DB physics — don't.
DEFAULT_XML = (
    Path(__file__).resolve().parent.parent.parent
    / "namo_cpp" / "data" / "nominal_primitive_scene_wide_1x_car.xml"
)

# Mirror of the C++ compiled constant kCarWheelMaxSpeedMs (m/s) in
# namo_cpp/src/planning/namo_push_controller.cpp:22. Multiplying the
# follower fraction by this gives the commanded chassis m/s. If you
# recompile namo_cpp with a different value, pass --k-car-wheel-max-speed-ms
# here too — they must match for sim/real parity comparisons to be
# meaningful.
DEFAULT_K_CAR_WHEEL_MAX_SPEED_MS = 1.0

# Wheel cylinder radius from
# namo_cpp/test_xml/little-car-modeling-package/assets/mjcf/little_car.xml
# (geom size="0.015 0.0005" on left_wheel_collision / right_wheel_collision).
# Used to convert commanded chassis m/s → wheel rad/s.
DEFAULT_WHEEL_RADIUS_M = 0.015

# Sample sim qpos at ~this many Hz. The camera service produces ~15 Hz
# during motion (measured 2026-05-22) so we density-match by default;
# the analysis is robust to any rate ≥ a few Hz over a steady window.
DEFAULT_TARGET_SAMPLE_HZ = 15.0


# ─────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────


def default_push_fraction() -> float:
    """Mirror real's default push speed for sim's fraction knob.

    Real uses ``controller.yaml: push.max_speed`` as the PWM value driven
    during a push. We use the SAME number as sim's wheel-command fraction
    so the calibration experiment compares like-named inputs across sides.
    """
    return load_controller_configs().push.max_speed


def actuator_id(model, name: str) -> int:
    """Find a named MuJoCo actuator's ID, raising a clear error if missing."""
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if idx < 0:
        raise RuntimeError(
            f"Actuator '{name}' not found. Available actuators: "
            f"{[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]}"
        )
    return idx


def joint_qpos_adr(model, joint_name: str) -> int:
    """Find a named joint's qpos start index, raising on missing."""
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if idx < 0:
        raise RuntimeError(
            f"Joint '{joint_name}' not found. Available joints: "
            f"{[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]}"
        )
    return int(model.jnt_qposadr[idx])


def body_freejoint_qpos_adr(model, body_name: str) -> int:
    """Find a body's freejoint qpos slot. Used for the obstacle, whose
    freejoint is unnamed in the template (``<joint type="free"/>``) so
    ``mj_name2id`` on the joint won't find it. Look up via the body name
    instead.

    Raises RuntimeError if the body doesn't exist or doesn't have a
    freejoint.
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise RuntimeError(f"Body '{body_name}' not found in model.")
    if model.body_jntnum[body_id] < 1:
        raise RuntimeError(f"Body '{body_name}' has no joints.")
    jnt_idx = int(model.body_jntadr[body_id])
    if int(model.jnt_type[jnt_idx]) != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError(
            f"Body '{body_name}'s first joint is not a freejoint."
        )
    return int(model.jnt_qposadr[jnt_idx])


def park_obstacle_far_away(
    model, data, body_name: str = "obstacle_1_movable",
    park_xy_m: tuple = (5.0, 5.0),
) -> None:
    """Teleport the obstacle far from the car so chassis-motion testing
    isn't perturbed by contact.

    The template scene loads the car at (0, 0, 0.01) and the obstacle at
    (0, 0, 0.05) — overlapping. For primitive generation that's fine
    because the generator repositions both before each run. For our
    chassis test we want a clean free-driving car, so we punt the
    obstacle to ``park_xy_m`` (5 m in +X, +Y by default — comfortably
    outside the scene's walls at ±0.83 m, but still inside the world
    bounds MuJoCo cares about).
    """
    obs_adr = body_freejoint_qpos_adr(model, body_name)
    data.qpos[obs_adr + 0] = park_xy_m[0]
    data.qpos[obs_adr + 1] = park_xy_m[1]
    # leave z, qw=1, qx/y/z=0 as-is; recompute kinematics so the move
    # takes effect for the next step.
    mujoco.mj_forward(model, data)


def read_car_pose(data, car_qpos_adr: int, t_s: float) -> PoseSample:
    """Extract car pose from qpos at the car's freejoint slot.

    Layout (free joint, 7 qpos): [x, y, z, qw, qx, qy, qz] in metres /
    unit quaternion. We convert to cm and degrees and assume the chassis
    stays flat (qx ≈ qy ≈ 0).
    """
    qpos = data.qpos
    x_m, y_m, _z_m = qpos[car_qpos_adr], qpos[car_qpos_adr + 1], qpos[car_qpos_adr + 2]
    qw, qz = qpos[car_qpos_adr + 3], qpos[car_qpos_adr + 6]
    return PoseSample(
        t_s=t_s,
        x_cm=float(x_m) * 100.0,
        y_cm=float(y_m) * 100.0,
        theta_deg=quat_z_to_theta_deg(float(qw), float(qz)),
    )


# ─────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).resolve()

    if not args.no_default_speed and args.left_frac is None:
        args.left_frac = default_push_fraction()
    if not args.no_default_speed and args.right_frac is None:
        args.right_frac = default_push_fraction()
    if args.left_frac is None or args.right_frac is None:
        print(
            "[sim] --left-frac and --right-frac must both be set when "
            "--no-default-speed is used.",
            file=sys.stderr,
        )
        return 2

    xml_path = Path(args.xml).resolve()
    if not xml_path.exists():
        print(f"[sim] XML not found: {xml_path}", file=sys.stderr)
        return 1

    print(f"[sim] Loading scene {xml_path.name}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    left_actuator = actuator_id(model, "left_wheel_drive")
    right_actuator = actuator_id(model, "right_wheel_drive")
    car_qpos_adr = joint_qpos_adr(model, "car_freejoint")

    # The template loads the obstacle at (0, 0) — same as the car. For a
    # chassis-motion test we don't want any obstacle contact, so park
    # the obstacle far outside the driving range before driving.
    park_obstacle_far_away(model, data)

    # Convert wheel fractions → wheel angular velocities (rad/s).
    omega_left = (args.left_frac * args.k_car_wheel_max_speed_ms) / args.wheel_radius_m
    omega_right = (args.right_frac * args.k_car_wheel_max_speed_ms) / args.wheel_radius_m

    timestep_s = float(model.opt.timestep)
    total_ticks = int(round(args.duration / timestep_s))
    sample_every = max(1, int(round((1.0 / args.target_sample_hz) / timestep_s)))

    print(
        f"[sim] Driving for {args.duration}s "
        f"({total_ticks} ticks @ {timestep_s*1000:.1f} ms/tick): "
        f"left_frac={args.left_frac:+.3f} → ω_L={omega_left:+.3f} rad/s, "
        f"right_frac={args.right_frac:+.3f} → ω_R={omega_right:+.3f} rad/s. "
        f"Sampling every {sample_every} ticks "
        f"(~{1.0/(sample_every*timestep_s):.2f} Hz)."
    )

    poses: list[PoseSample] = []
    for tick in range(total_ticks):
        data.ctrl[left_actuator] = omega_left
        data.ctrl[right_actuator] = omega_right
        mujoco.mj_step(model, data)
        if tick % sample_every == 0:
            poses.append(read_car_pose(data, car_qpos_adr, t_s=tick * timestep_s))

    meta = TrialMeta(
        trial_kind=infer_trial_kind(args.left_frac, args.right_frac),
        source="sim",
        left_cmd=float(args.left_frac),
        right_cmd=float(args.right_frac),
        cmd_units="fraction",
        duration_s=float(args.duration),
        started_at_utc_iso=dt.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        n_samples=len(poses),
        sim_meta={
            "xml": str(xml_path),
            "timestep_s": timestep_s,
            "sample_every_ticks": sample_every,
            "target_sample_hz": args.target_sample_hz,
            "k_car_wheel_max_speed_ms": args.k_car_wheel_max_speed_ms,
            "wheel_radius_m": args.wheel_radius_m,
            "omega_left_rad_s": omega_left,
            "omega_right_rad_s": omega_right,
            "mujoco_version": mujoco.__version__,
            "default_speed_source": (
                "controller.yaml:push.max_speed"
                if not args.no_default_speed
                and args.left_frac == default_push_fraction()
                else "cli"
            ),
        },
    )
    write_trial(out_dir, meta, poses)
    print(f"[sim] Wrote {len(poses)} samples → {out_dir}")
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--out-dir",
        required=True,
        help="Trial directory to write (poses.jsonl + trial_meta.json).",
    )
    p.add_argument(
        "--left-frac",
        type=float,
        default=None,
        help="Left wheel fraction in [-1, 1]. If unset, defaults to "
        "controller.yaml: push.max_speed (mirrors real's PWM number).",
    )
    p.add_argument(
        "--right-frac",
        type=float,
        default=None,
        help="Right wheel fraction in [-1, 1]. Same default as --left-frac.",
    )
    p.add_argument(
        "--no-default-speed",
        action="store_true",
        help="Disable the controller.yaml default; require explicit "
        "--left-frac and --right-frac.",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Sim seconds to drive (default: 10).",
    )
    p.add_argument(
        "--xml",
        default=str(DEFAULT_XML),
        help=f"Scene XML to load. Default: {DEFAULT_XML.name} from namo_cpp/data/.",
    )
    p.add_argument(
        "--target-sample-hz",
        type=float,
        default=DEFAULT_TARGET_SAMPLE_HZ,
        help=f"Sub-sample qpos at ~this many Hz (default: {DEFAULT_TARGET_SAMPLE_HZ}).",
    )
    p.add_argument(
        "--k-car-wheel-max-speed-ms",
        type=float,
        default=DEFAULT_K_CAR_WHEEL_MAX_SPEED_MS,
        help="Mirror of the C++ compiled constant kCarWheelMaxSpeedMs "
        "(m/s when wheel fraction = 1.0). Default: %(default)s. "
        "MUST match the value used to build namo_cpp/namo_rl.",
    )
    p.add_argument(
        "--wheel-radius-m",
        type=float,
        default=DEFAULT_WHEEL_RADIUS_M,
        help="Wheel radius in metres (default: %(default)s, matches "
        "little_car.xml).",
    )
    return p.parse_args()


if __name__ == "__main__":
    raise SystemExit(main())
