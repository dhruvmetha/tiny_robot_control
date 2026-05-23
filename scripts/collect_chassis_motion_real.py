"""Drive the real robot at a fixed wheel command for N seconds, log every
camera pose to disk.

For chassis-velocity calibration. One trial per invocation.

Usage:
    # Straight line at the default push speed (push.max_speed from
    # controller.yaml — currently 0.2 PWM).
    python scripts/collect_chassis_motion_real.py \\
        --out-dir chassis_calibration/<session>/real/straight_cmd0p20

    # Override the wheel command:
    python scripts/collect_chassis_motion_real.py \\
        --left-pwm 0.3 --right-pwm 0.3 \\
        --duration 10 \\
        --out-dir chassis_calibration/<session>/real/straight_cmd0p30

    # Turn in place (left wheel backward, right wheel forward → CCW):
    python scripts/collect_chassis_motion_real.py \\
        --left-pwm -0.2 --right-pwm 0.2 \\
        --out-dir chassis_calibration/<session>/real/turn_cmd0p20_ccw

The default value for --left-pwm / --right-pwm comes from
``config/controller.yaml: push.max_speed`` — the same speed at which
production push primitives drive the wheels. Override on the CLI to
sweep other speeds.
"""

from __future__ import annotations

import argparse
import datetime as dt
import signal
import sys
import time
from pathlib import Path
from typing import Optional

# Make in-tree imports work whether or not the package is pip-install -e'd.
HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import yaml

from robot_control.controller.config import load_controller_configs
from robot_control.core.types import Action
from robot_control.diagnostics.chassis_motion import (
    PoseSample,
    TrialMeta,
    infer_trial_kind,
    pose_dedupe,
    write_trial,
)
from robot_control.environment.real import RealEnv, RealEnvConfig
from robot_control.nodes.remote_observer import RemoteObserverNode


# ─────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────


def default_push_pwm() -> float:
    """Read ``push.max_speed`` from the controller YAML config.

    This is the wheel-PWM value run_namo.py drives the robot at during a
    push primitive. We use it as the default calibration speed so the
    chassis test runs at the same speed production push runs at.

    Falls back to the dataclass default (0.3) if the YAML can't be loaded.
    """
    return load_controller_configs().push.max_speed


def load_serial_config(config_path: Path) -> RealEnvConfig:
    """Build a RealEnvConfig from the ``serial:`` block of real.yaml.

    Mirrors the loader path in runtime.py:_load_real_config but lighter —
    we don't need camera / observer config for chassis testing.
    """
    data = yaml.safe_load(config_path.read_text()) or {}
    serial = data.get("serial", {})
    robot = data.get("robot", {})
    return RealEnvConfig(
        port=serial.get("port", "/dev/ttyACM0"),
        baudrate=int(serial.get("baudrate", 115200)),
        send_hz=float(serial.get("send_hz", 30.0)),
        invert_right_wheel=bool(serial.get("invert_right_wheel", False)),
        robot_id=int(robot.get("marker_id", 1)),
    )


def wait_for_camera(
    observer: RemoteObserverNode, timeout_s: float = 10.0
) -> "Observation":
    """Block until the first valid observation arrives. Raise on timeout."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        obs = observer.get()
        if obs is not None:
            return obs
        time.sleep(0.05)
    raise TimeoutError(
        f"No observation from camera_service within {timeout_s} s. "
        "Is camera_service.py running and is the robot marker visible?"
    )


# ─────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).resolve()

    if not args.no_default_speed and args.left_pwm is None:
        args.left_pwm = default_push_pwm()
    if not args.no_default_speed and args.right_pwm is None:
        args.right_pwm = default_push_pwm()
    if args.left_pwm is None or args.right_pwm is None:
        print(
            "[real] --left-pwm and --right-pwm must both be set when "
            "--no-default-speed is used.",
            file=sys.stderr,
        )
        return 2

    # 1. Connect to camera service.
    print(f"[real] Connecting to camera_service at {args.camera_service} ...")
    observer = RemoteObserverNode(address=args.camera_service, object_sizes={})
    observer.start()

    env: Optional[RealEnv] = None
    e_stop_installed = False

    try:
        # 2. Wait for camera; capture t0 + the at-rest starting pose.
        # The starting pose is recorded explicitly in trial_meta so the
        # trial is fully self-describing — the analyzer never needs to
        # assume the robot started at any specific position. Velocity
        # computation is invariant to absolute starting pose anyway
        # (it's a slope of pose-vs-time, not absolute positions).
        first_obs = wait_for_camera(observer, timeout_s=10.0)
        t0 = first_obs.timestamp
        start_pose_at_rest = (
            float(first_obs.robot_x),
            float(first_obs.robot_y),
            float(first_obs.robot_theta),
        )
        print(
            f"[real] Robot at rest: ({start_pose_at_rest[0]:.2f}, "
            f"{start_pose_at_rest[1]:.2f}) θ={start_pose_at_rest[2]:.2f}°. "
            f"Trial starts from THIS pose — no operator repositioning needed."
        )

        # 3. Bring up serial.
        env_config = load_serial_config(Path(args.config).resolve())
        env = RealEnv(env_config)
        if not env.start():
            print("[real] RealEnv failed to start (serial down?)", file=sys.stderr)
            return 1

        # Install Ctrl+C handler that stops the robot before exiting.
        def _stop_on_sigint(signum, frame):
            print("\n[real] SIGINT — stopping wheels.", flush=True)
            if env is not None:
                try:
                    env.apply(Action(left_speed=0.0, right_speed=0.0))
                except Exception:
                    pass
            sys.exit(130)

        signal.signal(signal.SIGINT, _stop_on_sigint)
        e_stop_installed = True

        # 4. Send the drive command, log obs while waiting.
        print(
            f"[real] Driving for {args.duration}s: "
            f"left_pwm={args.left_pwm:+.3f}, right_pwm={args.right_pwm:+.3f}"
        )
        env.apply(Action(left_speed=args.left_pwm, right_speed=args.right_pwm))

        poses: list[PoseSample] = []
        last_ts: Optional[float] = None
        deadline = time.time() + args.duration
        while time.time() < deadline:
            obs = observer.get()
            if obs is None or obs.timestamp == last_ts:
                time.sleep(0.001)
                continue
            last_ts = obs.timestamp
            poses.append(
                PoseSample(
                    t_s=obs.timestamp - t0,
                    x_cm=obs.robot_x,
                    y_cm=obs.robot_y,
                    theta_deg=obs.robot_theta,
                )
            )

        # 5. Stop the robot. Linger a moment for inertia, then capture
        # the at-rest end pose so the trial records "where the robot
        # actually finished" rather than implying anything from the
        # operator's choice of teardown position.
        env.apply(Action(left_speed=0.0, right_speed=0.0))
        time.sleep(0.5)
        last_obs = observer.get()
        end_pose_after_drive = (
            (float(last_obs.robot_x), float(last_obs.robot_y), float(last_obs.robot_theta))
            if last_obs is not None
            else None
        )
        if end_pose_after_drive is not None:
            print(
                f"[real] Robot at rest after drive: "
                f"({end_pose_after_drive[0]:.2f}, "
                f"{end_pose_after_drive[1]:.2f}) "
                f"θ={end_pose_after_drive[2]:.2f}°."
            )

        # 6. Dedupe + write.
        poses = pose_dedupe(poses)
        if len(poses) < 5:
            print(
                f"[real] WARNING: only {len(poses)} samples landed. "
                "Camera service may not have been running, or detection "
                "failed during motion. Writing anyway for diagnosis.",
                file=sys.stderr,
            )

        meta = TrialMeta(
            trial_kind=infer_trial_kind(args.left_pwm, args.right_pwm),
            source="real",
            left_cmd=float(args.left_pwm),
            right_cmd=float(args.right_pwm),
            cmd_units="pwm",
            duration_s=float(args.duration),
            started_at_utc_iso=(
                dt.datetime.utcnow().isoformat(timespec="seconds") + "Z"
            ),
            n_samples=len(poses),
            real_meta={
                "camera_service": args.camera_service,
                "config": str(Path(args.config).resolve()),
                "serial_port": env_config.port,
                "send_hz": env_config.send_hz,
                "default_speed_source": (
                    "controller.yaml:push.max_speed"
                    if not args.no_default_speed and args.left_pwm == default_push_pwm()
                    else "cli"
                ),
                # Both captured from camera before/after the drive command
                # — independent of operator pre-positioning. Velocity fits
                # use pose deltas (slope of x/y/theta vs time), so these
                # are diagnostic, not load-bearing.
                "start_pose_at_rest_cm_deg": list(start_pose_at_rest),
                "end_pose_after_drive_cm_deg": (
                    list(end_pose_after_drive)
                    if end_pose_after_drive is not None
                    else None
                ),
            },
        )
        write_trial(out_dir, meta, poses)
        print(f"[real] Wrote {len(poses)} samples → {out_dir}")
        return 0

    finally:
        if env is not None:
            try:
                env.apply(Action(left_speed=0.0, right_speed=0.0))
            except Exception:
                pass
            time.sleep(0.2)
            env.stop()
        observer.stop()


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
        "--left-pwm",
        type=float,
        default=None,
        help="Left wheel PWM in [-1, 1]. If unset, defaults to "
        "controller.yaml: push.max_speed (the speed run_namo.py uses).",
    )
    p.add_argument(
        "--right-pwm",
        type=float,
        default=None,
        help="Right wheel PWM in [-1, 1]. Same default as --left-pwm.",
    )
    p.add_argument(
        "--no-default-speed",
        action="store_true",
        help="Disable the controller.yaml default; require explicit "
        "--left-pwm and --right-pwm.",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds to hold the wheel command (default: 10).",
    )
    p.add_argument(
        "--camera-service",
        default="tcp://localhost:5556",
        help="ZMQ PUB address of the camera service.",
    )
    p.add_argument(
        "--config",
        default="config/real.yaml",
        help="Real robot config (for serial port). Default: config/real.yaml.",
    )
    return p.parse_args()


if __name__ == "__main__":
    raise SystemExit(main())
