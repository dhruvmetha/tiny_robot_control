"""Drive the real robot along a straight pure-pursuit path of length L for
N trials. Log per-frame poses + commanded actions per trial. Compute the
median forward-cruise velocity at the end.

Used for Tier-1 SIM2REAL calibration: the median + MAD-SE of these trials
becomes the *target* and *tolerance* for the sim binary-search script
(``find_sim_fraction.py``).

Why pure-pursuit instead of open-loop fixed PWM:
    Open-loop runs are contaminated by wheel-asymmetry drift, which can
    flip sign trial-to-trial. With pure pursuit, the controller actively
    corrects drift; the measured *forward progress along the path*
    becomes a clean, low-variance velocity signal.

Visual: a Qt window stays open the whole session and shows live robot
pose + planned path + lookahead target — same drawings the run_namo.py
window uses.

Usage:
    python scripts/collect_pp_trial_real.py \\
        --session-dir chassis_calibration/<timestamp>_pp_session \\
        --n-trials 5
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

# Make in-tree imports work whether or not the package is pip-install -e'd.
HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import yaml

from robot_control.camera import make_real_workspace_config
from robot_control.controller.config import load_controller_configs
from robot_control.controller.follow_path import FollowPathController
from robot_control.core.types import Action, NavigateSubgoal, Observation
from robot_control.utils.robot_geometry import effective_robot_size_cm
from robot_control.diagnostics.chassis_motion import (
    CommandSample,
    PoseSample,
    TrialMeta,
    cruise_forward_velocity,
    median_with_mad_se,
    pose_dedupe,
    write_command_log,
    write_trial,
)
from robot_control.environment.real import RealEnv, RealEnvConfig
from robot_control.gui.window import Window
from robot_control.nodes.remote_observer import RemoteObserverNode


# ─────────────────────────────────────────────────────────────────────────
# Constants — see CLAUDE.md "no magic constants"
# ─────────────────────────────────────────────────────────────────────────

# Lookahead + goal-tolerance ratios. Mirrors what NavigationController
# (controller/navigation.py:164-168) gives FollowPathController, NOT what
# PushController does. Why nav, not push:
#   - PushController uses lookahead = 0.3 × car_size = 2.1 cm, very short.
#     This is tuned for the push *contact* phase, where the object's
#     friction damps lateral wheel oscillation. Free-running with such
#     a short lookahead amplifies any camera-heading noise into visible
#     wheel-command swings (PP κ = 2 sin(α)/L grows as L shrinks).
#   - NavigationController leaves lookahead at FollowPathController's
#     default (0.5 × car_size = 3.5 cm) and runs fine in free space —
#     the user observes nav has no wiggle.
# This calibration is a free-running chassis-velocity test (no object
# contact), so we use the nav-style settings to avoid the wiggle artifact.
# The wheel-to-chassis-velocity mapping we're calibrating is the same
# regardless of which controller hosts the PP, so the result still
# applies to push_tracker_max_speed.
REAL_LOOKAHEAD_RATIO = 1.0   # = 7 cm with 7 cm robot. Long-lookahead choice
                             # for a free-running straight-path test, per
                             # the PP literature (small lookahead causes
                             # oscillation on straight segments). Production
                             # nav uses 0.5×, push uses 0.3×; we go longer
                             # because we have no contact damping AND no
                             # path curvature to worry about.
REAL_GOAL_TOLERANCE_RATIO = 0.2

# Control-loop frequency. Set to the measured camera publish rate
# (~14.98 Hz) rather than runtime.py's nominal 30 Hz. Why: when the
# control loop ticks faster than the camera publishes, half the ticks
# reuse a stale obs. FollowPathController computes cte_dot from
# Δt = obs.timestamp − prev_cte_time, so a stale-obs tick has Δt = 0,
# the cte_dot=0 special-case fires, and the smoothed cte_dot_filt gets
# attenuated by (1-α)=0.75 every stale tick — quietly halving PD term
# responsiveness. Running at the actual obs rate keeps Δt = camera
# period (~67 ms) on every tick and the PD math stays clean.
#
# This deviates from RuntimeConfig.frequency (default 30 Hz in
# runtime.py:147). For our calibration the wheel-command profile is what
# matters, and that profile is determined by when obs updates fire — so
# matching the obs rate produces the same wheel commands as production
# does *when production sees a fresh obs*, with no stale-tick distortion.
CONTROL_LOOP_HZ = 15.0

# Brief lingering wait after stopping wheels at end-of-trial so the
# camera captures the at-rest end pose for the trial metadata.
END_OF_TRIAL_LINGER_S = 0.5


# ─────────────────────────────────────────────────────────────────────────
# Config loaders
# ─────────────────────────────────────────────────────────────────────────


def load_real_env_config(config_path: Path) -> Tuple[RealEnvConfig, dict]:
    """Build a RealEnvConfig from real.yaml. Returns (config, raw_yaml)."""
    raw = yaml.safe_load(config_path.read_text()) or {}
    serial = raw.get("serial", {})
    robot = raw.get("robot", {})
    cfg = RealEnvConfig(
        port=serial.get("port", "/dev/ttyACM0"),
        baudrate=int(serial.get("baudrate", 115200)),
        send_hz=float(serial.get("send_hz", 30.0)),
        invert_right_wheel=bool(serial.get("invert_right_wheel", False)),
        robot_id=int(robot.get("marker_id", 1)),
    )
    return cfg, raw


def default_push_max_speed() -> float:
    """Read push.max_speed from controller.yaml — the production push PWM."""
    return load_controller_configs().push.max_speed


# ─────────────────────────────────────────────────────────────────────────
# Camera helpers
# ─────────────────────────────────────────────────────────────────────────


def wait_for_camera(observer: RemoteObserverNode, timeout_s: float = 10.0) -> Observation:
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
# Single-trial execution
# ─────────────────────────────────────────────────────────────────────────


def run_one_trial(
    env: RealEnv,
    observer: RemoteObserverNode,
    window: Window,
    controller: FollowPathController,
    path_length_cm: float,
    timeout_s: float,
    e_stop_event: threading.Event,
) -> dict:
    """Run a single PP trial; returns a dict the session manager can save.

    The path is built from the current robot pose: it always points in the
    robot's current heading direction so no pre-rotation is needed (heading
    error ≈ 0 at t=0 → FollowPathController's ALIGN mode never triggers).
    """
    first_obs = wait_for_camera(observer)
    t0 = first_obs.timestamp
    x0, y0, theta0 = (
        float(first_obs.robot_x),
        float(first_obs.robot_y),
        float(first_obs.robot_theta),
    )

    theta0_rad = math.radians(theta0)
    ux, uy = math.cos(theta0_rad), math.sin(theta0_rad)
    goal_x = x0 + path_length_cm * ux
    goal_y = y0 + path_length_cm * uy
    path: List[Tuple[float, float]] = [(x0, y0), (goal_x, goal_y)]

    controller.reset()
    controller.set_path(path)

    print(
        f"[trial] start=({x0:.1f}, {y0:.1f}) θ={theta0:.1f}°  "
        f"goal=({goal_x:.1f}, {goal_y:.1f})  L={path_length_cm:.0f} cm  "
        f"timeout={timeout_s:.0f}s"
    )

    poses: List[PoseSample] = []
    commands: List[CommandSample] = []
    dummy_subgoal = NavigateSubgoal(x=goal_x, y=goal_y)
    goal_reached = False
    time_to_goal_s: Optional[float] = None
    # Initialize so the timed-out print at the bottom can reference it
    # even if no observation arrives during the trial (the first
    # wait_for_camera already succeeded, but the stream could drop after).
    dist_to_goal = float("inf")
    deadline = time.time() + timeout_s

    # Fixed-rate control loop, matching runtime.py:960-1030 (production).
    # Each tick: latest cached obs (even if unchanged), step PP, apply,
    # log, then sleep to maintain CONTROL_LOOP_HZ. Variable-rate stepping
    # was the previous behavior; it fed non-constant Δt into PP's CTE-PD
    # derivative and produced different wheel commands than production.
    dt = 1.0 / CONTROL_LOOP_HZ

    while time.time() < deadline:
        loop_start = time.time()

        # E-stop early-exit.
        if e_stop_event.is_set():
            print("[trial] e-stop set — aborting trial.")
            break

        # Get LATEST cached obs. Production uses self._world.get() which
        # returns the most recent obs, not necessarily a fresh one. Same
        # behavior: PP sees stale obs if camera is slow, controller still
        # ticks at CONTROL_LOOP_HZ.
        obs = observer.get()
        if obs is None:
            # No obs yet at all — skip this tick, maintain rate.
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            continue

        # Step controller
        action = controller.step(obs, dummy_subgoal)

        # Track distance for the timed-out diagnostic print only.
        dist_to_goal = math.hypot(obs.robot_x - goal_x, obs.robot_y - goal_y)

        # Goal: use controller status so we honor its goal_tolerance
        # (0.3 × car_size = 2.1 cm) exactly.
        if controller.get_status() == "FINISHED":
            goal_reached = True
            time_to_goal_s = obs.timestamp - t0
            break

        env.apply(action)

        # Log every tick — matches the per-tick "drawings + canvas update"
        # cadence in runtime.py. Note: log timestamp is the obs's, not
        # wall time. If the obs is stale (camera dropped a frame), two
        # successive samples may share the same timestamp; cruise_forward_velocity
        # sorts/dedupes upstream so this is harmless.
        t_rel = obs.timestamp - t0
        poses.append(
            PoseSample(
                t_s=t_rel,
                x_cm=float(obs.robot_x),
                y_cm=float(obs.robot_y),
                theta_deg=float(obs.robot_theta),
            )
        )
        commands.append(
            CommandSample(
                t_s=t_rel,
                left_cmd=float(action.left_speed),
                right_cmd=float(action.right_speed),
                mode=str(controller.metadata.get("mode", "")),
            )
        )

        # Live display
        window.update(obs)
        window.update_drawings(controller.get_drawings())

        # Maintain control-loop rate (mirrors runtime.py:1028-1030).
        elapsed = time.time() - loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    # Stop wheels & let inertia settle so the at-rest end pose is captured.
    env.apply(Action.stop())
    time.sleep(END_OF_TRIAL_LINGER_S)
    end_obs = observer.get()
    end_pose: Optional[Tuple[float, float, float]] = (
        (float(end_obs.robot_x), float(end_obs.robot_y), float(end_obs.robot_theta))
        if end_obs is not None
        else None
    )

    if goal_reached:
        print(f"[trial] ✓ goal reached in {time_to_goal_s:.2f}s")
    else:
        print(f"[trial] ⏱  timed out after {timeout_s:.0f}s "
              f"(dist_to_goal={dist_to_goal:.1f} cm)")

    return {
        "poses": pose_dedupe(poses),
        "commands": commands,
        "path": path,
        "start_pose": (x0, y0, theta0),
        "end_pose": end_pose,
        "goal_reached": goal_reached,
        "time_to_goal_s": time_to_goal_s,
        "path_unit_vec": (ux, uy),
    }


# ─────────────────────────────────────────────────────────────────────────
# Live-monitor thread: keeps the canvas updated between trials
# ─────────────────────────────────────────────────────────────────────────


def monitor_loop(
    observer: RemoteObserverNode,
    window: Window,
    stop_event: threading.Event,
) -> None:
    """Feed observations into the window so the operator can see live
    robot position while repositioning between trials. Stops when
    ``stop_event`` is set.

    We only push the empty-drawings clear ONCE at start, not every iter,
    to avoid spamming Qt with no-op signals during the pause window.
    """
    last_ts: Optional[float] = None
    window.update_drawings([])  # one-shot clear of the previous trial's path
    while not stop_event.is_set():
        obs = observer.get()
        if obs is not None and obs.timestamp != last_ts:
            last_ts = obs.timestamp
            window.update(obs)
        time.sleep(1.0 / 20.0)  # 20 Hz monitor rate (gentler than trial loop)


# ─────────────────────────────────────────────────────────────────────────
# Session runner — N trials with pauses between
# ─────────────────────────────────────────────────────────────────────────


def session_runner(
    env: RealEnv,
    observer: RemoteObserverNode,
    window: Window,
    workspace_config,
    args: argparse.Namespace,
    session_dir: Path,
    e_stop_event: threading.Event,
) -> None:
    """Background thread: drive all N trials, save each, compute summary."""
    # Match PushController's construction (controller/push.py:178-183):
    # explicit lookahead_distance + goal_tolerance at 0.3 × car_size,
    # otherwise FollowPathController would use its own class defaults
    # (0.5 × / 0.2 ×) which is NOT what production does during pushes.
    # Use the same effective_robot_size_cm helper PushController uses so
    # there's zero chance of formula drift.
    car_size_cm = effective_robot_size_cm(workspace_config.car_width, workspace_config.car_height)
    controller = FollowPathController(
        config=workspace_config,
        lookahead_distance=REAL_LOOKAHEAD_RATIO * car_size_cm,
        goal_tolerance=REAL_GOAL_TOLERANCE_RATIO * car_size_cm,
        max_speed=args.max_speed,
    )

    summaries: List[dict] = []

    for trial_idx in range(1, args.n_trials + 1):
        if e_stop_event.is_set():
            print("[session] e-stop requested, aborting.")
            break

        print(f"\n{'=' * 60}")
        print(f"  TRIAL {trial_idx} / {args.n_trials}")
        print(f"{'=' * 60}")
        print("  Position the robot with ~50 cm of clear runway in front.")
        print("  Press Enter to start. (Ctrl+C to abort the session.)")

        # Start live-pose monitor so operator can see current robot pose.
        stop_monitor = threading.Event()
        monitor = threading.Thread(
            target=monitor_loop,
            args=(observer, window, stop_monitor),
            daemon=True,
            name=f"monitor-trial-{trial_idx}",
        )
        monitor.start()
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\n[session] aborted at trial pause.")
            stop_monitor.set()
            break
        stop_monitor.set()
        monitor.join(timeout=0.5)
        # Clear leftover drawings before we draw the new path
        window.update_drawings([])

        # Run the trial
        try:
            result = run_one_trial(
                env=env,
                observer=observer,
                window=window,
                controller=controller,
                path_length_cm=args.path_length_cm,
                timeout_s=args.timeout_s,
                e_stop_event=e_stop_event,
            )
        except Exception as exc:
            print(f"[trial {trial_idx}] FAILED: {exc!r}")
            env.apply(Action.stop())
            continue

        # Save trial dir
        trial_dir = session_dir / "real" / f"trial_{trial_idx:02d}"
        save_trial(
            trial_dir=trial_dir,
            result=result,
            max_speed=args.max_speed,
        )

        # Compute this trial's metric
        try:
            cruise = cruise_forward_velocity(
                result["poses"],
                path_start_cm=(result["start_pose"][0], result["start_pose"][1]),
                path_unit_vec=result["path_unit_vec"],
            )
            cruise.goal_reached = result["goal_reached"]
            cruise.time_to_goal_s = result["time_to_goal_s"]
            print(
                f"[trial {trial_idx}] cruise={cruise.forward_cruise_cm_s:.2f} cm/s, "
                f"arc_len={cruise.arc_length_cm:.1f} cm, "
                f"n_cruise_samples={cruise.n_cruise_samples}"
            )
            summaries.append(
                {
                    "trial_idx": trial_idx,
                    "forward_cruise_cm_s": cruise.forward_cruise_cm_s,
                    "arc_length_cm": cruise.arc_length_cm,
                    "n_cruise_samples": cruise.n_cruise_samples,
                    "cruise_window_s": cruise.cruise_window_s,
                    "goal_reached": cruise.goal_reached,
                    "time_to_goal_s": cruise.time_to_goal_s,
                }
            )
        except ValueError as exc:
            print(f"[trial {trial_idx}] metric computation failed: {exc}")
            summaries.append({"trial_idx": trial_idx, "error": str(exc)})

    # Session summary
    velocities = [
        s["forward_cruise_cm_s"]
        for s in summaries
        if "forward_cruise_cm_s" in s
    ]
    summary = {
        "session_dir": str(session_dir.resolve()),
        "max_speed_used": float(args.max_speed),
        "path_length_cm": float(args.path_length_cm),
        "n_trials_requested": int(args.n_trials),
        "n_trials_completed": len(summaries),
        "n_trials_with_metric": len(velocities),
        "per_trial": summaries,
    }
    if velocities:
        median, se = median_with_mad_se(velocities)
        summary["median_forward_cruise_cm_s"] = median
        summary["se_median_cm_s"] = se
        print(
            f"\n[session] DONE. "
            f"median={median:.2f} cm/s, SE(median)={se:.2f} cm/s, "
            f"n={len(velocities)}"
        )
    else:
        print("\n[session] DONE — but no valid trials. Nothing to summarize.")

    summary_path = session_dir / "real" / "session_summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"[session] summary → {summary_path}")
    print("[session] Close the window to exit.")


# ─────────────────────────────────────────────────────────────────────────
# Per-trial disk I/O
# ─────────────────────────────────────────────────────────────────────────


def save_trial(
    trial_dir: Path,
    result: dict,
    max_speed: float,
) -> None:
    """Write poses.jsonl, commands.jsonl, trial_meta.json, path.json."""
    trial_dir.mkdir(parents=True, exist_ok=True)
    poses: List[PoseSample] = result["poses"]
    commands: List[CommandSample] = result["commands"]
    path = result["path"]
    start_pose = result["start_pose"]
    end_pose = result["end_pose"]

    meta = TrialMeta(
        trial_kind="straight",
        source="real",
        left_cmd=float(max_speed),       # nominal cap (controller modulates around this)
        right_cmd=float(max_speed),
        cmd_units="pwm",                 # PP wheel cmds passed to firmware as PWM fraction
        duration_s=float(poses[-1].t_s) if poses else 0.0,
        started_at_utc_iso=(
            dt.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        ),
        n_samples=len(poses),
        real_meta={
            "controller": "FollowPathController",
            "max_speed": float(max_speed),
            "path_start_cm": [start_pose[0], start_pose[1]],
            "path_end_cm": [path[-1][0], path[-1][1]],
            "path_unit_vec": list(result["path_unit_vec"]),
            "start_pose_cm_deg": list(start_pose),
            "end_pose_cm_deg": list(end_pose) if end_pose else None,
            "goal_reached": bool(result["goal_reached"]),
            "time_to_goal_s": result["time_to_goal_s"],
        },
    )
    write_trial(trial_dir, meta, poses)
    write_command_log(trial_dir / "commands.jsonl", commands)
    (trial_dir / "path.json").write_text(
        json.dumps({"path_cm": [list(pt) for pt in path]}, indent=2)
    )


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
        default=None,
        help="Top-level directory for this calibration session. "
        "Defaults to chassis_calibration/<UTC-timestamp>_pp_session.",
    )
    p.add_argument(
        "--n-trials",
        type=int,
        default=5,
        help="Number of real trials to run (default: 5).",
    )
    p.add_argument(
        "--max-speed",
        type=float,
        default=None,
        help="PP max_speed (= max wheel PWM the controller will command). "
        "Default = controller.yaml: push.max_speed.",
    )
    p.add_argument(
        "--path-length-cm",
        type=float,
        default=50.0,
        help="Straight-line path length in cm (default: 50).",
    )
    p.add_argument(
        "--timeout-s",
        type=float,
        default=12.0,
        help="Per-trial timeout in seconds (default: 12).",
    )
    p.add_argument(
        "--camera-service",
        default="tcp://localhost:5556",
        help="ZMQ PUB address of the camera service.",
    )
    p.add_argument(
        "--config",
        default="config/real.yaml",
        help="Real robot config (for serial port, marker, robot dims).",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if args.max_speed is None:
        args.max_speed = default_push_max_speed()
        print(f"[main] --max-speed not set; using push.max_speed = {args.max_speed}")

    if args.session_dir is None:
        ts = dt.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        args.session_dir = f"chassis_calibration/{ts}_pp_session"
    session_dir = Path(args.session_dir).resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f"[main] session dir: {session_dir}")

    # Real env config — gives us serial port + robot geometry
    env_config, raw_yaml = load_real_env_config(Path(args.config).resolve())
    robot = raw_yaml.get("robot", {})
    car_width = float(robot.get("width_cm", 7.0))
    car_height = float(robot.get("height_cm", 7.0))

    # WorkspaceConfig — drives the canvas + FollowPathController
    workspace_config = make_real_workspace_config(
        car_width=car_width,
        car_height=car_height,
        wheel_base=car_width,   # diff-drive car: wheels at ±car_width/2
    )

    # Camera + RealEnv
    print(f"[main] connecting to camera_service at {args.camera_service} ...")
    observer = RemoteObserverNode(address=args.camera_service, object_sizes={})
    observer.start()

    env = RealEnv(env_config)
    if not env.start():
        print("[main] RealEnv failed to start (serial down?)", file=sys.stderr)
        observer.stop()
        return 1

    # Qt window (main thread)
    window = Window(
        workspace_config,
        show_camera=False,
        show_connection=True,
        show_settings=False,
        autonomous=True,
        target_robot_id=env_config.robot_id,
    )
    window.enable_canvas_click(False)
    window.set_controller("PP Calibration")

    # E-stop on SIGINT — also fires when the user Ctrl+C's during a trial loop.
    e_stop = threading.Event()

    def _sigint_handler(signum, frame):
        print("\n[main] SIGINT — stopping wheels.")
        e_stop.set()
        try:
            env.apply(Action.stop())
        except Exception:
            pass

    signal.signal(signal.SIGINT, _sigint_handler)

    # Wake the Qt event loop every 100 ms so Python's signal handlers
    # (notably SIGINT) get a chance to run. Without this, Ctrl+C during
    # window.run() is swallowed by the Qt C++ loop and the handler never
    # fires. The QTimer slot is intentionally a no-op.
    try:
        from PySide6.QtCore import QTimer
        _sigint_pulse = QTimer()
        _sigint_pulse.start(100)
        _sigint_pulse.timeout.connect(lambda: None)
    except ImportError:
        _sigint_pulse = None

    # Session runner in background thread
    bg = threading.Thread(
        target=session_runner,
        args=(env, observer, window, workspace_config, args, session_dir, e_stop),
        daemon=True,
        name="session-runner",
    )
    bg.start()

    # Main thread blocks on Qt loop until user closes the window.
    try:
        rc = window.run()
    finally:
        e_stop.set()
        try:
            env.apply(Action.stop())
        except Exception:
            pass
        time.sleep(0.2)
        env.stop()
        observer.stop()
    return int(rc) if isinstance(rc, int) else 0


if __name__ == "__main__":
    raise SystemExit(main())
