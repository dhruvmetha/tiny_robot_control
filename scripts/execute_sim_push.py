"""Execute a single (edge_idx, depth) push in MuJoCo sim using pure pursuit.

What this does (and what it deliberately does NOT do):

    * Loads a captured MuJoCo XML (the same one ``capture_to_xml.py
      --robot-model car`` writes) and runs *full MuJoCo physics* — robot and
      object actually collide, the obstacle actually moves under push.
    * Drives the robot with the same Python ``PushController`` (pure pursuit
      + CTE-PD, then forward push) that ``run_namo.py`` and
      ``collect_real_primitives.py`` use on the real robot. So the *sim
      and real controllers are byte-identical*; only the environment
      differs.
    * Takes ``--edge-idx`` and ``--depth`` directly on the command line.
      No primitives-DB lookup. No trial-spec YAML. One push per
      invocation, fed straight to the controller.

Why this is *not* ``sim_replay``: ``sim_replay`` calls
``RLEnvironment.step()`` which dispatches to the C++ ``NAMOPushController``,
whose push phase is *open-loop motion-primitive replay* (looking up a
pre-recorded wheel-tick sequence in ``data/car_motion_primitives_*.dat``).
That's not pure pursuit, and the "drift in place" you see during sim_replay
comes from primitive-vs-physics mismatch.

Example:
    # 1) capture the current real scene as a car-format XML
    python scripts/capture_to_xml.py \\
        --camera-service tcp://localhost:5556 \\
        --robot-model car --scale-factor 1.0 \\
        --output /tmp/sim_scene.xml

    # 2) execute edge 29, depth 9 in MuJoCo with a viewer
    python scripts/execute_sim_push.py \\
        --mujoco-xml /tmp/sim_scene.xml \\
        --edge-idx 29 --depth 9 \\
        --mujoco-viewer
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
from pathlib import Path
from typing import Optional, Tuple

import yaml

# Make the script importable both from the repo root and after `pip install -e .`
HERE = Path(__file__).resolve().parent
ROBOT_CONTROL_ROOT = HERE.parent
SRC = ROBOT_CONTROL_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from _diag_setup import bootstrap_diagnostics  # type: ignore  # noqa: E402

# Reuse the YAML-driven planner from collect_real_primitives. We pass it a
# single-trial list synthesised from --edge-idx/--depth; the planner sees the
# same shape as if we'd handed it a one-line trial-spec YAML.
from collect_real_primitives import TrialListPlanner  # noqa: E402

from pubsub import pub  # noqa: E402

from robot_control.controller.edge_points import get_edge_point  # noqa: E402
from robot_control.core.topics import Topics  # noqa: E402
from robot_control.core.types import PushSubgoal  # noqa: E402
from robot_control.environment.sim import SimConfig  # noqa: E402
from robot_control.runtime import Runtime, RuntimeConfig  # noqa: E402
from robot_control.utils import NAMOXMLGenerator  # noqa: E402


# ----------------------------------------------- real-run rewind helpers
#
# The "rewind real" mode reconstructs the pre-push state of a real-side
# ``collect_real_primitives`` run so the sim starts from the exact same
# configuration. Reads three things from the real run directory:
#
#   - pushes.jsonl                  : pushed object's pose at PUSH start
#   - subgoals.jsonl                : robot pose at DISPATCH (≈ same instant)
#   - (plus camera_service obs)     : non-pushed object poses (walls etc.)
#
# We don't reuse a captured XML for the non-pushed objects because the user's
# concern is that *they don't move physically* between real and sim runs —
# so a fresh live observation is the right source for them.


def _load_real_run_state(run_dir: Path) -> dict:
    """Read object/robot starting poses + push parameters from a real run dir.

    Returns a dict with keys: object_id, object_pose_before (x,y,θ),
    robot_pose (x,y,θ), edge_idx, push_steps. Raises with a clear message
    if the directory doesn't have the expected logs.
    """
    pushes_path = run_dir / "pushes.jsonl"
    subgoals_path = run_dir / "subgoals.jsonl"
    if not pushes_path.exists():
        raise FileNotFoundError(f"{pushes_path} not found — was the real run aborted?")
    if not subgoals_path.exists():
        raise FileNotFoundError(f"{subgoals_path} not found")

    pushes = [json.loads(l) for l in pushes_path.read_text().splitlines() if l.strip()]
    subgoals = [json.loads(l) for l in subgoals_path.read_text().splitlines() if l.strip()]
    if not pushes:
        raise RuntimeError(f"{pushes_path} has no records — the real push never completed")
    if not subgoals:
        raise RuntimeError(f"{subgoals_path} has no records")

    push_rec = pushes[0]
    # Match the subgoal record to this push by subgoal_id (typically 1 for
    # a one-trial spec, but be explicit).
    push_sgid = push_rec.get("subgoal_id")
    sg_rec = next((s for s in subgoals if s.get("subgoal_id") == push_sgid), subgoals[0])

    robot_pose = sg_rec.get("dispatched_robot_pose_cm")
    if robot_pose is None:
        raise RuntimeError(
            f"subgoal record missing dispatched_robot_pose_cm — older log format? "
            f"({subgoals_path})"
        )

    return {
        "object_id": push_rec["object_id"],
        "object_pose_before": tuple(push_rec["object_pose_before"]),  # (x, y, θ_deg)
        "robot_pose": tuple(robot_pose),                              # (x, y, θ_deg)
        "edge_idx": int(push_rec["expected_edge"]),
        "push_steps": int(push_rec["expected_push_steps"]),
    }


def _build_rewind_xml(
    real_state: dict,
    camera_service: str,
    real_yaml_path: Path,
    output_xml: Path,
) -> None:
    """Write a MuJoCo car XML matching the pre-push state of a real run.

    The pushed object's pose comes from real's recorded ``object_pose_before``.
    Every other object (walls, other movables) comes from a fresh camera
    observation — they don't move between trials so 'fresh' is accurate.
    The robot pose is baked into the XML the same way capture_to_xml.py
    does it (NAMOXMLGenerator.from_observation), then teleported again at
    MujocoSimEnv construction via mujoco_starting_robot_pose_cm for
    insurance (little_car.xml's freejoint resets the body to its spawn).
    """
    # Reuse the same ZMQ capture helper capture_to_xml.py uses, importing
    # lazily to avoid a hard dep when --from-real-run isn't passed.
    sys.path.insert(0, str(HERE))
    from capture_to_xml import _capture_from_camera_service  # noqa: E402

    objects_yaml = ROBOT_CONTROL_ROOT / "config" / "objects.yaml"
    live_obs = _capture_from_camera_service(camera_service, str(objects_yaml))
    if live_obs is None:
        raise RuntimeError("Failed to capture live observation for non-pushed objects")

    # Build the objects dict NAMOXMLGenerator expects, overriding the pushed
    # object's pose with the real-side "before" pose.
    pushed_id = real_state["object_id"]
    bx, by, bth = real_state["object_pose_before"]

    objects: dict = {}
    for name, o in live_obs.objects.items():
        if name == pushed_id:
            # Rewind: use real's pre-push pose, keep live dims (object shape
            # didn't change between real and now).
            objects[name] = (bx, by, bth, o.width, o.depth, o.height, o.is_static)
        else:
            objects[name] = (o.x, o.y, o.theta, o.width, o.depth, o.height, o.is_static)

    rx, ry, _ = real_state["robot_pose"]

    # Workspace dims from real.yaml — same source capture_to_xml uses.
    ws_data = yaml.safe_load(real_yaml_path.read_text()) if real_yaml_path.exists() else {}
    ws_cfg = (ws_data.get("workspace") or {}) if ws_data else {}
    workspace_w = float(ws_cfg.get("width_cm", 49.0))
    workspace_h = float(ws_cfg.get("height_cm", 77.5))

    # Auto-goal: opposite corner from robot, same fallback as capture_to_xml.
    goal_x = workspace_w - 5 if rx < workspace_w / 2 else 5
    goal_y = workspace_h - 5 if ry < workspace_h / 2 else 5

    gen = NAMOXMLGenerator(scale_factor=1.0, robot_model="car")
    xml_str = gen.from_observation(
        robot_x_cm=rx,
        robot_y_cm=ry,
        objects=objects,
        goal_x_cm=goal_x,
        goal_y_cm=goal_y,
        workspace_bounds_cm=(0.0, workspace_w, 0.0, workspace_h),
    )
    gen.save(xml_str, str(output_xml))
    print(
        f"[rewind] wrote {output_xml}  robot=({rx:.2f},{ry:.2f}) "
        f"{pushed_id}={bx:.2f},{by:.2f},θ={bth:.1f}° "
        f"(+ {len(objects) - 1} other objects from camera)",
        flush=True,
    )


# --------------------------------------------------------------------- main

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--mujoco-xml",
        default=None,
        help="Path to a captured MuJoCo car XML (from capture_to_xml.py "
             "--robot-model car --scale-factor 1.0). Required unless "
             "--from-real-run is set, in which case the XML is built from "
             "the real run's recorded pre-push state.",
    )
    p.add_argument(
        "--edge-idx",
        type=int,
        default=None,
        help="Edge index to push (0..4*points_per_face-1; 0..59 with "
             "points_per_face=15). See scripts/show_edges.py to visualise. "
             "Auto-filled from real's pushes.jsonl when --from-real-run is set.",
    )
    p.add_argument(
        "--depth",
        type=int,
        default=None,
        help="Push depth. push_steps fed to the controller = depth + 1. "
             "Auto-filled from real's pushes.jsonl when --from-real-run is set.",
    )
    p.add_argument(
        "--from-real-run",
        type=str,
        default=None,
        metavar="DIR",
        help="Path to a collect_real_primitives run directory (the one "
             "containing pushes.jsonl + subgoals.jsonl). When set, the "
             "sim starts from the EXACT pre-push state of that real run "
             "— same robot pose, same pushed-object pose — so sim/real "
             "are directly diff-able. Implies --camera-service (needed "
             "for the non-pushed objects, which haven't moved). "
             "Overrides --mujoco-xml, --edge-idx, --depth, --object-id, "
             "--robot-pose with values read from the real run.",
    )
    p.add_argument(
        "--camera-service",
        type=str,
        default="tcp://localhost:5556",
        metavar="ADDRESS",
        help="ZMQ address of camera_service. Only used by --from-real-run "
             "to capture non-pushed objects (walls etc). Default: "
             "tcp://localhost:5556.",
    )
    p.add_argument(
        "--object-id",
        default=None,
        help="Name of the object to push. Defaults to auto-detect when there "
             "is exactly one movable in the scene.",
    )
    p.add_argument(
        "--mujoco-viewer",
        action="store_true",
        help="Spawn a passive MuJoCo viewer window so you can watch the push.",
    )
    p.add_argument(
        "--robot-pose",
        type=float,
        nargs=3,
        metavar=("X_CM", "Y_CM", "THETA_DEG"),
        default=None,
        help="Starting robot pose in workspace coords. Required for car "
             "XMLs (little_car.xml bakes a 0,0 spawn that won't match the "
             "captured scene). Copy this from capture_to_xml.py's summary "
             "line (e.g. 'Robot: (37.97, 5.53) cm @ 83.2°').",
    )
    p.add_argument(
        "--no-gui",
        action="store_true",
        help="Skip the micromvp PyQt window. Independent of --mujoco-viewer.",
    )
    p.add_argument(
        "--config",
        default="config/real.yaml",
        help="real.yaml — only used to pull workspace + robot dimensions so "
             "the controllers see the right workspace bounds. (default: "
             "config/real.yaml)",
    )
    p.add_argument(
        "--diag-path",
        default="/tmp/sim_push",
        help="Root diagnostics directory (default: /tmp/sim_push — throwaway).",
    )
    p.add_argument(
        "--run-name",
        default="sim_push_{date}_{time}",
        help="Subdirectory name under --diag-path.",
    )
    p.add_argument("--push-speed", type=float, default=None,
                   help="Override push.max_speed from controller.yaml [0,1].")
    p.add_argument("--nav-speed", type=float, default=None,
                   help="Override navigation.max_speed from controller.yaml.")
    p.add_argument("--allow-overwrite", action="store_true",
                   help="Reuse existing --run-name directory.")
    return p.parse_args()


def _sim_config_from_real_yaml(path: Path) -> SimConfig:
    """Build a SimConfig matching the workspace + robot dims from real.yaml.

    Only the dimensions are used here — MujocoSimEnv reads everything else
    (robot pose, object poses, object dims) from the loaded XML.
    """
    if not path.exists():
        return SimConfig()
    data = yaml.safe_load(path.read_text()) or {}
    ws = data.get("workspace", {}) or {}
    rb = data.get("robot", {}) or {}
    return SimConfig(
        width=float(ws.get("width_cm", SimConfig.width)),
        height=float(ws.get("height_cm", SimConfig.height)),
        car_width=float(rb.get("width_cm", SimConfig.car_width)),
        car_height=float(rb.get("height_cm", SimConfig.car_height)),
    )


def main() -> int:
    args = parse_args()

    # Same diagnostics bootstrap collect_real_primitives uses.
    recorder, log_file = bootstrap_diagnostics(args)
    if recorder is None or not recorder.enabled:
        print("Error: --diag-path is required.", file=sys.stderr)
        return 2
    args._diagnostics_recorder = recorder

    # --from-real-run mode: rewind sim to the exact pre-push state of a
    # collect_real_primitives run. Auto-fills every CLI arg that's normally
    # required (xml, edge, depth, object_id, robot_pose). Writes an XML to
    # the diag dir so the run is self-contained.
    if args.from_real_run:
        real_dir = Path(args.from_real_run)
        try:
            real_state = _load_real_run_state(real_dir)
        except (FileNotFoundError, RuntimeError) as exc:
            print(f"Error: --from-real-run: {exc}", file=sys.stderr)
            return 2
        rewind_xml = Path(recorder.root) / "rewind_scene.xml"
        try:
            _build_rewind_xml(
                real_state,
                args.camera_service,
                Path(args.config),
                rewind_xml,
            )
        except Exception as exc:
            print(f"Error: --from-real-run: failed to build rewind XML: {exc!r}",
                  file=sys.stderr)
            return 2
        # Overlay onto args so the rest of main() doesn't need branching.
        args.mujoco_xml = str(rewind_xml)
        args.edge_idx = real_state["edge_idx"]
        args.depth = real_state["push_steps"] - 1
        args.object_id = real_state["object_id"]
        args.robot_pose = list(real_state["robot_pose"])
        print(
            f"[rewind] sim starts at: robot={tuple(args.robot_pose)}  "
            f"object={args.object_id} pose={real_state['object_pose_before']}  "
            f"edge={args.edge_idx} depth={args.depth}",
            flush=True,
        )

    # Now the standard required-arg validation (after rewind has filled them).
    if args.mujoco_xml is None:
        print("Error: --mujoco-xml is required (or use --from-real-run).",
              file=sys.stderr)
        return 2
    if args.edge_idx is None:
        print("Error: --edge-idx is required (or use --from-real-run).",
              file=sys.stderr)
        return 2
    if args.depth is None:
        print("Error: --depth is required (or use --from-real-run).",
              file=sys.stderr)
        return 2
    if args.depth < 0:
        print(f"Error: --depth must be >= 0, got {args.depth}", file=sys.stderr)
        return 2
    push_steps = args.depth + 1

    # Single-trial planner. We don't need the YAML round-trip for one push.
    trial: dict = {"edge_idx": args.edge_idx, "push_steps": push_steps}
    if args.object_id:
        trial["object_id"] = args.object_id
    planner = TrialListPlanner(
        trials=[trial],
        obstacle_target_xy=(0.0, 0.0),
        reset_tolerance_cm=0.0,
        lock_filter=None,
        skip_reset=True,  # sim has no operator-reset between trials
    )

    sim_config = _sim_config_from_real_yaml(Path(args.config))

    # Same Runtime that drives real, but pointed at MujocoSimEnv via
    # mujoco_xml. Pure-pursuit push controller → MuJoCo physics, no C++
    # motion-primitive replay anywhere in the stack.
    runtime_config = RuntimeConfig(
        mode="sim",
        sim_config=sim_config,
        planner=planner,
        quit_on_complete=True,
        show_gui=not args.no_gui,
        nav_speed_override=args.nav_speed,
        push_speed_override=args.push_speed,
        mujoco_xml=args.mujoco_xml,
        mujoco_viewer=args.mujoco_viewer,
        mujoco_starting_robot_pose_cm=(
            tuple(args.robot_pose) if args.robot_pose is not None else None
        ),
        # Always record a video of the sim run alongside the JSONL diags so
        # every invocation produces something visual. Throwaway by default
        # (lives in the same /tmp diag dir as pushes.jsonl).
        mujoco_video_path=str(Path(recorder.root) / "sim_push.mp4"),
    )
    runtime_config.diagnostics_recorder = recorder
    runtime = Runtime(runtime_config)

    # Patch executor.step / set_subgoal to always pull from _world.get(),
    # matching collect_real_primitives.py. Without this the controllers
    # would operate on a snapshot taken before the planner's plan() call.
    #
    # Also: pre-teleport the car to the edge point at the push heading
    # before set_subgoal runs. PushController already has a "skip approach"
    # branch (push.py:312) for when robot is within approach_skip_distance
    # + approach_skip_angle of the target — by teleporting to *exactly* the
    # edge point with *exactly* the right heading, that branch fires every
    # time. Approach phase becomes a no-op; only the pure-pursuit push
    # phase actually runs. Mirrors the C++ NAMOPushController's "teleport
    # approach" pattern (namo_push_controller.cpp:437-441) but with
    # pure-pursuit push instead of motion-primitive replay. This is the
    # only sim execution mode — pure pursuit is for *pushing only*; the
    # nav-via-pure-pursuit alternative is not exposed here.
    orig_setup = runtime._setup

    def _patched_setup():
        orig_setup()
        world = runtime._world
        executor = runtime._executor
        env = runtime._env
        push_ctrl = runtime._controllers.get("push") if runtime._controllers else None
        if world is not None:
            planner.attach_obs_source(world.get)
        if world is not None and executor is not None:
            def _fresh(obs):
                wf = world.get()
                return wf if wf is not None else obs
            orig_step = executor.step
            orig_set = executor.set_subgoal

            def _teleport_to_edge(subgoal, obs):
                """Snap the car to the edge point + push heading before the
                PushController sees the subgoal. The standoff and
                points_per_face values come from PushController so the
                computed edge point matches exactly what PushController
                will compute internally — driving its approach_skip_distance
                check to True and short-circuiting straight to PUSHING.
                Returns a fresh observation reflecting the teleported pose.
                """
                if not isinstance(subgoal, PushSubgoal):
                    return obs
                if env is None or push_ctrl is None:
                    return obs
                if not hasattr(env, "set_robot_pose"):
                    return obs
                obj = obs.objects.get(subgoal.object_id)
                if obj is None:
                    return obs
                ep = get_edge_point(
                    obj,
                    subgoal.edge_idx,
                    push_ctrl._standoff_distance,
                    push_ctrl._points_per_face,
                )
                env.set_robot_pose(ep.position[0], ep.position[1], ep.approach_theta)
                print(f"[teleport-approach] car → ({ep.position[0]:.1f}, "
                      f"{ep.position[1]:.1f}) θ={ep.approach_theta:.1f}° "
                      f"(edge {subgoal.edge_idx}, obj {subgoal.object_id})",
                      flush=True)
                # SubgoalExecutor.set_subgoal(PushSubgoal) just stashes the
                # subgoal — the obs we pass it isn't forwarded to the
                # PushController. PushController only sees an obs on the
                # next executor.step() call, which reads from WorldState.
                # WorldState is updated by SimSensorNode at 30 Hz, so
                # without this republish step() would see a pre-teleport
                # snapshot up to ~33 ms stale, miss the skip-approach
                # branch, and start a full navigation. Force-publish the
                # post-teleport observation so the very next step() call
                # sees the teleported pose.
                post = env.observe()
                pub.sendMessage(Topics.SENSOR_SIM, obs=post)
                return post

            executor.step = lambda obs: orig_step(_fresh(obs))
            executor.set_subgoal = lambda sg, obs: orig_set(sg, _teleport_to_edge(sg, _fresh(obs)))

    runtime._setup = _patched_setup

    def _handle_sigint(signum, frame):
        print("\n[execute-sim-push] SIGINT — shutting down...", flush=True)
        planner.request_shutdown()
        try:
            runtime.stop()
        except Exception:
            pass
    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        runtime.run()
    except KeyboardInterrupt:
        pass
    finally:
        planner.close()
        if log_file:
            try:
                log_file.close()
            except Exception:
                pass

    diag_root = Path(recorder.root)
    print()
    print("=" * 60)
    print(f"Done. Records under {diag_root}")
    print(f"  - {diag_root / 'pushes.jsonl'}      (Δobject, stuck flag)")
    print(f"  - {diag_root / 'subgoals.jsonl'}    (dispatch + outcome)")
    print(f"  - {diag_root / 'run.log'}           (tee'd stdout/stderr)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
