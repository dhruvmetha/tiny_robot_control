"""Collect real-robot push primitives for sim-real calibration.

For each (edge_idx, push_steps) trial listed in a YAML spec, drives the
real robot through the FULL push skill (the one ``run_namo.py`` uses) and
records the obstacle Δ to disk. Between trials, prompts a human to slide
the obstacle back to its start position.

Internally this is a one-line tweak of ``run_namo``: same Runtime, same
controllers, same diagnostics — only the planner is replaced. Instead of
``NAMOPlanner`` (which asks namo_cpp for push sequences), we plug in a
``TrialListPlanner`` that emits the next push from a YAML list, prompting
between trials.

Output:
  <diag-path>/<run-name>/pushes.jsonl   — one record per push (controller's
                                          own Δ measurement; same schema
                                          run_namo writes for every push)
  <diag-path>/<run-name>/subgoals.jsonl — per-trial dispatch + outcome
  <diag-path>/<run-name>/run.log        — tee'd stdout/stderr
  <diag-path>/<run-name>/config.json    — args + git state snapshot

Example:
  python scripts/collect_real_primitives.py \\
      --config config/real.yaml \\
      --camera-service tcp://localhost:5556 \\
      --diag-path ./recordings \\
      --run-name "{date}_{time}_real_prims_iter1" \\
      --trial-spec config/real_primitive_trials_example.yaml
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

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

from robot_control.core.types import ObjectPose, Observation, PushSubgoal, Subgoal  # noqa: E402
from robot_control.planner.base import Planner  # noqa: E402
from robot_control.runtime import Runtime, RuntimeConfig  # noqa: E402


# ----------------------------------------------------- MarkerLockFilter
#
# Defends against ArUco bit-decode misreads. ArUco DICT_6X6_50 has Hamming-
# close IDs (e.g. marker 5 and marker 6 differ by only a couple of bits). A
# bit flip under glare/motion/oblique-angle conditions makes the decoder
# confidently return the wrong ID — same physical sticker shows up as obj_2
# in obs.objects instead of obj_1. Each obj_X has different dimensions in
# objects.yaml, so the push controller's edge-point math jumps + the GUI
# rectangle changes size. Result: robot wiggles trying to chase a contact
# point that's moving frame-to-frame.
#
# Fix here is downstream of ArUco — we wrap the runtime's observer with a
# filter that, once locked onto a target object name, drops every other
# movable from the observation and synthesizes the locked object from any
# misread that comes through. Controllers + GUI never see the misread.


class MarkerLockFilter:
    """Filters obs.objects to enforce a single locked movable identity.

    Use:
        f = MarkerLockFilter()
        # ...later, once the planner picks one:
        f.set_lock("obj_1", template_pose)
        # then every obs passed through f.apply(obs) only contains obj_1
        # (plus all statics, untouched).
    """

    def __init__(self) -> None:
        self._lock_name: Optional[str] = None
        # Stores the canonical dims for the locked name. Captured from the
        # first successful detection — those dims came from objects.yaml
        # via the observer's per-frame ObjectDefinition lookup, so they're
        # guaranteed correct for that name.
        self._lock_template: Optional[ObjectPose] = None
        self._remap_count = 0  # frames where we synthesized a remap
        self._drop_count = 0   # frames where we dropped a stray movable

    def set_lock(self, name: str, template: ObjectPose) -> None:
        if self._lock_name is not None:
            return  # don't re-lock; ignore subsequent sets
        self._lock_name = name
        # Clone template, zero pose — we only need width/depth/height/static.
        self._lock_template = ObjectPose(
            x=0.0, y=0.0, theta=0.0,
            width=template.width,
            depth=template.depth,
            height=template.height,
            is_static=False,
        )
        print(
            f"[lock] locked to {name!r} "
            f"(width={template.width}, depth={template.depth}). "
            "Stray movables in obs will be dropped / remapped."
        )

    def stats(self) -> Dict[str, int]:
        return {"remaps": self._remap_count, "drops": self._drop_count}

    def apply(self, obs: Optional[Observation]) -> Optional[Observation]:
        if obs is None or self._lock_name is None or self._lock_template is None:
            return obs

        kept_objects: Dict[str, ObjectPose] = {}
        # Pick out statics + the locked movable (if present), tally strays.
        first_stray: Optional[ObjectPose] = None
        for name, o in obs.objects.items():
            if getattr(o, "is_static", False):
                kept_objects[name] = o
                continue
            if name == self._lock_name:
                # ALWAYS override dims with the locked template, even when
                # the camera-detected name matches. Defends against another
                # publisher (e.g. a second RemoteObserverNode without
                # object_sizes populated) putting an obs on pubsub with
                # width=0 / depth=0. Without this override, the GUI would
                # fall back to default_size and flicker.
                kept_objects[name] = ObjectPose(
                    x=o.x,
                    y=o.y,
                    theta=o.theta,
                    width=self._lock_template.width,
                    depth=self._lock_template.depth,
                    height=self._lock_template.height,
                    is_static=False,
                )
            else:
                # Stray movable — drop it. Remember the first one in case we
                # need to remap (when the locked name is missing).
                self._drop_count += 1
                if first_stray is None:
                    first_stray = o

        # If the locked name is missing this frame but a stray was seen,
        # take the stray's position and pretend it's the locked object.
        if self._lock_name not in kept_objects and first_stray is not None:
            self._remap_count += 1
            kept_objects[self._lock_name] = ObjectPose(
                x=first_stray.x,
                y=first_stray.y,
                theta=first_stray.theta,
                width=self._lock_template.width,
                depth=self._lock_template.depth,
                height=self._lock_template.height,
                is_static=False,
            )

        return Observation(
            robot_x=obs.robot_x,
            robot_y=obs.robot_y,
            robot_theta=obs.robot_theta,
            objects=kept_objects,
            timestamp=obs.timestamp,
        )


# ----------------------------------------------------------- TrialListPlanner


class TrialListPlanner(Planner):
    """Emit a fixed YAML-driven sequence of PushSubgoals, with a human-in-the-
    loop obstacle reset prompt between trials.

    Plugged into Runtime in place of NAMOPlanner. Runtime calls ``plan(obs)``
    every time the currently-executing controller reports ``is_done()``. So
    each call corresponds to the boundary between trials:

      - First call: print "trial 1/N", verify obstacle is at target, return
        the first PushSubgoal.
      - Subsequent calls: the previous trial just finished. Verify obstacle
        is at target (prompt user to reset if not), return the next subgoal.
      - List exhausted: ``is_complete`` returns True, Runtime shuts down.
    """

    def __init__(
        self,
        trials: List[Dict[str, Any]],
        obstacle_target_xy: tuple,
        reset_tolerance_cm: float,
        lock_filter: Optional["MarkerLockFilter"] = None,
        skip_reset: bool = False,
    ) -> None:
        self._trials = trials
        self._idx = 0
        self._obstacle_target = obstacle_target_xy
        self._reset_tolerance = reset_tolerance_cm
        # When True, ``_wait_for_obstacle_reset`` does not block on a
        # tolerance check or prompt the operator. The push fires from
        # wherever the object currently is. Δpose is still meaningful
        # because ``pushes.jsonl`` records ``object_pose_before`` and
        # ``object_pose_after`` (both include θ).
        self._skip_reset = skip_reset
        # Cache the auto-detected object id so we don't re-prompt the camera
        # every trial when the spec doesn't pin one explicitly.
        self._auto_object_id: Optional[str] = None
        # Once auto-detection picks an obj_X, we tell this filter to
        # exclude every other movable identity from downstream
        # observations. Defends against ArUco misreads of the same
        # physical sticker.
        self._lock_filter = lock_filter
        # Fresh-obs callable injected by main() once the runtime's
        # _world has been constructed (after _setup runs). Returns the
        # lock-filtered Observation. We use this instead of running our
        # own RemoteObserverNode — a second ZMQ subscriber publishing to
        # the same pubsub topic caused a race in WorldState that made
        # the GUI rectangle flicker between correct dims and default.
        self._fresh_obs_source = None  # set by attach_obs_source
        # Set by the main-thread SIGINT handler so Ctrl+C breaks us out of
        # the blocking stdin wait inside _read_input_with_live_display. The
        # planner runs on the Runtime control thread, and Python only
        # delivers signals to the main thread, so the planner can't see
        # SIGINT directly — it has to poll this flag between select() ticks.
        self._shutdown_requested = threading.Event()

    def attach_obs_source(self, fresh_get) -> None:
        """Wire in the runtime's WorldState.get() callable. main() does this
        after Runtime._setup() has constructed _world.

        fresh_get returns the lock-filtered Observation — the right thing
        for both live display (we want the canonical obstacle) and the
        reset-tolerance check.
        """
        self._fresh_obs_source = fresh_get

    def close(self) -> None:
        """Nothing to clean up — planner no longer owns any threads or
        sockets. Kept as a no-op so main()'s finally block stays uniform."""
        return

    # -- Planner interface --

    def plan(self, obs: Observation) -> Optional[Subgoal]:
        if self._idx >= len(self._trials):
            return None  # is_complete will report True

        trial = self._trials[self._idx]
        trial_num = self._idx + 1
        total = len(self._trials)

        # Object resolution: prefer YAML pin; else auto-detect single movable.
        if trial.get("object_id"):
            object_id = str(trial["object_id"])
            # If a pin is given, lock to it the first time we see it in obs
            # (so we capture its canonical dims).
            if (
                self._lock_filter is not None
                and self._auto_object_id is None
                and object_id in obs.objects
            ):
                self._auto_object_id = object_id
                self._lock_filter.set_lock(object_id, obs.objects[object_id])
        else:
            object_id = self._resolve_object_id(obs)
            if object_id is None:
                print(
                    f"[trial {trial_num}/{total}] no usable obstacle in view; "
                    "skipping."
                )
                self._idx += 1
                return self.plan(obs)

        edge_idx = int(trial["edge_idx"])
        if "push_steps" in trial:
            push_steps = int(trial["push_steps"])
        elif "depth" in trial:
            push_steps = int(trial["depth"]) + 1
        else:
            print(f"[trial {trial_num}/{total}] missing depth/push_steps; skipping.")
            self._idx += 1
            return self.plan(obs)

        print()
        print(
            f"=== Trial {trial_num}/{total} === "
            f"object={object_id} edge_idx={edge_idx} push_steps={push_steps}"
        )

        # Wait for the obstacle to be back at target. The robot is allowed
        # to be anywhere — the push skill's APPROACHING phase will
        # navigate to the right contact point from wherever the robot
        # ended up. Only the obstacle pose matters for the measurement.
        self._wait_for_obstacle_reset(obs, object_id)

        self._idx += 1
        return PushSubgoal(object_id=object_id, edge_idx=edge_idx, push_steps=push_steps)

    def is_complete(self, obs: Observation) -> bool:
        # Tell Runtime to shut down once we've dispatched everything.
        return self._idx >= len(self._trials)

    def reset(self) -> None:
        # Don't reset trial progress; otherwise re-running would replay
        # trials we already finished. If a true reset is wanted, build a
        # fresh planner.
        return

    # -- internal --

    def _resolve_object_id(self, obs: Observation) -> Optional[str]:
        if self._auto_object_id is not None:
            return self._auto_object_id
        movables = [
            name for name, o in obs.objects.items()
            if not getattr(o, "is_static", False)
        ]
        if len(movables) == 1:
            self._auto_object_id = movables[0]
            print(f"[planner] auto-detected obstacle: {self._auto_object_id!r}")
            # Lock the filter to this name (captures the YAML-defined dims
            # from the current detection — those dims came through the
            # observer's lookup, so they're canonical for this name).
            if self._lock_filter is not None:
                template = obs.objects[self._auto_object_id]
                self._lock_filter.set_lock(self._auto_object_id, template)
            return self._auto_object_id
        if len(movables) == 0:
            print(
                "[planner] no movable object visible. "
                f"Visible: {list(obs.objects.keys())}"
            )
            return None
        print(
            f"[planner] multiple movable objects visible: {movables}. "
            "Pin 'object_id' explicitly in the trial spec."
        )
        return None

    def request_shutdown(self) -> None:
        """Called from the main thread (e.g. SIGINT handler) to unblock the
        planner from any stdin wait so the Runtime can shut down cleanly."""
        self._shutdown_requested.set()

    def _wait_for_obstacle_reset(
        self, obs: Observation, object_id: str
    ) -> Observation:
        """Block until the obstacle is within tolerance of the target.

        Reprints obstacle position at 1Hz while waiting. Robot can be
        anywhere — the push skill's APPROACHING phase handles navigating
        to the contact point from wherever the robot is parked.

        When ``skip_reset`` is True, this just fetches one fresh obs,
        prints the recorded start pose for the log, and returns.
        """
        if self._skip_reset:
            latest = self._fresh_obs(obs)
            obj = latest.objects.get(object_id)
            if obj is not None:
                print(
                    f"  [skip_reset] start pose: ({obj.x:.2f}, {obj.y:.2f}) "
                    f"θ={obj.theta:.2f}°. pushing..."
                )
            return latest

        target_x, target_y = self._obstacle_target
        latest = obs
        while True:
            if self._shutdown_requested.is_set():
                raise SystemExit("[planner] shutdown requested (Ctrl+C)")
            latest = self._fresh_obs(latest)
            obj = latest.objects.get(object_id)
            if obj is None:
                print(
                    f"  Object {object_id!r} not in view "
                    f"(visible: {list(latest.objects.keys())})."
                )
            else:
                d = math.hypot(obj.x - target_x, obj.y - target_y)
                if d <= self._reset_tolerance:
                    print(
                        f"  obstacle at ({obj.x:.1f},{obj.y:.1f}) cm — OK "
                        f"({d:.1f} cm from target).  "
                        f"robot at ({latest.robot_x:.1f},{latest.robot_y:.1f}) "
                        f"θ={latest.robot_theta:.1f}°. pushing..."
                    )
                    return latest
                print(
                    f"  obstacle at ({obj.x:.1f},{obj.y:.1f}) cm — "
                    f"{d:.1f} cm from target {self._obstacle_target}.  "
                    f"robot at ({latest.robot_x:.1f},{latest.robot_y:.1f}) "
                    f"θ={latest.robot_theta:.1f}°."
                )
            print("  reset obstacle (and move robot if you want); press ENTER when done (or 'q' to quit)")
            try:
                ans = self._read_input_with_live_pose(object_id, target_x, target_y)
            except EOFError:
                ans = "q"
            if ans == "q":
                raise SystemExit("[planner] user quit")

    def _read_input_with_live_pose(
        self, object_id: str, target_x: float, target_y: float
    ) -> str:
        """Block on stdin while reprinting the obstacle's live pose at 1Hz."""
        if self._fresh_obs_source is None:
            return input("  > ").strip().lower()
        import select
        while True:
            if self._shutdown_requested.is_set():
                return "q"
            r, _, _ = select.select([sys.stdin], [], [], 1.0)
            if r:
                return sys.stdin.readline().strip().lower()
            o = self._fresh_obs_source()
            if o is None:
                continue
            obj = o.objects.get(object_id)
            if obj is None:
                continue
            d = math.hypot(obj.x - target_x, obj.y - target_y)
            print(
                f"    [live] obj ({obj.x:.1f},{obj.y:.1f}) cm "
                f"({d:.1f} cm from target)   "
                f"robot ({o.robot_x:.1f},{o.robot_y:.1f}) θ={o.robot_theta:.1f}°",
                flush=True,
            )

    def _fresh_obs(self, fallback: Observation) -> Observation:
        """Return the runtime's freshest lock-filtered obs, falling back to
        the supplied obs if the source isn't wired yet."""
        if self._fresh_obs_source is None:
            return fallback
        o = self._fresh_obs_source()
        return o if o is not None else fallback


# --------------------------------------------------------------------- main


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Three required flags. Everything else is optional with sane defaults
    # or lives in the trial-spec YAML.
    p.add_argument("--config", required=True, help="Path to real.yaml")
    p.add_argument(
        "--camera-service",
        required=True,
        metavar="ADDRESS",
        help="ZMQ address of the camera service (e.g. tcp://localhost:5556).",
    )
    p.add_argument("--trial-spec", required=True, help="YAML file listing trials.")
    p.add_argument(
        "--diag-path",
        default="./recordings",
        help="Root diagnostics directory (default: ./recordings).",
    )
    p.add_argument(
        "--run-name",
        default="real_prims_{date}_{time}",
        help="Subdirectory name under --diag-path. Supports placeholders "
             "{date} {time} {timestamp} etc. (default: real_prims_{date}_{time}).",
    )
    p.add_argument(
        "--push-speed",
        type=float,
        default=None,
        help="Override push.max_speed from controller.yaml (PWM scale [0,1]). "
             "Calibration knob — handy for sweeping speeds across runs "
             "without editing YAML.",
    )
    p.add_argument(
        "--nav-speed",
        type=float,
        default=None,
        help="Override navigation.max_speed from controller.yaml.",
    )
    p.add_argument(
        "--allow-overwrite",
        action="store_true",
        help="Reuse an existing --run-name directory instead of erroring out. "
             "Handy when re-running the same calibration iteration.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't send wheel commands. Useful for testing the prompt loop "
             "without a robot.",
    )
    p.add_argument(
        "--no-reset-check",
        action="store_true",
        help="Skip the obstacle-target tolerance check. Push fires from "
             "wherever the object currently is. pushes.jsonl records "
             "object_pose_before (with θ) so per-trial Δpose is still "
             "meaningful — operator no longer needs to slide the object "
             "back to a fixed target between trials.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    # Same diagnostics bootstrap run_namo uses — gives us config.json + the
    # stdout/stderr Tee to run.log.
    recorder, log_file = bootstrap_diagnostics(args)
    if recorder is None or not recorder.enabled:
        print("Error: --diag-path is required.", file=sys.stderr)
        return 2
    args._diagnostics_recorder = recorder  # Runtime picks this up

    # Load trials.
    spec = yaml.safe_load(Path(args.trial_spec).read_text()) or {}
    trials = spec.get("trials") or []
    if not trials:
        print(f"Error: no 'trials' in {args.trial_spec}", file=sys.stderr)
        return 2
    obstacle_target = spec.get("obstacle_target") or [25.0, 35.0]
    reset_tolerance = float(spec.get("reset_tolerance_cm") or 3.0)
    print(f"[collect] {len(trials)} trial(s) loaded from {args.trial_spec}")
    print(
        f"[collect] obstacle_target = {tuple(obstacle_target)} cm  "
        f"reset_tolerance = {reset_tolerance} cm"
    )

    # The marker-misread filter. Wraps the runtime's observer once we know
    # which obj_X the planner locked onto.
    lock_filter = MarkerLockFilter()

    # The planner.
    planner = TrialListPlanner(
        trials=trials,
        obstacle_target_xy=(float(obstacle_target[0]), float(obstacle_target[1])),
        reset_tolerance_cm=reset_tolerance,
        lock_filter=lock_filter,
        skip_reset=args.no_reset_check,
    )

    # Build a Runtime exactly the way run_namo does for real mode. The only
    # difference between this script and `run_namo --config config/real.yaml`
    # is which Planner gets plugged in.
    runtime_config = RuntimeConfig(
        mode="real",
        config_path=args.config,
        planner=planner,
        dry_run=args.dry_run,
        quit_on_complete=True,
        camera_service_address=args.camera_service,
        nav_speed_override=args.nav_speed,
        push_speed_override=args.push_speed,
    )
    runtime_config.diagnostics_recorder = recorder

    runtime = Runtime(runtime_config)

    # IMPORTANT: runtime._world and runtime._executor are None right after
    # construction — they're created inside runtime._setup(), which is
    # called from runtime.run() RIGHT BEFORE the control loop starts. So
    # we can't install our patches here directly; we have to defer them
    # until after _setup() has run.
    #
    # We do that by monkey-patching _setup() itself: when the runtime
    # calls _setup() from run(), our wrapper runs first (calls the real
    # _setup), then installs the patches on the freshly-created _world
    # and _executor before the control thread starts.

    orig_setup = runtime._setup

    def _patched_setup():
        orig_setup()  # builds _world, _executor, etc.

        world = runtime._world
        executor = runtime._executor

        # 1) Lock filter on WorldState.get(). WorldState is what the
        # control loop polls each tick. Wrapping .get() means every
        # consumer (push controller, GUI, nav) sees the filtered obs.
        if world is not None:
            original_world_get = world.get

            def _filtered_world_get():
                return lock_filter.apply(original_world_get())

            world.get = _filtered_world_get
            print("[lock] installed MarkerLockFilter on runtime._world.get()")

            # Give the planner the runtime's WorldState as its fresh-obs
            # source. Replaces the (removed) second RemoteObserverNode the
            # planner used to spin up — see TrialListPlanner.__init__ doc.
            planner.attach_obs_source(world.get)
            print("[fresh] planner.attach_obs_source(world.get) wired")
        else:
            print("[lock] WARN: runtime._world is still None after _setup().")

        # 2) Staleness fix. plan() blocks on user input for long stretches.
        # During that block the runtime's local `obs` is a snapshot from
        # before the user moved anything. set_subgoal + the first step()
        # afterward use that stale obs, so COMPUTING_APPROACH reads the
        # OLD obstacle pose AND the OLD robot pose. Monkey-patch
        # executor.step / set_subgoal to always pull from _world.get(),
        # which RemoteObserverNode keeps current via pubsub. Both the
        # obstacle reset AND the (newly) hand-moved robot position land
        # in the controller correctly; and every subsequent tick during
        # the push uses the live robot pose, not a frozen snapshot.
        if world is not None and executor is not None:
            def _fresh(obs):
                wf = world.get()
                return wf if wf is not None else obs

            orig_step = executor.step
            orig_set = executor.set_subgoal

            def _fresh_step(obs):
                return orig_step(_fresh(obs))

            def _fresh_set(subgoal, obs):
                return orig_set(subgoal, _fresh(obs))

            executor.step = _fresh_step
            executor.set_subgoal = _fresh_set
            print("[fresh] executor.step/set_subgoal patched to always use _world.get()")
        else:
            print(
                "[fresh] WARN: executor or world still None after _setup(); "
                "staleness fix NOT installed."
            )

    runtime._setup = _patched_setup


    # SIGINT handler: Runtime runs the planner on a background control
    # thread, so the planner can't catch SIGINT itself. The handler flips
    # the planner's shutdown flag (unblocks select()) AND stops Runtime so
    # its main-thread busy loop exits. Without this, Ctrl+C either does
    # nothing or wedges _shutdown() trying to join the stdin-blocked
    # control thread.
    def _handle_sigint(signum, frame):
        print("\n[collect] SIGINT received — shutting down...", flush=True)
        planner.request_shutdown()
        try:
            runtime.stop()
        except Exception:
            pass
    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        runtime.run()
    except KeyboardInterrupt:
        pass  # already handled by SIGINT handler
    finally:
        # Marker-misread filter stats, if it caught anything.
        stats = lock_filter.stats()
        if stats["remaps"] or stats["drops"]:
            print()
            print("=" * 60)
            print(
                f"[lock] filter caught {stats['remaps']} misread frame(s) "
                f"(remapped to locked obj) and {stats['drops']} stray "
                f"movable frame(s) (dropped). Push behavior was protected "
                "from these."
            )
            print("=" * 60)
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
    print(f"  - {diag_root / 'pushes.jsonl'}      (per-push controller summary: object_id, edge, push_steps, Δxy, Δθ, stuck)")
    print(f"  - {diag_root / 'subgoals.jsonl'}    (per-trial dispatch + outcome)")
    print(f"  - {diag_root / 'run.log'}           (tee'd stdout/stderr)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
