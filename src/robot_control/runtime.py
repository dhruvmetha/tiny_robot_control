"""Runtime for orchestrating robot control with automatic node spawning."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from robot_control.controller import (
    Controller,
    KeyboardController,
    NavigationController,
    FollowPathController,
    PushController,
)
from robot_control.controller.config import load_controller_configs
from robot_control.coordinator import ControlCoordinator
from robot_control.core.types import (
    Action,
    NavigateSubgoal,
    Observation,
    PushSubgoal,
    WorkspaceConfig,
)
from robot_control.core.world_state import WorldState
from robot_control.executor import SubgoalExecutor
from robot_control.planner import RVGPlanner, WavefrontPathPlanner
from robot_control.planner.base import Planner

# Conditional imports for simulation
try:
    from robot_control.environment.sim import SimEnv, SimConfig
    from robot_control.nodes import SimSensorNode
    SIM_AVAILABLE = True
except ImportError:
    SIM_AVAILABLE = False
    SimEnv = None
    SimConfig = None
    SimSensorNode = None

# Conditional imports for real robot
try:
    from robot_control.camera import (
        ArucoObserver,
        ObserverConfig,
        make_real_workspace_config,
    )
    from robot_control.camera.observer import ObjectDefinition
    from robot_control.environment.real import RealEnv, RealEnvConfig
    from robot_control.nodes import CameraSensorNode, CameraConfig
    from robot_control.utils import CameraRecorder
    REAL_AVAILABLE = True
except ImportError:
    REAL_AVAILABLE = False
    ArucoObserver = None
    ObserverConfig = None
    make_real_workspace_config = None
    ObjectDefinition = None
    RealEnv = None
    RealEnvConfig = None
    CameraSensorNode = None
    CameraConfig = None
    CameraRecorder = None

# Conditional import for remote camera service
try:
    from robot_control.nodes.remote_observer import RemoteObserverNode, ObjectSizeInfo
    REMOTE_AVAILABLE = True
except ImportError:
    REMOTE_AVAILABLE = False
    RemoteObserverNode = None
    ObjectSizeInfo = None

# Conditional import for GUI
try:
    from robot_control.gui import Window
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False
    Window = None


PLAN_COMPLETE_STATUS = "Plan Complete"
PLANNING_FAILED_STATUS = "Planning Failed"
AUTONOMOUS_TERMINAL_STATUSES = frozenset(
    {PLAN_COMPLETE_STATUS, PLANNING_FAILED_STATUS}
)


def _load_objects_yaml(objects_path: str) -> Dict[str, "ObjectDefinition"]:
    """Load object definitions from objects.yaml."""
    with open(objects_path, "r") as f:
        data = yaml.safe_load(f)

    object_defs = {}
    for name, obj in data.get("objects", {}).items():
        obj_type = obj.get("type", "movable")
        shape = obj.get("shape", {})
        offset = obj.get("marker_offset", {})
        object_defs[name] = ObjectDefinition(
            marker_id=obj["marker_id"],
            is_static=(obj_type == "static"),
            is_goal=(obj_type == "goal"),
            width_cm=shape.get("width", 0.0),
            depth_cm=shape.get("depth", 0.0),
            height_cm=shape.get("height", 0.0),
            marker_offset_x_cm=offset.get("x", 0.0),
            marker_offset_y_cm=offset.get("y", 0.0),
        )
    return object_defs


@dataclass
class RuntimeConfig:
    """Configuration for Runtime."""

    # Mode: "sim" or "real"
    mode: str = "sim"

    # Simulation config (uses defaults if None)
    sim_config: Optional["SimConfig"] = None

    # Real robot config paths
    config_path: str = "config/real.yaml"
    objects_path: str = "config/objects.yaml"
    serial_port: Optional[str] = None  # Override from YAML
    dry_run: bool = False  # Don't send commands to robot

    # GUI options
    show_gui: bool = True
    show_camera: bool = True  # Only for real mode
    show_settings: bool = True

    # Remote camera service (ZMQ address, e.g. "tcp://localhost:5556")
    camera_service_address: Optional[str] = None

    # If set, ask the remote camera_service to record video to this directory
    # for the duration of this runtime session. Requires camera_service_address.
    record_video_dir: Optional[str] = None

    # Optional overrides for navigation/push max_speed. If None, the
    # respective controller falls back to its YAML config value
    # (controller.yaml: navigation.max_speed / push.max_speed).
    nav_speed_override: Optional[float] = None
    push_speed_override: Optional[float] = None

    # Control options
    initial_controller: str = "keyboard"
    initial_speed: float = 0.3
    frequency: float = 30.0

    # Autonomous mode: if planner is set, runs in autonomous mode
    planner: Optional["Planner"] = None
    quit_on_complete: bool = True  # Quit when autonomous plan completes
    step_confirm: bool = False  # Pause for user confirmation before each subgoal dispatch

    # Diagnostics. When recorder is enabled, Runtime emits structured events
    # (plans, subgoals, pushes, connectivity, scene snapshots) into the
    # recorder's directory. When recorder is None, every call site is a no-op.
    diagnostics_recorder: Optional[object] = None  # DiagnosticsRecorder, but kept loose-typed to avoid circular import
    capture_scene: bool = False                    # If True, recorder captures scene_before/scene_after
    capture_sim_success: bool = False              # If True, on success write success_chain.json + sim replay MP4 (fail/abort → partial_chain.json only)


class Runtime:
    """
    Unified runtime that spawns all nodes and GUI.

    Eliminates manual setup by automatically creating:
    - Environment (SimEnv or RealEnv)
    - Sensor nodes (SimSensorNode or CameraSensorNode + ArucoObserver)
    - WorldState
    - Planner (RVGPlanner)
    - Controllers (Keyboard, Navigation, FollowPath)
    - ControlCoordinator
    - Window (GUI, optional)
    - Control loop thread

    Usage:
        # Simulation with GUI
        runtime = Runtime(RuntimeConfig(mode="sim"))
        runtime.run()  # Blocks until quit

        # Real robot
        runtime = Runtime(RuntimeConfig(
            mode="real",
            config_path="config/real.yaml",
        ))
        runtime.run()

        # Headless (no GUI)
        runtime = Runtime(RuntimeConfig(mode="sim", show_gui=False))
        runtime.start()
        # Use runtime.world and runtime.coordinator in your own loop
        runtime.stop()
    """

    def __init__(self, config: RuntimeConfig) -> None:
        """Initialize runtime with config. Does NOT start anything yet."""
        self._config = config
        self._running = False
        self._shutdown_complete = False

        # Components (created in _setup)
        self._env: Optional[SimEnv | RealEnv] = None
        self._sensor: Optional[SimSensorNode | CameraSensorNode] = None
        self._observer: Optional[ArucoObserver] = None
        self._recorder: Optional[CameraRecorder] = None
        self._remote_observer: Optional[RemoteObserverNode] = None
        self._remote_recorder: Optional["RemoteRecordClient"] = None
        self._record_session_dir: Optional[str] = None
        self._record_subgoal_index: int = 0
        self._record_active: bool = False
        self._world: Optional[WorldState] = None
        self._coordinator: Optional[ControlCoordinator] = None
        self._window: Optional[Window] = None
        self._workspace_config: Optional[WorkspaceConfig] = None
        self._controllers: Dict[str, Controller] = {}
        self._control_thread: Optional[threading.Thread] = None

        # Real env config (loaded from YAML)
        self._real_env_config: Optional[RealEnvConfig] = None

        # Autonomous mode components
        self._planner: Optional[Planner] = None
        self._executor: Optional[SubgoalExecutor] = None
        self._terminal_announced = False
        self._terminal_outcome: Optional[Tuple[str, str]] = None
        self._subgoal_start_time: Optional[float] = None  # Wall-clock timer for subgoal duration logging

        # Connectivity warning debounce. Without this the 30Hz loop would spam
        # the log when the robot is offline; we throttle to one warn per N sec.
        self._last_offline_warn_time: float = 0.0
        self._offline_warn_interval_sec: float = 5.0
        self._last_seen_robot_online: Optional[bool] = None  # None = never checked

        # Diagnostics — recorder is shared across all runtime callsites.
        # All recorder methods are no-ops when recorder is None or disabled.
        self._diag = config.diagnostics_recorder
        self._capture_scene = config.capture_scene
        self._capture_sim_success = config.capture_sim_success
        # Sim-success replay state. The chain is appended on each successful
        # push subgoal; the start XML is captured once when the runtime starts.
        # Push-meta is stashed at dispatch so _record_subgoal_done can append
        # without needing the original subgoal object.
        self._sim_success_chain: List[Dict[str, Any]] = []
        self._sim_replay_start_xml: Optional[str] = None
        # Pose (x_m, y_m, theta_rad) the replay env must teleport to before
        # stepping. Captured at start alongside the start XML, used by
        # _write_sim_success_artifacts when handing the chain to MuJoCo.
        self._sim_replay_start_pose_sim: Optional[Tuple[float, float, float]] = None
        self._pending_push_meta: Optional[Dict[str, Any]] = None
        # Per-plan sim-success capture counter. Incremented every time the
        # planner-side callback fires (i.e. every plan() with at least one
        # PushSubgoal). Drives the sequential plan_NNN_*.{xml,json,mp4} naming
        # under <diag-root>/sim_replays/.
        self._sim_plan_capture_count: int = 0
        self._started_at_epoch: Optional[float] = None
        self._ended_at_epoch: Optional[float] = None
        self._offline_total_sec: float = 0.0
        self._offline_since: Optional[float] = None
        self._online_at_start: Optional[bool] = None

    def run(self) -> None:
        """Start all components and run until quit."""
        self._setup()
        self._start()

        # Begin diagnostics timeline.
        self._started_at_epoch = time.time()
        scene_before = self._capture_scene_snapshot("before")
        # Sim-success replay needs a starting XML snapshot. Capture it once
        # here, independent of --capture-scene / --record-video.
        self._capture_sim_replay_start_xml()

        outcome = "unknown"
        outcome_reason = "no signal recorded"
        try:
            if self._window:
                self._print_banner()
                self._window.run()  # Blocks until window closed
            else:
                # Headless mode - run until stop() called
                self._print_banner()
                while self._running:
                    time.sleep(0.1)

            # Determine outcome from planner state (best-effort).
            outcome, outcome_reason = self._determine_outcome()
        except KeyboardInterrupt:
            outcome = "aborted"
            outcome_reason = "KeyboardInterrupt"
            raise
        except Exception as exc:
            outcome = "crashed"
            outcome_reason = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            self._ended_at_epoch = time.time()
            # Always attempt the post-run snapshot, even on crash. Errors are
            # captured into scene_capture_errors.log by the recorder rather
            # than propagated — diagnostics must not mask the real exception.
            try:
                scene_after = self._capture_scene_snapshot("after")
            except Exception as _exc:
                print(f"[Runtime] scene_after capture raised: {_exc!r}", flush=True)
                scene_after = {}
            try:
                self._write_summary(outcome, outcome_reason, scene_before, scene_after)
            except Exception as _exc:
                print(f"[Runtime] summary write raised: {_exc!r}", flush=True)
            try:
                self._write_sim_success_artifacts(outcome)
            except Exception as _exc:
                print(f"[Runtime] sim-success write raised: {_exc!r}", flush=True)
            self._shutdown()

    def start(self) -> None:
        """Start all components without blocking. Use for headless mode."""
        if self._running:
            return  # Already running
        self._shutdown_complete = False  # Reset for potential restart
        self._setup()
        self._start()

    def stop(self) -> None:
        """Stop all components and cleanup resources."""
        if not self._running:
            return  # Already stopped
        self._running = False
        if self._window:
            self._window.close_window()
        self._shutdown()

    @property
    def coordinator(self) -> Optional[ControlCoordinator]:
        """Access the control coordinator."""
        return self._coordinator

    @property
    def world(self) -> Optional[WorldState]:
        """Access the world state."""
        return self._world

    @property
    def env(self) -> Optional[SimEnv | RealEnv]:
        """Access the environment."""
        return self._env

    @property
    def is_running(self) -> bool:
        """Check if runtime is running."""
        return self._running

    # --- Setup methods ---

    def _setup(self) -> None:
        """Create all components based on config."""
        if self._config.mode == "sim":
            self._setup_sim()
        else:
            self._setup_real()

        # Create coordinator (used in interactive mode)
        self._coordinator = ControlCoordinator(
            controllers=self._controllers,
            world=self._world,
            initial_controller=self._config.initial_controller,
            initial_speed=self._config.initial_speed,
        )

        # Create executor if planner provided (autonomous mode)
        if self._config.planner:
            self._planner = self._config.planner
            self._executor = SubgoalExecutor(
                self._workspace_config,
                self._controllers["navigation"],
                self._controllers.get("push"),
            )

        # Create GUI
        if self._config.show_gui:
            if not GUI_AVAILABLE:
                print("[Runtime] Warning: GUI not available, running headless")
            else:
                self._setup_gui()

    def _setup_sim(self) -> None:
        """Setup simulation mode components."""
        if not SIM_AVAILABLE:
            raise RuntimeError(
                "Simulation not available. Check that SimEnv can be imported."
            )

        # Use provided config or defaults
        sim_config = self._config.sim_config or SimConfig(
            car_width=8,
            car_height=10,
            x=10,
            y=10,
            theta=0,
            objects={
                "box1": (22, 32, 0),
                "box2": (12, 45, 45),
            }
        )
        self._workspace_config = sim_config.workspace_config

        # Create environment and sensor
        self._env = SimEnv(sim_config)
        self._sensor = SimSensorNode(self._env, rate=self._config.frequency)
        self._world = WorldState()

        # Create controllers
        self._controllers = self._create_controllers()

    def _setup_real(self) -> None:
        """Setup real robot mode components."""
        if not REAL_AVAILABLE:
            raise RuntimeError(
                "Real robot mode not available. "
                "Ensure camera and serial dependencies are installed."
            )

        # Load configuration from YAML
        config_path = Path(self._config.config_path)
        objects_path = Path(self._config.objects_path)

        if config_path.exists():
            camera_config, observer_config, real_env_config, robot_width, robot_height = (
                self._load_real_config(str(config_path), str(objects_path))
            )
        else:
            print(f"[Runtime] Config not found: {config_path}, using defaults")
            camera_config = CameraConfig()
            observer_config = ObserverConfig()
            real_env_config = RealEnvConfig()
            robot_width = 8.0
            robot_height = 10.0

        # Override serial port if specified
        if self._config.serial_port:
            real_env_config.port = self._config.serial_port

        self._real_env_config = real_env_config

        # Create workspace config
        self._workspace_config = make_real_workspace_config(
            car_width=robot_width,
            car_height=robot_height,
        )

        # Create world state
        self._world = WorldState()

        if self._config.camera_service_address:
            # Remote camera mode: use ZMQ observer, skip local camera
            if not REMOTE_AVAILABLE:
                raise RuntimeError(
                    "Remote camera service requires pyzmq. Install with: pip install pyzmq"
                )
            # Build object size lookup from object definitions (loaded from objects.yaml)
            object_sizes = {}
            if observer_config and observer_config.object_defs:
                for name, obj_def in observer_config.object_defs.items():
                    if not obj_def.is_goal:
                        object_sizes[name] = ObjectSizeInfo(
                            width=obj_def.width_cm,
                            depth=obj_def.depth_cm,
                            height=obj_def.height_cm,
                            is_static=obj_def.is_static,
                        )
            self._remote_observer = RemoteObserverNode(
                self._config.camera_service_address,
                object_sizes=object_sizes,
            )
            print(f"[Runtime] Using remote camera service at {self._config.camera_service_address}")
        else:
            # Local camera mode: create camera sensor + observer + recorder
            self._sensor = CameraSensorNode(camera_config)
            self._observer = ArucoObserver(observer_config)
            self._recorder = CameraRecorder(output_dir="recordings", fps=30.0)

        # Create real environment
        self._env = RealEnv(real_env_config)

        # Create controllers
        self._controllers = self._create_controllers()

    def _load_real_config(
        self, config_path: str, objects_path: str
    ) -> Tuple[CameraConfig, ObserverConfig, RealEnvConfig, float, float]:
        """Load configuration from YAML files for real robot."""
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        camera_cfg = config.get("camera", {})
        robot_cfg = config.get("robot", {})
        workspace_cfg = config.get("workspace", {})
        serial_cfg = config.get("serial", {})

        # Parse marker offset
        marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
        if isinstance(marker_offset, list) and len(marker_offset) == 2:
            marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
        else:
            marker_offset_tuple = (0.0, 0.0)

        # Load object definitions from objects.yaml
        object_defs = {}
        if Path(objects_path).exists():
            object_defs = _load_objects_yaml(objects_path)

        # Camera sensor config
        camera_config = CameraConfig(
            camera_device=camera_cfg.get("device", 0),
            resolution=camera_cfg.get("resolution", "720p"),
            fps=camera_cfg.get("fps", 60),
            exposure=camera_cfg.get("exposure", -6),
        )

        # ArUco observer config
        # object_marker_size_mm can be at root level or in robot section
        object_marker_size = config.get("object_marker_size_mm") or robot_cfg.get("object_marker_size_mm", 30.0)

        observer_config = ObserverConfig(
            calibration_file=camera_cfg.get("calibration_file", ""),
            undistort=camera_cfg.get("undistort", False),
            robot_marker_id=robot_cfg.get("marker_id", 1),
            robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
            marker_to_wheel_offset_cm=marker_offset_tuple,
            object_defs=object_defs,
            object_marker_size_mm=object_marker_size,
            warmup_frames=workspace_cfg.get("warmup_frames", 30),
            min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
        )

        # Real environment config
        real_env_config = RealEnvConfig(
            port=serial_cfg.get("port", "/dev/ttyACM0"),
            baudrate=serial_cfg.get("baudrate", 115200),
            robot_id=robot_cfg.get("marker_id", 1),
            send_hz=serial_cfg.get("send_hz", 30.0),
            invert_right_wheel=serial_cfg.get("invert_right_wheel", False),
        )

        # Robot dimensions
        robot_width = robot_cfg.get("width_cm", 8.0)
        robot_height = robot_cfg.get("height_cm", 10.0)

        return camera_config, observer_config, real_env_config, robot_width, robot_height

    def _create_controllers(self) -> Dict[str, Controller]:
        """Create standard controller set."""
        # Load controller configs from YAML
        controller_configs = load_controller_configs()

        # Select planner based on config
        if controller_configs.navigation.planner == "wavefront":
            planner = WavefrontPathPlanner(
                workspace_width=self._workspace_config.width,
                workspace_height=self._workspace_config.height,
                robot_width=self._workspace_config.car_width,
                robot_height=self._workspace_config.car_height,
                debug_dir=controller_configs.navigation.wavefront_debug_dir,
                obstacle_proximity_distance_cm=controller_configs.navigation.obstacle_proximity_distance_cm,
                obstacle_proximity_weight=controller_configs.navigation.obstacle_proximity_weight,
            )
        else:
            planner = RVGPlanner(
                workspace_width=self._workspace_config.width,
                workspace_height=self._workspace_config.height,
                robot_width=self._workspace_config.car_width,
                robot_height=self._workspace_config.car_height,
                robot_geometry_scale=controller_configs.navigation.robot_geometry_scale,
            )

        # Create keyboard controller (interactive control — uses --speed)
        keyboard = KeyboardController(max_speed=self._config.initial_speed)

        # Create navigation controller (used by push controller for approach phase).
        # Pass nav_speed_override if user set --nav-speed; else use YAML
        # navigation.max_speed.
        navigation = NavigationController(
            self._workspace_config,
            planner,
            nav_config=controller_configs.navigation,
            max_speed=self._config.nav_speed_override,
        )

        # Create follow path controller (uses --speed as a starting value;
        # gets overridden when a push hands it a planned path)
        follow_path = FollowPathController(
            self._workspace_config, max_speed=self._config.initial_speed
        )

        # Create push controller with navigation controller for approach phase.
        # Pass push_speed_override if user set --push-speed; else use YAML
        # push.max_speed.
        push = PushController(
            self._workspace_config,
            nav_controller=navigation,
            push_config=controller_configs.push,
            max_speed=self._config.push_speed_override,
        )

        # Confirm what speeds are actually in effect (useful when debugging
        # whether --nav-speed/--push-speed or YAML defaults are winning).
        print(
            f"[Runtime] Speeds in effect: nav={navigation.max_speed:.2f}, "
            f"push={push.max_speed:.2f}, keyboard/follow_path={self._config.initial_speed:.2f}"
        )

        return {
            "keyboard": keyboard,
            "navigation": navigation,
            "follow_path": follow_path,
            "push": push,
        }

    def _setup_gui(self) -> None:
        """Setup GUI window and bind callbacks."""
        is_real = self._config.mode == "real"
        # No camera panel when using remote camera service (no vis_frame)
        show_camera = (
            is_real
            and self._config.show_camera
            and self._remote_observer is None
        )

        self._window = Window(
            self._workspace_config,
            show_camera=show_camera,
            show_connection=(is_real and not self._config.dry_run),
            show_settings=self._config.show_settings,
            autonomous=(self._planner is not None),
            target_robot_id=(
                self._real_env_config.robot_id
                if is_real and self._real_env_config and not self._config.dry_run
                else None
            ),
        )
        # Set initial controller state
        if self._planner:
            # Autonomous mode - disable interactive elements
            self._window.enable_canvas_click(False)
            self._window.set_controller("Autonomous")

            # Speed presets for +/- adjustment
            speed_presets = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 0.9]

            # Autonomous key handler
            def autonomous_key_handler(key: str):
                if key == "escape":
                    self.stop()
                elif key == "space":
                    print("[Runtime] Emergency stop")
                    if self._executor:
                        self._executor.cancel()
                    if is_real and not self._config.dry_run:
                        if hasattr(self._env, "stop_robot"):
                            self._env.stop_robot()
                elif key.lower() == "r" and is_real and self._recorder:
                    self._on_key_r()
                elif key in ("=", "+"):
                    # Increase speed
                    nav = self._controllers.get("navigation")
                    if nav and hasattr(nav, "max_speed"):
                        current = nav.max_speed
                        for preset in speed_presets:
                            if preset > current:
                                nav.set_speed(preset)
                                self._window.update_speed(preset)
                                break
                elif key in ("-", "_"):
                    # Decrease speed
                    nav = self._controllers.get("navigation")
                    if nav and hasattr(nav, "max_speed"):
                        current = nav.max_speed
                        for preset in reversed(speed_presets):
                            if preset < current:
                                nav.set_speed(preset)
                                self._window.update_speed(preset)
                                break

            self._window.register_callback("on_key_press", autonomous_key_handler)

            # Speed button callback
            def on_speed_change(direction: int):
                nav = self._controllers.get("navigation")
                if nav and hasattr(nav, "max_speed"):
                    current = nav.max_speed
                    if direction > 0:
                        for preset in speed_presets:
                            if preset > current:
                                nav.set_speed(preset)
                                self._window.update_speed(preset)
                                break
                    else:
                        for preset in reversed(speed_presets):
                            if preset < current:
                                nav.set_speed(preset)
                                self._window.update_speed(preset)
                                break

            self._window.register_callback("on_speed_change", on_speed_change)

            # Cancel/Stop button callback
            def on_cancel():
                print("[Runtime] Stop button pressed")
                if self._executor:
                    self._executor.cancel()
                if is_real and not self._config.dry_run:
                    if hasattr(self._env, "stop_robot"):
                        self._env.stop_robot()

            self._window.register_callback("on_cancel", on_cancel)

            # Set initial speed display
            self._window.update_speed(self._config.initial_speed)

            # Recording toggle button still works
            if is_real and self._recorder:
                self._window.register_callback("on_record_toggled", self._on_record_toggled)
        else:
            # Interactive mode - full coordinator binding
            self._window.enable_canvas_click(False)  # Start with keyboard mode
            self._window.set_controller("Keyboard")
            self._coordinator.bind_to_window(self._window)

            # Register event handlers
            self._coordinator.on_quit(self._on_quit)
            self._coordinator.on_emergency_stop(self._on_emergency_stop)

            # Register recording callbacks for real mode
            if is_real and self._recorder:
                self._window.register_callback("on_record_toggled", self._on_record_toggled)

                # Override key handler to add R key for recording
                original_key_handler = self._coordinator._on_key_press

                def extended_key_handler(key: str):
                    if key.lower() == "r":
                        self._on_key_r()
                    else:
                        original_key_handler(key)

                self._window.register_callback("on_key_press", extended_key_handler)

    # --- Start/Stop methods ---

    def _start(self) -> None:
        """Start all background threads."""
        self._running = True

        if self._config.mode == "sim":
            self._start_sim()
        else:
            self._start_real()

        # If a remote camera_service is available and recording is requested,
        # initialize the RemoteRecordClient. Actual start/stop is bracketed
        # around each subgoal dispatch in _video_subgoal_start/_done so that
        # each push produces its own MP4 inside a per-session subdirectory.
        # Also writes a session.json + per-subgoal JSON/XML for replay.
        if (
            self._config.record_video_dir
            and self._config.camera_service_address
        ):
            try:
                from robot_control.nodes.remote_record_client import RemoteRecordClient
                from datetime import datetime as _dt
                from pathlib import Path as _Path
                import json as _json

                self._remote_recorder = RemoteRecordClient(
                    self._config.camera_service_address
                )
                # Per-session subdir under the user-provided base dir.
                session_tag = _dt.now().strftime("%Y-%m-%d_%H-%M-%S")
                self._record_session_dir = str(
                    _Path(self._config.record_video_dir) / f"session_{session_tag}"
                )
                _Path(self._record_session_dir).mkdir(parents=True, exist_ok=True)
                self._record_subgoal_index = 0

                robot_goal = None
                if self._planner is not None and hasattr(self._planner, "_robot_goal_cm"):
                    rg = getattr(self._planner, "_robot_goal_cm", None)
                    if rg is not None:
                        robot_goal = list(rg)
                session_meta = {
                    "session_tag": session_tag,
                    "started_at": _dt.now().isoformat(),
                    "camera_service_address": self._config.camera_service_address,
                    "robot_goal_cm": robot_goal,
                    "mode": self._config.mode,
                    "config_path": self._config.config_path,
                }
                try:
                    (_Path(self._record_session_dir) / "session.json").write_text(
                        _json.dumps(session_meta, indent=2)
                    )
                except Exception as exc:
                    print(f"[Runtime] Could not write session.json: {exc!r}")

                print(
                    f"[Runtime] Per-subgoal video + meta will write to "
                    f"{self._record_session_dir}"
                )
            except Exception as exc:
                print(f"[Runtime] Could not init remote video recording: {exc!r}")
                self._remote_recorder = None
                self._record_session_dir = None
        elif self._config.record_video_dir and not self._config.camera_service_address:
            print(
                "[Runtime] --record-video requested but no --camera-service set; "
                "video recording requires the camera_service."
            )

        # Attach the diagnostics recorder to the planner so plan() emissions
        # appear in plans.jsonl. We do this AFTER _start_*() so the planner
        # is fully initialised.
        if self._diag is not None and self._planner is not None:
            try:
                self._planner.set_diagnostics_recorder(self._diag)
            except AttributeError:
                # Older planner type without diagnostics support — skip silently.
                pass

        # Per-plan sim-success capture. With --capture-sim-success on, every
        # plan() that returns at least one push triggers an inline
        # (xml, chain.json, replay.mp4) dump under <diag-root>/sim_replays/.
        # This is independent of whether the real robot succeeds — useful
        # when the run-end finally block doesn't fire (e.g., the PyQt-viewer
        # event loop stays parked on close), since artifacts land per-plan
        # rather than only at run end.
        if (
            self._capture_sim_success
            and self._diag is not None
            and self._diag.enabled
            and self._planner is not None
        ):
            try:
                self._planner.set_sim_capture_callback(self._capture_plan_replay)
            except AttributeError:
                pass

        # Run an initial connectivity check so the user sees the robot's
        # state in the log immediately — instead of waiting for the first
        # plan() call to surface it. In real mode this is critical since
        # an offline robot means the run will silently produce no execution.
        # Wait briefly so the SerialSender reader thread has a chance to
        # parse its first [STATUS] line from the AP before we sample state.
        if self._config.mode == "real":
            time.sleep(0.5)
            self._check_robot_connectivity(context="startup")

        # Start control loop
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="Runtime-ControlLoop"
        )
        self._control_thread.start()

    def _start_sim(self) -> None:
        """Start simulation components."""
        self._env.start()
        self._sensor.start()

    def _start_real(self) -> None:
        """Start real robot components."""
        if self._remote_observer:
            # Remote camera mode: just start the ZMQ receiver
            print("[Runtime] Starting RemoteObserverNode...")
            if not self._remote_observer.start():
                raise RuntimeError(
                    f"Failed to connect to camera service at "
                    f"{self._config.camera_service_address}"
                )
        else:
            # Local camera mode: start camera + observer + recorder
            print("[Runtime] Starting CameraSensorNode...")
            if not self._sensor.start():
                raise RuntimeError("Failed to start camera")

            self._recorder.subscribe()

            print("[Runtime] Starting ArucoObserver...")
            if not self._observer.start():
                self._recorder.unsubscribe()
                self._sensor.stop()
                raise RuntimeError("Failed to start ArUco observer")

        # Start real environment (serial connection)
        if not self._config.dry_run:
            if not self._env.start():
                if self._remote_observer:
                    self._remote_observer.stop()
                else:
                    self._observer.stop()
                    self._recorder.unsubscribe()
                    self._sensor.stop()
                raise RuntimeError("Failed to start RealEnv")
        else:
            print("[Runtime] DRY RUN - not sending commands to robot")

    def _shutdown(self) -> None:
        """Clean shutdown of all components. Idempotent - safe to call multiple times."""
        if self._shutdown_complete:
            return
        self._shutdown_complete = True
        self._running = False

        if self._config.mode == "sim":
            self._shutdown_sim()
        else:
            self._shutdown_real()

        # Unsubscribe world state
        if self._world:
            self._world.unsubscribe()

        print("[Runtime] Shutdown complete")

    def _shutdown_sim(self) -> None:
        """Shutdown simulation components."""
        if self._sensor:
            self._sensor.stop()
        if self._env:
            self._env.stop()

    def _shutdown_real(self) -> None:
        """Shutdown real robot components."""
        if self._remote_recorder is not None:
            try:
                if self._record_active:
                    self._remote_recorder.stop()
                    self._record_active = False
                self._remote_recorder.close()
            except Exception as exc:
                print(f"[Runtime] Error stopping remote video recording: {exc!r}")
            self._remote_recorder = None
        if self._recorder:
            self._recorder.unsubscribe()
        if not self._config.dry_run and self._env:
            # Send stop command to robot BEFORE closing connection
            print("[Runtime] Sending stop command to robot...")
            try:
                # Send explicit zero speeds
                from robot_control.core.types import Action
                self._env.apply(Action(left_speed=0.0, right_speed=0.0))
                # Also call stop_robot if available
                if hasattr(self._env, "stop_robot"):
                    self._env.stop_robot()
            except Exception as e:
                print(f"[Runtime] Error stopping robot: {e}")
            self._env.stop()
        if self._remote_observer:
            self._remote_observer.stop()
        if self._observer:
            self._observer.stop()
        if self._sensor:
            self._sensor.stop()

    # --- Control loop ---

    def _record_terminal_outcome(self, outcome: str, reason: str) -> None:
        """Retain the first autonomous terminal decision for summary writing."""
        if self._terminal_outcome is None:
            self._terminal_outcome = (outcome, reason)

    def _handle_autonomous_terminal_status(
        self,
        status: str,
        *,
        is_real: bool,
    ) -> bool:
        """Stop a terminal autonomous run and return whether the loop should exit."""
        if status not in AUTONOMOUS_TERMINAL_STATUSES:
            return False

        if not self._terminal_announced:
            self._terminal_announced = True
            if status == PLAN_COMPLETE_STATUS:
                print("[Runtime] Plan complete - stopping robot")
            else:
                print("[Runtime] Planning failed - stopping robot")
            if is_real and not self._config.dry_run:
                if hasattr(self._env, "stop_robot"):
                    self._env.stop_robot()

        if not self._config.quit_on_complete:
            return False

        print("[Runtime] Shutting down")
        self._running = False
        if self._window is not None:
            try:
                self._window.close_window()
            except Exception as exc:
                print(f"[Runtime] window close failed: {exc!r}", flush=True)
        return True

    def _control_loop(self) -> None:
        """Main control loop at configured frequency."""
        dt = 1.0 / self._config.frequency
        is_real = self._config.mode == "real"

        while self._running:
            start = time.time()

            obs = self._world.get()
            if obs is not None:
                if self._planner and self._executor:
                    # === AUTONOMOUS MODE ===
                    action, drawings, status = self._autonomous_step(obs)

                    # Both autonomous terminal states use the same stop/close
                    # path. The GUI-thread-safe close call is load-bearing:
                    # without it run() cannot reach summary writing.
                    if self._handle_autonomous_terminal_status(
                        status, is_real=is_real
                    ):
                        break
                else:
                    # === INTERACTIVE MODE ===
                    action = self._coordinator.step(obs)
                    drawings = self._coordinator.get_drawings()
                    status = self._coordinator.get_status()

                # Apply action
                if not (is_real and self._config.dry_run):
                    self._env.apply(action)

                # Update GUI
                if self._window:
                    self._window.update(obs)
                    self._window.set_status(status)
                    self._window.update_drawings(drawings)
                    self._window.update_action(action)

            # Real mode: update camera view and connection status
            if is_real and self._window:
                if self._observer:
                    vis_frame = self._observer.get_vis_frame()
                    if vis_frame is not None:
                        self._window.update_camera(vis_frame)

                if not self._config.dry_run:
                    self._window.update_connection(self._env.get_status())

            # Maintain rate
            elapsed = time.time() - start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def _autonomous_step(self, obs: Observation) -> Tuple[Action, list, str]:
        """Execute one step of autonomous planning."""
        # Check if current subgoal is done (or no subgoal yet)
        if self._executor.is_done(obs):
            # Only notify if we actually completed a subgoal (not first call)
            if self._executor.has_active_subgoal():
                failed = self._executor.did_fail()
                duration = time.time() - self._subgoal_start_time if self._subgoal_start_time else 0.0
                outcome = "FAILED" if failed else "SUCCESS"
                print(
                    f"[Subgoal] <-- DONE ({outcome}) after {duration:.1f}s, "
                    f"robot now at ({obs.robot_x:.1f},{obs.robot_y:.1f},θ={obs.robot_theta:.1f}°)"
                )
                # Mirror to diagnostics recorder (subgoal lifecycle close +
                # push-physics record). All hooks are no-ops if disabled.
                self._record_subgoal_done(obs, failed=failed)
                self._video_subgoal_done(obs, failed=failed)  # finalize MP4 + meta
                self._planner.notify_subgoal_done(obs, failed=failed)
                self._subgoal_start_time = None

            # Check if plan is complete
            if self._planner.is_complete(obs):
                self._record_terminal_outcome("success", "goal reached")
                return (
                    Action.stop(),
                    self._planner.get_drawings(),
                    PLAN_COMPLETE_STATUS,
                )

            # Get next subgoal — log robot connectivity around the planning call
            # so the user can correlate "no plan found" diagnostics with whether
            # the robot was actually online at the time.
            self._check_robot_connectivity(context="plan()")
            subgoal = self._planner.plan(obs)
            if subgoal:
                if self._config.step_confirm:
                    decision = self._confirm_subgoal(subgoal, obs)
                    if decision == "abort":
                        self._running = False
                        return Action.stop(), self._planner.get_drawings(), "Aborted by user"
                    if decision == "skip":
                        self._planner.notify_subgoal_done(obs, failed=True)
                        return Action.stop(), self._planner.get_drawings(), "Skipped by user"
                self._log_subgoal_dispatch(subgoal, obs)
                self._record_subgoal_start(subgoal, obs)
                self._video_subgoal_start(subgoal, obs)  # start MP4 + write meta
                self._subgoal_start_time = time.time()
                self._executor.set_subgoal(subgoal, obs)
            else:
                # Some generic planners transition to complete inside plan(),
                # so check once more. An empty plan by itself is never proof
                # of success.
                if self._planner.is_complete(obs):
                    self._record_terminal_outcome("success", "goal reached")
                    status = PLAN_COMPLETE_STATUS
                else:
                    self._record_terminal_outcome(
                        "failure",
                        "planner returned no subgoal while goal was unreachable",
                    )
                    status = PLANNING_FAILED_STATUS
                return Action.stop(), self._planner.get_drawings(), status

        # Execute
        action = self._executor.step(obs)

        # Combine drawings: plan overview + execution details
        drawings = self._planner.get_drawings() + self._executor.get_drawings()

        # Status
        status = f"Autonomous: {self._executor.get_status()}"

        return action, drawings, status

    # ------------------------------------------------------------ diagnostics

    def _capture_scene_snapshot(self, when: str) -> Dict[str, Optional[str]]:
        # Capture a scene snapshot (jpg + json + xml) when --capture-scene
        # is enabled. `when` ∈ {"before", "after"}. Returns a dict naming
        # which artifacts succeeded; empty dict when capture is disabled or
        # the recorder is None.
        if self._diag is None or not self._capture_scene:
            return {}
        try:
            # Lazy import — diagnostics submodules only loaded when needed.
            from robot_control.diagnostics.scene_serializer import build_scene_state
            from robot_control.diagnostics.sim_renderer import render_top_down
        except Exception as exc:
            self._diag.log_capture_error(when, "imports", repr(exc))
            return {}

        # We need a current observation. In real mode the WorldState is
        # populated by the camera service / ArucoObserver; in sim mode by
        # SimSensorNode. If no observation has arrived yet we still want to
        # try the snapshot — most fields just become null.
        obs = None
        try:
            if self._world is not None:
                obs = self._world.get()
        except Exception:
            obs = None
        if obs is None:
            self._diag.log_capture_error(when, "observation", "no observation available")
            return {}

        # 1) JPEG: real mode requests from the camera service; sim mode
        # renders a synthetic top-down view from the same observation.
        jpg_bytes: Optional[bytes] = None
        if self._config.mode == "real":
            addr = self._config.camera_service_address
            if not addr:
                self._diag.log_capture_error(
                    when, "jpg",
                    "real mode but --camera-service not set",
                )
            else:
                try:
                    from robot_control.diagnostics.capture import request_camera_frame
                    jpg_bytes = request_camera_frame(addr, kind="vis", timeout_sec=2.0)
                    if jpg_bytes is None:
                        self._diag.log_capture_error(
                            when, "jpg",
                            f"camera_service at {addr} did not respond "
                            f"(REP socket at port+1 — restart camera_service "
                            f"to enable diagnostic frames)",
                        )
                except Exception as exc:
                    self._diag.log_capture_error(when, "jpg",
                                                f"capture failed: {exc!r}")
        else:
            # Sim mode: render synthetic top-down view from current obs.
            try:
                if self._workspace_config is None:
                    raise RuntimeError("workspace_config is None")
                jpg_bytes = render_top_down(
                    self._workspace_config, obs,
                    title=f"scene_{when}",
                )
            except Exception as exc:
                self._diag.log_capture_error(when, "jpg",
                                            f"render failed: {exc!r}")

        # 2) JSON: structured state from observation.
        json_payload: Optional[Dict[str, Any]] = None
        try:
            if self._workspace_config is None:
                raise RuntimeError("workspace_config is None")
            origin_offset = None
            target_id = None
            if self._real_env_config is not None:
                target_id = self._real_env_config.robot_id
            json_payload = build_scene_state(
                mode=self._config.mode,
                workspace=self._workspace_config,
                observation=obs,
                workspace_origin_offset_cm=origin_offset,
                robot_marker_id=target_id,
            )
        except Exception as exc:
            self._diag.log_capture_error(when, "json", f"serializer failed: {exc!r}")

        # 3) XML: the planner generates a MuJoCo XML on each plan() call.
        # When --capture-scene is on, we pointed planner.debug_xml_path at
        # scene_{before|after}.xml via runtime startup (set below in _start()
        # for "before"). For "after" we explicitly trigger one more XML
        # generation.
        xml_source: Optional[Path] = None
        try:
            xml_source = self._get_scene_xml_for(when)
        except Exception as exc:
            self._diag.log_capture_error(when, "xml", f"acquire failed: {exc!r}")

        # save_scene copies/writes everything atomically and logs errors.
        return self._diag.save_scene(when, jpg_bytes, json_payload, xml_source)

    # ---------------------------------------------------- sim-success capture

    def _capture_sim_replay_start_xml(self) -> None:
        # Persist a single XML snapshot of the scene as it stands at runtime
        # start. _write_sim_success_artifacts feeds this into a headless
        # RLEnvironment when the real run succeeds.
        if not self._capture_sim_success:
            return
        if self._diag is None or not self._diag.enabled or self._planner is None:
            return
        if not hasattr(self._planner, "dump_scene_xml"):
            return

        obs = None
        try:
            if self._world is not None:
                obs = self._world.get()
        except Exception:
            obs = None
        if obs is None:
            print("[Runtime] sim-success: no observation at start; skipping start XML",
                  flush=True)
            return

        try:
            from pathlib import Path as _Path
            xml_path = _Path(self._diag.root) / "sim_replay_start.xml"
            written = self._planner.dump_scene_xml(obs, str(xml_path))
            self._sim_replay_start_xml = written if written else None
            if self._sim_replay_start_xml:
                print(f"[Runtime] sim-success: start XML → {self._sim_replay_start_xml}",
                      flush=True)

                # Stash the start pose in sim units so the success-replay
                # MP4 can teleport the car before stepping (car XMLs don't
                # bake pose into the included little_car.xml). Convert via
                # the bridge's cm→sim helper so we match planning_service's
                # set_robot_pose call exactly.
                bridge = getattr(self._planner, "_bridge", None)
                if bridge is not None and hasattr(bridge, "_cm_to_sim"):
                    try:
                        import math as _math
                        rx_m, ry_m = bridge._cm_to_sim(
                            float(obs.robot_x), float(obs.robot_y)
                        )
                        rtheta_rad = _math.radians(float(obs.robot_theta))
                        self._sim_replay_start_pose_sim = (rx_m, ry_m, rtheta_rad)
                    except Exception as exc:
                        print(f"[Runtime] sim-success: start pose conversion "
                              f"failed: {exc!r}", flush=True)
        except Exception as exc:
            print(f"[Runtime] sim-success: start XML dump failed: {exc!r}", flush=True)

    def _write_sim_success_artifacts(self, outcome: str) -> None:
        # On success: write success_chain.json and (separately) trigger the
        # sim replay → MP4. On fail/abort/crashed: write partial_chain.json
        # only. Anything that goes wrong here is best-effort — never raises.
        if not self._capture_sim_success:
            return
        if self._diag is None or not self._diag.enabled:
            return

        from pathlib import Path as _Path
        import json as _json

        diag_root = _Path(self._diag.root)
        is_success = (outcome == "success")
        chain_filename = "success_chain.json" if is_success else "partial_chain.json"
        chain_path = diag_root / chain_filename

        payload = {
            "outcome": outcome,
            "start_xml": (
                _Path(self._sim_replay_start_xml).name
                if self._sim_replay_start_xml else None
            ),
            "chain": self._sim_success_chain,
            # [x_m, y_m, theta_rad] used by sim_replay_subprocess to
            # teleport the car before stepping. Null for sphere or when
            # no start observation was available.
            "starting_robot_pose_sim": (
                list(self._sim_replay_start_pose_sim)
                if self._sim_replay_start_pose_sim is not None else None
            ),
        }
        try:
            chain_path.write_text(_json.dumps(payload, indent=2))
            print(f"[Runtime] sim-success: chain ({len(self._sim_success_chain)} "
                  f"push(es)) → {chain_path}", flush=True)
        except Exception as exc:
            print(f"[Runtime] sim-success: writing {chain_filename} failed: {exc!r}",
                  flush=True)
            return

        if not is_success:
            return

        # Replay only fires on success. Skip if we never captured a start XML
        # or never executed any pushes.
        if self._sim_replay_start_xml is None:
            print("[Runtime] sim-success: no start XML captured; skipping replay",
                  flush=True)
            return
        if not self._sim_success_chain:
            print("[Runtime] sim-success: empty chain; skipping replay", flush=True)
            return

        try:
            from robot_control.diagnostics.sim_replay import render_chain_to_mp4
            mp4_path = diag_root / "success_sim_replay.mp4"
            # NAMOPlanner threads its YAML through NAMOPlanBridge, which
            # writes an "effective" overlaid config — that's the one RLEnv
            # should load so sim and real share the same wavefront tuning.
            bridge = getattr(self._planner, "_bridge", None)
            namo_config_path = (
                str(getattr(bridge, "_effective_namo_config_path", "")) or None
                if bridge is not None else None
            )
            workspace_cfg = self._workspace_config
            render_chain_to_mp4(
                start_xml=self._sim_replay_start_xml,
                namo_config=namo_config_path,
                chain=self._sim_success_chain,
                output_mp4=str(mp4_path),
                workspace=workspace_cfg,
                starting_robot_pose_sim=self._sim_replay_start_pose_sim,
            )
        except Exception as exc:
            print(f"[Runtime] sim-success: replay → MP4 failed: {exc!r}", flush=True)

    def _capture_plan_replay(
        self,
        *,
        xml_content: str,
        push_chain: List[Dict[str, Any]],
        attempt_index: int,
        starting_robot_pose_sim: Tuple[float, float, float],
    ) -> None:
        """Per-plan sim-success capture — installed on the planner.

        Called from inside ``NAMOPlanner.plan()`` immediately after a plan
        with at least one PushSubgoal is produced. Writes
        ``<diag-root>/sim_replays/plan_NNN_start.xml``,
        ``plan_NNN_chain.json``, and renders ``plan_NNN_replay.mp4`` via
        :func:`sim_replay.render_chain_to_mp4`.

        Errors are caught and logged — sim-capture is best-effort and must
        never break planning.
        """
        if self._diag is None or not self._diag.enabled:
            return
        if not push_chain:
            return

        self._sim_plan_capture_count += 1
        plan_idx = self._sim_plan_capture_count

        from pathlib import Path as _Path
        import json as _json

        diag_root = _Path(self._diag.root)
        out_dir = diag_root / "sim_replays"
        out_dir.mkdir(parents=True, exist_ok=True)

        # Zero-padded so files sort lexicographically (matches up to 999 plans
        # per run, which is well above what we observe in practice).
        prefix = f"plan_{plan_idx:03d}"
        xml_path = out_dir / f"{prefix}_start.xml"
        chain_path = out_dir / f"{prefix}_chain.json"
        mp4_path = out_dir / f"{prefix}_replay.mp4"

        try:
            xml_path.write_text(xml_content)
        except Exception as exc:
            print(f"[Runtime] per-plan capture: writing {xml_path.name} failed: {exc!r}",
                  flush=True)
            return

        payload = {
            "plan_index": plan_idx,
            "attempt_index": attempt_index,
            "start_xml": xml_path.name,
            "chain": push_chain,
            # [x_m, y_m, theta_rad] — the pose the replay env teleports the
            # car to before stepping. Captured here because car XMLs don't
            # bake the robot pose (see little_car.xml freejoint spawn).
            "starting_robot_pose_sim": list(starting_robot_pose_sim),
        }
        try:
            chain_path.write_text(_json.dumps(payload, indent=2))
        except Exception as exc:
            print(f"[Runtime] per-plan capture: writing {chain_path.name} failed: {exc!r}",
                  flush=True)
            return

        # Render MP4 inline. The C++ RLEnvironment resolves motion-primitive
        # paths in the namo config (e.g., "data/motion_primitives_1x.dat")
        # relative to the *current working directory*, so we have to chdir
        # into namo_cpp/ for the duration of the render — same workaround
        # NAMOPlanBridge.plan() uses (see namo_bridge.py around line 318).
        # Without this the ctor raises "Motion primitives missing for shape
        # 'square': expected data/motion_primitives_1x_square.dat".
        import os as _os
        from pathlib import Path as _Path2

        mp4_written: Optional[str] = None
        try:
            from robot_control.diagnostics.sim_replay import render_chain_to_mp4
            bridge = getattr(self._planner, "_bridge", None)
            namo_config_path = (
                str(getattr(bridge, "_effective_namo_config_path", "")) or None
                if bridge is not None else None
            )

            from robot_control.planner.namo_binding_loader import resolve_namo_cpp_dir
            namo_cpp_dir = resolve_namo_cpp_dir(_Path2(__file__).resolve())
            prev_cwd = _os.getcwd()
            try:
                if namo_cpp_dir.is_dir():
                    _os.chdir(str(namo_cpp_dir))
                mp4_written = render_chain_to_mp4(
                    start_xml=str(xml_path),
                    namo_config=namo_config_path,
                    chain=push_chain,
                    output_mp4=str(mp4_path),
                    workspace=self._workspace_config,
                    starting_robot_pose_sim=starting_robot_pose_sim,
                )
            finally:
                _os.chdir(prev_cwd)
        except Exception as exc:
            print(f"[Runtime] per-plan capture: replay → {mp4_path.name} raised: {exc!r}",
                  flush=True)
            return

        if mp4_written is None:
            # render_chain_to_mp4 logs its own [sim_replay] error line and
            # returns None on failure; surface that as a plan-level failure
            # so the user knows the MP4 didn't land even though XML+JSON did.
            print(
                f"[Runtime] sim-success per-plan: plan #{plan_idx} "
                f"({len(push_chain)} push(es)) XML+JSON written but MP4 render "
                f"failed (see [sim_replay] line above) → {out_dir}/",
                flush=True,
            )
            return

        print(
            f"[Runtime] sim-success per-plan: plan #{plan_idx} ({len(push_chain)} "
            f"push(es)) → {out_dir}/", flush=True
        )

    def _determine_outcome(self) -> Tuple[str, str]:
        # The control loop's retained decision is authoritative. Re-evaluating
        # here previously let console status and summary.json disagree.
        terminal_outcome = getattr(self, "_terminal_outcome", None)
        if terminal_outcome is not None:
            return terminal_outcome

        # Best-effort fallback for aborts or legacy callers that stopped before
        # _autonomous_step recorded a terminal state. Three signals:
        #   1. Planner reported "planning failed" (no plan possible) → failure
        #   2. Latest obs satisfies planner.is_complete() → success
        #   3. Otherwise → aborted (e.g. user closed the window)
        if self._planner is None:
            return "unknown", "no planner attached"

        # Planning failure flag exposed by NAMOPlanner.
        if getattr(self._planner, "_planning_failed", False):
            return "failure", "planner exhausted retries without finding a plan"

        # Try is_complete on the latest observation.
        try:
            if self._world is not None:
                obs = self._world.get()
                if obs is not None and self._planner.is_complete(obs):
                    return "success", "goal reached"
        except Exception as exc:
            return "unknown", f"is_complete check failed: {exc!r}"

        # Normal exit without success → aborted.
        return "aborted", "runtime stopped without a terminal planner result"

    def _write_summary(
        self,
        outcome: str,
        outcome_reason: str,
        scene_before: Dict[str, Optional[str]],
        scene_after: Dict[str, Optional[str]],
    ) -> None:
        # No-op if recorder is disabled. Anything that fails inside this
        # method is caught and logged so a buggy summary write can't break
        # a successful run.
        if self._diag is None or not self._diag.enabled:
            return
        try:
            started = self._started_at_epoch or 0.0
            ended = self._ended_at_epoch or time.time()
            duration = max(0.0, ended - started)

            # Final pose + distance to goal — best effort.
            final_pose: Optional[list] = None
            final_dist: Optional[float] = None
            goal_target_cm: Optional[list] = None
            try:
                if self._planner is not None:
                    goal = getattr(self._planner, "_robot_goal_cm", None)
                    if goal is not None:
                        goal_target_cm = list(goal)
                if self._world is not None:
                    obs = self._world.get()
                    if obs is not None:
                        final_pose = [obs.robot_x, obs.robot_y, obs.robot_theta]
                        if goal_target_cm is not None:
                            dx = obs.robot_x - goal_target_cm[0]
                            dy = obs.robot_y - goal_target_cm[1]
                            final_dist = float((dx * dx + dy * dy) ** 0.5)
            except Exception:
                pass

            # Accumulate offline duration if currently still offline at exit.
            offline_total = self._offline_total_sec
            if self._offline_since is not None:
                offline_total += time.time() - self._offline_since

            payload = {
                "run_name": self._diag.root.name,
                "outcome": outcome,
                "outcome_reason": outcome_reason,
                "started_at_epoch": started,
                "ended_at_epoch": ended,
                "duration_sec": duration,
                "mode": self._config.mode,
                "strategy": getattr(self._planner, "_goal_strategy", None),
                "algorithm": getattr(self._planner, "_algorithm", None),
                "goal_target_cm": goal_target_cm,
                "final_robot_pose_cm": final_pose,
                "final_distance_to_goal_cm": final_dist,
                "totals": dict(self._diag.totals),
                "connectivity": {
                    "online_at_start": self._online_at_start,
                    "transitions": self._diag.totals.get("connectivity_transitions", 0),
                    "total_offline_sec": offline_total,
                    "offline_during_plan_count": self._diag.totals.get(
                        "offline_during_plan_count", 0
                    ),
                },
                "scene_capture": {
                    "scene_before_jpg": scene_before.get("jpg"),
                    "scene_before_json": scene_before.get("json"),
                    "scene_before_xml": scene_before.get("xml"),
                    "scene_after_jpg": scene_after.get("jpg"),
                    "scene_after_json": scene_after.get("json"),
                    "scene_after_xml": scene_after.get("xml"),
                    "errors": [],  # Aggregated in scene_capture_errors.log
                },
            }
            self._diag.write_summary(payload)
        except Exception as exc:
            print(f"[DIAG] ⚠️ summary write failed: {exc!r}", flush=True)

    def _get_scene_xml_for(self, when: str) -> Optional[Path]:
        # For scene_before/after we generate a fresh XML representing the
        # current observation, without invoking the search. The planner
        # exposes dump_scene_xml(obs, path) for this. We write to a transient
        # path inside the run dir; the recorder.save_scene() copies it to its
        # final destination (scene_{when}.xml).
        if self._planner is None or self._diag is None:
            return None
        if not hasattr(self._planner, "dump_scene_xml"):
            return None
        obs = None
        try:
            if self._world is not None:
                obs = self._world.get()
        except Exception:
            obs = None
        if obs is None:
            return None
        # Temporary path — save_scene copies to scene_{when}.xml.
        tmp_path = self._diag.root / f"_scene_{when}_tmp.xml"
        try:
            written = self._planner.dump_scene_xml(obs, tmp_path)
        except Exception as exc:
            print(f"[DIAG] dump_scene_xml failed: {exc!r}", flush=True)
            return None
        if written is None:
            return None
        return Path(written)

    def _check_robot_connectivity(self, context: str) -> bool:
        # Snapshot robot connectivity from the sender and log a one-liner
        # when state matters. Returns True if the target robot is currently
        # registered with the AP, False otherwise (sim mode always returns
        # True since there's no AP).
        if self._env is None or not hasattr(self._env, "get_status"):
            return True
        if self._real_env_config is None:
            # sim or no real config — nothing to check
            return True

        try:
            status = self._env.get_status()
        except Exception:
            # Don't let a status read failure crash the loop
            return True

        target_id = self._real_env_config.robot_id
        ap_connected = bool(getattr(status, "is_connected", False))
        alive_ids = list(getattr(status, "alive_robot_ids", []) or [])
        last_update_time = float(getattr(status, "last_update_time", 0.0) or 0.0)
        now = time.time()
        never_received_status = (last_update_time == 0.0)
        stale = never_received_status or (now - last_update_time > 3.0)
        online = ap_connected and (target_id in alive_ids) and not stale

        # Format "since last [STATUS] from AP" without the epoch-seconds gotcha
        # when last_update_time has never been set.
        if never_received_status:
            last_status_str = "never (no [STATUS] line parsed since startup)"
        else:
            last_status_str = f"{now - last_update_time:.1f}s ago"

        # Print a clear transition event whenever state flips, regardless of
        # debounce, so the log shows when the robot went online/offline.
        if self._last_seen_robot_online is not None and self._last_seen_robot_online != online:
            transition = "ONLINE" if online else "OFFLINE"
            print(
                f"[Runtime] 🔌 Robot {target_id} → {transition} "
                f"(alive_ids={alive_ids}, ap_connected={ap_connected}, "
                f"last_status={last_status_str})",
                flush=True,
            )
            self._last_offline_warn_time = now  # Reset debounce on transition

            # Update offline-duration accumulator for the summary.
            if not online:
                self._offline_since = now
            else:
                if self._offline_since is not None:
                    self._offline_total_sec += now - self._offline_since
                    self._offline_since = None

            # Mirror to diagnostics recorder as a structured event.
            if self._diag is not None:
                self._diag.record_connectivity({
                    "event": "transition",
                    "online": online,
                    "alive_ids": alive_ids,
                    "ap_connected": ap_connected,
                    "last_status_age_sec": (
                        None if never_received_status else now - last_update_time
                    ),
                    "target_robot_id": target_id,
                    "context": context,
                })

        # Capture the first observed state for the summary's "online_at_start".
        # On the first check, print a prominent line either way so the user
        # immediately sees the connectivity state in the log without having
        # to wait for the first plan() call's periodic warn.
        if self._online_at_start is None:
            self._online_at_start = online
            if online:
                print(
                    f"[Runtime] 🔌 Robot {target_id} ONLINE at startup "
                    f"(alive_ids={alive_ids}, ap_connected={ap_connected})",
                    flush=True,
                )
            else:
                # Bigger banner — robot not registered with AP means execution
                # will fail. The user needs to see this immediately.
                stale_str = "stale" if stale else "fresh"
                print(
                    "\n" + "!" * 70 + "\n"
                    f"!! ROBOT {target_id} OFFLINE at startup !!\n"
                    f"!!   ap_connected={ap_connected}, alive_ids={alive_ids}, "
                    f"last_status={last_status_str} [{stale_str}]\n"
                    f"!!   Planning will run (sim-only), but the robot CANNOT execute.\n"
                    f"!!   Restart the robot or check the camera_service serial link.\n"
                    + "!" * 70 + "\n",
                    flush=True,
                )
                self._last_offline_warn_time = now  # Suppress immediate dup warn
            if self._diag is not None:
                self._diag.record_connectivity({
                    "event": "initial",
                    "online": online,
                    "alive_ids": alive_ids,
                    "ap_connected": ap_connected,
                    "target_robot_id": target_id,
                    "context": context,
                })

        self._last_seen_robot_online = online

        # When offline, periodically remind the user that subsequent stats
        # may be misleading (planning will run in sim regardless, but the
        # robot can't execute, and any "no subgoals" diagnostic should be
        # interpreted alongside this connectivity state).
        if not online and (now - self._last_offline_warn_time) >= self._offline_warn_interval_sec:
            stale_str = "stale" if stale else "fresh"
            print(
                f"[Runtime] ⚠️ Robot {target_id} OFFLINE during {context} "
                f"(ap_connected={ap_connected}, alive_ids={alive_ids}, "
                f"last_status={last_status_str} [{stale_str}]). "
                f"Diagnostic stats below this point reflect planning only; "
                f"execution will not run until robot reconnects.",
                flush=True,
            )
            self._last_offline_warn_time = now
            if self._diag is not None and context == "plan()":
                # Only count "offline_during_plan" events — periodic warns
                # outside planning are noise for the summary aggregate.
                self._diag.record_connectivity({
                    "event": "offline_during_plan",
                    "online": False,
                    "alive_ids": alive_ids,
                    "ap_connected": ap_connected,
                    "context": context,
                })

        return online

    # State for pairing subgoal_start / subgoal_end records via the recorder.
    # We track only the most recent active subgoal_id since the executor only
    # ever has one active subgoal at a time.
    _active_subgoal_id: int = 0
    _active_subgoal_type: str = "unknown"

    def _record_subgoal_start(self, subgoal, obs: Observation) -> None:
        # Stash push metadata up-front so _record_subgoal_done can append to
        # the sim-success chain on success without needing the original
        # subgoal object. Independent of diagnostics being enabled.
        if isinstance(subgoal, PushSubgoal):
            self._pending_push_meta = {
                "object_id": subgoal.object_id,
                "edge_idx": subgoal.edge_idx,
                "push_steps": subgoal.push_steps,
                "depth": subgoal.push_steps - 1,
            }
        else:
            self._pending_push_meta = None

        # No-op when diagnostics is disabled.
        if self._diag is None:
            return
        try:
            if isinstance(subgoal, PushSubgoal):
                obj = obs.objects.get(subgoal.object_id)
                obj_pose = [obj.x, obj.y, obj.theta] if obj is not None else None
                payload = {
                    "type": "push",
                    "object_id": subgoal.object_id,
                    "edge_idx": subgoal.edge_idx,
                    "push_steps": subgoal.push_steps,
                    "dispatched_robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                    "dispatched_object_pose_cm": obj_pose,
                    "dispatched_obs_timestamp": (
                        float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
                    ),
                }
                self._active_subgoal_type = "push"
            elif isinstance(subgoal, NavigateSubgoal):
                payload = {
                    "type": "navigate",
                    "target_cm": [subgoal.x, subgoal.y],
                    "target_theta_deg": subgoal.theta,
                    "dispatched_robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                    "dispatched_obs_timestamp": (
                        float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
                    ),
                }
                self._active_subgoal_type = "navigate"
            else:
                payload = {
                    "type": type(subgoal).__name__,
                    "repr": repr(subgoal),
                    "dispatched_robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                    "dispatched_obs_timestamp": (
                        float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
                    ),
                }
                self._active_subgoal_type = "other"
            self._active_subgoal_id = self._diag.record_subgoal_start(payload)
        except Exception as exc:
            print(f"[DIAG] record_subgoal_start failed: {exc!r}", flush=True)
            self._active_subgoal_id = 0

    def _record_subgoal_done(self, obs: Observation, failed: bool) -> None:
        # Append successful pushes to the sim-success chain. We do this
        # regardless of whether diagnostics is enabled — the chain only gets
        # written out at run-end if _capture_sim_success is on, and the
        # bookkeeping cost is negligible.
        if not failed and self._pending_push_meta is not None:
            self._sim_success_chain.append(dict(self._pending_push_meta))
        self._pending_push_meta = None

        # No-op when diagnostics is disabled OR there's no open subgoal record.
        if self._diag is None or self._active_subgoal_id == 0:
            return
        try:
            outcome = {
                "outcome": "failed" if failed else "success",
                "completed_robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                "completed_obs_timestamp": (
                    float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
                ),
            }
            self._diag.record_subgoal_end(self._active_subgoal_id, outcome)
        except Exception as exc:
            print(f"[DIAG] record_subgoal_end failed: {exc!r}", flush=True)

        # When the closed subgoal was a push, also emit a push record from the
        # controller's tracked state (Δpose, stuck flag, etc.).
        if self._active_subgoal_type == "push":
            try:
                push_controller = self._controllers.get("push")
                if push_controller is not None and hasattr(push_controller, "get_last_push_summary"):
                    summary = push_controller.get_last_push_summary(obs)
                    if summary is not None:
                        # Tag with the subgoal_id for cross-reference.
                        summary["subgoal_id"] = self._active_subgoal_id
                        self._diag.record_push(summary)
            except Exception as exc:
                print(f"[DIAG] record_push failed: {exc!r}", flush=True)

        # Clear active subgoal marker.
        self._active_subgoal_id = 0
        self._active_subgoal_type = "unknown"

    # --- Video recording + meta capture: per-subgoal bracket ---

    @staticmethod
    def _subgoal_tag(subgoal) -> str:
        """Filename tag describing this subgoal (used by both video and meta)."""
        if isinstance(subgoal, PushSubgoal):
            return f"push_{subgoal.object_id}_e{subgoal.edge_idx}_s{subgoal.push_steps}"
        if isinstance(subgoal, NavigateSubgoal):
            return "navigate"
        return type(subgoal).__name__.lower()

    def _video_subgoal_start(self, subgoal, obs: Observation = None) -> None:
        """Start a new MP4 clip for this subgoal + write meta JSON + XML snapshot."""
        if self._record_session_dir is None:
            return
        self._record_subgoal_index += 1
        tag = self._subgoal_tag(subgoal)
        base = f"subgoal_{self._record_subgoal_index:03d}_{tag}"

        if self._remote_recorder is not None:
            try:
                path = self._remote_recorder.start(self._record_session_dir, filename=base)
                self._record_active = path is not None
            except Exception as exc:
                print(f"[Runtime] _video_subgoal_start (video) failed: {exc!r}", flush=True)
                self._record_active = False
        else:
            self._record_active = False

        if obs is None:
            return
        try:
            from pathlib import Path as _Path
            from datetime import datetime as _dt
            import json as _json

            session_dir = _Path(self._record_session_dir)
            xml_path = session_dir / f"{base}.xml"
            written_xml = None
            if self._planner is not None and hasattr(self._planner, "dump_scene_xml"):
                try:
                    written_xml = self._planner.dump_scene_xml(obs, str(xml_path))
                except Exception as exc:
                    print(f"[Runtime] dump_scene_xml failed for {base}: {exc!r}", flush=True)

            meta = {
                "subgoal_index": self._record_subgoal_index,
                "tag": tag,
                "timestamp": _dt.now().isoformat(),
                "video_path": f"{base}.mp4" if self._record_active else None,
                "xml_path": f"{base}.xml" if written_xml else None,
                "dispatched_robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                "dispatched_object_poses_cm": {
                    name: [o.x, o.y, o.theta] for name, o in obs.objects.items()
                },
            }
            if isinstance(subgoal, PushSubgoal):
                meta.update({
                    "type": "push",
                    "object_id": subgoal.object_id,
                    "edge_idx": subgoal.edge_idx,
                    "push_steps": subgoal.push_steps,
                })
            elif isinstance(subgoal, NavigateSubgoal):
                meta.update({
                    "type": "navigate",
                    "target_x_cm": subgoal.x,
                    "target_y_cm": subgoal.y,
                    "target_theta_deg": subgoal.theta,
                })
            else:
                meta["type"] = type(subgoal).__name__

            (session_dir / f"{base}.json").write_text(_json.dumps(meta, indent=2))
        except Exception as exc:
            print(f"[Runtime] _video_subgoal_start (meta) failed: {exc!r}", flush=True)

    def _video_subgoal_done(self, obs: Observation = None, failed: bool = False) -> None:
        """Stop the current MP4 clip + append outcome to per-subgoal meta JSON."""
        if self._remote_recorder is not None and self._record_active:
            try:
                self._remote_recorder.stop()
            except Exception as exc:
                print(f"[Runtime] _video_subgoal_done (video) failed: {exc!r}", flush=True)
            finally:
                self._record_active = False

        if obs is None or self._record_session_dir is None or self._record_subgoal_index == 0:
            return
        try:
            from pathlib import Path as _Path
            import json as _json

            session_dir = _Path(self._record_session_dir)
            prefix = f"subgoal_{self._record_subgoal_index:03d}_"
            matches = sorted(session_dir.glob(f"{prefix}*.json"))
            if not matches:
                return
            meta_path = matches[-1]
            meta = _json.loads(meta_path.read_text())
            meta["outcome"] = "failed" if failed else "success"
            meta["completed_robot_pose_cm"] = [obs.robot_x, obs.robot_y, obs.robot_theta]
            meta["completed_object_poses_cm"] = {
                name: [o.x, o.y, o.theta] for name, o in obs.objects.items()
            }
            meta_path.write_text(_json.dumps(meta, indent=2))
        except Exception as exc:
            print(f"[Runtime] _video_subgoal_done (meta) failed: {exc!r}", flush=True)

    def _log_subgoal_dispatch(self, subgoal, obs: Observation) -> None:
        """Print a one-line summary when a subgoal is dispatched to the executor."""
        if isinstance(subgoal, PushSubgoal):
            obj = obs.objects.get(subgoal.object_id)
            obj_desc = (
                f"obj '{subgoal.object_id}' at ({obj.x:.1f},{obj.y:.1f},θ={obj.theta:.1f}°)"
                if obj is not None else f"obj '{subgoal.object_id}' (NOT IN OBS)"
            )
            print(
                f"[Subgoal] --> DISPATCH PUSH: {obj_desc}, edge={subgoal.edge_idx}, "
                f"push_steps={subgoal.push_steps}, "
                f"robot at ({obs.robot_x:.1f},{obs.robot_y:.1f},θ={obs.robot_theta:.1f}°)"
            )
        elif isinstance(subgoal, NavigateSubgoal):
            theta_part = f" θ={subgoal.theta:.1f}°" if subgoal.theta is not None else ""
            print(
                f"[Subgoal] --> DISPATCH NAVIGATE: target=({subgoal.x:.1f},{subgoal.y:.1f}){theta_part}, "
                f"robot at ({obs.robot_x:.1f},{obs.robot_y:.1f},θ={obs.robot_theta:.1f}°)"
            )
        else:
            print(f"[Subgoal] --> DISPATCH {type(subgoal).__name__}: {subgoal!r}")

    def _confirm_subgoal(self, subgoal, obs: Observation) -> str:
        """Block on stdin until user confirms, skips, or aborts the subgoal.

        Returns one of: "execute", "skip", "abort".
        """
        print("\n" + "=" * 60)
        print("[Confirm] Next subgoal:")
        if isinstance(subgoal, PushSubgoal):
            print(f"  type:       PUSH")
            print(f"  object:     {subgoal.object_id}")
            print(f"  edge_idx:   {subgoal.edge_idx}")
            print(f"  push_steps: {subgoal.push_steps}")
            obj = obs.objects.get(subgoal.object_id)
            if obj is not None:
                print(f"  object at:  ({obj.x:.1f}, {obj.y:.1f}) theta={obj.theta:.1f}°")
        elif isinstance(subgoal, NavigateSubgoal):
            print(f"  type:       NAVIGATE")
            print(f"  target:     ({subgoal.x:.1f}, {subgoal.y:.1f})"
                  + (f" theta={subgoal.theta:.1f}°" if subgoal.theta is not None else ""))
        else:
            print(f"  type:       {type(subgoal).__name__}")
            print(f"  repr:       {subgoal!r}")
        print(f"  robot at:   ({obs.robot_x:.1f}, {obs.robot_y:.1f}) theta={obs.robot_theta:.1f}°")
        print("=" * 60)
        try:
            response = input("[Confirm] ENTER=execute, s=skip (blacklist), q=abort: ").strip().lower()
        except EOFError:
            response = ""
        if response == "q":
            return "abort"
        if response == "s":
            return "skip"
        return "execute"

    # --- Event handlers ---

    def _on_quit(self) -> None:
        """Handle quit event (escape key)."""
        self.stop()

    def _on_emergency_stop(self) -> None:
        """Handle emergency stop (space key)."""
        print("[Runtime] Emergency stop")
        if self._config.mode == "real" and not self._config.dry_run:
            if hasattr(self._env, "stop_robot"):
                self._env.stop_robot()

    def _on_record_toggled(self, recording: bool) -> None:
        """Handle record button toggle from GUI."""
        if self._recorder:
            if recording:
                self._recorder.start()
            else:
                self._recorder.stop()

    def _on_key_r(self) -> None:
        """Handle R key for recording toggle."""
        if self._recorder and self._window:
            is_recording = self._recorder.toggle()
            self._window.set_recording(is_recording)

    # --- Banner ---

    def _print_banner(self) -> None:
        """Print startup banner."""
        mode = "Simulation" if self._config.mode == "sim" else "Real Robot"
        control_mode = "Autonomous" if self._planner else "Interactive"
        print("\n" + "=" * 50)
        print(f"Robot Control - {mode} ({control_mode})")
        print("=" * 50)

        if self._config.mode == "real" and self._real_env_config:
            print(f"Serial port: {self._real_env_config.port}")
            print(f"Robot ID: {self._real_env_config.robot_id}")
            print(f"Dry run: {self._config.dry_run}")

        if self._planner:
            nav = self._controllers.get("navigation")
            push = self._controllers.get("push")
            nav_s = f"{nav.max_speed:.2f}" if nav is not None else "?"
            push_s = f"{push.max_speed:.2f}" if push is not None else "?"
            print(f"Speed (autonomous): nav={nav_s}, push={push_s}")
        else:
            print(f"Speed (manual): {self._coordinator.current_speed}")
        print("=" * 50)
        if self._planner:
            print("Running in autonomous mode")
        else:
            print("Select controller from right panel")
            print("Press +/- to change speed")
        if self._config.mode == "real":
            print("Press 'r' to toggle recording")
        print("Press 'space' for emergency stop")
        print("Press 'escape' to quit")
        print("=" * 50 + "\n")
