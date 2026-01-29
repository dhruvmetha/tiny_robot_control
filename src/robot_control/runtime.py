"""Runtime for orchestrating robot control with automatic node spawning."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

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
from robot_control.core.types import Action, Observation, WorkspaceConfig
from robot_control.core.world_state import WorldState
from robot_control.executor import SubgoalExecutor
from robot_control.planner import RVGPlanner
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

# Conditional import for GUI
try:
    from robot_control.gui import Window
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False
    Window = None


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

    # Control options
    initial_controller: str = "keyboard"
    initial_speed: float = 0.3
    frequency: float = 30.0

    # Autonomous mode: if planner is set, runs in autonomous mode
    planner: Optional["Planner"] = None
    quit_on_complete: bool = True  # Quit when autonomous plan completes


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
        self._plan_completed = False  # Track if plan has completed (for stop command)

    def run(self) -> None:
        """Start all components and run until quit."""
        self._setup()
        self._start()

        if self._window:
            self._print_banner()
            self._window.run()  # Blocks until window closed
        else:
            # Headless mode - run until stop() called
            self._print_banner()
            while self._running:
                time.sleep(0.1)

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

        # Create camera sensor
        self._sensor = CameraSensorNode(camera_config)

        # Create ArUco observer
        self._observer = ArucoObserver(observer_config)

        # Create camera recorder
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
        observer_config = ObserverConfig(
            calibration_file=camera_cfg.get("calibration_file", ""),
            undistort=camera_cfg.get("undistort", False),
            robot_marker_id=robot_cfg.get("marker_id", 1),
            robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
            marker_to_wheel_offset_cm=marker_offset_tuple,
            object_defs=object_defs,
            object_marker_size_mm=robot_cfg.get("object_marker_size_mm", 30.0),
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

        planner = RVGPlanner(
            workspace_width=self._workspace_config.width,
            workspace_height=self._workspace_config.height,
            robot_width=self._workspace_config.car_width,
            robot_height=self._workspace_config.car_height,
            robot_geometry_scale=controller_configs.navigation.robot_geometry_scale,
        )

        # Create keyboard controller
        keyboard = KeyboardController(max_speed=self._config.initial_speed)

        # Create navigation controller (used by push controller for approach phase)
        navigation = NavigationController(
            self._workspace_config,
            planner,
            nav_config=controller_configs.navigation,
            max_speed=self._config.initial_speed,
        )

        # Create follow path controller
        follow_path = FollowPathController(
            self._workspace_config, max_speed=self._config.initial_speed
        )

        # Create push controller with navigation controller for approach phase
        push = PushController(
            self._workspace_config,
            nav_controller=navigation,
            push_config=controller_configs.push,
            max_speed=self._config.initial_speed,
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

        self._window = Window(
            self._workspace_config,
            show_camera=(is_real and self._config.show_camera),
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
        # Start camera sensor
        print("[Runtime] Starting CameraSensorNode...")
        if not self._sensor.start():
            raise RuntimeError("Failed to start camera")

        # Subscribe recorder to camera frames
        self._recorder.subscribe()

        # Start ArUco observer
        print("[Runtime] Starting ArucoObserver...")
        if not self._observer.start():
            self._recorder.unsubscribe()
            self._sensor.stop()
            raise RuntimeError("Failed to start ArUco observer")

        # Start real environment (serial connection)
        if not self._config.dry_run:
            if not self._env.start():
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
        if self._observer:
            self._observer.stop()
        if self._sensor:
            self._sensor.stop()

    # --- Control loop ---

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

                    # Handle plan completion
                    if status == "Plan Complete":
                        # Send immediate stop when plan first completes
                        if not self._plan_completed:
                            self._plan_completed = True
                            print("[Runtime] Plan complete - stopping robot")
                            if is_real and not self._config.dry_run:
                                if hasattr(self._env, "stop_robot"):
                                    self._env.stop_robot()

                        # Quit if configured to do so
                        if self._config.quit_on_complete:
                            if self._window:
                                self._window.update(obs)
                                self._window.set_status(status)
                                self._window.update_drawings(drawings)
                            print("[Runtime] Shutting down")
                            self._running = False
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
                self._planner.notify_subgoal_done(obs)

            # Check if plan is complete
            if self._planner.is_complete(obs):
                return Action.stop(), self._planner.get_drawings(), "Plan Complete"

            # Get next subgoal
            subgoal = self._planner.plan(obs)
            if subgoal:
                self._executor.set_subgoal(subgoal, obs)
            else:
                # No more subgoals
                return Action.stop(), self._planner.get_drawings(), "Plan Complete"

        # Execute
        action = self._executor.step(obs)

        # Combine drawings: plan overview + execution details
        drawings = self._planner.get_drawings() + self._executor.get_drawings()

        # Status
        status = f"Autonomous: {self._executor.get_status()}"

        return action, drawings, status

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

        print(f"Speed: {self._coordinator.current_speed}")
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
