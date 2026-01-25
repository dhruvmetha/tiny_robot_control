"""Main window for robot_control GUI."""

from __future__ import annotations

from typing import Callable, Dict, Optional

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QMainWindow,
    QSplitter,
    QWidget,
)

from robot_control.core.types import Action, Observation, WorkspaceConfig
from robot_control.gui.canvas import Canvas
from robot_control.gui.settings_panel import SettingsPanel
from robot_control.gui.sidebar import Sidebar


class UpdateEmitter(QObject):
    """Thread-safe signal emitter for GUI updates."""
    update_requested = Signal(object)  # Observation
    drawings_requested = Signal(object)  # List of drawings
    status_requested = Signal(str)  # Status string
    camera_requested = Signal(object)  # Camera frame (numpy array)
    connection_requested = Signal(object)  # SenderStatus
    speed_requested = Signal(float)  # Speed value
    action_requested = Signal(float, float)  # Left, right wheel speeds
    recording_requested = Signal(bool)  # Set recording state


class KeyboardEmitter(QObject):
    """Signal emitter for keyboard events."""
    key_pressed = Signal(str)
    key_released = Signal(str)


class Window(QMainWindow):
    """
    Main GUI window for robot visualization.

    Layout matches micromvp:
    - Left: Sidebar with robot inspector and keyboard help
    - Right: Canvas for workspace visualization

    Usage:
        window = Window(config)
        window.register_callback("on_key_press", handle_key)

        # From any thread:
        window.update(observation)

        window.run()  # Blocks
    """

    def __init__(
        self,
        config: WorkspaceConfig,
        show_camera: bool = False,
        show_connection: bool = False,
        show_settings: bool = False,
        target_robot_id: Optional[int] = None,
        parent: Optional[QWidget] = None,
    ):
        # Ensure QApplication exists
        self._app = QApplication.instance()
        if self._app is None:
            self._app = QApplication([])

        super().__init__(parent)
        self._config = config
        self._show_camera = show_camera
        self._show_connection = show_connection
        self._show_settings = show_settings
        self._target_robot_id = target_robot_id
        self._callbacks: Dict[str, Callable] = {}

        # Thread-safe update mechanism
        self._update_emitter = UpdateEmitter()
        self._update_emitter.update_requested.connect(self._handle_update)
        self._update_emitter.drawings_requested.connect(self._handle_drawings)
        self._update_emitter.status_requested.connect(self._handle_status)
        self._update_emitter.camera_requested.connect(self._handle_camera)
        self._update_emitter.connection_requested.connect(self._handle_connection)
        self._update_emitter.speed_requested.connect(self._handle_speed)
        self._update_emitter.action_requested.connect(self._handle_action)
        self._update_emitter.recording_requested.connect(self._handle_recording)

        # Keyboard handling
        self._keyboard_emitter = KeyboardEmitter()
        self._keyboard_emitter.key_pressed.connect(self._on_key_press)
        self._keyboard_emitter.key_released.connect(self._on_key_release)

        self._setup_ui()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def _setup_ui(self) -> None:
        self.setWindowTitle("Robot Control")
        self.setMinimumSize(800, 600)
        self.resize(1200 if self._show_settings else 1000, 700)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # Left: Sidebar
        self._sidebar = Sidebar(
            show_camera=self._show_camera,
            show_connection=self._show_connection,
            target_robot_id=self._target_robot_id,
        )
        self._sidebar.record_toggled.connect(self._on_record_toggled)
        splitter.addWidget(self._sidebar)

        # Vertical separator (left)
        separator_left = QFrame()
        separator_left.setFrameShape(QFrame.Shape.VLine)
        separator_left.setFrameShadow(QFrame.Shadow.Sunken)
        splitter.addWidget(separator_left)

        # Center: Canvas
        self._canvas = Canvas(self._config)
        splitter.addWidget(self._canvas)

        # Right: Settings panel (optional)
        self._settings_panel: Optional[SettingsPanel] = None
        if self._show_settings:
            # Vertical separator (right)
            separator_right = QFrame()
            separator_right.setFrameShape(QFrame.Shape.VLine)
            separator_right.setFrameShadow(QFrame.Shadow.Sunken)
            splitter.addWidget(separator_right)

            # Settings panel
            self._settings_panel = SettingsPanel(
                workspace_width=self._config.width,
                workspace_height=self._config.height,
            )
            self._settings_panel.controller_changed.connect(self._on_controller_changed)
            self._settings_panel.speed_change_requested.connect(self._on_speed_change)
            self._settings_panel.navigation_goal_submitted.connect(self._on_navigation_goal)
            self._settings_panel.cancel_requested.connect(self._on_cancel)
            splitter.addWidget(self._settings_panel)

        # Configure splitter
        if self._show_settings:
            splitter.setStretchFactor(0, 0)  # Sidebar: fixed
            splitter.setStretchFactor(1, 0)  # Separator left: fixed
            splitter.setStretchFactor(2, 1)  # Canvas: expandable
            splitter.setStretchFactor(3, 0)  # Separator right: fixed
            splitter.setStretchFactor(4, 0)  # Settings: fixed
            splitter.setCollapsible(0, False)
            splitter.setCollapsible(1, False)
            splitter.setCollapsible(2, False)
            splitter.setCollapsible(3, False)
            splitter.setCollapsible(4, False)
            splitter.setSizes([250, 2, 700, 2, 180])
        else:
            splitter.setStretchFactor(0, 0)  # Sidebar: fixed
            splitter.setStretchFactor(1, 0)  # Separator: fixed
            splitter.setStretchFactor(2, 1)  # Canvas: expandable
            splitter.setCollapsible(0, False)
            splitter.setCollapsible(1, False)
            splitter.setCollapsible(2, False)
            splitter.setSizes([250, 2, 750])

        # Connect canvas signals
        self._canvas.canvas_clicked.connect(self._on_canvas_click)
        self._canvas.curve_drawn.connect(self._on_curve_drawn)

    def register_callback(self, name: str, func: Callable) -> None:
        """
        Register callback for events.

        Supported events:
            - "on_key_press": func(key: str)
            - "on_key_release": func(key: str)
            - "on_canvas_click": func(x: float, y: float)
            - "on_curve_drawn": func(points: List[Tuple[float, float]])
            - "on_controller_changed": func(controller_id: str)
            - "on_speed_change": func(direction: int)  # -1 for decrease, +1 for increase
            - "on_navigation_goal": func(x: float, y: float, theta: Optional[float])
            - "on_cancel": func()  # Cancel current action
            - "on_record_toggled": func(recording: bool)  # Recording started/stopped
        """
        self._callbacks[name] = func

    def _on_controller_changed(self, controller_id: str) -> None:
        cb = self._callbacks.get("on_controller_changed")
        if cb:
            cb(controller_id)

    def _on_speed_change(self, direction: int) -> None:
        cb = self._callbacks.get("on_speed_change")
        if cb:
            cb(direction)

    def _on_navigation_goal(self, x: float, y: float, theta: object) -> None:
        cb = self._callbacks.get("on_navigation_goal")
        if cb:
            cb(x, y, theta)

    def _on_cancel(self) -> None:
        cb = self._callbacks.get("on_cancel")
        if cb:
            cb()

    def _on_record_toggled(self, recording: bool) -> None:
        cb = self._callbacks.get("on_record_toggled")
        if cb:
            cb(recording)

    def enable_canvas_click(self, enabled: bool) -> None:
        """Enable/disable canvas click events."""
        self._canvas.enable_click(enabled)

    def enable_draw(self, enabled: bool) -> None:
        """Enable/disable curve drawing on canvas."""
        self._canvas.enable_draw(enabled)

    # -------------------------------------------------------------------------
    # Update API (thread-safe)
    # -------------------------------------------------------------------------

    def update(self, obs: Observation) -> None:
        """Update GUI with new observation. Thread-safe."""
        self._update_emitter.update_requested.emit(obs)

    def _handle_update(self, obs: Observation) -> None:
        """Handle update on main thread."""
        self._canvas.update_robot(obs)
        self._sidebar.update_robot(obs)

    def _handle_drawings(self, drawings) -> None:
        """Handle drawings update on main thread."""
        self._canvas.update_drawings(drawings)

    def _handle_status(self, status: str) -> None:
        """Handle status update on main thread."""
        self._sidebar.set_status(status)

    def _handle_camera(self, frame) -> None:
        """Handle camera frame update on main thread."""
        self._sidebar.update_camera(frame)

    def _handle_connection(self, status) -> None:
        """Handle connection status update on main thread."""
        self._sidebar.update_connection(status)

    def _handle_speed(self, speed: float) -> None:
        """Handle speed update on main thread."""
        if self._settings_panel is not None:
            self._settings_panel.set_speed(speed)

    def _handle_action(self, left: float, right: float) -> None:
        """Handle action update on main thread."""
        if self._settings_panel is not None:
            self._settings_panel.set_action(left, right)

    def _handle_recording(self, recording: bool) -> None:
        """Handle recording state update on main thread."""
        self._sidebar.set_recording(recording)

    def set_controller(self, name: str) -> None:
        """Set the controller name displayed in sidebar."""
        self._sidebar.set_controller(name)

    def set_status(self, status: str) -> None:
        """Set the status displayed in sidebar. Thread-safe."""
        self._update_emitter.status_requested.emit(status)

    def update_drawings(self, drawings) -> None:
        """Update canvas drawings (paths, targets, etc.). Thread-safe."""
        self._update_emitter.drawings_requested.emit(drawings)

    def update_camera(self, frame) -> None:
        """Update camera view with a new frame. Thread-safe."""
        self._update_emitter.camera_requested.emit(frame)

    def update_connection(self, status) -> None:
        """Update connection status display. Thread-safe."""
        self._update_emitter.connection_requested.emit(status)

    def update_speed(self, speed: float) -> None:
        """Update speed display in settings panel. Thread-safe."""
        self._update_emitter.speed_requested.emit(speed)

    def update_action(self, action: Action) -> None:
        """Update action display in settings panel. Thread-safe."""
        self._update_emitter.action_requested.emit(action.left_speed, action.right_speed)

    def set_recording(self, recording: bool) -> None:
        """Set recording state (updates UI button). Thread-safe."""
        self._update_emitter.recording_requested.emit(recording)

    @property
    def is_recording(self) -> bool:
        """Check if recording is active."""
        return self._sidebar.is_recording

    # -------------------------------------------------------------------------
    # Keyboard handling
    # -------------------------------------------------------------------------

    def keyPressEvent(self, event) -> None:
        key = self._key_to_string(event)
        if key and not event.isAutoRepeat():
            self._keyboard_emitter.key_pressed.emit(key)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        key = self._key_to_string(event)
        if key and not event.isAutoRepeat():
            self._keyboard_emitter.key_released.emit(key)
        super().keyReleaseEvent(event)

    def _key_to_string(self, event) -> Optional[str]:
        """Convert Qt key event to string."""
        key = event.key()

        # Letters
        if Qt.Key.Key_A <= key <= Qt.Key.Key_Z:
            return chr(key).lower()

        # Numbers
        if Qt.Key.Key_0 <= key <= Qt.Key.Key_9:
            return chr(key)

        # Special keys
        special = {
            Qt.Key.Key_Space: "space",
            Qt.Key.Key_Tab: "tab",
            Qt.Key.Key_Return: "return",
            Qt.Key.Key_Escape: "escape",
            Qt.Key.Key_Up: "up",
            Qt.Key.Key_Down: "down",
            Qt.Key.Key_Left: "left",
            Qt.Key.Key_Right: "right",
        }
        return special.get(key)

    def _on_key_press(self, key: str) -> None:
        cb = self._callbacks.get("on_key_press")
        if cb:
            cb(key)

    def _on_key_release(self, key: str) -> None:
        cb = self._callbacks.get("on_key_release")
        if cb:
            cb(key)

    def _on_canvas_click(self, x: float, y: float) -> None:
        cb = self._callbacks.get("on_canvas_click")
        if cb:
            cb(x, y)

    def _on_curve_drawn(self, points) -> None:
        cb = self._callbacks.get("on_curve_drawn")
        if cb:
            cb(points)

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    def run(self) -> int:
        """Start GUI main loop. Blocks until closed."""
        self.show()
        return self._app.exec()

    def close_window(self) -> None:
        """Programmatically close the window."""
        self.close()
