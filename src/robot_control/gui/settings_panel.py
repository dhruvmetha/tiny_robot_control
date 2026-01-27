"""Right sidebar settings panel for robot_control GUI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


@dataclass
class ControllerSettings:
    """Current controller settings for display."""

    max_speed: float = 0.3
    controller_type: str = "None"
    # Add more settings as needed


class ControllerSelector(QGroupBox):
    """Panel for selecting active controller mode."""

    # Signal emitted when controller selection changes
    controller_changed = Signal(str)

    CONTROLLERS = [
        ("keyboard", "Keyboard (WASD)"),
        ("navigation", "Navigation (Click)"),
        ("follow_path", "Follow Path"),
    ]

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Controller", parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(4)

        self._button_group = QButtonGroup(self)
        self._buttons: dict[str, QRadioButton] = {}

        for controller_id, label in self.CONTROLLERS:
            btn = QRadioButton(label)
            btn.setStyleSheet("color: #333;")
            self._buttons[controller_id] = btn
            self._button_group.addButton(btn)
            layout.addWidget(btn)

        # Default to keyboard
        self._buttons["keyboard"].setChecked(True)

        # Connect signal
        self._button_group.buttonClicked.connect(self._on_button_clicked)

    def _on_button_clicked(self, button: QRadioButton) -> None:
        """Handle button click."""
        for controller_id, btn in self._buttons.items():
            if btn == button:
                self.controller_changed.emit(controller_id)
                break

    def get_selected(self) -> str:
        """Get the currently selected controller ID."""
        for controller_id, btn in self._buttons.items():
            if btn.isChecked():
                return controller_id
        return "keyboard"

    def set_selected(self, controller_id: str) -> None:
        """Set the selected controller."""
        if controller_id in self._buttons:
            self._buttons[controller_id].setChecked(True)


class NavigationGoalPanel(QGroupBox):
    """Panel for setting navigation goal (x, y, theta)."""

    # Signal emitted when Go button is clicked: (x, y, theta or None)
    goal_submitted = Signal(float, float, object)  # object for Optional[float]
    # Signal emitted when Rotate button is clicked: (theta)
    rotate_submitted = Signal(float)

    def __init__(self, workspace_width: float = 640.0, workspace_height: float = 480.0,
                 parent: Optional[QWidget] = None):
        super().__init__("Navigation Goal", parent)
        self._workspace_width = workspace_width
        self._workspace_height = workspace_height

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(6)

        # X input
        x_row = QHBoxLayout()
        x_row.setSpacing(4)
        x_label = QLabel("X:")
        x_label.setFixedWidth(20)
        x_label.setStyleSheet("color: #333;")
        self._x_spin = QDoubleSpinBox()
        self._x_spin.setRange(0, workspace_width)
        self._x_spin.setValue(workspace_width / 2)
        self._x_spin.setDecimals(1)
        self._x_spin.setSuffix(" cm")
        self._x_spin.setStyleSheet("background-color: white; color: #333;")
        x_row.addWidget(x_label)
        x_row.addWidget(self._x_spin, 1)
        layout.addLayout(x_row)

        # Y input
        y_row = QHBoxLayout()
        y_row.setSpacing(4)
        y_label = QLabel("Y:")
        y_label.setFixedWidth(20)
        y_label.setStyleSheet("color: #333;")
        self._y_spin = QDoubleSpinBox()
        self._y_spin.setRange(0, workspace_height)
        self._y_spin.setValue(workspace_height / 2)
        self._y_spin.setDecimals(1)
        self._y_spin.setSuffix(" cm")
        self._y_spin.setStyleSheet("background-color: white; color: #333;")
        y_row.addWidget(y_label)
        y_row.addWidget(self._y_spin, 1)
        layout.addLayout(y_row)

        # Theta input with checkbox
        theta_row = QHBoxLayout()
        theta_row.setSpacing(4)
        self._theta_check = QCheckBox("θ:")
        self._theta_check.setFixedWidth(35)
        self._theta_check.setStyleSheet("color: #333;")
        self._theta_check.setChecked(False)
        self._theta_check.toggled.connect(self._on_theta_toggled)
        self._theta_spin = QDoubleSpinBox()
        self._theta_spin.setRange(0, 360)
        self._theta_spin.setValue(0)
        self._theta_spin.setDecimals(1)
        self._theta_spin.setSuffix("°")
        self._theta_spin.setEnabled(False)
        self._theta_spin.setStyleSheet("background-color: #ddd; color: #999;")
        theta_row.addWidget(self._theta_check)
        theta_row.addWidget(self._theta_spin, 1)
        layout.addLayout(theta_row)

        # Button row: Go and Rotate buttons
        button_row = QHBoxLayout()
        button_row.setSpacing(4)

        # Go button
        self._go_btn = QPushButton("Go")
        self._go_btn.setStyleSheet("""
            QPushButton {
                background-color: #0066cc;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #0077ee;
            }
            QPushButton:pressed {
                background-color: #0055aa;
            }
        """)
        self._go_btn.clicked.connect(self._on_go_clicked)
        button_row.addWidget(self._go_btn, 1)

        # Rotate button
        self._rotate_btn = QPushButton("Rotate")
        self._rotate_btn.setStyleSheet("""
            QPushButton {
                background-color: #666666;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #777777;
            }
            QPushButton:pressed {
                background-color: #555555;
            }
        """)
        self._rotate_btn.clicked.connect(self._on_rotate_clicked)
        button_row.addWidget(self._rotate_btn, 1)

        layout.addLayout(button_row)

    def _on_theta_toggled(self, checked: bool) -> None:
        """Handle theta checkbox toggle."""
        self._theta_spin.setEnabled(checked)
        if checked:
            self._theta_spin.setStyleSheet("background-color: white; color: #333;")
        else:
            self._theta_spin.setStyleSheet("background-color: #ddd; color: #999;")

    def _on_go_clicked(self) -> None:
        """Handle Go button click."""
        x = self._x_spin.value()
        y = self._y_spin.value()
        theta = self._theta_spin.value() if self._theta_check.isChecked() else None
        self.goal_submitted.emit(x, y, theta)

    def _on_rotate_clicked(self) -> None:
        """Handle Rotate button click."""
        theta = self._theta_spin.value()
        self.rotate_submitted.emit(theta)

    def set_workspace_size(self, width: float, height: float) -> None:
        """Update workspace size limits."""
        self._workspace_width = width
        self._workspace_height = height
        self._x_spin.setRange(0, width)
        self._y_spin.setRange(0, height)

    def get_goal(self) -> tuple:
        """Get current goal values."""
        x = self._x_spin.value()
        y = self._y_spin.value()
        theta = self._theta_spin.value() if self._theta_check.isChecked() else None
        return (x, y, theta)


class SpeedPanel(QGroupBox):
    """Panel displaying speed settings with +/- controls."""

    # Signal emitted when speed change is requested: (direction: int) -1 or +1
    speed_change_requested = Signal(int)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Speed", parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(8)

        # Speed control row: [-] [bar] [+]
        control_row = QHBoxLayout()
        control_row.setSpacing(4)

        # Minus button
        self._minus_btn = QPushButton("-")
        self._minus_btn.setFixedSize(28, 28)
        self._minus_btn.setStyleSheet("""
            QPushButton {
                background-color: #444;
                color: white;
                border: 1px solid #666;
                border-radius: 4px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #555;
            }
            QPushButton:pressed {
                background-color: #333;
            }
        """)
        self._minus_btn.clicked.connect(lambda: self.speed_change_requested.emit(-1))
        control_row.addWidget(self._minus_btn)

        # Speed bar
        self._speed_bar = QProgressBar()
        self._speed_bar.setRange(0, 100)
        self._speed_bar.setValue(30)
        self._speed_bar.setTextVisible(True)
        self._speed_bar.setFormat("%v%")
        self._speed_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #666;
                border-radius: 3px;
                text-align: center;
                background-color: #333;
                color: white;
            }
            QProgressBar::chunk {
                background-color: #0066cc;
            }
        """)
        control_row.addWidget(self._speed_bar, 1)  # Stretch factor 1

        # Plus button
        self._plus_btn = QPushButton("+")
        self._plus_btn.setFixedSize(28, 28)
        self._plus_btn.setStyleSheet("""
            QPushButton {
                background-color: #444;
                color: white;
                border: 1px solid #666;
                border-radius: 4px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #555;
            }
            QPushButton:pressed {
                background-color: #333;
            }
        """)
        self._plus_btn.clicked.connect(lambda: self.speed_change_requested.emit(+1))
        control_row.addWidget(self._plus_btn)

        layout.addLayout(control_row)

        # Speed value label
        form = QFormLayout()
        form.setSpacing(4)

        self._speed_label = QLabel("0.30")
        self._speed_label.setStyleSheet("color: #0066cc; font-weight: bold; font-size: 14px;")
        form.addRow("Max Speed:", self._speed_label)

        self._percent_label = QLabel("30%")
        self._percent_label.setStyleSheet("color: #666;")
        form.addRow("Motor Power:", self._percent_label)

        layout.addLayout(form)

    def set_speed(self, speed: float) -> None:
        """Set the displayed speed value."""
        speed = max(0.0, min(1.0, speed))
        percent = int(speed * 100)

        self._speed_bar.setValue(percent)
        self._speed_label.setText(f"{speed:.2f}")
        self._percent_label.setText(f"{percent}%")

        # Color based on speed level
        if speed <= 0.3:
            color = "#009900"  # Green - safe
        elif speed <= 0.5:
            color = "#0066cc"  # Blue - moderate
        elif speed <= 0.7:
            color = "#cc6600"  # Orange - fast
        else:
            color = "#cc0000"  # Red - max

        self._speed_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #666;
                border-radius: 3px;
                text-align: center;
                background-color: #333;
                color: white;
            }}
            QProgressBar::chunk {{
                background-color: {color};
            }}
        """)


class ParametersPanel(QGroupBox):
    """Panel displaying controller parameters."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Parameters", parent)

        layout = QFormLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(4)

        # Lookahead distance
        self._lookahead_label = QLabel("-")
        self._lookahead_label.setStyleSheet("color: #666;")
        layout.addRow("Lookahead:", self._lookahead_label)

        # Arrival threshold
        self._arrival_label = QLabel("-")
        self._arrival_label.setStyleSheet("color: #666;")
        layout.addRow("Arrival:", self._arrival_label)

        # Rotation tolerance
        self._rotation_label = QLabel("-")
        self._rotation_label.setStyleSheet("color: #666;")
        layout.addRow("Rot. Tol:", self._rotation_label)

    def set_lookahead(self, value: float) -> None:
        """Set lookahead distance."""
        self._lookahead_label.setText(f"{value:.1f} cm")

    def set_arrival_threshold(self, value: float) -> None:
        """Set arrival threshold."""
        self._arrival_label.setText(f"{value:.1f} cm")

    def set_rotation_tolerance(self, value: float) -> None:
        """Set rotation tolerance."""
        self._rotation_label.setText(f"{value:.1f} deg")

    def clear(self) -> None:
        """Clear all values."""
        self._lookahead_label.setText("-")
        self._arrival_label.setText("-")
        self._rotation_label.setText("-")


class ActionPanel(QGroupBox):
    """Panel displaying current action/motor commands."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Action", parent)

        layout = QFormLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(4)

        # Left wheel
        self._left_label = QLabel("0.00")
        self._left_label.setStyleSheet("color: #0066cc; font-family: monospace;")
        layout.addRow("Left:", self._left_label)

        # Right wheel
        self._right_label = QLabel("0.00")
        self._right_label.setStyleSheet("color: #0066cc; font-family: monospace;")
        layout.addRow("Right:", self._right_label)

    def set_action(self, left: float, right: float) -> None:
        """Set the displayed action values."""
        self._left_label.setText(f"{left:+.2f}")
        self._right_label.setText(f"{right:+.2f}")

        # Color based on direction
        left_color = "#009900" if left >= 0 else "#cc0000"
        right_color = "#009900" if right >= 0 else "#cc0000"
        self._left_label.setStyleSheet(f"color: {left_color}; font-family: monospace;")
        self._right_label.setStyleSheet(f"color: {right_color}; font-family: monospace;")

    def clear(self) -> None:
        """Clear action display."""
        self._left_label.setText("0.00")
        self._right_label.setText("0.00")
        self._left_label.setStyleSheet("color: #666; font-family: monospace;")
        self._right_label.setStyleSheet("color: #666; font-family: monospace;")


class SettingsPanel(QWidget):
    """
    Right sidebar widget displaying controller settings and parameters.

    Layout:
    - Controller selector (top)
    - Navigation goal panel (shown when navigation selected)
    - Speed panel
    - Parameters panel
    - Action panel (bottom)
    """

    # Signal emitted when controller selection changes
    controller_changed = Signal(str)
    # Signal emitted when speed change is requested: (direction: int) -1 or +1
    speed_change_requested = Signal(int)
    # Signal emitted when navigation goal is submitted: (x, y, theta or None)
    navigation_goal_submitted = Signal(float, float, object)
    # Signal emitted when rotation goal is submitted: (theta)
    rotation_goal_submitted = Signal(float)
    # Signal emitted when cancel is requested
    cancel_requested = Signal()

    MIN_WIDTH = 150
    MAX_WIDTH = 250

    def __init__(self, show_controller_selector: bool = True,
                 workspace_width: float = 640.0, workspace_height: float = 480.0,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumWidth(self.MIN_WIDTH)
        self.setMaximumWidth(self.MAX_WIDTH)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        # Controller selector (optional)
        self._controller_selector: Optional[ControllerSelector] = None
        if show_controller_selector:
            self._controller_selector = ControllerSelector()
            self._controller_selector.controller_changed.connect(self._on_controller_changed)
            layout.addWidget(self._controller_selector)

        # Navigation goal panel (hidden by default)
        self._nav_goal_panel = NavigationGoalPanel(workspace_width, workspace_height)
        self._nav_goal_panel.goal_submitted.connect(self._on_nav_goal_submitted)
        self._nav_goal_panel.rotate_submitted.connect(self._on_rotate_submitted)
        self._nav_goal_panel.setVisible(False)
        layout.addWidget(self._nav_goal_panel)

        # Speed panel
        self._speed_panel = SpeedPanel()
        self._speed_panel.speed_change_requested.connect(self._on_speed_change)
        layout.addWidget(self._speed_panel)

        # Parameters panel
        self._params_panel = ParametersPanel()
        layout.addWidget(self._params_panel)

        # Action panel
        self._action_panel = ActionPanel()
        layout.addWidget(self._action_panel)

        # Cancel button (always visible)
        self._cancel_btn = QPushButton("Cancel")
        self._cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #cc3333;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #dd4444;
            }
            QPushButton:pressed {
                background-color: #aa2222;
            }
        """)
        self._cancel_btn.clicked.connect(self._on_cancel_clicked)
        layout.addWidget(self._cancel_btn)

        # Flexible space
        layout.addStretch()

    def _on_controller_changed(self, controller_id: str) -> None:
        """Forward controller change signal and update UI."""
        # Show/hide navigation goal panel
        self._nav_goal_panel.setVisible(controller_id == "navigation")
        self.controller_changed.emit(controller_id)

    def _on_speed_change(self, direction: int) -> None:
        """Forward speed change signal."""
        self.speed_change_requested.emit(direction)

    def _on_nav_goal_submitted(self, x: float, y: float, theta: object) -> None:
        """Forward navigation goal signal."""
        self.navigation_goal_submitted.emit(x, y, theta)

    def _on_rotate_submitted(self, theta: float) -> None:
        """Forward rotation goal signal."""
        self.rotation_goal_submitted.emit(theta)

    def _on_cancel_clicked(self) -> None:
        """Handle cancel button click."""
        self.cancel_requested.emit()

    @property
    def controller_selector(self) -> Optional[ControllerSelector]:
        """Access the controller selector."""
        return self._controller_selector

    @property
    def nav_goal_panel(self) -> NavigationGoalPanel:
        """Access the navigation goal panel."""
        return self._nav_goal_panel

    @property
    def speed_panel(self) -> SpeedPanel:
        """Access the speed panel."""
        return self._speed_panel

    @property
    def params_panel(self) -> ParametersPanel:
        """Access the parameters panel."""
        return self._params_panel

    @property
    def action_panel(self) -> ActionPanel:
        """Access the action panel."""
        return self._action_panel

    def get_selected_controller(self) -> str:
        """Get the currently selected controller ID."""
        if self._controller_selector is not None:
            return self._controller_selector.get_selected()
        return "keyboard"

    def set_selected_controller(self, controller_id: str) -> None:
        """Set the selected controller."""
        if self._controller_selector is not None:
            self._controller_selector.set_selected(controller_id)

    def set_speed(self, speed: float) -> None:
        """Set the displayed speed."""
        self._speed_panel.set_speed(speed)

    def set_action(self, left: float, right: float) -> None:
        """Set the displayed action."""
        self._action_panel.set_action(left, right)

    def clear(self) -> None:
        """Clear all panels."""
        self._speed_panel.set_speed(0.0)
        self._params_panel.clear()
        self._action_panel.clear()
