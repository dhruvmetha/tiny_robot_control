"""Sidebar components for robot_control GUI."""

from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from robot_control.core.types import Observation
from robot_control.environment.action_sender import SenderStatus


class ControllerPanel(QGroupBox):
    """Panel displaying controller information."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Controller", parent)

        layout = QFormLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(4)

        # Controller name
        self._controller_label = QLabel("-")
        self._controller_label.setStyleSheet("color: #009900; font-weight: bold;")
        layout.addRow("Mode:", self._controller_label)

        # Status
        self._status_label = QLabel("-")
        self._status_label.setStyleSheet("color: #cc6600; font-weight: bold;")
        layout.addRow("Status:", self._status_label)

    def set_controller(self, name: str) -> None:
        """Set the controller name."""
        self._controller_label.setText(name)

    def set_status(self, status: str) -> None:
        """Set the current status."""
        self._status_label.setText(status)

    def clear(self) -> None:
        """Clear all fields."""
        self._controller_label.setText("-")
        self._status_label.setText("-")


class ConnectionPanel(QGroupBox):
    """Panel displaying robot connection status."""

    def __init__(self, target_robot_id: Optional[int] = None, parent: Optional[QWidget] = None):
        super().__init__("Connection", parent)
        self._target_robot_id = target_robot_id

        layout = QFormLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(4)

        # AP status (serial port + AP response)
        self._ap_label = QLabel("Not started")
        self._ap_label.setStyleSheet("color: #999999;")
        layout.addRow("AP:", self._ap_label)

        # Target robot status
        self._robot_label = QLabel("-")
        self._robot_label.setStyleSheet("color: #999999;")
        if target_robot_id is not None:
            layout.addRow(f"Robot {target_robot_id}:", self._robot_label)
        else:
            layout.addRow("Robot:", self._robot_label)

        # All alive robots
        self._alive_label = QLabel("-")
        self._alive_label.setStyleSheet("color: #666666; font-size: 10px;")
        layout.addRow("Alive:", self._alive_label)

        # Recent event
        self._event_label = QLabel("-")
        self._event_label.setStyleSheet("color: #666666; font-size: 10px;")
        self._event_label.setWordWrap(True)
        layout.addRow("Event:", self._event_label)

    def set_target_robot_id(self, robot_id: int) -> None:
        """Set the target robot ID to monitor."""
        self._target_robot_id = robot_id

    def update_status(self, status: SenderStatus) -> None:
        """Update display with new connection status."""
        # AP status (like micromvp: "normal", "no response", "disconnected")
        if not status.is_connected:
            ap_str = "Disconnected"
            ap_color = "#cc0000"  # Red
        elif status.last_update_time == 0.0:
            # Never received a STATUS message
            ap_str = "Waiting..."
            ap_color = "#cc6600"  # Orange
        elif time.time() - status.last_update_time > 3.0:
            ap_str = "No response"
            ap_color = "#cc6600"  # Orange
        else:
            ap_str = "Normal"
            ap_color = "#009900"  # Green

        self._ap_label.setText(ap_str)
        self._ap_label.setStyleSheet(f"color: {ap_color}; font-weight: bold;")

        # Target robot status
        if self._target_robot_id is not None:
            if self._target_robot_id in status.alive_robot_ids:
                robot_str = "Online"
                robot_color = "#009900"  # Green
            else:
                robot_str = "Offline"
                robot_color = "#cc0000"  # Red
        else:
            if status.alive_robot_ids:
                robot_str = f"{len(status.alive_robot_ids)} online"
                robot_color = "#009900"
            else:
                robot_str = "None"
                robot_color = "#cc0000"

        self._robot_label.setText(robot_str)
        self._robot_label.setStyleSheet(f"color: {robot_color}; font-weight: bold;")

        # All alive robots
        if status.alive_robot_ids:
            alive_str = ", ".join(str(id) for id in sorted(status.alive_robot_ids))
        else:
            alive_str = "None"
        self._alive_label.setText(alive_str)

        # Most recent event
        if status.recent_events:
            event = status.recent_events[-1]
            if event.startswith("[INFO] "):
                event = event[7:]
            self._event_label.setText(event)
        else:
            self._event_label.setText("-")

    def clear(self) -> None:
        """Clear all fields."""
        self._ap_label.setText("Not started")
        self._ap_label.setStyleSheet("color: #999999;")
        self._robot_label.setText("-")
        self._robot_label.setStyleSheet("color: #999999;")
        self._alive_label.setText("-")
        self._event_label.setText("-")


class RobotInspector(QGroupBox):
    """Panel displaying robot observation/state."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Observation", parent)

        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 12, 8, 8)

        # Scroll area for content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        # Content container
        content = QWidget()
        self._form_layout = QFormLayout(content)
        self._form_layout.setContentsMargins(0, 0, 0, 0)
        self._form_layout.setSpacing(4)

        scroll.setWidget(content)
        main_layout.addWidget(scroll)

        # Field labels
        self._fields: Dict[str, QLabel] = {}
        self._init_fields()

    def _init_fields(self) -> None:
        """Initialize the display fields."""
        field_names = [
            ("x", "X"),
            ("y", "Y"),
            ("theta", "Theta (deg)"),
            ("objects", "Objects"),
        ]

        for field_key, label_text in field_names:
            value_label = QLabel("-")
            value_label.setStyleSheet("color: #0066cc;")
            self._fields[field_key] = value_label
            self._form_layout.addRow(f"{label_text}:", value_label)

    def update_state(self, obs: Observation) -> None:
        """Update display with new observation."""
        self._fields["x"].setText(f"{obs.robot_x:.2f}")
        self._fields["y"].setText(f"{obs.robot_y:.2f}")
        self._fields["theta"].setText(f"{obs.robot_theta:.1f}")
        self._fields["objects"].setText(str(len(obs.objects)))

    def clear(self) -> None:
        """Clear all fields."""
        for label in self._fields.values():
            label.setText("-")


class KeyboardHelp(QGroupBox):
    """Panel showing keyboard controls."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Keyboard Controls", parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)

        controls = [
            ("W", "Forward"),
            ("S", "Backward"),
            ("A", "Spin Left"),
            ("D", "Spin Right"),
            ("W+A", "Turn Left"),
            ("W+D", "Turn Right"),
        ]

        for key, action in controls:
            row = QLabel(f"<b>{key}</b>: {action}")
            row.setStyleSheet("color: #333;")
            layout.addWidget(row)

        layout.addStretch()


class CameraView(QGroupBox):
    """Panel displaying camera feed with record button."""

    # Signal emitted when record button is toggled
    record_toggled = Signal(bool)  # True = start recording, False = stop

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Camera", parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 12, 4, 4)

        # Image label
        self._image_label = QLabel()
        self._image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._image_label.setMinimumSize(160, 120)
        self._image_label.setStyleSheet("background-color: #222; border: 1px solid #444;")
        self._image_label.setText("No camera")
        layout.addWidget(self._image_label)

        # Record button
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 4, 0, 0)

        self._record_button = QPushButton("⏺ Record")
        self._record_button.setCheckable(True)
        self._record_button.setStyleSheet("""
            QPushButton {
                background-color: #444;
                color: white;
                border: 1px solid #666;
                border-radius: 3px;
                padding: 4px 8px;
            }
            QPushButton:checked {
                background-color: #cc0000;
                border-color: #ff0000;
            }
            QPushButton:hover {
                background-color: #555;
            }
            QPushButton:checked:hover {
                background-color: #dd0000;
            }
        """)
        self._record_button.clicked.connect(self._on_record_clicked)
        button_layout.addWidget(self._record_button)

        layout.addLayout(button_layout)

        self._is_recording = False

    def _on_record_clicked(self) -> None:
        """Handle record button click."""
        self._is_recording = self._record_button.isChecked()
        if self._is_recording:
            self._record_button.setText("⏹ Stop")
        else:
            self._record_button.setText("⏺ Record")
        self.record_toggled.emit(self._is_recording)

    def set_recording(self, recording: bool) -> None:
        """Set recording state (for external control, e.g., keyboard shortcut)."""
        if recording != self._is_recording:
            self._is_recording = recording
            self._record_button.setChecked(recording)
            if recording:
                self._record_button.setText("⏹ Stop")
            else:
                self._record_button.setText("⏺ Record")

    @property
    def is_recording(self) -> bool:
        """Check if recording is active."""
        return self._is_recording

    def update_frame(self, frame: np.ndarray) -> None:
        """Update the camera view with a new frame (BGR numpy array)."""
        if frame is None:
            return

        # Convert BGR to RGB
        rgb = frame[..., ::-1].copy()
        h, w, ch = rgb.shape

        # Create QImage
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        # Scale to fit the label while maintaining aspect ratio
        label_size = self._image_label.size()
        pixmap = QPixmap.fromImage(qimg).scaled(
            label_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._image_label.setPixmap(pixmap)

    def clear(self) -> None:
        """Clear the camera view."""
        self._image_label.clear()
        self._image_label.setText("No camera")


class Sidebar(QWidget):
    """
    Sidebar widget containing controller panel, robot inspector, camera view, and keyboard help.

    Layout:
    - Controller panel (top)
    - Connection panel (optional, for real robot)
    - Robot inspector
    - Keyboard help
    - Camera view (bottom)
    """

    # Signal emitted when record button is toggled
    record_toggled = Signal(bool)

    MIN_WIDTH = 200
    MAX_WIDTH = 350

    def __init__(
        self,
        show_camera: bool = False,
        show_connection: bool = False,
        target_robot_id: Optional[int] = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.setMinimumWidth(self.MIN_WIDTH)
        self.setMaximumWidth(self.MAX_WIDTH)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        # Controller panel
        self._controller_panel = ControllerPanel()
        layout.addWidget(self._controller_panel)

        # Connection panel (optional, for real robot)
        self._connection_panel: Optional[ConnectionPanel] = None
        if show_connection:
            self._connection_panel = ConnectionPanel(target_robot_id=target_robot_id)
            layout.addWidget(self._connection_panel)

        # Robot inspector (observation)
        self._robot_inspector = RobotInspector()
        layout.addWidget(self._robot_inspector)

        # Keyboard help
        self._keyboard_help = KeyboardHelp()
        layout.addWidget(self._keyboard_help)

        # Flexible space
        layout.addStretch()

        # Camera view (optional, at bottom)
        self._camera_view: Optional[CameraView] = None
        if show_camera:
            self._camera_view = CameraView()
            self._camera_view.record_toggled.connect(self.record_toggled)
            layout.addWidget(self._camera_view)

    @property
    def controller_panel(self) -> ControllerPanel:
        """Access the controller panel."""
        return self._controller_panel

    @property
    def robot_inspector(self) -> RobotInspector:
        """Access the robot inspector."""
        return self._robot_inspector

    @property
    def camera_view(self) -> Optional[CameraView]:
        """Access the camera view (may be None if not enabled)."""
        return self._camera_view

    @property
    def connection_panel(self) -> Optional[ConnectionPanel]:
        """Access the connection panel (may be None if not enabled)."""
        return self._connection_panel

    def update_robot(self, obs: Observation) -> None:
        """Update the robot inspector with new observation."""
        self._robot_inspector.update_state(obs)

    def update_camera(self, frame: np.ndarray) -> None:
        """Update the camera view with a new frame."""
        if self._camera_view is not None:
            self._camera_view.update_frame(frame)

    def update_connection(self, status: SenderStatus) -> None:
        """Update the connection panel with new status."""
        if self._connection_panel is not None:
            self._connection_panel.update_status(status)

    def set_controller(self, name: str) -> None:
        """Set the controller name."""
        self._controller_panel.set_controller(name)

    def set_status(self, status: str) -> None:
        """Set the current status."""
        self._controller_panel.set_status(status)

    def set_recording(self, recording: bool) -> None:
        """Set recording state (for external control, e.g., keyboard shortcut)."""
        if self._camera_view is not None:
            self._camera_view.set_recording(recording)

    @property
    def is_recording(self) -> bool:
        """Check if camera recording is active."""
        if self._camera_view is not None:
            return self._camera_view.is_recording
        return False

    def clear(self) -> None:
        """Clear all panels."""
        self._controller_panel.clear()
        self._robot_inspector.clear()
        if self._connection_panel is not None:
            self._connection_panel.clear()
        if self._camera_view is not None:
            self._camera_view.clear()
