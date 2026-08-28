"""Regression coverage for closing the GUI from the runtime worker."""

from __future__ import annotations

import os
import threading

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QTimer

from robot_control.core.types import WorkspaceConfig
from robot_control.gui.window import Window


def _workspace_config() -> WorkspaceConfig:
    return WorkspaceConfig(
        width=70.0,
        height=55.0,
        car_width=7.0,
        car_height=7.0,
        offset_w=0.0,
        offset_h=0.0,
    )


def test_plain_python_worker_can_close_window_and_end_event_loop():
    """The runtime control loop uses threading.Thread, not QThread."""
    window = Window(_workspace_config())
    close_called = threading.Event()

    def close_from_worker() -> None:
        window.close_window()
        close_called.set()

    worker = threading.Thread(target=close_from_worker)
    safety_timer = QTimer()
    safety_timer.setSingleShot(True)
    safety_timer.timeout.connect(lambda: window._app.exit(73))
    safety_timer.start(1000)
    QTimer.singleShot(0, worker.start)

    try:
        exit_code = window.run()
    finally:
        safety_timer.stop()
        worker.join(timeout=1.0)
        window.close()

    assert close_called.is_set()
    assert not worker.is_alive()
    assert exit_code == 0, "GUI-thread safety timer had to stop the event loop"
