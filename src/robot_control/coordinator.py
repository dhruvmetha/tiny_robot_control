"""Control coordinator for managing controllers and UI event routing."""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Set, Tuple

from robot_control.controller.base import Controller
from robot_control.core.types import Action, Observation
from robot_control.core.world_state import WorldState
from robot_control.gui import Window

SPEED_PRESETS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 0.9]


class ControlCoordinator:
    """
    Coordinates controllers and routes UI events.

    Manages:
    - Active controller selection
    - Speed changes across all controllers
    - Keyboard state for KeyboardController
    - Navigation/rotation goals for NavigationController
    - Path setting for FollowPathController

    Usage:
        coordinator = ControlCoordinator(controllers, world)
        coordinator.bind_to_window(window)

        while running:
            action = coordinator.step(world.get())
            env.apply(action)
    """

    def __init__(
        self,
        controllers: Dict[str, Controller],
        world: WorldState,
        initial_controller: str = "keyboard",
        initial_speed: float = 0.3,
    ):
        self._controllers = controllers
        self._world = world
        self._active = initial_controller
        self._speed = initial_speed
        self._pressed_keys: Set[str] = set()
        self._window: Optional[Window] = None

        # Event handlers (script subscribes to these)
        self._quit_handler: Optional[Callable[[], None]] = None
        self._emergency_stop_handler: Optional[Callable[[], None]] = None

        # Apply initial speed
        for ctrl in controllers.values():
            if hasattr(ctrl, "set_speed"):
                ctrl.set_speed(initial_speed)

    def bind_to_window(self, window: Window) -> None:
        """Register all standard callbacks with window."""
        self._window = window
        window.register_callback("on_controller_changed", self._on_controller_changed)
        window.register_callback("on_canvas_click", self._on_canvas_click)
        window.register_callback("on_curve_drawn", self._on_curve_drawn)
        window.register_callback("on_navigation_goal", self._on_navigation_goal)
        window.register_callback("on_rotation_goal", self._on_rotation_goal)
        window.register_callback("on_speed_change", self._on_speed_change)
        window.register_callback("on_cancel", self._on_cancel)
        window.register_callback("on_key_press", self._on_key_press)
        window.register_callback("on_key_release", self._on_key_release)

        # Set initial UI state
        window.update_speed(self._speed)

    def step(self, obs: Optional[Observation]) -> Action:
        """Get action from active controller."""
        if obs is None:
            return Action.stop()
        ctrl = self._controllers.get(self._active)
        if ctrl is None:
            return Action.stop()
        return ctrl.step(obs, subgoal=None)

    def get_status(self) -> str:
        """Get status string from active controller."""
        ctrl = self._controllers.get(self._active)
        if ctrl and hasattr(ctrl, "get_status"):
            return ctrl.get_status()
        return self._active.title()

    def get_drawings(self) -> list:
        """Get drawings from active controller."""
        ctrl = self._controllers.get(self._active)
        if ctrl and hasattr(ctrl, "get_drawings"):
            return ctrl.get_drawings()
        return []

    @property
    def active_controller(self) -> str:
        return self._active

    @property
    def current_speed(self) -> float:
        return self._speed

    # --- Event subscriptions (for script) ---

    def on_quit(self, handler: Callable[[], None]) -> None:
        """Subscribe to quit event (escape key)."""
        self._quit_handler = handler

    def on_emergency_stop(self, handler: Callable[[], None]) -> None:
        """Subscribe to emergency stop event (space key)."""
        self._emergency_stop_handler = handler

    # --- Event handlers ---

    def _on_controller_changed(self, controller_id: str) -> None:
        # Cancel all controllers
        for ctrl in self._controllers.values():
            if hasattr(ctrl, "cancel"):
                ctrl.cancel()

        if self._window:
            self._window.update_drawings([])

        self._active = controller_id

        # Update window mode
        if self._window:
            if controller_id == "keyboard":
                self._window.set_controller("Keyboard")
                self._window.enable_canvas_click(False)
                self._window.enable_draw(False)
            elif controller_id == "navigation":
                self._window.set_controller("Navigation")
                self._window.enable_canvas_click(True)
                self._window.enable_draw(False)
            elif controller_id == "follow_path":
                self._window.set_controller("Follow Path")
                self._window.enable_canvas_click(False)
                self._window.enable_draw(True)

    def _on_canvas_click(self, x: float, y: float) -> None:
        if self._active != "navigation":
            return
        self._navigate_to(x, y, theta=None)

    def _on_navigation_goal(self, x: float, y: float, theta: Optional[float]) -> None:
        # Switch to navigation if needed
        if self._active != "navigation":
            self._on_controller_changed("navigation")
        self._navigate_to(x, y, theta)

    def _on_rotation_goal(self, theta: float) -> None:
        nav = self._controllers.get("navigation")
        if nav and hasattr(nav, "rotate_to"):
            nav.rotate_to(theta)

    def _navigate_to(self, x: float, y: float, theta: Optional[float]) -> None:
        nav = self._controllers.get("navigation")
        if not nav or not hasattr(nav, "navigate_to"):
            return

        obs = self._world.get()
        if obs is None:
            return

        # Build obstacle list
        obstacles = [
            (o.x, o.y, o.theta, o.width, o.depth)
            for o in obs.objects.values()
            if o.width > 0 and o.depth > 0
        ]

        nav.navigate_to(x, y, theta, (obs.robot_x, obs.robot_y), obstacles)

    def _on_curve_drawn(self, points: List[Tuple[float, float]]) -> None:
        if self._active != "follow_path":
            return
        fp = self._controllers.get("follow_path")
        if fp and hasattr(fp, "set_path") and len(points) >= 2:
            fp.set_path(points)

    def _on_speed_change(self, direction: int) -> None:
        # Find next speed preset
        if direction > 0:
            for preset in SPEED_PRESETS:
                if preset > self._speed:
                    self._speed = preset
                    break
        else:
            for preset in reversed(SPEED_PRESETS):
                if preset < self._speed:
                    self._speed = preset
                    break

        # Apply to all controllers
        for ctrl in self._controllers.values():
            if hasattr(ctrl, "set_speed"):
                ctrl.set_speed(self._speed)

        if self._window:
            self._window.update_speed(self._speed)

    def _on_cancel(self) -> None:
        ctrl = self._controllers.get(self._active)
        if ctrl and hasattr(ctrl, "cancel"):
            ctrl.cancel()
        if self._window:
            self._window.update_drawings([])

    def _on_key_press(self, key: str) -> None:
        key = key.lower()

        # Handle special keys
        if key == "escape":
            if self._quit_handler:
                self._quit_handler()
            return

        if key == "space":
            # Emergency stop - cancel all controllers
            for ctrl in self._controllers.values():
                if hasattr(ctrl, "cancel"):
                    ctrl.cancel()
            if self._emergency_stop_handler:
                self._emergency_stop_handler()
            return

        # Speed adjustment
        if key in ("=", "+"):
            self._on_speed_change(+1)
            return
        if key in ("-", "_"):
            self._on_speed_change(-1)
            return

        # Regular key - add to pressed set
        self._pressed_keys.add(key)

        if self._active == "keyboard":
            kb = self._controllers.get("keyboard")
            if kb and hasattr(kb, "set_keys"):
                kb.set_keys(self._pressed_keys)

    def _on_key_release(self, key: str) -> None:
        key = key.lower()
        self._pressed_keys.discard(key)

        if self._active == "keyboard":
            kb = self._controllers.get("keyboard")
            if kb and hasattr(kb, "set_keys"):
                kb.set_keys(self._pressed_keys)
