"""
Follow path controller using Pure Pursuit + CTE-PD algorithm.

Features:
- Path resampling for smooth following
- Arc-length gating to prevent skipping at self-intersections
- CTE-PD correction for drift compensation
- Rotate-in-place when heading error is large
- Speed scheduling based on curvature
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from robot_control.controller.base import Controller
from robot_control.core.types import Action, Observation, Subgoal, WorkspaceConfig

Point = Tuple[float, float]


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _enforce_deadband_scale(
    vl: float, vr: float, deadband: float = 0.05
) -> Tuple[float, float]:
    """
    Scale wheel speeds to overcome motor deadband.

    If both wheels are below deadband but non-zero, scale up so the
    larger wheel reaches deadband. This prevents motors from stalling.
    """
    max_abs = max(abs(vl), abs(vr))

    # Both near zero - stop
    if max_abs < 1e-6:
        return 0.0, 0.0

    # Already above deadband - no change needed
    if max_abs >= deadband:
        return vl, vr

    # Scale up to reach deadband
    scale = deadband / max_abs
    vl2 = vl * scale
    vr2 = vr * scale

    # Ensure still in [-1, 1]
    m2 = max(abs(vl2), abs(vr2))
    if m2 > 1.0:
        s2 = 1.0 / m2
        vl2 *= s2
        vr2 *= s2

    return vl2, vr2


def _wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    return rad


def _project_point_to_segment(p: Point, a: Point, b: Point) -> Tuple[Point, float]:
    """Project point p onto line segment ab. Returns (projection, t)."""
    ax, ay = a
    bx, by = b
    px, py = p

    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return a, 0.0

    t = ((px - ax) * vx + (py - ay) * vy) / denom
    t = _clamp(t, 0.0, 1.0)
    return (ax + t * vx, ay + t * vy), t


class FollowPathController(Controller):
    """
    Pure Pursuit + CTE-PD path following controller.

    Features:
    - Path resampling (max gap <= 1% of car_size)
    - Arc-length gating (prevents skipping at self-intersections)
    - CTE-PD correction (proportional + derivative on cross-track error)
    - ALIGN mode with hysteresis (rotate in-place when heading error large)
    - Speed scheduling based on curvature
    - Wheel deadband handling

    Status labels:
    - "IDLE": No path assigned
    - "FOLLOWING": Actively following path
    - "FINISHED": Reached end of path
    """

    # ALIGN mode hysteresis thresholds (degrees)
    ALIGN_ENTER_DEG = 60.0  # Enter ALIGN mode when heading error > this
    ALIGN_EXIT_DEG = 45.0  # Exit ALIGN mode when heading error < this
    WHEEL_DEADBAND = 0.05  # Motor deadband threshold

    def __init__(
        self,
        config: WorkspaceConfig,
        lookahead_distance: Optional[float] = None,
        max_speed: float = 0.3,
        goal_tolerance: Optional[float] = None,
        max_point_gap_ratio: float = 0.01,  # Path point gap <= ratio * car_size
        no_skip_ratio: float = 0.5,  # Max arc-length progress per step
    ) -> None:
        """
        Initialize follow path controller.

        Args:
            config: Workspace configuration (dimensions, robot geometry)
            lookahead_distance: Distance to look ahead (default: 0.5 * car_size)
            max_speed: Maximum wheel speed [0, 1] (default: 0.4)
            goal_tolerance: Distance to consider goal reached (default: 0.2 * car_size)
            max_point_gap_ratio: Max gap between path points as ratio of car_size
            no_skip_ratio: Max arc-length to search ahead as ratio of car_size
        """
        self._config = config

        # Size baseline
        self._car_size = max(config.car_width, config.car_height)

        # Parameters
        self._max_point_gap_ratio = max(1e-4, float(max_point_gap_ratio))
        self._no_skip_ratio = max(0.1, float(no_skip_ratio))
        self._lookahead_distance = lookahead_distance or (0.5 * self._car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))
        self._goal_tolerance = goal_tolerance or (0.2 * self._car_size)

        # CTE-PD parameters
        self._cte_dot_alpha = 0.25  # Derivative smoothing (0-1, higher = less smoothing)
        self._min_turn_factor = 0.25  # Minimum speed multiplier for turns

        # Path state
        self._path_raw: List[Point] = []
        self._path: List[Point] = []  # Resampled path
        self._path_s: List[float] = []  # Prefix arc-length
        self._path_index: int = 0

        # PD state
        self._prev_cte: Optional[float] = None
        self._prev_cte_time: Optional[float] = None
        self._cte_dot_filt: float = 0.0

        # ALIGN mode state (hysteresis)
        self._align_active: bool = False

        # Status
        self._status = "IDLE"
        self._metadata: Dict[str, Any] = {}

    @property
    def path(self) -> List[Point]:
        """Get current resampled path."""
        return self._path.copy()

    @property
    def path_index(self) -> int:
        """Get current progress along path."""
        return self._path_index

    @property
    def lookahead_distance(self) -> float:
        return self._lookahead_distance

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def metadata(self) -> Dict[str, Any]:
        return self._metadata

    # -------------------------------------------------------------------------
    # Path preprocessing
    # -------------------------------------------------------------------------

    def _resample_path(self, path: List[Point], max_step: float) -> List[Point]:
        """Resample path so adjacent points are at most max_step apart."""
        if len(path) <= 1:
            return list(path)

        out: List[Point] = []
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            d = math.hypot(dx, dy)

            if d < 1e-12:
                continue

            n = int(math.ceil(d / max_step))

            if not out:
                out.append((x0, y0))

            for k in range(1, n + 1):
                t = k / n
                out.append((x0 + t * dx, y0 + t * dy))

        return out

    def _build_prefix_s(self) -> None:
        """Build prefix arc-length array for gating."""
        self._path_s = [0.0]
        for i in range(len(self._path) - 1):
            x0, y0 = self._path[i]
            x1, y1 = self._path[i + 1]
            self._path_s.append(self._path_s[-1] + math.hypot(x1 - x0, y1 - y0))

    def _max_reachable_index(self, start_idx: int) -> int:
        """Get max index reachable from start within no_skip_ratio * car_size."""
        if not self._path_s:
            return start_idx

        start_idx = int(_clamp(start_idx, 0, len(self._path_s) - 1))
        s0 = self._path_s[start_idx]
        s_max = s0 + self._no_skip_ratio * self._car_size

        j = start_idx
        n = len(self._path_s)
        while j + 1 < n and self._path_s[j + 1] <= s_max:
            j += 1
        return j

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def set_path(self, path: List[Point]) -> None:
        """Set a new path to follow."""
        self._car_size = max(self._config.car_width, self._config.car_height)

        self._path_raw = list(path)
        max_step = self._max_point_gap_ratio * self._car_size
        self._path = self._resample_path(self._path_raw, max_step)
        self._build_prefix_s()

        self._path_index = 0
        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0
        self._align_active = False  # Reset ALIGN state

        if self._path:
            self._status = "FOLLOWING"
            self._metadata["path"] = self._path
            self._metadata["path_index"] = self._path_index
        else:
            self._status = "IDLE"
            self._metadata.pop("path", None)
            self._metadata.pop("path_index", None)

        self._metadata.pop("target_point", None)

    def clear_path(self) -> None:
        """Clear current path and stop."""
        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0
        self._status = "IDLE"

        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0
        self._align_active = False

        self._metadata.pop("path", None)
        self._metadata.pop("path_index", None)
        self._metadata.pop("target_point", None)

    def cancel(self) -> None:
        """Cancel path following. Alias for clear_path()."""
        self.clear_path()

    def set_speed(self, speed: float) -> None:
        """Set maximum speed."""
        self._max_speed = max(0.0, min(1.0, speed))

    def get_status(self) -> str:
        """Get current status label."""
        return self._status

    def step(self, obs: Observation, subgoal: Subgoal) -> Action:
        """Compute action to follow path."""
        return self._calculate_action(obs)

    def is_done(self, obs: Observation, subgoal: Subgoal) -> bool:
        """Check if path following is complete."""
        return self._status == "FINISHED"

    def reset(self) -> None:
        """Reset controller state."""
        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0
        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0
        self._align_active = False
        self._status = "IDLE"
        self._metadata.clear()

    # -------------------------------------------------------------------------
    # Core control logic
    # -------------------------------------------------------------------------

    def _calculate_action(self, obs: Observation) -> Action:
        if not self._path:
            self._status = "IDLE"
            return Action.stop()

        robot_x = obs.robot_x
        robot_y = obs.robot_y
        robot_theta = obs.robot_theta

        self._car_size = max(self._config.car_width, self._config.car_height)
        wheel_base = self._config.wheel_base

        # Check if reached goal
        final_point = self._path[-1]
        dist_to_goal = math.hypot(robot_x - final_point[0], robot_y - final_point[1])
        if dist_to_goal < self._goal_tolerance:
            self._status = "FINISHED"
            self._metadata.pop("target_point", None)
            return Action.stop()

        # Find target point (gated)
        target_point = self._find_target_point(robot_x, robot_y)
        if target_point is None:
            target_point = final_point

        # Find closest reference for CTE (gated)
        ref_point, theta_ref_rad, cte_raw, seg_idx = self._find_closest_reference(
            (robot_x, robot_y)
        )
        self._path_index = max(self._path_index, seg_idx)

        # Update metadata
        self._metadata["target_point"] = target_point
        self._metadata["path_index"] = self._path_index
        self._status = "FOLLOWING"

        # Pure pursuit curvature
        curvature_pp, heading_error_rad, dist_to_target = self._pure_pursuit_curvature(
            robot_x, robot_y, robot_theta, target_point
        )

        heading_deg = abs(math.degrees(heading_error_rad))

        # ALIGN mode hysteresis (rotate in-place when heading error is large)
        if self._align_active:
            if heading_deg <= self.ALIGN_EXIT_DEG:
                self._align_active = False
        else:
            if heading_deg >= self.ALIGN_ENTER_DEG:
                self._align_active = True

        # Handle ALIGN mode (rotate in-place)
        if self._align_active:
            w = 0.6 * self._max_speed
            left_speed = -w if heading_error_rad > 0 else w
            right_speed = w if heading_error_rad > 0 else -w

            # Apply deadband scaling
            left_speed, right_speed = _enforce_deadband_scale(
                left_speed, right_speed, self.WHEEL_DEADBAND
            )

            self._metadata["mode"] = "ALIGN"
            self._metadata["heading_error_deg"] = math.degrees(heading_error_rad)
            self._metadata["target_point"] = target_point

            return Action(
                left_speed=_clamp(left_speed, -1.0, 1.0),
                right_speed=_clamp(right_speed, -1.0, 1.0),
            )

        # Gating parameters (car_size-based)
        dist_to_path = abs(cte_raw)
        cte_pd_enable_dist = 2.5 * self._car_size
        cte_clip = 1.0 * self._car_size
        cte_deadband = 0.10 * self._car_size
        curv_pd_max = 2.0 / max(self._car_size, 1e-9)
        curvature_max = 3.5 / max(self._car_size, 1e-9)
        rotate_in_place_rad = math.radians(110.0)

        # Speed scheduling
        turn_factor = max(self._min_turn_factor, math.cos(abs(heading_error_rad)))
        base_speed = self._max_speed * turn_factor

        # PD enable when close to path
        use_pd = dist_to_path <= cte_pd_enable_dist

        # CTE with deadband and clamp
        cte_used = cte_raw
        if abs(cte_used) < cte_deadband:
            cte_used = 0.0
        cte_used = _clamp(cte_used, -cte_clip, cte_clip)

        # Compute CTE-PD correction
        now_t = obs.timestamp
        if not use_pd or now_t is None:
            self._prev_cte = None
            self._prev_cte_time = None
            self._cte_dot_filt = 0.0
            cte_pd_curv = 0.0
        else:
            if self._prev_cte is None or self._prev_cte_time is None:
                cte_dot = 0.0
            else:
                dt = now_t - self._prev_cte_time
                cte_dot = (cte_used - self._prev_cte) / dt if dt > 1e-6 else 0.0

            # Smoothed derivative
            self._cte_dot_filt = (
                (1.0 - self._cte_dot_alpha) * self._cte_dot_filt
                + self._cte_dot_alpha * cte_dot
            )

            self._prev_cte = cte_used
            self._prev_cte_time = now_t

            # PD gains
            kp = 1.2
            kd = 0.35

            # Normalize by car_size
            cte_n = cte_used / max(self._car_size, 1e-9)
            cte_dot_n = self._cte_dot_filt / max(self._car_size, 1e-9)

            cte_pd_curv = (kp * cte_n + kd * cte_dot_n) / max(self._car_size, 1e-9)
            cte_pd_curv = _clamp(cte_pd_curv, -curv_pd_max, curv_pd_max)

        # Combined curvature
        curvature_cmd = curvature_pp + cte_pd_curv
        curvature_cmd = _clamp(curvature_cmd, -curvature_max, curvature_max)

        # Slow down for high curvature
        curv_slow = _clamp(
            1.0 - 0.65 * (abs(curvature_cmd) / max(curvature_max, 1e-9)), 0.35, 1.0
        )
        base_speed *= curv_slow

        # Rotate-in-place when far from path and facing wrong way
        if (not use_pd) and (abs(heading_error_rad) > rotate_in_place_rad):
            w = 0.6 * self._max_speed
            left_speed = -w if heading_error_rad > 0 else w
            right_speed = w if heading_error_rad > 0 else -w

            # Apply deadband scaling
            left_speed, right_speed = _enforce_deadband_scale(
                left_speed, right_speed, self.WHEEL_DEADBAND
            )

            self._metadata["mode"] = "ROTATE_IN_PLACE"
            return Action(
                left_speed=_clamp(left_speed, -1.0, 1.0),
                right_speed=_clamp(right_speed, -1.0, 1.0),
            )

        # Convert curvature to differential drive
        diff = curvature_cmd * wheel_base / 2.0
        left_speed = base_speed * (1.0 - diff)
        right_speed = base_speed * (1.0 + diff)

        # Normalize to max speed
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        # Apply deadband scaling
        left_speed, right_speed = _enforce_deadband_scale(
            left_speed, right_speed, self.WHEEL_DEADBAND
        )

        left_speed = _clamp(left_speed, -1.0, 1.0)
        right_speed = _clamp(right_speed, -1.0, 1.0)

        self._metadata["mode"] = "TRACK" if use_pd else "ACQUIRE"

        return Action(left_speed=left_speed, right_speed=right_speed)

    # -------------------------------------------------------------------------
    # Path search (gated to prevent skipping)
    # -------------------------------------------------------------------------

    def _find_target_point(self, robot_x: float, robot_y: float) -> Optional[Point]:
        """Find target point using pure pursuit with arc-length gating."""
        if not self._path:
            return None

        start = self._path_index
        imax = self._max_reachable_index(start)

        # Find closest point in window
        best_idx = start
        best_dist = float("inf")

        for i in range(start, imax + 1):
            px, py = self._path[i]
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        self._path_index = max(self._path_index, best_idx)

        # Find lookahead point in window
        start = self._path_index
        imax = self._max_reachable_index(start)

        for i in range(start, imax + 1):
            px, py = self._path[i]
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist >= self._lookahead_distance:
                self._path_index = i
                return (px, py)

        return self._path[min(imax, len(self._path) - 1)]

    def _pure_pursuit_curvature(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta_deg: float,
        target: Point,
    ) -> Tuple[float, float, float]:
        """Calculate pure pursuit curvature."""
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            return 0.0, 0.0, 0.0

        angle_to_target = math.atan2(dy, dx)
        robot_theta_rad = math.radians(robot_theta_deg)
        heading_error_rad = _wrap_to_pi(angle_to_target - robot_theta_rad)

        L = max(distance, self._lookahead_distance)
        curvature_pp = 2.0 * math.sin(heading_error_rad) / L
        return curvature_pp, heading_error_rad, distance

    def _find_closest_reference(
        self, pos: Point
    ) -> Tuple[Point, float, float, int]:
        """Find closest point on path for CTE calculation (gated)."""
        x, y = pos
        n = len(self._path)

        if n == 1:
            px, py = self._path[0]
            cte = math.hypot(x - px, y - py)
            return (px, py), 0.0, cte, 0

        start_i = min(self._path_index, n - 2)
        end_i = min(self._max_reachable_index(start_i), n - 1)
        seg_end = max(start_i, min(end_i - 1, n - 2))

        best_dist2 = float("inf")
        best_proj: Point = self._path[start_i]
        best_theta = 0.0
        best_cte = 0.0
        best_i = start_i

        for i in range(start_i, seg_end + 1):
            a = self._path[i]
            b = self._path[i + 1]
            proj, _t = _project_point_to_segment((x, y), a, b)

            dx = x - proj[0]
            dy = y - proj[1]
            dist2 = dx * dx + dy * dy

            if dist2 < best_dist2:
                best_dist2 = dist2
                best_proj = proj
                best_i = i

                seg_dx = b[0] - a[0]
                seg_dy = b[1] - a[1]
                theta_ref = math.atan2(seg_dy, seg_dx)
                best_theta = theta_ref

                # Signed CTE (positive = left of path)
                nx = -math.sin(theta_ref)
                ny = math.cos(theta_ref)
                best_cte = nx * (x - proj[0]) + ny * (y - proj[1])

        return best_proj, best_theta, best_cte, best_i

    # -------------------------------------------------------------------------
    # Visualization
    # -------------------------------------------------------------------------

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Get drawing commands for canvas visualization."""
        drawings = []

        # Draw path
        path = self._metadata.get("path")
        if path and len(path) >= 2:
            drawings.append({
                "uuid": "follow_path",
                "type": "path",
                "points": path,
                "color": "#00FF00",
                "width": 2,
            })

        # Draw target point
        target = self._metadata.get("target_point")
        if target and self._status == "FOLLOWING":
            drawings.append({
                "uuid": "target_point",
                "type": "point",
                "position": target,
                "radius": 5,
                "color": "#FF0000",
                "fill": "#FF0000",
            })

        return drawings
