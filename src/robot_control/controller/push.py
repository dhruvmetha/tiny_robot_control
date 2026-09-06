"""Push controller using pure pursuit to push objects.

The controller continuously steers toward a target point that updates each tick
based on the object's current position. This creates a feedback loop where the
robot tracks the moving object as it pushes.

Edge Index Layout (namo_cpp compatible):
    Uses multi-point edge generation matching namo_cpp/src/planning/namo_push_controller.cpp.
    With points_per_face=1 (legacy): 4 edge points (indices 0-3)
    With points_per_face=3 (standard): 12 edge points (indices 0-11)

    For points_per_face=1:
        Index 0: Top (+Y face), push toward -Y
        Index 1: Bottom (-Y face), push toward +Y
        Index 2: Right (+X face), push toward -X
        Index 3: Left (-X face), push toward +X

    For points_per_face=3 (see edge_points.py for full layout):
        Indices 0-5: Top/Bottom pairs (3 samples along width)
        Indices 6-11: Right/Left pairs (3 samples along depth)

Approach Phase:
    Before pushing, the robot navigates to a standoff position behind the
    object face it will push from. The approach phase:
    1. Computes approach position (standoff from object face)
    2. Computes approach orientation (robot faces push direction)
    3. Validates position with wavefront (finds nearest free if blocked)
    4. Navigates to approach position using NavigationController
    5. Then executes push via pure pursuit
"""

from __future__ import annotations

import json
import math
import os
from enum import Enum
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

from robot_control.controller.base import Controller
from robot_control.controller.retreat import find_retreat_target, reverse_toward
from robot_control.controller.config import PushConfig
from robot_control.controller.edge_points import (
    EdgePoint,
    generate_edge_points,
    get_edge_point,
)
from robot_control.controller.follow_path import FollowPathController
from robot_control.controller.safety_filter import SafetyFilter, Violation
from robot_control.core.types import (
    Action,
    NavigateSubgoal,
    ObjectPose,
    Observation,
    PushSubgoal,
    Subgoal,
    WorkspaceConfig,
)
from robot_control.utils.wavefront_inflation_config import get_wavefront_inflation_config
from robot_control.utils.wavefront import WavefrontConfig, WavefrontPlanner
from robot_control.utils.robot_geometry import (
    effective_robot_radius_cm,
    effective_robot_radius_m_from_cm,
    effective_robot_size_cm,
)

if TYPE_CHECKING:
    from robot_control.controller.navigation import NavigationController

Point = Tuple[float, float]


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _enforce_deadband_scale(
    vl: float, vr: float, deadband: float = 0.05
) -> Tuple[float, float]:
    """Scale wheel speeds to overcome motor deadband."""
    max_abs = max(abs(vl), abs(vr))
    if max_abs < 1e-6:
        return 0.0, 0.0
    if max_abs >= deadband:
        return vl, vr

    scale = deadband / max_abs
    vl2 = vl * scale
    vr2 = vr * scale

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


def _wheel_log_paths_from_env(raw: str) -> List[str]:
    """Split NAMO_PUSH_WHEEL_LOG into one or more output paths."""
    parts = [p for p in raw.split(os.pathsep) if p]
    return parts or [raw]


class PushState(Enum):
    """Push controller state machine."""

    IDLE = "IDLE"
    COMPUTING_APPROACH = "COMPUTING_APPROACH"
    APPROACHING = "APPROACHING"
    ADVANCING = "ADVANCING"
    PUSHING = "PUSHING"
    RETREATING = "RETREATING"
    FINISHED = "FINISHED"
    FAILED = "FAILED"


class PushController(Controller):
    """
    Push controller with approach phase and pure pursuit pushing.

    State machine:
        IDLE -> COMPUTING_APPROACH -> APPROACHING -> PUSHING -> RETREATING -> FINISHED
                        |                                |           |
                        v                                v           v
                      FAILED                          FAILED     FINISHED

    The approach phase navigates the robot to a standoff position behind
    the object face before pushing. This ensures the robot is correctly
    positioned regardless of where it starts.

    Each tick during PUSHING:
    1. Get object position from observation
    2. Compute target point on object face based on edge_idx
    3. Pure pursuit toward target
    4. Target updates as object moves → feedback loop

    Usage:
        nav_controller = NavigationController(config, planner)
        controller = PushController(workspace_config, nav_controller)
        subgoal = PushSubgoal(object_id="box1", edge_idx=0, push_steps=100)

        while not controller.is_done(obs, subgoal):
            action = controller.step(obs, subgoal)
            env.apply(action)
    """

    def __init__(
        self,
        config: WorkspaceConfig,
        nav_controller: Optional["NavigationController"] = None,
        push_config: Optional[PushConfig] = None,
        max_speed: Optional[float] = None,
        safety_filter: Optional[SafetyFilter] = None,
    ) -> None:
        """
        Initialize push controller.

        Args:
            config: Workspace configuration
            nav_controller: Navigation controller for approach phase (optional)
            push_config: Push parameters (from YAML). If None, uses defaults.
            max_speed: Override max speed (if None, uses push_config value)
            safety_filter: Optional wall-clearance guard. None leaves every
                phase exactly as it runs without it.
        """
        self._config = config
        self._nav_controller = nav_controller
        self._push_config = push_config or PushConfig()
        # Lives for the session, like the walls it knows about. Every use is
        # guarded by an `is not None` check so the filter is a pure add-on.
        self._safety_filter = safety_filter
        self._max_speed = max_speed if max_speed is not None else self._push_config.max_speed

        # Match the C++ planner's edge-point spawn offset exactly:
        # standoff = robot_radius + edge_offset_margin_cm.
        car_size = effective_robot_size_cm(config.car_width, config.car_height)
        if self._push_config.edge_offset_margin_cm is None:
            raise ValueError(
                "PushController requires push.edge_offset_margin_cm to be set "
                "so runtime standoff matches the C++ planner edge offset."
            )
        robot_radius_cm = effective_robot_radius_cm(
            config.car_width, config.car_height
        )
        self._standoff_distance = (
            robot_radius_cm + float(self._push_config.edge_offset_margin_cm)
        )

        # Edge point configuration (matches namo_cpp)
        self._points_per_face = self._push_config.points_per_face
        self._dynamic_direction = self._push_config.dynamic_direction

        # Follow path controller for push phase (Pure Pursuit + CTE-PD).
        # Reuses the same car_size scalar computed above (effective_robot_size_cm).
        self._follow_path_controller = FollowPathController(
            config=config,
            lookahead_distance=self._push_config.lookahead_ratio * car_size,
            max_speed=self._max_speed,
            goal_tolerance=car_size * 0.3,  # Slightly loose tolerance for pushing
        )

        # State
        self._state = PushState.IDLE
        self._step_count = 0
        self._advance_step_count = 0
        self._retreat_step_count = 0
        self._approach_step_count = 0  # BUG-025 watchdog
        self._current_subgoal: Optional[PushSubgoal] = None

        # Approach phase state
        self._approach_position: Optional[Point] = None
        self._approach_orientation: Optional[float] = None

        # Push path state
        self._push_path: Optional[List[Point]] = None
        self._push_path_at_start: Optional[List[Point]] = None

        # Retreat phase state
        self._retreat_target: Optional[Point] = None
        self._retreat_is_backward: bool = False
        # Robot pose when the retreat began, so the outcome can report how far
        # the robot actually travelled and not only whether it arrived.
        self._retreat_start_pose: Optional[Tuple[float, float]] = None

        # Push outcome telemetry: object pose at PUSHING entry, for Δ reporting at PUSH COMPLETE
        self._push_start_obj_pose: Optional[Tuple[float, float, float]] = None
        self._push_start_robot_pose: Optional[Tuple[float, float, float]] = None
        self._push_start_obs_timestamp: Optional[float] = None
        # Stuck flag: set at PUSH COMPLETE if Δobject < thresholds. Reported via
        # did_fail() so the upstream blacklist gets the (object_id, edge_idx) entry.
        self._push_movement_inadequate: bool = False

        # Safety filter outcome for this push. `_robot_collision_abort` feeds
        # did_fail() so the planner blacklists the edge, matching sim's
        # robot-wall failure. `_clearance_at_start` is taken lazily at the
        # first check of each phase (advance/push, then retreat) so the
        # started-inside rule compares against that phase's own start.
        self._abort_violation: Optional[Violation] = None
        self._robot_collision_abort: bool = False
        self._clearance_at_start: Optional[float] = None
        self._last_outline: Optional[List[Point]] = None
        self._last_check_violated: bool = False

        # Visualization
        self._target_point: Optional[Point] = None
        self._target_point_at_start: Optional[Point] = None
        self._object_pose: Optional[ObjectPose] = None

    @property
    def max_speed(self) -> float:
        return self._max_speed

    def set_speed(self, speed: float) -> None:
        """Set maximum speed."""
        self._max_speed = max(0.0, min(1.0, speed))
        self._follow_path_controller.set_speed(self._max_speed)

    def step(self, obs: Observation, subgoal: Subgoal) -> Action:
        """Compute action to push object."""
        if not isinstance(subgoal, PushSubgoal):
            return Action.stop()

        self._current_subgoal = subgoal

        # Debug: log state at start of each step
        # print(f"[PUSH] step() state={self._state.value} robot=({obs.robot_x:.1f},{obs.robot_y:.1f})")

        # Get object from observation
        obj = obs.objects.get(subgoal.object_id)
        if obj is None:
            # Object not visible - ArucoObserver should provide cached position for a few frames
            # If still None, the object has been lost for too long
            print(f"[PUSH] FAILED: Object '{subgoal.object_id}' not visible in observation!")
            print(f"[PUSH]   Available objects: {list(obs.objects.keys())}")
            self._state = PushState.FAILED
            return Action.stop()

        self._object_pose = obj

        # Every frame before the push itself is a chance to see a wall marker
        # the camera missed on another frame. Walls never move, so a wall is
        # recorded once and the call is a set-membership check afterwards.
        if self._safety_filter is not None and self._state in (
            PushState.IDLE,
            PushState.COMPUTING_APPROACH,
            PushState.APPROACHING,
            PushState.ADVANCING,
        ):
            self._safety_filter.add_statics(obs)

        # State machine
        if self._state == PushState.IDLE:
            # Start by computing approach
            self._state = PushState.COMPUTING_APPROACH

        if self._state == PushState.COMPUTING_APPROACH:
            action = self._handle_computing_approach(obs, obj, subgoal)
        elif self._state == PushState.APPROACHING:
            action = self._handle_approaching(obs, obj, subgoal)
        elif self._state == PushState.ADVANCING:
            action = self._handle_advancing(obs, obj, subgoal)
        elif self._state == PushState.PUSHING:
            action = self._handle_pushing(obs, obj, subgoal)
        elif self._state == PushState.RETREATING:
            action = self._handle_retreating(obs)
        elif self._state == PushState.FINISHED:
            action = Action.stop()
        elif self._state == PushState.FAILED:
            action = Action.stop()
        else:
            action = Action.stop()

        # Per-tick action log across ALL push-state phases. Enabled via
        # NAMO_PUSH_WHEEL_LOG=<path>. Use this to diagnose "robot is slow
        # during the push" — the `state` field tells you whether the slow
        # phase is APPROACHING (Nav handing back the action), PUSHING
        # (our FollowPathController), RETREATING, etc.
        log_path = os.getenv("NAMO_PUSH_WHEEL_LOG")
        if log_path:
            obs_ts = float(getattr(obs, "timestamp", 0.0) or 0.0)
            if not hasattr(self, "_wheel_log_t0_global"):
                self._wheel_log_t0_global = obs_ts
            t_rel = obs_ts - self._wheel_log_t0_global
            prev_obs_ts = getattr(self, "_wheel_log_prev_obs_timestamp", None)
            obs_reused = prev_obs_ts is not None and abs(obs_ts - prev_obs_ts) < 1e-9
            self._wheel_log_prev_obs_timestamp = obs_ts
            mode = str(self._follow_path_controller.metadata.get("mode", ""))
            sub = self._current_subgoal
            target_point = self._follow_path_controller.metadata.get("target_point")
            rec = {
                "t_s": t_rel,
                "obs_timestamp": obs_ts,
                "obs_reused": obs_reused,
                "left_cmd": float(action.left_speed),
                "right_cmd": float(action.right_speed),
                "mode": mode,
                "state": self._state.value,
                "path_index": int(self._follow_path_controller.metadata.get("path_index", 0) or 0),
                "heading_error_deg": self._follow_path_controller.metadata.get("heading_error_deg"),
                "target_point_cm": list(target_point) if target_point is not None else None,
                "max_speed_cap": float(self._max_speed),
                "object_id": getattr(sub, "object_id", None),
                "edge_idx": getattr(sub, "edge_idx", None),
                "push_steps": getattr(sub, "push_steps", None),
            }
            payload = json.dumps(rec)
            for path in _wheel_log_paths_from_env(log_path):
                parent = os.path.dirname(path)
                if parent:
                    os.makedirs(parent, exist_ok=True)
                with open(path, "a") as f:
                    f.write(payload + "\n")
        return action

    def _handle_computing_approach(
        self, obs: Observation, obj: ObjectPose, subgoal: PushSubgoal
    ) -> Action:
        """Compute approach position and start navigation."""
        # Get edge point with position and mid_point
        ep = self._get_edge_point(obj, subgoal.edge_idx)
        approach_pos = ep.position
        approach_theta = ep.approach_theta
        mid_pt = ep.mid_point

        self._approach_position = approach_pos
        self._approach_orientation = approach_theta

        # Check if we're already close enough to skip approach
        robot_pos = (obs.robot_x, obs.robot_y)
        dist_to_approach = math.hypot(
            approach_pos[0] - robot_pos[0], approach_pos[1] - robot_pos[1]
        )
        angle_error = abs(_wrap_to_pi(math.radians(approach_theta - obs.robot_theta)))
        angle_error_deg = math.degrees(angle_error)

        if dist_to_approach < self._push_config.approach_skip_distance and angle_error_deg < self._push_config.approach_skip_angle:
            # Already at approach position, skip to pushing.
            # Capture pre-push poses here too — _handle_advancing normally sets
            # them at line 423-424, but we're bypassing ADVANCING entirely. Without
            # this, _push_start_obj_pose stays None and the PUSH COMPLETE block
            # silently skips the Δobject log + the stuck check + the pushes.jsonl
            # record (push.py:987, 1027).
            self._push_start_obj_pose = (obj.x, obj.y, obj.theta)
            self._push_start_robot_pose = (obs.robot_x, obs.robot_y, obs.robot_theta)
            self._push_start_obs_timestamp = (
                float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
            )
            print(
                f"[PUSH] >>> PUSHING start (skip-approach): obj '{subgoal.object_id}' at "
                f"({obj.x:.1f},{obj.y:.1f},θ={obj.theta:.1f}°), "
                f"robot at ({obs.robot_x:.1f},{obs.robot_y:.1f},θ={obs.robot_theta:.1f}°), "
                f"edge={subgoal.edge_idx}, push_steps={subgoal.push_steps}"
            )
            self._state = PushState.PUSHING
            return self._handle_pushing(obs, obj, subgoal)

        # Validate approach position along push line (not arbitrary directions)
        validated_pos = self._validate_approach_position(obs, approach_pos, mid_pt)
        if validated_pos is None:
            # Unreachable - fail
            print(f"[PUSH] Approach position unreachable: ({approach_pos[0]:.1f}, {approach_pos[1]:.1f})")
            self._state = PushState.FAILED
            return Action.stop()

        # Check if wavefront adjusted the position
        if validated_pos != approach_pos:
            dist_moved = math.hypot(validated_pos[0] - approach_pos[0], validated_pos[1] - approach_pos[1])
            if dist_moved > 3.0:
                print(f"[PUSH] WARNING: Approach adjusted by {dist_moved:.1f}cm (edge blocked by nearby obstacle)")
            print(f"[PUSH]   Original edge: ({approach_pos[0]:.1f}, {approach_pos[1]:.1f})")
            print(f"[PUSH]   Validated:     ({validated_pos[0]:.1f}, {validated_pos[1]:.1f})")

        self._approach_position = validated_pos

        # Start navigation to approach position
        if self._nav_controller is None:
            # No nav controller - skip approach, go straight to pushing
            self._state = PushState.PUSHING
            return self._handle_pushing(obs, obj, subgoal)

        # Build obstacle list for navigation (include ALL objects)
        # The approach position is already outside the target object,
        # so the path planner should avoid all objects including the target
        obstacles = self._build_obstacles(obs)

        # Navigate to approach position with correct orientation
        print(f"[PUSH] Starting navigation: robot=({robot_pos[0]:.1f},{robot_pos[1]:.1f}) → approach=({validated_pos[0]:.1f},{validated_pos[1]:.1f})")
        success = self._nav_controller.navigate_to(
            validated_pos[0],
            validated_pos[1],
            approach_theta,
            robot_pos,
            obstacles,
        )
        print(f"[PUSH] navigate_to() returned: {success}, nav_state={self._nav_controller.state.value}")

        if not success:
            # Navigation planning failed - add debug info
            print(f"[PUSH] Navigation planning failed for approach to ({validated_pos[0]:.1f}, {validated_pos[1]:.1f})")
            print(f"[PUSH]   Robot position: ({robot_pos[0]:.1f}, {robot_pos[1]:.1f})")
            print(f"[PUSH]   Distance to approach: {math.hypot(validated_pos[0] - robot_pos[0], validated_pos[1] - robot_pos[1]):.1f}cm")
            print(f"[PUSH]   Obstacles passed to navigation: {len(obstacles)}")
            for i, o in enumerate(obstacles):
                print(f"[PUSH]     [{i}] pos=({o[0]:.1f}, {o[1]:.1f}) theta={o[2]:.1f}° size=({o[3]:.0f}x{o[4]:.0f})")

            # Check if robot position is free in wavefront
            wavefront = self._build_wavefront(obs)
            robot_m = (robot_pos[0] / 100.0, robot_pos[1] / 100.0)
            goal_m = (validated_pos[0] / 100.0, validated_pos[1] / 100.0)
            robot_free = wavefront.is_free(robot_m[0], robot_m[1])
            goal_free = wavefront.is_free(goal_m[0], goal_m[1])
            print(f"[PUSH]   Wavefront check: robot={'FREE' if robot_free else 'BLOCKED'}, goal={'FREE' if goal_free else 'BLOCKED'}")

            # Try wavefront path planning
            wavefront_path = wavefront.plan(robot_m, goal_m)
            if wavefront_path:
                print(f"[PUSH]   Wavefront CAN find path ({len(wavefront_path)} waypoints) - RVG is the issue")
            else:
                print(f"[PUSH]   Wavefront also cannot find path - position truly unreachable")

            self._state = PushState.FAILED
            return Action.stop()

        self._state = PushState.APPROACHING
        self._approach_step_count = 0
        return self._nav_controller.step(obs, None)

    def _handle_approaching(
        self, obs: Observation, obj: ObjectPose, subgoal: PushSubgoal
    ) -> Action:
        """Execute approach navigation."""
        if self._nav_controller is None:
            print("[PUSH] No nav controller - skipping approach")
            self._state = PushState.ADVANCING
            self._advance_step_count = 0
            return self._handle_advancing(obs, obj, subgoal)

        # Check if navigation is done
        if self._nav_controller.is_done(obs, None):
            print(f"[PUSH] Navigation done - transitioning to ADVANCING (nav_state={self._nav_controller.state.value})")
            self._state = PushState.ADVANCING
            self._advance_step_count = 0
            return self._handle_advancing(obs, obj, subgoal)

        # Watchdog disabled per request 2026-05-20: BUG-025's APPROACHING
        # timeout (180 ticks ≈ 6 s) was tripping on slow-but-progressing nav
        # — e.g., a 175° in-place rotation followed by a 10 cm drive can
        # legitimately exceed 6 s. The deadlock case BUG-025 was guarding
        # against is rarer than the false-trip case. Counter still tracked
        # for get_status() display only; no longer compared to a limit.
        self._approach_step_count += 1

        # Continue navigation
        return self._nav_controller.step(obs, None)

    def _handle_advancing(
        self, obs: Observation, obj: ObjectPose, subgoal: PushSubgoal
    ) -> Action:
        """Advance toward object to close gap before pushing."""
        # Check if advance is complete
        if self._advance_step_count >= self._push_config.advance_steps:
            # Capture pre-push poses for outcome reporting at PUSH COMPLETE
            self._push_start_obj_pose = (obj.x, obj.y, obj.theta)
            self._push_start_robot_pose = (obs.robot_x, obs.robot_y, obs.robot_theta)
            self._push_start_obs_timestamp = (
                float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
            )
            print(
                f"[PUSH] >>> PUSHING start: obj '{subgoal.object_id}' at "
                f"({obj.x:.1f},{obj.y:.1f},θ={obj.theta:.1f}°), "
                f"robot at ({obs.robot_x:.1f},{obs.robot_y:.1f},θ={obs.robot_theta:.1f}°), "
                f"edge={subgoal.edge_idx}, push_steps={subgoal.push_steps} "
                f"({subgoal.push_steps * self._push_config.push_steps} ticks)"
            )
            self._state = PushState.PUSHING
            return self._handle_pushing(obs, obj, subgoal)

        self._advance_step_count += 1

        # Simple forward motion at fixed speed
        advance_speed = self._push_config.advance_speed
        action = Action(left_speed=advance_speed, right_speed=advance_speed)

        violation = self._safety_check(obs)
        if violation is not None:
            self._finish_push(obs, violation)
            return Action.stop()
        return action

    def _handle_pushing(
        self, obs: Observation, obj: ObjectPose, subgoal: PushSubgoal
    ) -> Action:
        """Execute push phase with Pure Pursuit + CTE-PD path following."""
        # Check if push is complete, transition to retreating
        # Total ticks = NAMO push_steps * config.push_steps (ticks per NAMO step)
        total_ticks = subgoal.push_steps * self._push_config.push_steps
        if self._step_count >= total_ticks:
            self._finish_push(obs, None)
            return self._handle_retreating(obs)

        # Compute push direction based on dynamic_direction setting
        # dynamic=True: update every step, dynamic=False: fix at start
        should_compute = self._dynamic_direction or self._step_count == 0

        if should_compute:
            # Get theoretical edge_pt and mid_pt from edge point calculation
            ep = self._get_edge_point(obj, subgoal.edge_idx)
            edge_pt = ep.position
            mid_pt = ep.mid_point

            # Compute THEORETICAL push direction (edge_pt → mid_pt)
            # This is the intended push direction, perpendicular to the face
            dx = mid_pt[0] - edge_pt[0]
            dy = mid_pt[1] - edge_pt[1]
            length = math.hypot(dx, dy)

            if length > 1e-6:
                # Normalize direction
                dir_x = dx / length
                dir_y = dy / length

                # Robot's actual position
                actual_pos = (obs.robot_x, obs.robot_y)

                # Construct PARALLEL line through actual_pos with same direction
                # This keeps the exact push direction but shifts the line
                # to pass through where the robot actually is
                new_mid_pt = (
                    actual_pos[0] + dir_x * length,
                    actual_pos[1] + dir_y * length,
                )

                # Extend push target well beyond
                extend_dist = 50.0  # cm

                extended_pt = (
                    new_mid_pt[0] + dir_x * extend_dist,
                    new_mid_pt[1] + dir_y * extend_dist,
                )
                self._target_point = extended_pt

                # Create path: actual_pos → new_mid_pt → extended_pt
                # Parallel to theoretical line, same direction, straight motion
                self._push_path = [actual_pos, new_mid_pt, extended_pt]
            else:
                # Fallback: just use mid_pt
                actual_pos = (obs.robot_x, obs.robot_y)
                new_mid_pt = mid_pt
                self._target_point = mid_pt
                self._push_path = [actual_pos, mid_pt, mid_pt]

            if self._step_count == 0 and self._push_path is not None:
                self._push_path_at_start = [tuple(pt) for pt in self._push_path]
                self._target_point_at_start = (
                    tuple(self._target_point) if self._target_point is not None else None
                )

            # Update path in follow controller
            self._follow_path_controller.set_path(self._push_path)
            self._follow_path_controller.set_speed(self._max_speed)

            # Debug output on first step only
            if self._step_count == 0:
                mode = "DYNAMIC (updates every step)" if self._dynamic_direction else "FIXED"
                print(f"[PUSH DEBUG] ========== PUSH START ==========")
                print(f"[PUSH DEBUG] Object: pos=({obj.x:.1f},{obj.y:.1f}) theta={obj.theta:.1f}° size=({obj.width}x{obj.depth})")
                print(f"[PUSH DEBUG] Edge {subgoal.edge_idx}: face={ep.face_idx} sample={ep.sample_idx}")
                print(f"[PUSH DEBUG] Theoretical: edge_pt ({edge_pt[0]:.1f},{edge_pt[1]:.1f}) → mid_pt ({mid_pt[0]:.1f},{mid_pt[1]:.1f})")
                print(f"[PUSH DEBUG] Actual line: ({actual_pos[0]:.1f},{actual_pos[1]:.1f}) → ({new_mid_pt[0]:.1f},{new_mid_pt[1]:.1f})")
                print(f"[PUSH DEBUG] Mode: {mode}")
                print(f"[PUSH DEBUG] ================================")

        # Use FollowPathController for action (Pure Pursuit + CTE-PD)
        action = self._follow_path_controller.step(obs, None)

        # Increment step count
        self._step_count += 1

        violation = self._safety_check(obs)
        if violation is not None:
            self._finish_push(obs, violation)
            return Action.stop()
        return action

    def _safety_check(self, obs: Observation) -> Optional[Violation]:
        """Ask the safety filter whether the robot is entering a wall band.

        None when no filter is installed, so the phases run untouched.
        """
        if self._safety_filter is None:
            return None
        if self._clearance_at_start is None:
            self._clearance_at_start = self._safety_filter.robot_clearance(
                obs.robot_x, obs.robot_y, obs.robot_theta
            ).distance_cm
        xs, ys = self._safety_filter.robot_outline(obs.robot_x, obs.robot_y, obs.robot_theta)
        self._last_outline = list(zip(xs.tolist(), ys.tolist()))
        violation = self._safety_filter.check_entering(
            obs.robot_x, obs.robot_y, obs.robot_theta, self._clearance_at_start
        )
        self._last_check_violated = violation is not None
        return violation

    def _finish_push(self, obs: Observation, violation: Optional[Violation]) -> None:
        """End the push phase and arm the retreat.

        The normal end (tick budget spent) and a safety-filter abort share
        this so the stuck evaluation, the PUSH COMPLETE report and the
        retreat set-up cannot drift apart. An abort from ADVANCING has no
        push-start pose yet; it takes the current one so the push still
        produces a pushes.jsonl record, which the stuck rule then marks.
        """
        if violation is not None:
            self._abort_violation = violation
            self._robot_collision_abort = True
            start = (
                f"{violation.distance_at_start_cm:.2f}"
                if violation.distance_at_start_cm is not None else "n/a"
            )
            print(
                f"[PUSH] !! CLEARANCE: robot {violation.distance_now_cm:.2f}cm from "
                f"'{violation.static_name}' (was {start}cm at start, margin "
                f"{self._safety_filter.margin_cm:.2f}cm) in {self._state.value}. "
                f"Stopping the push and marking the edge FAILED."
            )
            if self._push_start_obj_pose is None:
                obj = self._object_pose
                if obj is not None:
                    self._push_start_obj_pose = (obj.x, obj.y, obj.theta)
                self._push_start_robot_pose = (obs.robot_x, obs.robot_y, obs.robot_theta)
                self._push_start_obs_timestamp = (
                    float(obs.timestamp) if getattr(obs, "timestamp", None) is not None else None
                )
        self._print_wavefront_status(obs, show=self._push_config.show_wavefront)
        self._state = PushState.RETREATING
        self._retreat_step_count = 0
        # The retreat measures "started inside" against its own first tick,
        # not the push's, or an abort at 1.4 cm would re-fire on the first
        # reverse tick while the robot is already moving away.
        self._clearance_at_start = None

    def _find_retreat_target(self, obs: Observation) -> Tuple[Optional[Point], bool]:
        """Nearest free cell to back into, backward cone preferred.

        The search itself lives in controller/retreat.py, shared with
        NavigationController so the two cannot drift into backing off
        differently. This supplies the wavefront and the push block's numbers.
        """
        return find_retreat_target(
            wavefront=self._build_wavefront(obs),
            robot_xy=(obs.robot_x, obs.robot_y),
            heading_deg=obs.robot_theta,
            cone_half_angle_deg=self._push_config.retreat_cone_angle,
            min_dist_cm=self._push_config.retreat_min_dist,
            max_dist_cm=self._push_config.retreat_max_dist,
        )

    def _reverse_toward(self, obs: Observation, target: Point) -> Action:
        """Reverse toward target with steering correction. See retreat.py."""
        return reverse_toward(
            obs, target,
            speed=self._push_config.retreat_speed,
            steer_gain=self._push_config.retreat_steer_gain,
        )

    def _blind_retreat(self, obs: Observation) -> Action:
        """Fallback: blind reverse for fixed number of steps."""
        if self._retreat_step_count >= self._push_config.retreat_steps:
            self._report_retreat_outcome(obs, self.RETREAT_BLIND_DONE)
            self._state = PushState.FINISHED
            return Action.stop()

        self._retreat_step_count += 1
        retreat_speed = -self._push_config.retreat_speed
        action = Action(left_speed=retreat_speed, right_speed=retreat_speed)

        # The blind reverse is the one motion nothing validated. The smart
        # retreat aims at a wavefront-free cell; this one just backs up.
        violation = self._safety_check(obs)
        if violation is not None:
            print(
                f"[PUSH] !! CLEARANCE: blind retreat {violation.distance_now_cm:.2f}cm "
                f"from '{violation.static_name}', stopping."
            )
            self._report_retreat_outcome(obs, self.RETREAT_CLEARANCE_STOP)
            self._state = PushState.FINISHED
            return Action.stop()
        return action

    # Every way the retreat can end. All five set PushState.FINISHED, which is
    # why the outcome has to be said out loud: downstream sees one state.
    RETREAT_ARRIVED = "reached target"
    RETREAT_TIMEOUT = "TIMEOUT"
    RETREAT_BLIND_DONE = "blind reverse ended"
    RETREAT_NAV_DONE = "navigation reported done"
    RETREAT_CLEARANCE_STOP = "stopped by the safety filter"

    def _report_retreat_outcome(self, obs: Observation, outcome: str) -> None:
        """Say how far the robot actually travelled, not only that it stopped.

        Displacement is the number that separates a clean retreat from one that
        left the robot against the object it just pushed. Across the real runs
        under closed_loop_sessions/, arriving retreats moved 3.0 cm while
        timing-out ones moved 0.2-0.5 cm despite commanding a clean reverse on
        all 199 ticks.

        The blind fallback has no target, because it runs precisely when the
        wavefront search found no free cell to aim at. It still reports, since
        that path ending silently is how a push with nowhere to retreat to looks
        exactly like a good one.
        """
        moved = 0.0
        if self._retreat_start_pose is not None:
            moved = math.hypot(
                obs.robot_x - self._retreat_start_pose[0],
                obs.robot_y - self._retreat_start_pose[1],
            )
        if self._retreat_target is None:
            where = (
                f"no retreat target (commanded speed "
                f"{self._push_config.retreat_speed:.2f})"
            )
            target = ""
        else:
            dist = math.hypot(
                obs.robot_x - self._retreat_target[0],
                obs.robot_y - self._retreat_target[1],
            )
            where = (
                f"stopped {dist:.1f} cm from target "
                f"(tolerance {self._push_config.retreat_tolerance:.1f} cm, "
                f"commanded speed {self._push_config.retreat_speed:.2f})"
            )
            target = (
                f", target ({self._retreat_target[0]:.1f}, "
                f"{self._retreat_target[1]:.1f})"
            )
        print(
            f"[PUSH] Retreat {outcome} after {self._retreat_step_count} steps: "
            f"moved {moved:.1f} cm, {where}, "
            f"robot at ({obs.robot_x:.1f}, {obs.robot_y:.1f}){target}"
        )

    def _handle_retreating(self, obs: Observation) -> Action:
        """Back up to nearest free cell after pushing.

        Smart retreat algorithm:
        1. On first call, find retreat target using dual-cone wavefront search
        2. Prefer backward cone (no rotation needed) → custom reverse steering
        3. Forward cone → use NavigationController (already facing forward)
        4. Fall back to blind reverse if no free cell found
        """
        # On first call, find retreat target
        if self._retreat_target is None:
            # Recorded before the search, so the blind fallback below can still
            # report displacement. It has no target to measure against, which is
            # exactly when knowing whether the robot moved matters most.
            if self._retreat_start_pose is None:
                self._retreat_start_pose = (obs.robot_x, obs.robot_y)
            target, is_backward = self._find_retreat_target(obs)
            if target is None:
                # No free cell found - fall back to blind reverse
                print("[PUSH] No free cell found in cones, using blind reverse")
                return self._blind_retreat(obs)

            self._retreat_target = target
            self._retreat_is_backward = is_backward
            direction = "BACKWARD" if is_backward else "FORWARD"
            print(f"[PUSH] Retreat target: ({target[0]:.1f}, {target[1]:.1f}) [{direction}]")

            # If forward target and we have nav controller, start navigation
            if not is_backward and self._nav_controller is not None:
                robot_pos = (obs.robot_x, obs.robot_y)
                # Build obstacles (include all objects)
                obstacles = self._build_obstacles(obs)
                # Navigate to retreat target (keep current orientation)
                success = self._nav_controller.navigate_to(
                    target[0], target[1],
                    obs.robot_theta,  # Keep current heading
                    robot_pos,
                    obstacles,
                )
                if not success:
                    print("[PUSH] Navigation to forward retreat failed, using blind reverse")
                    self._retreat_is_backward = True  # Fall back to reverse

        # Check if we've reached the target
        dist = math.hypot(
            obs.robot_x - self._retreat_target[0],
            obs.robot_y - self._retreat_target[1],
        )
        if dist < self._push_config.retreat_tolerance:
            self._report_retreat_outcome(obs, self.RETREAT_ARRIVED)
            self._state = PushState.FINISHED
            if self._nav_controller is not None:
                self._nav_controller.cancel()
            return Action.stop()

        # Safety: don't retreat forever (max 2x fallback steps)
        self._retreat_step_count += 1
        if self._retreat_step_count >= self._push_config.retreat_steps * 2:
            self._report_retreat_outcome(obs, self.RETREAT_TIMEOUT)
            self._state = PushState.FINISHED
            if self._nav_controller is not None:
                self._nav_controller.cancel()
            return Action.stop()

        # Move toward target
        if self._retreat_is_backward:
            return self._reverse_toward(obs, self._retreat_target)
        else:
            # Forward target - use navigation controller
            if self._nav_controller is not None:
                if self._nav_controller.is_done(obs, None):
                    self._report_retreat_outcome(obs, self.RETREAT_NAV_DONE)
                    self._state = PushState.FINISHED
                    return Action.stop()
                return self._nav_controller.step(obs, None)
            else:
                # No nav controller - fall back to reverse
                return self._reverse_toward(obs, self._retreat_target)

    def _get_edge_point(self, obj: ObjectPose, edge_idx: int) -> EdgePoint:
        """Get edge point using namo_cpp-compatible algorithm.

        Args:
            obj: Object pose
            edge_idx: Edge index (0 to 4*points_per_face - 1)

        Returns:
            EdgePoint with position, mid_point, and approach_theta
        """
        return get_edge_point(
            obj, edge_idx, self._standoff_distance, self._points_per_face
        )

    def _compute_approach_position(
        self, obj: ObjectPose, edge_idx: int
    ) -> Point:
        """
        Compute standoff position outside the object face.

        Uses namo_cpp-compatible edge point generation.

        Args:
            obj: Object pose
            edge_idx: Edge index (0 to 4*points_per_face - 1)

        Returns:
            (x, y) approach position in cm
        """
        return self._get_edge_point(obj, edge_idx).position

    def _compute_approach_orientation(self, obj: ObjectPose, edge_idx: int) -> float:
        """
        Compute robot orientation for approach (facing push direction).

        Uses namo_cpp-compatible edge point generation.

        Args:
            obj: Object pose
            edge_idx: Edge index (0 to 4*points_per_face - 1)

        Returns:
            Target orientation in degrees
        """
        return self._get_edge_point(obj, edge_idx).approach_theta

    def _build_obstacles(
        self, obs: Observation, exclude_id: Optional[str] = None
    ) -> List[Tuple[float, float, float, float, float]]:
        """Build obstacle list for navigation."""
        obstacles = []
        for obj_id, obj in obs.objects.items():
            if obj_id == exclude_id:
                continue
            if obj.width > 0 and obj.depth > 0:
                obstacles.append((obj.x, obj.y, obj.theta, obj.width, obj.depth))
        return obstacles

    def _build_wavefront(self, obs: Observation) -> WavefrontPlanner:
        """Build wavefront from current observation.

        Match planner/sim exactly: 5 mm grid, same robot radius, and tier-1
        base inflation only. Do not add a runtime-only push margin here, or
        the real controller will reject approach cells that planner/sim accepts.
        """
        # Effective robot inflation radius (meters).
        # See robot_geometry.effective_robot_radius_m_from_cm for the formula.
        robot_radius_m = effective_robot_radius_m_from_cm(
            self._config.car_width,
            self._config.car_height,
        )
        inflation_cfg = get_wavefront_inflation_config()
        inflation_margin_m = (
            inflation_cfg.tier1_base_inflation_margin_m
            + inflation_cfg.push_approach_additional_margin_m
        )

        wavefront = WavefrontPlanner(WavefrontConfig(
            resolution=0.005,           # 5mm
            robot_radius=robot_radius_m,
            inflation_margin=inflation_margin_m,
        ))

        # Bounds in meters
        bounds = (
            0.0,
            self._config.width / 100.0,
            0.0,
            self._config.height / 100.0,
        )

        # Objects in wavefront format: (x_m, y_m, half_x, half_y, theta_deg)
        # Convention from objects.yaml: depth=X (along heading), width=Y (perpendicular)
        objects = {}
        for obj_id, o in obs.objects.items():
            if o.width > 0 and o.depth > 0:
                objects[obj_id] = (
                    o.x / 100.0,      # x in meters
                    o.y / 100.0,      # y in meters
                    o.depth / 200.0,  # half-depth = X extent in meters
                    o.width / 200.0,  # half-width = Y extent in meters
                    o.theta,          # theta in degrees
                )

        wavefront.build_grid(bounds, objects)
        return wavefront

    def _validate_approach_position(
        self, obs: Observation, edge_pt: Point, mid_pt: Point
    ) -> Optional[Point]:
        """Validate approach position along the push line.

        Algorithm (from PUSH_GEOMETRY.md):
        1. Check edge_pt first (ideal position)
        2. If blocked, sample along push line away from object
        3. Use wavefront granularity (~0.5cm)
        4. Return first free position found (nearest to object)
        5. Max search distance 15cm

        Args:
            obs: Current observation (for building wavefront)
            edge_pt: The edge point (robot approach position) in cm
            mid_pt: The mid point (on object centerline) in cm

        Returns:
            Valid approach position in cm, or None if no free position found
        """
        wavefront = self._build_wavefront(obs)

        # Direction: mid_pt → edge_pt (toward approach side, away from object)
        dx = edge_pt[0] - mid_pt[0]
        dy = edge_pt[1] - mid_pt[1]
        length = math.hypot(dx, dy)

        if length < 1e-6:
            # Degenerate case - mid_pt == edge_pt
            return None

        dir_x = dx / length
        dir_y = dy / length

        # Step 1: Check edge_pt first (ideal position)
        edge_m = (edge_pt[0] / 100.0, edge_pt[1] / 100.0)
        if wavefront.is_free(edge_m[0], edge_m[1]):
            return edge_pt

        # Step 2: Sample along push line, away from object
        step_cm = 0.5  # Match wavefront resolution (~5mm)
        max_dist_cm = 5.0  # Max search distance - keep small to avoid ending up on wrong side of nearby obstacles

        dist = step_cm
        while dist <= max_dist_cm:
            candidate_x = edge_pt[0] + dir_x * dist
            candidate_y = edge_pt[1] + dir_y * dist

            # Convert to meters for wavefront query
            if wavefront.is_free(candidate_x / 100.0, candidate_y / 100.0):
                return (candidate_x, candidate_y)

            dist += step_cm

        # No free position found along push line
        return None

    def _compute_push_target(
        self, obj: ObjectPose, edge_idx: int
    ) -> Point:
        """
        Compute target point to push toward.

        Uses the mid_point from edge point generation, which represents the
        push direction target. The robot pushes from its edge position toward
        the mid_point, which is the center between the edge point and its mate
        on the opposite face.

        Args:
            obj: Object pose
            edge_idx: Edge index (0 to 4*points_per_face - 1)

        Returns:
            (x, y) push target position in cm
        """
        return self._get_edge_point(obj, edge_idx).mid_point

    def _pure_pursuit(self, obs: Observation, target: Point) -> Action:
        """
        Steer toward target using pure pursuit.

        Returns differential drive action.
        """
        robot_x = obs.robot_x
        robot_y = obs.robot_y
        robot_theta_rad = math.radians(obs.robot_theta)

        # Vector to target
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        distance = math.hypot(dx, dy)

        if distance < 1e-6:
            return Action.stop()

        # Angle to target
        angle_to_target = math.atan2(dy, dx)
        heading_error = _wrap_to_pi(angle_to_target - robot_theta_rad)

        # Lookahead distance (based on distance to target).
        # See robot_geometry.effective_robot_size_cm for the formula.
        car_size = effective_robot_size_cm(self._config.car_width, self._config.car_height)
        lookahead = max(distance, self._push_config.lookahead_ratio * car_size)

        # Pure pursuit curvature
        curvature = 2.0 * math.sin(heading_error) / lookahead

        # Speed scheduling - slow down for large heading errors
        speed_factor = max(0.4, math.cos(heading_error))
        base_speed = self._max_speed * speed_factor

        # Convert curvature to differential drive
        wheel_base = self._config.wheel_base
        diff = curvature * wheel_base / 2.0

        left_speed = base_speed * (1.0 - diff)
        right_speed = base_speed * (1.0 + diff)

        # Normalize to max speed
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        # Apply deadband
        left_speed, right_speed = _enforce_deadband_scale(
            left_speed, right_speed, self._push_config.wheel_deadband
        )

        return Action(
            left_speed=_clamp(left_speed, -1.0, 1.0),
            right_speed=_clamp(right_speed, -1.0, 1.0),
        )

    def is_done(self, obs: Observation, subgoal: Subgoal) -> bool:
        """Check if push is complete (after push + retreat) or failed."""
        if not isinstance(subgoal, PushSubgoal):
            return True

        if self._state == PushState.FAILED:
            return True

        if self._state == PushState.FINISHED:
            return True

        return False

    def did_fail(self) -> bool:
        """Check if push failed.

        Failure modes:
        - state == FAILED: approach unreachable, navigation planning failed, etc.
        - _push_movement_inadequate: push completed but object barely moved
          (below min_push_displacement_cm AND min_push_rotation_deg thresholds).
          The robot still retreats normally, but did_fail returns True so the
          upstream planner blacklists (object_id, edge_idx).
        """
        return (
            self._state == PushState.FAILED
            or self._push_movement_inadequate
            or self._robot_collision_abort
        )

    def get_last_push_summary(self, post_obs: "Observation") -> Optional[Dict[str, object]]:
        """Return a structured summary of the most recently completed push,
        or None if no push has run yet / required state is missing.

        Used by the diagnostics pipeline to write one record per physical
        push (pushes.jsonl). Reads from already-tracked controller state —
        does not duplicate the inadequate-movement detection logic.
        """
        sub = self._current_subgoal
        if sub is None or self._push_start_obj_pose is None:
            return None
        obj_now = post_obs.objects.get(getattr(sub, "object_id", None)) if post_obs else None
        if obj_now is None:
            return None
        dx = obj_now.x - self._push_start_obj_pose[0]
        dy = obj_now.y - self._push_start_obj_pose[1]
        dtheta_deg = obj_now.theta - self._push_start_obj_pose[2]
        # Wrap dtheta to [-180, 180] for stability.
        dtheta_deg = ((dtheta_deg + 180.0) % 360.0) - 180.0
        disp = math.hypot(dx, dy)
        path_unit_vec = None
        path_length_cm = None
        if self._push_path_at_start and len(self._push_path_at_start) >= 2:
            sx, sy = self._push_path_at_start[0]
            ex, ey = self._push_path_at_start[-1]
            path_len = math.hypot(ex - sx, ey - sy)
            if path_len > 1e-9:
                path_unit_vec = [(ex - sx) / path_len, (ey - sy) / path_len]
                path_length_cm = path_len
        return {
            "object_id": getattr(sub, "object_id", None),
            "expected_edge": getattr(sub, "edge_idx", None),
            "expected_push_steps": getattr(sub, "push_steps", None),
            "object_pose_before": list(self._push_start_obj_pose),
            "object_pose_after": [obj_now.x, obj_now.y, obj_now.theta],
            "robot_pose_before_cm_deg": (
                list(self._push_start_robot_pose) if self._push_start_robot_pose is not None else None
            ),
            "robot_pose_after_cm_deg": [post_obs.robot_x, post_obs.robot_y, post_obs.robot_theta],
            "push_start_obs_timestamp": self._push_start_obs_timestamp,
            "push_end_obs_timestamp": (
                float(post_obs.timestamp) if getattr(post_obs, "timestamp", None) is not None else None
            ),
            "push_path_cm": (
                [list(pt) for pt in self._push_path_at_start]
                if self._push_path_at_start is not None
                else None
            ),
            "push_target_cm": (
                list(self._target_point_at_start)
                if self._target_point_at_start is not None
                else None
            ),
            "push_path_unit_vec": path_unit_vec,
            "push_path_length_cm": path_length_cm,
            "push_controller_max_speed": float(self._max_speed),
            "push_lookahead_distance_cm": float(self._follow_path_controller.lookahead_distance),
            "push_dynamic_direction": bool(self._dynamic_direction),
            "push_ticks_executed": int(self._step_count),
            "delta_pos_cm": [dx, dy],
            "delta_pos_magnitude_cm": disp,
            "delta_theta_deg": dtheta_deg,
            "stuck": bool(self._push_movement_inadequate),
            "stuck_threshold_cm": self._push_config.min_push_displacement_cm,
            "stuck_threshold_deg": self._push_config.min_push_rotation_deg,
            "safety_filter": self._safety_filter is not None,
            "abort_reason": (
                self._abort_violation.reason if self._abort_violation is not None else None
            ),
            "abort_distance_now_cm": (
                self._abort_violation.distance_now_cm
                if self._abort_violation is not None else None
            ),
            "abort_distance_at_start_cm": (
                self._abort_violation.distance_at_start_cm
                if self._abort_violation is not None else None
            ),
        }

    def _print_wavefront_status(self, obs: Observation, show: bool = False) -> None:
        """Print wavefront status at end of push for debugging.

        Args:
            obs: Current observation
            show: If True, display wavefront image in a window
        """
        import time
        from pathlib import Path

        print(f"\n[PUSH] ========== PUSH COMPLETE ==========")
        print(f"[PUSH] Robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) theta={obs.robot_theta:.1f}°")

        # Object displacement summary (vs. PUSHING start pose)
        sub = self._current_subgoal
        if sub is not None and self._push_start_obj_pose is not None:
            obj_now = obs.objects.get(sub.object_id) if hasattr(sub, "object_id") else None
            if obj_now is not None:
                dx = obj_now.x - self._push_start_obj_pose[0]
                dy = obj_now.y - self._push_start_obj_pose[1]
                dtheta = _wrap_to_pi(math.radians(obj_now.theta - self._push_start_obj_pose[2]))
                dtheta_deg = math.degrees(dtheta)
                displacement_cm = math.hypot(dx, dy)
                print(
                    f"[PUSH] Δobject '{sub.object_id}': "
                    f"Δpos=({dx:+.2f},{dy:+.2f}) |Δ|={displacement_cm:.2f}cm, "
                    f"Δθ={dtheta_deg:+.2f}°"
                )
                # Stuck detection: mark push as FAILED (for upstream blacklist) if
                # the object moved less than the configured thresholds. Robot still
                # retreats normally to clear the area; did_fail() returns True so
                # notify_subgoal_done(failed=True) → blacklist entry added.
                min_disp = self._push_config.min_push_displacement_cm
                min_rot = self._push_config.min_push_rotation_deg
                if displacement_cm < min_disp and abs(dtheta_deg) < min_rot:
                    print(
                        f"[PUSH] !! STUCK: object moved {displacement_cm:.2f}cm "
                        f"(<{min_disp}cm) AND rotated {dtheta_deg:.2f}° "
                        f"(<{min_rot}°). Marking subgoal as FAILED for blacklist."
                    )
                    self._push_movement_inadequate = True

        # Build wavefront and check positions
        wavefront = self._build_wavefront(obs)
        robot_m = (obs.robot_x / 100.0, obs.robot_y / 100.0)
        robot_free = wavefront.is_free(robot_m[0], robot_m[1])
        print(f"[PUSH] Robot position is {'FREE' if robot_free else 'BLOCKED'} in wavefront")

        # Goal position
        goal_m = None
        if obs.goal_x is not None and obs.goal_y is not None:
            goal_m = (obs.goal_x / 100.0, obs.goal_y / 100.0)
            goal_free = wavefront.is_free(goal_m[0], goal_m[1])
            print(f"[PUSH] Goal: ({obs.goal_x:.1f}, {obs.goal_y:.1f}) - {'FREE' if goal_free else 'BLOCKED'}")

        # Show object positions
        for name, obj in obs.objects.items():
            print(f"[PUSH] Object '{name}': ({obj.x:.1f}, {obj.y:.1f}) theta={obj.theta:.1f}° size=({obj.width}x{obj.depth})")

            # Check distance from robot to object center
            dist = math.hypot(obs.robot_x - obj.x, obs.robot_y - obj.y)
            print(f"[PUSH]   Distance robot→object center: {dist:.1f}cm")

        # Save wavefront grid image
        output_dir = Path("/tmp/wavefront")
        output_dir.mkdir(exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filepath = output_dir / f"wavefront_{timestamp}.png"
        wavefront.save(str(filepath), robot_pos=robot_m, goal_pos=goal_m, show=show)
        print(f"[PUSH] Wavefront saved to: {filepath}")

        print(f"[PUSH] ======================================\n")

    def reset(self) -> None:
        """Reset controller state."""
        self._state = PushState.IDLE
        self._step_count = 0
        self._advance_step_count = 0
        self._retreat_step_count = 0
        self._approach_step_count = 0
        self._current_subgoal = None
        self._approach_position = None
        self._approach_orientation = None
        self._target_point = None
        self._target_point_at_start = None
        self._object_pose = None
        self._push_path = None
        self._push_path_at_start = None

        # Reset retreat state
        self._retreat_target = None
        self._retreat_is_backward = False
        self._retreat_start_pose = None

        # Reset push outcome telemetry
        self._push_start_obj_pose = None
        self._push_start_robot_pose = None
        self._push_start_obs_timestamp = None
        self._push_movement_inadequate = False

        # Reset safety filter outcome (the filter itself outlives the push)
        self._abort_violation = None
        self._robot_collision_abort = False
        self._clearance_at_start = None
        self._last_outline = None
        self._last_check_violated = False

        # Reset follow path controller
        self._follow_path_controller.reset()

        # Also reset nav controller if we have one
        if self._nav_controller is not None:
            self._nav_controller.cancel()

    def cancel(self) -> None:
        """Cancel current push operation."""
        self.reset()

    def get_status(self) -> str:
        """Get current status label."""
        if self._state == PushState.PUSHING:
            if self._current_subgoal:
                total_ticks = self._current_subgoal.push_steps * self._push_config.push_steps
                return f"PUSHING ({self._step_count}/{total_ticks})"
            return f"PUSHING ({self._step_count})"
        if self._state == PushState.ADVANCING:
            return f"ADVANCING ({self._advance_step_count}/{self._push_config.advance_steps})"
        if self._state == PushState.RETREATING:
            suffix = " CLEARANCE ABORT" if self._abort_violation is not None else ""
            if self._retreat_target:
                return f"RETREATING (step {self._retreat_step_count}){suffix}"
            return f"RETREATING ({self._retreat_step_count}/{self._push_config.retreat_steps}){suffix}"
        if self._state == PushState.APPROACHING:
            return (
                f"APPROACHING ({self._approach_step_count}/"
                f"{self._push_config.max_approach_steps})"
            )
        if self._state == PushState.FAILED:
            return "PUSH FAILED"
        return self._state.value

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Get drawing commands for visualization."""
        drawings = []

        # Draw approach position (cyan circle) during approach phase
        if self._approach_position and self._state in (
            PushState.COMPUTING_APPROACH,
            PushState.APPROACHING,
        ):
            drawings.append({
                "uuid": "approach_position",
                "type": "point",
                "position": self._approach_position,
                "radius": 6,
                "color": "#00FFFF",  # Cyan
                "fill": "#00FFFF",
            })

            # Draw arrow showing intended push direction (edge_point to mid_point)
            if self._object_pose and self._current_subgoal:
                ep = self._get_edge_point(self._object_pose, self._current_subgoal.edge_idx)
                drawings.append({
                    "uuid": "approach_direction",
                    "type": "path",
                    "points": [self._approach_position, ep.mid_point],
                    "color": "#00FFFF",
                    "width": 2,
                })

        # Draw navigation path during approach (delegate to nav controller)
        if self._state == PushState.APPROACHING and self._nav_controller is not None:
            drawings.extend(self._nav_controller.get_drawings())

        # Draw target point during pushing
        if self._target_point and self._state == PushState.PUSHING:
            drawings.append({
                "uuid": "push_target",
                "type": "point",
                "position": self._target_point,
                "radius": 4,
                "color": "#FF00FF",  # Magenta
                "fill": "#FF00FF",
            })

        # Draw push direction arrow from edge_point to mid_point (target)
        # _approach_position is the edge_point
        if self._approach_position and self._target_point and self._state == PushState.PUSHING:
            drawings.append({
                "uuid": "push_direction",
                "type": "path",
                "points": [self._approach_position, self._target_point],
                "color": "#FF00FF",
                "width": 2,
            })

        # Draw retreat target during retreating phase
        if self._retreat_target and self._state == PushState.RETREATING:
            color = "#FFA500" if self._retreat_is_backward else "#FF6600"  # Orange shades
            drawings.append({
                "uuid": "retreat_target",
                "type": "point",
                "position": self._retreat_target,
                "radius": 5,
                "color": color,
                "fill": color,
            })

        # Always show all edge points when we have an object (for debugging)
        if self._object_pose:
            drawings.extend(self.get_all_edge_drawings(self._object_pose))

        # Safety filter: the walls it knows and the outline it last checked,
        # red once the check has fired.
        if self._safety_filter is not None:
            for box in self._safety_filter.statics:
                corners = box.corners()
                drawings.append({
                    "uuid": f"safety_static_{box.name}",
                    "type": "path",
                    "points": corners + [corners[0]],
                    "color": "#FF4040" if self._last_check_violated else "#808080",
                    "width": 1,
                })
            if self._last_outline:
                drawings.append({
                    "uuid": "safety_robot_outline",
                    "type": "path",
                    "points": self._last_outline + [self._last_outline[0]],
                    "color": "#FF0000" if self._last_check_violated else "#00C000",
                    "width": 2,
                })

        return drawings

    def get_all_edge_drawings(self, obj: ObjectPose) -> List[Dict[str, Any]]:
        """Get drawing commands for all edge points (for debugging).

        Args:
            obj: Object to show edge points for

        Returns:
            List of drawing commands for visualization
        """
        drawings: List[Dict[str, Any]] = []
        all_points = generate_edge_points(
            obj, self._standoff_distance, self._points_per_face
        )

        # Colors for different faces
        face_colors = {
            0: "#00FF00",  # Green - Top (+Y)
            1: "#FF0000",  # Red - Bottom (-Y)
            2: "#0000FF",  # Blue - Right (+X)
            3: "#FFFF00",  # Yellow - Left (-X)
        }

        for ep in all_points:
            color = face_colors.get(ep.face_idx, "#888888")
            # Draw edge point
            drawings.append({
                "uuid": f"edge_{ep.edge_idx}",
                "type": "point",
                "position": ep.position,
                "radius": 3,
                "color": color,
                "fill": color,
            })
            # Draw arrow to mid-point (push direction)
            drawings.append({
                "uuid": f"edge_{ep.edge_idx}_dir",
                "type": "path",
                "points": [ep.position, ep.mid_point],
                "color": color,
                "width": 1,
            })

        return drawings
