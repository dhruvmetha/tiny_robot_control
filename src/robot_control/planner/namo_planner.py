"""NAMO planner for robot_control.

Implements the Planner interface for NAMO push manipulation,
integrating with namo_cpp planning via NAMOPlanBridge.

Now includes reachability checking:
1. Before NAMO planning: Check if goal is already reachable
2. After push execution: Re-check if goal is now reachable
3. Loop until goal is reachable, then navigate to goal
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Sequence, Set, Tuple

from robot_control.core.types import NavigateSubgoal, Observation, PushSubgoal, Subgoal
from robot_control.planner.base import Planner
# The opening bar a region target is chosen under. Mirrors namo_cpp's
# CANONICAL_MIN_REACHABLE_FRACTION: a boundary counts as open when 20% of its
# sampled points are reachable. Recorded on the target so a later config change
# cannot silently re-grade a subproblem already in flight.
CANONICAL_OPEN_FRACTION = 0.2

# What counts as an object having moved, for deciding whether its blacklisted
# edges are still meaningful. Matches the tolerances closed_loop_session uses to
# match objects between a scene and its XML, which is the repo's existing
# statement of "the same object, unmoved" against real camera measurements.
from robot_control.planner.region_target import (
    objects_that_moved,
    ADVANCE_EXHAUSTED,
    ADVANCE_PLANNED,
    MAX_BOUNDARY_ADVANCES,
    STATUS_EXHAUSTED,
    STATUS_OPENED,
    RegionOpeningTarget,
    advance_boundary,
)
from robot_control.planner.search_config import LocalSearchConfig
from robot_control.planner.namo_bridge import NAMOPlanBridge
from robot_control.planner.reachability_filter import (
    unreachable_contact_points,
)
from robot_control.planner.goal_retarget import find_goal_retarget

# Farthest a nearest-free-cell retarget may sit from the goal point before
# the planner gives up on retargeting and runs a NAMO push instead. Half the
# largest movable object's long side (15/2 = 7.5) + robot inflation radius
# (3.5) + wavefront tier1 margin (0.5) = 11.5, rounded up. That sum is the
# maximum distance the pushed object itself can put between the goal point
# and free space -- beyond it, something other than the pushed object is
# blocking the goal, so retargeting would only paper over that.
#
# This value and the conditions under which a retarget fires are a MEASURED
# QUANTITY, not only an implementation choice. The ICRA real-robot study
# reports strict-marker and with-retarget success side by side, and the gap
# between those two columns is exactly the behaviour below. Changing the cap,
# or changing when a retarget is attempted, changes what the study reports
# rather than only what the robot does. Treat an edit here as a change to the
# pre-registration and see docs/ICRA_REAL_ROBOT_STUDY.md before making one.
GOAL_RETARGET_CAP_CM = 12.0


# Keys from the planner's algorithm_stats dict that are safe to serialize to
# JSON for the diagnostics recorder. Excludes raw fields like `attempt_results`
# and `all_solutions` which carry C++ Action objects that can't be JSON-encoded.
_DIAG_SAFE_STAT_KEYS = frozenset({
    "total_primitives_attempted",
    "rejection_breakdown",
    "successful_openings",
    "total_attempts",
    "iterations",
    "total_pushes",
    "regions_opened",
    # A whitelist drops anything it does not name, so a scalar the bridge sets
    # and nobody adds here reaches no log at all. Which decision rule produced
    # the push is one of those, and for a crossed trial matrix a row without it
    # is a row whose arm has to be recovered from a command string.
    "exec_mode",
    "resolved_target",
    "simulations_used",
})


def _filter_algorithm_stats_for_diagnostics(stats: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Return JSON-safe stats with one simulation-count field on every path."""
    if not stats:
        return {}
    out: Dict[str, Any] = {}
    for k, v in stats.items():
        if k not in _DIAG_SAFE_STAT_KEYS:
            continue
        # Coerce common non-serializable types defensively.
        if isinstance(v, (list, tuple)):
            out[k] = [str(x) if not isinstance(x, (str, int, float, bool, type(None))) else x for x in v]
        else:
            out[k] = v

    # Held-boundary planning already exports ``simulations_used``. Whole-
    # problem full_namo reports the same env.step count under its budget-scope
    # keys instead. Normalize those spellings so changing planner paths does
    # not silently change the paper metric's schema.
    if "simulations_used" not in out:
        for key in (
            "simulation_budget_used_total",
            "simulation_budget_used",
            "total_primitives_attempted",
        ):
            raw_count = stats.get(key)
            if raw_count is None:
                continue
            try:
                out["simulations_used"] = int(raw_count)
            except (TypeError, ValueError):
                continue
            break
    return out


class NAMOPlanner(Planner):
    """Planner that uses NAMO for push manipulation planning.

    Generates a sequence of PushSubgoals by:
    1. Capturing current state to MuJoCo XML
    2. Running NAMO planning (RegionOpeningPlanner)
    3. Converting results to PushSubgoals

    The planner maintains a queue of subgoals and can optionally
    replan when the queue is exhausted.

    Usage:
        planner = NAMOPlanner(
            robot_goal_cm=(50.0, 40.0),
            namo_config_path="namo_cpp/config/namo_config_complete_skill15_car_1x.yaml",
        )

        # In runtime loop:
        subgoal = planner.plan(observation)
        # ... execute subgoal ...
        planner.notify_subgoal_done(observation)
    """

    def __init__(
        self,
        robot_goal_cm: Tuple[float, float],
        namo_config_path: str,
        algorithm: str = "full_namo",
        execution_mode: Literal["open_loop", "mpc"] = "mpc",
        goal_strategy: str = "primitive",
        scale_factor: float = 1.0,
        primitive_data_dir: str = "data",
        replan_on_completion: bool = True,
        max_chain_depth: int = 2,
        frontier_beam_width: int = 10000,
        chain_link_cost: int = 11,
        selection_strategy: str = "cost_first",
        goals_per_region: Optional[int] = None,
        shuffle_edges: bool = True,
        shuffle_seed: Optional[int] = None,
        rollout_samples_per_state: Optional[int] = None,
        verbose: bool = False,
        debug_xml_path: Optional[str] = None,
        enable_viewer: bool = False,
        pause_after_load: bool = False,
        # ML goal model
        ml_goal_model_path: Optional[str] = None,
        ml_device: str = "cuda",
        ml_samples: Optional[int] = None,
        ml_num_steps: Optional[int] = None,
        ml_sampler_method: Optional[str] = None,
        # Which local search namo_cpp runs per region boundary.
        local_search: Optional[LocalSearchConfig] = None,
        # Hold one region boundary across pushes instead of re-choosing
        # every replan. Off by default: unchanged runs keep the whole-problem
        # planning path exactly as it was.
        hold_region_target: bool = False,
        active_target_path: Optional[str] = None,
        # Retry budgets — see [Runtime] section in run_namo.py docstring.
        max_planning_retries: int = 5,
        max_replan_attempts: int = 20,
        # Workspace config for reachability checking (must match navigation planner)
        workspace_width_cm: float = 70.0,
        workspace_height_cm: float = 55.0,
        show_push_scores: bool = False,
        robot_width_cm: float = 7.0,
        robot_height_cm: float = 7.0,
        # Robot body model for the planning simulator
        robot_model: str = "car",
        # Path to manual primitive sequence YAML, only meaningful when
        # goal_strategy == "manual_primitives".
        manual_primitives_file: Optional[str] = None,
    ):
        """Initialize the NAMO planner.

        Args:
            robot_goal_cm: Goal position in cm (x, y)
            namo_config_path: Path to NAMO config YAML
            algorithm: Planning algorithm ("full_namo" or "region_opening")
            execution_mode: Plan-level execution strategy. "mpc" (default)
                queues only the first push from each planning call and replans
                after each push completes — closed-loop at the plan level,
                robust to real-world drift. "open_loop" queues the entire
                planned sequence and only replans when the queue empties
                without reaching the goal — fewer planning calls, but
                commits to the simulator's predicted future across pushes.
            goal_strategy: Goal sampling strategy ("primitive", "ml", etc.)
            scale_factor: Fixed at 1.0.
            primitive_data_dir: Directory containing motion primitive data
            replan_on_completion: If True, replan when subgoal queue is empty
            max_chain_depth: Maximum chain depth for multi-push solutions (1 or 2)
            frontier_beam_width: Beam width for frontier search
            chain_link_cost: Additional cost per chain link
            selection_strategy: Frontier priority ("cost_first" or "ml_first")
            goals_per_region: Goal samples per region for validation.
                None defers to namo_cpp's canonical 100.
            shuffle_edges: Randomize edge ordering during planning (default True)
            shuffle_seed: Random seed for reproducible shuffling (None = random)
            verbose: Enable verbose logging
            debug_xml_path: If set, save generated XML to this path
            enable_viewer: Enable MuJoCo visualization window during planning
            pause_after_load: Pause after loading XML for interactive viewer inspection
            ml_goal_model_path: Path to trained ML goal model (for ml strategies)
            ml_device: PyTorch device for ML model ("cuda" or "cpu")
            local_search: Which local search namo_cpp runs per region
                boundary (region_bfs or best_first). Defaults to region_bfs
                and forwards nothing, so existing runs are unaffected.
            workspace_width_cm: Workspace width for reachability check (must match navigation)
            workspace_height_cm: Workspace height for reachability check (must match navigation)
            robot_width_cm: Robot width for reachability check
            robot_height_cm: Robot height for reachability check
        """
        if execution_mode not in ("open_loop", "mpc"):
            raise ValueError(
                f"execution_mode must be 'open_loop' or 'mpc', got {execution_mode!r}"
            )

        # The manual_primitives strategy returns the entire user-specified
        # chain in a single bridge.plan() call after sim-verifying it. If
        # the runtime were in MPC mode it'd discard everything but the first
        # push and replan — which immediately re-runs the same sim
        # verification, almost certainly with the same result (or worse,
        # if intermediate state drifted). Force open_loop so the chain
        # dispatches as a single block.
        if goal_strategy == "manual_primitives" and execution_mode != "open_loop":
            print(
                f"[NAMOPlanner] manual_primitives strategy requires open_loop "
                f"execution; overriding execution_mode {execution_mode!r} → 'open_loop'",
                flush=True,
            )
            execution_mode = "open_loop"

        if goal_strategy == "manual_primitives" and not manual_primitives_file:
            raise ValueError(
                "goal_strategy='manual_primitives' requires --manual-primitives-file"
            )

        self._robot_goal_cm = robot_goal_cm
        self._algorithm = algorithm
        self._execution_mode = execution_mode
        self._goal_strategy = goal_strategy
        self._manual_primitives_file = manual_primitives_file
        self._replan_on_completion = replan_on_completion
        self._max_chain_depth = max_chain_depth
        self._frontier_beam_width = frontier_beam_width
        self._chain_link_cost = chain_link_cost
        self._selection_strategy = selection_strategy
        self._goals_per_region = goals_per_region
        self._shuffle_edges = shuffle_edges
        self._shuffle_seed = shuffle_seed
        self._rollout_samples_per_state = rollout_samples_per_state
        self._ml_goal_model_path = ml_goal_model_path
        self._ml_device = ml_device
        self._local_search = local_search or LocalSearchConfig()
        self._scale_factor = float(scale_factor)
        self._hold_region_target = bool(hold_region_target)
        self._active_target_path = Path(active_target_path) if active_target_path else None
        self._active_target: Optional[RegionOpeningTarget] = None
        # Scene as it was when the current chain was committed, so a push's
        # side effects on other objects can be detected afterwards.
        self._observation_at_commit: Optional[Observation] = None
        self._ml_samples = ml_samples
        self._ml_num_steps = ml_num_steps
        self._ml_sampler_method = ml_sampler_method
        self._verbose = verbose

        # Create bridge for NAMO planning
        self._bridge = NAMOPlanBridge(
            namo_config_path=namo_config_path,
            scale_factor=scale_factor,
            primitive_data_dir=primitive_data_dir,
            verbose=verbose,
            debug_xml_path=debug_xml_path,
            enable_viewer=enable_viewer,
            pause_after_load=pause_after_load,
            show_push_scores=show_push_scores,
            robot_width_cm=robot_width_cm,
            robot_height_cm=robot_height_cm,
            robot_model=robot_model,
            manual_primitives_file=manual_primitives_file,
        )

        # Geometry the reachability filter rebuilds the navigator's grid from.
        # Kept rather than only forwarded, so the filter and the navigator
        # cannot drift onto different workspace or footprint numbers.
        self._workspace_width_cm = float(workspace_width_cm)
        self._workspace_height_cm = float(workspace_height_cm)
        self._robot_width_cm = float(robot_width_cm)
        self._robot_height_cm = float(robot_height_cm)

        # Cached summary from unified C++ reachability query
        self._last_reachability_summary: Optional[Dict[str, Any]] = None

        # Subgoal queue state
        self._subgoals: List[PushSubgoal] = []
        self._current_idx: int = 0
        self._plan_generated: bool = False
        self._planning_failed: bool = False
        self._committed_chain: List[PushSubgoal] = []
        self._committed_chain_origin: Optional[str] = None
        self._pending_reuse_chain: Optional[List[PushSubgoal]] = None
        self._pending_reuse_origin: Optional[str] = None

        # Reachability check state
        self._navigating_to_goal: bool = False
        self._goal_tolerance: float = 5.0  # cm - close enough to goal
        # Set when plan() retargets navigation to a free cell near the goal
        # point instead of the goal point itself (see _select_goal_retarget).
        # None means "navigating straight at self._robot_goal_cm".
        self._retarget_point_cm: Optional[Tuple[float, float]] = None

        # Cumulative planning time
        self._total_planning_ms: float = 0.0
        self._plan_count: int = 0

        # One-time model startup is measured separately from planning decisions.
        self._model_ranker_warmed: bool = False

        # Retry state for unreachable approach positions
        self._max_replan_attempts: int = int(max_replan_attempts)
        self._replan_attempt: int = 0  # Current replan attempt count
        self._failed_subgoal: Optional[PushSubgoal] = None  # Track which subgoal failed

        # Retry state for planning failures (no solution found)
        self._max_planning_retries: int = int(max_planning_retries)

        # Failed-push blacklist (failure feedback to the planner).
        # Each entry is (object_id, edge_idx) in real-world (robot_control)
        # naming. When a push fails, its (object_id, edge_idx) is added here
        # and forwarded to the bridge, which drops any planned push matching
        # a blacklisted pair before returning. Cleared on reset(); persists
        # across replans within a single planning episode.
        self._failed_pushes: Set[Tuple[str, int]] = set()

        # Diagnostics — wired from the Runtime via set_diagnostics_recorder().
        # When recorder is None, all hooks are no-ops.
        self._diag = None  # type: Optional[Any]
        self._latest_xml_path: Optional[str] = None  # set by dump_scene_xml()

        # Per-plan sim-success capture. When set by the Runtime (only when
        # --capture-sim-success is enabled), every plan() call that returns
        # at least one PushSubgoal invokes this callback with the bridge's
        # plan-time XML content and the push chain — the callback is then
        # responsible for writing artifacts and rendering MP4s. None means
        # no-op; the planner doesn't care what the callback does.
        self._sim_capture_callback = None  # type: Optional[Any]

    # -------------------------------------------------------------- diagnostics

    def set_diagnostics_recorder(self, recorder) -> None:
        """Attach a DiagnosticsRecorder so plan() calls emit record_plan."""
        self._diag = recorder

    def set_sim_capture_callback(self, callback) -> None:
        """Attach a callback invoked once per successful (non-empty) plan.

        Callback signature:
            callback(
                xml_content: str,
                push_chain: list[dict],
                attempt_index: int,
                starting_robot_pose_sim: tuple[float, float, float],
            ) -> None

        ``starting_robot_pose_sim`` is (x_m, y_m, theta_rad) in sim units —
        feed straight to env.set_robot_pose() in the replay env so the car
        body (which spawns at little_car.xml's fixed pos) is teleported to
        the captured planner state before pushes execute.

        Each ``push_chain`` entry has keys: object_id, edge_idx, push_steps, depth.
        Exceptions raised by the callback are caught and logged so a misbehaving
        sim-capture sink never breaks planning.
        """
        self._sim_capture_callback = callback

    def dump_scene_xml(self, obs: Observation, output_path) -> Optional[str]:
        """Generate and write the MuJoCo XML representation of the current
        observation to output_path WITHOUT running planning. Returns the
        written path on success, or None on failure.

        Used by the diagnostics pipeline to capture scene_before.xml and
        scene_after.xml — both fully reproducible scene snapshots compatible
        with namo_cpp's replay tooling.
        """
        try:
            xml_content = self._bridge._generate_xml(obs, self._robot_goal_cm)
        except Exception as exc:
            print(f"[NAMOPlanner] dump_scene_xml: _generate_xml failed: {exc!r}",
                  flush=True)
            return None
        if xml_content is None:
            return None
        try:
            output_path = str(output_path)
            with open(output_path, "w") as f:
                f.write(xml_content)
            self._latest_xml_path = output_path
            return output_path
        except Exception as exc:
            print(f"[NAMOPlanner] dump_scene_xml: write failed: {exc!r}",
                  flush=True)
            return None

    def _copy_push_chain(self, chain: List[PushSubgoal]) -> List[PushSubgoal]:
        """Return a defensive copy of a push chain."""
        return [
            PushSubgoal(
                object_id=str(sg.object_id),
                edge_idx=int(sg.edge_idx),
                push_steps=int(sg.push_steps),
            )
            for sg in chain
        ]

    def _queue_mpc_chain(
        self,
        obs: Observation,
        chain: List[PushSubgoal],
        *,
        origin: str,
        attempt_index: int,
        xml_content: Optional[str] = None,
        object_mapping: Optional[Dict[str, Dict[str, str]]] = None,
    ) -> None:
        """Preserve the full chain, but queue only its first push for MPC."""
        self._observation_at_commit = obs
        self._committed_chain = self._copy_push_chain(chain)
        self._committed_chain_origin = origin
        self._subgoals = [self._committed_chain[0]]
        self._current_idx = 0
        self._plan_generated = True
        self._pending_reuse_chain = None
        self._pending_reuse_origin = None

        # Exactly one physical push leaves here per commit, whatever produced
        # the chain, so this is where a held subproblem's counters advance.
        # with_push_attempted had no caller at all, which left
        # physical_pushes_attempted at 0 and last_iteration frozen at the
        # iteration the target was created in, for the whole life of the
        # subproblem. Both are read back from disk by the next process.
        if self._active_target is not None:
            self._store_active_target(
                self._active_target.with_push_attempted(self._plan_count)
            )

        if len(chain) > 1:
            print(
                f"[NAMOPlanner] MPC mode: preserving {len(chain)} pushes from {origin}, "
                f"queuing only the first for real execution"
            )

        if self._verbose:
            print(
                f"[NAMOPlanner] Queued {len(self._subgoals)} subgoal(s) "
                f"({self._execution_mode} mode, chain origin={origin}, "
                f"full_chain={len(chain)}):"
            )
            for i, sg in enumerate(self._subgoals):
                print(
                    f"  [{i}] {sg.object_id} edge={sg.edge_idx} "
                    f"steps={sg.push_steps}"
                )

        self._emit_sim_capture_for_plan(
            obs=obs,
            subgoals=chain,
            attempt_index=attempt_index,
            xml_content=xml_content,
            object_mapping=object_mapping,
        )

    def _emit_sim_capture_for_plan(
        self,
        *,
        obs: Observation,
        subgoals: List[PushSubgoal],
        attempt_index: int,
        xml_content: Optional[str] = None,
        object_mapping: Optional[Dict[str, Dict[str, str]]] = None,
    ) -> None:
        """Best-effort per-plan sim-success capture."""
        if self._sim_capture_callback is None or not subgoals:
            return

        push_chain: List[Dict[str, Any]] = []
        real_to_sim: Dict[str, str] = {}
        if isinstance(object_mapping, dict):
            maybe_real_to_sim = object_mapping.get("real_to_sim")
            if isinstance(maybe_real_to_sim, dict):
                real_to_sim = {str(k): str(v) for k, v in maybe_real_to_sim.items()}

        obj_map = getattr(self._bridge, "_object_mapping", None)
        for sg in subgoals:
            sim_object_id = real_to_sim.get(str(sg.object_id))
            if sim_object_id is None and obj_map is not None:
                sim_object_id = obj_map.get_sim_name(sg.object_id)
            if sim_object_id is None:
                sim_object_id = str(sg.object_id)
            push_chain.append({
                "object_id": sg.object_id,
                "sim_object_id": sim_object_id,
                "edge_idx": sg.edge_idx,
                "push_steps": sg.push_steps,
                "depth": sg.push_steps - 1,
            })

        if not push_chain:
            return

        try:
            xml_payload = xml_content if xml_content is not None else getattr(self._bridge, "last_xml_content", None)
            if xml_payload is None:
                return
            rx_m, ry_m = self._bridge._cm_to_sim(
                float(obs.robot_x), float(obs.robot_y)
            )
            rtheta_rad = math.radians(float(obs.robot_theta))
            self._sim_capture_callback(
                xml_content=xml_payload,
                push_chain=push_chain,
                attempt_index=attempt_index,
                starting_robot_pose_sim=(rx_m, ry_m, rtheta_rad),
            )
        except Exception as exc:
            print(f"[DIAG] sim-capture callback failed: {exc!r}", flush=True)

    def _record_reuse_diagnostics(
        self,
        *,
        obs: Observation,
        reuse_kind: str,
        verification_time_ms: float,
        planning_wall_time_ms: float,
        simulations_used: int,
        success: bool,
        chain: List[PushSubgoal],
        failed_step_index: Optional[int],
        failure_reason: Optional[str],
    ) -> None:
        """Best-effort diagnostics record for a reuse verification attempt."""
        if self._diag is None:
            return
        try:
            first_subgoal = None
            if chain:
                sg0 = chain[0]
                first_subgoal = {
                    "object_id": sg0.object_id,
                    "edge_idx": sg0.edge_idx,
                    "push_steps": sg0.push_steps,
                }
            self._diag.record_plan({
                "attempt_index": 0,
                "attempt_seed": None,
                "search_time_ms": verification_time_ms,
                "cumulative_ms": self._total_planning_ms,
                "planning_operation": "reuse_verification",
                "planning_wall_time_ms": planning_wall_time_ms,
                "simulations_used": int(simulations_used),
                "model_warmup_ms": 0.0,
                "model_warmup_excluded_from_planning_time": False,
                "success": success,
                "subgoals_returned": len(chain) if success else 0,
                "first_subgoal": first_subgoal,
                "blacklist_size_before": len(self._failed_pushes),
                "plan_source": reuse_kind,
                "failed_step_index": failed_step_index,
                "failure_reason": failure_reason,
                "robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                "object_poses_cm": {
                    name: [o.x, o.y, o.theta]
                    for name, o in obs.objects.items()
                },
            })
        except Exception as exc:
            print(f"[DIAG] record_plan failed: {exc!r}", flush=True)

    def _try_pending_chain_reuse(self, obs: Observation) -> bool:
        """Try suffix/full-chain reuse from the fresh post-push observation."""
        if self._execution_mode != "mpc":
            return False
        if not self._pending_reuse_chain:
            return False

        source_chain = self._copy_push_chain(self._pending_reuse_chain)
        source_origin = self._pending_reuse_origin or "fresh_plan"
        print(
            f"[NAMOPlanner] Reuse check from updated real scene: "
            f"source={source_origin}, chain_len={len(source_chain)}"
        )

        def _verify_reuse(chain: List[PushSubgoal], reuse_kind: str):
            # While a boundary is held, verification is graded against its
            # frozen points rather than the final goal -- otherwise a chain
            # could verify by making the goal reachable while abandoning the
            # boundary being opened.
            held = self._load_active_target() if self._hold_region_target else None
            planning_wall_start = time.perf_counter()
            result = self._bridge.verify_chain(
                observation=obs,
                robot_goal_cm=self._robot_goal_cm,
                chain=chain,
                target_points=list(held.target_samples_m) if held else None,
                min_reachable=held.minimum_reachable() if held else None,
            )
            planning_wall_time_ms = (
                time.perf_counter() - planning_wall_start
            ) * 1000.0
            self._plan_count += 1
            self._total_planning_ms += result.verification_time_ms
            self._record_reuse_diagnostics(
                obs=obs,
                reuse_kind=reuse_kind,
                verification_time_ms=result.verification_time_ms,
                planning_wall_time_ms=planning_wall_time_ms,
                simulations_used=result.sim_pushes_tried,
                success=result.success,
                chain=result.verified_subgoals if result.success else chain,
                failed_step_index=result.failed_step_index,
                failure_reason=result.failure_reason,
            )
            return result

        suffix_result = None
        if len(source_chain) > 1:
            suffix_chain = self._copy_push_chain(source_chain[1:])
            suffix_result = _verify_reuse(suffix_chain, "reuse_suffix")
            if suffix_result.success:
                print(
                    f"[NAMOPlanner] Reusing suffix chain ({len(suffix_result.verified_subgoals)} pushes) "
                    f"from updated real scene"
                )
                self._queue_mpc_chain(
                    obs=obs,
                    chain=suffix_result.verified_subgoals,
                    origin="reuse_suffix",
                    attempt_index=0,
                    xml_content=suffix_result.planner_scene_xml,
                    object_mapping=suffix_result.object_mapping,
                )
                return True
            if suffix_result.failed_step_index != 0:
                print(
                    "[NAMOPlanner] Post-push reuse failed; falling back to full replanning"
                )
                self._pending_reuse_chain = None
                self._pending_reuse_origin = None
                self._committed_chain = []
                self._committed_chain_origin = None
                return False

        full_result = _verify_reuse(source_chain, "reuse_full")
        if full_result.success:
            if suffix_result is None:
                print(
                    f"[NAMOPlanner] Reusing full chain ({len(full_result.verified_subgoals)} pushes) "
                    f"from updated real scene"
                )
            else:
                print(
                    f"[NAMOPlanner] Reusing full chain ({len(full_result.verified_subgoals)} pushes) "
                    f"from updated real scene after suffix first-step failure"
                )
            self._queue_mpc_chain(
                obs=obs,
                chain=full_result.verified_subgoals,
                origin="reuse_full",
                attempt_index=0,
                xml_content=full_result.planner_scene_xml,
                object_mapping=full_result.object_mapping,
            )
            return True

        print(
            "[NAMOPlanner] Post-push reuse failed; falling back to full replanning"
        )
        self._pending_reuse_chain = None
        self._pending_reuse_origin = None
        self._committed_chain = []
        self._committed_chain_origin = None
        return False

    def plan(self, obs: Observation) -> Optional[Subgoal]:
        """Generate next subgoal from current observation.

        Now includes reachability checking:
        1. If the goal point itself is covered but a nearby cell is free and
           robot-reachable within GOAL_RETARGET_CAP_CM, returns a
           NavigateSubgoal to that cell instead (see _select_goal_retarget).
        2. Else if goal is already reachable, returns NavigateSubgoal to goal
        3. Else goal is blocked, runs NAMO planning and returns PushSubgoals
        4. After pushes complete, re-checks reachability and loops

        Args:
            obs: Current observation

        Returns:
            Next Subgoal to execute (NavigateSubgoal or PushSubgoal),
            or None if complete/failed
        """
        # If we have pending push subgoals, return next one
        if self._current_idx < len(self._subgoals):
            return self._subgoals[self._current_idx]

        # A covered goal point with an open region around it is handled
        # before the coarser region-reachability check below: that check can
        # say REACHABLE while the exact point is still un-drivable (the
        # navigator then fails to path to it), or BLOCKED when a short
        # retarget would avoid a NAMO push entirely. Executor-level only —
        # does not touch _is_goal_reachable or any search-side behavior.
        retarget = self._select_goal_retarget(obs)
        if retarget is not None:
            rx, ry, dist_cm = retarget
            self._navigating_to_goal = True
            self._retarget_point_cm = (rx, ry)
            print(
                f"[NAMOPlanner] Goal point BLOCKED but region is open - "
                f"retargeting to ({rx:.1f}, {ry:.1f}) "
                f"[{dist_cm:.1f}cm from goal, cap={GOAL_RETARGET_CAP_CM:.1f}cm]"
            )
            self._record_goal_retarget_diagnostics(obs, rx, ry, dist_cm)
            return NavigateSubgoal(x=rx, y=ry, theta=None)

        # Check if goal is reachable (either initially or after pushes)
        if self._is_goal_reachable(obs):
            self._navigating_to_goal = True
            self._retarget_point_cm = None
            dist = math.hypot(
                obs.robot_x - self._robot_goal_cm[0],
                obs.robot_y - self._robot_goal_cm[1],
            )
            print(
                f"[NAMOPlanner] Goal REACHABLE - navigating to "
                f"({self._robot_goal_cm[0]:.1f}, {self._robot_goal_cm[1]:.1f}) "
                f"[dist={dist:.1f}cm]"
            )
            return NavigateSubgoal(
                x=self._robot_goal_cm[0],
                y=self._robot_goal_cm[1],
                theta=None,  # Any orientation is fine
            )

        # Goal is blocked - run NAMO planning
        self._navigating_to_goal = False  # Reset in case we were navigating
        self._retarget_point_cm = None
        if self._try_pending_chain_reuse(obs):
            if self._current_idx < len(self._subgoals):
                return self._subgoals[self._current_idx]
        if not self._plan_generated:
            print("[NAMOPlanner] Goal BLOCKED - running NAMO planning...")
            self._generate_plan(obs)

        # Return first push subgoal if available
        if self._current_idx < len(self._subgoals):
            return self._subgoals[self._current_idx]

        return None

    def notify_subgoal_done(self, obs: Observation, failed: bool = False) -> None:
        """Called when current subgoal is completed (or failed).

        Advances to next subgoal in queue. If queue is exhausted,
        clears state so next plan() call re-checks reachability.

        When a subgoal fails (e.g., approach position unreachable),
        triggers replanning with a different random seed to try
        alternative edges/approaches.

        Args:
            obs: Current observation after subgoal completion
            failed: True if the subgoal failed (vs succeeded)
        """
        if self._navigating_to_goal:
            # Success is picked up by is_complete()'s distance check on the
            # next call; nothing to do here.
            if not failed:
                return
            # The navigate-to-goal (or navigate-to-retarget) subgoal failed
            # at the executor level -- e.g. the navigator's own wavefront
            # couldn't path to the target after all. Bounded retry via the
            # same replan-attempt budget push failures use: reset so the
            # next plan() call re-runs the full retarget-or-reachable-or-push
            # decision from scratch, instead of silently repeating a
            # navigate that just failed. Never freeze.
            self._navigating_to_goal = False
            self._retarget_point_cm = None
            self._replan_attempt += 1
            if self._replan_attempt >= self._max_replan_attempts:
                print(
                    f"[NAMOPlanner] Max replan attempts ({self._max_replan_attempts}) "
                    f"exceeded after navigate-to-goal failure - stopping execution"
                )
                self._planning_failed = True
                self._subgoals = []
                self._current_idx = 0
                return
            print(
                f"[NAMOPlanner] Navigate-to-goal FAILED - re-evaluating goal "
                f"reachability (attempt {self._replan_attempt}/{self._max_replan_attempts})"
            )
            self._plan_generated = False
            return

        if failed:
            self._pending_reuse_chain = None
            self._pending_reuse_origin = None
            self._committed_chain = []
            self._committed_chain_origin = None
            # Track which subgoal failed
            if self._current_idx < len(self._subgoals):
                self._failed_subgoal = self._subgoals[self._current_idx]
                print(
                    f"[NAMOPlanner] Subgoal FAILED: {self._failed_subgoal.object_id} "
                    f"edge={self._failed_subgoal.edge_idx}"
                )
                # Feed the failure back into the planner: blacklist this
                # (object_id, edge_idx) pair so future plans never suggest it
                # again within this planning episode.
                blacklist_entry = (
                    self._failed_subgoal.object_id,
                    self._failed_subgoal.edge_idx,
                )
                if blacklist_entry not in self._failed_pushes:
                    self._failed_pushes.add(blacklist_entry)
                    if self._active_target is not None:
                        # Also against the subproblem, so the failure outlives
                        # this planner instance and reaches the next process.
                        self._store_active_target(
                            self._active_target.with_failed_push(*blacklist_entry)
                        )
                    print(
                        f"[NAMOPlanner] Blacklisted ({blacklist_entry[0]}, edge={blacklist_entry[1]}); "
                        f"blacklist size now {len(self._failed_pushes)}"
                    )

            # Increment replan attempt counter
            self._replan_attempt += 1

            if self._replan_attempt >= self._max_replan_attempts:
                # Exceeded max retries - give up
                print(
                    f"[NAMOPlanner] Max replan attempts ({self._max_replan_attempts}) "
                    f"exceeded - stopping execution"
                )
                self._planning_failed = True
                self._subgoals = []
                self._current_idx = 0
                return

            # Replan with new random seed to try different edge ordering
            print(
                f"[NAMOPlanner] Replanning (attempt {self._replan_attempt}/"
                f"{self._max_replan_attempts}) with new random seed..."
            )
            self._subgoals = []
            self._current_idx = 0
            self._plan_generated = False  # Will trigger replan on next plan() call
            # Note: shuffle_seed will be updated in _generate_plan based on attempt number
            return

        # Success - reset replan counter
        self._replan_attempt = 0
        self._failed_subgoal = None

        # Drop blacklist entries for the object we just pushed, and only those.
        #
        # The key is (object_id, edge_idx) with edge_idx in the object's BODY
        # frame, so moving or rotating an object makes its own stale entries
        # meaningless -- edge 17 is a different world-frame approach afterwards.
        # That is why these must go.
        #
        # Entries for every OTHER object stay. This push did not change their
        # body frames, so a push that physically failed on them is still just
        # as unavailable, and forgetting it means the next plan can propose it
        # again -- which on the real robot is a loop, since one push is
        # executed per replan. Clearing the whole set on any success was
        # discarding the only failure memory the planner has.
        pushed = self._subgoals[self._current_idx] if self._current_idx < len(self._subgoals) else None
        pushed_object_id = getattr(pushed, "object_id", None)
        moved = self._objects_that_moved(self._observation_at_commit, obs)
        if pushed_object_id is not None:
            moved.add(pushed_object_id)
        if moved:
            # The persisted target carries its own copy of these entries and is
            # what the next process reads, so it has to forget them too.
            if self._active_target is not None:
                pruned = self._active_target.forgetting_moved(sorted(moved))
                if pruned is not self._active_target:
                    self._store_active_target(pruned)
            stale = {entry for entry in self._failed_pushes if entry[0] in moved}
            if stale:
                self._failed_pushes -= stale
                print(
                    f"[NAMOPlanner] Dropped {len(stale)} blacklist entry/entries for "
                    f"{sorted(moved)} (their body frames moved); "
                    f"{len(self._failed_pushes)} retained"
                )

        if self._execution_mode == "mpc":
            executed_chain = self._copy_push_chain(self._committed_chain)
            self._subgoals = []
            self._current_idx = 0
            self._plan_generated = False

            if self._is_goal_reachable(obs):
                print(
                    "[NAMOPlanner] Goal is NOW REACHABLE - skipping committed-chain reuse"
                )
                self._pending_reuse_chain = None
                self._pending_reuse_origin = None
                self._committed_chain = []
                self._committed_chain_origin = None
                return

            if executed_chain:
                self._pending_reuse_chain = executed_chain
                self._pending_reuse_origin = self._committed_chain_origin or "fresh_plan"
                if len(executed_chain) > 1:
                    print(
                        f"[NAMOPlanner] Push complete. Will try reuse from updated real scene "
                        f"for remaining chain len={len(executed_chain) - 1}"
                    )
                else:
                    print(
                        "[NAMOPlanner] Push complete. Will retry the committed single-push "
                        "plan from the updated real scene before fresh replanning"
                    )
            else:
                self._pending_reuse_chain = None
                self._pending_reuse_origin = None
                self._committed_chain = []
                self._committed_chain_origin = None
                print(
                    "[NAMOPlanner] Push sequence complete. "
                    "Will re-check reachability."
                )
            return

        self._current_idx += 1
        remaining = len(self._subgoals) - self._current_idx

        if self._verbose:
            print(f"[NAMOPlanner] Subgoal done. Remaining: {remaining}")

        # After every push, check if goal is now reachable
        # This lets us skip remaining pushes and go straight to the goal
        if remaining > 0:
            print(
                f"[NAMOPlanner] Checking reachability after push "
                f"({remaining} subgoals remaining)..."
            )
            if self._is_goal_reachable(obs):
                print(
                    f"[NAMOPlanner] Goal is NOW REACHABLE - "
                    f"skipping {remaining} remaining push(es)"
                )
                self._subgoals = []
                self._current_idx = 0
                self._plan_generated = False
                return

        # If all pushes done, clear queue so next plan() re-checks reachability
        if self._current_idx >= len(self._subgoals):
            self._subgoals = []
            self._current_idx = 0
            self._plan_generated = False
            print(
                "[NAMOPlanner] Push sequence complete. "
                "Will re-check reachability."
            )

    def is_complete(self, obs: Observation) -> bool:
        """Return whether goal navigation has physically reached its target.

        Reachability causes :meth:`plan` to dispatch a ``NavigateSubgoal``.
        Planning exhaustion is a failure state, not completion.
        """
        if self._planning_failed or not self._navigating_to_goal:
            return False

        target = self._retarget_point_cm or self._robot_goal_cm
        dist = math.hypot(obs.robot_x - target[0], obs.robot_y - target[1])
        if dist >= self._goal_tolerance:
            return False

        if self._retarget_point_cm is not None:
            print(
                f"[NAMOPlanner] GOAL REACHED (success-with-retarget)! "
                f"Distance to retarget point: {dist:.1f}cm | "
                f"Total planning: {self._plan_count} calls, "
                f"{self._total_planning_ms:.0f}ms cumulative"
            )
        else:
            print(
                f"[NAMOPlanner] GOAL REACHED! Distance: {dist:.1f}cm | "
                f"Total planning: {self._plan_count} calls, "
                f"{self._total_planning_ms:.0f}ms cumulative"
            )
        return True

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Return drawings for visualization.

        Draws:
        - Goal position
        - Current subgoal object highlight
        """
        drawings = []

        # Draw goal
        drawings.append(
            {
                "uuid": "namo_goal",
                "type": "point",
                "position": self._robot_goal_cm,
                "radius": 10,
                "color": "#00FF00",
                "fill": "#00FF0044",
            }
        )

        # Draw current subgoal info
        if self._current_idx < len(self._subgoals):
            subgoal = self._subgoals[self._current_idx]
            drawings.append(
                {
                    "uuid": "namo_target",
                    "type": "text",
                    "position": (5, 5),
                    "text": f"Target: {subgoal.object_id} edge={subgoal.edge_idx}",
                    "color": "#FFFFFF",
                }
            )

        return drawings

    def reset(self) -> None:
        """Reset planner state."""
        self._subgoals = []
        self._current_idx = 0
        self._plan_generated = False
        self._planning_failed = False
        self._navigating_to_goal = False
        self._retarget_point_cm = None
        self._replan_attempt = 0
        self._failed_subgoal = None
        self._failed_pushes = set()
        self._committed_chain = []
        self._committed_chain_origin = None
        self._pending_reuse_chain = None
        self._pending_reuse_origin = None

    def _select_goal_retarget(self, obs: Observation) -> Optional[Tuple[float, float, float]]:
        """Nearest free, robot-reachable cell to the goal point.

        Returns None when the goal cell itself is free (nothing to
        retarget) or when the nearest reachable free cell is farther than
        GOAL_RETARGET_CAP_CM (something other than the pushed object is
        blocking the goal; keep planning pushes instead of retargeting).
        Otherwise returns (x_cm, y_cm, distance_cm).

        Uses the navigator's own wavefront grid (WavefrontPlanner at
        reachability_filter's 5mm resolution / tier1 margin), not the C++
        region-reachability check _is_goal_reachable uses -- the two
        disagree exactly on this "point covered, region open" case, and
        this decision has to agree with what the navigator can actually
        drive to.
        """
        return find_goal_retarget(
            observation=obs,
            robot_goal_cm=self._robot_goal_cm,
            cap_cm=GOAL_RETARGET_CAP_CM,
            workspace_width_cm=self._workspace_width_cm,
            workspace_height_cm=self._workspace_height_cm,
            robot_width_cm=self._robot_width_cm,
            robot_height_cm=self._robot_height_cm,
        )

    def _record_goal_retarget_diagnostics(
        self, obs: Observation, x_cm: float, y_cm: float, distance_cm: float
    ) -> None:
        """Best-effort diagnostics record for a goal-retarget decision."""
        if self._diag is None:
            return
        try:
            self._diag.record_plan({
                "attempt_index": 0,
                "attempt_seed": None,
                "search_time_ms": 0.0,
                "cumulative_ms": self._total_planning_ms,
                "planning_operation": "decision_only",
                "planning_wall_time_ms": 0.0,
                "simulations_used": 0,
                "model_warmup_ms": 0.0,
                "model_warmup_excluded_from_planning_time": False,
                "success": True,
                "subgoals_returned": 0,
                "first_subgoal": None,
                "blacklist_size_before": len(self._failed_pushes),
                "plan_source": "goal_retarget",
                "retarget_point_cm": [x_cm, y_cm],
                "retarget_distance_cm": distance_cm,
                "goal_point_cm": list(self._robot_goal_cm),
                "failed_step_index": None,
                "failure_reason": None,
                "robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                "object_poses_cm": {
                    name: [o.x, o.y, o.theta]
                    for name, o in obs.objects.items()
                },
            })
        except Exception as exc:
            print(f"[DIAG] record_plan (goal_retarget) failed: {exc!r}", flush=True)

    def _is_goal_reachable(self, obs: Observation) -> bool:
        """Check if robot can reach goal without pushing any objects.

        Uses unified C++ wavefront reachability from NAMO bindings.

        Args:
            obs: Current observation with robot and object positions

        Returns:
            True if a collision-free path to goal exists
        """
        summary = self._bridge.analyze_reachability(
            observation=obs,
            robot_goal_cm=self._robot_goal_cm,
            analysis_mode=False,
        )
        self._last_reachability_summary = summary
        reachable = bool(summary.get("goal_reachable", False))

        if self._verbose:
            status = "REACHABLE" if reachable else "BLOCKED"
            obj_summary = summary.get("objects", {})
            reachable_objects = sum(1 for s in obj_summary.values() if s.get("reachable", False))
            print(
                f"[NAMOPlanner] Goal reachability: {status} "
                f"(reachable_objects={reachable_objects}/{len(obj_summary)})"
            )
            if summary.get("error_message"):
                print(f"[NAMOPlanner] Reachability error: {summary['error_message']}")

        return reachable

    # A plan call may cross several boundaries that turn out to be already
    # open -- opening one can merge regions and clear the next. Bounded so a
    # graph/opener disagreement cannot spin here instead of returning.
    MAX_BOUNDARY_ADVANCES_PER_PLAN = MAX_BOUNDARY_ADVANCES

    def _load_active_target(self) -> Optional[RegionOpeningTarget]:
        """Prefer the persisted target, so a separate process sees the same one.

        The loaded target is adopted as this instance's, not just returned. The
        common case is that an earlier process wrote the file, and without this
        _release_active_target has nothing to release: it would find
        self._active_target still None and quietly skip the write, leaving a
        dead boundary marked active for the next process to pick up again.
        """
        if self._active_target_path is not None:
            loaded = RegionOpeningTarget.load(self._active_target_path)
            if loaded is not None:
                self._active_target = loaded
            return loaded
        return self._active_target if (self._active_target and self._active_target.is_active) else None

    def _store_active_target(self, target: Optional[RegionOpeningTarget]) -> None:
        self._active_target = target
        if self._active_target_path is not None and target is not None:
            target.save(self._active_target_path)

    def _release_active_target(self, status: str) -> None:
        if self._active_target is not None:
            released = self._active_target.released(status)
            if self._active_target_path is not None:
                released.save(self._active_target_path)
            print(f"[NAMOPlanner] Region target {released.target_id} released: {status}")
        self._active_target = None

    @staticmethod
    def _objects_that_moved(
        before: Optional[Observation], after: Observation
    ) -> Set[str]:
        """Adapter onto the shared rule, from this path's Observation objects.

        Without a before-observation this returns nothing and the caller falls
        back to dropping only the pushed object.
        """
        if before is None:
            return set()

        def poses(obs: Observation) -> Dict[str, Tuple[float, float, float]]:
            return {
                name: (p.x, p.y, p.theta) for name, p in (obs.objects or {}).items()
            }

        return objects_that_moved(poses(before), poses(after))


    def _unreachable_contact_points(self, obs) -> Set[Tuple[str, int]]:
        """Edges whose approach pose sits in an obstacle, as (object, edge) pairs.

        Collision-freedom, not reachability. See reachability_filter's module
        docstring for why that distinction matters at chain depth.

        Never raises. A filter that fails should cost nothing rather than stop
        the robot, so any error degrades to "filter nothing" AND SAYS SO. The
        filter itself raises rather than returning empty, so this print is the
        only thing separating a broken filter from a clean scene.
        """
        try:
            blocked = unreachable_contact_points(
                obs,
                workspace_width_cm=self._workspace_width_cm,
                workspace_height_cm=self._workspace_height_cm,
                robot_width_cm=self._robot_width_cm,
                robot_height_cm=self._robot_height_cm,
            )
        except Exception as exc:
            print(f"[NAMOPlanner] reachability filter skipped: {exc!r}")
            return set()
        if blocked and self._verbose:
            per_object: Dict[str, int] = {}
            for name, _edge in blocked:
                per_object[name] = per_object.get(name, 0) + 1
            print(f"[NAMOPlanner] blocked {len(blocked)} contact points "
                  f"the navigator cannot reach: {per_object}")
        return blocked

    def _search_planner_kwargs(self, shuffle_seed: Optional[int]) -> Dict[str, Any]:
        """Every planner option both the whole-problem and held paths must send.

        bridge.plan names most of these and solve_boundary forwards them as
        kwargs, but both land in the same service, which maps them through one
        table. Held mode used to build its own shorter list. Anything it left
        out fell back to namo_cpp's opener defaults, and region_max_chain_depth
        defaults to 1: no setup-then-finish chain can exist at that depth, which
        is the only reason to hold a boundary across pushes. Building one dict
        here is what stops the two paths searching differently again.
        """
        kwargs: Dict[str, Any] = {
            "goal_strategy": self._goal_strategy,
            "max_chain_depth": self._max_chain_depth,
            "frontier_beam_width": self._frontier_beam_width,
            "chain_link_cost": self._chain_link_cost,
            "selection_strategy": self._selection_strategy,
            "goals_per_region": self._goals_per_region,
            "shuffle_edges": self._shuffle_edges,
            "shuffle_seed": shuffle_seed,
        }
        if self._rollout_samples_per_state is not None:
            kwargs["rollout_samples_per_state"] = self._rollout_samples_per_state
        kwargs.update(self._local_search.as_planner_kwargs())
        if self._ml_goal_model_path:
            kwargs["ml_goal_model_path"] = self._ml_goal_model_path
            kwargs["ml_device"] = self._ml_device
            if self._ml_samples is not None:
                kwargs["ml_samples"] = self._ml_samples
            if self._ml_num_steps is not None:
                kwargs["ml_num_steps"] = self._ml_num_steps
            if self._ml_sampler_method is not None:
                kwargs["ml_sampler_method"] = self._ml_sampler_method
        return kwargs

    def _warmup_model_ranker_once(self) -> float:
        """Warm the learned best-first ranker once outside planning wall time."""
        if not self._local_search.uses_ranker or self._model_ranker_warmed:
            return 0.0

        checkpoint = self._local_search.scorer_ckpt
        device = self._local_search.ml_device or "cpu"
        warmup_ms = self._bridge.warmup_best_first_scorer(
            checkpoint=checkpoint,
            device=device,
        )
        self._model_ranker_warmed = True
        print(
            f"[NAMOPlanner] Best-first scorer warmup: {warmup_ms:.0f}ms "
            "(recorded separately; excluded from planning_wall_time_ms)",
            flush=True,
        )
        return warmup_ms

    def _held_mode_planner_kwargs(self) -> Dict[str, Any]:
        """The same options the whole-problem path sends, at the base seed, plus the mode.

        Held mode does not retry with a bumped seed the way the whole-problem
        path does; a held boundary is re-solved on the next replan instead.

        The execution mode is added HERE and not in _search_planner_kwargs,
        which both paths share. `mode` is a named parameter of
        `solve_boundary_from_xml`, and only this path calls it. On the
        whole-problem path it would ride into `plan_from_xml`'s
        `algorithm_params` and be dropped without a word, which is how a run
        ends up filed under an arm it never ran.
        """
        kwargs = self._search_planner_kwargs(self._shuffle_seed)
        kwargs.update(self._local_search.as_boundary_kwargs())
        return kwargs

    def _record_plan_diagnostics(
        self,
        obs: Observation,
        *,
        attempt_index: int,
        attempt_seed: Optional[int],
        subgoals: Sequence[Any],
        success: bool,
        planning_wall_time_ms: float,
        model_warmup_ms: float = 0.0,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        """One JSONL line per planning attempt, from either planning path.

        Shared so the two paths cannot drift into two schemas. The held path had
        no diagnostics at all, which was survivable while it was a variant of
        the same search and is not now that it carries a factor of the trial
        matrix.

        Diagnostics must never break the planner, so this swallows its own
        failures the way the caller it was extracted from did.
        """
        if self._diag is None:
            return
        try:
            first_subgoal = None
            if subgoals:
                sg0 = subgoals[0]
                first_subgoal = {
                    "object_id": getattr(sg0, "object_id", None),
                    "edge_idx": getattr(sg0, "edge_idx", None),
                    "push_steps": getattr(sg0, "push_steps", None),
                }
            algorithm_stats = _filter_algorithm_stats_for_diagnostics(
                self._bridge.last_algorithm_stats or {}
            )
            record: Dict[str, Any] = {
                "attempt_index": attempt_index,
                "attempt_seed": attempt_seed,
                "search_time_ms": self._bridge.last_search_time_ms,
                "cumulative_ms": self._total_planning_ms,
                "planning_operation": "fresh_search",
                "planning_wall_time_ms": planning_wall_time_ms,
                "simulations_used": int(algorithm_stats.get("simulations_used", 0)),
                "model_warmup_ms": float(model_warmup_ms),
                "model_warmup_excluded_from_planning_time": model_warmup_ms > 0.0,
                "success": bool(success),
                "subgoals_returned": len(subgoals),
                "first_subgoal": first_subgoal,
                "blacklist_size_before": len(self._failed_pushes),
                "algorithm_stats": algorithm_stats,
                "robot_pose_cm": [obs.robot_x, obs.robot_y, obs.robot_theta],
                "object_poses_cm": {
                    name: [o.x, o.y, o.theta] for name, o in obs.objects.items()
                },
            }
            record.update(extra or {})
            self._diag.record_plan(record)
        except Exception as _e:
            print(f"[DIAG] record_plan failed: {_e!r}", flush=True)

    def _generate_plan_holding_target(
        self,
        obs: Observation,
        model_warmup_ms: float = 0.0,
    ) -> None:
        """Plan against one held boundary, advancing only when it opens.

        The advance step itself is shared with run_namo's plan-only mode, which
        drives the same loop from a separate process.
        """
        before = self._load_active_target()
        planning_wall_start = time.perf_counter()
        plan, target, status, released = advance_boundary(
            self._bridge,
            obs,
            self._robot_goal_cm,
            target=before,
            open_fraction=CANONICAL_OPEN_FRACTION,
            scale_factor=self._scale_factor,
            iteration=self._plan_count,
            max_advances=self.MAX_BOUNDARY_ADVANCES_PER_PLAN,
            planner_kwargs=self._held_mode_planner_kwargs(),
        )
        planning_wall_time_ms = (
            time.perf_counter() - planning_wall_start
        ) * 1000.0
        self._plan_count += 1
        self._total_planning_ms += self._bridge.last_search_time_ms

        # Recorded here and not only on the whole-problem path. Reactive runs
        # exclusively through this loop -- it is the only path that reaches
        # solve_boundary_from_xml, which is the only method that reads the mode
        # -- so without this line the arm that motivates the whole comparison
        # writes to no log but the console banner.
        self._record_plan_diagnostics(
            obs,
            attempt_index=1,   # held mode re-solves on the next replan, never retries here
            attempt_seed=self._shuffle_seed,
            subgoals=list(getattr(plan, "subgoals", []) or []),
            success=(status == ADVANCE_PLANNED),
            planning_wall_time_ms=planning_wall_time_ms,
            model_warmup_ms=model_warmup_ms,
            extra={
                "origin": "region_target",
                "boundary_status": str(status),
                "target_id": str(getattr(target, "target_id", "") or ""),
                "failure_reason": str(getattr(plan, "failure_reason", "") or ""),
            },
        )

        # advance_boundary reports what became of the target it was handed, so
        # a transient failure no longer looks like the boundary opened.
        if released is not None:
            self._release_active_target(released)

        if target is None:
            self._subgoals = []
            return

        self._store_active_target(target)
        if status != ADVANCE_PLANNED:
            print(
                f"[NAMOPlanner] Boundary {getattr(plan, 'resolved_target', '?') or '?'} "
                f"produced no plan ({getattr(plan, 'failure_reason', '') or 'unknown'})"
            )
            self._subgoals = []
            return

        self._queue_mpc_chain(
            obs=obs,
            chain=plan.subgoals,
            origin="region_target",
            attempt_index=0,
            xml_content=self._bridge.last_xml_content,
            object_mapping=None,
        )

    def _generate_plan(self, obs: Observation) -> None:
        """Generate subgoal queue via NAMO planning.

        Retries up to _max_planning_retries times with different random seeds
        when planning returns no solution (empty subgoals or exception).
        """
        self._plan_generated = True
        model_warmup_ms = self._warmup_model_ranker_once()
        if self._hold_region_target:
            self._generate_plan_holding_target(obs, model_warmup_ms)
            return
        max_retries = self._max_planning_retries

        # Aggregate diagnostic stats across all attempts so the failure
        # message can show a real breakdown of why pushes were rejected,
        # instead of a canned "possible causes" list.
        aggregate_rejections: Dict[str, int] = {}
        aggregate_primitives_attempted = 0

        for attempt in range(max_retries):
            # Compute shuffle seed: combine replan attempt (execution retry)
            # with planning attempt (no-solution retry) for maximum variation
            combined_attempt = self._replan_attempt * max_retries + attempt
            if combined_attempt > 0:
                if self._shuffle_seed is not None:
                    effective_seed = self._shuffle_seed + combined_attempt
                else:
                    effective_seed = combined_attempt * 12345
                if attempt > 0:
                    print(
                        f"[NAMOPlanner] Planning retry {attempt}/{max_retries - 1} "
                        f"with shuffle_seed={effective_seed}"
                    )
                elif self._replan_attempt > 0:
                    print(
                        f"[NAMOPlanner] Using shuffle_seed={effective_seed} "
                        f"for replan attempt {self._replan_attempt}"
                    )
            else:
                effective_seed = self._shuffle_seed

            try:
                # Approach poses the executor's own grid says sit inside an
                # obstacle. Collision-freedom only, NOT reachability: the two
                # grids rasterise the same inflation rule differently near a
                # wall, so the planner would otherwise spend simulations on
                # poses that put a corner through it. Deliberately not a
                # reachability test, since this ban applies at every chain
                # depth and would delete 2-chains. See reachability_filter's
                # module docstring. Recomputed per attempt because objects move.
                blocked = self._unreachable_contact_points(obs)
                planning_wall_start = time.perf_counter()
                subgoals = self._bridge.plan(
                    observation=obs,
                    robot_goal_cm=self._robot_goal_cm,
                    algorithm=self._algorithm,
                    failed_pushes=self._failed_pushes | blocked,
                    **self._search_planner_kwargs(effective_seed),
                )
                planning_wall_time_ms = (
                    time.perf_counter() - planning_wall_start
                ) * 1000.0

                # Accumulate planning time
                self._plan_count += 1
                self._total_planning_ms += self._bridge.last_search_time_ms
                print(
                    f"[NAMOPlanner] Plan #{self._plan_count}: "
                    f"{self._bridge.last_search_time_ms:.0f}ms, "
                    f"cumulative: {self._total_planning_ms:.0f}ms"
                )

                # Aggregate diagnostic stats from this attempt (best-effort —
                # missing keys are normal for non-region-opening planners).
                _stats = self._bridge.last_algorithm_stats or {}
                _attempt_breakdown = _stats.get("rejection_breakdown") or {}
                for _k, _v in _attempt_breakdown.items():
                    aggregate_rejections[_k] = aggregate_rejections.get(_k, 0) + int(_v)
                aggregate_primitives_attempted += int(_stats.get("total_primitives_attempted", 0))

                # Forward this attempt to the diagnostics recorder (no-op when
                # recorder is None). One JSONL line per planning attempt — the
                # recorder assigns a plan_id, we contribute the structured fields.
                # algorithm_stats is filtered to JSON-safe summary fields only;
                # raw fields like `attempt_results` carry C++ Action objects that
                # can't be pickled or JSON-serialized.
                self._record_plan_diagnostics(
                    obs,
                    attempt_index=attempt + 1,
                    attempt_seed=effective_seed,
                    subgoals=list(subgoals or []),
                    success=bool(subgoals),
                    planning_wall_time_ms=planning_wall_time_ms,
                    model_warmup_ms=(
                        model_warmup_ms if attempt == 0 else 0.0
                    ),
                    extra={"origin": "fresh_plan"},
                )

                if subgoals:
                    if self._execution_mode == "mpc":
                        self._queue_mpc_chain(
                            obs=obs,
                            chain=subgoals,
                            origin="fresh_plan",
                            attempt_index=attempt + 1,
                        )
                    else:
                        # open_loop: commit to the full planned sequence.
                        self._committed_chain = self._copy_push_chain(subgoals)
                        self._committed_chain_origin = "fresh_plan"
                        self._subgoals = subgoals
                        self._current_idx = 0
                        if self._verbose:
                            print(
                                f"[NAMOPlanner] Queued {len(self._subgoals)} subgoal(s) "
                                f"({self._execution_mode} mode, planner returned "
                                f"{len(subgoals)}):"
                            )
                            for i, sg in enumerate(self._subgoals):
                                print(
                                    f"  [{i}] {sg.object_id} edge={sg.edge_idx} "
                                    f"steps={sg.push_steps}"
                                )
                        self._emit_sim_capture_for_plan(
                            obs=obs,
                            subgoals=subgoals,
                            attempt_index=attempt + 1,
                        )
                    return  # Success

                # No subgoals - will retry if attempts remain
                print(
                    f"[NAMOPlanner] Planning returned NO SUBGOALS "
                    f"(attempt {attempt + 1}/{max_retries})"
                )

            except Exception as e:
                print(
                    f"[NAMOPlanner] Planning FAILED (attempt {attempt + 1}/{max_retries}): {e}"
                )
                if self._verbose:
                    import traceback
                    traceback.print_exc()

        # All retries exhausted — print a real diagnostic breakdown of why
        # the planner rejected every push it considered. Counts are aggregated
        # across all retry attempts in this plan() call.
        print(
            f"[NAMOPlanner] All {max_retries} planning attempts failed"
        )
        # Reminder to correlate with robot connectivity state. Planning runs
        # in sim regardless of robot state, but if the robot is offline,
        # execution can't happen — and a perception/state drift while the
        # robot is dead may make planning look impossible when it isn't.
        print(
            "[NAMOPlanner]   (planning is sim-only; check Runtime '🔌 Robot ... OFFLINE' "
            "lines above to verify the robot was online during this attempt)"
        )
        if aggregate_primitives_attempted == 0 and not aggregate_rejections:
            # No diagnostic data — likely a non-region-opening planner or a hard error.
            print("[NAMOPlanner]   (no diagnostic data available from planner)")
        else:
            print(
                f"[NAMOPlanner]   {aggregate_primitives_attempted} primitives enumerated "
                f"across {max_retries} attempts; breakdown:"
            )
            # Categorize the keys so the output reads top-down by severity.
            outcome_order = [
                ("executed_in_sim", "pushes simulated"),
                ("push_opened_region", "successfully opened region (but plan rejected — check chain logic)"),
                ("push_did_not_open_region", "push ran but didn't open the target region"),
                ("edge_unreachable", "edge approach pose unreachable (robot can't get to push position)"),
                ("controller_stuck", "controller stuck mid-push (object resisted motion)"),
                ("push_collided_with_wall", "push trajectory collided with a wall"),
                ("env_step_exception", "env.step() raised an exception"),
                ("skipped_edge_blacklisted_deeper", "skipped (edge previously stuck at shallower depth)"),
                ("skipped_edge_already_solved", "skipped (edge already opened a region this skill)"),
            ]
            printed_keys: Set[str] = set()
            for key, label in outcome_order:
                if key in aggregate_rejections:
                    print(f"[NAMOPlanner]     {aggregate_rejections[key]:>6}  {label}")
                    printed_keys.add(key)
            # Surface any unknown keys we didn't categorize (e.g. failure_type_N).
            for key, count in sorted(aggregate_rejections.items()):
                if key not in printed_keys:
                    print(f"[NAMOPlanner]     {count:>6}  {key}")
        self._planning_failed = True
        self._subgoals = []
        self._current_idx = 0
        self._committed_chain = []
        self._committed_chain_origin = None
        self._pending_reuse_chain = None
        self._pending_reuse_origin = None

    def get_subgoal_queue(self) -> List[PushSubgoal]:
        """Get the current subgoal queue (for debugging)."""
        return list(self._subgoals)

    def get_current_index(self) -> int:
        """Get current index in subgoal queue (for debugging)."""
        return self._current_idx
