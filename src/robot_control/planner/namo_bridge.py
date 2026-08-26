"""Bridge between robot_control observations and NAMO planning.

Handles:
- State capture from Observation to MuJoCo XML
- Object ID mapping between real robot names and simulation names
- Coordinate transformation (cm -> scaled meters)
- Plan conversion to PushSubgoal sequence
"""

from __future__ import annotations

import math
import os
import sys
import shutil
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import yaml

from robot_control.camera.workspace import WORKSPACE_HEIGHT_CM, WORKSPACE_WIDTH_CM
from robot_control.core.types import Observation, PushSubgoal
from robot_control.planner.namo_binding_loader import (
    ensure_namo_cpp_paths,
    load_canonical_namo_rl,
    resolve_namo_cpp_dir,
)
from robot_control.planner.search_config import DEFAULT_EXEC_MODE
from robot_control.utils import NAMOXMLGenerator
from robot_control.utils.robot_geometry import (
    effective_robot_radius_cm,
    scaled_half_extents_m_from_full_extents_cm,
)


# Sidecar namo_cpp reads for its wavefront inflation margin. It resolves the
# name relative to the primary config, so the two must stay in one directory.
WAVEFRONT_INFLATION_SIDECAR = "wavefront_inflation.yaml"


@dataclass
class ObjectMapping:
    """Bidirectional mapping between real and simulation object IDs."""

    real_to_sim: Dict[str, str] = field(default_factory=dict)
    sim_to_real: Dict[str, str] = field(default_factory=dict)

    def add(self, real_name: str, sim_name: str) -> None:
        """Add a mapping between real and simulation names."""
        self.real_to_sim[real_name] = sim_name
        self.sim_to_real[sim_name] = real_name

    def get_sim_name(self, real_name: str) -> str:
        """Get simulation name for a real object ID."""
        return self.real_to_sim.get(real_name, real_name)

    def get_real_name(self, sim_name: str) -> str:
        """Get real name for a simulation object ID."""
        return self.sim_to_real.get(sim_name, sim_name)

    def clear(self) -> None:
        """Clear all mappings."""
        self.real_to_sim.clear()
        self.sim_to_real.clear()


@dataclass
class ChainVerificationResult:
    """Structured result of exact-chain simulation verification."""

    success: bool
    verified_subgoals: List[PushSubgoal]
    sim_pushes_tried: int
    failed_step_index: Optional[int]
    failure_reason: Optional[str]
    goal_reachable_after: bool
    verification_time_ms: float
    planner_scene_xml: str
    object_mapping: Dict[str, Dict[str, str]]
    # None when the chain was verified against the final goal rather than a
    # held boundary. Populated only when target points were supplied.
    target_open_after: Optional[bool] = None


def evaluate_chain_outcome(
    env: Any,
    target_points: Optional[Sequence[Tuple[float, float]]] = None,
    min_reachable: Optional[int] = None,
) -> Tuple[bool, bool, Optional[bool]]:
    """Decide whether a simulated chain succeeded.

    Returns ``(succeeded, goal_reachable_after, target_open_after)``.

    Without target points the question is the one this has always asked: does
    the final state make the robot's goal reachable. While a boundary is held
    that is the wrong question -- a chain can make the goal reachable while
    abandoning the boundary being opened, and accepting it would silently undo
    the subproblem. So when points are supplied they decide instead.

    The goal check is still evaluated and reported either way, so no existing
    reader of ``goal_reachable_after`` changes meaning. It costs one extra
    wavefront update (~7 ms) against a ~160 ms simulated push.
    """
    goal_reachable_after = bool(env.is_robot_goal_reachable())
    if not target_points:
        return goal_reachable_after, goal_reachable_after, None

    reachable_count, _first = env.count_reachable_points(list(target_points))
    target_open_after = int(reachable_count) >= int(min_reachable or 1)
    # Either ends the chain's job. Opening the boundary advances the subproblem;
    # making the goal reachable ends the whole problem, and a push can do that by
    # re-partitioning free space along a different route than the one being
    # worked on. Rejecting that would discard a solution to the actual task.
    return (target_open_after or goal_reachable_after), goal_reachable_after, target_open_after


@dataclass
class BoundaryChoice:
    """Which boundary namo_cpp says to open next, in real-object naming."""

    found: bool = False
    target_points_m: List[Tuple[float, float]] = field(default_factory=list)
    blocker_real_ids: List[str] = field(default_factory=list)
    region_path: List[str] = field(default_factory=list)
    goal_already_reachable: bool = False
    # Blocked pairs namo_cpp says name no edge in the scene it just looked at.
    # A caller carrying a blocklist across a push learns here that some of it
    # stopped meaning anything, instead of routing around nothing and calling
    # that success.
    stale_blocked_boundaries: List[Tuple[str, str]] = field(default_factory=list)
    failure_reason: str = ""


@dataclass
class BoundaryPlan:
    """Result of asking namo_cpp to open one pinned boundary."""

    subgoals: List[PushSubgoal] = field(default_factory=list)
    success: bool = False
    # The boundary already cleared its bar with no pushes: success, nothing to do.
    already_open: bool = False
    # Every push against this boundary failed; the caller should stop trying it.
    boundary_exhausted: bool = False
    failure_reason: str = ""
    # Both ends of the boundary as namo_cpp labelled them in THIS call. A caller
    # excluding this boundary from its next selection needs two labels from one
    # snapshot; mixing one of these with a label persisted before the last push
    # can name a live boundary that is not this one, because labels get
    # reassigned rather than retired.
    resolved_source: str = ""
    resolved_target: str = ""


class NAMOPlanBridge:
    """Bridge for NAMO planning from robot_control observations.

    Captures current robot state, generates scaled MuJoCo XML,
    invokes NAMO planning, and converts results to PushSubgoals.

    Usage:
        bridge = NAMOPlanBridge(
            namo_config_path="namo_cpp/config/namo_config_complete_skill15_car_1x.yaml",
            scale_factor=1.0,
        )

        subgoals = bridge.plan(
            observation=obs,
            robot_goal_cm=(50.0, 40.0),
            algorithm="full_namo",
        )
    """

    def __init__(
        self,
        namo_config_path: str,
        scale_factor: float = 1.0,
        primitive_data_dir: Optional[str] = None,
        show_push_scores: bool = False,
        verbose: bool = False,
        debug_xml_path: Optional[str] = None,
        enable_viewer: bool = False,
        pause_after_load: bool = False,
        robot_width_cm: float = 7.0,
        robot_height_cm: float = 7.0,
        robot_model: str = "car",
        manual_primitives_file: Optional[str] = None,
    ):
        """Initialize the NAMO bridge.

        Args:
            namo_config_path: Path to NAMO config YAML
            scale_factor: Fixed at 1.0.
            primitive_data_dir: Directory containing motion primitive data.
                               If None, uses namo_cpp/data/ (auto-detected).
            verbose: Enable verbose logging
            debug_xml_path: If set, save generated XML to this path for debugging
            enable_viewer: Enable MuJoCo visualization window during planning
            pause_after_load: Pause for user input after loading XML (for inspection)
            robot_width_cm: Robot width in cm (for XML generation)
            robot_height_cm: Robot height in cm (for XML generation)
            robot_model: Fixed at ``car``.
        """
        if robot_model != "car" or abs(scale_factor - 1.0) > 1e-9:
            raise ValueError(
                "NAMOPlanBridge supports only robot_model='car' "
                "with scale_factor=1.0"
            )
        self._namo_config_path = namo_config_path
        self._scale_factor = scale_factor
        self._verbose = verbose
        self._debug_xml_path = debug_xml_path
        self._enable_viewer = enable_viewer
        self._pause_after_load = pause_after_load
        self._robot_width_cm = float(robot_width_cm)
        self._robot_height_cm = float(robot_height_cm)
        self._robot_model = robot_model
        self._generated_config_path: Optional[Path] = None
        self._max_push_steps: Optional[int] = None
        # Path to a YAML file listing manual primitives (object_id, edge_idx,
        # push_steps) to try when goal_strategy == "manual_primitives".
        # Validated lazily at plan()-time so the bridge can still be
        # constructed for non-manual strategies even when this is None.
        self._manual_primitives_file = manual_primitives_file

        # Compute absolute path for primitive_data_dir
        if primitive_data_dir is None:
            # Auto-detect: namo_cpp/data/ relative to this file
            bridge_path = Path(__file__).resolve()
            namo_cpp_dir = resolve_namo_cpp_dir(bridge_path)
            self._primitive_data_dir = str(namo_cpp_dir / "data")
        else:
            self._primitive_data_dir = primitive_data_dir

        self._object_mapping = ObjectMapping()

        # Blocking heat-map of the ranker's first-push scores before each
        # plan. Off by default: it stops the robot until a window closes,
        # which is the point when inspecting and unacceptable unattended.
        self._show_push_scores = bool(show_push_scores)

        # Rotation-safe circular radius from full extents.
        # Robot inflation radius used when generating MuJoCo XML (sphere geom
        # for the robot body). See robot_geometry.effective_robot_radius_cm.
        robot_radius_cm = effective_robot_radius_cm(robot_width_cm, robot_height_cm)
        self._xml_generator = NAMOXMLGenerator(
            scale_factor=scale_factor,
            robot_radius_cm=robot_radius_cm,
            robot_model=robot_model,
        )
        if verbose:
            print(
                f"[NAMOBridge] Robot size: {robot_width_cm}x{robot_height_cm}cm "
                f"-> rotation-safe radius={robot_radius_cm:.3f}cm"
            )

        # Align namo_cpp planning.robot_size (half-extents, meters in scaled frame)
        # to runtime robot dimensions so C++ wavefront inflation matches robot_control.
        self._effective_namo_config_path = self._build_effective_namo_config()

        # Ensure namo_cpp python path is available
        self._setup_namo_path()

        # Lazy import planning service
        self._planning_service = None

        # Timing from last plan_from_xml() call
        self.last_search_time_ms: float = 0.0
        self.last_sim_pushes_tried: Optional[int] = None

        # Algorithm-specific diagnostics from the last plan() call. Populated
        # whenever the planner attaches stats to its PlannerResult. Consumed
        # by NAMOPlanner to render a diagnostic failure summary.
        self.last_algorithm_stats: Optional[Dict[str, Any]] = None

        # XML content of the scene fed to the planner in the most recent plan()
        # call. Held in memory (not the temp file, which is deleted on
        # plan()-exit) so the per-plan sim-success capture in NAMOPlanner can
        # snapshot it as the start-of-replay scene. None until the first plan.
        self.last_xml_content: Optional[str] = None

    def __del__(self) -> None:
        self._cleanup_generated_config()

    def _cleanup_generated_config(self) -> None:
        if self._generated_config_path is None:
            return
        try:
            # The config now lives in its own directory alongside the copied
            # wavefront sidecar, so removing just the file would leak the rest.
            shutil.rmtree(self._generated_config_path.parent, ignore_errors=True)
        except Exception:
            pass
        self._generated_config_path = None

    def _resolve_namo_config_path(self, path_str: str) -> Path:
        path = Path(path_str)
        if path.exists():
            return path.resolve()

        bridge_path = Path(__file__).resolve()
        namo_cpp_dir = resolve_namo_cpp_dir(bridge_path)
        candidate = namo_cpp_dir.parent / path_str
        if candidate.exists():
            return candidate.resolve()

        candidate2 = namo_cpp_dir / path_str
        if candidate2.exists():
            return candidate2.resolve()

        return path

    def _build_effective_namo_config(self) -> str:
        source_path = self._resolve_namo_config_path(self._namo_config_path)
        if not source_path.exists():
            if self._verbose:
                print(f"[NAMOBridge] Config not found, using raw path: {self._namo_config_path}")
            return self._namo_config_path

        try:
            with open(source_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}

            planning = cfg.setdefault("planning", {})
            motion_primitives = cfg.get("motion_primitives") or {}
            skill_cfg = cfg.get("skill") or {}
            raw_max_push_steps = (
                motion_primitives.get("max_push_steps")
                or skill_cfg.get("max_push_steps")
            )
            try:
                self._max_push_steps = int(raw_max_push_steps) if raw_max_push_steps is not None else None
            except (TypeError, ValueError):
                self._max_push_steps = None
            half_x_m, half_y_m = scaled_half_extents_m_from_full_extents_cm(
                self._robot_width_cm,
                self._robot_height_cm,
                self._scale_factor,
            )
            planning["robot_size"] = [half_x_m, half_y_m]

            # namo_cpp finds wavefront_inflation.yaml by looking beside the
            # primary config, then in parent `config/` directories
            # (config_manager.cpp:284). A bare mkstemp lands this file in /tmp,
            # where neither ever matches, so every run silently fell back to a
            # built-in inflation margin and logged
            # "wavefront_inflation.yaml not found near /tmp". Give the runtime
            # config its own directory and carry the sidecar in with it, so the
            # margin comes from the same file namo_cpp would have used.
            tmp_dir = Path(tempfile.mkdtemp(prefix="namo_config_runtime_"))
            tmp_path = tmp_dir / "namo_config_runtime.yaml"
            tmp_path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
            sidecar = source_path.parent / WAVEFRONT_INFLATION_SIDECAR
            if sidecar.is_file():
                shutil.copy2(sidecar, tmp_dir / WAVEFRONT_INFLATION_SIDECAR)
            elif self._verbose:
                print(f"[NAMOBridge] No {WAVEFRONT_INFLATION_SIDECAR} beside {source_path}")
            self._generated_config_path = tmp_path

            if self._verbose:
                print(
                    f"[NAMOBridge] Runtime config override: planning.robot_size="
                    f"[{half_x_m:.6f}, {half_y_m:.6f}] m (half-extents, scaled)"
                )
                if self._max_push_steps is not None:
                    print(f"[NAMOBridge] Runtime config max_push_steps={self._max_push_steps}")
                print(f"[NAMOBridge] Effective NAMO config: {self._generated_config_path}")

            return str(self._generated_config_path)
        except Exception as e:
            if self._verbose:
                print(f"[NAMOBridge] Failed to build runtime config override: {e}")
            return str(source_path)

    def _setup_namo_path(self) -> None:
        """Add namo_cpp python paths to sys.path."""
        ensure_namo_cpp_paths(Path(__file__).resolve())

    def _assert_canonical_namo_rl_import(self) -> None:
        load_canonical_namo_rl(Path(__file__).resolve())

    def _get_planning_service(self):
        """Lazy import and create planning service."""
        if self._planning_service is None:
            self._assert_canonical_namo_rl_import()
            from namo.services import NAMOPlanningService

            self._planning_service = NAMOPlanningService(
                config_path=self._effective_namo_config_path,
                primitive_data_dir=self._primitive_data_dir,
                verbose=self._verbose,
                enable_viewer=self._enable_viewer,
                pause_after_load=self._pause_after_load,
            )
        return self._planning_service

    def _starting_robot_pose_sim(
        self,
        observation: Observation,
    ) -> Optional[Tuple[float, float, float]]:
        """Return planner-consistent robot start pose in sim units."""
        if self._robot_model != "car":
            return None
        rx_m, ry_m = self._cm_to_sim(
            float(observation.robot_x),
            float(observation.robot_y),
        )
        return (rx_m, ry_m, math.radians(float(observation.robot_theta)))

    def _serialize_object_mapping(self) -> Dict[str, Dict[str, str]]:
        """Return the current mapping in a JSON/YAML-friendly form."""
        return {
            "real_to_sim": dict(self._object_mapping.real_to_sim),
            "sim_to_real": dict(self._object_mapping.sim_to_real),
        }

    def _chain_entry_to_push_subgoal(self, entry: Any) -> PushSubgoal:
        """Normalize a chain entry to PushSubgoal in real-object naming."""
        if isinstance(entry, PushSubgoal):
            if int(entry.push_steps) < 1:
                raise ValueError(
                    f"push_steps must be >= 1 for {entry.object_id} edge={entry.edge_idx}"
                )
            return PushSubgoal(
                object_id=str(entry.object_id),
                edge_idx=int(entry.edge_idx),
                push_steps=int(entry.push_steps),
            )
        if isinstance(entry, dict):
            object_id = str(entry["object_id"])
            edge_idx = int(entry["edge_idx"])
            push_steps = int(entry["push_steps"])
            if push_steps < 1:
                raise ValueError(
                    f"push_steps must be >= 1 for {object_id} edge={edge_idx}"
                )
            return PushSubgoal(
                object_id=object_id,
                edge_idx=edge_idx,
                push_steps=push_steps,
            )
        raise TypeError(f"unsupported chain entry type: {type(entry)!r}")

    def _resolve_sim_object_id(self, object_id: str) -> str:
        """Resolve a chain entry object ID to the planner scene's sim body ID."""
        if object_id in self._object_mapping.real_to_sim:
            return self._object_mapping.real_to_sim[object_id]
        if object_id in self._object_mapping.sim_to_real:
            return object_id
        raise KeyError(
            f"object_id {object_id!r} is not present in mapping "
            f"(real={sorted(self._object_mapping.real_to_sim)}, "
            f"sim={sorted(self._object_mapping.sim_to_real)})"
        )

    def _build_rl_env_for_scene(
        self,
        namo_rl: Any,
        xml_path: str,
        starting_robot_pose: Optional[Tuple[float, float, float]],
    ):
        """Construct RLEnvironment exactly like the planner path does."""
        if starting_robot_pose is None:
            return namo_rl.RLEnvironment(
                xml_path,
                self._effective_namo_config_path,
                False,
            )

        env = namo_rl.RLEnvironment(
            xml_path,
            self._effective_namo_config_path,
            False,
            True,
        )
        env.set_robot_pose(*starting_robot_pose)
        env.warm_up()
        return env

    def verify_chain(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
        chain: Sequence[Any],
        *,
        target_points: Optional[Sequence[Tuple[float, float]]] = None,
        min_reachable: Optional[int] = None,
    ) -> ChainVerificationResult:
        """Sim-verify an exact push chain from the current observation.

        Returns success only if every push succeeds and the final state makes
        the robot goal reachable under the planner's C++ wavefront check.
        """
        t0 = perf_counter()
        planner_scene_xml = self._generate_xml(observation, robot_goal_cm)
        if planner_scene_xml is None:
            return ChainVerificationResult(
                success=False,
                verified_subgoals=[],
                sim_pushes_tried=0,
                failed_step_index=None,
                failure_reason="failed_to_generate_xml",
                goal_reachable_after=False,
                verification_time_ms=(perf_counter() - t0) * 1000.0,
                planner_scene_xml="",
                object_mapping=self._serialize_object_mapping(),
            )

        self.last_xml_content = planner_scene_xml
        xml_path = self._write_xml(planner_scene_xml)
        if xml_path is None:
            return ChainVerificationResult(
                success=False,
                verified_subgoals=[],
                sim_pushes_tried=0,
                failed_step_index=None,
                failure_reason="failed_to_write_xml",
                goal_reachable_after=False,
                verification_time_ms=(perf_counter() - t0) * 1000.0,
                planner_scene_xml=planner_scene_xml,
                object_mapping=self._serialize_object_mapping(),
            )

        normalized_chain: List[PushSubgoal] = []
        try:
            for entry in chain:
                normalized_chain.append(self._chain_entry_to_push_subgoal(entry))
        except (KeyError, TypeError, ValueError) as exc:
            if self._debug_xml_path is None:
                try:
                    Path(xml_path).unlink()
                except OSError:
                    pass
            return ChainVerificationResult(
                success=False,
                verified_subgoals=[],
                sim_pushes_tried=0,
                failed_step_index=None,
                failure_reason=f"invalid_chain_entry: {exc}",
                goal_reachable_after=False,
                verification_time_ms=(perf_counter() - t0) * 1000.0,
                planner_scene_xml=planner_scene_xml,
                object_mapping=self._serialize_object_mapping(),
            )

        bridge_path = Path(__file__).resolve()
        namo_cpp_dir = resolve_namo_cpp_dir(bridge_path)
        original_cwd = os.getcwd()
        os.chdir(str(namo_cpp_dir))

        try:
            namo_rl, _, _ = load_canonical_namo_rl(bridge_path)
            starting_robot_pose = self._starting_robot_pose_sim(observation)
            env = self._build_rl_env_for_scene(namo_rl, xml_path, starting_robot_pose)
            goal_sim = self._cm_to_sim(robot_goal_cm[0], robot_goal_cm[1])
            env.set_robot_goal(goal_sim[0], goal_sim[1], 0.0)

            if target_points:
                # A chain is graded by whether the boundary is open AFTER it.
                # If it is already open before anything runs, that test proves
                # nothing and any stale chain would pass, so the robot would
                # execute a push against a boundary already finished. Refuse,
                # and let the caller re-solve; it will see already_open and move
                # to the next boundary.
                open_before, _goal_before, _t = evaluate_chain_outcome(
                    env, target_points, min_reachable
                )
                if open_before:
                    return ChainVerificationResult(
                        success=False,
                        verified_subgoals=[],
                        sim_pushes_tried=0,
                        failed_step_index=None,
                        failure_reason="target_already_open_before_chain",
                        goal_reachable_after=_goal_before,
                        target_open_after=True,
                        verification_time_ms=(perf_counter() - t0) * 1000.0,
                        planner_scene_xml=planner_scene_xml,
                        object_mapping=self._serialize_object_mapping(),
                    )

            verified_subgoals: List[PushSubgoal] = []
            for idx, subgoal in enumerate(normalized_chain):
                sim_object_id = self._resolve_sim_object_id(subgoal.object_id)
                action = namo_rl.Action()
                action.object_id = sim_object_id
                action.edge_idx = int(subgoal.edge_idx)
                action.depth = int(subgoal.push_steps) - 1
                action.x = 0.0
                action.y = 0.0
                action.theta = 0.0

                try:
                    step_result = env.step(action)
                except Exception as exc:
                    return ChainVerificationResult(
                        success=False,
                        verified_subgoals=verified_subgoals,
                        sim_pushes_tried=idx + 1,
                        failed_step_index=idx,
                        failure_reason=f"env_step_exception: {exc!r}",
                        goal_reachable_after=False,
                        verification_time_ms=(perf_counter() - t0) * 1000.0,
                        planner_scene_xml=planner_scene_xml,
                        object_mapping=self._serialize_object_mapping(),
                    )

                info = getattr(step_result, "info", {}) or {}
                if not bool(getattr(step_result, "done", False)):
                    failure_reason = str(
                        info.get("failure_reason")
                        or f"step_returned_done_false_for_{sim_object_id}"
                    )
                    return ChainVerificationResult(
                        success=False,
                        verified_subgoals=verified_subgoals,
                        sim_pushes_tried=idx + 1,
                        failed_step_index=idx,
                        failure_reason=failure_reason,
                        goal_reachable_after=False,
                        verification_time_ms=(perf_counter() - t0) * 1000.0,
                        planner_scene_xml=planner_scene_xml,
                        object_mapping=self._serialize_object_mapping(),
                    )

                verified_subgoals.append(subgoal)

            succeeded, goal_reachable_after, target_open_after = evaluate_chain_outcome(
                env, target_points, min_reachable
            )
            if not succeeded:
                return ChainVerificationResult(
                    success=False,
                    verified_subgoals=verified_subgoals,
                    sim_pushes_tried=len(normalized_chain),
                    failed_step_index=None,
                    failure_reason=(
                        "target_not_open_after_chain"
                        if target_points
                        else "goal_not_reachable_after_chain"
                    ),
                    goal_reachable_after=goal_reachable_after,
                    target_open_after=target_open_after,
                    verification_time_ms=(perf_counter() - t0) * 1000.0,
                    planner_scene_xml=planner_scene_xml,
                    object_mapping=self._serialize_object_mapping(),
                )

            return ChainVerificationResult(
                success=True,
                verified_subgoals=verified_subgoals,
                sim_pushes_tried=len(normalized_chain),
                failed_step_index=None,
                failure_reason=None,
                goal_reachable_after=goal_reachable_after,
                target_open_after=target_open_after,
                verification_time_ms=(perf_counter() - t0) * 1000.0,
                planner_scene_xml=planner_scene_xml,
                object_mapping=self._serialize_object_mapping(),
            )
        except Exception as exc:
            return ChainVerificationResult(
                success=False,
                verified_subgoals=[],
                sim_pushes_tried=0,
                failed_step_index=None,
                failure_reason=f"verification_exception: {exc!r}",
                goal_reachable_after=False,
                verification_time_ms=(perf_counter() - t0) * 1000.0,
                planner_scene_xml=planner_scene_xml,
                object_mapping=self._serialize_object_mapping(),
            )
        finally:
            os.chdir(original_cwd)
            if self._debug_xml_path is None:
                try:
                    Path(xml_path).unlink()
                except OSError:
                    pass

    def plan(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
        algorithm: str = "full_namo",
        goal_strategy: str = "primitive",
        max_chain_depth: int = 2,
        frontier_beam_width: int = 10000,
        chain_link_cost: int = 11,
        selection_strategy: str = "cost_first",
        goals_per_region: Optional[int] = None,
        failed_pushes: Optional[Set[Tuple[str, int]]] = None,
        **kwargs,
    ) -> List[PushSubgoal]:
        """Plan push actions from current observation.

        Args:
            observation: Current robot and object state
            robot_goal_cm: Goal position in cm (x, y)
            algorithm: Planning algorithm
            goal_strategy: Goal sampling strategy ("primitive", "ml", etc.)
            max_chain_depth: Maximum chain depth for multi-push solutions (1 or 2)
            frontier_beam_width: Beam width for frontier search
            chain_link_cost: Additional cost per chain link
            selection_strategy: Frontier priority ("cost_first" or "ml_first")
            goals_per_region: Goal samples per region for validation.
                None defers to namo_cpp's canonical 100.
            failed_pushes: Set of (real_object_id, edge_idx) pairs to blacklist.
                If any action in the returned plan matches a blacklisted pair,
                the entire plan is discarded (returns []). This is the failure-
                feedback mechanism: previously failed pushes are never proposed
                to the executor again within an episode.
            **kwargs: Additional algorithm parameters

        Returns:
            List of PushSubgoals to execute
        """
        # Reset planning-time telemetry on entry so early-return failure paths
        # don't leave a stale value from a previous successful call. The success
        # path overwrites this at line ~351 after the service call. BUG-019.
        self.last_search_time_ms = 0.0
        self.last_sim_pushes_tried = None
        self.last_algorithm_stats = None

        # Manual-primitives strategy: bypass the C++ planner entirely.
        # Load a YAML file of (object_id, edge_idx, push_steps) entries,
        # apply them in sequence against a fresh RLEnvironment built from
        # the same XML, and return PushSubgoals only if the post-chain
        # state has the robot goal reachable. Failure → empty list (the
        # runtime treats this as "planning failed" and aborts the run).
        if goal_strategy == "manual_primitives":
            return self._simulate_manual_chain(
                observation=observation,
                robot_goal_cm=robot_goal_cm,
            )

        # Generate XML and object mapping
        xml_content = self._generate_xml(observation, robot_goal_cm)
        if xml_content is None:
            if self._verbose:
                print("[NAMOBridge] Failed to generate XML")
            return []

        # Cache content so the per-plan sim-success capture (in NAMOPlanner)
        # can snapshot the scene that produced this plan, even after the temp
        # file is unlinked below.
        self.last_xml_content = xml_content

        # Write XML to temp file
        xml_path = self._write_xml(xml_content)
        if xml_path is None:
            return []

        # Change to namo_cpp directory so relative paths in config work
        # (e.g., motion_primitives_file: "data/motion_primitives_15.dat")
        bridge_path = Path(__file__).resolve()
        namo_cpp_dir = resolve_namo_cpp_dir(bridge_path)
        original_cwd = os.getcwd()
        os.chdir(str(namo_cpp_dir))

        try:
            # Convert goal to simulation coordinates
            goal_sim = self._cm_to_sim(robot_goal_cm[0], robot_goal_cm[1])

            # Get planning service (and preload ML model if needed)
            service = self._get_planning_service()
            service.preload_goal_model(goal_strategy, **kwargs)

            # Real-robot execution: enable region-opener early exit once a single
            # candidate object yields a successful opening for a neighbor. The outer
            # FullNAMOPlanner handles the next region on the path, so additional
            # candidates per neighbor are pure waste at execution time. Callers can
            # override via kwargs.
            kwargs.setdefault("region_early_exit_on_first_success", True)

            # Forward the runtime's failed-push blacklist into the planner so it
            # skips those edges from the start, instead of relying on the
            # post-hoc filter below to throw away whole plans. Keys must use
            # the planner's SIM object naming, not the runtime's real naming.
            if self._max_push_steps is not None:
                kwargs.setdefault("max_push_steps", int(self._max_push_steps))
            if failed_pushes:
                external_blacklist: Dict[str, Set[int]] = {}
                for real_id, edge_idx in failed_pushes:
                    sim_id = self._object_mapping.get_sim_name(real_id)
                    external_blacklist.setdefault(sim_id, set()).add(int(edge_idx))
                existing = kwargs.get("external_edge_blacklist") or {}
                merged: Dict[str, Set[int]] = {k: set(v) for k, v in existing.items()}
                for k, edges in external_blacklist.items():
                    merged.setdefault(k, set()).update(edges)
                kwargs["external_edge_blacklist"] = merged

            # Robot starting pose in sim meters + radians. Only matters
            # for the car body — the sphere XML bakes pose into the geom
            # and the planning_service's defer-warmup branch is a no-op
            # without this arg. We always pass it for car so the env
            # constructor skips its 3-tick warm-up, the service teleports
            # to this pose, then warms up cleanly from the right state.
            starting_robot_pose = self._starting_robot_pose_sim(observation)

            # Show the ranker's view and wait, when asked. This is the last
            # point where every input exists together: the scene XML the
            # planner will read, the checkpoint, the goal and the start pose.
            if self._show_push_scores:
                self._show_push_score_plot(
                    xml_path, goal_sim, starting_robot_pose,
                    kwargs.get("scorer_ckpt"), max_chain_depth,
                    kwargs.get("external_edge_blacklist") or {},
                )

            # Run planning
            result = service.plan_from_xml(
                xml_path=xml_path,
                robot_goal=(goal_sim[0], goal_sim[1], 0.0),
                algorithm=algorithm,
                goal_strategy=goal_strategy,
                max_chain_depth=max_chain_depth,
                frontier_beam_width=frontier_beam_width,
                chain_link_cost=chain_link_cost,
                selection_strategy=selection_strategy,
                goals_per_region=goals_per_region,
                starting_robot_pose=starting_robot_pose,
                **kwargs,
            )

            self.last_search_time_ms = result.search_time_ms
            self.last_algorithm_stats = getattr(result, "algorithm_stats", None)
            total_primitives_attempted = None
            if isinstance(self.last_algorithm_stats, dict):
                raw_total = self.last_algorithm_stats.get("total_primitives_attempted")
                if raw_total is not None:
                    try:
                        total_primitives_attempted = int(raw_total)
                    except (TypeError, ValueError):
                        total_primitives_attempted = None
            self.last_sim_pushes_tried = total_primitives_attempted

            if self._verbose:
                print(
                    f"[NAMOBridge] Planning result: success={result.success}, "
                    f"actions={len(result.actions)}, time={result.search_time_ms:.1f}ms"
                )
                if result.error_message:
                    print(f"[NAMOBridge] Error: {result.error_message}")

            # Bug fix: full_namo's _failure_result returns success=False with a
            # partial action_sequence for diagnostics. Without this guard the
            # bridge would forward those partial pushes as if they were a
            # complete plan, and the runtime/--sim-xml path would then record
            # them in solution.yaml as success=True because the only success
            # signal downstream was `bool(plan_list)`. Reject failed plans here.
            if not result.success:
                if self._verbose:
                    print(
                        f"[NAMOBridge] Planner reported success=False; "
                        f"discarding {len(result.actions)} partial action(s)."
                    )
                return []

            # Apply blacklist filter: drop the entire plan if any action's
            # (real_object_id, edge_idx) is in the blacklist. We discard the
            # whole plan rather than individual actions because subsequent
            # pushes are conditioned on earlier ones; cherry-picking would
            # break the chain semantics. The caller will retry with a new
            # random seed (different edge ordering) and eventually abort if
            # nothing valid is found.
            if failed_pushes:
                blacklisted_in_plan = []
                for action in result.actions:
                    real_id = self._object_mapping.get_real_name(action.object_id)
                    if (real_id, action.edge_idx) in failed_pushes:
                        blacklisted_in_plan.append((real_id, action.edge_idx))
                if blacklisted_in_plan:
                    if self._verbose:
                        print(
                            f"[NAMOBridge] Plan contains {len(blacklisted_in_plan)} "
                            f"blacklisted push(es) {blacklisted_in_plan}; "
                            f"discarding plan to force retry"
                        )
                    return []

            # Convert to PushSubgoals
            return self._convert_to_subgoals(result.actions)

        finally:
            # Restore original working directory
            os.chdir(original_cwd)

            # Clean up temp file (unless debug path was provided)
            if self._debug_xml_path is None:
                try:
                    Path(xml_path).unlink()
                except OSError:
                    pass

    def analyze_reachability(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
        analysis_mode: bool = False,
    ) -> Dict[str, Any]:
        """Compute unified C++ reachability summary for the current observation."""
        xml_content = self._generate_xml(observation, robot_goal_cm)
        if xml_content is None:
            return {
                "goal_reachable": False,
                "analysis_mode": analysis_mode,
                "objects": {},
                "error_message": "Failed to generate XML",
            }

        xml_path = self._write_xml(xml_content)
        if xml_path is None:
            return {
                "goal_reachable": False,
                "analysis_mode": analysis_mode,
                "objects": {},
                "error_message": "Failed to write XML",
            }

        bridge_path = Path(__file__).resolve()
        namo_cpp_dir = resolve_namo_cpp_dir(bridge_path)
        original_cwd = os.getcwd()
        os.chdir(str(namo_cpp_dir))

        try:
            goal_sim = self._cm_to_sim(robot_goal_cm[0], robot_goal_cm[1])
            service = self._get_planning_service()

            # Same pose-conversion pattern as bridge.plan(): the car body
            # spawns at little_car.xml's fixed pos, so reachability has to
            # be computed AFTER teleporting to the observation pose. Without
            # this, _is_goal_reachable always answers from the XML spawn
            # (typically (0,0)) and the planner never sees "goal reachable"
            # after a successful push — it falls into NAMO planning, which
            # short-circuits with 0 actions because the goal IS reachable
            # from the actual pose, and the run aborts as "all attempts
            # failed". Sphere XMLs bake pose into the geom; skip.
            starting_robot_pose = self._starting_robot_pose_sim(observation)

            summary = service.analyze_reachability_from_xml(
                xml_path=xml_path,
                robot_goal=(goal_sim[0], goal_sim[1], 0.0),
                analysis_mode=analysis_mode,
                starting_robot_pose=starting_robot_pose,
            )
            if self._verbose and summary.get("error_message"):
                print(f"[NAMOBridge] Reachability error: {summary['error_message']}")
            return summary
        finally:
            os.chdir(original_cwd)
            if self._debug_xml_path is None:
                try:
                    Path(xml_path).unlink()
                except OSError:
                    pass

    @contextmanager
    def _scene_session(self, observation: Observation, robot_goal_cm: Tuple[float, float]):
        """Build the scene and enter namo_cpp's directory for the call.

        Yields ``(xml_path, goal_sim, starting_robot_pose, service)``, or
        ``None`` when the scene could not be generated.

        The pose is not optional for the car. Its MuJoCo model has a fixed
        freejoint spawn at (0, 0, 0.01), so without it the service builds the
        region graph around a robot at the origin and picks the wrong boundary. The chdir is required because namo_cpp resolves
        motion-primitive paths relative to its own root, and the temp XML is
        removed afterwards unless a debug path was configured.

        ``plan()`` predates this and still inlines the same sequence; it is left
        alone deliberately, as the live path, rather than refactored alongside a
        feature change.
        """
        xml_content = self._generate_xml(observation, robot_goal_cm)
        if xml_content is None:
            yield None
            return
        self.last_xml_content = xml_content

        xml_path = self._write_xml(xml_content)
        if xml_path is None:
            yield None
            return

        namo_cpp_dir = resolve_namo_cpp_dir(Path(__file__).resolve())
        original_cwd = os.getcwd()
        os.chdir(str(namo_cpp_dir))
        try:
            goal_sim = self._cm_to_sim(robot_goal_cm[0], robot_goal_cm[1])
            starting_robot_pose = self._starting_robot_pose_sim(observation)
            yield xml_path, goal_sim, starting_robot_pose, self._get_planning_service()
        finally:
            os.chdir(original_cwd)
            if self._debug_xml_path is None:
                try:
                    Path(xml_path).unlink()
                except OSError:
                    pass

    def select_boundary(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
        *,
        blocked_boundaries: Optional[Sequence[Tuple[str, str]]] = None,
        **kwargs: Any,
    ) -> BoundaryChoice:
        """Ask which boundary to open next, translated into real object names.

        The blocking objects come back in simulator naming, which is a rank over
        the movables present in this one observation. They are mapped to real
        ids here because that is what survives a rescan.
        """
        with self._scene_session(observation, robot_goal_cm) as session:
            if session is None:
                return BoundaryChoice(failure_reason="xml_generation_failed")
            xml_path, goal_sim, starting_robot_pose, service = session

            selection = service.select_boundary_from_xml(
                xml_path,
                (goal_sim[0], goal_sim[1], 0.0),
                blocked_boundaries=blocked_boundaries,
                starting_robot_pose=starting_robot_pose,
                **kwargs,
            )
            return self._choice_from_selection(selection)

    def _choice_from_selection(self, selection: Any) -> BoundaryChoice:
        """Translate namo_cpp's BoundarySelection into real-object naming.

        Every branch carries stale_blocked_boundaries. It is most useful on the
        paths that found nothing, because "no route" and "no route once I
        excluded pairs that no longer exist" are different problems and used to
        look identical from here.
        """
        stale = [
            (str(a), str(b))
            for a, b in getattr(selection, "stale_blocked_boundaries", ())
        ]
        if selection.goal_already_reachable:
            return BoundaryChoice(
                goal_already_reachable=True, stale_blocked_boundaries=stale
            )
        if not selection.found:
            return BoundaryChoice(
                failure_reason=selection.failure_reason,
                stale_blocked_boundaries=stale,
            )

        return BoundaryChoice(
            found=True,
            target_points_m=[tuple(p) for p in selection.target_points],
            blocker_real_ids=[
                self._object_mapping.get_real_name(sim_id)
                for sim_id in selection.blocking_objects
            ],
            region_path=list(selection.region_path),
            stale_blocked_boundaries=stale,
            failure_reason="",
        )

    def solve_boundary(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
        target: Any,
        **kwargs: Any,
    ) -> BoundaryPlan:
        """Open one pinned boundary, graded against the target's frozen points.

        ``target`` is a RegionOpeningTarget. Its blockers are stored in real
        naming and resolved to simulator ids against the mapping built for
        *this* scene, because those ids shift whenever the set of visible
        movables changes.
        """
        with self._scene_session(observation, robot_goal_cm) as session:
            if session is None:
                return BoundaryPlan(failure_reason="xml_generation_failed")
            xml_path, goal_sim, starting_robot_pose, service = session

            target_scale = getattr(target, "scale_factor", None)
            if target_scale is not None and float(target_scale) != float(self._scale_factor):
                # target_samples_m are cm/100 times the scale factor. Grading
                # them under a different scale silently compares points in one
                # frame against a world built in another.
                if self._verbose:
                    print(
                        f"[NAMOBridge] Active target was recorded at scale "
                        f"{target_scale}, this bridge runs at {self._scale_factor}"
                    )
                return BoundaryPlan(failure_reason="scale_factor_mismatch")

            solve_kwargs = target.as_solve_kwargs()
            blocking_sim_ids = []
            for real_id in solve_kwargs.pop("blocking_objects", []):
                try:
                    blocking_sim_ids.append(self._resolve_sim_object_id(real_id))
                except KeyError:
                    # The blocker is not in this observation -- occluded, or it
                    # left the scene. The boundary cannot be identified from it,
                    # so report rather than silently opening a different one.
                    return BoundaryPlan(failure_reason="blocker_not_observed")

            if target.failed_pushes:
                blacklist: Dict[str, Set[int]] = {}
                for real_id, edge_idx in target.failed_pushes:
                    try:
                        sim_id = self._resolve_sim_object_id(real_id)
                    except KeyError:
                        continue
                    blacklist.setdefault(sim_id, set()).add(int(edge_idx))
                if blacklist:
                    kwargs["external_edge_blacklist"] = blacklist

            # LocalSearchConfig emits full_namo_local_search, the key full_namo
            # reads out of algorithm_params. solve_boundary_from_xml selects the
            # opener from its own local_search parameter instead, so without this
            # rename the choice lands in algorithm_params and is ignored while
            # the startup banner reports it as applied.
            if "full_namo_local_search" in kwargs:
                kwargs["local_search"] = kwargs.pop("full_namo_local_search")

            result = service.solve_boundary_from_xml(
                xml_path,
                (goal_sim[0], goal_sim[1], 0.0),
                blocking_objects=blocking_sim_ids,
                starting_robot_pose=starting_robot_pose,
                **solve_kwargs,
                **kwargs,
            )
            self.last_search_time_ms = result.search_time_ms
            self.last_algorithm_stats = {
                "resolved_target": result.resolved_target,
                "simulations_used": result.simulations_used,
                "target_summary": result.target_summary,
                # Read back from the call that was actually made, not from
                # config. With execution mode crossed against the ranker arm,
                # this is a grouping key for a paired analysis, and a row whose
                # arm has to be recovered by parsing a command string is a row
                # that can be recovered wrong.
                "exec_mode": str(kwargs.get("mode", DEFAULT_EXEC_MODE)),
            }

            return BoundaryPlan(
                subgoals=self._convert_to_subgoals(result.actions),
                success=bool(result.success),
                already_open=bool(result.already_open),
                boundary_exhausted=bool(result.boundary_exhausted),
                failure_reason=result.failure_reason,
                resolved_source=str(getattr(result, "resolved_source", "") or ""),
                resolved_target=result.resolved_target,
            )

    def _generate_xml(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
    ) -> Optional[str]:
        """Generate MuJoCo XML from observation.

        Builds object mapping during generation.
        """
        # Clear previous mapping
        self._object_mapping.clear()

        # Build objects dict for XML generator
        # Format: (x_cm, y_cm, theta_deg, width_cm, depth_cm, height_cm, is_static)
        objects: Dict[str, Tuple[float, float, float, float, float, float, bool]] = {}

        # Sort object names for deterministic mapping
        movable_idx = 0
        for name in sorted(observation.objects.keys()):
            obj = observation.objects[name]

            # Add to objects dict
            objects[name] = (
                obj.x,
                obj.y,
                obj.theta,
                obj.width,
                obj.depth,
                obj.height,
                obj.is_static,
            )

            # Build mapping for movable objects
            if not obj.is_static:
                movable_idx += 1
                sim_name = f"obstacle_{movable_idx}_movable"
                self._object_mapping.add(name, sim_name)

        if self._verbose:
            print(f"[NAMOBridge] Object mapping: {self._object_mapping.real_to_sim}")
            print(f"[NAMOBridge] Objects for XML:")
            for name, (x, y, theta, w, d, h, static) in objects.items():
                static_str = "STATIC" if static else "movable"
                print(f"[NAMOBridge]   {name}: pos=({x:.1f},{y:.1f}) theta={theta:.1f}° size=({w:.1f}x{d:.1f}) [{static_str}]")

        if self._verbose:
            print(f"[NAMOBridge] Robot: ({observation.robot_x:.1f}, {observation.robot_y:.1f}) cm")
            print(f"[NAMOBridge] Goal: ({robot_goal_cm[0]:.1f}, {robot_goal_cm[1]:.1f}) cm")
            print(f"[NAMOBridge] Workspace: {WORKSPACE_WIDTH_CM}x{WORKSPACE_HEIGHT_CM} cm")

        try:
            xml_content = self._xml_generator.from_observation(
                robot_x_cm=observation.robot_x,
                robot_y_cm=observation.robot_y,
                objects=objects,
                goal_x_cm=robot_goal_cm[0],
                goal_y_cm=robot_goal_cm[1],
                workspace_bounds_cm=(
                    0.0,
                    WORKSPACE_WIDTH_CM,
                    0.0,
                    WORKSPACE_HEIGHT_CM,
                ),
            )
            return xml_content
        except Exception as e:
            if self._verbose:
                print(f"[NAMOBridge] XML generation failed: {e}")
            return None



    def _sim_object_half_extents_cm(self, sim_name: str, half: bool = False):
        """Object extents in cm, read from the scene XML this bridge wrote.

        MuJoCo geom ``size`` is half-extents in metres. Returns full extents by
        default, matching ObjectPose.depth/width; ``half=True`` returns halves,
        which is what a rectangle patch wants. Falls back to a small square
        rather than raising, since this only feeds a diagnostic plot.
        """
        import re
        pattern = rf'name="{re.escape(sim_name)}"[^>]*size="([0-9.eE+-]+) ([0-9.eE+-]+)'
        match = re.search(pattern, self.last_xml_content or "")
        if not match:
            return (2.5, 2.5)
        hx, hy = float(match.group(1)) * 100.0, float(match.group(2)) * 100.0
        return (hx, hy) if half else (2 * hx, 2 * hy)

    def _show_push_score_plot(self, xml_path, goal_sim, start_pose,
                              scorer_ckpt, max_chain_depth, blacklist) -> None:
        """Score every first push and display it, blocking until closed.

        Costs one extra scoring pass per plan, which is the price of seeing
        what the search is about to act on. Never raises.
        """
        if not scorer_ckpt:
            print("[PushScores] no scorer checkpoint, nothing to plot", flush=True)
            return
        try:
            import math
            from types import SimpleNamespace
            import namo_rl
            from namo.runtime_profile import CANONICAL_PRIMITIVE_PREFIX
            from namo.planners.opening.best_first_search import rank_first_pushes_h2
            from namo.strategies.primitive_goal_strategy import PrimitiveGoalStrategy
            from namo.strategies.scorer_goal_strategy import _get_scorer
            from robot_control.diagnostics.push_score_plot import show_push_scores

            config = str(self._effective_namo_config_path)
            env = namo_rl.RLEnvironment(xml_path, config, False)
            env.reset()
            env.set_robot_pose(*start_pose)
            state = env.get_full_state()
            obs = env.get_observation()

            planner = SimpleNamespace(
                prim=PrimitiveGoalStrategy(
                    data_dir=str(self._primitive_data_dir),
                    primitive_prefix=CANONICAL_PRIMITIVE_PREFIX),
                scorer=_get_scorer(str(scorer_ckpt), config, "cpu"))
            _ranked, grid = rank_first_pushes_h2(
                planner, env, (goal_sim[0], goal_sim[1], 0.0), xml_path,
                state, max_chain_depth, score=True, return_grid=True)
            if not grid:
                print("[PushScores] empty grid, nothing to plot", flush=True)
                return

            target = next(iter(blacklist), None) or next(
                (k[:-5] for k in obs if k.endswith("_pose") and "movable" in k), None)
            if target is None:
                return
            pose = obs[f"{target}_pose"]
            size = self._sim_object_half_extents_cm(target)
            # Everything in the scene except the object being scored, with its
            # real rotation. The arena border walls 1-4 are the workspace edge,
            # already drawn as the boundary, so they would only clutter it.
            others = {}
            for key in obs:
                if not key.endswith("_pose") or key == "robot_pose":
                    continue
                name = key[:-5]
                if name == target:
                    continue
                if name.startswith("wall_") and name.split("_")[-1].isdigit() \
                        and int(name.split("_")[-1]) <= 4:
                    continue
                ex, ey = self._sim_object_half_extents_cm(name)
                others[name] = (obs[key][0] * 100, obs[key][1] * 100,
                                math.degrees(obs[key][2]), ex, ey,
                                name.startswith("wall_"))
            # The actual network input, rendered by the same LiveScorer the
            # ranker uses. Everything else in the figure is a readable stand-in
            # for these five rasters; only these are what the model saw.
            masks, mask_names = None, None
            try:
                import sys as _sys
                sandbox = str(resolve_namo_cpp_dir(Path(__file__).resolve())
                              / "scripts" / "sandbox")
                if sandbox not in _sys.path:
                    _sys.path.insert(0, sandbox)
                from live_scorer import LiveScorer, CHANS
                renderer = LiveScorer(ckpt=str(scorer_ckpt), render_config=config,
                                      device="cpu")
                masks, _meta = renderer.render_ctx(
                    env, target, (goal_sim[0], goal_sim[1], 0.0), xml_path)
                mask_names = CHANS
            except Exception as exc:
                print(f"[PushScores] input masks unavailable: {exc!r}", flush=True)

            show_push_scores(
                grid,
                (pose[0] * 100, pose[1] * 100, math.degrees(pose[2])),
                size,
                (obs["robot_pose"][0] * 100, obs["robot_pose"][1] * 100),
                (goal_sim[0] * 100, goal_sim[1] * 100),
                other_objects=others,
                blocked_edges=set(blacklist.get(target, set())),
                input_masks=masks,
                mask_names=mask_names,
                title=f"HY5U first-push score for {target}",
            )
        except Exception as exc:
            print(f"[PushScores] skipped: {exc!r}", flush=True)

    def _write_xml(self, xml_content: str) -> Optional[str]:
        """Write XML content to file."""
        if self._debug_xml_path:
            # Write to debug path
            with open(self._debug_xml_path, "w") as f:
                f.write(xml_content)
            return self._debug_xml_path

        # Write to temp file
        try:
            fd, path = tempfile.mkstemp(suffix=".xml", prefix="namo_env_")
            # Close the descriptor immediately; we re-open `path` by name below.
            # Leaving `fd` open here leaks one FD per call (BUG-018).
            os.close(fd)
            with open(path, "w") as f:
                f.write(xml_content)
            return path
        except Exception as e:
            if self._verbose:
                print(f"[NAMOBridge] Failed to write XML: {e}")
            return None

    def _cm_to_sim(self, x_cm: float, y_cm: float) -> Tuple[float, float]:
        """Convert cm coordinates to simulation coordinates."""
        x_m = x_cm / 100.0 * self._scale_factor
        y_m = y_cm / 100.0 * self._scale_factor
        return (x_m, y_m)

    def _convert_to_subgoals(self, actions) -> List[PushSubgoal]:
        """Convert NAMOActions to PushSubgoals."""
        subgoals = []

        for action in actions:
            # Map simulation object ID to real object ID
            real_object_id = self._object_mapping.get_real_name(action.object_id)

            # Pass the planner's depth through 1:1 (no lossy bucketing).
            # The planner's depth=N means (N+1) primitive applications, each
            # of which corresponds to 2.5s of simulated push (250 mj_steps ×
            # 0.01s timestep). The executor's controller.yaml is calibrated
            # with push_steps=75 ticks (2.5s at 30Hz) per unit so real-robot
            # push duration matches sim across the full 0-9 depth range.
            push_steps = action.depth + 1

            subgoals.append(
                PushSubgoal(
                    object_id=real_object_id,
                    edge_idx=action.edge_idx,
                    push_steps=push_steps,
                )
            )

        return subgoals

    def _simulate_manual_chain(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
    ) -> List[PushSubgoal]:
        """Apply a YAML-specified push chain in sim and return subgoals on success.

        The YAML schema:

            primitives:
              - object_id: obj_4
                edge_idx: 19
                push_steps: 9
              - object_id: obj_4
                edge_idx: 23
                push_steps: 2

        Returns the chain only on full planner-exact sim verification.
        """
        if not self._manual_primitives_file:
            print(
                "[NAMOBridge] manual_primitives strategy requires "
                "--manual-primitives-file but no path was supplied",
                flush=True,
            )
            return []

        path = Path(self._manual_primitives_file)
        if not path.exists():
            print(
                f"[NAMOBridge] manual_primitives_file not found: {path}",
                flush=True,
            )
            return []

        try:
            doc = yaml.safe_load(path.read_text()) or {}
        except yaml.YAMLError as exc:
            print(
                f"[NAMOBridge] manual_primitives_file YAML parse failed: {exc!r}",
                flush=True,
            )
            return []

        entries = doc.get("primitives", [])
        if not isinstance(entries, list) or not entries:
            print(
                f"[NAMOBridge] manual_primitives_file has no 'primitives' list: {path}",
                flush=True,
            )
            return []

        print(
            f"[NAMOBridge] manual_primitives: simulating {len(entries)} entries "
            f"from {path.name}",
            flush=True,
        )

        result = self.verify_chain(
            observation=observation,
            robot_goal_cm=robot_goal_cm,
            chain=entries,
        )
        self.last_search_time_ms = result.verification_time_ms
        self.last_sim_pushes_tried = result.sim_pushes_tried
        self.last_algorithm_stats = {
            "total_primitives_attempted": int(result.sim_pushes_tried),
        }

        if result.success:
            print(
                f"[NAMOBridge] manual_primitives: SIM VERIFIED — goal is "
                f"reachable after {len(result.verified_subgoals)} push(es); dispatching chain "
                f"to real robot",
                flush=True,
            )
            return result.verified_subgoals

        print(
            f"[NAMOBridge] manual_primitives: SIM REJECTED — goal NOT reachable "
            f"after {len(result.verified_subgoals)} verified push(es); "
            f"failure_reason={result.failure_reason!r}; returning empty plan "
            f"(real robot will NOT execute)",
            flush=True,
        )
        return []

    def get_object_mapping(self) -> ObjectMapping:
        """Get the current object ID mapping."""
        return self._object_mapping
