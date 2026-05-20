"""Bridge between robot_control observations and NAMO planning.

Handles:
- State capture from Observation to MuJoCo XML
- Object ID mapping between real robot names and simulation names
- Coordinate transformation (cm -> scaled meters)
- Plan conversion to PushSubgoal sequence
"""

from __future__ import annotations

import os
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import yaml

from robot_control.camera.workspace import WORKSPACE_HEIGHT_CM, WORKSPACE_WIDTH_CM
from robot_control.core.types import Observation, PushSubgoal
from robot_control.planner.namo_binding_loader import (
    ensure_namo_cpp_paths,
    load_canonical_namo_rl,
)
from robot_control.utils import NAMOXMLGenerator
from robot_control.utils.robot_geometry import (
    effective_robot_radius_cm,
    scaled_half_extents_m_from_full_extents_cm,
)


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


class NAMOPlanBridge:
    """Bridge for NAMO planning from robot_control observations.

    Captures current robot state, generates scaled MuJoCo XML,
    invokes NAMO planning, and converts results to PushSubgoals.

    Usage:
        bridge = NAMOPlanBridge(
            namo_config_path="namo_cpp/config/namo_config_complete_skill15.yaml",
            scale_factor=6.0,
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
        verbose: bool = False,
        debug_xml_path: Optional[str] = None,
        enable_viewer: bool = False,
        pause_after_load: bool = False,
        robot_width_cm: float = 6.0,
        robot_height_cm: float = 6.0,
        robot_model: str = "sphere",
    ):
        """Initialize the NAMO bridge.

        Args:
            namo_config_path: Path to NAMO config YAML
            scale_factor: Scale factor for simulation (1.0 = real meters/cm,
                production path; 6.0 = legacy 6×-scaled mode)
            primitive_data_dir: Directory containing motion primitive data.
                               If None, uses namo_cpp/data/ (auto-detected).
            verbose: Enable verbose logging
            debug_xml_path: If set, save generated XML to this path for debugging
            enable_viewer: Enable MuJoCo visualization window during planning
            pause_after_load: Pause for user input after loading XML (for inspection)
            robot_width_cm: Robot width in cm (for XML generation)
            robot_height_cm: Robot height in cm (for XML generation)
            robot_model: Robot body to embed in the generated XML
                ("sphere" = holonomic point robot, current behavior;
                 "car" = diff-drive little_car body, reserved for the
                 car-primitive follow-up phase — raises NotImplementedError
                 today).
        """
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

        # Compute absolute path for primitive_data_dir
        if primitive_data_dir is None:
            # Auto-detect: namo_cpp/data/ relative to this file
            bridge_path = Path(__file__).resolve()
            namo_cpp_dir = bridge_path.parents[4] / "namo_cpp"
            self._primitive_data_dir = str(namo_cpp_dir / "data")
        else:
            self._primitive_data_dir = primitive_data_dir

        self._object_mapping = ObjectMapping()

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
            self._generated_config_path.unlink(missing_ok=True)
        except Exception:
            pass
        self._generated_config_path = None

    def _resolve_namo_config_path(self, path_str: str) -> Path:
        path = Path(path_str)
        if path.exists():
            return path.resolve()

        bridge_path = Path(__file__).resolve()
        namo_root = bridge_path.parents[4]
        candidate = namo_root / path_str
        if candidate.exists():
            return candidate.resolve()

        candidate2 = namo_root / "namo_cpp" / path_str
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
            half_x_m, half_y_m = scaled_half_extents_m_from_full_extents_cm(
                self._robot_width_cm,
                self._robot_height_cm,
                self._scale_factor,
            )
            planning["robot_size"] = [half_x_m, half_y_m]

            fd, tmp_path = tempfile.mkstemp(
                prefix="namo_config_runtime_",
                suffix=".yaml",
            )
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                yaml.safe_dump(cfg, f, sort_keys=False)
            self._generated_config_path = Path(tmp_path)

            if self._verbose:
                print(
                    f"[NAMOBridge] Runtime config override: planning.robot_size="
                    f"[{half_x_m:.6f}, {half_y_m:.6f}] m (half-extents, scaled)"
                )
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

    def plan(
        self,
        observation: Observation,
        robot_goal_cm: Tuple[float, float],
        algorithm: str = "full_namo",
        goal_strategy: str = "primitive",
        max_chain_depth: int = 2,
        allow_collisions: bool = True,
        frontier_beam_width: int = 10000,
        chain_link_cost: int = 11,
        selection_strategy: str = "cost_first",
        goals_per_region: int = 10,
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
            allow_collisions: Allow collisions during push
            frontier_beam_width: Beam width for frontier search
            chain_link_cost: Additional cost per chain link
            selection_strategy: Frontier priority ("cost_first" or "ml_first")
            goals_per_region: Goal samples per region for validation
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
        namo_cpp_dir = bridge_path.parents[4] / "namo_cpp"
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

            # Run planning
            result = service.plan_from_xml(
                xml_path=xml_path,
                robot_goal=(goal_sim[0], goal_sim[1], 0.0),
                algorithm=algorithm,
                goal_strategy=goal_strategy,
                max_chain_depth=max_chain_depth,
                allow_collisions=allow_collisions,
                frontier_beam_width=frontier_beam_width,
                chain_link_cost=chain_link_cost,
                selection_strategy=selection_strategy,
                goals_per_region=goals_per_region,
                **kwargs,
            )

            self.last_search_time_ms = result.search_time_ms
            self.last_algorithm_stats = getattr(result, "algorithm_stats", None)

            if self._verbose:
                print(
                    f"[NAMOBridge] Planning result: success={result.success}, "
                    f"actions={len(result.actions)}, time={result.search_time_ms:.1f}ms"
                )
                if result.error_message:
                    print(f"[NAMOBridge] Error: {result.error_message}")

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
        namo_cpp_dir = bridge_path.parents[4] / "namo_cpp"
        original_cwd = os.getcwd()
        os.chdir(str(namo_cpp_dir))

        try:
            goal_sim = self._cm_to_sim(robot_goal_cm[0], robot_goal_cm[1])
            service = self._get_planning_service()
            summary = service.analyze_reachability_from_xml(
                xml_path=xml_path,
                robot_goal=(goal_sim[0], goal_sim[1], 0.0),
                analysis_mode=analysis_mode,
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

    def get_object_mapping(self) -> ObjectMapping:
        """Get the current object ID mapping."""
        return self._object_mapping
