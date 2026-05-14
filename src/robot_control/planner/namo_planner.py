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
from typing import Any, Dict, List, Literal, Optional, Set, Tuple

from robot_control.core.types import NavigateSubgoal, Observation, PushSubgoal, Subgoal
from robot_control.planner.base import Planner
from robot_control.planner.namo_bridge import NAMOPlanBridge
from robot_control.planner.wavefront_path_planner import WavefrontPathPlanner


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
            namo_config_path="namo_cpp/config/namo_config_complete_skill15.yaml",
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
        scale_factor: float = 6.0,
        primitive_data_dir: str = "data",
        replan_on_completion: bool = True,
        max_chain_depth: int = 1,
        allow_collisions: bool = True,
        frontier_beam_width: int = 10000,
        chain_link_cost: int = 11,
        selection_strategy: str = "cost_first",
        goals_per_region: int = 10,
        shuffle_edges: bool = True,
        shuffle_seed: Optional[int] = None,
        verbose: bool = False,
        debug_xml_path: Optional[str] = None,
        enable_viewer: bool = False,
        pause_after_load: bool = False,
        # ML goal model
        ml_goal_model_path: Optional[str] = None,
        ml_device: str = "cuda",
        # Workspace config for reachability checking (must match navigation planner)
        workspace_width_cm: float = 70.0,
        workspace_height_cm: float = 55.0,
        robot_width_cm: float = 8.0,
        robot_height_cm: float = 10.0,
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
            scale_factor: Scale factor for simulation (6.0 default)
            primitive_data_dir: Directory containing motion primitive data
            replan_on_completion: If True, replan when subgoal queue is empty
            max_chain_depth: Maximum chain depth for multi-push solutions (1 or 2)
            allow_collisions: Allow collisions during push
            frontier_beam_width: Beam width for frontier search
            chain_link_cost: Additional cost per chain link
            selection_strategy: Frontier priority ("cost_first" or "ml_first")
            goals_per_region: Goal samples per region for validation
            shuffle_edges: Randomize edge ordering during planning (default True)
            shuffle_seed: Random seed for reproducible shuffling (None = random)
            verbose: Enable verbose logging
            debug_xml_path: If set, save generated XML to this path
            enable_viewer: Enable MuJoCo visualization window during planning
            pause_after_load: Pause after loading XML for interactive viewer inspection
            ml_goal_model_path: Path to trained ML goal model (for ml strategies)
            ml_device: PyTorch device for ML model ("cuda" or "cpu")
            workspace_width_cm: Workspace width for reachability check (must match navigation)
            workspace_height_cm: Workspace height for reachability check (must match navigation)
            robot_width_cm: Robot width for reachability check
            robot_height_cm: Robot height for reachability check
        """
        if execution_mode not in ("open_loop", "mpc"):
            raise ValueError(
                f"execution_mode must be 'open_loop' or 'mpc', got {execution_mode!r}"
            )

        self._robot_goal_cm = robot_goal_cm
        self._algorithm = algorithm
        self._execution_mode = execution_mode
        self._goal_strategy = goal_strategy
        self._replan_on_completion = replan_on_completion
        self._max_chain_depth = max_chain_depth
        self._allow_collisions = allow_collisions
        self._frontier_beam_width = frontier_beam_width
        self._chain_link_cost = chain_link_cost
        self._selection_strategy = selection_strategy
        self._goals_per_region = goals_per_region
        self._shuffle_edges = shuffle_edges
        self._shuffle_seed = shuffle_seed
        self._ml_goal_model_path = ml_goal_model_path
        self._ml_device = ml_device
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
            robot_width_cm=robot_width_cm,
            robot_height_cm=robot_height_cm,
        )

        # Create wavefront planner for reachability checks
        # Uses same dimensions as navigation planner for consistency
        self._reachability_planner = WavefrontPathPlanner(
            workspace_width=workspace_width_cm,
            workspace_height=workspace_height_cm,
            robot_width=robot_width_cm,
            robot_height=robot_height_cm,
            debug_dir="output_dump/wavefront" if verbose else None,
        )
        if verbose:
            print(f"[NAMOPlanner] Reachability planner: {workspace_width_cm}x{workspace_height_cm}cm workspace")

        # Subgoal queue state
        self._subgoals: List[PushSubgoal] = []
        self._current_idx: int = 0
        self._plan_generated: bool = False
        self._planning_failed: bool = False

        # Reachability check state
        self._navigating_to_goal: bool = False
        self._goal_tolerance: float = 5.0  # cm - close enough to goal

        # Cumulative planning time
        self._total_planning_ms: float = 0.0
        self._plan_count: int = 0

        # Retry state for unreachable approach positions
        self._max_replan_attempts: int = 5  # Max replans before giving up
        self._replan_attempt: int = 0  # Current replan attempt count
        self._failed_subgoal: Optional[PushSubgoal] = None  # Track which subgoal failed

        # Retry state for planning failures (no solution found)
        self._max_planning_retries: int = 5  # Retry with different seeds

        # Failed-push blacklist (failure feedback to the planner).
        # Each entry is (object_id, edge_idx) in real-world (robot_control)
        # naming. When a push fails, its (object_id, edge_idx) is added here
        # and forwarded to the bridge, which drops any planned push matching
        # a blacklisted pair before returning. Cleared on reset(); persists
        # across replans within a single planning episode.
        self._failed_pushes: Set[Tuple[str, int]] = set()

    def plan(self, obs: Observation) -> Optional[Subgoal]:
        """Generate next subgoal from current observation.

        Now includes reachability checking:
        1. If goal is already reachable, returns NavigateSubgoal to goal
        2. If goal is blocked, runs NAMO planning and returns PushSubgoals
        3. After pushes complete, re-checks reachability and loops

        Args:
            obs: Current observation

        Returns:
            Next Subgoal to execute (NavigateSubgoal or PushSubgoal),
            or None if complete/failed
        """
        # If we have pending push subgoals, return next one
        if self._current_idx < len(self._subgoals):
            return self._subgoals[self._current_idx]

        # Check if goal is reachable (either initially or after pushes)
        if self._is_goal_reachable(obs):
            self._navigating_to_goal = True
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
        # If we just finished navigating to goal, nothing to do
        if self._navigating_to_goal:
            return

        if failed:
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
        """Check if overall task is complete.

        Returns True if:
        - Planning failed (no actions generated)
        - Robot has reached the goal position

        Args:
            obs: Current observation

        Returns:
            True if task is complete or failed
        """
        if self._planning_failed:
            print(
                f"[NAMOPlanner] Total planning: {self._plan_count} calls, "
                f"{self._total_planning_ms:.0f}ms cumulative"
            )
            return True

        # Only complete when navigating to goal AND close enough
        if not self._navigating_to_goal:
            return False

        # Check if robot reached goal
        dist = math.hypot(
            obs.robot_x - self._robot_goal_cm[0],
            obs.robot_y - self._robot_goal_cm[1],
        )
        if dist < self._goal_tolerance:
            print(
                f"[NAMOPlanner] GOAL REACHED! Distance: {dist:.1f}cm | "
                f"Total planning: {self._plan_count} calls, "
                f"{self._total_planning_ms:.0f}ms cumulative"
            )
            return True

        return False

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
        self._replan_attempt = 0
        self._failed_subgoal = None
        self._failed_pushes = set()

    def _is_goal_reachable(self, obs: Observation) -> bool:
        """Check if robot can reach goal without pushing any objects.

        Uses wavefront-based path planning to determine reachability.

        Args:
            obs: Current observation with robot and object positions

        Returns:
            True if a collision-free path to goal exists
        """
        # Build obstacle list from ALL objects (both static and movable)
        # Static walls block the path just like movable objects
        obstacles = []
        for name, obj in obs.objects.items():
            if obj.width > 0 and obj.depth > 0:
                obstacles.append((
                    obj.x,
                    obj.y,
                    obj.theta,
                    obj.width,
                    obj.depth,
                ))

        # Plan path from robot to goal
        path = self._reachability_planner.plan(
            start=(obs.robot_x, obs.robot_y),
            goal=self._robot_goal_cm,
            obstacles=obstacles,
        )

        reachable = len(path) > 0
        if self._verbose:
            status = "REACHABLE" if reachable else "BLOCKED"
            print(f"[NAMOPlanner] Goal reachability: {status}")

        return reachable

    def _generate_plan(self, obs: Observation) -> None:
        """Generate subgoal queue via NAMO planning.

        Retries up to _max_planning_retries times with different random seeds
        when planning returns no solution (empty subgoals or exception).
        """
        self._plan_generated = True
        max_retries = self._max_planning_retries

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
                # Build extra kwargs for ML strategies
                extra_kwargs: Dict[str, Any] = {
                    "shuffle_edges": self._shuffle_edges,
                    "shuffle_seed": effective_seed,
                }
                if self._ml_goal_model_path:
                    extra_kwargs["ml_goal_model_path"] = self._ml_goal_model_path
                    extra_kwargs["ml_device"] = self._ml_device

                subgoals = self._bridge.plan(
                    observation=obs,
                    robot_goal_cm=self._robot_goal_cm,
                    algorithm=self._algorithm,
                    goal_strategy=self._goal_strategy,
                    max_chain_depth=self._max_chain_depth,
                    allow_collisions=self._allow_collisions,
                    frontier_beam_width=self._frontier_beam_width,
                    chain_link_cost=self._chain_link_cost,
                    selection_strategy=self._selection_strategy,
                    goals_per_region=self._goals_per_region,
                    failed_pushes=self._failed_pushes,
                    **extra_kwargs,
                )

                # Accumulate planning time
                self._plan_count += 1
                self._total_planning_ms += self._bridge.last_search_time_ms
                print(
                    f"[NAMOPlanner] Plan #{self._plan_count}: "
                    f"{self._bridge.last_search_time_ms:.0f}ms, "
                    f"cumulative: {self._total_planning_ms:.0f}ms"
                )

                if subgoals:
                    if self._execution_mode == "mpc":
                        # MPC: enqueue only the first push; discard the rest.
                        # The next plan() call will replan from a fresh
                        # observation, ensuring closed-loop behaviour at the
                        # plan level.
                        if len(subgoals) > 1:
                            print(
                                f"[NAMOPlanner] MPC mode: keeping 1 of "
                                f"{len(subgoals)} planned pushes (rest "
                                f"discarded for replan)"
                            )
                        self._subgoals = [subgoals[0]]
                    else:
                        # open_loop: commit to the full planned sequence.
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

        # All retries exhausted
        print(
            f"[NAMOPlanner] All {max_retries} planning attempts failed"
        )
        print("[NAMOPlanner]   Possible causes:")
        print("[NAMOPlanner]   - No feasible push found for any edge/depth")
        print("[NAMOPlanner]   - Sim2real discrepancy (sim thinks goal reachable)")
        print("[NAMOPlanner]   - Objects too tightly packed for primitives")
        self._planning_failed = True
        self._subgoals = []
        self._current_idx = 0

    def get_subgoal_queue(self) -> List[PushSubgoal]:
        """Get the current subgoal queue (for debugging)."""
        return list(self._subgoals)

    def get_current_index(self) -> int:
        """Get current index in subgoal queue (for debugging)."""
        return self._current_idx
