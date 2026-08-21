#!/usr/bin/env python3
"""Test NAMO planning with visualization.

Captures current scene, runs NAMO planning, and visualizes the solution
without executing on the robot.

Usage:
    # Use benchmark environment
    python scripts/test_namo_planning.py --benchmark

    # Use custom XML
    python scripts/test_namo_planning.py --xml /path/to/env.xml

    # Generate from simulation observation
    python scripts/test_namo_planning.py --sim
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

from robot_control.planner.namo_binding_loader import (
    load_canonical_namo_rl,
    resolve_namo_cpp_dir,
    resolve_namo_root,
)

script_path = Path(__file__).resolve()
namo_root = resolve_namo_root(script_path)
_, _, canonical_build = load_canonical_namo_rl(script_path)


def run_planning_with_visualization(
    xml_path: str,
    config_path: str,
    goal: tuple = None,
    max_chain_depth: int = 1,
    visualize: bool = True,
    verbose: bool = True,
):
    """Run NAMO planning and optionally visualize."""
    import namo_rl
    from namo.core import PlannerConfig
    from namo.planners.opening import RegionOpeningPlanner

    loaded = Path(getattr(namo_rl, "__file__", "")).resolve()
    if canonical_build not in loaded.parents:
        raise RuntimeError(
            "Loaded namo_rl from non-canonical path.\n"
            f"  loaded:   {loaded}\n"
            f"  expected: {canonical_build}\n"
            "Fix PYTHONPATH so namo_cpp/build_python is first."
        )

    print(f"\n{'='*60}")
    print("NAMO Planning Test")
    print(f"{'='*60}")
    print(f"XML: {xml_path}")
    print(f"Config: {config_path}")

    # Load environment
    env = namo_rl.RLEnvironment(xml_path, config_path)

    # Get environment info
    obs = env.get_observation()
    bounds = env.get_world_bounds()

    print(f"\nEnvironment:")
    print(f"  Bounds: x=[{bounds[0]:.2f}, {bounds[1]:.2f}], y=[{bounds[2]:.2f}, {bounds[3]:.2f}]")
    print(f"  Objects: {[k for k in obs.keys() if 'movable' in k]}")

    # Get robot position
    robot_pos = obs.get('robot_pose', [0, 0, 0])
    print(f"  Robot: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, θ={robot_pos[2]:.2f})")

    # Set goal
    if goal is None:
        # Default: opposite corner from robot
        goal = (bounds[1] - 0.5, bounds[3] - 0.5, 0.0)

    print(f"  Goal: ({goal[0]:.2f}, {goal[1]:.2f}, θ={goal[2]:.2f})")

    # Create planner config
    algo_params = {
        "primitive_data_dir": "data",
        "goal_strategy": "primitive",
        "region_max_chain_depth": max_chain_depth,
        "region_max_solutions_per_neighbor": 1,
        "region_max_recorded_solutions_per_neighbor": 1,
        "region_frontier_beam_width": 10000,
        "region_chain_link_cost": 11,
        "region_selection_strategy": "cost_first",
        "xml_file": xml_path,
    }

    config = PlannerConfig(
        verbose=verbose,
        goals_per_region=10,
        algorithm_params=algo_params,
    )

    # Create planner
    planner = RegionOpeningPlanner(env, config)

    # Enable visualization if requested
    if visualize:
        planner.visualize_search = True
        planner.search_delay = 0.3  # Delay between steps
        planner.step_mode = False  # Set True to step through manually

    # Set robot goal
    env.set_robot_goal(goal[0], goal[1], goal[2])

    print(f"\n{'='*60}")
    print("Running NAMO Planning...")
    print(f"{'='*60}\n")

    # Run search
    result = planner.search(goal)

    print(f"\n{'='*60}")
    print("Planning Result")
    print(f"{'='*60}")
    print(f"Success: {result.success}")
    print(f"Solution found: {result.solution_found}")
    print(f"Search time: {result.search_time_ms:.1f}ms")

    if result.action_sequence:
        print(f"\nAction sequence ({len(result.action_sequence)} actions):")
        for i, action in enumerate(result.action_sequence):
            edge_idx = getattr(action, 'edge_idx', -1)
            depth = getattr(action, 'depth', -1)
            print(f"  [{i}] Push {action.object_id}")
            print(f"       Goal: ({action.x:.3f}, {action.y:.3f}, θ={action.theta:.3f})")
            print(f"       edge_idx={edge_idx}, depth={depth} (push_steps={depth+1})")
    else:
        print("\nNo solution found!")

    # Show attempt results summary
    if result.algorithm_stats:
        attempts = result.algorithm_stats.get("attempt_results", [])
        print(f"\nAttempt results: {len(attempts)} total")
        for ar in attempts:
            status = "✓" if ar.success else "✗"
            print(f"  {status} Neighbor '{ar.neighbour_region_label}': object={ar.chosen_object_id}")

    # If visualization enabled, show the final solution
    if visualize and result.success and result.action_sequence:
        print(f"\n{'='*60}")
        print("Visualizing Solution...")
        print("(Close the viewer window to exit)")
        print(f"{'='*60}")

        # Reset to initial state and execute the solution visually
        initial_state = env.get_full_state()

        for i, action in enumerate(result.action_sequence):
            print(f"\nExecuting action {i+1}/{len(result.action_sequence)}: Push {action.object_id}")

            # Create namo_rl.Action for execution
            namo_action = namo_rl.Action()
            namo_action.object_id = action.object_id
            namo_action.x = action.x
            namo_action.y = action.y
            namo_action.theta = action.theta
            namo_action.edge_idx = getattr(action, 'edge_idx', -1)
            namo_action.depth = getattr(action, 'depth', -1)

            # Execute with visualization
            env.render()
            import time
            time.sleep(0.5)

            step_result = env.step(namo_action)

            env.render()
            time.sleep(1.0)

            print(f"  Result: done={step_result.done}, reward={step_result.reward}")

        print("\nSolution visualization complete!")
        print("Press Enter to exit...")
        input()

    return result


def main():
    parser = argparse.ArgumentParser(description="Test NAMO planning with visualization")

    # Scene source
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--benchmark", action="store_true", help="Use benchmark environment")
    group.add_argument("--xml", type=str, help="Path to MuJoCo XML file")
    group.add_argument("--sim", action="store_true", help="Capture from simulation (not implemented)")

    # Planning options
    parser.add_argument("--goal", type=float, nargs=3, metavar=("X", "Y", "THETA"),
                       help="Goal position in meters (simulation coordinates)")
    parser.add_argument("--max-chain-depth", type=int, default=1,
                       help="Maximum chain depth (default: 1)")

    # Visualization
    parser.add_argument("--no-viz", action="store_true", help="Disable visualization")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")

    # Config paths
    parser.add_argument("--config", type=str,
                       default="config/namo_config_complete_skill15_car_1x.yaml",
                       help="NAMO config path")

    args = parser.parse_args()

    # Change to namo_cpp directory for relative paths
    import os
    os.chdir(str(resolve_namo_cpp_dir(script_path)))

    # Determine XML path
    if args.xml:
        xml_path = args.xml
    elif args.benchmark:
        xml_path = "data/benchmark_env.xml"
    else:
        # Default to benchmark
        xml_path = "data/benchmark_env.xml"
        print("No scene specified, using benchmark environment")

    # Parse goal
    goal = tuple(args.goal) if args.goal else None

    # Run planning
    result = run_planning_with_visualization(
        xml_path=xml_path,
        config_path=args.config,
        goal=goal,
        max_chain_depth=args.max_chain_depth,
        visualize=not args.no_viz,
        verbose=args.verbose,
    )

    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
