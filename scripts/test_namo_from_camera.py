#!/usr/bin/env python3
"""Capture scene from camera and run NAMO planning.

Captures current robot/object positions via ArUco markers,
generates MuJoCo XML, runs NAMO planning, and visualizes the solution.

Goal can be specified via --goal or detected from goal marker (ArUco 6x6, ID 0).

Usage:
    # Use detected goal marker (recommended)
    python scripts/test_namo_from_camera.py
    python scripts/test_namo_from_camera.py -v

    # Override with manual goal
    python scripts/test_namo_from_camera.py --goal 50 60
    python scripts/test_namo_from_camera.py --goal 50 60 --no-viz
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import time
from pathlib import Path

import cv2
import yaml

from robot_control.planner.namo_binding_loader import (
    load_canonical_namo_rl,
    resolve_namo_cpp_dir,
    resolve_namo_root,
)

script_path = Path(__file__).resolve()
namo_root = resolve_namo_root(script_path)
_, _, canonical_build = load_canonical_namo_rl(script_path)

from robot_control.camera import ArucoObserver, ObserverConfig
from robot_control.camera.observer import ObjectDefinition
from robot_control.camera.workspace import WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM
from robot_control.nodes import CameraSensorNode, CameraConfig
from robot_control.utils import NAMOXMLGenerator


def load_config(config_path: str, objects_path: str):
    """Load camera and observer config."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    camera_cfg = config.get("camera", {})
    robot_cfg = config.get("robot", {})
    workspace_cfg = config.get("workspace", {})

    # Parse marker offset
    marker_offset = robot_cfg.get("marker_to_wheel_offset", [0.0, 0.0])
    if isinstance(marker_offset, list) and len(marker_offset) == 2:
        marker_offset_tuple = (float(marker_offset[0]), float(marker_offset[1]))
    else:
        marker_offset_tuple = (0.0, 0.0)

    # Load object definitions
    object_defs = {}
    if Path(objects_path).exists():
        with open(objects_path, "r") as f:
            obj_config = yaml.safe_load(f)
        for name, obj_cfg in obj_config.get("objects", {}).items():
            obj_type = obj_cfg.get("type", "movable")
            shape = obj_cfg.get("shape", {})
            offset = obj_cfg.get("marker_offset", {})
            object_defs[name] = ObjectDefinition(
                marker_id=obj_cfg["marker_id"],
                is_static=(obj_type == "static"),
                is_goal=(obj_type == "goal"),
                width_cm=shape.get("width", 0.0),
                depth_cm=shape.get("depth", 0.0),
                height_cm=shape.get("height", 0.0),
                marker_offset_x_cm=offset.get("x", 0.0),
                marker_offset_y_cm=offset.get("y", 0.0),
            )

    object_marker_size = config.get("object_marker_size_mm") or robot_cfg.get("object_marker_size_mm", 30.0)

    camera_config = CameraConfig(
        camera_device=camera_cfg.get("device", 0),
        resolution=camera_cfg.get("resolution", "720p"),
        fps=camera_cfg.get("fps", 60),
        exposure=camera_cfg.get("exposure", -6),
        calibration_file=camera_cfg.get("calibration_file", ""),
    )

    observer_config = ObserverConfig(
        calibration_file=camera_cfg.get("calibration_file", ""),
        robot_marker_id=robot_cfg.get("marker_id", 1),
        robot_marker_size_mm=robot_cfg.get("marker_size_mm", 36.0),
        marker_to_wheel_offset_cm=marker_offset_tuple,
        object_defs=object_defs,
        object_marker_size_mm=object_marker_size,
        warmup_frames=workspace_cfg.get("warmup_frames", 30),
        min_workspace_inliers=workspace_cfg.get("min_inliers", 12),
    )

    return camera_config, observer_config


def capture_scene(camera_config, observer_config, stable_frames=10):
    """Capture stable scene from camera."""
    print("\nStarting camera...")
    camera = CameraSensorNode(camera_config)
    if not camera.start():
        print("Failed to start camera!")
        return None

    print("Starting ArUco observer...")
    observer = ArucoObserver(observer_config)
    if not observer.start():
        print("Failed to start observer!")
        camera.stop()
        return None

    print("\n" + "=" * 50)
    print("Waiting for stable detection...")
    print("Press 'c' to capture, 'q' to quit")
    print("=" * 50)

    cv2.namedWindow("Capture", cv2.WINDOW_NORMAL)

    stable_count = 0
    last_obs = None
    captured_obs = None

    try:
        while captured_obs is None:
            vis = observer.get_vis_frame()
            if vis is not None:
                cv2.putText(vis, f"Stable: {stable_count}/{stable_frames} - Press 'c' to capture",
                           (10, vis.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Capture", vis)

            obs = observer.get()
            if obs is not None:
                if last_obs is not None:
                    dx = abs(obs.robot_x - last_obs.robot_x)
                    dy = abs(obs.robot_y - last_obs.robot_y)
                    if dx < 1.0 and dy < 1.0:
                        stable_count += 1
                    else:
                        stable_count = 0
                last_obs = obs

                goal_str = f"({obs.goal_x:.1f}, {obs.goal_y:.1f})" if obs.goal_x is not None else "N/A"
                print(f"\rRobot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) @ {obs.robot_theta:.1f}° | "
                      f"Objects: {len(obs.objects)} | Goal: {goal_str} | Stable: {stable_count}/{stable_frames}  ",
                      end="", flush=True)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                print("\n\nCancelled.")
                break
            elif key == ord('c') and obs is not None:
                captured_obs = obs
                print("\n\nCaptured!")

            time.sleep(0.01)

    finally:
        cv2.destroyAllWindows()
        observer.stop()
        camera.stop()

    return captured_obs


def run_namo_planning(obs, goal_cm, scale_factor=6.0, max_chain_depth=1, visualize=True, verbose=True, enable_viewer=True):
    """Generate XML and run NAMO planning."""
    import os
    os.chdir(str(resolve_namo_cpp_dir(script_path)))

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

    # Step 1: Generate XML
    print(f"\n{'='*60}")
    print("STEP 1: Generating MuJoCo XML...")
    print(f"{'='*60}")

    generator = NAMOXMLGenerator(scale_factor=scale_factor)

    objects = {}
    for name, obj in obs.objects.items():
        objects[name] = (obj.x, obj.y, obj.theta, obj.width, obj.depth, obj.height, obj.is_static)

    xml_content = generator.from_observation(
        robot_x_cm=obs.robot_x,
        robot_y_cm=obs.robot_y,
        objects=objects,
        goal_x_cm=goal_cm[0],
        goal_y_cm=goal_cm[1],
        workspace_bounds_cm=(0.0, WORKSPACE_WIDTH_CM, 0.0, WORKSPACE_HEIGHT_CM),
    )

    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_content)
        xml_path = f.name

    print(f"XML saved to: {xml_path}")
    input("\n[Press Enter to continue to Step 2...]")

    # Step 2: Print scene info
    print(f"\n{'='*60}")
    print("STEP 2: Scene Summary")
    print(f"{'='*60}")
    print(f"Robot: ({obs.robot_x:.1f}, {obs.robot_y:.1f}) cm @ {obs.robot_theta:.1f}°")
    print(f"Goal: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm")
    print(f"Scale factor: {scale_factor}x (real -> sim)")
    print(f"\nObjects ({len(obs.objects)}):")
    for name, obj in obs.objects.items():
        type_str = "[STATIC]" if obj.is_static else "[MOVABLE]"
        print(f"  {type_str} {name}: ({obj.x:.1f}, {obj.y:.1f}) cm @ {obj.theta:.1f}°, size: {obj.width:.1f}x{obj.depth:.1f} cm")

    # Convert goal to simulation coordinates
    goal_sim = (goal_cm[0] / 100.0 * scale_factor, goal_cm[1] / 100.0 * scale_factor, 0.0)
    print(f"\nSimulation coordinates:")
    print(f"  Robot: ({obs.robot_x/100*scale_factor:.3f}, {obs.robot_y/100*scale_factor:.3f}) m")
    print(f"  Goal: ({goal_sim[0]:.3f}, {goal_sim[1]:.3f}) m")

    input("\n[Press Enter to continue to Step 3...]")

    # Step 3: Load environment
    print(f"\n{'='*60}")
    print("STEP 3: Loading NAMO Environment...")
    print(f"{'='*60}")

    config_path = "config/namo_config_complete_skill15_car_1x.yaml"
    env = namo_rl.RLEnvironment(xml_path, config_path, enable_viewer)
    print(f"Environment loaded from: {xml_path}")
    print(f"Config: {config_path}")

    # Show environment bounds
    bounds = env.get_world_bounds()
    print(f"World bounds: x=[{bounds[0]:.2f}, {bounds[1]:.2f}], y=[{bounds[2]:.2f}, {bounds[3]:.2f}] m")

    input("\n[Press Enter to continue to Step 4...]")

    # Step 4: Create planner
    print(f"\n{'='*60}")
    print("STEP 4: Creating NAMO Planner...")
    print(f"{'='*60}")

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

    print(f"Algorithm parameters:")
    print(f"  goal_strategy: primitive")
    print(f"  max_chain_depth: {max_chain_depth}")
    print(f"  frontier_beam_width: 10000")

    config = PlannerConfig(
        verbose=verbose,
        goals_per_region=10,
        algorithm_params=algo_params,
    )

    planner = RegionOpeningPlanner(env, config)

    if visualize:
        planner.visualize_search = True
        planner.search_delay = 0.2
        print(f"Visualization: ENABLED (delay={planner.search_delay}s)")
    else:
        print(f"Visualization: DISABLED")

    env.set_robot_goal(goal_sim[0], goal_sim[1], goal_sim[2])
    print(f"\nRobot goal set to: ({goal_sim[0]:.3f}, {goal_sim[1]:.3f}, {goal_sim[2]:.3f})")

    input("\n[Press Enter to RUN PLANNING (Step 5)...]")

    # Step 5: Run search
    print(f"\n{'='*60}")
    print("STEP 5: Running NAMO Planning...")
    print(f"{'='*60}\n")

    result = planner.search(goal_sim)

    # Step 6: Print results
    print(f"\n{'='*60}")
    print("STEP 6: Planning Result")
    print(f"{'='*60}")
    print(f"Success: {result.success}")
    print(f"Search time: {result.search_time_ms:.1f}ms")

    # Build object ID mapping for display
    movable_objects = sorted([n for n, o in obs.objects.items() if not o.is_static])
    id_mapping = {f"obstacle_{i+1}_movable": name for i, name in enumerate(movable_objects)}
    print(f"\nObject ID mapping (sim -> real):")
    for sim_id, real_id in id_mapping.items():
        print(f"  {sim_id} -> {real_id}")

    if result.action_sequence:
        print(f"\n" + "="*40)
        print(f"PLAN: {len(result.action_sequence)} action(s)")
        print("="*40)
        for i, action in enumerate(result.action_sequence):
            edge_idx = getattr(action, 'edge_idx', -1)
            depth = getattr(action, 'depth', -1)

            # Map back to real object name
            real_name = id_mapping.get(action.object_id, action.object_id)

            print(f"\n  [{i+1}] Push '{real_name}' (sim: {action.object_id})")
            print(f"       edge_idx: {edge_idx}")
            print(f"       depth: {depth} (push_steps = {depth + 1})")
            print(f"       goal: ({action.x:.3f}, {action.y:.3f}, θ={action.theta:.3f}) [sim meters]")
    else:
        print("\n" + "!"*40)
        print("NO SOLUTION FOUND!")
        print("!"*40)

    input("\n[Press Enter to continue...]")

    # Step 7: Visualize solution execution
    if visualize and result.success and result.action_sequence:
        print(f"\n{'='*60}")
        print("STEP 7: Visualize Solution in Simulation")
        print(f"{'='*60}")

        # Reset and execute
        initial_state = env.get_full_state()
        env.set_full_state(initial_state)
        env.render()

        print("\nSimulation viewer should be open.")
        input("\n[Press Enter to execute solution step by step...]")

        for i, action in enumerate(result.action_sequence):
            real_name = id_mapping.get(action.object_id, action.object_id)
            edge_idx = getattr(action, 'edge_idx', -1)
            depth = getattr(action, 'depth', -1)

            print(f"\n--- Action {i+1}/{len(result.action_sequence)} ---")
            print(f"  Push '{real_name}'")
            print(f"  edge_idx={edge_idx}, push_steps={depth+1}")

            input(f"[Press Enter to execute action {i+1}...]")

            namo_action = namo_rl.Action()
            namo_action.object_id = action.object_id
            namo_action.x = action.x
            namo_action.y = action.y
            namo_action.theta = action.theta
            namo_action.edge_idx = edge_idx
            namo_action.depth = depth

            step_result = env.step(namo_action)
            env.render()

            print(f"  Result: done={step_result.done}, reward={step_result.reward}")
            print(f"  Info: {step_result.info}")

        print(f"\n{'='*60}")
        print("Solution visualization complete!")
        print(f"{'='*60}")
        input("\n[Press Enter to exit...]")

    # Cleanup
    import os
    os.unlink(xml_path)

    return result


def main():
    parser = argparse.ArgumentParser(description="Capture from camera and run NAMO planning")
    parser.add_argument("--goal", "-g", type=float, nargs=2, default=None,
                       metavar=("X", "Y"), help="Goal position in cm (default: use detected goal marker)")
    parser.add_argument("--config", "-c", type=str, default="config/real.yaml",
                       help="Camera config path")
    parser.add_argument("--objects", type=str, default="config/objects.yaml",
                       help="Objects definition path")
    parser.add_argument("--max-chain-depth", type=int, default=1,
                       help="Max chain depth (default: 1)")
    parser.add_argument("--no-viz", action="store_true", help="Disable solution visualization")
    parser.add_argument("--no-viewer", action="store_true", help="Disable MuJoCo viewer window")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--stable-frames", type=int, default=10,
                       help="Stable frames before capture (default: 10)")

    args = parser.parse_args()

    # Change to robot_control directory
    import os
    os.chdir(str(robot_control_dir))

    # Load config
    config_path = Path(args.config)
    objects_path = Path(args.objects)

    if not config_path.exists():
        print(f"Config not found: {config_path}")
        return 1

    camera_config, observer_config = load_config(str(config_path), str(objects_path))

    # Capture scene
    obs = capture_scene(camera_config, observer_config, args.stable_frames)
    if obs is None:
        print("No observation captured!")
        return 1

    # Determine goal: use --goal if provided, otherwise use detected goal marker
    if args.goal is not None:
        goal_cm = (args.goal[0], args.goal[1])
        print(f"\nUsing manual goal: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm")
    elif obs.goal_x is not None and obs.goal_y is not None:
        goal_cm = (obs.goal_x, obs.goal_y)
        print(f"\nUsing detected goal marker: ({goal_cm[0]:.1f}, {goal_cm[1]:.1f}) cm")
    else:
        print("\nError: No goal specified!")
        print("Either provide --goal X Y or place goal marker (ArUco 6x6, ID 0) in the scene.")
        return 1

    # Run planning
    result = run_namo_planning(
        obs=obs,
        goal_cm=goal_cm,
        max_chain_depth=args.max_chain_depth,
        visualize=not args.no_viz,
        verbose=args.verbose,
        enable_viewer=not args.no_viewer,
    )

    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
