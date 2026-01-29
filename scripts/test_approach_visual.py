#!/usr/bin/env python3
"""Visual test of approach phase in simulation.

Shows the robot navigating to the approach position before pushing.
Stops after approach completes (no push dynamics in sim).

This tests the FULL PushController approach flow:
1. PushController._compute_approach_position() - geometry
2. PushController._validate_approach_position() - WavefrontPlanner
3. NavigationController.navigate_to() - RVGPlanner path planning
4. NavigationController.step() - path following

Usage:
    python scripts/test_approach_visual.py
    python scripts/test_approach_visual.py --edge 0  # approach from +X
    python scripts/test_approach_visual.py --edge 1  # approach from +Y
    python scripts/test_approach_visual.py --edge 2  # approach from -X
    python scripts/test_approach_visual.py --edge 3  # approach from -Y
    python scripts/test_approach_visual.py --show-all-edges  # visualize all edge points
"""

import argparse
from robot_control import Runtime, RuntimeConfig, SimConfig
from robot_control.core.types import Observation, PushSubgoal, ObjectPose
from robot_control.core.object_defs import ObjectDef
from robot_control.planner.base import Planner
from robot_control.controller.push import PushController, PushState
from robot_control.controller.edge_points import generate_edge_points, get_face_name
from typing import Any, Dict, List, Optional


class ApproachOnlyPlanner(Planner):
    """
    Planner that issues a PushSubgoal to test the full approach phase.

    Uses PushController's approach logic:
    - _compute_approach_position() for geometry
    - _validate_approach_position() with WavefrontPlanner
    - NavigationController for path planning (RVGPlanner)

    Monitors PushController state and stops when approach completes
    (before actual pushing, since sim has no push dynamics).
    """

    def __init__(self, object_id: str, edge_idx: int, push_steps: int = 10,
                 show_all_edges: bool = False, points_per_face: int = 1,
                 standoff: float = 5.0):
        self._object_id = object_id
        self._edge_idx = edge_idx
        self._push_steps = push_steps  # Short push, we'll stop before it finishes
        self._issued = False
        self._done = False
        self._push_controller: Optional[PushController] = None
        self._show_all_edges = show_all_edges
        self._points_per_face = points_per_face
        self._standoff = standoff
        self._cached_obj: Optional[ObjectPose] = None

    def set_push_controller(self, controller: PushController) -> None:
        """Set reference to push controller to monitor its state."""
        self._push_controller = controller

    def plan(self, obs: Observation) -> Optional[PushSubgoal]:
        if self._done:
            return None

        if self._issued:
            # Log when we reach PUSHING state (approach complete)
            if self._push_controller and self._push_controller._state == PushState.PUSHING:
                if not hasattr(self, '_push_started'):
                    self._push_started = True
                    print("\n[Planner] === APPROACH COMPLETE, STARTING PUSH ===")
                    print(f"  Robot reached approach position!")
                    print(f"  Approach pos: {self._push_controller._approach_position}")
                    print(f"  Approach orientation: {self._push_controller._approach_orientation}°")
                    print(f"  Now pushing for {self._push_steps} steps...")
            return None

        # Get object
        obj = obs.objects.get(self._object_id)
        if obj is None:
            print(f"[Planner] Object '{self._object_id}' not found")
            self._done = True
            return None

        self._cached_obj = obj
        self._issued = True

        print(f"\n[Planner] === STARTING APPROACH TEST ===")
        print(f"  Object: {self._object_id} at ({obj.x:.1f}, {obj.y:.1f})")
        print(f"  Edge: {self._edge_idx}")
        print(f"\n  Flow:")
        print(f"  1. PushController._compute_approach_position() - geometry")
        print(f"  2. PushController._validate_approach_position() - WavefrontPlanner")
        print(f"  3. NavigationController.navigate_to() - RVGPlanner")
        print(f"  4. NavigationController.step() - path following")

        return PushSubgoal(
            object_id=self._object_id,
            edge_idx=self._edge_idx,
            push_steps=self._push_steps,
        )

    def notify_subgoal_done(self, obs: Observation) -> None:
        self._done = True

    def is_complete(self, obs: Observation) -> bool:
        return self._done

    def get_drawings(self) -> List[Dict[str, Any]]:
        drawings: List[Dict[str, Any]] = []

        if not self._show_all_edges or self._cached_obj is None:
            return drawings

        # Generate all edge points
        all_points = generate_edge_points(
            self._cached_obj, self._standoff, self._points_per_face
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
            # Highlight selected edge
            if ep.edge_idx == self._edge_idx:
                color = "#FF00FF"  # Magenta for selected
                radius = 5
            else:
                radius = 3

            # Draw edge point
            drawings.append({
                "uuid": f"edge_{ep.edge_idx}",
                "type": "point",
                "position": ep.position,
                "radius": radius,
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

    def reset(self) -> None:
        self._issued = False
        self._done = False


def main():
    parser = argparse.ArgumentParser(description="Visual test of approach phase")
    parser.add_argument("--edge", type=int, default=2,
                        help="Edge index to approach (default: 2)")
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--show-all-edges", action="store_true",
                        help="Show all edge points for visualization")
    parser.add_argument("--points-per-face", type=int, default=1,
                        help="Points per face (1, 3, or 15)")
    args = parser.parse_args()

    # With multi-point edges, describe by face
    n = args.points_per_face
    max_edge = 4 * n - 1
    if args.edge < 0 or args.edge > max_edge:
        print(f"Error: edge must be 0-{max_edge} for points_per_face={n}")
        return

    # Determine face from edge index
    if args.edge < 2 * n:
        face_idx = 0 if args.edge % 2 == 0 else 1
        sample_idx = args.edge // 2
    else:
        face_idx = 2 if (args.edge - 2 * n) % 2 == 0 else 3
        sample_idx = (args.edge - 2 * n) // 2

    face_names = ["Top (+Y)", "Bottom (-Y)", "Right (+X)", "Left (-X)"]
    edge_desc = f"{face_names[face_idx]}, sample {sample_idx}"

    print(f"\n=== Approach Phase Visual Test ===")
    print(f"Edge {args.edge}: {edge_desc}")
    print(f"Points per face: {args.points_per_face} ({4 * args.points_per_face} total edges)")
    if args.show_all_edges:
        print(f"\nEdge point colors:")
        print(f"  Green  = Top (+Y) face    - pushes downward")
        print(f"  Red    = Bottom (-Y) face - pushes upward")
        print(f"  Blue   = Right (+X) face  - pushes leftward")
        print(f"  Yellow = Left (-X) face   - pushes rightward")
        print(f"  Magenta = Selected edge (idx {args.edge})")
        print(f"\nArrows show push direction (edge → mid_point)")
    else:
        print(f"\nThis tests the FULL PushController approach flow:")
        print(f"  - WavefrontPlanner: validates approach position is free")
        print(f"  - RVGPlanner: plans collision-free path")
        print(f"  - NavigationController: executes path with rotation")
        print(f"\nWatch for:")
        print(f"  - Cyan circle: approach position (from PushController)")
        print(f"  - Green path: RVG navigation path")
        print(f"  - Robot rotates, follows path, rotates to final orientation")
    print(f"\nPress Escape to quit\n")

    # Realistic robot dynamics (MicroMVP ~5cm robot)
    car_w, car_h = 5, 5
    sim_config = SimConfig(
        x=10, y=30, theta=0,  # Robot at left side
        width=60, height=60,
        car_width=car_w,
        car_height=car_h,
        offset_w=car_w / 2,
        offset_h=car_h / 2,
        wheel_base=4.0,
        max_wheel_speed=10.0,  # Slower for visualization
        objects={"push_box": (30, 30, 0)},  # Box in center
    )

    # Add object dimensions - push_box
    # Convention: depth = X (along heading), width = Y (perpendicular)
    # So 12cm perpendicular (Y) x 4cm along heading (X)
    sim_config.object_defs = {
        "push_box": ObjectDef(name="push_box", width=12.0, depth=4.0, height=4.0),
    }

    # Standoff = 0.5 * robot_size (matches config)
    standoff = 0.5 * max(car_w, car_h)
    planner = ApproachOnlyPlanner(
        "push_box", args.edge, push_steps=300,  # 10 seconds of pushing
        show_all_edges=args.show_all_edges,
        points_per_face=args.points_per_face,
        standoff=standoff,
    )

    config = RuntimeConfig(
        mode="sim",
        sim_config=sim_config,
        planner=planner,
        initial_speed=args.speed,
        quit_on_complete=False,  # Keep window open after approach
    )

    runtime = Runtime(config)

    # Give planner access to push controller to monitor state
    runtime._setup()
    push_controller = runtime._controllers.get("push")
    if push_controller:
        planner.set_push_controller(push_controller)

    runtime._start()

    if runtime._window:
        runtime._print_banner()
        runtime._window.run()
    else:
        import time
        runtime._print_banner()
        while runtime._running:
            time.sleep(0.1)

    runtime._shutdown()


if __name__ == "__main__":
    main()
