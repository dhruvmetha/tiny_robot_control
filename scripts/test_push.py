#!/usr/bin/env python3
"""Test push controller with real robot.

Usage:
    # Real robot with camera tracking
    python scripts/test_push.py --object push_box --edge 0

    # Dry run (vision only, no robot commands)
    python scripts/test_push.py --object push_box --edge 0 --dry-run

    # Simulation mode
    python scripts/test_push.py --object box1 --edge 0 --sim

    # Custom push duration (in ticks at 30Hz)
    python scripts/test_push.py --object push_box --edge 0 --push-steps 150

Controls:
    Space   : Emergency stop
    Escape  : Quit

Push directions (edge_idx) - depends on points_per_face in controller.yaml:
    points_per_face=1:  4 edges (0-3)
    points_per_face=3:  12 edges (0-11)
    points_per_face=15: 60 edges (0-59)

    Even indices = Top/Right faces (push down/left)
    Odd indices  = Bottom/Left faces (push up/right)
"""

import argparse
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt

from robot_control import RuntimeConfig, Runtime
from robot_control.core.types import Observation, PushSubgoal
from robot_control.planner.base import Planner


class SimplePushPlanner(Planner):
    """
    Simple planner that pushes a single object for a fixed duration.

    Tracks push progress by counting subgoal completions rather than
    measuring distance, since real-world distance measurement is noisy.
    """

    def __init__(
        self,
        object_id: str,
        edge_idx: int,
        push_steps: int = 100,
        continuous: bool = False,
        num_repeats: int = 1,
    ) -> None:
        """
        Initialize push planner.

        Args:
            object_id: ID of object to push (must match objects.yaml)
            edge_idx: Which face to push (0-3)
            push_steps: Number of control ticks per push subgoal
            continuous: If True, keeps pushing indefinitely
            num_repeats: Number of times to repeat the push subgoal
        """
        self._object_id = object_id
        self._edge_idx = edge_idx
        self._push_steps = push_steps
        self._continuous = continuous
        self._num_repeats = num_repeats
        self._done = False
        self._initial_pos: Optional[tuple] = None
        self._subgoal_count = 0

        # Track object positions: list of (x, y, theta) tuples
        # Records position before and after each push
        self._positions: List[Tuple[float, float, float]] = []
        self._pending_start_record = True  # Record start of next push

    def plan(self, obs: Observation) -> Optional[PushSubgoal]:
        """Generate push subgoal."""
        if self._done:
            return None

        # Track initial position on first call
        obj = obs.objects.get(self._object_id)
        if obj is None:
            print(f"[SimplePushPlanner] Object '{self._object_id}' not found")
            return None

        if self._initial_pos is None:
            self._initial_pos = (obj.x, obj.y)
            print(f"[SimplePushPlanner] Initial position: ({obj.x:.1f}, {obj.y:.1f})")

        # Record position before push starts
        if self._pending_start_record:
            self._positions.append((obj.x, obj.y, obj.theta))
            print(f"[SimplePushPlanner] Push {self._subgoal_count + 1} START: "
                  f"({obj.x:.1f}, {obj.y:.1f}, θ={obj.theta:.1f}°)")
            self._pending_start_record = False

        return PushSubgoal(
            object_id=self._object_id,
            edge_idx=self._edge_idx,
            push_steps=self._push_steps,
        )

    def notify_subgoal_done(self, obs: Observation) -> None:
        """Called when push subgoal completes."""
        self._subgoal_count += 1

        obj = obs.objects.get(self._object_id)
        if obj:
            # Record position after push
            self._positions.append((obj.x, obj.y, obj.theta))
            print(f"[SimplePushPlanner] Push {self._subgoal_count} END: "
                  f"({obj.x:.1f}, {obj.y:.1f}, θ={obj.theta:.1f}°)")

            if self._initial_pos:
                dx = obj.x - self._initial_pos[0]
                dy = obj.y - self._initial_pos[1]
                dist = math.hypot(dx, dy)
                print(
                    f"[SimplePushPlanner] Subgoal {self._subgoal_count}/{self._num_repeats} done. "
                    f"Object moved {dist:.1f}cm total"
                )

        # Ready to record start of next push
        self._pending_start_record = True

        if not self._continuous and self._subgoal_count >= self._num_repeats:
            self._done = True

    def is_complete(self, obs: Observation) -> bool:
        """Check if push plan is complete."""
        return self._done

    def get_drawings(self) -> List[Dict[str, Any]]:
        """Visualize push direction."""
        return []

    def reset(self) -> None:
        """Reset planner state."""
        self._done = False
        self._initial_pos = None
        self._subgoal_count = 0
        self._positions = []
        self._pending_start_record = True

    def save_trajectory_plot(self, output_path: Optional[str] = None) -> str:
        """Save x,y plot of object positions before and after each push.

        Args:
            output_path: Optional path for output file. If None, auto-generates.

        Returns:
            Path to saved plot file.
        """
        if len(self._positions) < 2:
            print("[SimplePushPlanner] Not enough positions to plot")
            return ""

        fig, ax = plt.subplots(figsize=(8, 8))

        # Plot before/after for each push
        for i in range(0, len(self._positions) - 1, 2):
            before = self._positions[i]
            after = self._positions[i + 1]
            push_num = i // 2 + 1

            # Before point (blue)
            ax.plot(before[0], before[1], 'bo', markersize=8)

            # After point (red)
            ax.plot(after[0], after[1], 'ro', markersize=8)

            # Arrow from before to after
            ax.annotate('', xy=(after[0], after[1]), xytext=(before[0], before[1]),
                       arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title(f'Push: {self._object_id}')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Generate output path
        if output_path is None:
            output_dir = Path(__file__).parent.parent / "output_dump"
            output_dir.mkdir(exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = str(output_dir / f"push_trajectory_{timestamp}.png")

        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()

        print(f"[SimplePushPlanner] Plot saved: {output_path}")
        return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Test push controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--object",
        type=str,
        default="push_box",
        help="Object ID to push (default: push_box)",
    )
    parser.add_argument(
        "--edge",
        type=int,
        default=0,
        help="Edge index (0 to 4*points_per_face-1, default: 0)",
    )
    parser.add_argument(
        "--push-steps",
        type=int,
        default=None,
        help="Control ticks per push (default: from config/controller.yaml)",
    )
    parser.add_argument(
        "--repeat", "-n",
        type=int,
        default=2,
        help="Number of times to repeat the push (default: 2)",
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Keep pushing indefinitely",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Vision only, don't send commands to robot",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Use simulation instead of real robot",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Maximum speed (default: 0.3)",
    )
    parser.add_argument(
        "--objects-config",
        type=str,
        default=None,
        help="Path to objects YAML (default: config/push_objects.yaml for real, config/objects.yaml for sim)",
    )
    args = parser.parse_args()

    # Load config
    from robot_control.controller.config import load_controller_configs
    from robot_control.controller.edge_points import get_face_name
    configs = load_controller_configs()
    n = configs.push.points_per_face
    max_edge = 4 * n - 1

    # Use config push_steps if not specified on command line
    push_steps = args.push_steps if args.push_steps is not None else configs.push.push_steps

    # Validate edge index
    if args.edge < 0 or args.edge > max_edge:
        print(f"Error: edge must be 0-{max_edge} for points_per_face={n}")
        return

    # Determine face from edge index
    if args.edge < 2 * n:
        face_idx = 0 if args.edge % 2 == 0 else 1
    else:
        face_idx = 2 if (args.edge - 2 * n) % 2 == 0 else 3

    print("\n" + "=" * 50)
    print("Push Controller Test")
    print("=" * 50)
    print(f"Object: {args.object}")
    print(f"Edge: {args.edge} = {get_face_name(face_idx)}")
    print(f"Points per face: {n} ({4*n} total edges, 0-{max_edge})")
    print(f"Push steps: {push_steps} ticks (~{push_steps/30:.1f}s)")
    print(f"Repeat: {args.repeat} times")
    print(f"Continuous: {args.continuous}")
    print(f"Mode: {'Simulation' if args.sim else 'Real Robot'}")
    if not args.sim:
        print(f"Dry run: {args.dry_run}")
    print("=" * 50 + "\n")

    # Create planner
    planner = SimplePushPlanner(
        object_id=args.object,
        edge_idx=args.edge,
        push_steps=push_steps,
        continuous=args.continuous,
        num_repeats=args.repeat,
    )

    # Determine objects config path
    if args.objects_config:
        objects_path = args.objects_config
    else:
        # Default: use push_objects.yaml for real, objects.yaml for sim
        config_dir = Path(__file__).parent.parent / "config"
        if args.sim:
            objects_path = str(config_dir / "objects.yaml")
        else:
            objects_path = str(config_dir / "push_objects.yaml")

    if args.sim:
        # Simulation mode
        try:
            from robot_control import SimConfig
        except ImportError:
            print("Error: Simulation mode requires SimConfig")
            return

        # Create sim config with object
        sim_config = SimConfig(
            x=10, y=20, theta=0,
            objects={
                args.object: (25, 35, 0),  # Place object in center-ish
            }
        )

        config = RuntimeConfig(
            mode="sim",
            sim_config=sim_config,
            objects_path=objects_path,
            planner=planner,
            initial_speed=args.speed,
            quit_on_complete=not args.continuous,
        )
    else:
        # Real robot mode
        config = RuntimeConfig(
            mode="real",
            config_path="config/real.yaml",
            objects_path=objects_path,
            dry_run=args.dry_run,
            planner=planner,
            initial_speed=args.speed,
            quit_on_complete=not args.continuous,
        )

    # Run
    runtime = Runtime(config)
    runtime.run()

    # Save trajectory plot
    planner.save_trajectory_plot()

    print("\n[test_push] Done")


if __name__ == "__main__":
    main()
