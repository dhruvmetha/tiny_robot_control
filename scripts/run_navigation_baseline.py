"""Score captured scenes with the pure-navigation baselines.

Answers one question per scene: could a robot that never pushes have got to
the goal, and if it drove straight there, what would it have hit?

    python scripts/run_navigation_baseline.py real_test_envs/1push/1hop/env1
    python scripts/run_navigation_baseline.py --all
    python scripts/run_navigation_baseline.py --all --csv baseline.csv

The scene comes from `namo_rl`, not from parsing the XML here. MuJoCo accepts
rotations as `quat`, `euler`, `axisangle`, `zaxis` or `xyaxes` and resolves
them all into one quaternion. A reader that understands only some of those
sees a different room from the planner it is meant to sit below, which is not
a subtle failure: the twohop scenes rotate their inner walls with `euler`, and
a hand parser looking only for `quat` reads four walls flat that are actually
turned 2.7, 109, 171 and 89.7 degrees. Flat, they span the table and seal it.
Turned, they do not. That produced a whole afternoon of "the two planners
disagree" before the disagreement turned out to be the reader.

So the geometry is taken from `get_object_info` (statics, with pose) and
`get_observation` (movable and robot poses), which is what the physics engine
resolved. The robot's start comes from there too, so no scene is skipped for
want of a sidecar.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).parent))
from run_namo import _robot_pose_from_real_run  # noqa: E402

from robot_control.planner.navigation_baseline import (
    MOVABLE_COST_IGNORED,
    MOVABLE_COST_PENALISED,
    NavigationBaseline,
    ObjectBoxes,
    sweep_movable_cost,
)

# The table the robot drives on, from the build-card validator.
WORKSPACE_W_CM = 49.0
WORKSPACE_H_CM = 77.5
ROBOT_W_CM = 7.0
ROBOT_H_CM = 7.0

# Penalties the sweep tries, ascending. The first one that clears a scene is
# reported. Spread rather than dense, because the useful distinction is
# "cheap detour available" against "only a long way round" against "no way
# round at all", not the second decimal place.
SWEEP_PENALTIES = (1.0, 1.25, 1.5, 2.0, 3.0, 5.0, 10.0, 25.0, 100.0)

# A body whose name ends this way is pushable. Same rule the rest of the
# pipeline uses.
MOVABLE_SUFFIX = "_movable"

# A robot within this distance of (0, 0) was never placed. The origin is the
# bottom-left table corner, inside the boundary wall, so no scene starts there.
ORIGIN_EPS_M = 0.01

# `get_object_info` reports statics with a pose and movables with size only,
# because a movable's pose is live state and belongs in the observation. The
# presence of this key is what separates the two.
STATIC_POSE_KEY = "pos_x"


def yaw_deg_from_quat(quat_w: float, quat_z: float) -> float:
    """MuJoCo's resolved quaternion, z-axis rotation only, in degrees."""
    return math.degrees(2.0 * math.atan2(quat_z, quat_w))


def read_goal_xy(xml_path: Path) -> Tuple[float, float]:
    """The goal marker, which is a plain position with no rotation to get wrong."""
    root = ET.parse(xml_path).getroot()
    for site in root.iter("site"):
        if site.get("name") == "goal":
            gp = [float(v) for v in (site.get("pos") or "0 0 0").split()]
            return (gp[0], gp[1])
    raise ValueError(f"{xml_path} has no site named 'goal'")


def read_scene_via_mujoco(
    xml_path: Path, config_path: str
) -> Tuple[ObjectBoxes, ObjectBoxes, Tuple[float, float], Tuple[float, float]]:
    """Statics, movables, robot start and goal, as the physics engine sees them.

    Everything geometric comes from `namo_rl` so the baseline plans against the
    same world the planner does. Parsing the XML here instead is what produced
    four flat walls out of four rotated ones.
    """
    import contextlib
    import io

    import namo_rl

    with contextlib.redirect_stdout(io.StringIO()):
        env = namo_rl.RLEnvironment(str(xml_path), config_path, False)
        info = env.get_object_info()
        obs = env.get_observation()

    statics: ObjectBoxes = {}
    movables: ObjectBoxes = {}
    for name, fields in info.items():
        if name == "robot":
            continue
        half_x = float(fields["size_x"])
        half_y = float(fields["size_y"])
        if STATIC_POSE_KEY in fields:
            theta_deg = yaw_deg_from_quat(float(fields["quat_w"]), float(fields["quat_z"]))
            statics[name] = (float(fields["pos_x"]), float(fields["pos_y"]),
                             half_x, half_y, theta_deg)
        else:
            pose = obs.get(f"{name}_pose")
            if pose is None:
                raise ValueError(
                    f"{xml_path}: {name} has no size-only entry paired with a "
                    f"pose in the observation; cannot place it."
                )
            movables[name] = (float(pose[0]), float(pose[1]),
                              half_x, half_y, math.degrees(float(pose[2])))

    return statics, movables, read_robot_start(xml_path, obs), read_goal_xy(xml_path)


def read_robot_start(xml_path: Path, obs: Dict) -> Tuple[float, float]:
    """Where the robot actually starts, in metres.

    A scene that places its robot body carries the pose in the observation and
    that is the answer. The real_test_envs captures do not: they `<include>`
    the car model instead of positioning it, so MuJoCo puts the body at the
    origin. The origin is the bottom-left corner of the table, buried inside
    the boundary wall, and a baseline scored from there reports no route and
    means nothing by it.

    Those captures keep the true pose in `mid_obs.jsonl`, the same sidecar
    run_namo reads, so read it from there. A scene with neither is refused
    rather than scored from a placeholder, because a wrong start produces a
    confident answer to a question nobody asked.
    """
    pose = obs.get("robot_pose")
    if pose is not None and math.hypot(float(pose[0]), float(pose[1])) > ORIGIN_EPS_M:
        return (float(pose[0]), float(pose[1]))

    sidecar = _robot_pose_from_real_run(xml_path.parent)
    if sidecar is not None:
        return (sidecar[0] / 100.0, sidecar[1] / 100.0)

    raise ValueError(
        f"{xml_path}: the robot sits at the origin and {xml_path.parent}/"
        f"mid_obs.jsonl carries no pose, so the start is unknown"
    )


def save_route_image(
    baseline: NavigationBaseline,
    result,
    out_path: Path,
    penalty: float,
) -> None:
    """Draw the scene and the route the baseline chose.

    `WavefrontPlanner.save` already renders exactly what is worth seeing:
    static geometry black, cells shaded by their cost, and the route as a blue
    line. With proximity weighting off, a priced cell saturates rather than
    grading, so movables read as solid yellow against white floor, which is
    the distinction the whole baseline turns on.
    """
    baseline.plan_for_image(penalty)
    baseline.planner.save(str(out_path), path=result.waypoints or None)


def score_scene(
    scene_dir: Path,
    config_path: str,
    image_dir: Optional[Path] = None,
) -> Dict[str, object]:
    statics, movables, start_m, goal_m = read_scene_via_mujoco(
        scene_dir / "env.xml", config_path
    )

    baseline = NavigationBaseline(
        bounds=(0.0, WORKSPACE_W_CM / 100.0, 0.0, WORKSPACE_H_CM / 100.0),
        statics=statics,
        movables=movables,
        robot_width_cm=ROBOT_W_CM,
        robot_height_cm=ROBOT_H_CM,
    )
    ignored = baseline.plan(start_m, goal_m, MOVABLE_COST_IGNORED)
    penalised = baseline.plan(start_m, goal_m, MOVABLE_COST_PENALISED)
    threshold = sweep_movable_cost(baseline, start_m, goal_m, SWEEP_PENALTIES)

    if image_dir is not None:
        image_dir.mkdir(parents=True, exist_ok=True)
        stem = "_".join(scene_dir.parts[-3:])
        save_route_image(baseline, ignored, image_dir / f"{stem}_ignored.png",
                         MOVABLE_COST_IGNORED)
        save_route_image(baseline, penalised, image_dir / f"{stem}_penalised.png",
                         MOVABLE_COST_PENALISED)

    return {
        "scene": str(scene_dir),
        "movables": len(movables),
        "ignored_reached": ignored.reached,
        "ignored_crosses": sum(ignored.movable_cells_crossed.values()),
        "penalised_reached": penalised.reached,
        "penalised_crosses": sum(penalised.movable_cells_crossed.values()),
        "penalised_hits": ",".join(sorted(penalised.movable_cells_crossed)),
        # None means no penalty in the sweep found a clear route, so every way
        # to the goal goes through a block. That scene genuinely needs a push.
        "clear_route_at_penalty": threshold,
        "needs_a_push": threshold is None,
        "failure": penalised.failure or "",
        "start_was_trapped": penalised.start_was_trapped,
    }


def find_scenes(roots: List[str]) -> List[Path]:
    """Scene directories, skipping captures nested inside another scene.

    A scene folder often holds further `env.xml` files recording the world
    AFTER the push chain ran, `post_chain_scene_bundle` among them. Those are
    end states, not problems, and they are trivially navigable because the
    pushes already happened. Counting one as a scene the robot could have
    driven would be a finding about my globbing, not about the scene library.
    """
    found: List[Path] = []
    for root in roots:
        candidates = sorted(p.parent for p in Path(root).rglob("env.xml"))
        for scene in candidates:
            if any(other != scene and other in scene.parents for other in candidates):
                continue
            found.append(scene)
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenes", nargs="*", help="Scene directories holding env.xml.")
    parser.add_argument("--all", action="store_true",
                        help="Score every scene under real_test_envs and real_exp/environments.")
    parser.add_argument("--csv", type=str, default=None, help="Also write rows here.")
    parser.add_argument("--config", type=str,
                        default="/home/dhruv/projects_dhruv/namo/namo_cpp/config/"
                                "namo_config_complete_skill15_car_1x.yaml",
                        help="namo_cpp config the scene loads against.")
    parser.add_argument("--images", type=str, default=None,
                        help="Directory to write one PNG per scene per variant: "
                             "walls black, movables yellow, route blue.")
    args = parser.parse_args()

    if args.all:
        dirs = find_scenes(["real_test_envs", "real_exp/environments"])
    elif args.scenes:
        dirs = [Path(s) for s in args.scenes]
    else:
        parser.error("give scene directories or --all")

    rows: List[Dict[str, object]] = []
    skipped: List[Tuple[Path, str]] = []
    for scene_dir in dirs:
        try:
            rows.append(score_scene(scene_dir, args.config,
                                    Path(args.images) if args.images else None))
        except Exception as exc:
            # Named, never silently dropped. A scene the loader refuses is a
            # scene whose geometry nobody has checked.
            skipped.append((scene_dir, str(exc)[:90]))

    print(f"{'scene':58s} {'movables':>8s} {'drives through':>15s} {'clear at':>9s}")
    for row in rows:
        clear = row["clear_route_at_penalty"]
        print(f"  {str(row['scene'])[-56:]:56s} {row['movables']:>8d} "
              f"{row['penalised_crosses']:>15d} "
              f"{'never' if clear is None else f'{clear:g}':>9s}")

    needs = sum(1 for r in rows if r["needs_a_push"])
    nav_only = len(rows) - needs
    print(f"\n{len(rows)} scenes scored, {len(skipped)} skipped")
    print(f"  {needs} need a push: every route to the goal crosses a movable")
    print(f"  {nav_only} solvable by driving round everything")
    if nav_only:
        print("\n  Scenes a robot could have driven, which do not belong in a NAMO matrix:")
        for row in rows:
            if not row["needs_a_push"]:
                print(f"    {row['scene']}  (clear at penalty {row['clear_route_at_penalty']:g})")
    if skipped:
        print("\n  Skipped, with the reason:")
        for path, why in skipped[:10]:
            print(f"    {path}: {why}")
        if len(skipped) > 10:
            print(f"    ... and {len(skipped) - 10} more")

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else [])
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nwrote {args.csv}")
    if args.images:
        print(f"wrote route images to {args.images}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
