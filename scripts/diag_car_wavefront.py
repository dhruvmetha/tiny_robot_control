"""Diagnostic: find why C++ get_reachable_edges disagrees with is_robot_goal_reachable
on the same cell. NO C++ rebuild. Uses existing bindings + the pure-Python
WavefrontPlanner mirror in robot_control/utils/wavefront.py to independently
recompute the grid.
"""
from __future__ import annotations
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot_control.utils import NAMOXMLGenerator
from robot_control.utils.robot_geometry import effective_robot_radius_cm
from robot_control.utils.wavefront import WavefrontPlanner, WavefrontConfig

import namo_rl  # type: ignore

# Frozen scene from 200844_random_rollout_car/plans.jsonl plan #1
ROBOT_POSE_CM = (25.893, 9.971, 99.012)
OBJECTS_CM = {
    "obj_4":   (4.707, 27.035,  9.580, 12.0,  7.5, 5.0, False),
    "wall_11": (20.349, 26.332, 90.121, 5.5, 19.0, 10.0, True),
    "wall_9":  (40.333, 26.342,  1.419, 5.5, 19.0, 10.0, True),
}
GOAL_CM = (40.0, 60.0)
WS_BOUNDS_CM = (0.0, 49.0, 0.0, 77.5)
NAMO_CONFIG = (
    Path(__file__).resolve().parents[2] / "namo_cpp" / "config"
    / "namo_config_complete_skill15_car_1x.yaml"
)


def build_env():
    rcm = effective_robot_radius_cm(7.0, 7.0)
    gen = NAMOXMLGenerator(scale_factor=1.0, robot_radius_cm=rcm, robot_model="car")
    xml = gen.from_observation(
        robot_x_cm=ROBOT_POSE_CM[0], robot_y_cm=ROBOT_POSE_CM[1],
        objects=OBJECTS_CM,
        goal_x_cm=GOAL_CM[0], goal_y_cm=GOAL_CM[1],
        workspace_bounds_cm=WS_BOUNDS_CM,
    )
    fd, xml_path = tempfile.mkstemp(suffix=".xml", prefix="diag_wave_")
    Path(xml_path).write_text(xml)

    rx_m, ry_m = ROBOT_POSE_CM[0] / 100, ROBOT_POSE_CM[1] / 100
    rtheta = math.radians(ROBOT_POSE_CM[2])
    env = namo_rl.RLEnvironment(xml_path, str(NAMO_CONFIG), False, True)
    env.set_robot_pose(rx_m, ry_m, rtheta)
    env.warm_up()
    env.set_robot_goal(GOAL_CM[0] / 100, GOAL_CM[1] / 100, 0.0)
    return env, (rx_m, ry_m, rtheta)


def cpp_edge_world(obj_pose_cm, edge_idx, points_per_face=15,
                   robot_size_m=(0.035, 0.035), push_offset=0.01):
    """Mirror of namo_push_controller.cpp generate_rectangular_edge_points."""
    ox, oy, otheta_deg, w_full, d_full, _, _ = obj_pose_cm
    half_x = d_full / 200.0  # depth -> X half-extent (matches xml_generator)
    half_y = w_full / 200.0  # width -> Y half-extent
    yaw = math.radians(otheta_deg)
    n = points_per_face
    offset = max(robot_size_m[0], robot_size_m[1]) + push_offset

    locals_ = []
    # top/bottom loop
    for j in range(n):
        u = -half_x + (2 * half_x) * j / (n - 1) if n > 1 else 0.0
        locals_.append((u, half_y + offset))     # top j (even idx)
        locals_.append((u, -half_y - offset))    # bottom j (odd idx)
    # right/left loop
    for k in range(n):
        v = -half_y + (2 * half_y) * k / (n - 1) if n > 1 else 0.0
        locals_.append((half_x + offset, v))     # right k
        locals_.append((-half_x - offset, v))    # left k

    lx, ly = locals_[edge_idx]
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (cy * lx - sy * ly + ox / 100,
            sy * lx + cy * ly + oy / 100)


def main():
    env, (rx, ry, _) = build_env()

    sim_name = "obstacle_1_movable"
    bounds = env.get_world_bounds()
    obs = env.get_observation()

    # CRITICAL: use the *current* MuJoCo pose, not the input pose.
    # warm_up() runs 3 physics steps; the object can drift slightly.
    obj4_pose_key = sim_name + "_pose" if (sim_name + "_pose") in obs else sim_name
    robot_pose_key = "robot_pose" if "robot_pose" in obs else "robot"
    obj4_live = obs.get(obj4_pose_key)
    robot_live = obs.get(robot_pose_key)

    print(f"=== env state ===")
    print(f"  world bounds:        {bounds}")
    print(f"  obs keys:            {sorted(obs.keys())}")
    print(f"  robot input (cm):    ({rx*100:.4f}, {ry*100:.4f})")
    print(f"  robot live (m,rad):  {robot_live}")
    print(f"  obj_4 input (cm):    ({OBJECTS_CM['obj_4'][0]:.4f}, "
          f"{OBJECTS_CM['obj_4'][1]:.4f}, θ={OBJECTS_CM['obj_4'][2]:.4f}°)")
    print(f"  obj_4 live (m,rad):  {obj4_live}")
    if obj4_live is None:
        print("  [ERROR] obj_4 not found in observation; aborting")
        return

    # Build a "live" obj_4 description so cpp_edge_world matches what
    # C++ generate_rectangular_edge_points actually computes.
    live_obj4 = (
        obj4_live[0] * 100,           # x cm
        obj4_live[1] * 100,           # y cm
        math.degrees(obj4_live[2]),   # theta deg
        OBJECTS_CM["obj_4"][3],       # width cm (immutable)
        OBJECTS_CM["obj_4"][4],       # depth cm (immutable)
        OBJECTS_CM["obj_4"][5],
        OBJECTS_CM["obj_4"][6],
    )

    # Test 1: determinism — call get_reachable_edges TWICE in a row, no other ops
    print(f"\n=== Test 1: get_reachable_edges determinism ===")
    e1 = sorted(env.get_reachable_edges(sim_name))
    e2 = sorted(env.get_reachable_edges(sim_name))
    print(f"  call 1: {e1}")
    print(f"  call 2: {e2}")
    print(f"  match:  {e1 == e2}")

    # Test 2: probe is_robot_goal_reachable at each "claimed reachable" edge.
    # Use the LIVE obj_4 pose (post-warmup) — matches what get_reachable_edges saw.
    print(f"\n=== Test 2: cross-check each reachable edge with is_robot_goal_reachable ===")
    print(f"  Using LIVE obj_4 pose for edge math (post warm_up MuJoCo state)")
    print(f"  {'edge':>4}  {'world (cm)':>20}  {'goal_reachable':>15}  {'in_get_edges':>13}")
    # Probe also a few NOT-in-edges as sanity
    probe_set = list(e1) + [ei for ei in (1, 3, 5, 30, 31, 32) if ei not in e1]
    for ei in probe_set:
        wx, wy = cpp_edge_world(live_obj4, ei)
        env.set_robot_pose(rx, ry, math.radians(ROBOT_POSE_CM[2]))
        env.set_robot_goal(wx, wy, 0.0)
        gr = env.is_robot_goal_reachable()
        in_e1 = ei in e1
        flag = "" if (gr == in_e1) else "  ← MISMATCH"
        print(f"  {ei:>4}  ({wx*100:>7.2f}, {wy*100:>7.2f})   {str(gr):>15}  "
              f"{str(in_e1):>13}{flag}")

    # Test 3: call get_reachable_edges ONE MORE TIME after the probe loop
    e3 = sorted(env.get_reachable_edges(sim_name))
    print(f"\n=== Test 3: post-probe consistency ===")
    print(f"  initial:    {e1}")
    print(f"  post-probe: {e3}")
    print(f"  drift:      {set(e1) ^ set(e3)}")

    # Summary
    matches = sum(1 for ei in probe_set
                  if (env.is_robot_goal_reachable.__self__ is env) and True)
    print(f"\n=== Summary ===")
    print(f"  Total claimed-reachable edges: {len(e1)}")
    print(f"  Edge math used LIVE obj_4 pose: {live_obj4[:3]}")
    if obj4_live[0] * 100 == OBJECTS_CM['obj_4'][0] and obj4_live[1] * 100 == OBJECTS_CM['obj_4'][1]:
        print(f"  Live pose matches input pose exactly (no drift).")
    else:
        dx = obj4_live[0] * 100 - OBJECTS_CM['obj_4'][0]
        dy = obj4_live[1] * 100 - OBJECTS_CM['obj_4'][1]
        dth = math.degrees(obj4_live[2]) - OBJECTS_CM['obj_4'][2]
        print(f"  Drift: dx={dx:+.4f}cm dy={dy:+.4f}cm dθ={dth:+.4f}°")


if __name__ == "__main__":
    main()
