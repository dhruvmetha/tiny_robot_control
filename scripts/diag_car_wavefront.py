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
    obj_info = env.get_object_info()
    obj4_pose_m = (obj_info[sim_name].get("pos_x", 0.04707),
                   obj_info[sim_name].get("pos_y", 0.27035))

    print(f"=== env state ===")
    print(f"  world bounds:        {bounds}")
    print(f"  robot pose (cm):     ({rx*100:.2f}, {ry*100:.2f})")
    print(f"  obstacle_1_movable info: {obj_info.get(sim_name)}")

    # Test 1: determinism — call get_reachable_edges TWICE in a row, no other ops
    print(f"\n=== Test 1: get_reachable_edges determinism ===")
    e1 = sorted(env.get_reachable_edges(sim_name))
    e2 = sorted(env.get_reachable_edges(sim_name))
    print(f"  call 1: {e1}")
    print(f"  call 2: {e2}")
    print(f"  match:  {e1 == e2}")

    # Test 2: probe is_robot_goal_reachable at each "claimed reachable" edge
    print(f"\n=== Test 2: cross-check each reachable edge with is_robot_goal_reachable ===")
    print(f"  {'edge':>4}  {'world (cm)':>20}  {'goal_reachable':>15}  {'in_get_edges':>13}")
    for ei in e1:
        wx, wy = cpp_edge_world(OBJECTS_CM["obj_4"], ei)
        env.set_robot_pose(rx, ry, math.radians(ROBOT_POSE_CM[2]))
        env.set_robot_goal(wx, wy, 0.0)
        gr = env.is_robot_goal_reachable()
        print(f"  {ei:>4}  ({wx*100:>7.2f}, {wy*100:>7.2f})   {str(gr):>15}  {'True':>13}")

    # Test 3: call get_reachable_edges ONE MORE TIME after the probe loop
    e3 = sorted(env.get_reachable_edges(sim_name))
    print(f"\n=== Test 3: post-probe consistency ===")
    print(f"  initial:    {e1}")
    print(f"  post-probe: {e3}")
    print(f"  drift:      {set(e1) ^ set(e3)}")

    # Test 4: Independent Python wavefront recomputation
    print(f"\n=== Test 4: pure-Python wavefront recomputation ===")
    # Match C++ inflation: robot_radius=max(hx,hy)=0.035, tier1_margin=0.005
    res = 0.005
    cfg = WavefrontConfig(resolution=res, robot_radius=0.035, inflation_margin=0.005)
    wf = WavefrontPlanner(cfg)
    # Match what C++ get_environment_bounds returns: include padding + robot pos
    py_obj_data = {}
    for name, (x, y, th, w, d, _, is_static) in OBJECTS_CM.items():
        hx = d / 200.0  # depth -> X half-extent (cm/200)
        hy = w / 200.0  # width -> Y half-extent
        py_obj_data[name] = (x / 100, y / 100, hx, hy, th)
    # Boundary walls (matching xml_generator placement)
    wt = 0.01
    walls_outer = (-wt, WS_BOUNDS_CM[1] / 100 + wt,
                   -wt, WS_BOUNDS_CM[3] / 100 + wt)
    py_obj_data["wall_1"] = (walls_outer[0], (walls_outer[2] + walls_outer[3]) / 2,
                              wt, (walls_outer[3] - walls_outer[2]) / 2 + wt, 0)
    py_obj_data["wall_2"] = (walls_outer[1], (walls_outer[2] + walls_outer[3]) / 2,
                              wt, (walls_outer[3] - walls_outer[2]) / 2 + wt, 0)
    py_obj_data["wall_3"] = ((walls_outer[0] + walls_outer[1]) / 2, walls_outer[2],
                              (walls_outer[1] - walls_outer[0]) / 2, wt, 0)
    py_obj_data["wall_4"] = ((walls_outer[0] + walls_outer[1]) / 2, walls_outer[3],
                              (walls_outer[1] - walls_outer[0]) / 2, wt, 0)
    # Use bounds matching C++ for fair comparison
    py_bounds = (bounds[0], bounds[1], bounds[2], bounds[3])
    wf.build_grid(py_bounds, py_obj_data)
    # Run reachability BFS from robot pose
    wf.compute_reachability((rx, ry))

    print(f"  grid_shape (W,H):     {wf._grid.shape}")
    print(f"  cells with value=1:   {(wf._grid == 1).sum() if hasattr(wf, '_grid') else 'n/a'}")
    print(f"  cells with value=-1:  {(wf._grid == -1).sum() if hasattr(wf, '_grid') else 'n/a'}")
    print(f"  cells with value=0:   {(wf._grid == 0).sum() if hasattr(wf, '_grid') else 'n/a'}")

    # Compare edge points
    print(f"\n  Per-edge comparison (Python independent wavefront vs C++ get_reachable_edges):")
    print(f"  {'edge':>4}  {'world (cm)':>20}  {'py_cell_val':>11}  {'in_cpp_edges':>13}")
    for ei in sorted(set(e1) | set(range(0, 22, 2)) | {30, 31, 55, 57, 59}):
        wx, wy = cpp_edge_world(OBJECTS_CM["obj_4"], ei)
        # Query Python wavefront
        try:
            free = wf.is_free(wx, wy)
            # Also query reachability
            reachable = wf.is_reachable(wx, wy) if hasattr(wf, "is_reachable") else None
            val = "1" if reachable else ("0 (free,unreach)" if free else "-1 (obstacle)")
        except Exception as e:
            val = f"err:{e}"
        in_cpp = ei in e1
        print(f"  {ei:>4}  ({wx*100:>7.2f}, {wy*100:>7.2f})   {val:>11}  {str(in_cpp):>13}")


if __name__ == "__main__":
    main()
