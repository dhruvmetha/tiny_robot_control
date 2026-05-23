"""Verify: does MuJoCo interpret obj_4's euler as degrees or radians,
and is the <include> of little_car.xml's <compiler angle="radian"/> the cause?

Loads three variants of the same scene and reports obj_4's live theta:
  A) car-mode XML (current xml_generator output, includes little_car)
  B) car-mode XML with explicit <compiler angle="degree"/> injected BEFORE the include
  C) sphere-mode XML (no include)
"""
from __future__ import annotations
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot_control.utils import NAMOXMLGenerator
from robot_control.utils.robot_geometry import effective_robot_radius_cm

import namo_rl  # type: ignore

ROBOT_POSE_CM = (25.893, 9.971, 99.012)
OBJECTS_CM = {
    "obj_4":   (4.707, 27.035,  9.580, 12.0,  7.5, 5.0, False),
    "wall_11": (20.349, 26.332, 90.121, 5.5, 19.0, 10.0, True),
    "wall_9":  (40.333, 26.342,  1.419, 5.5, 19.0, 10.0, True),
}
GOAL_CM = (40.0, 60.0)
WS_BOUNDS_CM = (0.0, 49.0, 0.0, 77.5)

CONF_CAR = Path(__file__).resolve().parents[2] / "namo_cpp" / "config" / "namo_config_complete_skill15_car_1x.yaml"
CONF_SPH = Path(__file__).resolve().parents[2] / "namo_cpp" / "config" / "namo_config_complete_skill15_1x.yaml"


def gen_xml(model: str) -> str:
    rcm = effective_robot_radius_cm(7.0, 7.0)
    gen = NAMOXMLGenerator(scale_factor=1.0, robot_radius_cm=rcm, robot_model=model)
    return gen.from_observation(
        robot_x_cm=ROBOT_POSE_CM[0], robot_y_cm=ROBOT_POSE_CM[1],
        objects=OBJECTS_CM,
        goal_x_cm=GOAL_CM[0], goal_y_cm=GOAL_CM[1],
        workspace_bounds_cm=WS_BOUNDS_CM,
    )


def write(xml: str, tag: str) -> str:
    p = f"/tmp/diag_euler_{tag}.xml"
    Path(p).write_text(xml)
    return p


def load_and_probe(xml_path: str, config_path: str, label: str):
    rx, ry = ROBOT_POSE_CM[0] / 100, ROBOT_POSE_CM[1] / 100
    rth = math.radians(ROBOT_POSE_CM[2])
    env = namo_rl.RLEnvironment(xml_path, config_path, False, True)
    env.set_robot_pose(rx, ry, rth)
    env.warm_up()
    obs = env.get_observation()
    obj4 = obs.get("obstacle_1_movable_pose")
    expected_rad = math.radians(OBJECTS_CM["obj_4"][2])  # 9.58°
    expected_rad_alt = math.atan2(math.sin(OBJECTS_CM["obj_4"][2]), math.cos(OBJECTS_CM["obj_4"][2]))  # 9.58 rad wrapped
    actual_rad = obj4[2] if obj4 else float("nan")
    print(f"\n--- {label} ---")
    print(f"  obj_4 input θ: 9.58° = {expected_rad:.4f} rad")
    print(f"  obj_4 live  θ: {actual_rad:.4f} rad = {math.degrees(actual_rad):.2f}°")
    print(f"  matches 'as-degrees' (0.1672 rad)? {abs(actual_rad - 0.1672) < 0.01}")
    print(f"  matches 'as-radians wrapped' ({expected_rad_alt:.4f} rad)? "
          f"{abs(actual_rad - expected_rad_alt) < 0.01}")


def main():
    # A) car-mode current generator output
    xml_a = gen_xml("car")
    pa = write(xml_a, "A_car")
    load_and_probe(pa, str(CONF_CAR), "A: car-mode (current xml_generator)")

    # B) car-mode with explicit <compiler angle="degree"/> injected BEFORE the include
    # Insertion point: right after <mujoco model="..."> open tag, before any <option>.
    inject = '<compiler angle="degree"/>\n  '
    if "<compiler" in xml_a:
        print("\n[note] xml_generator already emits <compiler>; check ordering")
    # naive insertion: after the model open tag
    xml_b = xml_a.replace('<mujoco model="generated_environment">',
                          '<mujoco model="generated_environment">\n  ' + inject, 1)
    pb = write(xml_b, "B_car_compiler_deg")
    load_and_probe(pb, str(CONF_CAR), "B: car-mode with explicit <compiler angle='degree'/>")

    # C) sphere-mode
    xml_c = gen_xml("sphere")
    pc = write(xml_c, "C_sphere")
    load_and_probe(pc, str(CONF_SPH), "C: sphere-mode (no <include>)")

    print("\n=== Conclusion ===")
    print("If A says 'as-radians' but B and C say 'as-degrees', the bug is:")
    print("  the included little_car.xml's <compiler angle='radian'/> propagates")
    print("  to the parent model, making MuJoCo treat all euler=... attributes")
    print("  in the parent body as radians.")


if __name__ == "__main__":
    main()
