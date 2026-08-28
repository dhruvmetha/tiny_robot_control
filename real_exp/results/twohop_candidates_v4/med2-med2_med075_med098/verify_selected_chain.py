import json
import math
import sys
from pathlib import Path

WORKTREE = Path("/home/dhruv/.config/superpowers/worktrees/namo_cpp/real-inventory-twohop")
sys.path.insert(0, str(WORKTREE / "scripts" / "pipeline"))
sys.path.insert(0, "/home/dhruv/projects_dhruv/namo/namo_cpp/build_python")
sys.path.insert(0, str(WORKTREE / "python"))

import compose_keyhole_modules as compose
import namo_rl
from namo.core.xml_goal_parser import extract_goal_from_xml


SELECTED = Path(
    "/home/dhruv/projects_dhruv/namo/robot_control/real_exp/environments/"
    "twohop_selected/med2-med2/twohop_00001"
)
CONFIG = str(WORKTREE / "config/namo_config_complete_skill15_car_1x.yaml")
OUTPUT = Path(
    "/home/dhruv/projects_dhruv/namo/robot_control/real_exp/results/"
    "twohop_candidates_v4/med2-med2_med075_med098/selected_chain_replay.json"
)


def dtheta(left, right):
    return abs(math.atan2(math.sin(right - left), math.cos(right - left)))


def observation(env):
    return {
        name: [float(value[0]), float(value[1]), float(value[2])]
        for name, value in env.get_observation().items()
    }


def state_record(env):
    path, boundaries = compose._current_path(env)
    return {
        "path": path,
        "hop_count": len(path) - 1 if path else -1,
        "boundaries": boundaries,
        "goal_reachable": bool(env.is_robot_goal_reachable()),
        "observation": observation(env),
    }


def compare_to_xml(actual, xml_path):
    expected_env = namo_rl.RLEnvironment(str(xml_path), CONFIG, False)
    expected = observation(expected_env)
    common = sorted(set(actual) & set(expected))
    dxy = {name: math.dist(actual[name][:2], expected[name][:2]) for name in common}
    dth = {name: dtheta(actual[name][2], expected[name][2]) for name in common}
    return {
        "missing_in_replay": sorted(set(expected) - set(actual)),
        "extra_in_replay": sorted(set(actual) - set(expected)),
        "max_dxy_mm": round(1000.0 * max(dxy.values()), 4),
        "max_dtheta_deg": round(math.degrees(max(dth.values())), 4),
        "per_body_dxy_mm": {name: round(1000.0 * value, 4) for name, value in dxy.items()},
        "per_body_dtheta_deg": {
            name: round(math.degrees(value), 4) for name, value in dth.items()
        },
    }


sheet = json.loads((SELECTED / "build_sheet.json").read_text())
actions = [sheet["k1_canonical_opener"], sheet["k2_canonical_opener"]]
env = namo_rl.RLEnvironment(str(SELECTED / "env.xml"), CONFIG, False)
env.set_robot_goal(*extract_goal_from_xml(str(SELECTED / "env.xml")))

states = [state_record(env)]
steps = []
for gate_index, opener in enumerate(actions):
    object_id = opener[1]
    gate_steps = []
    for edge, depth in opener[2]:
        result = env.step(compose._action(object_id, edge, depth))
        gate_steps.append(
            {
                "object_id": object_id,
                "edge": edge,
                "depth": depth,
                "done": bool(result.done),
            }
        )
    steps.append(gate_steps)
    states.append(state_record(env))

replay_comparisons = [
    compare_to_xml(states[1]["observation"], SELECTED / "post_k1_env.xml"),
    compare_to_xml(states[2]["observation"], SELECTED / "post_k2_env.xml"),
]
static_probe = compose.probe_one((str(SELECTED / "env.xml"), CONFIG, 2))
static_ok, static_reason = compose.static_acceptance(static_probe, 2)

k1_to_k2_position_mm = 1000.0 * math.dist(
    states[0]["observation"]["obstacle_1_movable_pose"][:2],
    states[1]["observation"]["obstacle_1_movable_pose"][:2],
)
k1_to_k2_angle_deg = math.degrees(
    dtheta(
        states[0]["observation"]["obstacle_1_movable_pose"][2],
        states[1]["observation"]["obstacle_1_movable_pose"][2],
    )
)
k2_to_k1_position_mm = 1000.0 * math.dist(
    states[1]["observation"]["obstacle_0_movable_pose"][:2],
    states[2]["observation"]["obstacle_0_movable_pose"][:2],
)
k2_to_k1_angle_deg = math.degrees(
    dtheta(
        states[1]["observation"]["obstacle_0_movable_pose"][2],
        states[2]["observation"]["obstacle_0_movable_pose"][2],
    )
)

failure = None
if not static_ok:
    failure = f"static_{static_reason}"
elif [state["hop_count"] for state in states] != [2, 1, 0]:
    failure = "hop_trace"
elif states[0]["boundaries"] != [["obstacle_0_movable"], ["obstacle_1_movable"]]:
    failure = "initial_boundaries"
elif states[1]["boundaries"] != [["obstacle_1_movable"]]:
    failure = "post_k1_boundary"
elif not states[2]["goal_reachable"]:
    failure = "final_goal_unreachable"
elif not all(step["done"] for gate in steps for step in gate):
    failure = "push_failed"
elif any(comparison["max_dxy_mm"] > 0.1 for comparison in replay_comparisons):
    failure = "state_replay_position_divergence"
elif any(comparison["max_dtheta_deg"] > 0.1 for comparison in replay_comparisons):
    failure = "state_replay_angle_divergence"
elif k1_to_k2_position_mm > 2.0 or k1_to_k2_angle_deg > 1.0:
    failure = "k1_moved_k2"
elif k2_to_k1_position_mm > 2.0 or k2_to_k1_angle_deg > 1.0:
    failure = "k2_moved_k1"

record = {
    "status": "passed" if failure is None else "failed",
    "failure": failure,
    "selected_xml": str(SELECTED / "env.xml"),
    "actions": actions,
    "steps": steps,
    "hop_trace": [state["hop_count"] for state in states],
    "boundary_trace": [state["boundaries"] for state in states],
    "goal_reachable_trace": [state["goal_reachable"] for state in states],
    "mechanical_independence": {
        "k1_to_k2_position_delta_mm": round(k1_to_k2_position_mm, 4),
        "k1_to_k2_angle_delta_deg": round(k1_to_k2_angle_deg, 4),
        "k2_to_k1_position_delta_mm": round(k2_to_k1_position_mm, 4),
        "k2_to_k1_angle_delta_deg": round(k2_to_k1_angle_deg, 4),
    },
    "replay_state_comparisons": replay_comparisons,
    "static_probe": static_probe,
    "static_acceptance": {"passed": static_ok, "reason": static_reason},
}
OUTPUT.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
print(json.dumps({key: record[key] for key in (
    "status", "failure", "hop_trace", "boundary_trace", "goal_reachable_trace",
    "mechanical_independence", "replay_state_comparisons", "static_acceptance"
)}, indent=2, sort_keys=True))
raise SystemExit(0 if failure is None else 2)
