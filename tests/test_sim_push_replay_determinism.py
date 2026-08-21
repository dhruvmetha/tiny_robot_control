"""Simulator replay determinism: the same push from the same restored state.

This is item 5 of the section 10.3 simulator integration tests in
``MODEL_GUIDED_RO_INTEGRATION.local.md``. Every other test in that section
compares a planned chain against what execution produced, and each of those
comparisons is meaningless if replaying a push can drift. So this one comes
first.

The failure it guards against is real and has already happened here: before
``5daaed5`` the binding leaked ``ctrl`` and ``qacc_warmstart`` across
``set_full_state``, so a restored state was not the state you saved. Every BNG
number in the model registry predates that fix, which is why the registry warns
against comparing those absolute simulator counts to new runs.

A determinism test goes vacuous the moment the push stops happening, and a push
that collides with a wall reports success-shaped info while moving the object
zero centimetres. Both replays of a no-op agree perfectly. So every case here
asserts the object actually moved before it asserts the two runs match.

Runs against the canonical ``namo_cpp/build_python`` binding and the captured
``real_test_envs`` scenes. Skips when either is missing.

To verify:
  cd namo_cpp && source env.robotlearning.sh && cd ../robot_control
  pytest tests/test_sim_push_replay_determinism.py -v
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from robot_control.controller.edge_points import get_edge_point
from robot_control.core.types import ObjectPose
from robot_control.planner.namo_binding_loader import (
    load_canonical_namo_rl,
    resolve_namo_cpp_dir,
)
from robot_control.utils.robot_geometry import effective_robot_size_cm
from robot_control.utils.scene_xml import portable_scene

# ─── Named constants ────────────────────────────────────────────────────

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = REPO_ROOT / "real_test_envs"
# Car + 1x primitives. run_namo.find_namo_config() picks this same file for
# (scale_factor=1.0, robot_model="car"), which is what these scenes are.
# Resolved through the loader: the checkout is not called namo_cpp everywhere.
NAMO_CONFIG = (
    resolve_namo_cpp_dir(Path(__file__).resolve())
    / "config"
    / "namo_config_complete_skill15_car_1x.yaml"
)

# The scenes bake the car's spawn into the included little_car.xml, which puts
# it at the origin, wedged in the bottom-left wall corner. Measured there: zero
# reachable edges on every scene. The bridge calls set_robot_pose with the live
# observation for exactly this reason. This pose is open floor below the divider
# walls in all of these arenas.
START_POSE_M = (0.25, 0.10, 0.0)

# Push-approach geometry, mirroring scripts/execute_sim_push.py:408 so the test
# stands the car where a real dispatch would. points_per_face is production's 15
# (edge indices 0-59); the chassis is 7x7 cm.
CAR_FOOTPRINT_CM = (7.0, 7.0)
STANDOFF_FRACTION = 0.6
POINTS_PER_FACE = 15

# A push that moves the object less than this did not happen: the executor
# reports a wall collision and the object sits still. Every verified case below
# moves at least 6.5 cm, so this bar is far under the real signal and only trips
# when a case has gone vacuous.
MIN_DISPLACEMENT_M = 0.02

CM_PER_M = 100.0

# (scene, object, edge_idx, depth). Measured 2026-08-20 against binding
# 9e1b083; the comment is that run's displacement. Spans three scenes, both
# face pairs (even = top/+Y, odd = bottom/-Y), and depths 1 and 3.
PUSH_CASES = [
    ("1push/1hop/env1", "obstacle_1_movable", 14, 3),   # 19.1 cm
    ("1push/1hop/env1", "obstacle_1_movable", 12, 3),   # 18.8 cm
    ("1push/1hop/env1", "obstacle_1_movable", 4, 2),    # 11.8 cm
    ("2push/1hop/env3", "obstacle_1_movable", 1, 1),    # 6.5 cm
    ("1push/2hop/env2", "obstacle_1_movable", 14, 3),   # 18.9 cm
]

# A second push used only to dirty the simulator between replays. Different
# object face, so it drives different contacts than any case above.
INTERLEAVED_EDGE = 8
INTERLEAVED_DEPTH = 2


try:
    namo_rl, _, _ = load_canonical_namo_rl(Path(__file__))
except Exception as exc:  # pragma: no cover - environment-dependent
    pytest.skip(
        f"needs the canonical namo_cpp/build_python binding: {exc}",
        allow_module_level=True,
    )

if not SCENE_ROOT.is_dir():  # pragma: no cover - environment-dependent
    pytest.skip(f"missing captured scenes: {SCENE_ROOT}", allow_module_level=True)
if not NAMO_CONFIG.is_file():  # pragma: no cover - environment-dependent
    pytest.skip(f"missing NAMO config: {NAMO_CONFIG}", allow_module_level=True)


# ─── Helpers ────────────────────────────────────────────────────────────


def _make_env(scene: str, tmp_dir: Path):
    """Fresh environment at the start pose. Fresh per test, never shared.

    A cached environment would let one test's residue explain another's pass,
    which is the whole thing under examination here.

    The scene is loaded through portable_scene because the committed copies
    carry an absolute include from the box that captured them.
    """
    source = SCENE_ROOT / scene / "env.xml"
    if not source.is_file():
        pytest.skip(f"missing scene: {source}")
    xml = portable_scene(source, tmp_dir)
    env = namo_rl.RLEnvironment(str(xml), str(NAMO_CONFIG), False)
    env.reset()
    env.set_robot_pose(*START_POSE_M)
    return env


def _object_xy(env, object_id: str) -> tuple[float, float]:
    pose = env.get_observation()[f"{object_id}_pose"]
    return pose[0], pose[1]


def _place_robot_at_edge_point(env, object_id: str, edge_idx: int) -> None:
    """Teleport the car to the contact point for this edge.

    The C++ push skill checks wavefront reachability from wherever the robot
    stands, so starting it on the edge point keeps that check on a free cell.
    Same thing execute_sim_push.py does before a replay.
    """
    info = env.get_object_info()[object_id]
    x_m, y_m, theta_rad = env.get_observation()[f"{object_id}_pose"]
    obj = ObjectPose(
        x=x_m * CM_PER_M,
        y=y_m * CM_PER_M,
        theta=math.degrees(theta_rad),
        # get_object_info reports half-extents; ObjectPose wants full width and
        # depth in cm. depth is the X dimension, width is Y.
        width=info["size_y"] * 2 * CM_PER_M,
        depth=info["size_x"] * 2 * CM_PER_M,
        height=info["size_z"] * 2 * CM_PER_M,
        is_static=False,
    )
    standoff_cm = STANDOFF_FRACTION * effective_robot_size_cm(*CAR_FOOTPRINT_CM)
    edge_point = get_edge_point(obj, edge_idx, standoff_cm, POINTS_PER_FACE)
    env.set_robot_pose(
        edge_point.position[0] / CM_PER_M,
        edge_point.position[1] / CM_PER_M,
        math.radians(edge_point.approach_theta),
    )


def _action(object_id: str, edge_idx: int, depth: int):
    action = namo_rl.Action()
    action.object_id = object_id
    action.edge_idx = edge_idx
    action.depth = depth
    return action


def _push_from(env, saved_state, object_id: str, edge_idx: int, depth: int):
    """Restore, push, return (post-state, displacement in metres)."""
    env.set_full_state(saved_state)
    before = _object_xy(env, object_id)
    env.step(_action(object_id, edge_idx, depth))
    after = _object_xy(env, object_id)
    return env.get_full_state(), math.hypot(after[0] - before[0], after[1] - before[1])


def _assert_same_qpos(first, second, label: str) -> None:
    """Bit-exact, not approximate. Replay drift has no tolerable magnitude."""
    assert list(first.qpos) == list(second.qpos), (
        f"{label}: qpos differs; max delta "
        f"{max(abs(a - b) for a, b in zip(first.qpos, second.qpos)):.3e}"
    )


def _assert_identical(first, second, label: str) -> None:
    _assert_same_qpos(first, second, label)
    assert list(first.qvel) == list(second.qvel), (
        f"{label}: qvel differs; max delta "
        f"{max(abs(a - b) for a, b in zip(first.qvel, second.qvel)):.3e}"
    )


CASE_IDS = [f"{scene}-e{edge}-d{depth}" for scene, _, edge, depth in PUSH_CASES]


# ─── Tests ──────────────────────────────────────────────────────────────


@pytest.mark.parametrize("scene,object_id,edge_idx,depth", PUSH_CASES, ids=CASE_IDS)
def test_restore_replays_qpos_exactly_and_zeroes_qvel(scene, object_id, edge_idx, depth, tmp_path):
    """save -> push -> restore -> read back. Isolates restore from replay.

    Restore is not a full round trip and is not meant to be: rl_env.cpp:382
    writes zeros into qvel on every set_full_state, so a restored scene always
    resumes from rest no matter what the saved state was carrying. That is what
    makes replay reproducible at all, since two runs from one saved state start
    with identical velocities by construction rather than by luck.

    When this fails, the replay tests below are failing for a reason that has
    nothing to do with the physics step.
    """
    env = _make_env(scene, tmp_path)
    _place_robot_at_edge_point(env, object_id, edge_idx)
    saved = env.get_full_state()

    env.step(_action(object_id, edge_idx, depth))
    env.set_full_state(saved)
    restored = env.get_full_state()

    _assert_same_qpos(saved, restored, "restore")
    assert list(restored.qvel) == [0.0] * len(restored.qvel), (
        "restore left non-zero qvel; rl_env.cpp:382 is supposed to zero it, and "
        "replay determinism rests on that"
    )


@pytest.mark.parametrize("scene,object_id,edge_idx,depth", PUSH_CASES, ids=CASE_IDS)
def test_the_same_push_from_a_restored_state_is_bit_exact(scene, object_id, edge_idx, depth, tmp_path):
    env = _make_env(scene, tmp_path)
    _place_robot_at_edge_point(env, object_id, edge_idx)
    saved = env.get_full_state()

    first, displacement = _push_from(env, saved, object_id, edge_idx, depth)
    assert displacement >= MIN_DISPLACEMENT_M, (
        f"push moved the object {displacement * CM_PER_M:.2f} cm; this case no "
        f"longer exercises a real push, so replay agreement proves nothing"
    )

    second, _ = _push_from(env, saved, object_id, edge_idx, depth)

    _assert_identical(first, second, "replay")


@pytest.mark.parametrize("scene,object_id,edge_idx,depth", PUSH_CASES, ids=CASE_IDS)
def test_an_unrelated_push_between_replays_changes_nothing(scene, object_id, edge_idx, depth, tmp_path):
    """Dirty the simulator between the two replays, then demand the same answer.

    Back-to-back replays can agree while restore still leaks, if both runs
    inherit the same residue. Driving a different push in between gives the
    leak something distinguishable to carry, which is how the ``ctrl`` /
    ``qacc_warmstart`` bug would show itself.
    """
    env = _make_env(scene, tmp_path)
    _place_robot_at_edge_point(env, object_id, edge_idx)
    saved = env.get_full_state()

    first, displacement = _push_from(env, saved, object_id, edge_idx, depth)
    assert displacement >= MIN_DISPLACEMENT_M, "case went vacuous, see sibling test"

    env.set_full_state(saved)
    env.step(_action(object_id, INTERLEAVED_EDGE, INTERLEAVED_DEPTH))

    second, _ = _push_from(env, saved, object_id, edge_idx, depth)

    _assert_identical(first, second, "replay after an interleaved push")
