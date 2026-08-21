"""Two-doorway scenes: open the first boundary, then re-ask from the world it made.

Items 3 and 4 of the section 10.3 simulator tests in
``MODEL_GUIDED_RO_INTEGRATION.local.md``. One hop is covered by
``test_region_opening_1push_1hop``; what these add is the outer loop. After the
first boundary opens the region graph has changed, so the planner has to be
asked again against a scene built from the new state rather than handed the
leftover tail of the old plan.

Everything runs the deployed configuration: best-first ordered by HY5U, chain
depth 2. Plain ``region_bfs`` is not what the robot runs and cannot finish these
scenes in reasonable time; it burned 4,768 candidates in 40 minutes here without
returning.

Grading uses the state the solver verified, not a replay of its chain. Those are
not the same thing, and the difference is recorded rather than assumed: on
``2push/2hop/env1`` the verified state reaches 22 of 100 target points while
replaying the same chain from the same start reaches 18, either side of a bar of
20. The robot ends 4 mm apart. The search passes through ``set_full_state``
between pushes, which zeroes ``qvel``; a straight replay carries residual
velocity from one push into the next. Until that is settled, a test that graded
a replay would be pinning the divergence.

Slow by test standards, about 30 s per scene, nearly all of it the search. The
fixtures are module-scoped so each scene solves once.

To verify:
  cd namo_cpp && source env.robotlearning.sh && cd ../robot_control
  pytest tests/test_region_opening_two_hop.py -v
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from robot_control.planner.namo_binding_loader import (
    load_canonical_namo_rl,
    resolve_namo_cpp_dir,
)

# ─── Named constants ────────────────────────────────────────────────────

# Both two-hop scenes. 1push opens its first boundary with two pushes on one
# object; 2push does the same on a different object and a non-goal first target.
SCENES = ("1push/2hop/env1", "2push/2hop/env1")

GOAL_M = (0.37, 0.67, 0.0)
# Open floor below the divider walls. Without it every call reports found=False,
# because the scenes spawn the car at the origin wedged in the wall corner.
START_POSE_M = (0.25, 0.10, 0.0)

# Canonical opening bar from section 3.5 of the doc.
OPEN_FRACTION = 0.20

# The deployed ranker. A path swap away from BNG; verified identical in
# architecture, so the loader needs no special casing.
SCORER_CKPT = "/home/dhruv/projects_dhruv/namo/ranking/models/HY5U_s2.ckpt"
CHAIN_DEPTH = 2

# What select_boundary_from_xml reports on each scene, pinned so a changed scene
# or sampler fails here rather than quietly grading a different boundary.
# Measured 2026-08-21.
EXPECTED = {
    "1push/2hop/env1": {"target": "goal", "blocker": "obstacle_1_movable"},
    "2push/2hop/env1": {"target": "region_3", "blocker": "obstacle_2_movable"},
}
EXPECTED_SAMPLE_COUNT = 100


try:
    namo_rl, _, _ = load_canonical_namo_rl(Path(__file__))
    from namo.core.state_to_xml import write_state_xml
    from namo.runtime_profile import CANONICAL_CONFIG
    from namo.services import NAMOPlanningService
except Exception as exc:  # pragma: no cover - environment-dependent
    pytest.skip(f"needs the canonical namo_cpp build: {exc}", allow_module_level=True)

NAMO_CPP = resolve_namo_cpp_dir(Path(__file__))
NAMO_CONFIG = NAMO_CPP / CANONICAL_CONFIG

if not Path(SCORER_CKPT).is_file():  # pragma: no cover - environment-dependent
    pytest.skip(f"missing ranker checkpoint: {SCORER_CKPT}", allow_module_level=True)


# ─── Helpers ────────────────────────────────────────────────────────────


def _opening_bar(sample_count: int) -> int:
    return max(1, math.ceil(OPEN_FRACTION * sample_count))


@pytest.fixture(scope="module")
def service():
    return NAMOPlanningService(
        config_path=str(NAMO_CONFIG), primitive_data_dir=str(NAMO_CPP / "data")
    )


@pytest.fixture(scope="module")
def solved(service, tmp_path_factory):
    """Select and solve the first boundary of each scene. Runs once; it is slow."""
    from robot_control.utils.scene_xml import portable_scene

    out = {}
    tmp = tmp_path_factory.mktemp("scenes")
    for scene in SCENES:
        source = Path(__file__).resolve().parents[1] / "real_test_envs" / scene / "env.xml"
        if not source.is_file():
            pytest.skip(f"missing captured scene: {source}")
        # The captured scenes carry the absolute include of the box that made them.
        # A directory each, since portable_scene keeps the source filename and
        # every one of these is called env.xml.
        scene_dir = tmp / scene.replace("/", "_")
        scene_dir.mkdir(parents=True, exist_ok=True)
        xml = portable_scene(source, scene_dir)
        selection = service.select_boundary_from_xml(
            str(xml), GOAL_M, starting_robot_pose=START_POSE_M
        )
        solution = service.solve_boundary_from_xml(
            str(xml),
            GOAL_M,
            target_points=selection.target_points,
            blocking_objects=selection.blocking_objects,
            starting_robot_pose=START_POSE_M,
            max_chain_depth=CHAIN_DEPTH,
            local_search="best_first",
            best_first_prior="model",
            scorer_ckpt=SCORER_CKPT,
            ml_device="cpu",
        )
        out[scene] = (xml, selection, solution)
    return out


def _env_at(xml, state=None, robot_pose=START_POSE_M):
    env = namo_rl.RLEnvironment(str(xml), str(NAMO_CONFIG), False)
    env.reset()
    # Reachability is measured from wherever the robot stands, so a caller
    # grading a post-push scene has to say where the push left it.
    env.set_robot_pose(*robot_pose)
    if state is not None:
        restored = namo_rl.RLState()
        restored.qpos = list(state["qpos"])
        restored.qvel = list(state["qvel"])
        env.set_full_state(restored)
    return env


def _reachable(env, points) -> int:
    count, _first = env.count_reachable_points([list(p) for p in points])
    return int(count)


# ─── Tests ──────────────────────────────────────────────────────────────


@pytest.mark.parametrize("scene", SCENES)
def test_the_scene_presents_the_boundary_this_run_is_graded_against(solved, scene):
    _xml, selection, _solution = solved[scene]

    assert selection.found
    assert selection.target_label == EXPECTED[scene]["target"]
    assert selection.blocking_objects == [EXPECTED[scene]["blocker"]]
    assert len(selection.target_points) == EXPECTED_SAMPLE_COUNT


@pytest.mark.parametrize("scene", SCENES)
def test_the_first_boundary_needs_more_than_one_push(solved, scene):
    """Both scenes are two-push openings, which is why chain depth 2 is required.

    At depth 1 the same call returns no chain at all, so a regression that
    silently capped the chain would show up here rather than as a mystery
    failure later.
    """
    _xml, _selection, solution = solved[scene]

    assert solution.success, f"failed: {solution.failure_reason}"
    assert not solution.already_open
    assert len(solution.actions) == 2


@pytest.mark.parametrize("scene", SCENES)
def test_the_verified_state_clears_the_opening_bar(solved, scene):
    """Grade the state the search actually verified, from the frozen points."""
    xml, selection, solution = solved[scene]
    assert solution.resulting_state is not None, "a win has to carry its state"

    before = _reachable(_env_at(xml), selection.target_points)
    after = _reachable(_env_at(xml, solution.resulting_state), selection.target_points)
    bar = _opening_bar(len(selection.target_points))

    assert before < bar, "the boundary has to start shut or opening it proves nothing"
    assert after >= bar, (
        f"{after} of {len(selection.target_points)} reachable in the state the "
        f"solver verified; the bar is {bar}"
    )


@pytest.mark.parametrize("scene", SCENES)
def test_the_outer_loop_moves_on_after_the_first_boundary_opens(
    service, solved, scene, tmp_path
):
    """The point of two hops: re-select from the new world, do not reuse the plan.

    The post-push scene is materialized and handed back to selection. Region
    labels renumber whenever free space changes, so this compares what the
    planner is now pointing at, not the label it used before.
    """
    xml, selection, solution = solved[scene]
    env = _env_at(xml, solution.resulting_state)

    post = write_state_xml(
        env.get_observation(),
        str(xml),
        str(tmp_path / "post.xml"),
        robot_pose_set_by_caller=True,
    )
    robot_pose = env.get_observation()["robot_pose"]
    # Loads at all, which the include used to prevent, and from the pose the
    # push actually left the robot in.
    again = _env_at(post, robot_pose=tuple(robot_pose))
    next_choice = service.select_boundary_from_xml(
        post, GOAL_M, starting_robot_pose=tuple(robot_pose)
    )

    assert next_choice.found, "nothing left to open, so the run should have ended"
    moved_on = (
        next_choice.blocking_objects != selection.blocking_objects
        or _reachable(again, selection.target_points)
        >= _opening_bar(len(selection.target_points))
    )
    assert moved_on, (
        f"after opening {selection.blocking_objects}, the planner still points at "
        f"{next_choice.blocking_objects} and the first boundary is not open"
    )
