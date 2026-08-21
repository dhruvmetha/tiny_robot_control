"""One boundary, one push, graded against the points frozen before it ran.

Item 1 of the section 10.3 simulator tests in
``MODEL_GUIDED_RO_INTEGRATION.local.md``. The claim is small and the whole
integration rests on it: the planner names a boundary and a blocker, the search
returns a chain for it, and running that chain makes the target points
reachable.

Graded the way the real loop grades: the points come from
``select_boundary_from_xml`` and are held fixed across the push, never
resampled after. Region labels renumber whenever free space changes, so a test
that re-derived the target afterwards could pass while measuring a different
region than the one it opened. That is pitfall two in the doc.

What this does NOT pin is which push the search picks. Yesterday a physics
change moved a recorded chain from two pushes to one, and re-recording pinned
answers is a cost worth paying only where the answer is the point.
``test_best_first_sandbox_contract`` is that place. Here the point is that the
boundary opens.

To verify:
  cd namo_cpp && source env.robotlearning.sh && cd ../robot_control
  pytest tests/test_region_opening_1push_1hop.py -v
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

SCENE = "1push/1hop/env1/env.xml"

# The goal baked into every real_test_envs scene, in metres.
GOAL_M = (0.37, 0.67, 0.0)

# Open floor below the divider walls. The scenes spawn the car at the origin,
# wedged in the wall corner, where nothing is reachable and no boundary can be
# selected; the bridge overrides the pose from the live observation for the same
# reason. Without this every call below reports found=False and explains
# nothing.
START_POSE_M = (0.25, 0.10, 0.0)

# The canonical opening bar from section 3.5 of the doc: a target region counts
# as open once this fraction of its sampled points is reachable, computed as
# max(1, ceil(fraction * len(samples))) so it still works with fewer points.
OPEN_FRACTION = 0.20

# What select_boundary_from_xml reports on this scene. Pinned so a changed scene
# or sampler fails here, loudly, rather than quietly grading the run against a
# different boundary.
EXPECTED_BLOCKER = "obstacle_1_movable"
EXPECTED_SAMPLE_COUNT = 100


try:
    namo_rl, _, _ = load_canonical_namo_rl(Path(__file__))
    from namo.runtime_profile import CANONICAL_CONFIG
    from namo.services import NAMOPlanningService
except Exception as exc:  # pragma: no cover - environment-dependent
    pytest.skip(
        f"needs the canonical namo_cpp build and services: {exc}",
        allow_module_level=True,
    )

NAMO_CPP = resolve_namo_cpp_dir(Path(__file__))
NAMO_CONFIG = NAMO_CPP / CANONICAL_CONFIG


# ─── Helpers ────────────────────────────────────────────────────────────


def _opening_bar(sample_count: int) -> int:
    return max(1, math.ceil(OPEN_FRACTION * sample_count))


@pytest.fixture(scope="module")
def service():
    return NAMOPlanningService(
        config_path=str(NAMO_CONFIG),
        primitive_data_dir=str(NAMO_CPP / "data"),
    )


@pytest.fixture(scope="module")
def scene_path(tmp_path_factory):
    """One portable copy for the module. The scenes carry an absolute include."""
    from robot_control.utils.scene_xml import portable_scene

    source = Path(__file__).resolve().parents[1] / "real_test_envs" / SCENE
    if not source.is_file():
        pytest.skip(f"missing captured scene: {source}")
    return portable_scene(source, tmp_path_factory.mktemp("scene"))


@pytest.fixture(scope="module")
def selection(service, scene_path):
    """The boundary and the points every assertion below is graded against."""
    return service.select_boundary_from_xml(
        str(scene_path), GOAL_M, starting_robot_pose=START_POSE_M
    )


@pytest.fixture(scope="module")
def solution(service, scene_path, selection):
    """The chain the search returns for that boundary. Runs once; it is slow."""
    return service.solve_boundary_from_xml(
        str(scene_path),
        GOAL_M,
        target_points=selection.target_points,
        blocking_objects=selection.blocking_objects,
        starting_robot_pose=START_POSE_M,
    )


def _fresh_env(scene_path):
    env = namo_rl.RLEnvironment(str(scene_path), str(NAMO_CONFIG), False)
    env.reset()
    env.set_robot_pose(*START_POSE_M)
    return env


def _reachable_count(env, points) -> int:
    """How many of the frozen points the robot can reach right now.

    count_reachable_points returns (count, index of the first reachable point).
    The second value is -1 when none are, which is not a status code; only the
    count is meaningful here.
    """
    count, _first_reachable = env.count_reachable_points([list(p) for p in points])
    return int(count)


# ─── Tests ──────────────────────────────────────────────────────────────


def test_the_scene_presents_the_boundary_this_run_is_graded_against(selection):
    """A changed scene or sampler invalidates everything below, loudly."""
    assert selection.found
    assert selection.blocking_objects == [EXPECTED_BLOCKER]
    assert len(selection.target_points) == EXPECTED_SAMPLE_COUNT


def test_the_search_returns_a_chain_for_that_boundary(solution):
    assert solution.success
    assert not solution.already_open, (
        "the boundary was open before any push, so opening it proves nothing"
    )
    assert solution.actions, "success with no actions is not something to execute"


def test_the_boundary_is_shut_before_the_push(scene_path, selection):
    """Half of the evidence. Without it, an open-after is not attributable."""
    env = _fresh_env(scene_path)

    assert _reachable_count(env, selection.target_points) < _opening_bar(
        len(selection.target_points)
    )


def test_running_the_chain_opens_the_frozen_target_points(
    scene_path, selection, solution
):
    env = _fresh_env(scene_path)
    before = _reachable_count(env, selection.target_points)

    for action in solution.actions:
        step = namo_rl.Action()
        step.object_id = action.object_id
        step.edge_idx = action.edge_idx
        step.depth = action.depth
        result = env.step(step)
        assert not dict(result.info).get("failure_reason"), (
            f"push ({action.object_id}, {action.edge_idx}, {action.depth}) "
            f"failed: {dict(result.info)}"
        )

    after = _reachable_count(env, selection.target_points)
    bar = _opening_bar(len(selection.target_points))

    assert after >= bar, (
        f"{after} of {len(selection.target_points)} target points reachable "
        f"after the chain; the boundary needs {bar} to count as open"
    )
    assert after > before, "the push has to be what opened it"
