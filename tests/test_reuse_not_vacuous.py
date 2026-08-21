"""A reused chain has to prove something.

Reuse grades the chain by asking whether the boundary's frozen points are
reachable after it runs. If the previous physical push already opened the
boundary, that is true before the chain runs at all, so any stale chain
verifies and gets executed. The robot then performs a push it no longer needs,
against a boundary that is already done.

The goal-based path never had this problem. plan() checks goal reachability
first and returns a navigate subgoal, so reuse is never reached. The boundary
path had no equivalent check.
"""

from types import SimpleNamespace

import pytest

from robot_control.core.types import Observation, PushSubgoal
from robot_control.planner import namo_bridge as bridge_mod
from robot_control.planner.namo_bridge import NAMOPlanBridge

POINTS = [(0.30, 0.40), (0.31, 0.40), (0.32, 0.41), (0.33, 0.42), (0.34, 0.43)]
GOAL_CM = (40.0, 40.0)
CHAIN = [PushSubgoal("obj_4", 17, 1)]


class _Env:
    """Reports the same reachability before and after any push."""

    def __init__(self, reachable_count):
        self.reachable_count = reachable_count
        self.steps = 0

    def set_robot_goal(self, *_a):
        return None

    def is_robot_goal_reachable(self):
        return False

    def count_reachable_points(self, _points):
        return self.reachable_count, 0

    def step(self, _action):
        # done=True means the push executed; verify_chain treats False as a
        # failed step and stops there.
        self.steps += 1
        return SimpleNamespace(info={}, done=True, reward=0.0)


def _obs():
    return Observation(robot_x=0.0, robot_y=0.0, robot_theta=0.0, objects={}, timestamp=0.0)


@pytest.fixture
def bridge_with_env(monkeypatch, tmp_path):
    def _make(reachable_count):
        bridge = NAMOPlanBridge(namo_config_path="unused.yaml", robot_model="car")
        env = _Env(reachable_count)
        xml = tmp_path / "scene.xml"
        xml.write_text("<mujoco/>")
        monkeypatch.setattr(bridge, "_generate_xml", lambda o, g: "<mujoco/>")
        monkeypatch.setattr(bridge, "_write_xml", lambda c: str(xml))
        monkeypatch.setattr(bridge, "_debug_xml_path", str(xml))
        monkeypatch.setattr(bridge, "_build_rl_env_for_scene", lambda *a, **k: env)
        monkeypatch.setattr(bridge, "_resolve_sim_object_id", lambda rid: "obstacle_1_movable")
        monkeypatch.setattr(
            bridge_mod, "load_canonical_namo_rl",
            lambda _p: (SimpleNamespace(Action=lambda: SimpleNamespace()), None, None),
        )
        return bridge, env
    return _make


def test_an_already_open_boundary_does_not_verify_a_stale_chain(bridge_with_env):
    """The bug: everything is reachable before the chain, so it passed."""
    bridge, env = bridge_with_env(reachable_count=len(POINTS))

    result = bridge.verify_chain(
        observation=_obs(), robot_goal_cm=GOAL_CM, chain=CHAIN,
        target_points=POINTS, min_reachable=1,
    )

    assert result.success is False
    assert result.failure_reason == "target_already_open_before_chain"


def test_a_shut_boundary_still_verifies_normally(bridge_with_env):
    bridge, env = bridge_with_env(reachable_count=0)

    result = bridge.verify_chain(
        observation=_obs(), robot_goal_cm=GOAL_CM, chain=CHAIN,
        target_points=POINTS, min_reachable=1,
    )

    assert result.failure_reason == "target_not_open_after_chain"
    assert env.steps == len(CHAIN)


def test_the_chain_is_not_executed_when_the_boundary_is_already_open(bridge_with_env):
    """No point simulating pushes against a boundary that is already done."""
    bridge, env = bridge_with_env(reachable_count=len(POINTS))

    bridge.verify_chain(
        observation=_obs(), robot_goal_cm=GOAL_CM, chain=CHAIN,
        target_points=POINTS, min_reachable=1,
    )

    assert env.steps == 0


def test_without_target_points_nothing_changes(bridge_with_env):
    bridge, env = bridge_with_env(reachable_count=0)

    result = bridge.verify_chain(
        observation=_obs(), robot_goal_cm=GOAL_CM, chain=CHAIN,
    )

    assert result.failure_reason == "goal_not_reachable_after_chain"
    assert env.steps == len(CHAIN)
