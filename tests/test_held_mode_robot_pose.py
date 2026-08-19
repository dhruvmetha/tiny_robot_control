"""Held-boundary planning must start the simulated robot where the real one is.

The car's MuJoCo model has a fixed freejoint spawn at (0, 0, 0.01). Only
set_robot_pose() moves it, and the planning service only calls that when the
caller passes starting_robot_pose. plan(), verify_chain() and
analyze_reachability() all pass it.

If a held-boundary call omits it, the region graph is computed around a robot
sitting at the origin rather than where the camera says it is. The robot's
region is then wrong, so the boundary chosen as "next" is wrong too, and pushes
are graded from a pose that may be inside an obstacle.
"""

import math
from types import SimpleNamespace

import pytest

from robot_control.core.types import Observation
from robot_control.planner.namo_bridge import NAMOPlanBridge

ROBOT_CM = (37.0, 67.0)
ROBOT_THETA_DEG = 90.0
GOAL_CM = (40.0, 40.0)


class _RecordingService:
    """Captures what the bridge hands the planning service."""

    def __init__(self):
        self.select_kwargs = None
        self.solve_kwargs = None

    def select_boundary_from_xml(self, xml_path, robot_goal, **kwargs):
        self.select_kwargs = kwargs
        return SimpleNamespace(
            found=False, goal_already_reachable=False, failure_reason="none",
            target_points=[], blocking_objects=[], region_path=[],
        )

    def solve_boundary_from_xml(self, xml_path, robot_goal, **kwargs):
        self.solve_kwargs = kwargs
        return SimpleNamespace(
            success=False, already_open=False, boundary_exhausted=False,
            actions=[], failure_reason="none", resolved_target="",
            search_time_ms=1.0, simulations_used=0, target_summary=None,
        )


def _obs():
    return Observation(
        robot_x=ROBOT_CM[0], robot_y=ROBOT_CM[1], robot_theta=ROBOT_THETA_DEG,
        objects={}, timestamp=0.0,
    )


@pytest.fixture
def bridge_and_service(monkeypatch, tmp_path):
    bridge = NAMOPlanBridge(namo_config_path="unused.yaml", robot_model="car")
    service = _RecordingService()
    monkeypatch.setattr(bridge, "_get_planning_service", lambda: service)
    monkeypatch.setattr(bridge, "_generate_xml", lambda obs, goal: "<mujoco/>")
    xml = tmp_path / "scene.xml"
    xml.write_text("<mujoco/>")
    monkeypatch.setattr(bridge, "_write_xml", lambda content: str(xml))
    monkeypatch.setattr(bridge, "_debug_xml_path", str(xml))  # keep the file
    return bridge, service


def _expected_pose(bridge):
    x_m, y_m = bridge._cm_to_sim(*ROBOT_CM)
    return (x_m, y_m, math.radians(ROBOT_THETA_DEG))


def test_selection_starts_the_robot_at_the_observed_pose(bridge_and_service):
    bridge, service = bridge_and_service

    bridge.select_boundary(_obs(), GOAL_CM)

    assert service.select_kwargs["starting_robot_pose"] == _expected_pose(bridge)


def test_solving_starts_the_robot_at_the_observed_pose(bridge_and_service):
    bridge, service = bridge_and_service
    target = SimpleNamespace(
        as_solve_kwargs=lambda: {"target_points": [(0.3, 0.4)], "blocking_objects": []},
        failed_pushes=(),
    )

    bridge.solve_boundary(_obs(), GOAL_CM, target)

    assert service.solve_kwargs["starting_robot_pose"] == _expected_pose(bridge)


def test_the_pose_is_not_the_models_default_spawn(bridge_and_service):
    """The bug: without this the car is left at the XML freejoint origin."""
    bridge, service = bridge_and_service

    bridge.select_boundary(_obs(), GOAL_CM)

    assert service.select_kwargs["starting_robot_pose"][:2] != (0.0, 0.0)


def test_a_sphere_robot_still_passes_none(bridge_and_service, monkeypatch):
    """Only the car has a fixed spawn; the sphere geom carries its own pose."""
    bridge, service = bridge_and_service
    monkeypatch.setattr(bridge, "_robot_model", "sphere")

    bridge.select_boundary(_obs(), GOAL_CM)

    assert service.select_kwargs["starting_robot_pose"] is None
