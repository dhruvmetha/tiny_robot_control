"""The camera keeps streaming objects when the robot marker is out of frame.

On 2026-09-07 the build checker showed every brick as MISSING while the four
bricks sat in view, because the ArUco observer only built an Observation when
it saw the robot marker. The observer now publishes every frame with the robot
pose set to None when unseen; WorldState carries the last robot pose forward so
controllers never see None; the checker lists the bricks and marks only the
robot row missing.
"""

from __future__ import annotations

from importlib import import_module
from pathlib import Path
import sys

import numpy as np
from pubsub import pub

from robot_control.camera.observer import ArucoObserver, ObserverConfig
from robot_control.core.serialization import bytes_to_obs, obs_to_bytes
from robot_control.core.topics import Topics
from robot_control.core.types import ObjectPose, Observation
from robot_control.core.world_state import WorldState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
check_build = import_module("check_build")

BRICKS = {
    "wall_10": ObjectPose(x=9.9, y=24.3, theta=2.0, is_static=True),
    "obj_4": ObjectPose(x=23.6, y=38.5, theta=16.5),
}


def _obs(robot, objects=BRICKS, timestamp=1.0):
    x, y, theta = robot if robot is not None else (None, None, None)
    return Observation(
        robot_x=x, robot_y=y, robot_theta=theta, objects=dict(objects), timestamp=timestamp
    )


def test_observation_reports_whether_the_robot_was_seen():
    assert _obs((1.0, 2.0, 3.0)).has_robot
    assert not _obs(None).has_robot


def test_robotless_observation_round_trips_over_the_wire():
    back = bytes_to_obs(obs_to_bytes(_obs(None)))
    assert not back.has_robot
    assert set(back.objects) == set(BRICKS)
    assert back.objects["obj_4"].x == 23.6


def test_world_state_carries_the_last_robot_pose_over_a_robotless_frame():
    world = WorldState()
    try:
        world._on_sensor(_obs((7.5, 11.2, 90.0), objects={}, timestamp=1.0))
        world._on_sensor(_obs(None, objects=BRICKS, timestamp=2.0))
        merged = world.get()
        assert merged.has_robot
        assert (merged.robot_x, merged.robot_y, merged.robot_theta) == (7.5, 11.2, 90.0)
        assert set(merged.objects) == set(BRICKS)
        assert merged.timestamp == 2.0
    finally:
        world.unsubscribe()


def test_world_state_drops_robotless_frames_until_the_robot_is_seen_once():
    world = WorldState()
    try:
        world._on_sensor(_obs(None, timestamp=1.0))
        assert world.get() is None
        world._on_sensor(_obs((1.0, 1.0, 0.0), timestamp=2.0))
        assert world.get().timestamp == 2.0
    finally:
        world.unsubscribe()


def test_observer_publishes_objects_when_the_robot_marker_is_not_seen(monkeypatch):
    observer = ArucoObserver(ObserverConfig(draw_detections=False))
    observer._running = True
    observer._warmup_done = True
    observer._ws_fixed = True
    observer._rvec_ws = np.zeros(3)
    monkeypatch.setattr(observer, "_detect_robot", lambda gray, vis: None)
    monkeypatch.setattr(observer, "_detect_objects", lambda gray, vis: (dict(BRICKS), None))

    published = []

    def capture(obs):  # pubsub holds listeners weakly; a lambda would be collected
        published.append(obs)

    pub.subscribe(capture, Topics.SENSOR_VISION)
    try:
        observer._on_frame(np.zeros((4, 4, 3), dtype=np.uint8), timestamp=5.0)
    finally:
        pub.unsubscribe(capture, Topics.SENSOR_VISION)

    assert len(published) == 1
    obs = published[0]
    assert not obs.has_robot
    assert set(obs.objects) == set(BRICKS)
    assert observer.get() is obs


def test_checker_lists_bricks_and_omits_the_robot_when_it_is_unseen():
    class FakeNode:
        def get(self):
            return _obs(None)

    source = check_build.ServiceCameraSource.__new__(check_build.ServiceCameraSource)
    source._node = FakeNode()

    seen = source.get()

    assert set(seen) == set(BRICKS)
    assert check_build.ROBOT_KEY not in seen
    assert "MISSING" in check_build.line_for_robot(
        {"robot_start_x_cm": "7.5", "robot_start_y_cm": "11.2", "robot_start_bearing_deg": "0"},
        seen.get(check_build.ROBOT_KEY),
    )
