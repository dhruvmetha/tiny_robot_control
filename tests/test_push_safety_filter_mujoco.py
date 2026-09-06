"""End to end: the real PushController drives the car in MuJoCo, with and
without the safety filter.

The scene is built here so the outcome is deterministic. A 4 x 4 cm light
block sits at (20, 30) cm and is pushed along +X. A brick lies parallel to
the push line with its inner face 2.2 cm above the line and its near end
7 cm ahead of the robot's nose. The block passes under the brick; the 7 cm
car does not, its front corners reach the brick's end. With the filter on
the push must stop short of the brick and name it. With the filter off the
same loop must produce a car-brick contact, which is what proves the scene
discriminates.

The loop is the C++ replay's wiring in Python: a 30 Hz tick, wheel command
= speed * kCarWheelMaxSpeedMs / wheel_radius (namo_push_controller.cpp:26
and little_car.xml), then physics substeps to fill the tick. The push speed
is 0.06, which is 0.2 cm per tick, the fastest rate seen on the table.
"""

from __future__ import annotations

import contextlib
import io
import math
import re
from types import SimpleNamespace

import pytest

mujoco = pytest.importorskip("mujoco")

from robot_control.controller.config import PushConfig
from robot_control.controller.edge_points import get_edge_point
from robot_control.controller.push import PushController, PushState
from robot_control.controller.safety_filter import SafetyFilter
from robot_control.core.types import ObjectPose, Observation, PushSubgoal, WorkspaceConfig

CAR_SCENE = "1push/1hop/env1/env.xml"   # borrowed only for its little_car include
TABLE_W_CM, TABLE_H_CM = 49.0, 77.5     # env1's wall_1..4 frame
TICK_HZ = 30.0
K_CAR_WHEEL_MAX_SPEED_MS = 1.0          # namo_push_controller.cpp:26
WHEEL_RADIUS_M = 0.015                  # little_car.xml wheel cylinder
PUSH_SPEED = 0.06                       # 0.06 m/s = 0.2 cm per tick
MARGIN_CM = 1.5
ROBOT_SIDE_CM = 7.0
STANDOFF_CM = 3.5 + 1.0                 # robot radius + edge_offset_margin_cm
POINTS_PER_FACE = 15
PUSH_STEPS = 4                          # x 30 ticks = 120 ticks, 24 cm of travel budget
BLOCK_X_M, BLOCK_Y_M, BLOCK_HALF_M = 0.20, 0.30, 0.02
BRICK_NEAR_END_M, BRICK_HALF_DEPTH_M, BRICK_HALF_WIDTH_M = 0.24, 0.06, 0.0275
BRICK_GAP_ABOVE_LINE_M = 0.022          # block half 0.02 passes; car half 0.035 clips
BRICK_NAME = "wall_5"

SCENE_XML = """<mujoco model="safety_filter_e2e">
  <option cone="elliptic" integrator="implicitfast" iterations="100" timestep="0.002"/>
  <default><geom density="1"/></default>
  {include}
  <worldbody>
    <light dir="0 0 -1" directional="true" pos="0 0 1.5"/>
    <geom condim="4" friction="0.5 0.005 0.001" name="floor" size="0 0 0.05" type="plane"/>
    <body name="obstacle_1_movable" pos="{bx} {by} {bz}">
      <geom name="obstacle_1_movable" condim="4" friction="1 0.005 0.0001" rgba="1 1 0 1"
            size="{bh} {bh} {bh}" type="box" mass="0.02"/>
      <joint type="free"/>
    </body>
    <body name="{brick}" pos="{wx} {wy} 0.05">
      <geom name="{brick}" condim="4" friction="1 0.005 0.0001" rgba="0.8 0.8 0.8 1"
            size="{wd} {ww} 0.05" type="box"/>
    </body>
  </worldbody>
</mujoco>
"""

WS = WorkspaceConfig(width=TABLE_W_CM, height=TABLE_H_CM, car_width=ROBOT_SIDE_CM,
                     car_height=ROBOT_SIDE_CM, offset_w=3.5, offset_h=3.5)


def _push_config() -> PushConfig:
    return PushConfig(
        max_speed=PUSH_SPEED, edge_offset_margin_cm=1.0, wheel_deadband=0.05, lookahead_ratio=1.0,
        push_steps=30, dynamic_direction=False, approach_skip_distance=1.0, approach_skip_angle=30.0,
        points_per_face=POINTS_PER_FACE, advance_speed=PUSH_SPEED, advance_steps=0,
        retreat_speed=PUSH_SPEED, retreat_steps=10, retreat_min_dist=5.0, retreat_max_dist=15.0,
        retreat_tolerance=2.0, min_push_displacement_cm=2.0, min_push_rotation_deg=5.0,
        show_wavefront=False,
    )


# ---------------------------------------------------------------------------
# Scene and MuJoCo plumbing
# ---------------------------------------------------------------------------

def _scene_xml(scene_loader) -> str:
    portable = scene_loader(CAR_SCENE).read_text()
    include = re.search(r'<include\s+file="[^"]+"\s*/>', portable).group(0)
    return SCENE_XML.format(
        include=include,
        bx=BLOCK_X_M, by=BLOCK_Y_M, bz=BLOCK_HALF_M, bh=BLOCK_HALF_M,
        brick=BRICK_NAME,
        wx=BRICK_NEAR_END_M + BRICK_HALF_DEPTH_M,
        wy=BLOCK_Y_M + BRICK_GAP_ABOVE_LINE_M + BRICK_HALF_WIDTH_M,
        wd=BRICK_HALF_DEPTH_M, ww=BRICK_HALF_WIDTH_M,
    )


def _body_pose_cm_deg(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    xpos = data.xpos[body_id]
    q = data.xquat[body_id]
    theta = math.degrees(2.0 * math.atan2(float(q[3]), float(q[0])))
    return float(xpos[0] * 100.0), float(xpos[1] * 100.0), ((theta + 180.0) % 360.0) - 180.0


def _geom_size_cm(model, geom_name):
    """(depth, width, height) in cm from a box geom's half extents."""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    s = model.geom_size[gid]
    return float(s[0] * 200.0), float(s[1] * 200.0), float(s[2] * 200.0)


def _observe(model, data, tick: int) -> Observation:
    rx, ry, rth = _body_pose_cm_deg(model, data, "car")
    objects = {}
    for name, geom, is_static in (("obj_1", "obstacle_1_movable", False), (BRICK_NAME, BRICK_NAME, True)):
        x, y, th = _body_pose_cm_deg(model, data, geom)
        depth, width, height = _geom_size_cm(model, geom)
        objects[name] = ObjectPose(x=x, y=y, theta=th, width=width, depth=depth, height=height, is_static=is_static)
    return Observation(robot_x=rx, robot_y=ry, robot_theta=rth, objects=objects, timestamp=tick / TICK_HZ)


def _place_car(model, data, x_cm: float, y_cm: float, theta_deg: float) -> None:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "car_freejoint")
    adr = int(model.jnt_qposadr[jid])
    half = math.radians(theta_deg) / 2.0
    data.qpos[adr:adr + 7] = [x_cm / 100.0, y_cm / 100.0, 0.01, math.cos(half), 0.0, 0.0, math.sin(half)]
    dof = int(model.jnt_dofadr[jid])
    data.qvel[dof:dof + 6] = 0.0
    mujoco.mj_forward(model, data)


def _car_touches(model, data, geom_name: str) -> bool:
    car_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "car")
    wall_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    for i in range(data.ncon):
        c = data.contact[i]
        if wall_geom not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == wall_geom else c.geom1
        if int(model.body_rootid[int(model.geom_bodyid[other])]) == car_body:
            return True
    return False


def _run(model, data, controller: PushController, max_ticks: int):
    """Drive the controller against physics. Returns what happened, per tick."""
    mujoco.mj_forward(model, data)  # body poses are zero until the first forward pass
    obs0 = _observe(model, data, 0)
    obj = obs0.objects["obj_1"]
    facing_plus_x = [
        i for i in range(4 * POINTS_PER_FACE)
        if abs(get_edge_point(obj, i, STANDOFF_CM, POINTS_PER_FACE).approach_theta) < 1.0
    ]
    edge_idx = min(facing_plus_x, key=lambda i: abs(get_edge_point(obj, i, STANDOFF_CM, POINTS_PER_FACE).position[1] - obj.y))
    ep = get_edge_point(obj, edge_idx, STANDOFF_CM, POINTS_PER_FACE)
    _place_car(model, data, ep.position[0], ep.position[1], ep.approach_theta)

    left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_drive")
    right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_drive")
    lo, hi = model.actuator_ctrlrange[left]
    n_sub = int(round((1.0 / TICK_HZ) / model.opt.timestep))

    sub = PushSubgoal(object_id="obj_1", edge_idx=edge_idx, push_steps=PUSH_STEPS)
    controller.reset()
    out = SimpleNamespace(touched_at=None, retreat_at=None, ticks=0, robot_x_at_retreat=None, nose_x_start=ep.position[0] + ROBOT_SIDE_CM / 2)
    for t in range(max_ticks):
        obs = _observe(model, data, t)
        state_before = controller._state
        with contextlib.redirect_stdout(io.StringIO()):
            action = controller.step(obs, sub)
        if out.retreat_at is None and controller._state == PushState.RETREATING:
            out.retreat_at = t
            out.robot_x_at_retreat = obs.robot_x
        data.ctrl[left] = min(hi, max(lo, action.left_speed * K_CAR_WHEEL_MAX_SPEED_MS / WHEEL_RADIUS_M))
        data.ctrl[right] = min(hi, max(lo, action.right_speed * K_CAR_WHEEL_MAX_SPEED_MS / WHEEL_RADIUS_M))
        for _ in range(n_sub):
            mujoco.mj_step(model, data)
            if out.touched_at is None and _car_touches(model, data, BRICK_NAME):
                out.touched_at = t
        out.ticks = t + 1
        if controller.is_done(obs, sub):
            break
    out.summary = controller.get_last_push_summary(_observe(model, data, out.ticks))
    return out


@pytest.fixture
def scene(scene_loader):
    model = mujoco.MjModel.from_xml_string(_scene_xml(scene_loader))
    return model, mujoco.MjData(model)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

PUSH_TICK_BUDGET = PUSH_STEPS * 30


def test_the_filter_stops_the_car_short_of_the_brick(scene):
    model, data = scene
    controller = PushController(WS, nav_controller=None, push_config=_push_config(),
                                safety_filter=SafetyFilter(WS, margin_cm=MARGIN_CM))
    out = _run(model, data, controller, max_ticks=PUSH_TICK_BUDGET + 100)

    assert out.touched_at is None, f"car touched the brick at tick {out.touched_at}"
    assert out.retreat_at is not None and out.retreat_at < PUSH_TICK_BUDGET
    assert controller.did_fail() is True
    assert out.summary["abort_reason"] == f"robot_static_clearance:{BRICK_NAME}"
    nose_gap_cm = BRICK_NEAR_END_M * 100.0 - (out.robot_x_at_retreat + ROBOT_SIDE_CM / 2)
    # The lookup only ever reads under the true gap, so the stop is at or
    # before the margin, less one tick of motion at 0.2 cm.
    assert nose_gap_cm >= MARGIN_CM - 0.2, nose_gap_cm
    assert nose_gap_cm < BRICK_NEAR_END_M * 100.0 - out.nose_x_start, "the car never moved"


def test_without_the_filter_the_same_push_hits_the_brick(scene):
    model, data = scene
    controller = PushController(WS, nav_controller=None, push_config=_push_config())
    out = _run(model, data, controller, max_ticks=PUSH_TICK_BUDGET + 100)

    assert out.touched_at is not None
    assert out.touched_at < PUSH_TICK_BUDGET
    assert out.summary["safety_filter"] is False
    assert out.summary["abort_reason"] is None
