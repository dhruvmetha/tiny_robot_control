"""Backing a robot out of a spot it cannot drive out of.

`PushController` has needed this since it started pushing: after a push it must
leave without dragging the object back. `NavigationController` needs the same
move for a different reason, since a robot wedged against an obstacle has to
reach open space before anything can replan for it. One implementation, so the
two cannot drift into backing off differently.

Both functions are pure and take what they need as arguments. The two callers
read their numbers from different blocks of `config/controller.yaml`, and
neither should have to know that.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from robot_control.core.types import Action, Observation

Point = Tuple[float, float]

# Rays are sampled this far apart across the cone, and points this far apart
# along each ray. Both from the push controller's original search, which has
# run on hardware since the first trials.
CONE_ANGLE_STEP_DEG = 5.0
RAY_DISTANCE_STEP_CM = 1.0


def find_retreat_target(
    wavefront,
    robot_xy: Point,
    heading_deg: float,
    cone_half_angle_deg: float,
    min_dist_cm: float,
    max_dist_cm: float,
    allow_forward: bool = True,
) -> Tuple[Optional[Point], bool]:
    """Nearest free cell to back into, and whether it sits behind the robot.

    Searches the backward cone first because reaching it needs no rotation,
    which matters when the robot is wedged and rotating may not be possible.
    The forward cone is a fallback for callers that can steer to it.

    `allow_forward=False` restricts the search to the backward cone. Navigation
    uses that: its only way to reach a forward target is to navigate there, and
    navigating out of a failed navigation is how you get a retreat inside a
    retreat.

    Returns (None, False) when no free cell exists in the searched cones.
    """
    cone_half = math.radians(cone_half_angle_deg)
    heading = math.radians(heading_deg)
    angle_step = math.radians(CONE_ANGLE_STEP_DEG)

    def search(base_angle: float) -> Optional[Tuple[Point, float]]:
        best: Optional[Point] = None
        best_dist = float("inf")
        for offset in np.arange(-cone_half, cone_half + 0.001, angle_step):
            ray = base_angle + offset
            for dist in np.arange(min_dist_cm, max_dist_cm + 0.001, RAY_DISTANCE_STEP_CM):
                x = robot_xy[0] + dist * math.cos(ray)
                y = robot_xy[1] + dist * math.sin(ray)
                if wavefront.is_free(x / 100.0, y / 100.0):
                    if dist < best_dist:
                        best_dist, best = dist, (x, y)
                    break  # nearest free point on this ray; try the next ray
        return (best, best_dist) if best is not None else None

    backward = search(heading + math.pi)
    if backward is not None:
        return backward[0], True
    if allow_forward:
        forward = search(heading)
        if forward is not None:
            return forward[0], False
    return None, False


def reverse_toward(
    obs: Observation,
    target: Point,
    speed: float,
    steer_gain: float,
) -> Action:
    """Back toward a target, steering as it goes.

    Steering inverts in reverse. To swing the back of the robot right, the left
    wheel has to run faster backwards, which is why the correction is
    subtracted from the left and added to the right rather than the other way
    round.
    """
    heading = math.radians(obs.robot_theta)
    angle_to_target = math.atan2(target[1] - obs.robot_y, target[0] - obs.robot_x)
    # Reversing means the robot's direction of travel is opposite its heading.
    error = _wrap_to_pi(angle_to_target - (heading + math.pi))

    base = -abs(speed)
    steer = error * steer_gain
    return Action(
        left_speed=max(-1.0, min(1.0, base - steer)),
        right_speed=max(-1.0, min(1.0, base + steer)),
    )


def blind_reverse(speed: float) -> Action:
    """Straight back, no steering. For when no free cell was found at all."""
    return Action(left_speed=-abs(speed), right_speed=-abs(speed))


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
