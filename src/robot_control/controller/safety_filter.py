"""Safety filter: stop a push before the robot reaches a wall.

The walls on the table are heavy bricks that do not move during a trial, so
the distance from every point of the table to the nearest wall is computed
once and stored on a grid. Each control tick then samples the robot's outline
and looks every sample up in that grid. A push stops when the outline enters
the band of ``margin_cm`` around any wall or the table edge.

Only the robot is checked. The pushed block may touch a wall; sim treats that
as a normal outcome and the push controller's stuck detection covers a block
that stops moving.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from robot_control.core.types import Observation, WorkspaceConfig

# Grid pitch. Matches WavefrontConfig.resolution (0.005 m) so the two grids
# agree on what a cell is.
CELL_CM = 0.5

# A lookup reads the cell containing the query point, whose centre can sit up
# to half a cell diagonal away from the point. Subtracting it makes every
# lookup at most the true distance, never more. The price is that a lookup can
# read up to a full cell diagonal (0.71 cm) under the true distance, so every
# stop lands that much earlier, never later.
HALF_CELL_DIAGONAL_CM = CELL_CM * math.sqrt(2.0) / 2.0

# Marker position jitter seen in the formal_v2 scene captures. A clearance
# that shrinks by less than this since the push began is noise, not motion.
ARUCO_NOISE_BAND_CM = 0.5

# Below this clearance the robot stops whatever the push looked like at its
# start. Under the noise band on purpose, so it only fires on a real approach
# to contact.
HARD_STOP_CM = 0.3

# Name reported when the nearest unsafe thing is the table edge itself.
WORKSPACE_EDGE = "workspace_edge"


@dataclass(frozen=True)
class StaticBox:
    """An oriented rectangle, same size convention as ``ObjectPose``.

    ``half_x`` is half the depth, along the heading. ``half_y`` is half the
    width, across it.
    """

    name: str
    cx: float
    cy: float
    theta_deg: float
    half_x: float
    half_y: float

    def corners(self) -> List[Tuple[float, float]]:
        c = math.cos(math.radians(self.theta_deg))
        s = math.sin(math.radians(self.theta_deg))
        out = []
        for lx, ly in ((self.half_x, self.half_y), (-self.half_x, self.half_y),
                       (-self.half_x, -self.half_y), (self.half_x, -self.half_y)):
            out.append((self.cx + c * lx - s * ly, self.cy + s * lx + c * ly))
        return out


@dataclass(frozen=True)
class Clearance:
    """Distance from a queried outline to the nearest static, and which one."""

    distance_cm: float
    static_name: str


@dataclass(frozen=True)
class Violation:
    """Why the filter stopped the robot."""

    static_name: str
    distance_now_cm: float
    distance_at_start_cm: Optional[float]

    @property
    def reason(self) -> str:
        return f"robot_static_clearance:{self.static_name}"


def point_to_box_cm(px, py, box: StaticBox):
    """Exact distance from points to an oriented rectangle, negative inside.

    Accepts scalars or numpy arrays for ``px`` and ``py``.
    """
    c = math.cos(math.radians(box.theta_deg))
    s = math.sin(math.radians(box.theta_deg))
    dx = np.asarray(px, dtype=float) - box.cx
    dy = np.asarray(py, dtype=float) - box.cy
    # Rotate into the box frame.
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ex = np.abs(lx) - box.half_x
    ey = np.abs(ly) - box.half_y
    outside = np.hypot(np.maximum(ex, 0.0), np.maximum(ey, 0.0))
    inside = np.minimum(np.maximum(ex, ey), 0.0)
    result = outside + inside
    return float(result) if result.ndim == 0 else result


def outline_points(
    cx: float, cy: float, theta_deg: float, width_cm: float, height_cm: float,
    pitch_cm: float = CELL_CM,
) -> Tuple[np.ndarray, np.ndarray]:
    """Sample a rectangle's perimeter every ``pitch_cm``.

    ``height_cm`` runs along the heading and ``width_cm`` across it, the
    ``WorkspaceConfig`` convention (car_height is front to back). Corners
    alone would miss a brick corner poking into the middle of a face; at
    cell pitch the miss is at most ``pitch_cm / 2``.
    """
    hx = height_cm / 2.0
    hy = width_cm / 2.0
    corners = np.array([(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)], dtype=float)
    local = []
    for i in range(4):
        a = corners[i]
        b = corners[(i + 1) % 4]
        n = max(1, int(math.ceil(float(np.hypot(*(b - a))) / pitch_cm)))
        t = np.arange(n, dtype=float) / n  # [0, 1): the end corner is the next edge's start
        local.append(a[None, :] + t[:, None] * (b - a)[None, :])
    pts = np.concatenate(local, axis=0)
    c = math.cos(math.radians(theta_deg))
    s = math.sin(math.radians(theta_deg))
    xs = cx + c * pts[:, 0] - s * pts[:, 1]
    ys = cy + s * pts[:, 0] + c * pts[:, 1]
    return xs, ys


class SafetyFilter:
    """Distance-to-wall grid plus the per-tick robot check.

    Build one per session. Walls are fixed, so a static is recorded once and
    never updated or removed; ``add_statics`` exists only so that a marker the
    camera misses on one frame is picked up on another.
    """

    def __init__(
        self,
        workspace: WorkspaceConfig,
        margin_cm: float,
        cell_cm: float = CELL_CM,
    ) -> None:
        self._workspace = workspace
        self._margin_cm = float(margin_cm)
        self._cell_cm = float(cell_cm)
        # See HALF_CELL_DIAGONAL_CM; recomputed here so a finer grid rounds less.
        self._half_diag_cm = self._cell_cm * math.sqrt(2.0) / 2.0
        self._nx = int(math.ceil(workspace.width / cell_cm))
        self._ny = int(math.ceil(workspace.height / cell_cm))
        centres = (np.arange(self._nx, dtype=float) + 0.5) * cell_cm
        rows = (np.arange(self._ny, dtype=float) + 0.5) * cell_cm
        self._gx, self._gy = np.meshgrid(centres, rows)  # shape (ny, nx)
        self._statics: Dict[str, StaticBox] = {}
        self._names: List[str] = [WORKSPACE_EDGE]
        self._dist: np.ndarray = np.empty((self._ny, self._nx), dtype=float)
        self._nearest: np.ndarray = np.empty((self._ny, self._nx), dtype=np.int32)
        self.build()

    # ------------------------------------------------------------------
    # Static set
    # ------------------------------------------------------------------

    @property
    def margin_cm(self) -> float:
        return self._margin_cm

    @property
    def statics(self) -> List[StaticBox]:
        return list(self._statics.values())

    def add_statics(self, obs: Observation) -> bool:
        """Record every static in ``obs`` not yet seen. Returns True on a rebuild."""
        added = False
        for name, pose in obs.objects.items():
            if not pose.is_static or name in self._statics:
                continue
            if pose.width <= 0 or pose.depth <= 0:
                continue
            self._statics[name] = StaticBox(
                name=name,
                cx=pose.x,
                cy=pose.y,
                theta_deg=pose.theta,
                half_x=pose.depth / 2.0,
                half_y=pose.width / 2.0,
            )
            added = True
        if added:
            self.build()
        return added

    def build(self) -> None:
        """Fill the grid with the distance to the nearest static or table edge."""
        w = self._workspace.width
        h = self._workspace.height
        edge = np.minimum(
            np.minimum(self._gx, w - self._gx),
            np.minimum(self._gy, h - self._gy),
        )
        dist = edge
        nearest = np.zeros_like(edge, dtype=np.int32)
        self._names = [WORKSPACE_EDGE]
        for box in self._statics.values():
            self._names.append(box.name)
            idx = len(self._names) - 1
            d = point_to_box_cm(self._gx, self._gy, box)
            closer = d < dist
            dist = np.where(closer, d, dist)
            nearest = np.where(closer, idx, nearest)
        self._dist = dist
        self._nearest = nearest

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def lookup(self, xs, ys) -> Clearance:
        """Minimum clearance over the query points, and the static at it."""
        xs = np.atleast_1d(np.asarray(xs, dtype=float))
        ys = np.atleast_1d(np.asarray(ys, dtype=float))
        w = self._workspace.width
        h = self._workspace.height
        ix = np.floor(xs / self._cell_cm).astype(int)
        iy = np.floor(ys / self._cell_cm).astype(int)
        inside = (ix >= 0) & (ix < self._nx) & (iy >= 0) & (iy < self._ny)
        d = np.empty_like(xs)
        who = np.zeros(xs.shape, dtype=np.int32)
        if inside.any():
            d[inside] = self._dist[iy[inside], ix[inside]] - self._half_diag_cm
            who[inside] = self._nearest[iy[inside], ix[inside]]
        if (~inside).any():
            ox = np.maximum(np.maximum(-xs, xs - w), 0.0)
            oy = np.maximum(np.maximum(-ys, ys - h), 0.0)
            d[~inside] = -np.hypot(ox, oy)[~inside]
        k = int(np.argmin(d))
        return Clearance(distance_cm=float(d[k]), static_name=self._names[int(who[k])])

    def robot_outline(self, x_cm: float, y_cm: float, theta_deg: float):
        return outline_points(
            x_cm, y_cm, theta_deg,
            self._workspace.car_width, self._workspace.car_height,
            pitch_cm=self._cell_cm,
        )

    def robot_clearance(self, x_cm: float, y_cm: float, theta_deg: float) -> Clearance:
        xs, ys = self.robot_outline(x_cm, y_cm, theta_deg)
        return self.lookup(xs, ys)

    def check_entering(
        self,
        x_cm: float,
        y_cm: float,
        theta_deg: float,
        clearance_at_start_cm: Optional[float],
    ) -> Optional[Violation]:
        """Is the robot entering the unsafe band around a static?

        A robot that started clear of the band violates the moment it is
        inside. One that started inside violates only once its clearance has
        shrunk by more than the marker noise band, so a push that runs
        alongside a wall passes and one that closes on it stops. Below
        ``HARD_STOP_CM`` it always violates.
        """
        now = self.robot_clearance(x_cm, y_cm, theta_deg)
        if now.distance_cm < HARD_STOP_CM:
            return Violation(now.static_name, now.distance_cm, clearance_at_start_cm)
        if now.distance_cm >= self._margin_cm:
            return None
        started_inside = (
            clearance_at_start_cm is not None and clearance_at_start_cm < self._margin_cm
        )
        if not started_inside:
            return Violation(now.static_name, now.distance_cm, clearance_at_start_cm)
        if clearance_at_start_cm - now.distance_cm > ARUCO_NOISE_BAND_CM:
            return Violation(now.static_name, now.distance_cm, clearance_at_start_cm)
        return None
