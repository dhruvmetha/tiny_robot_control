"""Synthetic top-down PNG renderer for scene visualization.

Draws a simple top-down view of the workspace from observation data alone —
no Qt, no MuJoCo viewer, no live camera. Works in both sim and real modes.
The image is intended as a quick human-readable visual; for reproduction
you want the accompanying scene_*.json and scene_*.xml.

The renderer uses OpenCV which is already a dependency of robot_control.
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from robot_control.core.types import Observation, WorkspaceConfig
from robot_control.utils.robot_geometry import effective_robot_radius_cm


# Tunable constants — kept small here rather than threaded through args.
_PX_PER_CM: float = 8.0          # Image resolution in pixels per cm
_MARGIN_CM: float = 3.0          # Border around the workspace
_BG_COLOR = (245, 245, 245)      # Light gray background (BGR)
_WORKSPACE_BORDER = (60, 60, 60)
_WORKSPACE_FILL = (255, 255, 255)
_STATIC_FILL = (140, 140, 140)   # Walls in mid gray
_MOVABLE_FILL = (200, 140, 60)   # Movable objects in blue (BGR)
_MOVABLE_EDGE = (90, 60, 20)
_GOAL_COLOR = (40, 180, 40)      # Green
_ROBOT_BODY = (40, 40, 200)      # Red (BGR)
_ROBOT_HEADING = (10, 10, 80)    # Darker red
_TEXT_COLOR = (40, 40, 40)


def render_top_down(
    workspace: WorkspaceConfig,
    observation: Observation,
    title: Optional[str] = None,
) -> bytes:
    """Render a top-down view as JPEG bytes.

    Args:
        workspace: Workspace dimensions (cm). Determines image size.
        observation: Robot + object poses to draw.
        title: Optional title rendered in the top-left corner.

    Returns:
        JPEG-encoded image bytes.
    """
    # Image size derived from workspace + margin.
    w_cm = workspace.width + 2 * _MARGIN_CM
    h_cm = workspace.height + 2 * _MARGIN_CM
    img_w = int(round(w_cm * _PX_PER_CM))
    img_h = int(round(h_cm * _PX_PER_CM))

    img = np.full((img_h, img_w, 3), _BG_COLOR, dtype=np.uint8)

    def cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]:
        # Translate world coords (origin bottom-left of workspace) to pixel
        # coords (origin top-left of image), accounting for the margin and
        # flipping the Y axis so up-in-world is up-in-image.
        px = int(round((_MARGIN_CM + x_cm) * _PX_PER_CM))
        py = int(round(img_h - (_MARGIN_CM + y_cm) * _PX_PER_CM))
        return px, py

    # 1) Workspace box (white fill + dark border).
    p0 = cm_to_px(0.0, 0.0)
    p1 = cm_to_px(workspace.width, workspace.height)
    cv2.rectangle(img, p0, p1, _WORKSPACE_FILL, thickness=-1)
    cv2.rectangle(img, p0, p1, _WORKSPACE_BORDER, thickness=2)

    # 2) Static obstacles (walls), then movable objects on top of them so
    # overlapping markers stay visible.
    static_items = [(n, o) for n, o in observation.objects.items() if o.is_static]
    movable_items = [(n, o) for n, o in observation.objects.items() if not o.is_static]

    for name, obj in static_items + movable_items:
        _draw_object(img, cm_to_px, obj,
                     fill=_STATIC_FILL if obj.is_static else _MOVABLE_FILL,
                     edge=_WORKSPACE_BORDER if obj.is_static else _MOVABLE_EDGE,
                     label=name)

    # 3) Goal marker — only if known.
    if observation.goal_x is not None and observation.goal_y is not None:
        gx, gy = cm_to_px(observation.goal_x, observation.goal_y)
        # Draw a crosshair + circle.
        cv2.circle(img, (gx, gy), int(2.5 * _PX_PER_CM), _GOAL_COLOR, thickness=2)
        cv2.line(img, (gx - 10, gy), (gx + 10, gy), _GOAL_COLOR, thickness=2)
        cv2.line(img, (gx, gy - 10), (gx, gy + 10), _GOAL_COLOR, thickness=2)
        cv2.putText(img, "GOAL", (gx + 14, gy - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, _GOAL_COLOR, 1, cv2.LINE_AA)

    # 4) Robot — body + heading arrow.
    rx, ry = cm_to_px(observation.robot_x, observation.robot_y)
    body_radius = int(effective_robot_radius_cm(workspace.car_width, workspace.car_height) * _PX_PER_CM)
    cv2.circle(img, (rx, ry), body_radius, _ROBOT_BODY, thickness=-1)
    cv2.circle(img, (rx, ry), body_radius, _ROBOT_HEADING, thickness=2)
    # Heading indicator: arrow from robot center along its theta.
    theta_rad = np.deg2rad(observation.robot_theta)
    hx = int(rx + np.cos(theta_rad) * (body_radius + 8))
    hy = int(ry - np.sin(theta_rad) * (body_radius + 8))  # Image Y is inverted
    cv2.arrowedLine(img, (rx, ry), (hx, hy), _ROBOT_HEADING, thickness=2,
                    tipLength=0.4)
    cv2.putText(img, "R", (rx - 5, ry + 5), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1, cv2.LINE_AA)

    # 5) Title + minimal axes legend so the orientation is unambiguous.
    if title:
        cv2.putText(img, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, _TEXT_COLOR, 1, cv2.LINE_AA)
    # Tiny axes glyph in bottom-left margin: +X right, +Y up.
    ox = int(_MARGIN_CM * 0.5 * _PX_PER_CM)
    oy = img_h - int(_MARGIN_CM * 0.5 * _PX_PER_CM)
    cv2.arrowedLine(img, (ox, oy), (ox + 22, oy), _TEXT_COLOR, 1, tipLength=0.3)
    cv2.arrowedLine(img, (ox, oy), (ox, oy - 22), _TEXT_COLOR, 1, tipLength=0.3)
    cv2.putText(img, "x", (ox + 24, oy + 4), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, _TEXT_COLOR, 1, cv2.LINE_AA)
    cv2.putText(img, "y", (ox - 4, oy - 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, _TEXT_COLOR, 1, cv2.LINE_AA)

    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if not ok:
        # Fall back to PNG if JPEG fails for some reason — better than nothing.
        ok2, buf = cv2.imencode(".png", img)
        if not ok2:
            return b""
    return buf.tobytes()


def _draw_object(img, cm_to_px, obj, fill, edge, label: str) -> None:
    # ObjectPose stores depth (X-along-heading) and width (Y-perpendicular).
    # Treat the object as a rectangle centered on (obj.x, obj.y) and rotated
    # by obj.theta (degrees, CCW from +X).
    half_depth = obj.depth / 2.0
    half_width = obj.width / 2.0
    if half_depth <= 0 or half_width <= 0:
        # Object has unknown size — draw a small placeholder.
        cx, cy = cm_to_px(obj.x, obj.y)
        cv2.circle(img, (cx, cy), 6, fill, thickness=-1)
        cv2.circle(img, (cx, cy), 6, edge, thickness=1)
        cv2.putText(img, label, (cx + 8, cy - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, _TEXT_COLOR, 1, cv2.LINE_AA)
        return

    # Build local-frame corners (in order: front-right, front-left, back-left, back-right)
    corners_local = np.array([
        [ half_depth, -half_width],
        [ half_depth,  half_width],
        [-half_depth,  half_width],
        [-half_depth, -half_width],
    ])
    theta_rad = np.deg2rad(obj.theta)
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    R = np.array([[c, -s], [s, c]])
    corners_world = corners_local @ R.T
    corners_world[:, 0] += obj.x
    corners_world[:, 1] += obj.y

    pts = np.array([cm_to_px(p[0], p[1]) for p in corners_world], dtype=np.int32)
    cv2.fillPoly(img, [pts], fill)
    cv2.polylines(img, [pts], isClosed=True, color=edge, thickness=2)

    # Label
    cx, cy = cm_to_px(obj.x, obj.y)
    cv2.putText(img, label, (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, _TEXT_COLOR, 1, cv2.LINE_AA)
