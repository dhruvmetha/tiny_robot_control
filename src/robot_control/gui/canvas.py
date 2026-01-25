"""Canvas component for robot visualization."""

from __future__ import annotations

import importlib.resources as pkg_resources
from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtCore import Qt, Signal, QRectF
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPixmap
from PySide6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsObject,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
    QSizePolicy,
)

from robot_control.core.types import ObjectPose, Observation, WorkspaceConfig

Point = Tuple[float, float]


def _load_car_pixmap() -> QPixmap:
    """Load car image from package resources."""
    try:
        car_png = pkg_resources.files("robot_control.assets") / "carImage.png"
        with pkg_resources.as_file(car_png) as p:
            pm = QPixmap(str(p))
        if pm.isNull():
            return QPixmap()
        return pm
    except Exception:
        return QPixmap()


class RobotItem(QGraphicsObject):
    """
    Visual representation of the robot matching micromvp style.

    Uses car pixmap with collision box overlay.
    """

    # The car image points up (+Y), but our heading 0 is +X, so rotate 90°
    IMAGE_HEADING_OFFSET_DEG = 90.0

    def __init__(
        self,
        car_pixmap: QPixmap,
        show_collision_box: bool = True,
        parent: Optional[QGraphicsItem] = None,
    ):
        super().__init__(parent)

        # Pixel dimensions (updated in update_visuals)
        self._width_px = 1.0   # car_width in pixels
        self._height_px = 1.0  # car_height in pixels (length along heading)
        self._offset_w_px = 0.0
        self._offset_h_px = 0.0

        # Hit rect for collision/selection
        self._hit_rect = QRectF(-0.5, -0.5, 1.0, 1.0)
        self._hit_path = QPainterPath()
        self._hit_path.addRect(self._hit_rect)

        # Car pixmap (child item)
        self._pixmap_source = car_pixmap
        self._pixmap_item = QGraphicsPixmapItem(car_pixmap, self)
        self._pixmap_item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        # Center the pixmap using offset
        pm_w = max(1, car_pixmap.width())
        pm_h = max(1, car_pixmap.height())
        self._pixmap_item.setOffset(-pm_w / 2.0, -pm_h / 2.0)
        self._pixmap_item.setZValue(0)

        # Collision box overlay (child item)
        self._collision_rect: Optional[QGraphicsRectItem] = None
        if show_collision_box:
            self._collision_rect = QGraphicsRectItem(self)
            self._collision_rect.setBrush(Qt.GlobalColor.transparent)
            self._collision_rect.setPen(QPen(QColor(30, 60, 90), 2))
            self._collision_rect.setZValue(10)

    def paint(self, painter: QPainter, option, widget=None) -> None:
        # Children handle all drawing
        pass

    def boundingRect(self) -> QRectF:
        return self._hit_rect

    def shape(self) -> QPainterPath:
        return self._hit_path

    def update_visuals(
        self,
        width_px: float,
        height_px: float,
        offset_w_px: float,
        offset_h_px: float,
    ) -> None:
        """
        Update robot size matching micromvp convention.

        Args:
            width_px: car_width in pixels (perpendicular to heading)
            height_px: car_height in pixels (along heading)
            offset_w_px: Center offset from left edge in pixels
            offset_h_px: Center offset from bottom edge in pixels
        """
        width_px = max(1.0, float(width_px))
        height_px = max(1.0, float(height_px))

        self.prepareGeometryChange()
        self._width_px = width_px
        self._height_px = height_px
        self._offset_w_px = float(offset_w_px)
        self._offset_h_px = float(offset_h_px)

        # Hit rect with offset (matching micromvp)
        # The rect is positioned so that (0,0) is at the offset point
        self._hit_rect = QRectF(
            -(width_px - offset_w_px),
            -(height_px - offset_h_px),
            width_px,
            height_px,
        )
        self._hit_path = QPainterPath()
        self._hit_path.addRect(self._hit_rect)

        # Scale pixmap to match robot width
        pm_w = max(1, self._pixmap_source.width())
        img_scale = width_px / float(pm_w)
        self._pixmap_item.setScale(img_scale)
        self._pixmap_item.setPos(0.0, 0.0)

        # Update collision rect
        if self._collision_rect:
            self._collision_rect.setRect(self._hit_rect)

    def set_pose_pixels(self, px: float, py: float, theta_deg: float) -> None:
        """Set robot pose in pixel coordinates."""
        self.setPos(px, py)
        # Negate theta for screen coords (Y-down), add offset for image orientation
        screen_rot = -theta_deg + self.IMAGE_HEADING_OFFSET_DEG
        self.setRotation(screen_rot)


class Canvas(QGraphicsView):
    """Main canvas for robot visualization."""

    canvas_clicked = Signal(float, float)  # workspace coords
    curve_drawn = Signal(list)  # List of (x, y) workspace coords

    def __init__(self, config: WorkspaceConfig, parent=None):
        super().__init__(parent)
        self._config = config

        # Scene setup
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        # View settings
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setBackgroundBrush(QBrush(QColor(245, 245, 245)))
        self.setFrameShape(QGraphicsView.Shape.NoFrame)

        # Transform parameters
        self._pixels_per_unit = 1.0
        self._offset_x = 0.0
        self._offset_y = 0.0

        # Load car image
        self._car_pixmap = _load_car_pixmap()

        # Graphics items
        self._robot_item = RobotItem(self._car_pixmap, show_collision_box=True)
        self._scene.addItem(self._robot_item)
        self._boundary_item: Optional[QGraphicsRectItem] = None
        self._object_items: Dict[str, QGraphicsRectItem] = {}

        # Drawing items cache (for paths, targets, etc.)
        self._drawing_items: Dict[str, QGraphicsItem] = {}
        self._last_drawings_data: Dict[str, Dict[str, Any]] = {}

        # Interaction
        self._click_enabled = False
        self._draw_enabled = False
        self._is_drawing = False
        self._current_path_ws: List[Point] = []
        self._temp_path_item: Optional[QGraphicsPathItem] = None

    def enable_click(self, enabled: bool) -> None:
        self._click_enabled = enabled

    def enable_draw(self, enabled: bool) -> None:
        """Enable/disable curve drawing on canvas."""
        self._draw_enabled = enabled

    def _update_transform(self) -> None:
        """Calculate scale and offsets to fit workspace in view."""
        view_rect = self.viewport().rect()
        vw, vh = view_rect.width(), view_rect.height()
        ws_w, ws_h = self._config.width, self._config.height

        if ws_w == 0 or ws_h == 0 or vw == 0 or vh == 0:
            return

        # Fit with padding
        padding = 0.95
        scale_x = (vw * padding) / ws_w
        scale_y = (vh * padding) / ws_h
        self._pixels_per_unit = min(scale_x, scale_y)

        # Center
        content_w = ws_w * self._pixels_per_unit
        content_h = ws_h * self._pixels_per_unit
        self._offset_x = (vw - content_w) / 2.0
        self._offset_y = (vh - content_h) / 2.0

        self._scene.setSceneRect(0, 0, vw, vh)

    def workspace_to_pixel(self, wx: float, wy: float) -> Tuple[float, float]:
        """Convert workspace coords (Y-up) to pixel coords (Y-down)."""
        px = wx * self._pixels_per_unit + self._offset_x
        py = self._offset_y + (self._config.height - wy) * self._pixels_per_unit
        return px, py

    def pixel_to_workspace(self, px: float, py: float) -> Tuple[float, float]:
        """Convert pixel coords (Y-down) to workspace coords (Y-up)."""
        if self._pixels_per_unit <= 0:
            return 0, 0
        wx = (px - self._offset_x) / self._pixels_per_unit
        wy = self._config.height - (py - self._offset_y) / self._pixels_per_unit
        return wx, wy

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_transform()
        self._redraw()

    def _redraw(self) -> None:
        """Redraw all items with current transform."""
        self._draw_boundary()
        # Robot and objects will be updated via update() calls

    def _draw_boundary(self) -> None:
        """Draw workspace boundary."""
        if not self._boundary_item:
            self._boundary_item = QGraphicsRectItem()
            pen = QPen(QColor(100, 100, 100), 2, Qt.PenStyle.DashLine)
            self._boundary_item.setPen(pen)
            self._boundary_item.setBrush(Qt.GlobalColor.transparent)
            self._scene.addItem(self._boundary_item)

        tl_x, tl_y = self.workspace_to_pixel(0, self._config.height)
        br_x, br_y = self.workspace_to_pixel(self._config.width, 0)
        self._boundary_item.setRect(tl_x, tl_y, br_x - tl_x, br_y - tl_y)

    def update_robot(self, obs: Observation) -> None:
        """Update robot position from observation."""
        # Update robot visuals (matching micromvp convention)
        scale = self._pixels_per_unit
        width_px = self._config.car_width * scale
        height_px = self._config.car_height * scale
        offset_w_px = self._config.offset_w * scale
        offset_h_px = self._config.offset_h * scale
        self._robot_item.update_visuals(width_px, height_px, offset_w_px, offset_h_px)

        # Update robot pose
        px, py = self.workspace_to_pixel(obs.robot_x, obs.robot_y)
        self._robot_item.set_pose_pixels(px, py, obs.robot_theta)

        # Update objects
        self._update_objects(obs.objects)

    def _update_objects(self, objects: Dict[str, ObjectPose]) -> None:
        """Update object positions and sizes."""
        current_ids = set(objects.keys())
        existing_ids = set(self._object_items.keys())

        # Remove stale objects
        for obj_id in existing_ids - current_ids:
            self._scene.removeItem(self._object_items.pop(obj_id))

        # Default size (used when object has no size info)
        default_size = self._config.car_width * 0.8

        # Add/update objects
        for obj_id, pose in objects.items():
            if obj_id not in self._object_items:
                item = QGraphicsRectItem()
                item.setBrush(QBrush(QColor(139, 69, 19)))  # Brown
                item.setPen(QPen(QColor(80, 40, 10), 1))
                self._scene.addItem(item)
                self._object_items[obj_id] = item

            item = self._object_items[obj_id]

            # Use object size if available, otherwise default
            obj_width = pose.width if pose.width > 0 else default_size
            obj_height = pose.height if pose.height > 0 else default_size

            width_px = obj_width * self._pixels_per_unit
            height_px = obj_height * self._pixels_per_unit
            px, py = self.workspace_to_pixel(pose.x, pose.y)
            item.setRect(px - width_px/2, py - height_px/2, width_px, height_px)

    def update_drawings(self, drawings: List[Dict[str, Any]]) -> None:
        """
        Update retained-mode drawings (paths, targets, indicators, etc.).

        Supported drawing types:
        - "path": List of (x,y) points as a polyline
        - "point": Single point with radius
        - "line": Line from start to end
        - "circle": Circle with center and radius

        Args:
            drawings: List of drawing dictionaries with "uuid" and "type" keys
        """
        current_uuids = set()

        for drawing in drawings:
            uuid = drawing.get("uuid")
            if not uuid:
                continue

            current_uuids.add(uuid)
            self._last_drawings_data[uuid] = drawing

            if uuid not in self._drawing_items:
                item = self._create_drawing_item(drawing)
                if item:
                    self._scene.addItem(item)
                    self._drawing_items[uuid] = item

            if uuid in self._drawing_items:
                self._update_drawing_geometry(self._drawing_items[uuid], drawing)

        # Remove stale drawings
        stale_uuids = set(self._drawing_items.keys()) - current_uuids
        for uuid in stale_uuids:
            self._scene.removeItem(self._drawing_items.pop(uuid))
            self._last_drawings_data.pop(uuid, None)

    def _create_drawing_item(self, drawing: Dict[str, Any]) -> Optional[QGraphicsItem]:
        """Create a new graphics item for a drawing."""
        dtype = drawing.get("type", "")
        if dtype == "line":
            return QGraphicsLineItem()
        if dtype in ("circle", "point"):
            return QGraphicsEllipseItem()
        if dtype == "rect":
            return QGraphicsRectItem()
        if dtype == "path":
            return QGraphicsPathItem()
        return None

    def _update_drawing_geometry(
        self, item: QGraphicsItem, drawing: Dict[str, Any]
    ) -> None:
        """Update drawing item geometry from drawing data."""
        dtype = drawing.get("type", "")
        scale = self._pixels_per_unit

        # Apply style
        color = QColor(drawing.get("color", "#FF0000"))
        width = drawing.get("width", 2)
        pen = QPen(color, width)
        if hasattr(item, "setPen"):
            item.setPen(pen)

        if "fill" in drawing and hasattr(item, "setBrush"):
            item.setBrush(QBrush(QColor(drawing["fill"])))
        elif hasattr(item, "setBrush"):
            item.setBrush(Qt.GlobalColor.transparent)

        # Apply geometry
        if dtype == "line" and isinstance(item, QGraphicsLineItem):
            x1, y1 = drawing.get("start", (0, 0))
            x2, y2 = drawing.get("end", (0, 0))
            px1, py1 = self.workspace_to_pixel(x1, y1)
            px2, py2 = self.workspace_to_pixel(x2, y2)
            item.setLine(px1, py1, px2, py2)

        elif dtype == "circle" and isinstance(item, QGraphicsEllipseItem):
            cx, cy = drawing.get("center", (0, 0))
            r_m = drawing.get("radius", 10)
            px, py = self.workspace_to_pixel(cx, cy)
            r_px = r_m * scale
            item.setRect(px - r_px, py - r_px, r_px * 2, r_px * 2)

        elif dtype == "point" and isinstance(item, QGraphicsEllipseItem):
            cx, cy = drawing.get("position", (0, 0))
            r_px = drawing.get("radius", 4)  # Radius in pixels
            px, py = self.workspace_to_pixel(cx, cy)
            item.setRect(px - r_px, py - r_px, r_px * 2, r_px * 2)

        elif dtype == "rect" and isinstance(item, QGraphicsRectItem):
            x, y = drawing.get("position", (0, 0))
            w, h = drawing.get("size", (10, 10))
            px_l, py_t = self.workspace_to_pixel(x, y + h)
            w_px = w * scale
            h_px = h * scale
            item.setRect(px_l, py_t, w_px, h_px)

        elif dtype == "path" and isinstance(item, QGraphicsPathItem):
            points = drawing.get("points", [])
            path = QPainterPath()
            if points:
                px0, py0 = self.workspace_to_pixel(points[0][0], points[0][1])
                path.moveTo(px0, py0)
                for pt in points[1:]:
                    px, py = self.workspace_to_pixel(pt[0], pt[1])
                    path.lineTo(px, py)
            item.setPath(path)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            wx, wy = self.pixel_to_workspace(scene_pos.x(), scene_pos.y())

            if self._draw_enabled:
                # Start drawing a curve
                self._is_drawing = True
                self._current_path_ws = [(wx, wy)]

                # Create temporary path item for visual feedback
                self._temp_path_item = QGraphicsPathItem()
                self._temp_path_item.setPen(QPen(QColor(255, 100, 100), 2))
                self._scene.addItem(self._temp_path_item)

            elif self._click_enabled:
                self.canvas_clicked.emit(wx, wy)

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._is_drawing and self._temp_path_item:
            scene_pos = self.mapToScene(event.pos())
            wx, wy = self.pixel_to_workspace(scene_pos.x(), scene_pos.y())
            self._current_path_ws.append((wx, wy))

            # Update visual path
            path = QPainterPath()
            start_px, start_py = self.workspace_to_pixel(
                self._current_path_ws[0][0], self._current_path_ws[0][1]
            )
            path.moveTo(start_px, start_py)

            for pt in self._current_path_ws[1:]:
                px, py = self.workspace_to_pixel(pt[0], pt[1])
                path.lineTo(px, py)

            self._temp_path_item.setPath(path)

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self._is_drawing:
            self._is_drawing = False

            # Emit curve if enough points drawn
            if len(self._current_path_ws) > 5:
                self.curve_drawn.emit(self._current_path_ws)
            elif self._click_enabled and len(self._current_path_ws) > 0:
                # Treat as click if path is too short
                p = self._current_path_ws[0]
                self.canvas_clicked.emit(p[0], p[1])

            # Cleanup temp path
            if self._temp_path_item:
                self._scene.removeItem(self._temp_path_item)
                self._temp_path_item = None
            self._current_path_ws = []

        super().mouseReleaseEvent(event)
