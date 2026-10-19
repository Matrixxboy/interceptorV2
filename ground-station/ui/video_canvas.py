"""Video Canvas Widget (§10, T4.3).

Displays live video stream, handles letterboxing/pillarboxing, and maps pilot taps
from widget pixel coordinates to normalized [0, 1] video coordinates.
PySide6 is lazily imported (§2.3).
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def map_canvas_click_to_video(
    click_x: float,
    click_y: float,
    widget_w: float,
    widget_h: float,
    video_w: float,
    video_h: float,
) -> tuple[float, float] | None:
    """Map a click within widget bounds to normalized video coordinates [0, 1].

    Accounts for aspect-ratio preservation (letterboxing / pillarboxing).
    Pure function (§2.7).

    Args:
        click_x: X coordinate of click within widget (pixels).
        click_y: Y coordinate of click within widget (pixels).
        widget_w: Width of canvas widget (pixels).
        widget_h: Height of canvas widget (pixels).
        video_w: Native width of video frame (pixels).
        video_h: Native height of video frame (pixels).

    Returns:
        Tuple of (x_norm, y_norm) in [0, 1], or None if click was in letterbox padding.
    """
    if widget_w <= 0 or widget_h <= 0 or video_w <= 0 or video_h <= 0:
        return None

    # Compute video aspect ratio and widget aspect ratio
    video_aspect = video_w / video_h
    widget_aspect = widget_w / widget_h

    if widget_aspect > video_aspect:
        # Pillarboxed (black bars on left and right)
        displayed_h = widget_h
        displayed_w = displayed_h * video_aspect
        pad_x = (widget_w - displayed_w) / 2.0
        pad_y = 0.0
    else:
        # Letterboxed (black bars on top and bottom)
        displayed_w = widget_w
        displayed_h = displayed_w / video_aspect
        pad_x = 0.0
        pad_y = (widget_h - displayed_h) / 2.0

    # Check if click is inside the video rectangle
    if not (pad_x <= click_x <= pad_x + displayed_w):
        return None
    if not (pad_y <= click_y <= pad_y + displayed_h):
        return None

    x_norm = (click_x - pad_x) / displayed_w
    y_norm = (click_y - pad_y) / displayed_h

    # Clamp to [0, 1]
    x_norm = max(0.0, min(1.0, x_norm))
    y_norm = max(0.0, min(1.0, y_norm))

    return x_norm, y_norm


class VideoCanvas:
    """Video canvas wrapper and widget.

    PySide6 is imported lazily inside methods (§2.3).
    """

    def __init__(self, on_tap: Callable[[float, float], None] | None = None) -> None:
        self.on_tap = on_tap
        self._current_frame: np.ndarray | None = None
        self._last_telemetry: Any = None
        self._widget: Any = None
        self._bridge: Any = None
        self._last_tap_screen: tuple[float, float, float] | None = None  # (x, y, timestamp)

    def get_widget(self) -> Any:
        """Create and return PySide6 QWidget."""
        if self._widget is not None:
            return self._widget

        import time

        from PySide6.QtCore import QObject, Qt, Signal
        from PySide6.QtGui import QColor, QFont, QImage, QMouseEvent, QPainter, QPen, QPixmap
        from PySide6.QtWidgets import QLabel

        canvas_self = self

        class _CanvasBridge(QObject):
            frame_received = Signal(object)
            telemetry_received = Signal(object)

        class _CanvasWidget(QLabel):
            def __init__(self) -> None:
                super().__init__()
                self.setAlignment(Qt.AlignCenter)
                self.setStyleSheet("background-color: #121212; border: 1px solid #282828;")
                self.setMinimumSize(320, 240)
                self.setMouseTracking(True)

            def mousePressEvent(self, event: QMouseEvent) -> None:
                if not canvas_self.on_tap:
                    return
                click_x = float(event.position().x())
                click_y = float(event.position().y())
                widget_w = float(self.width())
                widget_h = float(self.height())
                frame_w = 1280.0
                frame_h = 720.0
                if canvas_self._current_frame is not None:
                    fh, fw = canvas_self._current_frame.shape[:2]
                    frame_w, frame_h = float(fw), float(fh)

                coords = map_canvas_click_to_video(
                    click_x, click_y, widget_w, widget_h, frame_w, frame_h
                )
                if coords is not None:
                    canvas_self._last_tap_screen = (click_x, click_y, time.monotonic())
                    logger.info(
                        "Canvas tap at norm (%.3f, %.3f), px (%.1f, %.1f)",
                        coords[0],
                        coords[1],
                        click_x,
                        click_y,
                    )
                    canvas_self.on_tap(coords[0], coords[1])
                    self.update()

            def on_frame(self, frame: np.ndarray) -> None:
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_BGR888)
                pixmap = QPixmap.fromImage(qimg)

                # Scale pixmap preserving aspect ratio
                scaled_pixmap = pixmap.scaled(
                    self.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )

                # Overlay HUD overlays onto pixmap
                painter = QPainter(scaled_pixmap)
                painter.setRenderHint(QPainter.Antialiasing)

                disp_w = float(scaled_pixmap.width())
                disp_h = float(scaled_pixmap.height())

                # 1. Draw target bounding box if in telemetry
                telem = canvas_self._last_telemetry
                if telem is not None:
                    bbox_norm = getattr(telem, "bbox_norm", None)
                    state = str(getattr(telem, "state", "IDLE"))
                    dist = getattr(telem, "distance_m", None)

                    if bbox_norm and len(bbox_norm) == 4:
                        bx1, by1, bx2, by2 = bbox_norm
                        x = bx1 * disp_w
                        y = by1 * disp_h
                        bw = (bx2 - bx1) * disp_w
                        bh = (by2 - by1) * disp_h

                        if state == "FOLLOWING":
                            box_color = QColor(0, 230, 118)  # Neon Green
                            label_txt = f"LOCKED {dist:.1f}m" if dist else "LOCKED"
                        elif state == "SELECTING":
                            box_color = QColor(255, 214, 0)  # Amber
                            label_txt = "ACQUIRING..."
                        else:
                            box_color = QColor(255, 23, 68)  # Red / Lost
                            label_txt = "TARGET LOST"

                        pen = QPen(box_color, 2, Qt.SolidLine)
                        painter.setPen(pen)
                        painter.drawRoundedRect(int(x), int(y), int(bw), int(bh), 4, 4)

                        # Label badge
                        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
                        painter.fillRect(
                            int(x), int(max(0, y - 20)), int(bw), 20, QColor(0, 0, 0, 180)
                        )
                        painter.setPen(box_color)
                        painter.drawText(int(x + 4), int(max(14, y - 5)), label_txt)

                # 2. Draw tap ripple / crosshair if tapped recently (< 1.5s)
                if canvas_self._last_tap_screen is not None:
                    tap_x, tap_y, tap_ts = canvas_self._last_tap_screen
                    age = time.monotonic() - tap_ts
                    if age < 1.5:
                        # Translate tap to scaled pixmap space
                        pad_x = (self.width() - disp_w) / 2.0
                        pad_y = (self.height() - disp_h) / 2.0
                        pix_x = tap_x - pad_x
                        pix_y = tap_y - pad_y

                        if 0 <= pix_x <= disp_w and 0 <= pix_y <= disp_h:
                            alpha = int(max(0.0, 1.0 - (age / 1.5)) * 255)
                            reticle_pen = QPen(QColor(0, 229, 255, alpha), 2)
                            painter.setPen(reticle_pen)
                            r = 15 + int(age * 20)
                            painter.drawEllipse(int(pix_x - r), int(pix_y - r), r * 2, r * 2)
                            painter.drawLine(
                                int(pix_x - r - 5), int(pix_y), int(pix_x + r + 5), int(pix_y)
                            )
                            painter.drawLine(
                                int(pix_x), int(pix_y - r - 5), int(pix_x), int(pix_y + r + 5)
                            )

                painter.end()
                self.setPixmap(scaled_pixmap)

        self._bridge = _CanvasBridge()
        self._widget = _CanvasWidget()
        self._bridge.frame_received.connect(self._widget.on_frame)
        self._bridge.telemetry_received.connect(lambda _m: self._widget.update())
        return self._widget

    def update_frame(self, frame: np.ndarray) -> None:
        """Update current displayed frame (thread-safe)."""
        self._current_frame = frame
        if self._bridge is not None:
            self._bridge.frame_received.emit(frame)

    def update_telemetry(self, telemetry: Any) -> None:
        """Update HUD telemetry overlay (thread-safe)."""
        self._last_telemetry = telemetry
        if self._bridge is not None:
            self._bridge.telemetry_received.emit(telemetry)
