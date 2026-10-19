"""FakeCamera — synthetic moving rectangle for testing (T2.1).

Generates frames with a moving colored rectangle for end-to-end testing
without any hardware. Implements CameraSource ABC.
"""

from __future__ import annotations

import math
import time

import numpy as np
from air_unit.hardware.interfaces import CameraSource


class FakeCamera(CameraSource):
    """Synthetic camera that renders a moving rectangle.

    The rectangle bounces around the frame for tracker testing.
    No hardware imports needed.
    """

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        rect_width: int = 80,
        rect_height: int = 160,
        rect_color: tuple[int, int, int] = (0, 255, 0),
        speed_px_per_s: float = 100.0,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.rect_w = rect_width
        self.rect_h = rect_height
        self.rect_color = rect_color
        self.speed = speed_px_per_s
        self._running = False
        self._frame_count = 0
        self._start_time: float = 0.0

    def start(self) -> None:
        """Start the fake camera."""
        self._running = True
        self._frame_count = 0
        self._start_time = time.monotonic()

    def read(self) -> tuple[np.ndarray, int]:
        """Generate a frame with a moving rectangle.

        Returns:
            Tuple of (frame as HWC uint8 ndarray, capture_ts_ns).
        """
        if not self._running:
            raise RuntimeError("FakeCamera not started")

        ts_ns = time.monotonic_ns()
        elapsed = time.monotonic() - self._start_time

        # Compute rectangle position (bouncing)
        x, y = self._compute_position(elapsed)

        # Create frame
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Draw rectangle
        x1 = max(0, int(x))
        y1 = max(0, int(y))
        x2 = min(self.width, int(x + self.rect_w))
        y2 = min(self.height, int(y + self.rect_h))
        frame[y1:y2, x1:x2] = self.rect_color

        self._frame_count += 1

        # Pace to fps
        expected_time = self._frame_count / self.fps
        sleep_time = expected_time - (time.monotonic() - self._start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

        return frame, ts_ns

    def stop(self) -> None:
        """Stop the fake camera."""
        self._running = False

    def _compute_position(self, elapsed: float) -> tuple[float, float]:
        """Compute bouncing rectangle position.

        Uses sinusoidal motion for smooth, predictable movement.
        """
        max_x = self.width - self.rect_w
        max_y = self.height - self.rect_h

        # Sinusoidal bouncing
        period_x = max_x * 2.0 / self.speed
        period_y = max_y * 2.0 / self.speed * 1.3  # slightly different period

        x = (math.sin(elapsed * 2 * math.pi / period_x) + 1) / 2 * max_x
        y = (math.sin(elapsed * 2 * math.pi / period_y + 0.5) + 1) / 2 * max_y

        return x, y


class FileCamera(CameraSource):
    """Camera source that reads from a video file (T2.1).

    Uses OpenCV to read frames from a video file.
    Import is lazy (inside __init__) per §2.3.
    """

    def __init__(self, file_path: str, loop: bool = True) -> None:
        self.file_path = file_path
        self.loop = loop
        self._cap: object | None = None  # cv2.VideoCapture, lazy import

    def start(self) -> None:
        """Open the video file."""
        import cv2  # lazy import (§2.3)

        self._cap = cv2.VideoCapture(self.file_path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video file: {self.file_path}")

    def read(self) -> tuple[np.ndarray, int]:
        """Read the next frame from the video file.

        Returns:
            Tuple of (frame, capture_ts_ns).
        """
        import cv2  # lazy import (§2.3)

        if self._cap is None:
            raise RuntimeError("FileCamera not started")

        ret, frame = self._cap.read()
        if not ret:
            if self.loop:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self._cap.read()
                if not ret:
                    raise RuntimeError("Failed to loop video")
            else:
                raise StopIteration("End of video file")

        ts_ns = time.monotonic_ns()
        return frame, ts_ns

    def stop(self) -> None:
        """Release the video capture."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
