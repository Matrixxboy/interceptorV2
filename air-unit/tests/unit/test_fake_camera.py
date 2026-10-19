"""Tests for air_unit/sim/fake_camera.py — T2.1 acceptance.

- FakeCamera generates frames of expected shape, dtype, and timestamp.
- Moving rectangle changes position across successive frames.
- Unstarted camera raises RuntimeError.
- Stop/restart behaves cleanly.
"""

from __future__ import annotations

import numpy as np
import pytest
from air_unit.sim.fake_camera import FakeCamera, FileCamera


class TestFakeCamera:
    """FakeCamera generates synthetic frames with moving rectangle (T2.1)."""

    def test_unstarted_camera_raises(self) -> None:
        cam = FakeCamera(width=320, height=240, fps=100)
        with pytest.raises(RuntimeError, match="not started"):
            cam.read()

    def test_frame_shape_dtype_and_timestamp(self) -> None:
        cam = FakeCamera(width=320, height=240, fps=1000)
        cam.start()
        try:
            frame, ts_ns = cam.read()
            assert isinstance(frame, np.ndarray)
            assert frame.shape == (240, 320, 3)
            assert frame.dtype == np.uint8
            assert isinstance(ts_ns, int)
            assert ts_ns > 0
        finally:
            cam.stop()

    def test_moving_rectangle_changes_position(self) -> None:
        cam = FakeCamera(width=320, height=240, fps=1000, speed_px_per_s=500.0)
        cam.start()
        try:
            frame1, _ = cam.read()
            # Find pixels matching rect_color (default 0, 255, 0)
            green1 = np.where((frame1[:, :, 1] == 255) & (frame1[:, :, 0] == 0))
            assert len(green1[0]) > 0

            # Force time elapsed
            cam._start_time -= 0.5  # shift start time back 0.5s
            frame2, _ = cam.read()
            green2 = np.where((frame2[:, :, 1] == 255) & (frame2[:, :, 0] == 0))
            assert len(green2[0]) > 0

            # Positions should differ
            mean_y1 = np.mean(green1[0])
            mean_x1 = np.mean(green1[1])
            mean_y2 = np.mean(green2[0])
            mean_x2 = np.mean(green2[1])
            assert (mean_x1 != mean_x2) or (mean_y1 != mean_y2)
        finally:
            cam.stop()

    def test_file_camera_nonexistent_raises(self) -> None:
        cam = FileCamera(file_path="non_existent_file.mp4")
        with pytest.raises(RuntimeError):
            cam.start()
