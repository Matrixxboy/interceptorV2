"""Tests for ground_station/ui/video_canvas.py — T4.3 acceptance.

Acceptance:
- map_canvas_click_to_video correctly handles letterboxing (top/bottom bars)
- map_canvas_click_to_video correctly handles pillarboxing (left/right bars)
- Click inside video area maps to precise [0, 1] normalized coordinates
- Click in black bar padding returns None
"""

from __future__ import annotations

import pytest
from ground_station.ui.video_canvas import map_canvas_click_to_video


class TestVideoCanvasMapping:
    """T4.3 coordinate mapping unit tests."""

    def test_exact_aspect_ratio_match(self) -> None:
        """When widget and video have same aspect ratio (no letterboxing)."""
        # Widget 640x480, Video 640x480
        # Click at center
        res = map_canvas_click_to_video(320, 240, 640, 480, 640, 480)
        assert res is not None
        assert res[0] == pytest.approx(0.5)
        assert res[1] == pytest.approx(0.5)

        # Click at top-left
        res_tl = map_canvas_click_to_video(0, 0, 640, 480, 640, 480)
        assert res_tl == (0.0, 0.0)

        # Click at bottom-right
        res_br = map_canvas_click_to_video(640, 480, 640, 480, 640, 480)
        assert res_br == (1.0, 1.0)

    def test_pillarboxed_mapping(self) -> None:
        """Wide widget with 4:3 video (pillarboxed: black bars on left and right)."""
        # Widget: 1000 x 480 (aspect ~2.08)
        # Video: 640 x 480 (aspect 1.33)
        # Displayed video height = 480, displayed width = 480 * 1.333 = 640
        # Pad X = (1000 - 640) / 2 = 180 on each side
        # Video occupies x: 180 .. 820

        # Click in left padding (x = 50) -> None
        assert map_canvas_click_to_video(50, 240, 1000, 480, 640, 480) is None

        # Click in right padding (x = 900) -> None
        assert map_canvas_click_to_video(900, 240, 1000, 480, 640, 480) is None

        # Click at left edge of video (x = 180, y = 240) -> (0.0, 0.5)
        res_left = map_canvas_click_to_video(180, 240, 1000, 480, 640, 480)
        assert res_left is not None
        assert res_left[0] == pytest.approx(0.0)
        assert res_left[1] == pytest.approx(0.5)

        # Click at center of video (x = 500, y = 240) -> (0.5, 0.5)
        res_center = map_canvas_click_to_video(500, 240, 1000, 480, 640, 480)
        assert res_center is not None
        assert res_center[0] == pytest.approx(0.5)
        assert res_center[1] == pytest.approx(0.5)

    def test_letterboxed_mapping(self) -> None:
        """Tall widget with 16:9 video (letterboxed: black bars on top and bottom)."""
        # Widget: 1280 x 1000 (aspect 1.28)
        # Video: 1280 x 720 (aspect 1.777)
        # Displayed width = 1280, displayed height = 1280 / 1.777 = 720
        # Pad Y = (1000 - 720) / 2 = 140 on top and bottom
        # Video occupies y: 140 .. 860

        # Click in top padding (y = 50) -> None
        assert map_canvas_click_to_video(640, 50, 1280, 1000, 1280, 720) is None

        # Click in bottom padding (y = 950) -> None
        assert map_canvas_click_to_video(640, 950, 1280, 1000, 1280, 720) is None

        # Click at center of video (x = 640, y = 500) -> (0.5, 0.5)
        res_center = map_canvas_click_to_video(640, 500, 1280, 1000, 1280, 720)
        assert res_center is not None
        assert res_center[0] == pytest.approx(0.5)
        assert res_center[1] == pytest.approx(0.5)

    def test_invalid_dimensions_return_none(self) -> None:
        assert map_canvas_click_to_video(10, 10, 0, 100, 100, 100) is None
        assert map_canvas_click_to_video(10, 10, 100, 0, 100, 100) is None
