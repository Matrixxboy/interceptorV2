"""Tests for vision/distance_estimator.py — T1.3 acceptance.

- Known bbox → known distance within 2%
- EMA smoothing
- Jump rejection
- Edge rejection
"""

from __future__ import annotations

import pytest
from air_unit.vision.distance_estimator import (
    DistanceEstimator,
    DistanceEstimatorConfig,
    ema,
    pinhole_distance,
)


class TestPinholeDistance:
    """Pure pinhole_distance function tests (T1.3 acceptance)."""

    def test_known_distance(self) -> None:
        """Known bbox height → known distance within 2%."""
        # Person 1.7m tall, focal length 600px, at 6m → bbox_h = 600*1.7/6 = 170px
        expected_distance = 6.0
        bbox_h = 600.0 * 1.7 / expected_distance  # = 170.0 px
        d = pinhole_distance(focal_length_px=600.0, real_height_m=1.7, bbox_height_px=bbox_h)
        assert d == pytest.approx(expected_distance, rel=0.02)

    def test_closer_target(self) -> None:
        """Closer target → larger bbox → smaller distance."""
        d_near = pinhole_distance(600.0, 1.7, 340.0)  # big bbox
        d_far = pinhole_distance(600.0, 1.7, 85.0)  # small bbox
        assert d_near < d_far

    def test_zero_bbox_height(self) -> None:
        d = pinhole_distance(600.0, 1.7, 0.0)
        assert d == float("inf")

    def test_negative_bbox_height(self) -> None:
        d = pinhole_distance(600.0, 1.7, -10.0)
        assert d == float("inf")


class TestEma:
    """EMA smoothing function."""

    def test_alpha_1_no_smoothing(self) -> None:
        assert ema(10.0, 5.0, alpha=1.0) == pytest.approx(10.0)

    def test_alpha_0_frozen(self) -> None:
        assert ema(10.0, 5.0, alpha=0.0) == pytest.approx(5.0)

    def test_alpha_half(self) -> None:
        assert ema(10.0, 5.0, alpha=0.5) == pytest.approx(7.5)


class TestDistanceEstimator:
    """Full estimator with smoothing, jump rejection, edge rejection."""

    def _make_estimator(self, **kwargs: object) -> DistanceEstimator:
        config = DistanceEstimatorConfig(
            focal_length_px=600.0,
            ema_alpha=1.0,  # no smoothing by default for easier testing
            max_jump_pct=0.5,
            **kwargs,  # type: ignore[arg-type]
        )
        return DistanceEstimator(config=config)

    def test_known_distance_within_2pct(self) -> None:
        """T1.3 acceptance: known bbox → known distance within 2%."""
        est = self._make_estimator()
        # Person at 8m: bbox_h = 600 * 1.7 / 8 = 127.5
        d = est.estimate(bbox_h_px=127.5, cls=0, frame_h=720, bbox_y1=200, bbox_y2=327.5)
        assert d is not None
        assert d == pytest.approx(8.0, rel=0.02)

    def test_unknown_class_returns_none(self) -> None:
        est = self._make_estimator()
        d = est.estimate(bbox_h_px=100, cls=999, frame_h=720, bbox_y1=100, bbox_y2=200)
        assert d is None

    def test_edge_rejection_top(self) -> None:
        est = self._make_estimator()
        d = est.estimate(bbox_h_px=100, cls=0, frame_h=720, bbox_y1=0, bbox_y2=100)
        assert d is None

    def test_edge_rejection_bottom(self) -> None:
        est = self._make_estimator()
        d = est.estimate(bbox_h_px=100, cls=0, frame_h=720, bbox_y1=619, bbox_y2=719)
        assert d is None

    def test_tiny_bbox_returns_none(self) -> None:
        est = self._make_estimator()
        d = est.estimate(bbox_h_px=2.0, cls=0, frame_h=720, bbox_y1=100, bbox_y2=102)
        assert d is None

    def test_jump_rejection(self) -> None:
        est = self._make_estimator()
        # First measurement at ~6m
        d1 = est.estimate(bbox_h_px=170, cls=0, frame_h=720, bbox_y1=100, bbox_y2=270)
        assert d1 is not None
        # Second measurement jumps to ~30m (>50% change) → rejected, keeps d1
        d2 = est.estimate(bbox_h_px=34, cls=0, frame_h=720, bbox_y1=100, bbox_y2=134)
        assert d2 == pytest.approx(d1)

    def test_ema_smoothing(self) -> None:
        est = DistanceEstimator(
            config=DistanceEstimatorConfig(
                focal_length_px=600.0,
                ema_alpha=0.5,
                max_jump_pct=1.0,  # disable jump rejection
            )
        )
        d1 = est.estimate(bbox_h_px=170, cls=0, frame_h=720, bbox_y1=100, bbox_y2=270)
        assert d1 is not None
        d2 = est.estimate(bbox_h_px=127.5, cls=0, frame_h=720, bbox_y1=100, bbox_y2=227.5)
        # d2 should be smoothed between d1 and raw d2
        assert d2 is not None
        assert d2 != pytest.approx(8.0)  # not raw value
        assert d2 != pytest.approx(d1)  # not previous value

    def test_reset(self) -> None:
        est = self._make_estimator()
        est.estimate(bbox_h_px=170, cls=0, frame_h=720, bbox_y1=100, bbox_y2=270)
        est.reset()
        # After reset, estimator has no previous — any value accepted
        d = est.estimate(bbox_h_px=34, cls=0, frame_h=720, bbox_y1=100, bbox_y2=134)
        assert d is not None
