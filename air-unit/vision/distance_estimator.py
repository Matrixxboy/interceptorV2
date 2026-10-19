"""Distance estimator — pinhole model + EMA smoothing (§8, T1.3).

d = f_px * H_real / h_px

With EMA smoothing, jump rejection, and edge detection.
Pure functions for math (§2.7) — easy to unit test.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from shared.constants import DISTANCE_EMA_ALPHA, DISTANCE_MAX_JUMP_PCT


@dataclass
class DistanceEstimatorConfig:
    """Configuration for distance estimation."""

    focal_length_px: float = 600.0  # from camera_intrinsics.yaml
    ema_alpha: float = DISTANCE_EMA_ALPHA
    max_jump_pct: float = DISTANCE_MAX_JUMP_PCT

    # Per-class real-world height priors (meters)
    # Key: COCO class ID, Value: typical height in meters
    class_heights: dict[int, float] = field(
        default_factory=lambda: {
            0: 1.7,  # person
            1: 0.5,  # bicycle
            2: 1.5,  # car
            3: 1.8,  # motorcycle
            5: 3.0,  # bus
            7: 3.5,  # truck
        }
    )


@dataclass
class DistanceEstimator:
    """Estimates distance to target using pinhole camera model.

    distance = focal_length_px * real_height / bbox_height_px

    Applies:
    - EMA smoothing for temporal stability
    - Jump rejection (> max_jump_pct per frame → reject)
    - Edge rejection (bbox touching frame edge → None)
    """

    config: DistanceEstimatorConfig = field(default_factory=DistanceEstimatorConfig)
    _prev_distance: float | None = None

    def estimate(
        self,
        bbox_h_px: float | tuple[float, float, float, float] | list[float],
        cls: int = 0,
        frame_h: int | tuple[int, int] = 720,
        bbox_y1: float = 0.0,
        bbox_y2: float = 0.0,
    ) -> float | None:
        """Estimate distance from bounding box height or full bbox.

        Args:
            bbox_h_px: Bounding box height in pixels, or (x1, y1, x2, y2) tuple.
            cls: COCO class ID for height prior lookup.
            frame_h: Frame height in pixels, or (w, h) tuple.
            bbox_y1: Top edge of bbox in pixels.
            bbox_y2: Bottom edge of bbox in pixels.

        Returns:
            Estimated distance in meters, or None if unreliable.
        """
        frame_h_val = int(frame_h[1]) if isinstance(frame_h, (tuple, list)) else int(frame_h)

        if isinstance(bbox_h_px, (tuple, list)) and len(bbox_h_px) == 4:
            _x1, y1, _x2, y2 = bbox_h_px
            h_px = float(y2 - y1)
            b_y1 = float(y1)
            b_y2 = float(y2)
        else:
            h_px = float(bbox_h_px)
            b_y1 = bbox_y1
            b_y2 = bbox_y2

        # Reject if bbox touches frame edge (unreliable)
        if _bbox_touches_edge(b_y1, b_y2, frame_h_val):
            return None

        # Reject tiny bboxes
        if h_px < 5.0:
            return None

        # Look up real height for this class
        real_height = self.config.class_heights.get(cls)
        if real_height is None:
            return None

        # Pinhole distance
        raw_distance = pinhole_distance(self.config.focal_length_px, real_height, h_px)

        if not math.isfinite(raw_distance) or raw_distance <= 0:
            return None

        # Jump rejection
        if self._prev_distance is not None:
            jump_pct = abs(raw_distance - self._prev_distance) / self._prev_distance
            if jump_pct > self.config.max_jump_pct:
                # Reject the jump — keep previous estimate
                return self._prev_distance

        # EMA smoothing
        if self._prev_distance is not None:
            smoothed = ema(raw_distance, self._prev_distance, self.config.ema_alpha)
        else:
            smoothed = raw_distance

        self._prev_distance = smoothed
        return smoothed

    def reset(self) -> None:
        """Reset the estimator (call on target change or state transition)."""
        self._prev_distance = None


# ── Pure functions ───────────────────────────


def pinhole_distance(focal_length_px: float, real_height_m: float, bbox_height_px: float) -> float:
    """Compute distance using the pinhole camera model.

    d = f_px * H_real / h_px

    Args:
        focal_length_px: Focal length in pixels.
        real_height_m: Real-world height of the object in meters.
        bbox_height_px: Bounding box height in pixels.

    Returns:
        Estimated distance in meters.
    """
    if bbox_height_px <= 0:
        return float("inf")
    return focal_length_px * real_height_m / bbox_height_px


def ema(current: float, previous: float, alpha: float) -> float:
    """Exponential moving average.

    Args:
        current: Current raw measurement.
        previous: Previous smoothed value.
        alpha: Smoothing factor (0 = frozen, 1 = no smoothing).

    Returns:
        Smoothed value.
    """
    return alpha * current + (1.0 - alpha) * previous


def _bbox_touches_edge(y1: float, y2: float, frame_h: int, margin: float = 2.0) -> bool:
    """Check if bounding box touches the top or bottom frame edge.

    Args:
        y1: Top edge of bbox in pixels.
        y2: Bottom edge of bbox in pixels.
        frame_h: Frame height in pixels.
        margin: Pixel margin for edge detection.

    Returns:
        True if the bbox is within margin of the frame edge.
    """
    return y1 <= margin or y2 >= (frame_h - margin)
