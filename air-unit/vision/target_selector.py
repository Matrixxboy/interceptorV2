"""Target selector — tap (x_norm, y_norm) → track ID (§6, T2.4).

Maps a pilot's screen tap to the nearest track within LOCK_RADIUS_NORM.
Re-acquire logic for lost targets.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from shared.constants import LOCK_RADIUS_NORM

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LockInfo:
    """Information about a locked target for re-acquire."""

    track_id: int
    last_bbox: tuple[float, float, float, float]  # normalized (x1, y1, x2, y2)
    last_velocity: tuple[float, float]  # (vx_norm, vy_norm) per second
    cls: int
    lost_time_s: float = 0.0  # how long target has been lost


@dataclass
class TargetSelector:
    """Maps tap coordinates to track IDs and handles re-acquire.

    Tap coordinates are normalized to [0, 1] (from GCS video canvas).
    Tracks must have their bboxes in pixel coordinates; we normalize using frame dims.
    """

    lock_radius_norm: float = LOCK_RADIUS_NORM
    _locked: LockInfo | None = None
    _pending_tap: tuple[float, float] | None = None

    def tap(self, x_norm: float, y_norm: float) -> None:
        """Store pending tap coordinates for selection."""
        self._pending_tap = (x_norm, y_norm)

    def select_target(
        self,
        tracks: Sequence[Any],
        frame_size: tuple[int, int],
    ) -> Any | None:
        """Select a target if tap is pending."""
        if self._pending_tap is None:
            return None

        tap_x, tap_y = self._pending_tap
        w, h = frame_size

        # Convert Track objects or tuples
        converted: list[tuple[int, tuple[float, float, float, float], float, int]] = []
        track_map: dict[int, Any] = {}
        for t in tracks:
            if hasattr(t, "id") and hasattr(t, "bbox"):
                converted.append((t.id, t.bbox, getattr(t, "score", 1.0), getattr(t, "cls", 0)))
                track_map[t.id] = t
            elif isinstance(t, tuple) and len(t) == 4:
                converted.append(t)
                track_map[t[0]] = t

        selected_id = self.select_from_tap(tap_x, tap_y, converted, w, h)
        if selected_id is not None:
            self._pending_tap = None
            return track_map.get(selected_id)

        return None

    def reacquire(
        self,
        tracks: Sequence[Any],
        frame_size: tuple[int, int],
        time_lost_s: float = 0.0,
    ) -> Any | None:
        """Re-acquire a locked target from current tracks."""
        w, h = frame_size
        converted: list[tuple[int, tuple[float, float, float, float], float, int]] = []
        track_map: dict[int, Any] = {}
        for t in tracks:
            if hasattr(t, "id") and hasattr(t, "bbox"):
                converted.append((t.id, t.bbox, getattr(t, "score", 1.0), getattr(t, "cls", 0)))
                track_map[t.id] = t
            elif isinstance(t, tuple) and len(t) == 4:
                converted.append(t)
                track_map[t[0]] = t

        reacquired_id = self.try_reacquire(converted, w, h, time_lost_s)
        if reacquired_id is not None:
            return track_map.get(reacquired_id)
        return None

    def select_from_tap(
        self,
        x_norm: float,
        y_norm: float,
        tracks: list[tuple[int, tuple[float, float, float, float], float, int]],
        frame_w: int,
        frame_h: int,
    ) -> int | None:
        """Select a track by tap position.

        Args:
            x_norm: Normalized tap X [0, 1].
            y_norm: Normalized tap Y [0, 1].
            tracks: List of (track_id, bbox_px, score, cls) tuples.
            frame_w: Frame width in pixels.
            frame_h: Frame height in pixels.

        Returns:
            Track ID of the selected track, or None if no track within radius.
        """
        best_id: int | None = None
        best_dist = float("inf")

        for track_id, bbox, _score, cls in tracks:
            # Normalize bbox center and boundaries
            cx = ((bbox[0] + bbox[2]) / 2.0) / frame_w
            cy = ((bbox[1] + bbox[3]) / 2.0) / frame_h

            x1_norm = bbox[0] / frame_w
            y1_norm = bbox[1] / frame_h
            x2_norm = bbox[2] / frame_w
            y2_norm = bbox[3] / frame_h

            dist = math.hypot(cx - x_norm, cy - y_norm)
            is_inside = (x1_norm <= x_norm <= x2_norm) and (y1_norm <= y_norm <= y2_norm)

            if is_inside or dist < self.lock_radius_norm:
                effective_dist = 0.0 if is_inside else dist
                if effective_dist < best_dist:
                    best_dist = effective_dist
                    best_id = track_id
                    # Store lock info for re-acquire
                    bbox_norm = (x1_norm, y1_norm, x2_norm, y2_norm)
                    self._locked = LockInfo(
                        track_id=track_id,
                        last_bbox=bbox_norm,
                        last_velocity=(0.0, 0.0),
                        cls=cls,
                    )

        if best_id is not None:
            logger.info("Target selected: track_id=%d (dist=%.3f)", best_id, best_dist)
        else:
            logger.info(
                "No track within radius %.3f of tap (%.2f, %.2f)",
                self.lock_radius_norm,
                x_norm,
                y_norm,
            )

        return best_id

    def try_reacquire(
        self,
        tracks: list[tuple[int, tuple[float, float, float, float], float, int]],
        frame_w: int,
        frame_h: int,
        time_lost_s: float,
    ) -> int | None:
        """Try to re-acquire a lost target from current tracks.

        Uses ID match first, then position/class gate with widening window.

        Args:
            tracks: Current tracks.
            frame_w: Frame width.
            frame_h: Frame height.
            time_lost_s: How long the target has been lost.

        Returns:
            Re-acquired track ID, or None.
        """
        if self._locked is None:
            return None

        # First: try exact ID match
        for track_id, bbox, _score, cls in tracks:
            if track_id == self._locked.track_id:
                logger.info("Re-acquired target by ID: %d", track_id)
                bbox_norm = (
                    bbox[0] / frame_w,
                    bbox[1] / frame_h,
                    bbox[2] / frame_w,
                    bbox[3] / frame_h,
                )
                self._locked = LockInfo(
                    track_id=track_id,
                    last_bbox=bbox_norm,
                    last_velocity=self._locked.last_velocity,
                    cls=cls,
                )
                return track_id

        # Second: position + class gate with widening window (§8)
        gate_radius = 0.1 + 0.05 * time_lost_s  # widen with time
        last_cx = (self._locked.last_bbox[0] + self._locked.last_bbox[2]) / 2.0
        last_cy = (self._locked.last_bbox[1] + self._locked.last_bbox[3]) / 2.0

        best_id: int | None = None
        best_dist = float("inf")

        for track_id, bbox, _score, cls in tracks:
            # Class must match
            if cls != self._locked.cls:
                continue

            cx = ((bbox[0] + bbox[2]) / 2.0) / frame_w
            cy = ((bbox[1] + bbox[3]) / 2.0) / frame_h
            dist = math.hypot(cx - last_cx, cy - last_cy)

            if dist < gate_radius and dist < best_dist:
                best_dist = dist
                best_id = track_id

        if best_id is not None:
            logger.info(
                "Re-acquired target by gate: track_id=%d (dist=%.3f, gate=%.3f)",
                best_id,
                best_dist,
                gate_radius,
            )

        return best_id

    @property
    def locked_id(self) -> int | None:
        """Currently locked track ID, or None."""
        return self._locked.track_id if self._locked else None

    def unlock(self) -> None:
        """Clear the lock."""
        self._locked = None
        logger.info("Target unlocked")
