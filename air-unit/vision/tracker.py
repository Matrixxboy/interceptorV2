"""ByteTrack implementation / wrapper (§7.1, §8, T2.3).

Associates detections across frames using two-stage IoU association.
Tracks objects with persistent IDs, bounding boxes, scores, and ages.
"""

from __future__ import annotations

import enum
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from air_unit.vision.interfaces import Detection, Track
from scipy.optimize import linear_sum_assignment


def compute_iou_matrix(
    boxes_a: np.ndarray,
    boxes_b: np.ndarray,
) -> np.ndarray:
    """Compute pairwise IoU between two sets of boxes (x1, y1, x2, y2).

    Args:
        boxes_a: (N, 4) ndarray.
        boxes_b: (M, 4) ndarray.

    Returns:
        (N, M) ndarray of IoU values in [0, 1].
    """
    if len(boxes_a) == 0 or len(boxes_b) == 0:
        return np.zeros((len(boxes_a), len(boxes_b)), dtype=np.float32)

    x1_a, y1_a, x2_a, y2_a = boxes_a[:, 0], boxes_a[:, 1], boxes_a[:, 2], boxes_a[:, 3]
    x1_b, y1_b, x2_b, y2_b = boxes_b[:, 0], boxes_b[:, 1], boxes_b[:, 2], boxes_b[:, 3]

    area_a = np.maximum(0.0, x2_a - x1_a) * np.maximum(0.0, y2_a - y1_a)
    area_b = np.maximum(0.0, x2_b - x1_b) * np.maximum(0.0, y2_b - y1_b)

    xx1 = np.maximum(x1_a[:, None], x1_b[None, :])
    yy1 = np.maximum(y1_a[:, None], y1_b[None, :])
    xx2 = np.minimum(x2_a[:, None], x2_b[None, :])
    yy2 = np.minimum(y2_a[:, None], y2_b[None, :])

    w = np.maximum(0.0, xx2 - xx1)
    h = np.maximum(0.0, yy2 - yy1)
    inter = w * h

    union = area_a[:, None] + area_b[None, :] - inter
    union = np.maximum(union, 1e-6)
    return inter / union


class TrackStatus(enum.Enum):
    """Internal track lifecycle state."""

    NEW = "NEW"
    TRACKED = "TRACKED"
    LOST = "LOST"
    REMOVED = "REMOVED"


class STrack:
    """Single tracked object."""

    _id_counter: int = 1

    @classmethod
    def reset_id(cls) -> None:
        """Reset the global ID counter."""
        cls._id_counter = 1

    def __init__(self, detection: Detection) -> None:
        self.track_id = STrack._id_counter
        STrack._id_counter += 1

        self.bbox: tuple[float, float, float, float] = (
            detection.x1,
            detection.y1,
            detection.x2,
            detection.y2,
        )
        self.score = detection.score
        self.cls = detection.cls
        self.status = TrackStatus.TRACKED
        self.age = 1
        self.hits = 1
        self.time_since_update = 0

        # Velocity in pixels/frame (vx, vy)
        self.vx = 0.0
        self.vy = 0.0

    @property
    def cx(self) -> float:
        return (self.bbox[0] + self.bbox[2]) / 2.0

    @property
    def cy(self) -> float:
        return (self.bbox[1] + self.bbox[3]) / 2.0

    def predict(self) -> None:
        """Predict forward by 1 frame using simple constant velocity."""
        x1, y1, x2, y2 = self.bbox
        self.bbox = (x1 + self.vx, y1 + self.vy, x2 + self.vx, y2 + self.vy)
        self.age += 1
        self.time_since_update += 1

    def update(self, detection: Detection) -> None:
        """Update track with a newly matched detection."""
        old_cx = self.cx
        old_cy = self.cy

        self.bbox = (
            detection.x1,
            detection.y1,
            detection.x2,
            detection.y2,
        )
        self.score = detection.score
        self.cls = detection.cls
        self.status = TrackStatus.TRACKED
        self.hits += 1
        self.time_since_update = 0

        # Update smoothed velocity (alpha = 0.5)
        new_vx = self.cx - old_cx
        new_vy = self.cy - old_cy
        self.vx = 0.5 * self.vx + 0.5 * new_vx
        self.vy = 0.5 * self.vy + 0.5 * new_vy

    def mark_lost(self) -> None:
        """Mark track as lost."""
        self.status = TrackStatus.LOST

    def mark_removed(self) -> None:
        """Mark track as permanently removed."""
        self.status = TrackStatus.REMOVED


@dataclass
class TrackerConfig:
    """ByteTrack configuration parameters."""

    high_thresh: float = 0.5
    low_thresh: float = 0.1
    match_thresh: float = 0.7  # minimum IoU cost = 1 - 0.7 = 0.3
    max_time_lost: int = 30  # frames before removing lost track
    min_hits_to_activate: int = 1


class ByteTracker:
    """ByteTrack multi-object tracker (T2.3).

    Performs two-stage association:
    1. High-confidence detections with existing tracks.
    2. Low-confidence detections with remaining tracks.
    """

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self.tracked_stracks: list[STrack] = []
        self.lost_stracks: list[STrack] = []
        self.removed_stracks: list[STrack] = []
        self.frame_id = 0

    def reset(self) -> None:
        """Reset tracker state."""
        STrack.reset_id()
        self.tracked_stracks.clear()
        self.lost_stracks.clear()
        self.removed_stracks.clear()
        self.frame_id = 0

    def update(self, detections: Sequence[Detection]) -> list[Track]:
        """Update tracks with detections from current frame.

        Args:
            detections: List of Detection objects in pixel coordinates.

        Returns:
            List of active confirmed Track objects.
        """
        self.frame_id += 1

        # ── 1. Separate detections by confidence score ──
        det_high: list[Detection] = []
        det_low: list[Detection] = []
        for d in detections:
            if d.score >= self.config.high_thresh:
                det_high.append(d)
            elif d.score >= self.config.low_thresh:
                det_low.append(d)

        # ── 2. Predict forward all existing tracks ──
        pool = self.tracked_stracks + self.lost_stracks
        for t in pool:
            t.predict()

        # ── 3. First association: Tracked tracks with High detections ──
        active_tracks = [t for t in pool if t.status != TrackStatus.REMOVED]
        matched_tracks_1, unmatched_tracks_1, unmatched_dets_1 = self._associate(
            active_tracks, det_high, self.config.match_thresh
        )

        for track, det in matched_tracks_1:
            track.update(det)

        # ── 4. Second association: Remaining tracks with Low detections ──
        remaining_tracks = [
            active_tracks[i]
            for i in unmatched_tracks_1
            if active_tracks[i].status == TrackStatus.TRACKED
        ]
        matched_tracks_2, unmatched_tracks_2, _ = self._associate(
            remaining_tracks, det_low, match_thresh=0.5
        )

        for track, det in matched_tracks_2:
            track.update(det)

        # Tracks that were not matched in second pass become LOST
        for idx in unmatched_tracks_2:
            track = remaining_tracks[idx]
            if track.status != TrackStatus.LOST:
                track.mark_lost()

        # ── 5. Unmatched high detections become NEW tracks ──
        for det_idx in unmatched_dets_1:
            new_track = STrack(det_high[det_idx])
            # If hits threshold is 1, activate immediately
            if self.config.min_hits_to_activate <= 1:
                new_track.status = TrackStatus.TRACKED
            self.tracked_stracks.append(new_track)

        # ── 6. Clean up pool and manage lifecycles ──
        new_tracked: list[STrack] = []
        new_lost: list[STrack] = []

        for t in pool + [t for t in self.tracked_stracks if t not in pool]:
            if t.status == TrackStatus.TRACKED:
                if t.hits >= self.config.min_hits_to_activate:
                    new_tracked.append(t)
                else:
                    new_tracked.append(t)
            elif t.status == TrackStatus.LOST:
                if t.time_since_update > self.config.max_time_lost:
                    t.mark_removed()
                    self.removed_stracks.append(t)
                else:
                    new_lost.append(t)

        self.tracked_stracks = new_tracked
        self.lost_stracks = new_lost

        # Return confirmed active tracks
        result: list[Track] = []
        for t in self.tracked_stracks:
            if t.status == TrackStatus.TRACKED and t.hits >= self.config.min_hits_to_activate:
                result.append(
                    Track(
                        id=t.track_id,
                        bbox=t.bbox,
                        score=t.score,
                        age=t.age,
                    )
                )

        return result

    def _associate(
        self,
        tracks: list[STrack],
        detections: list[Detection],
        match_thresh: float,
    ) -> tuple[list[tuple[STrack, Detection]], list[int], list[int]]:
        """Match tracks to detections via Hungarian algorithm on IoU cost."""
        if len(tracks) == 0:
            return [], [], list(range(len(detections)))
        if len(detections) == 0:
            return [], list(range(len(tracks))), []

        track_boxes = np.array([t.bbox for t in tracks], dtype=np.float32)
        det_boxes = np.array([[d.x1, d.y1, d.x2, d.y2] for d in detections], dtype=np.float32)

        iou_matrix = compute_iou_matrix(track_boxes, det_boxes)
        cost_matrix = 1.0 - iou_matrix

        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        matched: list[tuple[STrack, Detection]] = []
        unmatched_tracks = set(range(len(tracks)))
        unmatched_dets = set(range(len(detections)))

        for r, c in zip(row_indices, col_indices, strict=False):
            if iou_matrix[r, c] >= (1.0 - match_thresh):
                matched.append((tracks[r], detections[c]))
                unmatched_tracks.discard(r)
                unmatched_dets.discard(c)

        return matched, sorted(unmatched_tracks), sorted(unmatched_dets)
