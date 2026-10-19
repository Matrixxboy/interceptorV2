"""Tests for air_unit/vision/tracker.py — T2.3 acceptance.

Acceptance: IDs stable across a sequence of frames; ID switch rate is reported and 0 for continuous trajectory.
"""

from __future__ import annotations

from air_unit.vision.interfaces import Detection
from air_unit.vision.tracker import ByteTracker, TrackerConfig


class TestByteTracker:
    """ByteTracker tests (T2.3)."""

    def test_id_stability_moving_target(self) -> None:
        """Single target moving smoothly maintains the exact same track ID."""
        tracker = ByteTracker(TrackerConfig(min_hits_to_activate=1))
        tracker.reset()

        track_ids: list[int] = []
        # Target moves diagonally from (100, 100) to (200, 200) across 20 frames
        for i in range(20):
            x = 100.0 + i * 5.0
            y = 100.0 + i * 5.0
            det = Detection(x1=x, y1=y, x2=x + 40.0, y2=y + 80.0, score=0.9, cls=0)
            tracks = tracker.update([det])
            assert len(tracks) == 1
            track_ids.append(tracks[0].id)

        # ID switch rate check: all frames have the same ID
        assert len(set(track_ids)) == 1
        assert track_ids[0] == 1

    def test_multiple_targets_distinct_ids(self) -> None:
        """Two separated targets receive and maintain distinct IDs."""
        tracker = ByteTracker(TrackerConfig(min_hits_to_activate=1))
        tracker.reset()

        for _ in range(5):
            det1 = Detection(x1=50.0, y1=50.0, x2=90.0, y2=130.0, score=0.95, cls=0)
            det2 = Detection(x1=300.0, y1=300.0, x2=340.0, y2=380.0, score=0.92, cls=0)
            tracks = tracker.update([det1, det2])
            assert len(tracks) == 2
            ids = {t.id for t in tracks}
            assert len(ids) == 2

    def test_brief_occlusion_recovery(self) -> None:
        """Target disappears for 2 frames then reappears at predicted location."""
        tracker = ByteTracker(TrackerConfig(min_hits_to_activate=1, max_time_lost=5))
        tracker.reset()

        # Track for 5 frames
        for i in range(5):
            x = 100.0 + i * 2.0
            det = Detection(x1=x, y1=100.0, x2=x + 40.0, y2=180.0, score=0.9, cls=0)
            tracks = tracker.update([det])
            orig_id = tracks[0].id

        # Occlusion: 2 frames with no detections
        tracker.update([])
        tracker.update([])

        # Reappears
        reappear_x = 100.0 + 7 * 2.0
        reappear_det = Detection(
            x1=reappear_x, y1=100.0, x2=reappear_x + 40.0, y2=180.0, score=0.88, cls=0
        )
        tracks = tracker.update([reappear_det])

        # Track should be recovered with original ID
        assert len(tracks) == 1
        assert tracks[0].id == orig_id
