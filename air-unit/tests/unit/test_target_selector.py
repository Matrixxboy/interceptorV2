"""Tests for vision/target_selector.py — T2.4 acceptance.

- Tap inside bbox selects it
- Tap on empty area → None (lock_failed)
- Re-acquire by ID
- Re-acquire by position/class gate
"""

from __future__ import annotations

from air_unit.vision.target_selector import TargetSelector


def _make_tracks() -> list[tuple[int, tuple[float, float, float, float], float, int]]:
    """Create test tracks: two persons at known positions."""
    return [
        # Track 1: person at center-left (bbox: 200-280, 200-400 in 1280x720)
        (1, (200.0, 200.0, 280.0, 400.0), 0.9, 0),
        # Track 2: person at center-right (bbox: 800-880, 250-450 in 1280x720)
        (2, (800.0, 250.0, 880.0, 450.0), 0.85, 0),
    ]


class TestTapSelection:
    """T2.4 acceptance: tap inside bbox selects it."""

    def test_tap_on_track_1(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        # Tap at center of track 1: (240/1280, 300/720) = (0.1875, 0.417)
        result = sel.select_from_tap(0.1875, 0.417, tracks, 1280, 720)
        assert result == 1

    def test_tap_on_track_2(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        # Tap at center of track 2: (840/1280, 350/720) = (0.656, 0.486)
        result = sel.select_from_tap(0.656, 0.486, tracks, 1280, 720)
        assert result == 2

    def test_tap_on_empty_area(self) -> None:
        """T2.4 acceptance: tap on empty area → lock_failed (None)."""
        sel = TargetSelector()
        tracks = _make_tracks()
        # Tap far from any track
        result = sel.select_from_tap(0.5, 0.9, tracks, 1280, 720)
        assert result is None

    def test_tap_selects_nearest(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        # Tap between tracks but closer to track 1
        result = sel.select_from_tap(0.25, 0.42, tracks, 1280, 720)
        assert result == 1

    def test_no_tracks_returns_none(self) -> None:
        sel = TargetSelector()
        result = sel.select_from_tap(0.5, 0.5, [], 1280, 720)
        assert result is None


class TestReacquire:
    """Re-acquire logic."""

    def test_reacquire_by_id(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        sel.select_from_tap(0.1875, 0.417, tracks, 1280, 720)  # lock track 1

        # New frame with same track ID
        new_tracks = [
            (1, (210.0, 210.0, 290.0, 410.0), 0.88, 0),
        ]
        result = sel.try_reacquire(new_tracks, 1280, 720, time_lost_s=0.5)
        assert result == 1

    def test_reacquire_by_gate(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        sel.select_from_tap(0.1875, 0.417, tracks, 1280, 720)  # lock track 1

        # New frame with different ID but same position/class
        new_tracks = [
            (99, (210.0, 210.0, 290.0, 410.0), 0.88, 0),  # same area, new ID
        ]
        result = sel.try_reacquire(new_tracks, 1280, 720, time_lost_s=0.5)
        assert result == 99

    def test_reacquire_wrong_class_rejected(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        sel.select_from_tap(0.1875, 0.417, tracks, 1280, 720)  # lock person (cls=0)

        # New frame with different class at same position
        new_tracks = [
            (99, (210.0, 210.0, 290.0, 410.0), 0.88, 2),  # car, not person
        ]
        result = sel.try_reacquire(new_tracks, 1280, 720, time_lost_s=0.5)
        assert result is None

    def test_reacquire_no_lock_returns_none(self) -> None:
        sel = TargetSelector()
        result = sel.try_reacquire([], 1280, 720, time_lost_s=0.5)
        assert result is None


class TestUnlock:
    """Unlock clears the lock."""

    def test_unlock(self) -> None:
        sel = TargetSelector()
        tracks = _make_tracks()
        sel.select_from_tap(0.1875, 0.417, tracks, 1280, 720)
        assert sel.locked_id == 1
        sel.unlock()
        assert sel.locked_id is None
