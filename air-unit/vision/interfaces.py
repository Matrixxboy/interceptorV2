"""Vision interfaces — Detector ABC and Detection dataclass (§7.1).

No hardware imports at top level (§2.3).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    """Single detection from a frame (§7.1).

    Coordinates are in pixels relative to the input frame.
    """

    x1: float  # left
    y1: float  # top
    x2: float  # right
    y2: float  # bottom
    score: float
    cls: int

    @property
    def width(self) -> float:
        """Bounding box width in pixels."""
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        """Bounding box height in pixels."""
        return self.y2 - self.y1

    @property
    def cx(self) -> float:
        """Center X in pixels."""
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        """Center Y in pixels."""
        return (self.y1 + self.y2) / 2.0

    @property
    def area(self) -> float:
        """Bounding box area in pixels²."""
        return self.width * self.height


@dataclass(frozen=True)
class Track:
    """Tracked object with persistent ID (§7.1)."""

    id: int
    bbox: tuple[float, float, float, float]  # (x1, y1, x2, y2) pixels
    score: float
    age: int  # number of frames since first detection

    @property
    def cx(self) -> float:
        """Center X in pixels."""
        return (self.bbox[0] + self.bbox[2]) / 2.0

    @property
    def cy(self) -> float:
        """Center Y in pixels."""
        return (self.bbox[1] + self.bbox[3]) / 2.0

    @property
    def width(self) -> float:
        """Bounding box width in pixels."""
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> float:
        """Bounding box height in pixels."""
        return self.bbox[3] - self.bbox[1]


@dataclass(frozen=True)
class TrackState:
    """Composite tracking state passed between threads via LatestSlot (§7.1)."""

    capture_ts_ns: int
    frame_w: int
    frame_h: int
    target: Track | None
    distance_m: float | None
    latency_ms: float


class Detector(ABC):
    """Abstract detector interface (§7.2).

    Implementations: HailoDetector, OnnxDetector, FakeDetector.
    """

    @abstractmethod
    def infer(self, frame: np.ndarray) -> list[Detection]:
        """Run inference on a frame.

        Args:
            frame: Input frame as HWC uint8 ndarray.

        Returns:
            List of Detection objects.
        """
