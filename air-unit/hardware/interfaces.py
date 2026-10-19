"""Hardware abstract interfaces (§7.2).

Every hardware/IO dependency is an abstract class here with Fake/Mock implementations.
Core logic must run with zero hardware (§2.2).
No hardware imports at module top-level (§2.3).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass(frozen=True)
class FcState:
    """Flight controller state snapshot."""

    mode: str = "UNKNOWN"
    armed: bool = False
    battery_v: float = 0.0
    battery_pct: float = -1.0
    alt_m: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0


class CameraSource(ABC):
    """Abstract camera source (§7.2).

    Implementations: Picamera2Camera, V4L2Camera, FileCamera, FakeCamera.
    """

    @abstractmethod
    def start(self) -> None:
        """Start the camera capture."""

    @abstractmethod
    def read(self) -> tuple[np.ndarray, int]:
        """Read a frame and its capture timestamp.

        Returns:
            Tuple of (frame as HWC uint8 ndarray, capture_ts_ns monotonic).
        """

    @abstractmethod
    def stop(self) -> None:
        """Stop the camera and release resources."""


class FlightController(ABC):
    """Abstract flight controller interface (§7.2).

    Safety rule S1: No ARM/DISARM API exists. The companion computer NEVER arms.
    """

    @abstractmethod
    def state(self) -> FcState:
        """Get current flight controller state."""

    @abstractmethod
    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """Send a velocity setpoint to the FC.

        Args:
            vx: Forward velocity m/s (body frame).
            vy: Right velocity m/s (body frame).
            vz: Down velocity m/s (body frame, positive = down).
            yaw_rate: Yaw rate rad/s (positive = clockwise).
        """

    @abstractmethod
    def request_mode(self, mode: Literal["LOITER", "RTL"]) -> None:
        """Request a mode change (no ARM/DISARM — S1)."""

    @abstractmethod
    def close(self) -> None:
        """Close the FC connection."""
