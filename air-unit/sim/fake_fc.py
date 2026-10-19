"""FakeFC — in-process FlightController mock for testing (T3.1).

Implements FlightController ABC without any hardware.
Tracks mode, velocity commands, and mode change requests.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Literal

from air_unit.hardware.interfaces import FcState, FlightController

logger = logging.getLogger(__name__)


@dataclass
class FakeFC(FlightController):
    """In-process mock flight controller.

    Records all velocity commands and mode requests for test assertions.
    Safety rule S1: No ARM/DISARM API (by design — not even in the mock).
    """

    _mode: str = "GUIDED"
    _armed: bool = True
    _battery_v: float = 16.2
    _battery_pct: float = 85.0
    _alt_m: float = 10.0

    # Recording for test assertions
    _velocity_history: list[tuple[float, float, float, float]] = field(default_factory=list)
    _mode_requests: list[str] = field(default_factory=list)
    _closed: bool = False

    def state(self) -> FcState:
        """Return current simulated FC state."""
        return FcState(
            mode=self._mode,
            armed=self._armed,
            battery_v=self._battery_v,
            battery_pct=self._battery_pct,
            alt_m=self._alt_m,
        )

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """Record a velocity setpoint.

        Args:
            vx, vy, vz, yaw_rate: Body-frame velocity setpoint.
        """
        self._velocity_history.append((vx, vy, vz, yaw_rate))

    def request_mode(self, mode: Literal["LOITER", "RTL"]) -> None:
        """Record a mode change request."""
        logger.info("FakeFC: mode request → %s", mode)
        self._mode_requests.append(mode)
        self._mode = mode

    def close(self) -> None:
        """Mark as closed."""
        self._closed = True

    # ── Test helpers ─────────────────────────

    def set_mode(self, mode: str) -> None:
        """Set the simulated FC mode (for test setup)."""
        self._mode = mode

    def set_armed(self, armed: bool) -> None:
        """Set the simulated armed state (for test setup)."""
        self._armed = armed

    def set_battery(self, voltage: float, pct: float = -1.0) -> None:
        """Set simulated battery state."""
        self._battery_v = voltage
        self._battery_pct = pct

    @property
    def last_velocity(self) -> tuple[float, float, float, float] | None:
        """Get the last velocity command, or None if none sent."""
        return self._velocity_history[-1] if self._velocity_history else None

    @property
    def velocity_count(self) -> int:
        """Number of velocity commands sent."""
        return len(self._velocity_history)

    def clear_history(self) -> None:
        """Clear recorded commands."""
        self._velocity_history.clear()
        self._mode_requests.clear()
