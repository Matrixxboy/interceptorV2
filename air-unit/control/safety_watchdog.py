"""Safety watchdog — enforces S2, S4, S5, S6 (§1, T1.6).

Runs checks at control-loop rate. Uses monotonic time only (§2.6).

S2: Velocity setpoints sent ONLY when FC mode is GUIDED and state is FOLLOWING.
S4: No fresh target for TARGET_LOST_TIMEOUT_S → hover. After TARGET_LOST_RTL_S → LOITER.
S5: No GCS heartbeat for GCS_TIMEOUT_S → hover. After GCS_RTL_S → RTL.
S6: Pilot switching FC out of GUIDED instantly stops all setpoint output.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Literal

from shared.constants import (
    GCS_RTL_S,
    GCS_TIMEOUT_S,
    TARGET_LOST_RTL_S,
    TARGET_LOST_TIMEOUT_S,
)

logger = logging.getLogger(__name__)


@dataclass
class WatchdogConfig:
    """Safety watchdog configuration."""

    target_lost_timeout_s: float = TARGET_LOST_TIMEOUT_S
    target_lost_rtl_s: float = TARGET_LOST_RTL_S
    gcs_timeout_s: float = GCS_TIMEOUT_S
    gcs_rtl_s: float = GCS_RTL_S


@dataclass
class WatchdogAction:
    """Result of a watchdog check cycle."""

    allow_setpoints: bool = True
    request_mode: Literal["LOITER", "RTL"] | None = None
    reason: str = ""


@dataclass
class SafetyWatchdog:
    """Enforces safety rules S2, S4, S5, S6.

    Call `check()` at every control-loop tick with current timestamps and FC state.
    The watchdog returns a WatchdogAction that the control loop must obey.
    """

    config: WatchdogConfig = field(default_factory=WatchdogConfig)

    # Monotonic timestamps of last known-good events (None = no event received yet)
    _last_target_ts: float | None = None
    _last_gcs_heartbeat_ts: float | None = None

    # Flags for one-shot logging
    _target_lost_logged: bool = False
    _gcs_timeout_logged: bool = False

    def update_target_seen(self, now: float) -> None:
        """Call when a valid target detection is received.

        Args:
            now: Current monotonic time in seconds.
        """
        self._last_target_ts = now
        self._target_lost_logged = False

    def update_gcs_heartbeat(self, now: float) -> None:
        """Call when a GCS heartbeat is received.

        Args:
            now: Current monotonic time in seconds.
        """
        self._last_gcs_heartbeat_ts = now
        self._gcs_timeout_logged = False

    def check(
        self,
        now: float,
        fc_mode: str,
        state: str,
    ) -> WatchdogAction:
        """Run all safety checks and return the required action.

        Args:
            now: Current monotonic time in seconds.
            fc_mode: Current flight controller mode string (e.g., "GUIDED").
            state: Current state machine state string (e.g., "FOLLOWING").

        Returns:
            WatchdogAction indicating what the control loop must do.
        """
        # ── S6: Pilot switched FC out of GUIDED → stop immediately ──
        if fc_mode != "GUIDED" and state in ("FOLLOWING", "TARGET_LOST"):
            logger.warning("S6: FC mode is %s (not GUIDED) — stopping setpoints", fc_mode)
            return WatchdogAction(
                allow_setpoints=False,
                reason=f"S6: FC mode {fc_mode} is not GUIDED",
            )

        # ── S2: Only send setpoints in GUIDED + FOLLOWING ───────────
        if state != "FOLLOWING":
            return WatchdogAction(
                allow_setpoints=False,
                reason=f"S2: state is {state}, not FOLLOWING",
            )
        if fc_mode != "GUIDED":
            return WatchdogAction(
                allow_setpoints=False,
                reason=f"S2: FC mode is {fc_mode}, not GUIDED",
            )

        # ── S5: GCS heartbeat timeout ──────────────────────────────
        if self._last_gcs_heartbeat_ts is not None:
            gcs_elapsed = now - self._last_gcs_heartbeat_ts
            if gcs_elapsed > self.config.gcs_rtl_s:
                if not self._gcs_timeout_logged:
                    logger.error("S5: GCS heartbeat lost for %.1fs — requesting RTL", gcs_elapsed)
                    self._gcs_timeout_logged = True
                return WatchdogAction(
                    allow_setpoints=False,
                    request_mode="RTL",
                    reason=f"S5: GCS heartbeat lost for {gcs_elapsed:.1f}s",
                )
            if gcs_elapsed > self.config.gcs_timeout_s:
                if not self._gcs_timeout_logged:
                    logger.warning("S5: GCS heartbeat lost for %.1fs — hover", gcs_elapsed)
                    self._gcs_timeout_logged = True
                return WatchdogAction(
                    allow_setpoints=False,
                    reason=f"S5: GCS heartbeat lost for {gcs_elapsed:.1f}s — hover",
                )

        # ── S4: Target lost timeout ────────────────────────────────
        if self._last_target_ts is not None:
            target_elapsed = now - self._last_target_ts
            if target_elapsed > self.config.target_lost_rtl_s:
                if not self._target_lost_logged:
                    logger.error("S4: Target lost for %.1fs — requesting LOITER", target_elapsed)
                    self._target_lost_logged = True
                return WatchdogAction(
                    allow_setpoints=False,
                    request_mode="LOITER",
                    reason=f"S4: Target lost for {target_elapsed:.1f}s",
                )
            if target_elapsed > self.config.target_lost_timeout_s:
                if not self._target_lost_logged:
                    logger.warning("S4: Target lost for %.1fs — hover", target_elapsed)
                    self._target_lost_logged = True
                return WatchdogAction(
                    allow_setpoints=False,
                    reason=f"S4: Target lost for {target_elapsed:.1f}s — hover",
                )

        # ── All checks passed: allow setpoints ─────────────────────
        return WatchdogAction(allow_setpoints=True)

    def reset(self) -> None:
        """Reset all timestamps (call on system init)."""
        self._last_target_ts = None
        self._last_gcs_heartbeat_ts = None
        self._target_lost_logged = False
        self._gcs_timeout_logged = False
