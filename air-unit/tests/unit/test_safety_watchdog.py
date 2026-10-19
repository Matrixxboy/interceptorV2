"""Tests for control/safety_watchdog.py — T1.6 acceptance.

Tests named test_S<n>_* to map to safety rules (§1).
Uses fake monotonic clock for deterministic testing.
"""

from __future__ import annotations

from air_unit.control.safety_watchdog import SafetyWatchdog, WatchdogConfig


class TestS2SetpointConditions:
    """S2: Velocity setpoints sent ONLY when FC mode is GUIDED and state is FOLLOWING."""

    def test_S2_following_guided_allows_setpoints(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)
        action = wd.check(now=0.1, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is True

    def test_S2_idle_blocks_setpoints(self) -> None:
        wd = SafetyWatchdog()
        action = wd.check(now=0.0, fc_mode="GUIDED", state="IDLE")
        assert action.allow_setpoints is False
        assert "S2" in action.reason

    def test_S2_selecting_blocks_setpoints(self) -> None:
        wd = SafetyWatchdog()
        action = wd.check(now=0.0, fc_mode="GUIDED", state="SELECTING")
        assert action.allow_setpoints is False

    def test_S2_not_guided_blocks_setpoints(self) -> None:
        wd = SafetyWatchdog()
        action = wd.check(now=0.0, fc_mode="STABILIZE", state="FOLLOWING")
        assert action.allow_setpoints is False


class TestS4TargetLost:
    """S4: No fresh target → hover; after TARGET_LOST_RTL_S → LOITER."""

    def test_S4_target_fresh_allows_setpoints(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)
        action = wd.check(now=0.5, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is True

    def test_S4_target_lost_timeout_hover(self) -> None:
        """Target missing > 1.0s → hover."""
        wd = SafetyWatchdog(config=WatchdogConfig(target_lost_timeout_s=1.0))
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)

        action = wd.check(now=1.5, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is False
        assert "S4" in action.reason

    def test_S4_target_lost_rtl_loiter(self) -> None:
        """Target missing > 5.0s → request LOITER."""
        wd = SafetyWatchdog(
            config=WatchdogConfig(
                target_lost_timeout_s=1.0,
                target_lost_rtl_s=5.0,
            )
        )
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=6.0)

        action = wd.check(now=6.0, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is False
        assert action.request_mode == "LOITER"

    def test_S4_target_reacquired_allows(self) -> None:
        """Target reappears → setpoints allowed again."""
        wd = SafetyWatchdog(config=WatchdogConfig(target_lost_timeout_s=1.0))
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=2.6)

        # Lost
        action = wd.check(now=2.0, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is False

        # Reacquired
        wd.update_target_seen(now=2.5)
        wd.update_gcs_heartbeat(now=2.6)
        action = wd.check(now=2.6, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is True


class TestS5GcsHeartbeat:
    """S5: No GCS heartbeat → hover; after GCS_RTL_S → RTL."""

    def test_S5_gcs_fresh_allows_setpoints(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)
        action = wd.check(now=1.0, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is True

    def test_S5_gcs_timeout_hover(self) -> None:
        """GCS heartbeat missing > 2.0s → hover."""
        wd = SafetyWatchdog(config=WatchdogConfig(gcs_timeout_s=2.0))
        wd.update_target_seen(now=3.0)
        wd.update_gcs_heartbeat(now=0.0)

        action = wd.check(now=3.0, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is False
        assert "S5" in action.reason

    def test_S5_gcs_rtl(self) -> None:
        """GCS heartbeat missing > 10.0s → RTL."""
        wd = SafetyWatchdog(config=WatchdogConfig(gcs_timeout_s=2.0, gcs_rtl_s=10.0))
        wd.update_target_seen(now=11.0)
        wd.update_gcs_heartbeat(now=0.0)

        action = wd.check(now=11.0, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is False
        assert action.request_mode == "RTL"

    def test_S5_gcs_heartbeat_restores(self) -> None:
        """GCS heartbeat received again → setpoints allowed."""
        wd = SafetyWatchdog(config=WatchdogConfig(gcs_timeout_s=2.0))
        wd.update_target_seen(now=3.6)
        wd.update_gcs_heartbeat(now=0.0)

        # Timeout
        action = wd.check(now=3.0, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is False

        # Heartbeat received
        wd.update_target_seen(now=3.6)
        wd.update_gcs_heartbeat(now=3.5)
        action = wd.check(now=3.6, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is True


class TestS6PilotOverride:
    """S6: Pilot switching FC out of GUIDED instantly stops output."""

    def test_S6_mode_change_stops_setpoints(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)

        action = wd.check(now=0.1, fc_mode="STABILIZE", state="FOLLOWING")
        assert action.allow_setpoints is False
        assert "S6" in action.reason

    def test_S6_loiter_mode_stops_setpoints(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)

        action = wd.check(now=0.1, fc_mode="LOITER", state="FOLLOWING")
        assert action.allow_setpoints is False

    def test_S6_target_lost_state_also_stops(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)

        action = wd.check(now=0.1, fc_mode="ALT_HOLD", state="TARGET_LOST")
        assert action.allow_setpoints is False
        assert "S6" in action.reason

    def test_S6_idle_with_alt_hold_no_s6_reason(self) -> None:
        """In IDLE, non-GUIDED is handled by S2, not S6."""
        wd = SafetyWatchdog()
        action = wd.check(now=0.0, fc_mode="ALT_HOLD", state="IDLE")
        assert action.allow_setpoints is False
        # S2 blocks it (not FOLLOWING), S6 only triggers for FOLLOWING/TARGET_LOST
        assert "S2" in action.reason


class TestWatchdogReset:
    """Reset clears all state."""

    def test_reset(self) -> None:
        wd = SafetyWatchdog()
        wd.update_target_seen(now=5.0)
        wd.update_gcs_heartbeat(now=5.0)
        wd.reset()
        # After reset, timestamps are 0 — no timeouts triggered (since _last == 0)
        action = wd.check(now=0.1, fc_mode="GUIDED", state="FOLLOWING")
        assert action.allow_setpoints is True
