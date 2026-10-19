"""Tests for control/dynamic_limits.py — T1.5 acceptance.

- Output never exceeds any cap (hard, config, GCS)
- Slew rate respected
- NaN/inf sanitized to zero
- Battery scaling works
"""

from __future__ import annotations

import math

import pytest
from air_unit.control.dynamic_limits import (
    DynamicLimiter,
    LimitsConfig,
    _battery_scale,
    _sanitize,
    _slew_limit,
    _sym_clamp,
)

from shared.constants import HARD_MAX_VX, HARD_MAX_VY, HARD_MAX_VZ, HARD_MAX_YAW_RATE


class TestOutputNeverExceedsCap:
    """T1.5 acceptance: output never exceeds any cap."""

    def test_output_within_hard_limits(self) -> None:
        limiter = DynamicLimiter()
        vx, vy, vz, yr = limiter.clamp(100.0, 100.0, 100.0, 100.0, dt=0.033)
        assert abs(vx) <= HARD_MAX_VX
        assert abs(vy) <= HARD_MAX_VY
        assert abs(vz) <= HARD_MAX_VZ
        assert abs(yr) <= HARD_MAX_YAW_RATE

    def test_output_within_config_limits(self) -> None:
        config = LimitsConfig(max_vx=1.0, max_vy=0.5, max_vz=0.5, max_yaw_rate=0.5)
        limiter = DynamicLimiter(config=config)
        vx, vy, vz, yr = limiter.clamp(100.0, 100.0, 100.0, 100.0, dt=0.033)
        # Slew rate may limit further, but definitely within config
        assert abs(vx) <= 1.0 + 0.01  # small epsilon for float
        assert abs(vy) <= 0.5 + 0.01
        assert abs(vz) <= 0.5 + 0.01
        assert abs(yr) <= 0.5 + 0.01

    def test_output_within_gcs_limits(self) -> None:
        limiter = DynamicLimiter(config=LimitsConfig())
        limiter.update_gcs_limits(max_vx=1.0)
        vx, _, _, _ = limiter.clamp(100.0, 0.0, 0.0, 0.0, dt=0.033)
        assert abs(vx) <= 1.0 + 0.01

    def test_three_way_min(self) -> None:
        """Uses the minimum of (config, GCS, HARD_MAX)."""
        config = LimitsConfig(max_vx=2.0, gcs_max_vx=1.5)
        limiter = DynamicLimiter(config=config)
        # First clamp (from 0) limited by slew rate
        limiter.clamp(0.0, 0.0, 0.0, 0.0, dt=0.033)  # init prev to 0
        # Now request big value
        vx, _, _, _ = limiter.clamp(10.0, 0.0, 0.0, 0.0, dt=10.0)  # large dt = no slew limit
        assert abs(vx) <= 1.5 + 0.01  # GCS is the tightest

    @pytest.mark.parametrize("input_val", [-100.0, -50.0, 0.0, 50.0, 100.0])
    def test_property_output_always_within_hard_limits(self, input_val: float) -> None:
        """Property test: output always within hard limits regardless of input."""
        limiter = DynamicLimiter()
        vx, vy, vz, yr = limiter.clamp(input_val, input_val, input_val, input_val, dt=1.0)
        assert abs(vx) <= HARD_MAX_VX + 0.01
        assert abs(vy) <= HARD_MAX_VY + 0.01
        assert abs(vz) <= HARD_MAX_VZ + 0.01
        assert abs(yr) <= HARD_MAX_YAW_RATE + 0.01


class TestSlewRate:
    """T1.5 acceptance: slew rate respected."""

    def test_slew_limits_large_step(self) -> None:
        limiter = DynamicLimiter()
        dt = 0.033  # ~30 Hz
        # From 0, request max. Slew rate limits the jump.
        vx, _, _, _ = limiter.clamp(100.0, 0.0, 0.0, 0.0, dt=dt)
        # Max delta per step = SLEW_RATE_VX * dt = 4.0 * 0.033 ≈ 0.132
        from shared.constants import SLEW_RATE_VX

        assert abs(vx) <= SLEW_RATE_VX * dt + 0.01

    def test_slew_pure_function(self) -> None:
        result = _slew_limit(current=10.0, previous=0.0, max_rate=5.0, dt=0.1)
        assert result == pytest.approx(0.5)  # 5.0 * 0.1


class TestNanSanitization:
    """NaN/inf → 0 (S10: hover)."""

    def test_nan_sanitized(self) -> None:
        assert _sanitize(float("nan")) == 0.0

    def test_inf_sanitized(self) -> None:
        assert _sanitize(float("inf")) == 0.0

    def test_neg_inf_sanitized(self) -> None:
        assert _sanitize(float("-inf")) == 0.0

    def test_normal_value_passes(self) -> None:
        assert _sanitize(3.14) == 3.14

    def test_nan_input_to_clamp(self) -> None:
        limiter = DynamicLimiter()
        vx, vy, vz, yr = limiter.clamp(
            float("nan"), float("inf"), float("-inf"), float("nan"), dt=0.033
        )
        assert math.isfinite(vx)
        assert math.isfinite(vy)
        assert math.isfinite(vz)
        assert math.isfinite(yr)


class TestBatteryScaling:
    """Battery speed scaling."""

    def test_full_battery_no_scaling(self) -> None:
        assert _battery_scale(16.0) == 1.0

    def test_at_threshold(self) -> None:
        from shared.constants import BATTERY_SPEED_SCALE_THRESHOLD_V

        assert _battery_scale(BATTERY_SPEED_SCALE_THRESHOLD_V) == 1.0

    def test_at_min_voltage(self) -> None:
        from shared.constants import BATTERY_MIN_V

        assert _battery_scale(BATTERY_MIN_V) == pytest.approx(0.3)

    def test_mid_range(self) -> None:
        scale = _battery_scale(13.6)
        assert 0.3 < scale < 1.0

    def test_clamp_applied_with_low_battery(self) -> None:
        limiter = DynamicLimiter()
        vx_full, _, _, _ = limiter.clamp(3.0, 0.0, 0.0, 0.0, dt=10.0, battery_v=16.0)
        limiter.reset()
        vx_low, _, _, _ = limiter.clamp(3.0, 0.0, 0.0, 0.0, dt=10.0, battery_v=13.5)
        assert abs(vx_low) < abs(vx_full)


class TestSymClamp:
    """Symmetric clamping."""

    def test_clamp_positive(self) -> None:
        assert _sym_clamp(10.0, 5.0) == 5.0

    def test_clamp_negative(self) -> None:
        assert _sym_clamp(-10.0, 5.0) == -5.0

    def test_within_range(self) -> None:
        assert _sym_clamp(3.0, 5.0) == 3.0


class TestReset:
    """Reset clears slew-rate state."""

    def test_reset_allows_instant_change(self) -> None:
        limiter = DynamicLimiter()
        limiter.clamp(2.0, 0.0, 0.0, 0.0, dt=10.0)  # set prev to 2.0
        limiter.reset()
        # After reset, prev is 0 again → slew from 0
        vx, _, _, _ = limiter.clamp(2.0, 0.0, 0.0, 0.0, dt=0.033)
        # Should be slew-limited from 0
        from shared.constants import SLEW_RATE_VX

        assert abs(vx) <= SLEW_RATE_VX * 0.033 + 0.01
