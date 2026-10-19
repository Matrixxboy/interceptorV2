"""Dynamic limits — clamp + slew-rate limiter (§8, T1.5).

Clamps each axis to min(config_max, GCS_max, HARD_MAX), plus slew-rate limit
(max Δ per second) to avoid jerk. Scales speed down as battery falls below threshold.

Safety rule S3: Every setpoint passes through dynamic_limits.clamp() before MAVLink. No bypass.
Safety rule S8: All hard limits live in shared/constants.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from shared.constants import (
    BATTERY_MIN_V,
    BATTERY_SPEED_SCALE_THRESHOLD_V,
    HARD_MAX_VX,
    HARD_MAX_VY,
    HARD_MAX_VZ,
    HARD_MAX_YAW_RATE,
    SLEW_RATE_VX,
    SLEW_RATE_VY,
    SLEW_RATE_VZ,
    SLEW_RATE_YAW,
)


@dataclass(frozen=True)
class VelocitySetpoint:
    """Body-frame velocity setpoint (FRD) — duplicated here for independence.

    Import from visual_servoing.py in integration; this keeps limits self-contained.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0

    def __iter__(self):
        yield self.vx
        yield self.vy
        yield self.vz
        yield self.yaw_rate


@dataclass
class LimitsConfig:
    """Configurable limits (from settings.yaml + GCS tuning)."""

    max_vx: float = 3.0
    max_vy: float = 1.5
    max_vz: float = 1.5
    max_yaw_rate: float = 1.2

    # GCS-commanded maximums (initially same as config)
    gcs_max_vx: float = 3.0
    gcs_max_vy: float = 1.5
    gcs_max_vz: float = 1.5
    gcs_max_yaw_rate: float = 1.2


@dataclass
class DynamicLimiter:
    """Clamps and slew-rate-limits velocity setpoints before MAVLink output.

    Three-way clamp: min(config_max, GCS_max, HARD_MAX) per axis.
    Slew-rate limit: max change per second to avoid jerk.
    Battery scaling: reduce speed as voltage drops.
    """

    config: LimitsConfig = field(default_factory=LimitsConfig)
    _prev_vx: float = 0.0
    _prev_vy: float = 0.0
    _prev_vz: float = 0.0
    _prev_yaw_rate: float = 0.0

    def clamp(
        self,
        vx: float | VelocitySetpoint,
        vy: float | None = None,
        vz: float | None = None,
        yaw_rate: float | None = None,
        dt: float = 0.033,
        battery_v: float | None = None,
    ) -> VelocitySetpoint:
        """Apply all limits to a velocity setpoint.

        This is the ONLY path setpoints take before MAVLink (S3).

        Args:
            vx: Raw velocity setpoint values (or VelocitySetpoint object).
            vy, vz, yaw_rate: Component values if vx is float.
            dt: Time step in seconds (for slew-rate).
            battery_v: Current battery voltage (for speed scaling), or None.

        Returns:
            VelocitySetpoint (also iterable as 4 floats) after all limits applied.
        """
        if hasattr(vx, "vx"):
            sp = vx
            vx_val = float(sp.vx)
            vy_val = float(sp.vy)
            vz_val = float(sp.vz)
            yr_val = float(sp.yaw_rate)
        else:
            vx_val = float(vx)
            vy_val = float(vy if vy is not None else 0.0)
            vz_val = float(vz if vz is not None else 0.0)
            yr_val = float(yaw_rate if yaw_rate is not None else 0.0)

        # ── Sanitize NaN/inf → 0 (S10) ─────
        vx_val = _sanitize(vx_val)
        vy_val = _sanitize(vy_val)
        vz_val = _sanitize(vz_val)
        yr_val = _sanitize(yr_val)

        # ── Three-way clamp per axis ────────
        cap_vx = min(self.config.max_vx, self.config.gcs_max_vx, HARD_MAX_VX)
        cap_vy = min(self.config.max_vy, self.config.gcs_max_vy, HARD_MAX_VY)
        cap_vz = min(self.config.max_vz, self.config.gcs_max_vz, HARD_MAX_VZ)
        cap_yaw = min(self.config.max_yaw_rate, self.config.gcs_max_yaw_rate, HARD_MAX_YAW_RATE)

        # Battery speed scaling
        scale = _battery_scale(battery_v) if battery_v is not None else 1.0

        vx_val = _sym_clamp(vx_val, cap_vx * scale)
        vy_val = _sym_clamp(vy_val, cap_vy * scale)
        vz_val = _sym_clamp(vz_val, cap_vz * scale)
        yr_val = _sym_clamp(yr_val, cap_yaw * scale)

        # ── Slew-rate limit ─────────────────
        if dt > 0:
            vx_val = _slew_limit(vx_val, self._prev_vx, SLEW_RATE_VX, dt)
            vy_val = _slew_limit(vy_val, self._prev_vy, SLEW_RATE_VY, dt)
            vz_val = _slew_limit(vz_val, self._prev_vz, SLEW_RATE_VZ, dt)
            yr_val = _slew_limit(yr_val, self._prev_yaw_rate, SLEW_RATE_YAW, dt)

        # ── Store for next iteration ────────
        self._prev_vx = vx_val
        self._prev_vy = vy_val
        self._prev_vz = vz_val
        self._prev_yaw_rate = yr_val

        return VelocitySetpoint(vx=vx_val, vy=vy_val, vz=vz_val, yaw_rate=yr_val)

    def update_gcs_limits(
        self,
        max_vx: float | None = None,
        max_vy: float | None = None,
        max_vz: float | None = None,
        max_yaw_rate: float | None = None,
    ) -> None:
        """Update GCS-commanded limits (within hard caps)."""
        if max_vx is not None:
            self.config.gcs_max_vx = min(max_vx, HARD_MAX_VX)
        if max_vy is not None:
            self.config.gcs_max_vy = min(max_vy, HARD_MAX_VY)
        if max_vz is not None:
            self.config.gcs_max_vz = min(max_vz, HARD_MAX_VZ)
        if max_yaw_rate is not None:
            self.config.gcs_max_yaw_rate = min(max_yaw_rate, HARD_MAX_YAW_RATE)

    def reset(self) -> None:
        """Reset slew-rate state (call on mode change)."""
        self._prev_vx = 0.0
        self._prev_vy = 0.0
        self._prev_vz = 0.0
        self._prev_yaw_rate = 0.0


# ── Pure functions ───────────────────────────


def _sanitize(v: float) -> float:
    """Replace NaN/inf with 0.0 (S10: default for unknown is hover)."""
    if not math.isfinite(v):
        return 0.0
    return v


def _sym_clamp(v: float, cap: float) -> float:
    """Symmetric clamp: -cap ≤ v ≤ cap."""
    if v > cap:
        return cap
    if v < -cap:
        return -cap
    return v


def _slew_limit(current: float, previous: float, max_rate: float, dt: float) -> float:
    """Limit the rate of change to avoid jerk.

    Args:
        current: Desired value.
        previous: Value at the previous step.
        max_rate: Maximum change per second.
        dt: Time step in seconds.

    Returns:
        Value clamped to be within max_rate * dt of previous.
    """
    max_delta = max_rate * dt
    delta = current - previous
    if delta > max_delta:
        return previous + max_delta
    if delta < -max_delta:
        return previous - max_delta
    return current


def _battery_scale(voltage: float) -> float:
    """Scale speed linearly as battery drops below threshold.

    Returns 1.0 above threshold, scales down to 0.3 at BATTERY_MIN_V.
    """
    if voltage >= BATTERY_SPEED_SCALE_THRESHOLD_V:
        return 1.0
    if voltage <= BATTERY_MIN_V:
        return 0.3
    # Linear interpolation
    t = (voltage - BATTERY_MIN_V) / (BATTERY_SPEED_SCALE_THRESHOLD_V - BATTERY_MIN_V)
    return 0.3 + 0.7 * t


# Aliases
DynamicLimits = DynamicLimiter
DynamicLimitsConfig = LimitsConfig
