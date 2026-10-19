"""PID controller — anti-windup, derivative filter, deadband, output clamping (T1.2).

Pure function for the step — easy to unit test (§2.7).
Each PID: output clamp, integral clamp (anti-windup), low-pass on derivative, reset on state change.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PidConfig:
    """PID configuration parameters."""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -1.0
    output_max: float = 1.0
    integral_min: float = -5.0  # anti-windup clamp
    integral_max: float = 5.0
    deadband: float = 0.0  # ignore errors within ±deadband
    d_filter_alpha: float = 0.1  # low-pass on derivative (0 = no filtering, 1 = no smoothing)


@dataclass
class PidState:
    """Internal PID state — mutable, reset on state change."""

    integral: float = 0.0
    prev_error: float = 0.0
    prev_d_filtered: float = 0.0
    initialized: bool = False


@dataclass
class PidController:
    """PID controller with anti-windup, derivative filter, and deadband.

    All math is pure — no side effects, no I/O, easy to unit test.
    """

    config: PidConfig = field(default_factory=PidConfig)
    _state: PidState = field(default_factory=PidState)

    def step(self, error: float, dt: float) -> float:
        """Compute one PID step.

        Args:
            error: Current error signal (setpoint - measurement).
            dt: Time step in seconds (must be > 0).

        Returns:
            Clamped PID output.
        """
        if dt <= 0:
            return 0.0

        # ── Deadband ────────────────────────
        if abs(error) < self.config.deadband:
            error = 0.0

        # ── Proportional ────────────────────
        p_term = self.config.kp * error

        # ── Integral (with anti-windup) ─────
        self._state.integral += error * dt
        self._state.integral = _clamp(
            self._state.integral,
            self.config.integral_min,
            self.config.integral_max,
        )
        i_term = self.config.ki * self._state.integral

        # ── Derivative (with low-pass) ──────
        if not self._state.initialized:
            # First call — no derivative
            raw_d = 0.0
            self._state.initialized = True
        else:
            raw_d = (error - self._state.prev_error) / dt

        # Low-pass filter on derivative: alpha=1 → no smoothing, alpha=0 → frozen
        alpha = self.config.d_filter_alpha
        d_filtered = alpha * raw_d + (1.0 - alpha) * self._state.prev_d_filtered
        self._state.prev_d_filtered = d_filtered
        d_term = self.config.kd * d_filtered

        # ── Store for next step ─────────────
        self._state.prev_error = error

        # ── Sum and clamp output ────────────
        output = p_term + i_term + d_term
        return _clamp(output, self.config.output_min, self.config.output_max)

    def reset(self) -> None:
        """Reset internal state (call on state machine transition)."""
        self._state = PidState()

    @property
    def integral(self) -> float:
        """Current integral accumulator value."""
        return self._state.integral


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value between lo and hi."""
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def pid_step(
    error: float,
    dt: float,
    kp: float,
    ki: float,
    kd: float,
    integral: float,
    prev_error: float,
    prev_d_filtered: float,
    output_min: float = -1.0,
    output_max: float = 1.0,
    integral_min: float = -5.0,
    integral_max: float = 5.0,
    deadband: float = 0.0,
    d_filter_alpha: float = 0.1,
) -> tuple[float, float, float, float]:
    """Pure-function PID step (stateless, for testing and replay).

    Args:
        error: Current error.
        dt: Time step in seconds.
        kp, ki, kd: PID gains.
        integral: Current integral accumulator.
        prev_error: Previous error.
        prev_d_filtered: Previous filtered derivative.
        output_min, output_max: Output clamp range.
        integral_min, integral_max: Anti-windup clamp range.
        deadband: Error deadband.
        d_filter_alpha: Derivative low-pass filter coefficient.

    Returns:
        Tuple of (output, new_integral, new_prev_error, new_prev_d_filtered).
    """
    if dt <= 0:
        return 0.0, integral, prev_error, prev_d_filtered

    # Deadband
    if abs(error) < deadband:
        error = 0.0

    # P
    p_term = kp * error

    # I (anti-windup)
    new_integral = _clamp(integral + error * dt, integral_min, integral_max)
    i_term = ki * new_integral

    # D (low-pass)
    raw_d = (error - prev_error) / dt
    d_filtered = d_filter_alpha * raw_d + (1.0 - d_filter_alpha) * prev_d_filtered
    d_term = kd * d_filtered

    # Output
    output = _clamp(p_term + i_term + d_term, output_min, output_max)

    return output, new_integral, error, d_filtered
