"""Tests for control/pid.py — T1.2 acceptance.

- Step-response converges
- No windup (integral stays clamped)
- Output always clamped
- Deadband suppresses small errors
- Reset clears state
"""

from __future__ import annotations

import pytest
from air_unit.control.pid import PidConfig, PidController, pid_step


class TestPidStepResponse:
    """PID step response converges (T1.2 acceptance)."""

    def test_proportional_step_response(self) -> None:
        """Constant error with P-only → constant output."""
        pid = PidController(config=PidConfig(kp=1.0, ki=0.0, kd=0.0, output_min=-10, output_max=10))
        output = pid.step(error=5.0, dt=0.033)
        assert output == pytest.approx(5.0)

    def test_convergence_with_pi(self) -> None:
        """PI controller with decreasing error converges."""
        pid = PidController(config=PidConfig(kp=0.5, ki=0.1, kd=0.0, output_min=-10, output_max=10))
        error = 10.0
        for _ in range(200):
            output = pid.step(error, dt=0.033)
            error -= output * 0.033  # simulate simple integrator plant

        # Should converge near zero
        assert abs(error) < 1.0

    def test_pid_converges(self) -> None:
        """Full PID converges on step response."""
        pid = PidController(
            config=PidConfig(
                kp=1.0,
                ki=0.1,
                kd=0.05,
                output_min=-10,
                output_max=10,
                d_filter_alpha=0.3,
            )
        )
        error = 5.0
        for _ in range(500):
            output = pid.step(error, dt=0.033)
            error -= output * 0.033

        assert abs(error) < 0.5


class TestPidAntiWindup:
    """Integral stays clamped (anti-windup, T1.2 acceptance)."""

    def test_integral_clamp(self) -> None:
        """Large sustained error → integral clamps at limit."""
        pid = PidController(
            config=PidConfig(
                kp=0.0,
                ki=1.0,
                kd=0.0,
                output_min=-100,
                output_max=100,
                integral_min=-2.0,
                integral_max=2.0,
            )
        )
        # Apply large error for many steps
        for _ in range(1000):
            pid.step(error=100.0, dt=0.033)

        assert pid.integral == pytest.approx(2.0)  # clamped at max

    def test_negative_integral_clamp(self) -> None:
        pid = PidController(
            config=PidConfig(
                kp=0.0,
                ki=1.0,
                kd=0.0,
                integral_min=-3.0,
                integral_max=3.0,
                output_min=-100,
                output_max=100,
            )
        )
        for _ in range(1000):
            pid.step(error=-100.0, dt=0.033)

        assert pid.integral == pytest.approx(-3.0)


class TestPidOutputClamp:
    """Output is always clamped (T1.2 acceptance)."""

    def test_output_never_exceeds_max(self) -> None:
        pid = PidController(config=PidConfig(kp=100.0, output_min=-1.0, output_max=1.0))
        output = pid.step(error=100.0, dt=0.033)
        assert output <= 1.0

    def test_output_never_below_min(self) -> None:
        pid = PidController(config=PidConfig(kp=100.0, output_min=-1.0, output_max=1.0))
        output = pid.step(error=-100.0, dt=0.033)
        assert output >= -1.0


class TestPidDeadband:
    """Deadband suppresses small errors."""

    def test_error_within_deadband_is_zero(self) -> None:
        pid = PidController(config=PidConfig(kp=1.0, deadband=0.05, output_min=-10, output_max=10))
        output = pid.step(error=0.03, dt=0.033)
        assert output == pytest.approx(0.0)

    def test_error_outside_deadband_passes(self) -> None:
        pid = PidController(config=PidConfig(kp=1.0, deadband=0.05, output_min=-10, output_max=10))
        output = pid.step(error=0.1, dt=0.033)
        assert output == pytest.approx(0.1)


class TestPidReset:
    """Reset clears state."""

    def test_reset_clears_integral(self) -> None:
        pid = PidController(config=PidConfig(ki=1.0, output_min=-100, output_max=100))
        pid.step(10.0, 0.033)
        assert pid.integral != 0.0
        pid.reset()
        assert pid.integral == 0.0


class TestPidPureFunction:
    """Pure-function pid_step matches OOP PidController."""

    def test_pure_function_output_matches(self) -> None:
        output, _, _, _ = pid_step(
            error=5.0,
            dt=0.033,
            kp=1.0,
            ki=0.0,
            kd=0.0,
            integral=0.0,
            prev_error=0.0,
            prev_d_filtered=0.0,
            output_min=-10,
            output_max=10,
        )
        assert output == pytest.approx(5.0)

    def test_zero_dt_returns_zero(self) -> None:
        pid = PidController(config=PidConfig(kp=1.0))
        assert pid.step(5.0, 0.0) == 0.0
