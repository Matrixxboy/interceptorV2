"""Tests for control/visual_servoing.py — T1.4 acceptance.

Sign convention tests (§13):
- Target right of center → positive yaw_rate
- Target below center → positive vz (down)
- Target too far → positive vx (forward)
"""

from __future__ import annotations

from air_unit.control.pid import PidConfig
from air_unit.control.visual_servoing import VelocitySetpoint, VisualServo, VisualServoConfig


class TestSignConventions:
    """T1.4 acceptance: verify body-frame FRD sign conventions."""

    def _make_servo(self) -> VisualServo:
        return VisualServo(
            config=VisualServoConfig(
                yaw_pid=PidConfig(kp=1.0, output_min=-2.0, output_max=2.0, deadband=0.0),
                alt_pid=PidConfig(kp=1.0, output_min=-2.0, output_max=2.0, deadband=0.0),
                fwd_pid=PidConfig(kp=1.0, output_min=-5.0, output_max=5.0),
                follow_distance_m=6.0,
            )
        )

    def test_target_right_positive_yaw(self) -> None:
        """Target right of center → positive yaw_rate (clockwise)."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.5, cy_norm=0.0, distance_m=6.0, dt=0.033)
        assert sp.yaw_rate > 0

    def test_target_left_negative_yaw(self) -> None:
        """Target left of center → negative yaw_rate (counter-clockwise)."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=-0.5, cy_norm=0.0, distance_m=6.0, dt=0.033)
        assert sp.yaw_rate < 0

    def test_target_below_positive_vz(self) -> None:
        """Target below center → positive vz (down, FRD)."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.0, cy_norm=0.5, distance_m=6.0, dt=0.033)
        assert sp.vz > 0

    def test_target_above_negative_vz(self) -> None:
        """Target above center → negative vz (up, FRD)."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.0, cy_norm=-0.5, distance_m=6.0, dt=0.033)
        assert sp.vz < 0

    def test_target_too_far_positive_vx(self) -> None:
        """Target too far → positive vx (forward)."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.0, cy_norm=0.0, distance_m=10.0, dt=0.033)
        assert sp.vx > 0

    def test_target_too_close_negative_vx(self) -> None:
        """Target too close → negative vx (backward)."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.0, cy_norm=0.0, distance_m=2.0, dt=0.033)
        assert sp.vx < 0

    def test_target_at_desired_distance_near_zero_vx(self) -> None:
        """Target at desired distance → vx near zero."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.0, cy_norm=0.0, distance_m=6.0, dt=0.033)
        assert abs(sp.vx) < 0.1

    def test_centered_target_at_distance_near_zero(self) -> None:
        """Perfectly centered target at desired distance → near-zero output."""
        servo = self._make_servo()
        sp = servo.compute(cx_norm=0.0, cy_norm=0.0, distance_m=6.0, dt=0.033)
        assert abs(sp.yaw_rate) < 0.1
        assert abs(sp.vz) < 0.1
        assert abs(sp.vx) < 0.1
        assert sp.vy == 0.0


class TestVisualServoEdgeCases:
    """Edge cases and fallbacks."""

    def test_no_distance_no_bbox_hover(self) -> None:
        """No distance info → vx = 0 (S10: hover)."""
        servo = VisualServo()
        sp = servo.compute(cx_norm=0.3, cy_norm=0.0, distance_m=None, dt=0.033)
        assert sp.vx == 0.0

    def test_zero_dt_returns_zero(self) -> None:
        servo = VisualServo()
        sp = servo.compute(cx_norm=0.5, cy_norm=0.5, distance_m=10.0, dt=0.0)
        assert sp == VelocitySetpoint.zero()

    def test_bbox_ratio_fallback(self) -> None:
        """When distance_m is None but bbox_h_ratio available, use fallback."""
        servo = VisualServo()
        sp = servo.compute(
            cx_norm=0.0,
            cy_norm=0.0,
            distance_m=None,
            dt=0.033,
            bbox_h_ratio=0.1,  # smaller than desired → too far
        )
        assert sp.vx > 0  # should move forward

    def test_reset_clears_pids(self) -> None:
        servo = VisualServo()
        servo.compute(cx_norm=0.5, cy_norm=0.5, distance_m=10.0, dt=0.033)
        servo.reset()
        # After reset, integral should be zero
        assert servo._yaw_pid.integral == 0.0
        assert servo._alt_pid.integral == 0.0
        assert servo._fwd_pid.integral == 0.0


class TestLatencyCompensation:
    """Latency compensation extrapolates position."""

    def test_latency_compensation_shifts_target(self) -> None:
        servo = VisualServo(
            config=VisualServoConfig(
                yaw_pid=PidConfig(kp=1.0, output_min=-5, output_max=5, deadband=0.0),
            )
        )
        # Target at center, moving right at 0.5 norm/s, 100ms latency
        sp_no_comp = servo.compute(cx_norm=0.0, cy_norm=0.0, distance_m=6.0, dt=0.033)
        servo.reset()
        sp_with_comp = servo.compute(
            cx_norm=0.0,
            cy_norm=0.0,
            distance_m=6.0,
            dt=0.033,
            latency_ms=100.0,
            track_vx_norm=0.5,
        )
        # With compensation, target appears further right → larger yaw_rate
        assert sp_with_comp.yaw_rate > sp_no_comp.yaw_rate
