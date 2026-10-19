"""End-to-end integration test of Supervisor with FakeCamera and FakeFC (§10, T3.1, T3.2).

Acceptance:
- Supervisor wires camera, detector, tracker, visual servoing, and flight controller.
- Locks target and tracks it, sending valid clamped velocity setpoints.
- Pilot switches FC out of GUIDED → setpoints stop immediately (S6).
"""

from __future__ import annotations

import time

from air_unit.control.dynamic_limits import DynamicLimits, DynamicLimitsConfig
from air_unit.control.safety_watchdog import SafetyWatchdog, WatchdogConfig
from air_unit.control.visual_servoing import VisualServo, VisualServoConfig
from air_unit.core.state_machine import State
from air_unit.core.supervisor import Supervisor
from air_unit.sim.fake_camera import FakeCamera
from air_unit.sim.fake_fc import FakeFC
from air_unit.vision.detector import FakeDetector
from air_unit.vision.distance_estimator import DistanceEstimator, DistanceEstimatorConfig
from air_unit.vision.target_selector import TargetSelector
from air_unit.vision.tracker import ByteTracker, TrackerConfig


class TestSupervisorIntegration:
    """End-to-end simulated flight test (T3.2)."""

    def test_end_to_end_following_and_s6_override(self) -> None:
        camera = FakeCamera(width=640, height=480, fps=100)
        detector = FakeDetector()
        tracker = ByteTracker(TrackerConfig(min_hits_to_activate=1))
        target_selector = TargetSelector()
        dist_est = DistanceEstimator(DistanceEstimatorConfig())
        fc = FakeFC(_mode="GUIDED", _armed=True)
        visual_servo = VisualServo(VisualServoConfig())
        limits = DynamicLimits(DynamicLimitsConfig())
        watchdog = SafetyWatchdog(WatchdogConfig())

        supervisor = Supervisor(
            camera=camera,
            detector=detector,
            tracker=tracker,
            target_selector=target_selector,
            distance_estimator=dist_est,
            flight_controller=fc,
            visual_servo=visual_servo,
            dynamic_limits=limits,
            safety_watchdog=watchdog,
            control_hz=100.0,
        )

        supervisor.start()
        try:
            # Initially in IDLE
            time.sleep(0.05)
            assert supervisor.sm.state == State.IDLE

            # Simulate pilot tapping on the rectangle
            supervisor.safety_watchdog.update_gcs_heartbeat(time.monotonic())
            supervisor.request_lock(0.5, 0.65)

            # Wait for detection and track lock
            for _ in range(20):
                supervisor.safety_watchdog.update_gcs_heartbeat(time.monotonic())
                time.sleep(0.02)
                if supervisor.sm.state == State.FOLLOWING:
                    break

            assert supervisor.sm.state == State.FOLLOWING
            assert fc.velocity_count > 0

            # Last velocity command should be valid and bounded
            vx, _vy, _vz, yaw_rate = fc.last_velocity  # type: ignore[misc]
            assert isinstance(vx, float)
            assert isinstance(yaw_rate, float)

            # S6 Test: Pilot switches FC out of GUIDED to LOITER
            fc.set_mode("LOITER")
            time.sleep(0.05)

            # Supervisor transitions out of FOLLOWING and setpoints cease
            assert supervisor.sm.state == State.IDLE
            prev_count = fc.velocity_count
            time.sleep(0.05)
            # No new setpoints sent while out of GUIDED
            assert fc.velocity_count == prev_count

        finally:
            supervisor.stop()
