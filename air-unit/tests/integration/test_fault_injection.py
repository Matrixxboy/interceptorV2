"""Fault injection tests (§10, T3.3).

Acceptance:
- Exception in camera/vision thread → state FAULT, zero setpoint sent (S7, S10).
- Dropped GCS heartbeat → watchdog hover / RTL (S5).
- Dropped target → watchdog hover / LOITER (S4).
"""

from __future__ import annotations

import time

import numpy as np
from air_unit.control.dynamic_limits import DynamicLimits, DynamicLimitsConfig
from air_unit.control.safety_watchdog import SafetyWatchdog, WatchdogConfig
from air_unit.control.visual_servoing import VisualServo, VisualServoConfig
from air_unit.core.state_machine import State
from air_unit.core.supervisor import Supervisor
from air_unit.hardware.interfaces import CameraSource
from air_unit.sim.fake_fc import FakeFC
from air_unit.vision.detector import FakeDetector
from air_unit.vision.distance_estimator import DistanceEstimator, DistanceEstimatorConfig
from air_unit.vision.target_selector import TargetSelector
from air_unit.vision.tracker import ByteTracker, TrackerConfig


class FaultyCamera(CameraSource):
    """Camera that raises an unexpected exception on read."""

    def __init__(self) -> None:
        self._should_fail = False

    def start(self) -> None:
        pass

    def read(self) -> tuple[np.ndarray, int]:
        if self._should_fail:
            raise OSError("Hardware camera sensor disconnected!")
        return np.zeros((100, 100, 3), dtype=np.uint8), time.monotonic_ns()

    def stop(self) -> None:
        pass


class TestFaultInjection:
    """T3.3 Fault injection acceptance tests."""

    def test_camera_crash_triggers_fault_and_hover(self) -> None:
        """S7: Unhandled exception in vision thread triggers FAULT and sends hover."""
        cam = FaultyCamera()
        fc = FakeFC(_mode="GUIDED", _armed=True)

        supervisor = Supervisor(
            camera=cam,
            detector=FakeDetector(),
            tracker=ByteTracker(TrackerConfig()),
            target_selector=TargetSelector(),
            distance_estimator=DistanceEstimator(DistanceEstimatorConfig()),
            flight_controller=fc,
            visual_servo=VisualServo(VisualServoConfig()),
            dynamic_limits=DynamicLimits(DynamicLimitsConfig()),
            safety_watchdog=SafetyWatchdog(WatchdogConfig()),
            control_hz=50.0,
        )

        supervisor.start()
        try:
            time.sleep(0.05)
            # Inject camera crash
            cam._should_fail = True
            time.sleep(0.1)

            # State must transition to FAULT per S7
            assert supervisor.sm.state == State.FAULT
            # Last commanded velocity must be 0 m/s (hover per S10)
            assert fc.last_velocity == (0.0, 0.0, 0.0, 0.0)
        finally:
            supervisor.stop()

    def test_gcs_heartbeat_loss_requests_rtl(self) -> None:
        """S5: Loss of GCS heartbeat triggers RTL."""
        wd = SafetyWatchdog(WatchdogConfig(gcs_timeout_s=0.1, gcs_rtl_s=0.2))
        wd.update_target_seen(now=0.0)
        wd.update_gcs_heartbeat(now=0.0)

        # Heartbeat active at 0.05s
        action1 = wd.check(now=0.05, fc_mode="GUIDED", state="FOLLOWING")
        assert action1.allow_setpoints is True

        # Timeout at 0.15s (> 0.1s)
        action2 = wd.check(now=0.15, fc_mode="GUIDED", state="FOLLOWING")
        assert action2.allow_setpoints is False
        assert "S5" in action2.reason

        # RTL at 0.25s (> 0.2s)
        action3 = wd.check(now=0.25, fc_mode="GUIDED", state="FOLLOWING")
        assert action3.allow_setpoints is False
        assert action3.request_mode == "RTL"
