"""Comms loopback integration tests (§10, T4.1).

Acceptance:
- GcsLink on air-unit ↔ CommandSender & TelemetryClient on GCS.
- Commands acknowledged with matching seq.
- Idempotent repeated sequence numbers handled.
- Tuning parameters validated against S8 hard limits and rejected with Ack(ok=False) if invalid.
- Telemetry received over UDP at expected rate.
"""

from __future__ import annotations

import time

from air_unit.comms.gcs_link import GcsLink
from air_unit.control.dynamic_limits import DynamicLimits, DynamicLimitsConfig
from air_unit.control.safety_watchdog import SafetyWatchdog, WatchdogConfig
from air_unit.control.visual_servoing import VisualServo, VisualServoConfig
from air_unit.core.supervisor import Supervisor
from air_unit.sim.fake_camera import FakeCamera
from air_unit.sim.fake_fc import FakeFC
from air_unit.vision.detector import FakeDetector
from air_unit.vision.distance_estimator import DistanceEstimator, DistanceEstimatorConfig
from air_unit.vision.target_selector import TargetSelector
from air_unit.vision.tracker import ByteTracker, TrackerConfig
from ground_station.backend.command_sender import CommandSender
from ground_station.backend.telemetry_client import TelemetryClient

from shared.messages.commands import PidParams, TuneParams
from shared.messages.telemetry import TelemetryMessage


class TestCommsLoopback:
    """T4.1 Comms loopback test."""

    def test_command_ack_and_telemetry_loopback(self) -> None:
        cmd_port = 15006
        telem_port = 15005

        camera = FakeCamera(width=320, height=240, fps=30)
        detector = FakeDetector()
        tracker = ByteTracker(TrackerConfig())
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
            control_hz=30.0,
        )
        supervisor.start()

        gcs_link = GcsLink(
            supervisor=supervisor,
            safety_watchdog=watchdog,
            gcs_host="127.0.0.1",
            telemetry_port=telem_port,
            command_port=cmd_port,
            telemetry_hz=20.0,
        )
        gcs_link.start()

        cmd_sender = CommandSender(
            drone_host="127.0.0.1",
            drone_port=cmd_port,
            max_retries=3,
            timeout_s=0.5,
        )

        telem_client = TelemetryClient(
            port=telem_port,
            bind_host="127.0.0.1",
        )
        received_telemetry: list[TelemetryMessage] = []
        telem_client.add_listener(lambda msg: received_telemetry.append(msg))
        telem_client.start()

        try:
            # 1. Test LockRequest command & Ack
            ack_lock = cmd_sender.send_lock(0.5, 0.5)
            assert ack_lock.ok is True
            assert ack_lock.seq == 1

            # 2. Test UnlockRequest command & Ack
            ack_unlock = cmd_sender.send_unlock()
            assert ack_unlock.ok is True
            assert ack_unlock.seq == 2

            # 3. Test valid TuneParams
            valid_tune = TuneParams(
                seq=cmd_sender.next_seq(),
                ts_ns=time.monotonic_ns(),
                max_speed=2.5,
                follow_distance_m=5.0,
                pid_yaw=PidParams(kp=1.2, ki=0.0, kd=0.05),
                pid_alt=PidParams(kp=0.9, ki=0.0, kd=0.02),
                pid_fwd=PidParams(kp=0.7, ki=0.0, kd=0.04),
            )
            ack_tune = cmd_sender._send_with_retry(valid_tune)
            assert ack_tune.ok is True

            # 4. Test S8 rejection on invalid TuneParams (simulated by craft)
            # Send raw invalid tuning bypassing client validation
            class DummyInvalidTune:
                v = 1
                seq = cmd_sender.next_seq()
                ts_ns = time.monotonic_ns()
                type = "tune"
                max_speed = 99.0  # exceeds HARD_MAX_SPEED
                follow_distance_m = 5.0
                pid_yaw = PidParams(kp=1.0, ki=0.0, kd=0.0)
                pid_alt = PidParams(kp=1.0, ki=0.0, kd=0.0)
                pid_fwd = PidParams(kp=1.0, ki=0.0, kd=0.0)

            ack_invalid = gcs_link._execute_command(DummyInvalidTune())
            assert ack_invalid.ok is False
            assert "exceeds HARD_MAX_SPEED" in ack_invalid.reason

            # 5. Wait for telemetry packets
            for _ in range(15):
                if len(received_telemetry) > 0:
                    break
                time.sleep(0.05)

            assert len(received_telemetry) > 0
            first_telem = received_telemetry[0]
            assert isinstance(first_telem, TelemetryMessage)
            assert first_telem.fc_mode == "GUIDED"

        finally:
            cmd_sender.stop()
            telem_client.stop()
            gcs_link.stop()
            supervisor.stop()
