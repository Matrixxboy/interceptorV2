"""Supervisor: orchestrates vision and control workers (§5, §10, T3.2).

Enforces safety invariants:
- S2: Velocity setpoints sent only in GUIDED and FOLLOWING.
- S7: Any unhandled exception in vision or control thread → FAULT state,
      zero setpoints, dump ring buffer, stay alive.
- S9: Continuous resend at control loop rate (30 Hz).
- S10: Unknown/stale/NaN input → hover (zero velocity).
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from typing import Any

from air_unit.control.dynamic_limits import DynamicLimits
from air_unit.control.safety_watchdog import SafetyWatchdog
from air_unit.control.visual_servoing import VisualServo
from air_unit.core.latest_slot import LatestSlot
from air_unit.core.logging_setup import dump_ring_buffer_on_fault
from air_unit.core.state_machine import Event, State, StateMachine
from air_unit.hardware.interfaces import CameraSource, FlightController
from air_unit.vision.detector import Detector
from air_unit.vision.distance_estimator import DistanceEstimator
from air_unit.vision.interfaces import TrackState
from air_unit.vision.target_selector import TargetSelector
from air_unit.vision.tracker import ByteTracker

from shared.constants import CONTROL_HZ

logger = logging.getLogger(__name__)


class Supervisor:
    """Central supervisor coordinating camera, detector, tracker, FC, and control loop."""

    def __init__(
        self,
        camera: CameraSource,
        detector: Detector,
        tracker: ByteTracker,
        target_selector: TargetSelector,
        distance_estimator: DistanceEstimator,
        flight_controller: FlightController,
        visual_servo: VisualServo,
        dynamic_limits: DynamicLimits,
        safety_watchdog: SafetyWatchdog,
        state_machine: StateMachine | None = None,
        control_hz: float = float(CONTROL_HZ),
        video_streamer: Any | None = None,
    ) -> None:
        self.camera = camera
        self.detector = detector
        self.tracker = tracker
        self.target_selector = target_selector
        self.distance_estimator = distance_estimator
        self.fc = flight_controller
        self.visual_servo = visual_servo
        self.dynamic_limits = dynamic_limits
        self.safety_watchdog = safety_watchdog
        self.sm = state_machine or StateMachine()
        self.control_hz = control_hz
        self.video_streamer = video_streamer

        self.track_slot: LatestSlot[TrackState] = LatestSlot()
        self._running = False
        self._vision_thread: threading.Thread | None = None
        self._control_thread: threading.Thread | None = None
        self._last_control_time = 0.0

    def start(self) -> None:
        """Start camera, workers, and transition state to IDLE."""
        self._running = True
        self.camera.start()
        if self.video_streamer is not None and hasattr(self.video_streamer, "start"):
            with contextlib.suppress(Exception):
                self.video_streamer.start()
        self.sm.handle(Event.SYSTEM_READY)

        self._vision_thread = threading.Thread(
            target=self._vision_worker,
            name="VisionWorker",
            daemon=True,
        )
        self._control_thread = threading.Thread(
            target=self._control_worker,
            name="ControlWorker",
            daemon=True,
        )

        self._vision_thread.start()
        self._control_thread.start()
        logger.info("Supervisor started in state %s", self.sm.state.value)

    def stop(self) -> None:
        """Stop all workers and camera safely."""
        self._running = False
        if self._vision_thread and self._vision_thread.is_alive():
            self._vision_thread.join(timeout=1.0)
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=1.0)
        self.camera.stop()
        if self.video_streamer is not None and hasattr(self.video_streamer, "stop"):
            with contextlib.suppress(Exception):
                self.video_streamer.stop()
        self.fc.close()
        logger.info("Supervisor stopped")

    def request_lock(self, tap_x_norm: float, tap_y_norm: float) -> None:
        """Pilot tapped video to lock target."""
        fc_state = self.fc.state()
        if fc_state.mode != "GUIDED":
            logger.warning("Lock rejected: FC not in GUIDED (mode=%s)", fc_state.mode)
            return

        if self.sm.state in (State.FOLLOWING, State.TARGET_LOST):
            # Pilot tapped to switch or re-lock target
            self.target_selector.unlock()
            self.sm.state = State.IDLE

        res = self.sm.handle(Event.LOCK_REQUEST)
        if res == State.SELECTING or self.sm.state == State.SELECTING:
            self._selecting_start_time = time.monotonic()
            self.target_selector.tap(tap_x_norm, tap_y_norm)
            logger.info("Lock request queued at (%.3f, %.3f)", tap_x_norm, tap_y_norm)

    def request_unlock(self) -> None:
        """Pilot requested unlock."""
        self.target_selector.unlock()
        self.sm.handle(Event.UNLOCK_REQUEST)

    def _vision_worker(self) -> None:
        """Vision worker loop: camera → detect → track → distance → TrackState."""
        self._selecting_start_time = time.monotonic()
        while self._running:
            try:
                frame, ts_ns = self.camera.read()
                h, w = frame.shape[:2]

                if self.video_streamer is not None and hasattr(self.video_streamer, "send_frame"):
                    with contextlib.suppress(Exception):
                        self.video_streamer.send_frame(frame, ts_ns)

                detections = self.detector.infer(frame)
                tracks = self.tracker.update(detections)

                # Target selection & tracking logic
                curr_state = self.sm.state
                target = None
                distance_m = None

                if curr_state == State.SELECTING:
                    target = self.target_selector.select_target(tracks, (w, h))
                    if target is not None:
                        self.sm.handle(Event.TRACK_FOUND)
                        self.visual_servo.reset()
                        self.dynamic_limits.reset()
                    elif (
                        hasattr(self, "_selecting_start_time")
                        and time.monotonic() - self._selecting_start_time > 1.5
                    ):
                        logger.info("Selection timeout: no matching track near tap")
                        self.sm.handle(Event.LOCK_TIMEOUT)
                        self.target_selector._pending_tap = None
                elif curr_state in (State.FOLLOWING, State.TARGET_LOST):
                    target = self.target_selector.reacquire(tracks, (w, h))
                    if target is not None:
                        if curr_state == State.TARGET_LOST:
                            self.sm.handle(Event.TARGET_REACQUIRED)
                        # Distance estimation
                        distance_m = self.distance_estimator.estimate(
                            bbox_h_px=target.bbox, frame_h=(w, h), cls=0
                        )
                        # Notify watchdog
                        now_s = time.monotonic()
                        self.safety_watchdog.update_target_seen(now_s)
                    else:
                        if curr_state == State.FOLLOWING:
                            self.sm.handle(Event.TARGET_MISSING)

                now_ns = time.monotonic_ns()
                latency_ms = (now_ns - ts_ns) / 1_000_000.0

                track_state = TrackState(
                    capture_ts_ns=ts_ns,
                    frame_w=w,
                    frame_h=h,
                    target=target,
                    distance_m=distance_m,
                    latency_ms=latency_ms,
                )
                self.track_slot.put(track_state)

            except Exception as e:
                # S7: Any unhandled exception in vision or control thread → state FAULT
                logger.critical("S7: Unhandled exception in VisionWorker: %s", e, exc_info=True)
                self.sm.handle(Event.FAULT_DETECTED)
                dump_ring_buffer_on_fault()
                self._send_hover_setpoint()
                time.sleep(0.1)

    def _control_worker(self) -> None:
        """Control worker loop: 30 Hz fixed-rate visual servoing & safety checks."""
        dt = 1.0 / self.control_hz
        self._last_control_time = time.monotonic()

        while self._running:
            try:
                loop_start = time.monotonic()
                fc_state = self.fc.state()
                now_s = loop_start

                # Pilot takeover check (S6)
                if fc_state.mode != "GUIDED" and self.sm.state in (
                    State.FOLLOWING,
                    State.TARGET_LOST,
                ):
                    logger.warning("S6: Pilot switched FC out of GUIDED to %s", fc_state.mode)
                    self.sm.handle(Event.MODE_NOT_GUIDED)

                # Safety watchdog check (S2, S4, S5, S6)
                action = self.safety_watchdog.check(
                    now=now_s,
                    fc_mode=fc_state.mode,
                    state=self.sm.state.value,
                )

                if action.request_mode is not None:
                    self.fc.request_mode(action.request_mode)  # type: ignore[arg-type]

                if action.allow_setpoints and self.sm.state == State.FOLLOWING:
                    track_state = self.track_slot.get()
                    if track_state is not None and track_state.target is not None:
                        sp = self.visual_servo.step(
                            target_bbox=track_state.target.bbox,
                            frame_size=(track_state.frame_w, track_state.frame_h),
                            distance_m=track_state.distance_m,
                            dt=dt,
                            latency_ms=track_state.latency_ms,
                        )
                        # S3 & S8: Dynamic limits clamp
                        clamped_sp = self.dynamic_limits.clamp(
                            sp,
                            dt=dt,
                            battery_v=fc_state.battery_v,
                        )
                        self.fc.send_velocity(
                            clamped_sp.vx,
                            clamped_sp.vy,
                            clamped_sp.vz,
                            clamped_sp.yaw_rate,
                        )
                    else:
                        # S10: No fresh target -> zero velocity hover
                        self._send_hover_setpoint()
                else:
                    if fc_state.mode == "GUIDED" and self.sm.state != State.IDLE:
                        # Hover if in guided but setpoints not allowed (e.g. S4/S5/S10)
                        self._send_hover_setpoint()

                # Sleep to maintain fixed control rate
                elapsed = time.monotonic() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except Exception as e:
                # S7: Any unhandled exception in vision or control thread → state FAULT
                logger.critical("S7: Unhandled exception in ControlWorker: %s", e, exc_info=True)
                self.sm.handle(Event.FAULT_DETECTED)
                dump_ring_buffer_on_fault()
                self._send_hover_setpoint()
                time.sleep(0.1)

    def _send_hover_setpoint(self) -> None:
        """Send 0 m/s velocity setpoint (hover per S10)."""
        with contextlib.suppress(Exception):
            self.fc.send_velocity(0.0, 0.0, 0.0, 0.0)
