"""GCS Link — Air-unit side UDP communications (§7.3, §10, T4.1).

- Sends TelemetryMessage at 10–20 Hz to GCS host.
- Listens for commands: LockRequest, UnlockRequest, TuneParams, Heartbeat.
- Idempotent Ack response for all commands (tracking seq).
- Rejects TuneParams exceeding S8 hard limits with Ack(ok=False).
- Feeds GCS heartbeat to SafetyWatchdog.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Any

from air_unit.control.safety_watchdog import SafetyWatchdog
from air_unit.core.supervisor import Supervisor

from shared.constants import (
    DEFAULT_COMMAND_PORT,
    DEFAULT_GCS_HOST,
    DEFAULT_TELEMETRY_PORT,
    HARD_MAX_FOLLOW_DISTANCE_M,
    HARD_MAX_PID_KD,
    HARD_MAX_PID_KI,
    HARD_MAX_PID_KP,
    HARD_MAX_SPEED,
    HARD_MIN_FOLLOW_DISTANCE_M,
    TELEMETRY_HZ,
)
from shared.messages.codec import (
    decode_command,
    encode_ack,
    encode_telemetry,
)
from shared.messages.commands import (
    Ack,
    Heartbeat,
    LockRequest,
    TuneParams,
    UnlockRequest,
)
from shared.messages.telemetry import TelemetryMessage

logger = logging.getLogger(__name__)


class GcsLink:
    """Manages UDP telemetry broadcast and command reception on the air-unit."""

    def __init__(
        self,
        supervisor: Supervisor,
        safety_watchdog: SafetyWatchdog,
        gcs_host: str = DEFAULT_GCS_HOST,
        telemetry_port: int = DEFAULT_TELEMETRY_PORT,
        command_port: int = DEFAULT_COMMAND_PORT,
        telemetry_hz: float = TELEMETRY_HZ,
    ) -> None:
        self.supervisor = supervisor
        self.safety_watchdog = safety_watchdog
        self.gcs_host = gcs_host
        self.telemetry_port = telemetry_port
        self.command_port = command_port
        self.telemetry_interval = 1.0 / telemetry_hz

        self._running = False
        self._seq = 0
        self._processed_seqs: dict[int, Ack] = {}  # Cache recent Acks for idempotency

        self._cmd_sock: socket.socket | None = None
        self._telem_sock: socket.socket | None = None
        self._cmd_thread: threading.Thread | None = None
        self._telem_thread: threading.Thread | None = None

    def start(self) -> None:
        """Open sockets and start background threads."""
        self._running = True

        # Sockets
        self._telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cmd_sock.bind(("0.0.0.0", self.command_port))
        self._cmd_sock.settimeout(0.2)

        self._cmd_thread = threading.Thread(
            target=self._command_receiver_loop,
            name="GcsCommandReceiver",
            daemon=True,
        )
        self._telem_thread = threading.Thread(
            target=self._telemetry_sender_loop,
            name="GcsTelemetrySender",
            daemon=True,
        )

        self._cmd_thread.start()
        self._telem_thread.start()
        logger.info(
            "GcsLink started: commands on port %d, telemetry -> %s:%d",
            self.command_port,
            self.gcs_host,
            self.telemetry_port,
        )

    def stop(self) -> None:
        """Stop threads and close sockets."""
        self._running = False
        if self._cmd_thread and self._cmd_thread.is_alive():
            self._cmd_thread.join(timeout=1.0)
        if self._telem_thread and self._telem_thread.is_alive():
            self._telem_thread.join(timeout=1.0)

        if self._cmd_sock:
            self._cmd_sock.close()
            self._cmd_sock = None
        if self._telem_sock:
            self._telem_sock.close()
            self._telem_sock = None
        logger.info("GcsLink stopped")

    def _command_receiver_loop(self) -> None:
        """Receive and dispatch commands from GCS."""
        while self._running and self._cmd_sock is not None:
            try:
                data, addr = self._cmd_sock.recvfrom(4096)
                ack = self.handle_command_bytes(data)
                # Send ack back to sender
                ack_bytes = encode_ack(ack)
                self._cmd_sock.sendto(ack_bytes, addr)
            except TimeoutError:
                continue
            except Exception as e:
                if self._running:
                    logger.error("Error in command receiver: %s", e)

    def handle_command_bytes(self, data: bytes) -> Ack:
        """Decode and execute command bytes, returning Ack."""
        try:
            cmd = decode_command(data)
        except Exception as e:
            logger.warning("Failed to decode command: %s", e)
            return Ack(seq=0, ok=False, reason=f"Decode error: {e}")

        # Check idempotent cache
        if cmd.seq in self._processed_seqs:
            return self._processed_seqs[cmd.seq]

        ack = self._execute_command(cmd)
        # Keep cache bounded
        if len(self._processed_seqs) > 1000:
            self._processed_seqs.clear()
        self._processed_seqs[cmd.seq] = ack
        return ack

    def _execute_command(self, cmd: Any) -> Ack:
        """Execute validated command model."""
        if isinstance(cmd, Heartbeat):
            self.safety_watchdog.update_gcs_heartbeat(time.monotonic())
            return Ack(seq=cmd.seq, ok=True)

        if isinstance(cmd, LockRequest):
            self.safety_watchdog.update_gcs_heartbeat(time.monotonic())
            self.supervisor.request_lock(cmd.x_norm, cmd.y_norm)
            return Ack(seq=cmd.seq, ok=True)

        if isinstance(cmd, UnlockRequest):
            self.safety_watchdog.update_gcs_heartbeat(time.monotonic())
            self.supervisor.request_unlock()
            return Ack(seq=cmd.seq, ok=True)

        if isinstance(cmd, TuneParams) or getattr(cmd, "type", "") == "tune":
            self.safety_watchdog.update_gcs_heartbeat(time.monotonic())
            # S8 check
            if cmd.max_speed > HARD_MAX_SPEED:
                return Ack(
                    seq=cmd.seq,
                    ok=False,
                    reason=f"max_speed {cmd.max_speed} exceeds HARD_MAX_SPEED {HARD_MAX_SPEED}",
                )
            if not (
                HARD_MIN_FOLLOW_DISTANCE_M <= cmd.follow_distance_m <= HARD_MAX_FOLLOW_DISTANCE_M
            ):
                return Ack(
                    seq=cmd.seq,
                    ok=False,
                    reason="follow_distance_m out of range",
                )
            for name, pid in [
                ("yaw", cmd.pid_yaw),
                ("alt", cmd.pid_alt),
                ("fwd", cmd.pid_fwd),
            ]:
                if pid.kp > HARD_MAX_PID_KP or pid.ki > HARD_MAX_PID_KI or pid.kd > HARD_MAX_PID_KD:
                    return Ack(
                        seq=cmd.seq,
                        ok=False,
                        reason=f"{name} PID gains exceed hard limits",
                    )

            # Apply tuning
            self.supervisor.dynamic_limits.update_gcs_limits(max_vx=cmd.max_speed)
            # Update visual servo distance & gains
            cfg = self.supervisor.visual_servo.config
            cfg.follow_distance_m = cmd.follow_distance_m
            cfg.yaw_pid.kp, cfg.yaw_pid.ki, cfg.yaw_pid.kd = (
                cmd.pid_yaw.kp,
                cmd.pid_yaw.ki,
                cmd.pid_yaw.kd,
            )
            cfg.alt_pid.kp, cfg.alt_pid.ki, cfg.alt_pid.kd = (
                cmd.pid_alt.kp,
                cmd.pid_alt.ki,
                cmd.pid_alt.kd,
            )
            cfg.fwd_pid.kp, cfg.fwd_pid.ki, cfg.fwd_pid.kd = (
                cmd.pid_fwd.kp,
                cmd.pid_fwd.ki,
                cmd.pid_fwd.kd,
            )
            self.supervisor.visual_servo.update_config(cfg)
            return Ack(seq=cmd.seq, ok=True)

        return Ack(seq=getattr(cmd, "seq", 0), ok=False, reason="Unknown command type")

    def _telemetry_sender_loop(self) -> None:
        """Periodic loop broadcasting TelemetryMessage to GCS."""
        while self._running and self._telem_sock is not None:
            try:
                start = time.monotonic()
                msg = self._build_telemetry_message()
                data = encode_telemetry(msg)
                self._telem_sock.sendto(data, (self.gcs_host, self.telemetry_port))

                elapsed = time.monotonic() - start
                sleep_time = self.telemetry_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                if self._running:
                    logger.error("Error in telemetry sender: %s", e)
                    time.sleep(0.1)

    def _build_telemetry_message(self) -> TelemetryMessage:
        """Construct current TelemetryMessage snapshot."""
        self._seq += 1
        fc_state = self.supervisor.fc.state()
        track_state = self.supervisor.track_slot.get()

        target_lock = False
        bbox_norm = None
        distance_m = None
        latency_ms = 0.0

        if track_state is not None:
            latency_ms = track_state.latency_ms
            if track_state.target is not None:
                target_lock = True
                distance_m = track_state.distance_m
                w, h = track_state.frame_w, track_state.frame_h
                x1, y1, x2, y2 = track_state.target.bbox
                if w > 0 and h > 0:
                    bbox_norm = [x1 / w, y1 / h, x2 / w, y2 / h]

        return TelemetryMessage(
            seq=self._seq,
            ts_ns=time.monotonic_ns(),
            state=self.supervisor.sm.state.value,
            fc_mode=fc_state.mode,
            armed=fc_state.armed,
            battery_v=fc_state.battery_v,
            battery_pct=fc_state.battery_pct,
            target_lock=target_lock,
            bbox_norm=bbox_norm,
            distance_m=distance_m,
            fps=float(getattr(self.supervisor.camera, "fps", 30)),
            latency_ms=latency_ms,
            fault=None if not self.supervisor.sm.is_fault() else "System fault detected",
        )
