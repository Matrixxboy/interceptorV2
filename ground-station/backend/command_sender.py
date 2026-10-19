"""Command Sender for Ground Station (§7.3, §10, T4.1).

- Sends LockRequest, UnlockRequest, TuneParams, Heartbeat.
- Retries up to 3× on packet loss (idempotent seq).
- 50–100 ms debounce on TuneParams to avoid saturating network.
- Sends Heartbeat at 5 Hz (feeds air-unit S5).
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Any

from shared.constants import (
    DEFAULT_COMMAND_PORT,
    HEARTBEAT_INTERVAL_S,
)
from shared.messages.codec import decode_ack, encode_command
from shared.messages.commands import (
    Ack,
    Heartbeat,
    LockRequest,
    TuneParams,
    UnlockRequest,
)

logger = logging.getLogger(__name__)


class CommandSender:
    """Manages sending commands to drone with reliable retry and debounce."""

    def __init__(
        self,
        drone_host: str = "127.0.0.1",
        drone_port: int = DEFAULT_COMMAND_PORT,
        max_retries: int = 3,
        timeout_s: float = 0.25,
        debounce_ms: float = 80.0,
    ) -> None:
        self.drone_host = drone_host
        self.drone_port = drone_port
        self.max_retries = max_retries
        self.timeout_s = timeout_s
        self.debounce_s = debounce_ms / 1000.0

        self._seq = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout_s)
        self._heartbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._lock = threading.Lock()
        self._pending_tune: TuneParams | None = None
        self._tune_timer: threading.Timer | None = None

        self._heartbeat_running = False
        self._heartbeat_thread: threading.Thread | None = None

    def start_heartbeat(self) -> None:
        """Start 5 Hz heartbeat thread."""
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="GcsHeartbeatSender",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def stop(self) -> None:
        """Stop heartbeat and close sockets."""
        self._heartbeat_running = False
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)
        with self._lock:
            if self._tune_timer:
                self._tune_timer.cancel()
        self._sock.close()
        self._heartbeat_sock.close()

    def next_seq(self) -> int:
        """Increment and return sequence number."""
        with self._lock:
            self._seq += 1
            return self._seq

    def send_lock(self, x_norm: float, y_norm: float) -> Ack:
        """Send LockRequest to drone and wait for Ack."""
        cmd = LockRequest(
            seq=self.next_seq(),
            ts_ns=time.monotonic_ns(),
            x_norm=x_norm,
            y_norm=y_norm,
        )
        return self._send_with_retry(cmd)

    def send_unlock(self) -> Ack:
        """Send UnlockRequest to drone and wait for Ack."""
        cmd = UnlockRequest(
            seq=self.next_seq(),
            ts_ns=time.monotonic_ns(),
        )
        return self._send_with_retry(cmd)

    def send_tune(self, tune_cmd: TuneParams, debounce: bool = True) -> None:
        """Queue or send TuneParams with debounce."""
        with self._lock:
            self._pending_tune = tune_cmd
            if self._tune_timer:
                self._tune_timer.cancel()

            if not debounce:
                self._flush_tune()
            else:
                self._tune_timer = threading.Timer(self.debounce_s, self._flush_tune)
                self._tune_timer.daemon = True
                self._tune_timer.start()

    def _flush_tune(self) -> Ack | None:
        with self._lock:
            cmd = self._pending_tune
            self._pending_tune = None
        if cmd is not None:
            return self._send_with_retry(cmd)
        return None

    def _send_with_retry(self, cmd: Any) -> Ack:
        """Send command with retry up to max_retries."""
        data = encode_command(cmd)
        with self._lock:
            for attempt in range(1, self.max_retries + 1):
                try:
                    self._sock.sendto(data, (self.drone_host, self.drone_port))
                    reply, _ = self._sock.recvfrom(4096)
                    ack = decode_ack(reply)
                    if ack.seq == cmd.seq:
                        return ack
                except TimeoutError:
                    logger.warning(
                        "Command %s (seq=%d) timed out (attempt %d/%d)",
                        type(cmd).__name__,
                        cmd.seq,
                        attempt,
                        self.max_retries,
                    )
                except Exception as e:
                    logger.error("Error sending command: %s", e)

        return Ack(
            seq=cmd.seq,
            ok=False,
            reason=f"No Ack received after {self.max_retries} attempts",
        )

    def _heartbeat_loop(self) -> None:
        """Send Heartbeat at 5 Hz."""
        while self._heartbeat_running:
            try:
                cmd = Heartbeat(
                    seq=self.next_seq(),
                    ts_ns=time.monotonic_ns(),
                )
                data = encode_command(cmd)
                self._heartbeat_sock.sendto(data, (self.drone_host, self.drone_port))
            except Exception as e:
                logger.debug("Heartbeat send error: %s", e)
            time.sleep(HEARTBEAT_INTERVAL_S)
