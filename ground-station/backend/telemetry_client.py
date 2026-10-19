"""Telemetry Client for Ground Station (§7.3, T4.1).

Receives TelemetryMessage packets from air-unit over UDP and notifies listeners.
"""

from __future__ import annotations

import logging
import socket
import threading
from collections.abc import Callable

from shared.constants import DEFAULT_TELEMETRY_PORT
from shared.messages.codec import decode_telemetry
from shared.messages.telemetry import TelemetryMessage

logger = logging.getLogger(__name__)

TelemetryListener = Callable[[TelemetryMessage], None]


class TelemetryClient:
    """UDP client receiving and dispatching air-unit telemetry."""

    def __init__(
        self,
        port: int = DEFAULT_TELEMETRY_PORT,
        bind_host: str = "0.0.0.0",
    ) -> None:
        self.port = port
        self.bind_host = bind_host
        self._running = False
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._listeners: list[TelemetryListener] = []
        self._latest_telemetry: TelemetryMessage | None = None
        self._lock = threading.Lock()

    def add_listener(self, listener: TelemetryListener) -> None:
        """Register a callback for telemetry updates."""
        with self._lock:
            self._listeners.append(listener)

    @property
    def latest(self) -> TelemetryMessage | None:
        """Get most recent telemetry snapshot."""
        with self._lock:
            return self._latest_telemetry

    def start(self) -> None:
        """Start listening for telemetry packets."""
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_host, self.port))
        self._sock.settimeout(0.2)

        self._thread = threading.Thread(
            target=self._recv_loop,
            name="GcsTelemetryClient",
            daemon=True,
        )
        self._thread.start()
        logger.info("TelemetryClient listening on %s:%d", self.bind_host, self.port)

    def stop(self) -> None:
        """Stop listening and release socket."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._sock:
            self._sock.close()
            self._sock = None

    def _recv_loop(self) -> None:
        while self._running and self._sock is not None:
            try:
                data, _ = self._sock.recvfrom(4096)
                msg = decode_telemetry(data)
                with self._lock:
                    self._latest_telemetry = msg
                    listeners = list(self._listeners)

                for listener in listeners:
                    try:
                        listener(msg)
                    except Exception as e:
                        logger.error("Error in telemetry listener callback: %s", e)
            except TimeoutError:
                continue
            except Exception as e:
                if self._running:
                    logger.error("Error in telemetry client receive: %s", e)
