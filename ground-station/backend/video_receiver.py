"""Video Receiver — Ground-station side video receiver (§10, T4.2).

Receives UDP datagrams, extracts monotonic timestamp header, measures glass-to-glass latency,
and decodes frame for display on video canvas.
Imports cv2 lazily inside methods (§2.3).
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from collections.abc import Callable

import numpy as np

from shared.constants import DEFAULT_VIDEO_PORT

logger = logging.getLogger(__name__)

FrameCallback = Callable[[np.ndarray, float], None]  # frame, latency_ms


class VideoReceiver:
    """Receives UDP video frames and computes glass-to-glass latency."""

    def __init__(
        self,
        port: int = DEFAULT_VIDEO_PORT,
        bind_host: str = "0.0.0.0",
    ) -> None:
        self.port = port
        self.bind_host = bind_host
        self._running = False
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._callbacks: list[FrameCallback] = []
        self._latest_latency_ms: float = 0.0

    def add_frame_callback(self, cb: FrameCallback) -> None:
        """Register callback for received frames."""
        self._callbacks.append(cb)

    @property
    def latest_latency_ms(self) -> float:
        """Get latest glass-to-glass latency measurement."""
        return self._latest_latency_ms

    def start(self) -> None:
        """Start listening for video packets."""
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_host, self.port))
        self._sock.settimeout(0.2)

        self._thread = threading.Thread(
            target=self._recv_loop,
            name="GcsVideoReceiver",
            daemon=True,
        )
        self._thread.start()
        logger.info("VideoReceiver listening on %s:%d", self.bind_host, self.port)

    def stop(self) -> None:
        """Stop listening and close socket."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._sock:
            self._sock.close()
            self._sock = None

    def _recv_loop(self) -> None:
        while self._running and self._sock is not None:
            try:
                data, _ = self._sock.recvfrom(65536)
                if len(data) <= 8:
                    continue

                # Header is 8 bytes uint64 ts_ns
                (capture_ts_ns,) = struct.unpack(">Q", data[:8])
                now_ns = time.monotonic_ns()
                latency_ms = (now_ns - capture_ts_ns) / 1_000_000.0
                self._latest_latency_ms = latency_ms

                # Remaining bytes is JPEG image
                import cv2  # lazy import (§2.3)

                img_bytes = np.frombuffer(data[8:], dtype=np.uint8)
                frame = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
                if frame is not None:
                    for cb in list(self._callbacks):
                        try:
                            cb(frame, latency_ms)
                        except Exception as e:
                            logger.error("Error in frame callback: %s", e)

            except TimeoutError:
                continue
            except Exception as e:
                if self._running:
                    logger.error("Error in video receiver loop: %s", e)
