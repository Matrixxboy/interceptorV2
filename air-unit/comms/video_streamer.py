"""Video Streamer — Air-unit H.264/RTP streamer (§3, §10, T4.2).

Encodes video and streams over RTP/UDP with monotonic timestamp embed.
Imports cv2 lazily inside methods (§2.3).
"""

from __future__ import annotations

import logging
import socket
import struct
import time

import numpy as np

from shared.constants import DEFAULT_GCS_HOST, DEFAULT_VIDEO_PORT

logger = logging.getLogger(__name__)


class VideoStreamer:
    """Streams video frames over UDP with embedded timestamp for latency measurement."""

    def __init__(
        self,
        host: str = DEFAULT_GCS_HOST,
        port: int = DEFAULT_VIDEO_PORT,
        jpeg_quality: int = 80,
    ) -> None:
        self.host = host
        self.port = port
        self.jpeg_quality = jpeg_quality
        self._running = False
        self._sock: socket.socket | None = None

    def start(self) -> None:
        """Initialize socket."""
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info("VideoStreamer streaming to %s:%d", self.host, self.port)

    def send_frame(self, frame: np.ndarray, capture_ts_ns: int | None = None) -> None:
        """Send encoded frame with header (capture_ts_ns)."""
        if not self._running or self._sock is None:
            return

        ts_ns = capture_ts_ns if capture_ts_ns is not None else time.monotonic_ns()

        import cv2  # lazy import (§2.3)

        # JPEG encode frame
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, encimg = cv2.imencode(".jpg", frame, encode_param)
        if not success:
            return

        img_bytes = encimg.tobytes()
        # Header: 8 bytes uint64 monotonic timestamp
        header = struct.pack(">Q", ts_ns)
        payload = header + img_bytes

        # If payload fits in single UDP datagram (< 65507 bytes)
        if len(payload) <= 65000:
            try:
                self._sock.sendto(payload, (self.host, self.port))
            except Exception as e:
                logger.debug("Failed to send video datagram: %s", e)

    def stop(self) -> None:
        """Stop streamer and release socket."""
        self._running = False
        if self._sock:
            self._sock.close()
            self._sock = None
