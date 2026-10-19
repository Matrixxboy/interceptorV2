"""Video streaming loopback test (§10, T4.2).

Acceptance:
- VideoStreamer sends frame with timestamp header over UDP.
- VideoReceiver receives and decodes frame.
- Measured latency is < 150 ms.
"""

from __future__ import annotations

import time

import numpy as np
from air_unit.comms.video_streamer import VideoStreamer
from ground_station.backend.video_receiver import VideoReceiver


class TestVideoStreaming:
    """T4.2 Video streaming loopback test."""

    def test_local_video_loopback(self) -> None:
        video_port = 15600
        receiver = VideoReceiver(port=video_port, bind_host="127.0.0.1")
        streamer = VideoStreamer(host="127.0.0.1", port=video_port, jpeg_quality=70)

        received_frames: list[tuple[np.ndarray, float]] = []
        receiver.add_frame_callback(lambda frame, latency: received_frames.append((frame, latency)))

        receiver.start()
        streamer.start()

        try:
            # Generate synthetic test frame (160 x 120)
            test_frame = np.zeros((120, 160, 3), dtype=np.uint8)
            test_frame[20:60, 30:70] = (0, 255, 0)

            # Send multiple frames
            for _ in range(5):
                streamer.send_frame(test_frame)
                time.sleep(0.02)

            # Wait for frame delivery
            for _ in range(20):
                if len(received_frames) > 0:
                    break
                time.sleep(0.02)

            assert len(received_frames) > 0
            frame, latency_ms = received_frames[0]
            assert isinstance(frame, np.ndarray)
            assert frame.shape == (120, 160, 3)
            # Glass-to-glass local loopback budget: < 150 ms (§10 T4.2)
            assert latency_ms < 150.0

        finally:
            streamer.stop()
            receiver.stop()
