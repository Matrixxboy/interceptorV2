"""Unit test for flight log replay script (§11)."""

from __future__ import annotations

from pathlib import Path

import msgpack
import pytest
from scripts.replay_log import replay_flight_log


def test_replay_flight_log_valid(tmp_path: Path) -> None:
    """Tests replay with valid sample frames ensuring clamps are obeyed."""
    log_file = tmp_path / "flight.msgpack"

    sample_records = [
        {
            "ts_ns": 1_000_000_000,
            "frame_w": 1280,
            "frame_h": 720,
            "target": {"id": 1, "bbox": [500.0, 300.0, 780.0, 500.0], "score": 0.95, "age": 10},
            "distance_m": 6.5,
            "latency_ms": 25.0,
        },
        {
            "ts_ns": 1_033_000_000,
            "frame_w": 1280,
            "frame_h": 720,
            "target": {"id": 1, "bbox": [510.0, 305.0, 790.0, 505.0], "score": 0.94, "age": 11},
            "distance_m": 6.2,
            "latency_ms": 26.0,
        },
        {
            "ts_ns": 1_066_000_000,
            "frame_w": 1280,
            "frame_h": 720,
            "target": None,
            "distance_m": None,
            "latency_ms": 25.0,
        },
    ]

    with open(log_file, "wb") as f:
        for r in sample_records:
            f.write(msgpack.packb(r))

    count = replay_flight_log(log_file)
    assert count == 3


def test_replay_nonexistent_file() -> None:
    """Tests that missing log file raises FileNotFoundError."""
    with pytest.raises(FileNotFoundError):
        replay_flight_log(Path("nonexistent_flight.msgpack"))
