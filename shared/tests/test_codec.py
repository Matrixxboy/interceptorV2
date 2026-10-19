"""Tests for shared/messages/codec.py — T0.2 acceptance tests.

- Round-trip encode/decode for every message type
- Protocol version mismatch is rejected
- Malformed data raises CodecError
"""

from __future__ import annotations

import time

import msgpack  # type: ignore[import-untyped]
import pytest

from shared.constants import PROTOCOL_VERSION
from shared.messages.codec import (
    CodecError,
    ProtocolVersionError,
    decode_ack,
    decode_command,
    decode_telemetry,
    encode,
)
from shared.messages.commands import (
    Ack,
    Heartbeat,
    LockRequest,
    PidParams,
    TuneParams,
    UnlockRequest,
)
from shared.messages.telemetry import TelemetryMessage

# ── Helpers ──────────────────────────────────


def _ts() -> int:
    return time.monotonic_ns()


# ── Telemetry round-trip ─────────────────────


class TestTelemetryRoundTrip:
    """Round-trip encode/decode for TelemetryMessage."""

    def test_minimal_telemetry(self) -> None:
        msg = TelemetryMessage(seq=1, ts_ns=_ts(), state="IDLE")
        raw = encode(msg)
        decoded = decode_telemetry(raw)
        assert decoded.seq == 1
        assert decoded.state == "IDLE"
        assert decoded.v == PROTOCOL_VERSION

    def test_full_telemetry(self) -> None:
        msg = TelemetryMessage(
            seq=42,
            ts_ns=_ts(),
            state="FOLLOWING",
            fc_mode="GUIDED",
            armed=True,
            battery_v=16.2,
            battery_pct=85.0,
            target_lock=True,
            bbox_norm=[0.1, 0.2, 0.5, 0.8],
            distance_m=6.5,
            fps=30.0,
            latency_ms=12.3,
            fault=None,
        )
        raw = encode(msg)
        decoded = decode_telemetry(raw)
        assert decoded.seq == 42
        assert decoded.state == "FOLLOWING"
        assert decoded.target_lock is True
        assert decoded.bbox_norm == [0.1, 0.2, 0.5, 0.8]
        assert decoded.distance_m == 6.5
        assert decoded.battery_v == 16.2

    def test_telemetry_with_fault(self) -> None:
        msg = TelemetryMessage(seq=3, ts_ns=_ts(), state="FAULT", fault="Camera disconnected")
        raw = encode(msg)
        decoded = decode_telemetry(raw)
        assert decoded.state == "FAULT"
        assert decoded.fault == "Camera disconnected"


# ── Command round-trip ───────────────────────


class TestCommandRoundTrip:
    """Round-trip encode/decode for all Command types."""

    def test_lock_request(self) -> None:
        msg = LockRequest(seq=1, ts_ns=_ts(), x_norm=0.5, y_norm=0.3)
        raw = encode(msg)
        decoded = decode_command(raw)
        assert isinstance(decoded, LockRequest)
        assert decoded.x_norm == 0.5
        assert decoded.y_norm == 0.3

    def test_unlock_request(self) -> None:
        msg = UnlockRequest(seq=2, ts_ns=_ts())
        raw = encode(msg)
        decoded = decode_command(raw)
        assert isinstance(decoded, UnlockRequest)
        assert decoded.seq == 2

    def test_tune_params(self) -> None:
        msg = TuneParams(
            seq=3,
            ts_ns=_ts(),
            max_speed=3.0,
            follow_distance_m=6.0,
            pid_yaw=PidParams(kp=1.0, ki=0.0, kd=0.05),
            pid_alt=PidParams(kp=0.8, ki=0.0, kd=0.03),
            pid_fwd=PidParams(kp=0.6, ki=0.0, kd=0.05),
        )
        raw = encode(msg)
        decoded = decode_command(raw)
        assert isinstance(decoded, TuneParams)
        assert decoded.max_speed == 3.0
        assert decoded.pid_yaw.kp == 1.0

    def test_heartbeat(self) -> None:
        msg = Heartbeat(seq=4, ts_ns=_ts())
        raw = encode(msg)
        decoded = decode_command(raw)
        assert isinstance(decoded, Heartbeat)

    def test_command_preserves_seq(self) -> None:
        """Seq numbers survive round-trip for idempotent ack matching."""
        for seq in [0, 1, 999, 2**31]:
            msg = Heartbeat(seq=seq, ts_ns=_ts())
            decoded = decode_command(encode(msg))
            assert decoded.seq == seq


# ── Ack round-trip ───────────────────────────


class TestAckRoundTrip:
    """Round-trip encode/decode for Ack messages."""

    def test_ack_ok(self) -> None:
        msg = Ack(seq=1, ok=True)
        raw = encode(msg)
        decoded = decode_ack(raw)
        assert decoded.ok is True
        assert decoded.seq == 1

    def test_ack_rejected(self) -> None:
        msg = Ack(seq=2, ok=False, reason="max_speed exceeds HARD_MAX_SPEED")
        raw = encode(msg)
        decoded = decode_ack(raw)
        assert decoded.ok is False
        assert "HARD_MAX_SPEED" in decoded.reason


# ── Version mismatch rejection ───────────────


class TestProtocolVersionCheck:
    """Protocol version mismatches must be rejected (T0.2 acceptance)."""

    def test_telemetry_version_mismatch(self) -> None:
        data = {"v": PROTOCOL_VERSION + 1, "seq": 1, "ts_ns": 0, "state": "IDLE"}
        raw = msgpack.packb(data, use_bin_type=True)
        with pytest.raises(ProtocolVersionError) as exc_info:
            decode_telemetry(raw)
        assert exc_info.value.received == PROTOCOL_VERSION + 1

    def test_command_version_mismatch(self) -> None:
        data = {"v": 999, "seq": 1, "ts_ns": 0, "type": "heartbeat"}
        raw = msgpack.packb(data, use_bin_type=True)
        with pytest.raises(ProtocolVersionError):
            decode_command(raw)

    def test_ack_version_mismatch(self) -> None:
        data = {"v": 0, "seq": 1, "ok": True}
        raw = msgpack.packb(data, use_bin_type=True)
        with pytest.raises(ProtocolVersionError):
            decode_ack(raw)

    def test_missing_version_field(self) -> None:
        data = {"seq": 1, "ts_ns": 0, "state": "IDLE"}
        raw = msgpack.packb(data, use_bin_type=True)
        with pytest.raises(CodecError, match="missing 'v'"):
            decode_telemetry(raw)


# ── Malformed data ───────────────────────────


class TestMalformedData:
    """Malformed data raises CodecError."""

    def test_invalid_msgpack(self) -> None:
        with pytest.raises(CodecError, match="unpack"):
            decode_telemetry(b"\x00\x01\x02bad")

    def test_unknown_command_type(self) -> None:
        data = {"v": PROTOCOL_VERSION, "seq": 1, "ts_ns": 0, "type": "launch_missile"}
        raw = msgpack.packb(data, use_bin_type=True)
        with pytest.raises(CodecError, match="Unknown command type"):
            decode_command(raw)

    def test_missing_required_fields(self) -> None:
        data = {"v": PROTOCOL_VERSION}  # missing seq, ts_ns, state
        raw = msgpack.packb(data, use_bin_type=True)
        with pytest.raises(CodecError, match="validation failed"):
            decode_telemetry(raw)
