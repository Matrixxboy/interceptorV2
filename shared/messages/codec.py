"""Wire codec — msgpack encode/decode with protocol version checking (§7.3).

All messages are serialized as msgpack. On decode, PROTOCOL_VERSION is checked
and mismatches are rejected with ProtocolVersionError.
"""

from __future__ import annotations

from typing import TypeVar

import msgpack  # type: ignore[import-untyped]
from pydantic import BaseModel, ValidationError

from shared.constants import PROTOCOL_VERSION
from shared.messages.commands import (
    COMMAND_TYPE_MAP,
    Ack,
    Command,
)
from shared.messages.telemetry import TelemetryMessage

T = TypeVar("T", bound=BaseModel)


class ProtocolVersionError(Exception):
    """Raised when a received message has an incompatible protocol version."""

    def __init__(self, received: int, expected: int = PROTOCOL_VERSION) -> None:
        self.received = received
        self.expected = expected
        super().__init__(f"Protocol version mismatch: received v{received}, expected v{expected}")


class CodecError(Exception):
    """Raised for any encoding/decoding error."""


def encode(msg: BaseModel) -> bytes:
    """Serialize a Pydantic model to msgpack bytes.

    Args:
        msg: Any Pydantic model (TelemetryMessage, Command, Ack).

    Returns:
        msgpack-encoded bytes.
    """
    return msgpack.packb(msg.model_dump(), use_bin_type=True)  # type: ignore[no-any-return]


# Aliases
encode_command = encode
encode_telemetry = encode
encode_ack = encode


def _check_version(data: dict[str, object]) -> None:
    """Raise ProtocolVersionError if version field doesn't match."""
    v = data.get("v")
    if v is None:
        raise CodecError("Message missing 'v' (protocol version) field")
    if v != PROTOCOL_VERSION:
        raise ProtocolVersionError(received=int(v))  # type: ignore[arg-type]


def decode_telemetry(raw: bytes) -> TelemetryMessage:
    """Decode msgpack bytes into a TelemetryMessage.

    Args:
        raw: msgpack-encoded bytes.

    Returns:
        Validated TelemetryMessage.

    Raises:
        ProtocolVersionError: If version doesn't match.
        CodecError: If data is malformed.
    """
    try:
        data: dict[str, object] = msgpack.unpackb(raw, raw=False)  # type: ignore[assignment]
    except Exception as e:
        raise CodecError(f"Failed to unpack msgpack: {e}") from e

    _check_version(data)

    try:
        return TelemetryMessage.model_validate(data)
    except ValidationError as e:
        raise CodecError(f"Telemetry validation failed: {e}") from e


def decode_command(raw: bytes) -> Command:
    """Decode msgpack bytes into a typed Command.

    The 'type' field is used to dispatch to the correct Pydantic model.

    Args:
        raw: msgpack-encoded bytes.

    Returns:
        One of LockRequest, UnlockRequest, TuneParams, Heartbeat.

    Raises:
        ProtocolVersionError: If version doesn't match.
        CodecError: If data is malformed or type is unknown.
    """
    try:
        data: dict[str, object] = msgpack.unpackb(raw, raw=False)  # type: ignore[assignment]
    except Exception as e:
        raise CodecError(f"Failed to unpack msgpack: {e}") from e

    _check_version(data)

    msg_type = data.get("type")
    if not isinstance(msg_type, str) or msg_type not in COMMAND_TYPE_MAP:
        raise CodecError(f"Unknown command type: {msg_type!r}")

    model_cls = COMMAND_TYPE_MAP[msg_type]
    try:
        return model_cls.model_validate(data)  # type: ignore[return-value]
    except ValidationError as e:
        raise CodecError(f"Command validation failed for '{msg_type}': {e}") from e


def decode_ack(raw: bytes) -> Ack:
    """Decode msgpack bytes into an Ack.

    Args:
        raw: msgpack-encoded bytes.

    Returns:
        Validated Ack.

    Raises:
        ProtocolVersionError: If version doesn't match.
        CodecError: If data is malformed.
    """
    try:
        data: dict[str, object] = msgpack.unpackb(raw, raw=False)  # type: ignore[assignment]
    except Exception as e:
        raise CodecError(f"Failed to unpack msgpack: {e}") from e

    _check_version(data)

    try:
        return Ack.model_validate(data)
    except ValidationError as e:
        raise CodecError(f"Ack validation failed: {e}") from e
