"""Command messages — GCS → drone (§7.3).

Every message carries: v (PROTOCOL_VERSION), seq, ts_ns.
Drone replies with Ack to every command. GCS retries up to 3× (idempotent by seq).
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, field_validator

from shared.constants import (
    HARD_MAX_FOLLOW_DISTANCE_M,
    HARD_MAX_PID_KD,
    HARD_MAX_PID_KI,
    HARD_MAX_PID_KP,
    HARD_MAX_SPEED,
    HARD_MIN_FOLLOW_DISTANCE_M,
    PROTOCOL_VERSION,
)


class _BaseCommand(BaseModel):
    """Base for all command messages."""

    v: int = Field(default=PROTOCOL_VERSION, description="Protocol version")
    seq: int = Field(description="Sequence number for idempotent ack")
    ts_ns: int = Field(description="Monotonic timestamp in nanoseconds")

    model_config = {"frozen": True}


class LockRequest(_BaseCommand):
    """Pilot tapped the video at (x_norm, y_norm) to lock a target."""

    type: Literal["lock"] = "lock"
    x_norm: float = Field(ge=0.0, le=1.0, description="Normalized X tap position")
    y_norm: float = Field(ge=0.0, le=1.0, description="Normalized Y tap position")


class UnlockRequest(_BaseCommand):
    """Pilot requested to unlock / stop following the target."""

    type: Literal["unlock"] = "unlock"


class PidParams(BaseModel):
    """PID gains for one axis."""

    kp: float = Field(ge=0.0, le=HARD_MAX_PID_KP)
    ki: float = Field(ge=0.0, le=HARD_MAX_PID_KI)
    kd: float = Field(ge=0.0, le=HARD_MAX_PID_KD)

    model_config = {"frozen": True}


class TuneParams(_BaseCommand):
    """GCS sends updated tuning parameters (validated against S8 hard limits).

    If any value exceeds hard limits, the air-unit rejects with Ack{ok=False, reason}.
    """

    type: Literal["tune"] = "tune"
    max_speed: float = Field(ge=0.0, le=HARD_MAX_SPEED, description="Max speed m/s")
    follow_distance_m: float = Field(
        ge=HARD_MIN_FOLLOW_DISTANCE_M,
        le=HARD_MAX_FOLLOW_DISTANCE_M,
        description="Desired follow distance in meters",
    )
    pid_yaw: PidParams = Field(description="Yaw PID gains")
    pid_alt: PidParams = Field(description="Altitude PID gains")
    pid_fwd: PidParams = Field(description="Forward PID gains")

    @field_validator("max_speed")
    @classmethod
    def _clamp_max_speed(cls, v: float) -> float:
        """Ensure max_speed doesn't exceed hard limit (S8)."""
        if v > HARD_MAX_SPEED:
            msg = f"max_speed {v} exceeds HARD_MAX_SPEED {HARD_MAX_SPEED}"
            raise ValueError(msg)
        return v


class Heartbeat(_BaseCommand):
    """GCS heartbeat at 5 Hz — feeds safety rule S5."""

    type: Literal["heartbeat"] = "heartbeat"


class Ack(BaseModel):
    """Drone's acknowledgement to a command.

    Sent in reply to every command. GCS uses (seq) for dedup / retry logic.
    """

    v: int = Field(default=PROTOCOL_VERSION, description="Protocol version")
    seq: int = Field(description="Sequence number of the acked command")
    ok: bool = Field(description="Whether the command was accepted")
    reason: str = Field(default="", description="Rejection reason if ok=False")

    model_config = {"frozen": True}


# Union type for dispatching incoming commands
Command = LockRequest | UnlockRequest | TuneParams | Heartbeat

# Map type tag → model class for decoding
COMMAND_TYPE_MAP: dict[str, type[_BaseCommand]] = {
    "lock": LockRequest,
    "unlock": UnlockRequest,
    "tune": TuneParams,
    "heartbeat": Heartbeat,
}
