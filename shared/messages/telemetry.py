"""Telemetry messages — drone → GCS (§7.3).

Every message carries: v (PROTOCOL_VERSION), seq, ts_ns.
Telemetry is sent at 10–20 Hz from air-unit to ground-station.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from shared.constants import PROTOCOL_VERSION


class TelemetryMessage(BaseModel):
    """Telemetry payload sent from drone to GCS at 10–20 Hz.

    Fields match §7.3 spec exactly.
    """

    v: int = Field(default=PROTOCOL_VERSION, description="Protocol version")
    seq: int = Field(description="Sequence number")
    ts_ns: int = Field(description="Monotonic timestamp in nanoseconds")

    # State
    state: str = Field(description="Current state machine state (IDLE, FOLLOWING, etc.)")
    fc_mode: str = Field(default="UNKNOWN", description="ArduPilot flight mode")
    armed: bool = Field(default=False, description="Whether the FC is armed")

    # Battery
    battery_v: float = Field(default=0.0, description="Battery voltage")
    battery_pct: float = Field(default=-1.0, description="Battery percentage (-1 = unknown)")

    # Target
    target_lock: bool = Field(default=False, description="Whether a target is locked")
    bbox_norm: list[float] | None = Field(
        default=None,
        description="Normalized bounding box [x1, y1, x2, y2] or None",
    )
    distance_m: float | None = Field(
        default=None, description="Estimated distance to target in meters"
    )

    # Performance
    fps: float = Field(default=0.0, description="Current detection FPS")
    latency_ms: float = Field(default=0.0, description="End-to-end latency in ms")

    # Fault
    fault: str | None = Field(default=None, description="Fault description or None")

    model_config = {"frozen": True}
