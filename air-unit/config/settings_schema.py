"""Settings schema — Pydantic-validated configuration (§9).

Loads from `air-unit/config/settings.yaml`, validates all fields,
provides typed access. Invalid YAML raises a clear error.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field, field_validator

from shared.constants import (
    HARD_MAX_FOLLOW_DISTANCE_M,
    HARD_MAX_PID_KD,
    HARD_MAX_PID_KI,
    HARD_MAX_PID_KP,
    HARD_MAX_VX,
    HARD_MAX_VZ,
    HARD_MAX_YAW_RATE,
    HARD_MIN_FOLLOW_DISTANCE_M,
)


class CameraSettings(BaseModel):
    """Camera configuration."""

    width: int = Field(default=1280, ge=320, le=3840)
    height: int = Field(default=720, ge=240, le=2160)
    fps: int = Field(default=30, ge=1, le=120)
    source: Literal["picamera2", "v4l2", "file"] = Field(default="picamera2")
    file_path: str | None = Field(default=None, description="Path to video file when source='file'")

    model_config = {"frozen": True}


class FcSettings(BaseModel):
    """Flight controller connection settings."""

    port: str = Field(default="/dev/ttyAMA0")
    baud: int = Field(default=921_600)
    expected_mode: str = Field(default="GUIDED")

    model_config = {"frozen": True}


class VisionSettings(BaseModel):
    """Detection model settings."""

    model: str = Field(default="models/yolov8n.hef")
    conf: float = Field(default=0.35, ge=0.0, le=1.0)
    iou: float = Field(default=0.5, ge=0.0, le=1.0)
    classes: list[int] = Field(default=[0])

    model_config = {"frozen": True}


class ControlSettings(BaseModel):
    """Control loop settings (validated against hard limits S8)."""

    hz: int = Field(default=30, ge=10, le=100)
    max_vx: float = Field(default=3.0, ge=0.0)
    max_vy: float = Field(default=1.5, ge=0.0)
    max_vz: float = Field(default=1.5, ge=0.0)
    max_yaw_rate: float = Field(default=1.2, ge=0.0)
    follow_distance_m: float = Field(default=6.0)

    @field_validator("max_vx")
    @classmethod
    def _clamp_vx(cls, v: float) -> float:
        if v > HARD_MAX_VX:
            msg = f"max_vx {v} exceeds HARD_MAX_VX {HARD_MAX_VX}"
            raise ValueError(msg)
        return v

    @field_validator("max_vz")
    @classmethod
    def _clamp_vz(cls, v: float) -> float:
        if v > HARD_MAX_VZ:
            msg = f"max_vz {v} exceeds HARD_MAX_VZ {HARD_MAX_VZ}"
            raise ValueError(msg)
        return v

    @field_validator("max_yaw_rate")
    @classmethod
    def _clamp_yaw(cls, v: float) -> float:
        if v > HARD_MAX_YAW_RATE:
            msg = f"max_yaw_rate {v} exceeds HARD_MAX_YAW_RATE {HARD_MAX_YAW_RATE}"
            raise ValueError(msg)
        return v

    @field_validator("follow_distance_m")
    @classmethod
    def _clamp_follow(cls, v: float) -> float:
        if v < HARD_MIN_FOLLOW_DISTANCE_M or v > HARD_MAX_FOLLOW_DISTANCE_M:
            msg = (
                f"follow_distance_m {v} out of range "
                f"[{HARD_MIN_FOLLOW_DISTANCE_M}, {HARD_MAX_FOLLOW_DISTANCE_M}]"
            )
            raise ValueError(msg)
        return v

    model_config = {"frozen": True}


class PidGains(BaseModel):
    """PID gains for one axis."""

    kp: float = Field(default=0.0, ge=0.0)
    ki: float = Field(default=0.0, ge=0.0)
    kd: float = Field(default=0.0, ge=0.0)

    @field_validator("kp")
    @classmethod
    def _clamp_kp(cls, v: float) -> float:
        if v > HARD_MAX_PID_KP:
            msg = f"kp {v} exceeds HARD_MAX_PID_KP {HARD_MAX_PID_KP}"
            raise ValueError(msg)
        return v

    @field_validator("ki")
    @classmethod
    def _clamp_ki(cls, v: float) -> float:
        if v > HARD_MAX_PID_KI:
            msg = f"ki {v} exceeds HARD_MAX_PID_KI {HARD_MAX_PID_KI}"
            raise ValueError(msg)
        return v

    @field_validator("kd")
    @classmethod
    def _clamp_kd(cls, v: float) -> float:
        if v > HARD_MAX_PID_KD:
            msg = f"kd {v} exceeds HARD_MAX_PID_KD {HARD_MAX_PID_KD}"
            raise ValueError(msg)
        return v

    model_config = {"frozen": True}


class PidSettings(BaseModel):
    """PID settings for all axes."""

    yaw: PidGains = Field(default_factory=lambda: PidGains(kp=1.0, ki=0.0, kd=0.05))
    alt: PidGains = Field(default_factory=lambda: PidGains(kp=0.8, ki=0.0, kd=0.03))
    fwd: PidGains = Field(default_factory=lambda: PidGains(kp=0.6, ki=0.0, kd=0.05))

    model_config = {"frozen": True}


class SafetySettings(BaseModel):
    """Safety timeout settings."""

    target_lost_timeout_s: float = Field(default=1.0, ge=0.1)
    target_lost_rtl_s: float = Field(default=5.0, ge=1.0)
    gcs_timeout_s: float = Field(default=2.0, ge=0.5)
    gcs_rtl_s: float = Field(default=10.0, ge=2.0)

    model_config = {"frozen": True}


class NetworkSettings(BaseModel):
    """Network connection settings."""

    gcs_host: str = Field(default="192.168.1.10")
    telemetry_port: int = Field(default=5005, ge=1024, le=65535)
    command_port: int = Field(default=5006, ge=1024, le=65535)
    video_port: int = Field(default=5600, ge=1024, le=65535)

    model_config = {"frozen": True}


class Settings(BaseModel):
    """Root settings model — validated from settings.yaml."""

    camera: CameraSettings = Field(default_factory=CameraSettings)
    fc: FcSettings = Field(default_factory=FcSettings)
    vision: VisionSettings = Field(default_factory=VisionSettings)
    control: ControlSettings = Field(default_factory=ControlSettings)
    pid: PidSettings = Field(default_factory=PidSettings)
    safety: SafetySettings = Field(default_factory=SafetySettings)
    network: NetworkSettings = Field(default_factory=NetworkSettings)

    model_config = {"frozen": True}


class SettingsLoadError(Exception):
    """Raised when settings YAML cannot be loaded or validated."""


def load_settings(path: str | Path | None = None) -> Settings:
    """Load and validate settings from a YAML file.

    Args:
        path: Path to settings.yaml. If None, returns defaults.

    Returns:
        Validated Settings instance.

    Raises:
        SettingsLoadError: If the file cannot be read or validation fails.
    """
    if path is None:
        return Settings()

    path = Path(path)
    if not path.exists():
        raise SettingsLoadError(f"Settings file not found: {path}")

    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise SettingsLoadError(f"Invalid YAML in {path}: {e}") from e

    if raw is None:
        return Settings()

    if not isinstance(raw, dict):
        raise SettingsLoadError(f"Settings YAML must be a mapping, got {type(raw).__name__}")

    try:
        return Settings.model_validate(raw)
    except Exception as e:
        raise SettingsLoadError(f"Settings validation failed: {e}") from e
