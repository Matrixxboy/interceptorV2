"""Visual servoing — image error → velocity setpoints (§8, T1.4).

Inputs: target bbox center (cx, cy) normalized to [-1, 1], estimated distance d,
desired distance d*.

Outputs: VelocitySetpoint (vx, vy, vz, yaw_rate) in body frame FRD.

Sign conventions (§13):
- Body frame is FRD (Forward-Right-Down)
- vz positive = DOWN
- yaw_rate positive = clockwise (right turn)
- Target right of center → positive yaw_rate
- Target below center → positive vz (down)
- Target too far → positive vx (forward)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from air_unit.control.pid import PidConfig, PidController


@dataclass(frozen=True)
class VelocitySetpoint:
    """Body-frame velocity setpoint (FRD).

    vx: forward (m/s, positive = forward)
    vy: right (m/s, positive = right)
    vz: down (m/s, positive = down)
    yaw_rate: clockwise (rad/s, positive = clockwise)
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0

    @staticmethod
    def zero() -> VelocitySetpoint:
        """Return a zero (hover) setpoint — the safe default (S10)."""
        return VelocitySetpoint()


@dataclass
class VisualServoConfig:
    """Visual servoing configuration."""

    # PID configs for each axis
    yaw_pid: PidConfig = field(
        default_factory=lambda: PidConfig(
            kp=1.0,
            ki=0.0,
            kd=0.05,
            output_min=-2.0,
            output_max=2.0,
            deadband=0.03,
        )
    )
    alt_pid: PidConfig = field(
        default_factory=lambda: PidConfig(
            kp=0.8,
            ki=0.0,
            kd=0.03,
            output_min=-1.5,
            output_max=1.5,
            deadband=0.03,
        )
    )
    fwd_pid: PidConfig = field(
        default_factory=lambda: PidConfig(
            kp=0.6,
            ki=0.0,
            kd=0.05,
            output_min=-3.0,
            output_max=3.0,
            deadband=0.0,
        )
    )

    follow_distance_m: float = 6.0
    lateral_mode: bool = False  # optional lateral tracking (behind config flag)


@dataclass
class VisualServo:
    """Converts image-space tracking error into body-frame velocity setpoints.

    Uses three PID controllers:
    - yaw_rate = PID_yaw(cx)   — target right of center → positive yaw_rate
    - vz      = PID_alt(cy)    — target below center → positive vz (down)
    - vx      = PID_fwd(d-d*)  — target too far → positive vx (forward)

    Latency compensation: extrapolate bbox by track velocity × latency_ms.
    """

    config: VisualServoConfig = field(default_factory=VisualServoConfig)
    _yaw_pid: PidController = field(init=False)
    _alt_pid: PidController = field(init=False)
    _fwd_pid: PidController = field(init=False)

    def __post_init__(self) -> None:
        self._yaw_pid = PidController(config=self.config.yaw_pid)
        self._alt_pid = PidController(config=self.config.alt_pid)
        self._fwd_pid = PidController(config=self.config.fwd_pid)

    def compute(
        self,
        cx_norm: float,
        cy_norm: float,
        distance_m: float | None,
        dt: float,
        bbox_h_ratio: float | None = None,
        latency_ms: float = 0.0,
        track_vx_norm: float = 0.0,
        track_vy_norm: float = 0.0,
    ) -> VelocitySetpoint:
        """Compute velocity setpoints from image-space error.

        Args:
            cx_norm: Target center X, normalized to [-1, 1] (positive = right).
            cy_norm: Target center Y, normalized to [-1, 1] (positive = down).
            distance_m: Estimated distance to target (meters), or None.
            dt: Time step in seconds.
            bbox_h_ratio: bbox_height / frame_height, fallback for distance.
            latency_ms: End-to-end latency in ms, for compensation.
            track_vx_norm: Track velocity in X (norm/s), for latency compensation.
            track_vy_norm: Track velocity in Y (norm/s), for latency compensation.

        Returns:
            VelocitySetpoint in body frame (FRD).
        """
        if dt <= 0:
            return VelocitySetpoint.zero()

        # ── Latency compensation (§8) ───────
        latency_s = latency_ms / 1000.0
        cx_comp = cx_norm + track_vx_norm * latency_s
        cy_comp = cy_norm + track_vy_norm * latency_s

        # ── Yaw (horizontal centering) ──────
        # Target right of center (positive cx) → positive yaw_rate (clockwise)
        yaw_rate = self._yaw_pid.step(cx_comp, dt)

        # ── Altitude (vertical centering) ───
        # Target below center (positive cy) → positive vz (down, FRD)
        vz = self._alt_pid.step(cy_comp, dt)

        # ── Forward (distance tracking) ─────
        vx = self._compute_forward(distance_m, bbox_h_ratio, dt)

        # ── Lateral (optional) ──────────────
        vy = 0.0  # lateral mode behind config flag (§8)

        return VelocitySetpoint(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def _compute_forward(
        self,
        distance_m: float | None,
        bbox_h_ratio: float | None,
        dt: float,
    ) -> float:
        """Compute forward velocity from distance error.

        Uses direct distance if available, falls back to bbox height ratio.
        """
        d_star = self.config.follow_distance_m

        if distance_m is not None and math.isfinite(distance_m) and distance_m > 0:
            # Primary: distance error
            error = distance_m - d_star
            return self._fwd_pid.step(error, dt)

        if bbox_h_ratio is not None and bbox_h_ratio > 0:
            # Fallback: use bbox height ratio as proxy
            # Larger bbox = closer → negative error → slow down/back up
            # Typical ratio at desired distance ~ 0.2 (configurable)
            desired_ratio = 0.2
            error = desired_ratio - bbox_h_ratio  # positive = too far
            # Scale up since ratio errors are small numbers
            return self._fwd_pid.step(error * d_star, dt)

        # No distance info → hover (S10)
        return 0.0

    def step(
        self,
        target_bbox: tuple[float, float, float, float],
        frame_size: tuple[int, int],
        distance_m: float | None,
        dt: float,
        latency_ms: float = 0.0,
    ) -> VelocitySetpoint:
        """Compute setpoints from bbox in pixels and frame_size (w, h)."""
        w, h = frame_size
        x1, y1, x2, y2 = target_bbox
        cx_px = (x1 + x2) / 2.0
        cy_px = (y1 + y2) / 2.0
        cx_norm = (cx_px / w) * 2.0 - 1.0 if w > 0 else 0.0
        cy_norm = (cy_px / h) * 2.0 - 1.0 if h > 0 else 0.0
        bbox_h_ratio = (y2 - y1) / h if h > 0 else None
        return self.compute(
            cx_norm=cx_norm,
            cy_norm=cy_norm,
            distance_m=distance_m,
            dt=dt,
            bbox_h_ratio=bbox_h_ratio,
            latency_ms=latency_ms,
        )

    def reset(self) -> None:
        """Reset all PIDs (call on state transition)."""
        self._yaw_pid.reset()
        self._alt_pid.reset()
        self._fwd_pid.reset()

    def update_config(self, config: VisualServoConfig) -> None:
        """Update configuration (e.g., from GCS TuneParams)."""
        self.config = config
        self._yaw_pid = PidController(config=config.yaw_pid)
        self._alt_pid = PidController(config=config.alt_pid)
        self._fwd_pid = PidController(config=config.fwd_pid)
