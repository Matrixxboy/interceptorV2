"""Shared constants — hard limits, ports, baud rates, and protocol version.

All hard limits live here (Safety Rule S8). GCS may only tune *within* these caps.
"""

from __future__ import annotations

# ── Protocol ────────────────────────────────
PROTOCOL_VERSION: int = 1

# ── Network defaults ────────────────────────
DEFAULT_GCS_HOST: str = "192.168.1.10"
DEFAULT_TELEMETRY_PORT: int = 5005
DEFAULT_COMMAND_PORT: int = 5006
DEFAULT_VIDEO_PORT: int = 5600
HEARTBEAT_HZ: float = 5.0
HEARTBEAT_INTERVAL_S: float = 1.0 / HEARTBEAT_HZ

# ── FC / MAVLink ────────────────────────────
DEFAULT_FC_PORT: str = "/dev/ttyAMA0"
DEFAULT_FC_BAUD: int = 921_600

# ── Hard safety limits (S8) — NEVER exceed ──
HARD_MAX_VX: float = 5.0  # m/s forward
HARD_MAX_VY: float = 3.0  # m/s lateral
HARD_MAX_VZ: float = 3.0  # m/s vertical
HARD_MAX_YAW_RATE: float = 2.0  # rad/s
HARD_MAX_SPEED: float = 5.0  # m/s magnitude

# ── Hard limit ranges for GCS tuning (S8) ───
HARD_MIN_FOLLOW_DISTANCE_M: float = 2.0
HARD_MAX_FOLLOW_DISTANCE_M: float = 30.0
HARD_MAX_PID_KP: float = 5.0
HARD_MAX_PID_KI: float = 2.0
HARD_MAX_PID_KD: float = 2.0

# ── Slew-rate limits (max change per second) ─
SLEW_RATE_VX: float = 4.0  # m/s²
SLEW_RATE_VY: float = 3.0  # m/s²
SLEW_RATE_VZ: float = 3.0  # m/s²
SLEW_RATE_YAW: float = 3.0  # rad/s²

# ── Safety timeouts ─────────────────────────
TARGET_LOST_TIMEOUT_S: float = 1.0  # → hover (S4)
TARGET_LOST_RTL_S: float = 5.0  # → LOITER (S4)
GCS_TIMEOUT_S: float = 2.0  # → hover (S5)
GCS_RTL_S: float = 10.0  # → RTL (S5)

# ── Control loop ────────────────────────────
CONTROL_HZ: int = 30
SETPOINT_MIN_HZ: int = 10  # ArduPilot needs ≥ 10 Hz (S9)
LOCK_RADIUS_NORM: float = 0.15  # tap must land within this of a track center
LOCK_TIMEOUT_S: float = 1.0  # selection timeout

# ── Vision defaults ─────────────────────────
DEFAULT_CONF_THRESHOLD: float = 0.35
DEFAULT_IOU_THRESHOLD: float = 0.5
DEFAULT_CLASSES: list[int] = [0]  # 0 = person in COCO

# ── Distance estimator ──────────────────────
DISTANCE_EMA_ALPHA: float = 0.3
DISTANCE_MAX_JUMP_PCT: float = 0.5  # reject jumps > 50% per frame

# ── Battery safety ──────────────────────────
BATTERY_SPEED_SCALE_THRESHOLD_V: float = 14.0  # start scaling speed below this
BATTERY_MIN_V: float = 13.2  # minimum operating voltage

# ── Telemetry rates ─────────────────────────
TELEMETRY_HZ: float = 20.0
TELEMETRY_INTERVAL_S: float = 1.0 / TELEMETRY_HZ

# ── MAVLink SET_POSITION_TARGET_LOCAL_NED ───
# type_mask: enable only vx, vy, vz, yaw_rate (bits 0-2 position, 6-8 accel = IGNORE)
# Bit meanings (1=ignore): pos_x(0), pos_y(1), pos_z(2), vx(3), vy(4), vz(5),
#   ax(6), ay(7), az(8), unused(9), yaw(10), yaw_rate(11)
# We want vx, vy, vz, yaw_rate → ignore pos(0,1,2), accel(6,7,8), yaw(10)
VELOCITY_YAW_RATE_TYPE_MASK: int = 0b0000_0100_0111  # = 0x0447 = 1095
# MAV_FRAME_BODY_OFFSET_NED = 9
MAV_FRAME_BODY_OFFSET_NED: int = 9
