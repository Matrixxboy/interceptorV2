"""MAVLink Flight Controller Bridge (§7.2, T3.1).

Connects to ArduPilot via MAVLink over UART or UDP (for SITL).
Sends SET_POSITION_TARGET_LOCAL_NED velocity setpoints.
Enforces Safety Rule S1: NO ARM/DISARM command exists in this API.
Enforces Safety Rule S6: If mode is not GUIDED, stops output.
No top-level hardware imports (§2.3).
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Literal

from air_unit.hardware.interfaces import FcState, FlightController

logger = logging.getLogger(__name__)

# MAVLink type mask for SET_POSITION_TARGET_LOCAL_NED:
# Enable only vx, vy, vz, and yaw_rate.
# Ignore x, y, z (bits 0,1,2), ax, ay, az (bits 6,7,8), yaw (bit 9).
# 0b0000_0011_1100_0111 = 0x03C7 (967)
TYPEMASK_VELOCITY_AND_YAWRATE = 0x03C7


class MavlinkBridge(FlightController):
    """ArduPilot MAVLink bridge implementing FlightController ABC.

    Safety rule S1: The companion computer NEVER arms or disarms the vehicle.
    No ARM command exists anywhere in this class.
    """

    def __init__(
        self,
        connection_string: str = "udpin:127.0.0.1:14550",
        baud: int = 921600,
        source_system: int = 1,
        source_component: int = 191,  # MAV_COMP_ID_ONBOARD_COMPUTER
    ) -> None:
        self.connection_string = connection_string
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component

        self._conn: Any = None
        self._state_lock = threading.Lock()
        self._fc_state = FcState()
        self._running = False
        self._reader_thread: threading.Thread | None = None

    def connect(self) -> None:
        """Establish MAVLink connection and start telemetry background thread."""
        from pymavlink import mavutil  # lazy import (§2.3)

        logger.info("Connecting MAVLink to %s (baud=%d)...", self.connection_string, self.baud)
        self._conn = mavutil.mavlink_connection(
            self.connection_string,
            baud=self.baud,
            source_system=self.source_system,
            source_component=self.source_component,
        )

        self._running = True
        self._reader_thread = threading.Thread(
            target=self._telemetry_loop,
            name="MavlinkTelemetryReader",
            daemon=True,
        )
        self._reader_thread.start()

    def state(self) -> FcState:
        """Get latest flight controller state snapshot."""
        with self._state_lock:
            return self._fc_state

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """Send velocity setpoint via SET_POSITION_TARGET_LOCAL_NED.

        Body frame (MAV_FRAME_BODY_OFFSET_NED):
            vx: forward (+), backward (-)
            vy: right (+), left (-)
            vz: down (+), up (-)
            yaw_rate: rad/s (clockwise +)

        Enforces S6: If FC is not in GUIDED mode, blocks sending setpoints.
        """
        if self._conn is None:
            return

        with self._state_lock:
            current_mode = self._fc_state.mode

        # S6: Pilot switching FC out of GUIDED instantly stops all setpoint output
        if current_mode != "GUIDED":
            logger.debug(
                "S6: Suppressing velocity setpoint because FC mode is %s (not GUIDED)",
                current_mode,
            )
            return

        from pymavlink import mavutil  # lazy import (§2.3)

        # MAVLink SET_POSITION_TARGET_LOCAL_NED message
        # ArduPilot expects continuous stream while in GUIDED (S9)
        self._conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (0 = current)
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            TYPEMASK_VELOCITY_AND_YAWRATE,
            0.0,  # x (ignored)
            0.0,  # y (ignored)
            0.0,  # z (ignored)
            float(vx),
            float(vy),
            float(vz),
            0.0,  # afx (ignored)
            0.0,  # afy (ignored)
            0.0,  # afz (ignored)
            0.0,  # yaw (ignored)
            float(yaw_rate),
        )

    def request_mode(self, mode: Literal["LOITER", "RTL"]) -> None:
        """Request a mode change to LOITER or RTL (S1: NO ARM API)."""
        if self._conn is None:
            return

        logger.warning("Requesting FC mode change to %s", mode)
        # Using pymavlink helper which sends MAV_CMD_DO_SET_MODE
        mode_mapping = self._conn.mode_mapping()
        if mode_mapping and mode in mode_mapping:
            mode_id = mode_mapping[mode]
            self._conn.set_mode(mode_id)
        else:
            logger.error("Mode %s not found in vehicle mode mapping", mode)

    def close(self) -> None:
        """Stop background reader and close connection."""
        self._running = False
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self._conn:
            self._conn.close()
            self._conn = None
        logger.info("MAVLink bridge closed")

    def _telemetry_loop(self) -> None:
        """Background loop reading MAVLink messages and updating FcState."""
        while self._running and self._conn is not None:
            try:
                msg = self._conn.recv_match(
                    type=[
                        "HEARTBEAT",
                        "SYS_STATUS",
                        "GLOBAL_POSITION_INT",
                        "ATTITUDE",
                    ],
                    blocking=True,
                    timeout=0.1,
                )
                if msg is None:
                    continue

                msg_type = msg.get_type()
                with self._state_lock:
                    if msg_type == "HEARTBEAT":
                        from pymavlink import mavutil

                        mode_name = mavutil.mode_string_v10(msg)
                        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        self._fc_state = FcState(
                            mode=mode_name,
                            armed=armed,
                            battery_v=self._fc_state.battery_v,
                            battery_pct=self._fc_state.battery_pct,
                            alt_m=self._fc_state.alt_m,
                            roll_rad=self._fc_state.roll_rad,
                            pitch_rad=self._fc_state.pitch_rad,
                            yaw_rad=self._fc_state.yaw_rad,
                        )

                    elif msg_type == "SYS_STATUS":
                        voltage = (
                            msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else 0.0
                        )
                        pct = float(msg.battery_remaining) if msg.battery_remaining != -1 else -1.0
                        self._fc_state = FcState(
                            mode=self._fc_state.mode,
                            armed=self._fc_state.armed,
                            battery_v=voltage,
                            battery_pct=pct,
                            alt_m=self._fc_state.alt_m,
                            roll_rad=self._fc_state.roll_rad,
                            pitch_rad=self._fc_state.pitch_rad,
                            yaw_rad=self._fc_state.yaw_rad,
                        )

                    elif msg_type == "GLOBAL_POSITION_INT":
                        alt = msg.relative_alt / 1000.0  # mm to m
                        self._fc_state = FcState(
                            mode=self._fc_state.mode,
                            armed=self._fc_state.armed,
                            battery_v=self._fc_state.battery_v,
                            battery_pct=self._fc_state.battery_pct,
                            alt_m=alt,
                            roll_rad=self._fc_state.roll_rad,
                            pitch_rad=self._fc_state.pitch_rad,
                            yaw_rad=self._fc_state.yaw_rad,
                        )

                    elif msg_type == "ATTITUDE":
                        self._fc_state = FcState(
                            mode=self._fc_state.mode,
                            armed=self._fc_state.armed,
                            battery_v=self._fc_state.battery_v,
                            battery_pct=self._fc_state.battery_pct,
                            alt_m=self._fc_state.alt_m,
                            roll_rad=float(msg.roll),
                            pitch_rad=float(msg.pitch),
                            yaw_rad=float(msg.yaw),
                        )

            except Exception as e:
                logger.error("Error in MAVLink telemetry loop: %s", e)
                time.sleep(0.05)
