"""Telemetry Bar Widget (§10, T4.4).

Displays live telemetry: state machine state, FC mode, battery, distance, FPS, and latency.
PySide6 imported lazily inside methods (§2.3).
"""

from __future__ import annotations

import logging
from typing import Any

from shared.messages.telemetry import TelemetryMessage

logger = logging.getLogger(__name__)


class TelemetryBar:
    """Telemetry status bar widget."""

    def __init__(self) -> None:
        self._widget: Any = None
        self._state_label: Any = None
        self._mode_label: Any = None
        self._battery_label: Any = None
        self._distance_label: Any = None
        self._perf_label: Any = None

    def get_widget(self) -> Any:
        """Create and return PySide6 QWidget."""
        if self._widget is not None:
            return self._widget

        from PySide6.QtWidgets import QHBoxLayout, QLabel, QWidget  # lazy import (§2.3)

        self._widget = QWidget()
        layout = QHBoxLayout(self._widget)
        layout.setContentsMargins(10, 5, 10, 5)

        self._state_label = QLabel("State: INIT")
        self._mode_label = QLabel("FC: UNKNOWN")
        self._battery_label = QLabel("Bat: -- V")
        self._distance_label = QLabel("Dist: -- m")
        self._perf_label = QLabel("FPS: -- | Latency: -- ms")

        for lbl in (
            self._state_label,
            self._mode_label,
            self._battery_label,
            self._distance_label,
            self._perf_label,
        ):
            lbl.setStyleSheet("color: #E0E0E0; font-weight: bold; margin-right: 15px;")
            layout.addWidget(lbl)

        self._widget.setStyleSheet("background-color: #1E1E1E; border-top: 1px solid #333;")
        return self._widget

    def update_telemetry(self, msg: TelemetryMessage) -> None:
        """Update telemetry display labels."""
        if self._widget is None:
            return

        self._state_label.setText(f"State: {msg.state}")
        if msg.state == "FOLLOWING":
            self._state_label.setStyleSheet(
                "color: #00E676; font-weight: bold; margin-right: 15px;"
            )
        elif msg.state == "FAULT":
            self._state_label.setStyleSheet(
                "color: #FF1744; font-weight: bold; margin-right: 15px;"
            )
        else:
            self._state_label.setStyleSheet(
                "color: #E0E0E0; font-weight: bold; margin-right: 15px;"
            )

        armed_str = "ARMED" if msg.armed else "DISARMED"
        self._mode_label.setText(f"FC: {msg.fc_mode} ({armed_str})")

        bat_pct_str = f" ({msg.battery_pct:.0f}%)" if msg.battery_pct >= 0 else ""
        self._battery_label.setText(f"Bat: {msg.battery_v:.1f}V{bat_pct_str}")

        dist_str = f"{msg.distance_m:.1f} m" if msg.distance_m is not None else "-- m"
        lock_str = " [LOCKED]" if msg.target_lock else ""
        self._distance_label.setText(f"Dist: {dist_str}{lock_str}")

        self._perf_label.setText(f"FPS: {msg.fps:.0f} | Latency: {msg.latency_ms:.1f} ms")
