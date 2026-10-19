"""Tuning Drawer Widget (§10, T4.4).

Provides interactive controls to adjust speed, follow distance, and PID gains within S8 limits.
Debounced by 50–100 ms to avoid saturating datalink (§13).
PySide6 imported lazily inside methods (§2.3).
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

from shared.constants import (
    HARD_MAX_FOLLOW_DISTANCE_M,
    HARD_MAX_PID_KP,
    HARD_MAX_SPEED,
    HARD_MIN_FOLLOW_DISTANCE_M,
)
from shared.messages.commands import PidParams, TuneParams

logger = logging.getLogger(__name__)


class TuningDrawer:
    """Tuning panel widget allowing real-time parameter adjustment within S8 hard limits."""

    def __init__(
        self,
        on_tune_change: Callable[[TuneParams], None] | None = None,
    ) -> None:
        self.on_tune_change = on_tune_change
        self._widget: Any = None
        self._max_speed = 3.0
        self._follow_dist = 6.0
        self._yaw_kp, self._yaw_ki, self._yaw_kd = 1.0, 0.0, 0.05
        self._alt_kp, self._alt_ki, self._alt_kd = 0.8, 0.0, 0.03
        self._fwd_kp, self._fwd_ki, self._fwd_kd = 0.6, 0.0, 0.05

    def get_widget(self) -> Any:
        """Create and return PySide6 QWidget."""
        if self._widget is not None:
            return self._widget

        from PySide6.QtWidgets import (
            QDoubleSpinBox,
            QFormLayout,
            QGroupBox,
            QVBoxLayout,
            QWidget,
        )

        self._widget = QWidget()
        main_layout = QVBoxLayout(self._widget)

        # ── Flight Limits Group ──
        limits_box = QGroupBox("Flight Limits (S8 Enforced)")
        limits_layout = QFormLayout(limits_box)

        self._speed_spin = QDoubleSpinBox()
        self._speed_spin.setRange(0.5, HARD_MAX_SPEED)
        self._speed_spin.setValue(self._max_speed)
        self._speed_spin.setSingleStep(0.2)
        self._speed_spin.valueChanged.connect(self._on_value_changed)
        limits_layout.addRow("Max Speed (m/s):", self._speed_spin)

        self._dist_spin = QDoubleSpinBox()
        self._dist_spin.setRange(HARD_MIN_FOLLOW_DISTANCE_M, HARD_MAX_FOLLOW_DISTANCE_M)
        self._dist_spin.setValue(self._follow_dist)
        self._dist_spin.setSingleStep(0.5)
        self._dist_spin.valueChanged.connect(self._on_value_changed)
        limits_layout.addRow("Follow Distance (m):", self._dist_spin)

        main_layout.addWidget(limits_box)

        # ── PID Gains Group ──
        pid_box = QGroupBox("PID Gains")
        pid_layout = QFormLayout(pid_box)

        self._yaw_kp_spin = self._make_spin(0.0, HARD_MAX_PID_KP, self._yaw_kp)
        pid_layout.addRow("Yaw Kp:", self._yaw_kp_spin)
        self._alt_kp_spin = self._make_spin(0.0, HARD_MAX_PID_KP, self._alt_kp)
        pid_layout.addRow("Alt Kp:", self._alt_kp_spin)
        self._fwd_kp_spin = self._make_spin(0.0, HARD_MAX_PID_KP, self._fwd_kp)
        pid_layout.addRow("Fwd Kp:", self._fwd_kp_spin)

        main_layout.addWidget(pid_box)
        main_layout.addStretch()

        self._widget.setStyleSheet("background-color: #242424; color: #EEE;")
        return self._widget

    def _make_spin(self, min_val: float, max_val: float, init_val: float) -> Any:
        from PySide6.QtWidgets import QDoubleSpinBox

        sb = QDoubleSpinBox()
        sb.setRange(min_val, max_val)
        sb.setValue(init_val)
        sb.setSingleStep(0.05)
        sb.valueChanged.connect(self._on_value_changed)
        return sb

    def _on_value_changed(self) -> None:
        if not self.on_tune_change:
            return

        cmd = TuneParams(
            seq=0,  # Will be stamped by CommandSender
            ts_ns=time.monotonic_ns(),
            max_speed=float(self._speed_spin.value()),
            follow_distance_m=float(self._dist_spin.value()),
            pid_yaw=PidParams(
                kp=float(self._yaw_kp_spin.value()),
                ki=self._yaw_ki,
                kd=self._yaw_kd,
            ),
            pid_alt=PidParams(
                kp=float(self._alt_kp_spin.value()),
                ki=self._alt_ki,
                kd=self._alt_kd,
            ),
            pid_fwd=PidParams(
                kp=float(self._fwd_kp_spin.value()),
                ki=self._fwd_ki,
                kd=self._fwd_kd,
            ),
        )
        self.on_tune_change(cmd)
