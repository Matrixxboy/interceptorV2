"""Calibration Modal Dialog (§10, T4.4).

Allows pilot / engineer to inspect and tune camera intrinsics (focal length, principal point).
PySide6 imported lazily inside methods (§2.3).
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

logger = logging.getLogger(__name__)


class CalibrationModal:
    """Dialog for camera focal length and optical center calibration."""

    def __init__(
        self,
        focal_length_px: float = 1200.0,
        cx_px: float = 640.0,
        cy_px: float = 360.0,
        on_save: Callable[[float, float, float], None] | None = None,
    ) -> None:
        self.focal_length_px = focal_length_px
        self.cx_px = cx_px
        self.cy_px = cy_px
        self.on_save = on_save
        self._dialog: Any = None

    def show(self, parent: Any = None) -> None:
        """Create and display dialog."""
        from PySide6.QtWidgets import (  # lazy import (§2.3)
            QDialog,
            QDialogButtonBox,
            QDoubleSpinBox,
            QFormLayout,
            QVBoxLayout,
        )

        self._dialog = QDialog(parent)
        self._dialog.setWindowTitle("Camera Intrinsics Calibration")
        self._dialog.resize(350, 200)

        layout = QVBoxLayout(self._dialog)
        form = QFormLayout()

        focal_spin = QDoubleSpinBox()
        focal_spin.setRange(100.0, 10000.0)
        focal_spin.setValue(self.focal_length_px)
        form.addRow("Focal Length (px):", focal_spin)

        cx_spin = QDoubleSpinBox()
        cx_spin.setRange(0.0, 4000.0)
        cx_spin.setValue(self.cx_px)
        form.addRow("Optical Center X (px):", cx_spin)

        cy_spin = QDoubleSpinBox()
        cy_spin.setRange(0.0, 4000.0)
        cy_spin.setValue(self.cy_px)
        form.addRow("Optical Center Y (px):", cy_spin)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self._dialog)
        buttons.accepted.connect(self._dialog.accept)
        buttons.rejected.connect(self._dialog.reject)
        layout.addWidget(buttons)

        if self._dialog.exec():
            self.focal_length_px = float(focal_spin.value())
            self.cx_px = float(cx_spin.value())
            self.cy_px = float(cy_spin.value())
            if self.on_save:
                self.on_save(self.focal_length_px, self.cx_px, self.cy_px)
            logger.info(
                "Updated camera calibration: focal=%.1f, cx=%.1f, cy=%.1f",
                self.focal_length_px,
                self.cx_px,
                self.cy_px,
            )
