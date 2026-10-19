"""Ground Station Main Application (§0, §4, §10, T4.4).

Integrates:
- Video stream display with interactive target tap selection (VideoCanvas)
- Live telemetry bar (TelemetryBar)
- Parameter tuning drawer with S8 validation and debouncing (TuningDrawer)
- Camera calibration modal (CalibrationModal)
- CommandSender and TelemetryClient networking
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import Any

from ground_station.backend.command_sender import CommandSender
from ground_station.backend.telemetry_client import TelemetryClient
from ground_station.backend.video_receiver import VideoReceiver
from ground_station.ui.calibration_modal import CalibrationModal
from ground_station.ui.telemetry_bar import TelemetryBar
from ground_station.ui.tuning_drawer import TuningDrawer
from ground_station.ui.video_canvas import VideoCanvas

from shared.constants import (
    DEFAULT_COMMAND_PORT,
    DEFAULT_TELEMETRY_PORT,
    DEFAULT_VIDEO_PORT,
)

logger = logging.getLogger("ground_station")


class GroundStationApp:
    """Main Ground Station Controller."""

    def __init__(
        self,
        drone_host: str = "127.0.0.1",
        cmd_port: int = DEFAULT_COMMAND_PORT,
        telem_port: int = DEFAULT_TELEMETRY_PORT,
        video_port: int = DEFAULT_VIDEO_PORT,
    ) -> None:
        self.cmd_sender = CommandSender(drone_host=drone_host, drone_port=cmd_port)
        self.telem_client = TelemetryClient(port=telem_port)
        self.video_receiver = VideoReceiver(port=video_port)

        self.canvas = VideoCanvas(on_tap=self._on_canvas_tap)
        self.telem_bar = TelemetryBar()
        self.tuning_drawer = TuningDrawer(on_tune_change=self._on_tune_change)
        self.calib_modal = CalibrationModal()

        self.telem_client.add_listener(self.telem_bar.update_telemetry)
        self.telem_client.add_listener(self.canvas.update_telemetry)
        self.video_receiver.add_frame_callback(self._on_video_frame)

    def _on_canvas_tap(self, x_norm: float, y_norm: float) -> None:
        logger.info("Pilot tapped target at (%.3f, %.3f)", x_norm, y_norm)
        import threading

        threading.Thread(
            target=self.cmd_sender.send_lock,
            args=(x_norm, y_norm),
            daemon=True,
        ).start()

    def _on_tune_change(self, tune_cmd: Any) -> None:
        logger.info("Tuning updated, queueing debounce: %s", tune_cmd)
        self.cmd_sender.send_tune(tune_cmd, debounce=True)

    def _on_video_frame(self, frame: Any, latency_ms: float) -> None:
        self.canvas.update_frame(frame)

    def start_networking(self) -> None:
        """Start UDP listeners and heartbeat."""
        self.telem_client.start()
        self.video_receiver.start()
        self.cmd_sender.start_heartbeat()

    def stop_networking(self) -> None:
        """Stop networking."""
        self.cmd_sender.stop()
        self.telem_client.stop()
        self.video_receiver.stop()

    def build_main_window(self) -> Any:
        """Assemble PySide6 main window."""
        from PySide6.QtWidgets import (
            QHBoxLayout,
            QMainWindow,
            QPushButton,
            QVBoxLayout,
            QWidget,
        )

        window = QMainWindow()
        window.setWindowTitle("FPV Autonomous Tracker — Ground Station")
        window.resize(1280, 800)

        central = QWidget()
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Top control toolbar
        top_bar = QWidget()
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(10, 5, 10, 5)

        unlock_btn = QPushButton("UNLOCK / HOVER")
        unlock_btn.setStyleSheet(
            "background-color: #D32F2F; color: white; font-weight: bold; padding: 6px 12px;"
        )
        unlock_btn.clicked.connect(lambda: self.cmd_sender.send_unlock())
        top_layout.addWidget(unlock_btn)

        calib_btn = QPushButton("Camera Intrinsics")
        calib_btn.clicked.connect(lambda: self.calib_modal.show(window))
        top_layout.addWidget(calib_btn)
        top_layout.addStretch()

        main_layout.addWidget(top_bar)

        # Middle area: video canvas + tuning drawer
        middle = QWidget()
        mid_layout = QHBoxLayout(middle)
        mid_layout.setContentsMargins(0, 0, 0, 0)

        mid_layout.addWidget(self.canvas.get_widget(), stretch=4)
        mid_layout.addWidget(self.tuning_drawer.get_widget(), stretch=1)

        main_layout.addWidget(middle, stretch=1)

        # Bottom: telemetry status bar
        main_layout.addWidget(self.telem_bar.get_widget())

        window.setCentralWidget(central)
        return window


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(description="FPV Tracker Ground Station")
    parser.add_argument("--drone-host", default="127.0.0.1")
    args = parser.parse_args()

    app_instance = GroundStationApp(drone_host=args.drone_host)
    app_instance.start_networking()

    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    window = app_instance.build_main_window()
    window.show()

    ret = app.exec()
    app_instance.stop_networking()
    sys.exit(ret)


if __name__ == "__main__":
    main()
