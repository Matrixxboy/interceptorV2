"""Air-unit main entrypoint (§4, T3.2).

Wires all components according to settings.yaml and runs Supervisor.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

from air_unit.comms.gcs_link import GcsLink
from air_unit.comms.video_streamer import VideoStreamer
from air_unit.config.settings_schema import Settings, load_settings
from air_unit.control.dynamic_limits import DynamicLimits, DynamicLimitsConfig
from air_unit.control.pid import PidConfig
from air_unit.control.safety_watchdog import SafetyWatchdog, WatchdogConfig
from air_unit.control.visual_servoing import VisualServo, VisualServoConfig
from air_unit.core.logging_setup import setup_logging
from air_unit.core.supervisor import Supervisor
from air_unit.hardware.interfaces import CameraSource, FlightController
from air_unit.hardware.mavlink_bridge import MavlinkBridge
from air_unit.sim.fake_camera import FakeCamera, FileCamera
from air_unit.sim.fake_fc import FakeFC
from air_unit.vision.detector import Detector, FakeDetector, HailoDetector, OnnxDetector
from air_unit.vision.distance_estimator import DistanceEstimator, DistanceEstimatorConfig
from air_unit.vision.target_selector import TargetSelector
from air_unit.vision.tracker import ByteTracker, TrackerConfig

logger = logging.getLogger("air_unit")


def create_camera(settings: Settings, sim: bool = False) -> CameraSource:
    """Instantiate camera based on config."""
    if sim:
        return FakeCamera(
            width=settings.camera.width,
            height=settings.camera.height,
            fps=settings.camera.fps,
        )
    if settings.camera.source == "file":
        return FileCamera(file_path="test_clip.mp4")
    # For hardware raspberry pi: Picamera2 or V4L2 fallback
    return FakeCamera(
        width=settings.camera.width,
        height=settings.camera.height,
        fps=settings.camera.fps,
    )


def create_detector(settings: Settings, sim: bool = False) -> Detector:
    """Instantiate detector based on config."""
    if sim:
        return FakeDetector()
    if settings.vision.model.endswith(".hef"):
        return HailoDetector(
            hef_path=settings.vision.model,
            conf_thresh=settings.vision.conf,
            iou_thresh=settings.vision.iou,
            classes=settings.vision.classes,
        )
    return OnnxDetector(
        model_path=settings.vision.model,
        conf_thresh=settings.vision.conf,
        iou_thresh=settings.vision.iou,
        classes=settings.vision.classes,
    )


def create_fc(settings: Settings, sim: bool = False) -> FlightController:
    """Instantiate flight controller."""
    if sim:
        return FakeFC()
    bridge = MavlinkBridge(
        connection_string=f"udpin:0.0.0.0:{settings.fc.baud}"
        if ":" in str(settings.fc.port)
        else settings.fc.port,
        baud=settings.fc.baud,
    )
    bridge.connect()
    return bridge


def build_supervisor(
    settings: Settings,
    sim: bool = False,
    gcs_host: str | None = None,
) -> tuple[Supervisor, GcsLink]:
    """Build and wire all air-unit components."""
    camera = create_camera(settings, sim=sim)
    detector = create_detector(settings, sim=sim)
    tracker = ByteTracker(TrackerConfig())
    target_selector = TargetSelector()
    distance_estimator = DistanceEstimator(DistanceEstimatorConfig())
    fc = create_fc(settings, sim=sim)

    visual_servo = VisualServo(
        VisualServoConfig(
            yaw_pid=PidConfig(
                kp=settings.pid.yaw.kp,
                ki=settings.pid.yaw.ki,
                kd=settings.pid.yaw.kd,
            ),
            alt_pid=PidConfig(
                kp=settings.pid.alt.kp,
                ki=settings.pid.alt.ki,
                kd=settings.pid.alt.kd,
            ),
            fwd_pid=PidConfig(
                kp=settings.pid.fwd.kp,
                ki=settings.pid.fwd.ki,
                kd=settings.pid.fwd.kd,
            ),
            follow_distance_m=settings.control.follow_distance_m,
        )
    )

    dynamic_limits = DynamicLimits(
        DynamicLimitsConfig(
            max_vx=settings.control.max_vx,
            max_vz=settings.control.max_vz,
            max_yaw_rate=settings.control.max_yaw_rate,
        )
    )

    safety_watchdog = SafetyWatchdog(
        WatchdogConfig(
            target_lost_timeout_s=settings.safety.target_lost_timeout_s,
            target_lost_rtl_s=settings.safety.target_lost_rtl_s,
            gcs_timeout_s=settings.safety.gcs_timeout_s,
            gcs_rtl_s=settings.safety.gcs_rtl_s,
        )
    )

    target_host = gcs_host or ("127.0.0.1" if sim else settings.network.gcs_host)
    video_streamer = VideoStreamer(
        host=target_host,
        port=settings.network.video_port,
    )

    supervisor = Supervisor(
        camera=camera,
        detector=detector,
        tracker=tracker,
        target_selector=target_selector,
        distance_estimator=distance_estimator,
        flight_controller=fc,
        visual_servo=visual_servo,
        dynamic_limits=dynamic_limits,
        safety_watchdog=safety_watchdog,
        control_hz=float(settings.control.hz),
        video_streamer=video_streamer,
    )

    gcs_link = GcsLink(
        supervisor=supervisor,
        safety_watchdog=safety_watchdog,
        gcs_host=target_host,
        telemetry_port=settings.network.telemetry_port,
        command_port=settings.network.command_port,
    )

    return supervisor, gcs_link


def main() -> None:
    """Main CLI entrypoint."""
    parser = argparse.ArgumentParser(description="FPV Autonomous Tracker — Air Unit")
    parser.add_argument("--config", type=str, default=None, help="Path to settings.yaml")
    parser.add_argument("--sim", action="store_true", help="Run with simulated components")
    parser.add_argument("--gcs-host", type=str, default=None, help="GCS IP address override")
    args = parser.parse_args()

    setup_logging()
    settings = load_settings(path=args.config)
    supervisor, gcs_link = build_supervisor(settings, sim=args.sim, gcs_host=args.gcs_host)

    def handle_signal(sig: int, frame: object) -> None:
        logger.info("Signal received, stopping air unit...")
        gcs_link.stop()
        supervisor.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    supervisor.start()
    gcs_link.start()
    logger.info("Air-unit fully operational.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        handle_signal(0, None)


if __name__ == "__main__":
    main()
