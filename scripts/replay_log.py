"""scripts/replay_log.py — Replay recorded flight logs through the control stack (§11).

Reads recorded flight log records (TrackState, FC state) and computes velocity
setpoints through VisualServoingController and DynamicLimiter, asserting that all
setpoints stay strictly within safety limits (S3, S8, S10).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import msgpack
from air_unit.control.dynamic_limits import DynamicLimiter
from air_unit.control.pid import PidConfig
from air_unit.control.visual_servoing import VisualServo, VisualServoConfig
from air_unit.vision.interfaces import Track, TrackState

from shared.constants import (
    HARD_MAX_VX,
    HARD_MAX_VY,
    HARD_MAX_VZ,
    HARD_MAX_YAW_RATE,
)


def replay_flight_log(log_path: Path) -> int:
    """Replays flight log records and asserts all safety constraints hold.

    Returns the number of processed frames.
    """
    if not log_path.exists():
        raise FileNotFoundError(f"Flight log not found: {log_path}")

    # Set up pure control stack
    vs_config = VisualServoConfig(
        follow_distance_m=6.0,
        yaw_pid=PidConfig(
            kp=1.0,
            ki=0.0,
            kd=0.05,
            output_min=-HARD_MAX_YAW_RATE,
            output_max=HARD_MAX_YAW_RATE,
        ),
        alt_pid=PidConfig(
            kp=0.8,
            ki=0.0,
            kd=0.03,
            output_min=-HARD_MAX_VZ,
            output_max=HARD_MAX_VZ,
        ),
        fwd_pid=PidConfig(
            kp=0.6,
            ki=0.0,
            kd=0.05,
            output_min=-HARD_MAX_VX,
            output_max=HARD_MAX_VX,
        ),
    )
    controller = VisualServo(config=vs_config)
    limiter = DynamicLimiter()

    with open(log_path, "rb") as f:
        unpacker = msgpack.Unpacker(f, raw=False)
        frame_idx = 0
        last_ts_ns: int | None = None

        for record in unpacker:
            if not isinstance(record, dict):
                continue

            # Record format: {"ts_ns": ..., "target": {...}, "distance_m": ..., ...}
            ts_ns = record.get("ts_ns", 0)
            dt = (
                (ts_ns - last_ts_ns) * 1e-9
                if (last_ts_ns is not None and ts_ns > last_ts_ns)
                else 0.033
            )
            last_ts_ns = ts_ns

            target_dict: dict[str, Any] | None = record.get("target")
            target: Track | None = None
            if target_dict:
                target = Track(
                    id=target_dict["id"],
                    bbox=tuple(target_dict["bbox"]),  # type: ignore[arg-type]
                    score=target_dict.get("score", 1.0),
                    age=target_dict.get("age", 1),
                )

            frame_w = record.get("frame_w", 1280)
            frame_h = record.get("frame_h", 720)
            track_state = TrackState(
                capture_ts_ns=ts_ns,
                frame_w=frame_w,
                frame_h=frame_h,
                target=target,
                distance_m=record.get("distance_m"),
                latency_ms=record.get("latency_ms", 30.0),
            )

            if track_state.target is not None and frame_w > 0 and frame_h > 0:
                x1, y1, x2, y2 = track_state.target.bbox
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                cx_norm = (cx / frame_w) * 2.0 - 1.0
                cy_norm = (cy / frame_h) * 2.0 - 1.0
                raw_sp = controller.compute(
                    cx_norm=cx_norm,
                    cy_norm=cy_norm,
                    distance_m=track_state.distance_m,
                    dt=dt,
                    latency_ms=track_state.latency_ms,
                )
            else:
                raw_sp = controller.compute(
                    cx_norm=0.0,
                    cy_norm=0.0,
                    distance_m=None,
                    dt=dt,
                )
            clamped_sp = limiter.clamp(raw_sp, dt=dt)

            # Assert S3 & S8 safety invariants
            assert abs(clamped_sp.vx) <= HARD_MAX_VX + 1e-5, (
                f"vx {clamped_sp.vx} exceeded hard limit!"
            )
            assert abs(clamped_sp.vy) <= HARD_MAX_VY + 1e-5, (
                f"vy {clamped_sp.vy} exceeded hard limit!"
            )
            assert abs(clamped_sp.vz) <= HARD_MAX_VZ + 1e-5, (
                f"vz {clamped_sp.vz} exceeded hard limit!"
            )
            assert abs(clamped_sp.yaw_rate) <= HARD_MAX_YAW_RATE + 1e-5, (
                f"yaw_rate {clamped_sp.yaw_rate} exceeded hard limit!"
            )

            frame_idx += 1

    print(f"Replay verified successfully: {frame_idx} frames processed without limit violations.")
    return frame_idx


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay recorded flight log and verify safety limits."
    )
    parser.add_argument("log_file", type=Path, help="Path to .msgpack flight log file")
    args = parser.parse_args()

    try:
        replay_flight_log(args.log_file)
    except Exception as exc:
        print(f"Replay verification failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
