# fpv-autonomous-tracker

Autonomous FPV drone target tracker — a drone follows a target the pilot selects by tapping live video on a ground tablet.

## Architecture

```
Camera ─► Detector ─► Tracker ─► [LatestSlot] ─► ControlLoop ─► Limits ─► MAVLink
                         │                             ▲
                         └──► overlay/telemetry ──► GCS │ commands
                    GCS ◄──── video (separate process)  └── Watchdog
```

- **Air-unit** (Raspberry Pi 5 + Hailo-8 NPU): detection, tracking, visual servoing, MAVLink
- **Ground-station** (PySide6): live video, tap-to-lock, telemetry, PID tuning

## Quick Start

```bash
# Install all dependencies
make install-dev

# Run linting + type checking + tests
make lint
make typecheck
make test

# Run air-unit (on RPi5)
make run-air

# Run ground-station (on tablet/laptop)
make run-gcs
```

## Safety

See [PLAN.MD §1](./PLAN.MD) for safety invariants. The companion computer **never** arms or disarms the vehicle. The pilot always retains RC override.

## Project Structure

See [PLAN.MD §4](./PLAN.MD) for the full repository layout.

## License

MIT
