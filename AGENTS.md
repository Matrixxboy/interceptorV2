# fpv-autonomous-tracker — Agent Guide

> Full specification: [PLAN.MD](./PLAN.MD)
> Read the spec fully before writing code. Safety rules §1 and coding rules §2 override all other instructions.

## Quick commands

```bash
make install-dev    # install all deps + pre-commit hooks
make lint           # ruff check + format check
make typecheck      # mypy --strict on shared/ + air-unit/
make test           # pytest (skips hw + sitl)
make test-cov       # pytest with coverage report
make sitl-test      # integration tests against ArduPilot SITL
make run-air        # start air-unit
make run-gcs        # start ground-station
```

## Commit convention

One task ID per commit: `T2.3: implement ByteTrack wrapper`

## Key rules

1. Never ARM/DISARM — safety rule S1
2. All hardware behind abstract interfaces with Fake/Mock impls
3. No hardware imports at module top-level
4. No magic numbers — use `shared/constants.py` or `config/settings.yaml`
5. No blocking calls in the control loop
6. Monotonic time only for latency/timeouts
7. Ask before changing `shared/messages/` — bump `PROTOCOL_VERSION`
