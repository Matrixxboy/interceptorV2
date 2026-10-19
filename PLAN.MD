# fpv-autonomous-tracker — Agent Implementation Spec

> Read this file fully before writing code. Follow the rules in §1 and §2 over any other instruction.
> Work phase by phase (§10). Do not start a phase until the previous phase's acceptance checks pass.

---

## 0. Goal (one paragraph)

A drone follows a target the pilot selects by tapping the live video on a ground tablet.
The **air-unit** (Raspberry Pi 5 + Hailo-8 NPU) detects and tracks objects, computes velocity setpoints with visual servoing, and sends them to an ArduPilot flight controller over MAVLink. The **ground-station** (PySide6) shows low-latency video, sends lock/tuning commands, and displays telemetry. The pilot always retains RC override.

**Non-goals (v1):** obstacle avoidance, multi-drone, autonomous takeoff/arming, cloud connectivity, PX4 support.

---

## 1. Safety invariants (NEVER violate)

| # | Rule | Enforced in |
|---|------|-------------|
| S1 | The companion computer **never arms or disarms** the vehicle. | `mavlink_bridge.py` (no ARM command exists in API) |
| S2 | Velocity setpoints are sent **only** when FC mode is `GUIDED` and state is `FOLLOWING`. | `safety_watchdog.py` |
| S3 | Every setpoint passes through `dynamic_limits.clamp()` before MAVLink. No bypass. | `control/` |
| S4 | No fresh target for `TARGET_LOST_TIMEOUT_S` (default 1.0) → zero velocity (hover). After `TARGET_LOST_RTL_S` (default 5.0) → request `LOITER`, then hand back to pilot. | `safety_watchdog.py` |
| S5 | No GCS heartbeat for `GCS_TIMEOUT_S` (default 2.0) → hover; after `GCS_RTL_S` (default 10.0) → `RTL`. | `safety_watchdog.py` |
| S6 | Pilot switching FC out of `GUIDED` instantly stops all setpoint output (no fighting the pilot). | `mavlink_bridge.py` |
| S7 | Any unhandled exception in vision or control thread → state `FAULT`, zero setpoints, log, stay alive. | `core/supervisor.py` |
| S8 | All hard limits live in `shared/constants.py`; GCS may only tune *within* them. | `dynamic_limits.py` |
| S9 | ArduPilot stops honoring velocity targets after ~3 s without a new one → always resend at ≥ 10 Hz while following. | `control_loop` |
| S10 | Default for every unknown/NaN/stale input is **hover**, never "continue last command". | all |

Every safety rule must have a unit test named `test_S<n>_*`.

---

## 2. Rules for coding agents

1. **Python 3.11+**, full type hints, `mypy --strict` clean on `air-unit/`, `shared/`.
2. **Hardware behind interfaces.** Every hardware/IO dependency (camera, NPU, MAVLink, sockets) is an abstract class in `*/interfaces.py` with a `Fake*`/`Mock*` implementation. Core logic must run with zero hardware.
3. **No hardware imports at module top-level** (`hailo_platform`, `picamera2`, `PySide6` in air-unit). Import lazily inside the concrete class so tests run on a laptop.
4. **No magic numbers.** Values come from `config/settings.yaml` (validated by Pydantic) or `shared/constants.py`.
5. **No blocking calls in the control loop.** No file I/O, logging at DEBUG-only, no network calls.
6. **Monotonic time only** (`time.monotonic_ns()`) for latency/timeouts. Wall-clock only for log filenames.
7. **Pure functions for math** (PID step, pinhole distance, coordinate transforms) — easy to unit test.
8. **One responsibility per file.** If a file exceeds ~300 lines, split it.
9. **Each task ends with:** `make lint test` green, docstring on public classes, and the task's acceptance test (§10) passing.
10. **Don't invent MAVLink fields.** Use pymavlink's `ardupilotmega` dialect; cite the message name in a comment.
11. **Ask before changing** anything in `shared/messages/` — it is the wire contract; bump `PROTOCOL_VERSION` if changed.
12. Prefer small PRs: one task ID per commit, message format `T2.3: implement ByteTrack wrapper`.

---

## 3. Tech stack (decisions already made)

| Concern | Choice | Notes |
|---|---|---|
| Air-unit OS | Raspberry Pi OS 64-bit (Bookworm) | |
| NPU | Hailo-8 / 8L via HailoRT Python API | quantized YOLOv8n/s `.hef` |
| Camera | Picamera2 (CSI) behind `CameraSource` | fallback: V4L2/OpenCV, video file |
| Tracking | ByteTrack (own thin wrapper) + optional Kalman | |
| FC link | `pymavlink`, ArduPilot (ArduCopter), UART 921600 | |
| Setpoint msg | `SET_POSITION_TARGET_LOCAL_NED`, frame `MAV_FRAME_BODY_OFFSET_NED`, velocity+yaw_rate type_mask | |
| Video | H.264 HW encode → RTP/UDP (GStreamer) | RTSP optional later |
| Telemetry/commands | UDP, `msgpack` payload, Pydantic models | heartbeat both ways at 5 Hz |
| GCS UI | PySide6 (Qt 6) | |
| Config | YAML + Pydantic Settings | |
| Tooling | `uv` or `pip-tools`, `ruff`, `mypy`, `pytest`, `pre-commit`, GitHub Actions | |
| Simulation | ArduPilot SITL + MAVProxy; Gazebo optional | |

---

## 4. Repository layout (final)

```text
fpv-autonomous-tracker/
├── AGENTS.md                     # 20-line pointer to this file + commands (for IDE auto-load)
├── PROJECT_SPEC.md               # this file
├── Makefile                      # install, lint, typecheck, test, sitl, run-air, run-gcs
├── pyproject.toml                # ruff, mypy, pytest config; workspace deps
├── docs/
│   ├── adr/                      # 0001-threading-model.md, 0002-wire-format.md ...
│   ├── state-machine.md
│   └── hardware-wiring.md        # UART pins, power, CSI, Hailo HAT
├── shared/
│   ├── constants.py              # ports, baud, hard limits, PROTOCOL_VERSION
│   ├── time_utils.py             # monotonic helpers
│   └── messages/
│       ├── telemetry.py          # drone → ground
│       ├── commands.py           # ground → drone
│       └── codec.py              # encode/decode (msgpack) + version check
├── air-unit/
│   ├── main.py
│   ├── config/{settings.yaml, settings_schema.py, hailo_config.json}
│   ├── core/
│   │   ├── supervisor.py         # starts/stops workers, S7 fault handling
│   │   ├── state_machine.py      # §6
│   │   ├── latest_slot.py        # thread-safe "latest value only" mailbox
│   │   └── logging_setup.py      # structured JSON logs + ring buffer
│   ├── hardware/
│   │   ├── interfaces.py         # CameraSource, FlightController ABCs
│   │   ├── camera.py             # Picamera2 / V4L2 / FileCamera
│   │   └── mavlink_bridge.py     # real FC + shared parsing
│   ├── vision/
│   │   ├── interfaces.py         # Detector ABC, Detection dataclass
│   │   ├── detector.py           # HailoDetector, OnnxDetector (CPU dev fallback)
│   │   ├── tracker.py            # ByteTrack wrapper, track IDs, lock logic
│   │   ├── target_selector.py    # tap (x_norm,y_norm) → track ID; re-acquire
│   │   └── distance_estimator.py # pinhole model + EMA
│   ├── control/
│   │   ├── pid.py                # pure PID (anti-windup, d-filter, deadband)
│   │   ├── visual_servoing.py    # image error → vx, vy, vz, yaw_rate
│   │   ├── dynamic_limits.py     # clamp + slew-rate limit
│   │   └── safety_watchdog.py    # S2, S4, S5, S6
│   ├── comms/
│   │   ├── video_streamer.py
│   │   └── gcs_link.py           # UDP rx/tx, heartbeat, rate limiting
│   ├── sim/
│   │   ├── fake_camera.py        # replays video / synthetic moving box
│   │   ├── fake_fc.py            # in-process FlightController mock
│   │   └── sitl_runner.sh
│   └── tests/ (unit/, integration/, hardware/)   # hardware/ marked @pytest.mark.hw
├── ground-station/
│   ├── main.py
│   ├── backend/{video_receiver.py, telemetry_client.py, command_sender.py}
│   ├── ui/{main_window, video_canvas, tuning_drawer, telemetry_bar, calibration_modal}.py
│   ├── assets/
│   └── tests/
├── scripts/
│   ├── setup_rpi5_hailo.sh
│   ├── setup_openipc_link.sh
│   ├── calibrate_camera.py       # writes air-unit/config/camera_intrinsics.yaml
│   ├── replay_log.py             # replay recorded flight logs through control stack
│   └── services/{drone-tracker.service, video-stream.service}
├── .github/workflows/ci.yml      # lint + mypy + pytest (no hw)
├── .env.example
└── README.md
```

---

## 5. Process & threading model

Python's GIL makes CPU-heavy threads unreliable. Rules:

- **Vision thread** (capture → NPU → track): NPU inference releases the GIL; OK as a thread.
- **Control thread**: fixed-rate loop (`CONTROL_HZ`, default 30). Reads `LatestSlot[TrackState]`, never waits on vision.
- **Comms thread/asyncio loop**: GCS UDP, telemetry out, video is a **separate GStreamer process** (not in Python's hot path).
- **MAVLink reader**: own thread, updates `FcState` (mode, armed, battery, attitude, altitude).
- Inter-thread data flows through `LatestSlot` (overwrite semantics, no queues that grow). Frames carry `capture_ts_ns`.

```text
Camera ─► Detector ─► Tracker ─► [LatestSlot TrackState] ─► ControlLoop ─► Limits ─► MAVLink
                         │                                       ▲
                         └──► overlay/telemetry ──► GCS          │ commands (lock, tune, mode)
                    GCS ◄──── video (separate process)           └── Watchdog (S2,S4,S5,S6)
```

---

## 6. State machine (`core/state_machine.py`)

States: `INIT → IDLE → SELECTING → FOLLOWING → TARGET_LOST → IDLE`, plus `FAULT` (from anywhere).

| From | Event | To | Action |
|---|---|---|---|
| INIT | camera+FC+NPU healthy | IDLE | start telemetry |
| IDLE | `LockRequest` and FC mode == GUIDED | SELECTING | map tap → track |
| SELECTING | track found within `LOCK_RADIUS_NORM` | FOLLOWING | init PIDs, reset filters |
| SELECTING | no match in 1 s | IDLE | notify GCS `lock_failed` |
| FOLLOWING | track missing > `TARGET_LOST_TIMEOUT_S` | TARGET_LOST | zero setpoints |
| TARGET_LOST | re-acquired (same ID or appearance/position gate) | FOLLOWING | |
| TARGET_LOST | timeout `TARGET_LOST_RTL_S` | IDLE | request LOITER |
| FOLLOWING | `UnlockRequest` / FC mode ≠ GUIDED / GCS timeout | IDLE (or hover per S5) | stop output |
| any | exception / hw failure | FAULT | zero setpoints, alert GCS |

Transition table must be data-driven and unit tested exhaustively (every state × event).

---

## 7. Contracts

### 7.1 Core dataclasses (`vision/interfaces.py`, `core/`)

```python
@dataclass(frozen=True)
class Detection:
    x1: float; y1: float; x2: float; y2: float   # pixels
    score: float
    cls: int

@dataclass(frozen=True)
class Track:
    id: int
    bbox: tuple[float, float, float, float]      # pixels
    score: float
    age: int

@dataclass(frozen=True)
class TrackState:
    capture_ts_ns: int
    frame_w: int; frame_h: int
    target: Track | None
    distance_m: float | None
    latency_ms: float

@dataclass(frozen=True)
class VelocitySetpoint:                          # body frame, FRD
    vx: float; vy: float; vz: float              # m/s (vz + = down)
    yaw_rate: float                              # rad/s
```

### 7.2 Abstract interfaces (`hardware/interfaces.py`)

```python
class CameraSource(ABC):
    def start(self) -> None: ...
    def read(self) -> tuple[np.ndarray, int]: ...   # frame, capture_ts_ns
    def stop(self) -> None: ...

class FlightController(ABC):
    def state(self) -> FcState: ...                  # mode, armed, battery, alt, att
    def send_velocity(self, sp: VelocitySetpoint) -> None: ...
    def request_mode(self, mode: Literal["LOITER", "RTL"]) -> None: ...   # no ARM API (S1)
    def close(self) -> None: ...

class Detector(ABC):
    def infer(self, frame: np.ndarray) -> list[Detection]: ...
```

### 7.3 Wire messages (`shared/messages/`, Pydantic, msgpack on the wire)

Every message has: `v: int` (PROTOCOL_VERSION), `seq: int`, `ts_ns: int`.

- **Telemetry (drone→GCS, 10–20 Hz):** `state`, `fc_mode`, `armed`, `battery_v`, `battery_pct`, `target_lock: bool`, `bbox_norm: [x1,y1,x2,y2] | None`, `distance_m`, `fps`, `latency_ms`, `fault: str | None`.
- **Commands (GCS→drone):**
  - `LockRequest{x_norm, y_norm}`
  - `UnlockRequest{}`
  - `TuneParams{max_speed, follow_distance_m, pid: {yaw:{kp,ki,kd}, alt:{...}, fwd:{...}}}` — validated against S8 limits, **rejected with `Ack{ok=False, reason}`** if out of range
  - `Heartbeat{}` (5 Hz, feeds S5)
- Drone replies `Ack{seq, ok, reason}` to every command. GCS retries commands up to 3× (idempotent by `seq`).

---

## 8. Algorithms (spec, not suggestions)

**Visual servoing (`visual_servoing.py`)** — inputs: target bbox center `(cx, cy)` normalized to `[-1, 1]`, estimated distance `d`, desired distance `d*`.
- `yaw_rate = PID_yaw(cx)` (deadband ±0.03)
- `vz = PID_alt(cy)` (deadband ±0.03, sign-checked in test)
- `vx = PID_fwd(d - d*)` (use bbox height ratio as fallback if distance invalid)
- `vy = 0` default (optional lateral mode behind config flag)
- Each PID: output clamp, integral clamp (anti-windup), low-pass on derivative, reset on state change.
- Compensate latency: extrapolate bbox by track velocity × `latency_ms`.

**Distance (`distance_estimator.py`)** — `d = f_px * H_real / h_px`, with `H_real` from per-class prior table in config, `f_px` from `camera_intrinsics.yaml`. Apply EMA (`alpha` configurable), reject jumps > X% per frame, return `None` if bbox touches frame edge.

**Limits (`dynamic_limits.py`)** — clamp each axis to `min(config_max, GCS_max, HARD_MAX)`, plus slew-rate limit (max Δ per second) to avoid jerk. Scale speed down as battery falls below threshold.

**Tracker/re-acquire** — on lock, store `{id, last bbox, last velocity}`. On loss, candidate must be within a gated search window and class-compatible; widen the window with time lost.

---

## 9. Configuration (`air-unit/config/settings.yaml`, validated by Pydantic)

```yaml
camera: {width: 1280, height: 720, fps: 30, source: picamera2}   # picamera2 | v4l2 | file
fc: {port: /dev/ttyAMA0, baud: 921600, expected_mode: GUIDED}
vision: {model: models/yolov8n.hef, conf: 0.35, iou: 0.5, classes: [0]}   # 0=person
control: {hz: 30, max_vx: 3.0, max_vz: 1.5, max_yaw_rate: 1.2, follow_distance_m: 6.0}
pid:
  yaw: {kp: 1.0, ki: 0.0, kd: 0.05}
  alt: {kp: 0.8, ki: 0.0, kd: 0.03}
  fwd: {kp: 0.6, ki: 0.0, kd: 0.05}
safety: {target_lost_timeout_s: 1.0, target_lost_rtl_s: 5.0, gcs_timeout_s: 2.0, gcs_rtl_s: 10.0}
network: {gcs_host: 192.168.1.10, telemetry_port: 5005, video_port: 5600}
```

(Defaults are conservative starting points — tune in SITL then in the field. Hard caps live in `shared/constants.py`.)

---

## 10. Phased backlog (agent-executable)

Each task = one commit. "Accept" is the test that must pass.

### Phase 0 — Scaffolding
- **T0.1** Create tree from §4, `pyproject.toml`, `Makefile`, `pre-commit`, CI. *Accept:* `make lint test` runs green on empty tests.
- **T0.2** `shared/constants.py`, `shared/messages/*`, `codec.py`. *Accept:* round-trip encode/decode test; version mismatch rejected.
- **T0.3** `settings_schema.py` + loader. *Accept:* invalid YAML raises clear error; defaults load.

### Phase 1 — Core logic with zero hardware
- **T1.1** `latest_slot.py`, `state_machine.py` (+ exhaustive table test).
- **T1.2** `pid.py` (anti-windup, d-filter) — *Accept:* step-response test converges, no windup, output clamped.
- **T1.3** `distance_estimator.py` — *Accept:* known bbox → known distance within 2%.
- **T1.4** `visual_servoing.py` — *Accept:* target right of center → positive yaw_rate; below center → positive vz (down); too far → positive vx.
- **T1.5** `dynamic_limits.py` — *Accept:* property test: output never exceeds any cap; slew respected.
- **T1.6** `safety_watchdog.py` — *Accept:* tests `test_S2…S6`, using fake clock.

### Phase 2 — Vision
- **T2.1** `FileCamera` + `FakeCamera` (synthetic moving rectangle).
- **T2.2** `OnnxDetector` (CPU dev) then `HailoDetector`. *Accept:* same `Detector` interface; static-image test finds a person.
- **T2.3** `tracker.py` (ByteTrack). *Accept:* IDs stable across a recorded clip; ID switch rate reported.
- **T2.4** `target_selector.py` — *Accept:* tap inside bbox selects it; tap on empty area → `lock_failed`.

### Phase 3 — FC integration (SITL)
- **T3.1** `FakeFC` + `mavlink_bridge.py`. *Accept:* against ArduPilot SITL, `send_velocity` moves the copter in the expected direction in GUIDED; mode change stops output (S6).
- **T3.2** `supervisor.py` + `main.py` wiring. *Accept:* end-to-end SITL run with `FakeCamera` following a synthetic target; drone yaws/climbs/approaches correctly.
- **T3.3** Fault injection tests: kill camera, drop heartbeat, NaN input → hover (S10).

### Phase 4 — Comms & GCS
- **T4.1** `gcs_link.py` + `command_sender/telemetry_client` — *Accept:* loopback test with packet loss/reorder; debounce 50–100 ms on tuning.
- **T4.2** `video_streamer.py` / `video_receiver.py` — *Accept:* local loopback < 150 ms glass-to-glass (measured with timestamp overlay).
- **T4.3** `video_canvas.py` tap → normalized coords (letterbox-aware); unit test for coordinate mapping.
- **T4.4** `tuning_drawer`, `telemetry_bar`, `calibration_modal`.

### Phase 5 — Hardware bring-up (human in the loop)
- **T5.1** `setup_rpi5_hailo.sh`, systemd units. **T5.2** Bench: 30+ FPS inference. **T5.3** Props-off FC link test. **T5.4** Tethered / low-altitude test with RC override rehearsed. *Agents must not mark Phase 5 done; a human signs off.*

---

## 11. Testing strategy

- **Unit** (fast, no hw): all pure logic; coverage target ≥ 85% on `control/`, `core/`, `vision/tracker`.
- **Integration**: SITL + fake camera; run in CI nightly (`make sitl-test`).
- **Replay**: `scripts/replay_log.py` feeds recorded `TrackState` logs into the control stack and asserts setpoints stay within limits.
- **Hardware** tests marked `@pytest.mark.hw`, skipped in CI.
- **Latency budget** (log per frame): capture→detect ≤ 25 ms, detect→setpoint ≤ 10 ms, end-to-end control ≤ 60 ms. Fail integration test if p95 exceeds budget.

---

## 12. Observability

- Structured JSON logs (`ts_ns`, `component`, `event`, fields); rotating files + in-memory ring buffer dumped on FAULT.
- Record every flight: `TrackState`, setpoints, FC state at control rate → `logs/<date>/flight.msgpack` (input to replay tool).
- Telemetry exposes `fps`, `latency_ms`, `dropped_frames`, `cpu_temp`, `fault`.

---

## 13. Common pitfalls (read before coding)

- Body-frame sign conventions: FRD, `vz` positive = **down**. Write a test before the code.
- Resend setpoints continuously (S9); one-shot commands will be ignored by ArduPilot.
- `type_mask` must ignore position/acceleration fields; only velocity + yaw_rate enabled.
- Hailo input is typically uint8 and letterboxed; keep the letterbox params to map boxes back to original pixels.
- Normalize image error to `[-1, 1]` about the **optical center** (from calibration), not the frame center, once calibrated.
- Don't use `time.sleep()` for loop pacing without compensating for execution time (use a fixed-step scheduler).
- Don't let the GCS slider spam saturate the datalink: debounce and send only the last value.
- Thermals: RPi5 under NPU + encode load throttles; log `vcgencmd get_throttled`.

---

## 14. Definition of done (per task)

1. Code + type hints + docstrings
2. Tests including the mapped safety test (if relevant)
3. `make lint typecheck test` green
4. No new hardware import at module top-level
5. Spec/docs updated if a contract changed (and ADR added for architectural decisions)