"""Tests for air-unit/config/settings_schema.py — T0.3 acceptance tests.

- Defaults load successfully
- Invalid YAML raises clear error
- Values exceeding S8 hard limits are rejected
"""

from __future__ import annotations

from pathlib import Path

import pytest
from air_unit.config.settings_schema import (
    ControlSettings,
    PidGains,
    SettingsLoadError,
    load_settings,
)

from shared.constants import HARD_MAX_VX, HARD_MAX_YAW_RATE

# ── Default loading ──────────────────────────


class TestDefaultSettings:
    """Defaults load without any YAML file (T0.3 acceptance)."""

    def test_defaults_load(self) -> None:
        s = load_settings(path=None)
        assert s.camera.width == 1280
        assert s.camera.height == 720
        assert s.camera.fps == 30
        assert s.fc.baud == 921_600
        assert s.control.hz == 30
        assert s.control.max_vx == 3.0
        assert s.pid.yaw.kp == 1.0
        assert s.safety.target_lost_timeout_s == 1.0
        assert s.network.telemetry_port == 5005

    def test_settings_are_frozen(self) -> None:
        s = load_settings(path=None)
        with pytest.raises((TypeError, ValueError)):
            s.camera = None  # type: ignore[assignment]


# ── YAML file loading ───────────────────────


class TestYamlLoading:
    """Loading from YAML file."""

    def test_valid_yaml(self, tmp_path: Path) -> None:
        yaml_content = """
camera:
  width: 640
  height: 480
  fps: 60
  source: v4l2
control:
  max_vx: 2.0
pid:
  yaw: {kp: 0.5, ki: 0.1, kd: 0.02}
"""
        f = tmp_path / "settings.yaml"
        f.write_text(yaml_content, encoding="utf-8")
        s = load_settings(f)
        assert s.camera.width == 640
        assert s.camera.source == "v4l2"
        assert s.control.max_vx == 2.0
        assert s.pid.yaw.kp == 0.5
        # Unspecified fields get defaults
        assert s.fc.baud == 921_600

    def test_empty_yaml_returns_defaults(self, tmp_path: Path) -> None:
        f = tmp_path / "settings.yaml"
        f.write_text("", encoding="utf-8")
        s = load_settings(f)
        assert s.camera.width == 1280

    def test_file_not_found(self) -> None:
        with pytest.raises(SettingsLoadError, match="not found"):
            load_settings("/nonexistent/settings.yaml")


# ── Invalid YAML raises clear error ─────────


class TestInvalidYaml:
    """Invalid YAML must raise a clear error (T0.3 acceptance)."""

    def test_malformed_yaml(self, tmp_path: Path) -> None:
        f = tmp_path / "settings.yaml"
        f.write_text("camera:\n  width: [invalid", encoding="utf-8")
        with pytest.raises(SettingsLoadError, match="Invalid YAML"):
            load_settings(f)

    def test_non_dict_yaml(self, tmp_path: Path) -> None:
        f = tmp_path / "settings.yaml"
        f.write_text("- item1\n- item2", encoding="utf-8")
        with pytest.raises(SettingsLoadError, match="must be a mapping"):
            load_settings(f)


# ── S8 hard limit validation ────────────────


class TestHardLimitValidation:
    """Values exceeding S8 hard limits must be rejected."""

    def test_max_vx_exceeds_hard_limit(self) -> None:
        with pytest.raises(Exception, match="HARD_MAX_VX"):
            ControlSettings(max_vx=HARD_MAX_VX + 1.0)

    def test_max_yaw_rate_exceeds_hard_limit(self) -> None:
        with pytest.raises(Exception, match="HARD_MAX_YAW_RATE"):
            ControlSettings(max_yaw_rate=HARD_MAX_YAW_RATE + 1.0)

    def test_pid_kp_exceeds_hard_limit(self) -> None:
        with pytest.raises(Exception, match="HARD_MAX_PID_KP"):
            PidGains(kp=10.0)

    def test_follow_distance_below_min(self) -> None:
        with pytest.raises(Exception, match="follow_distance_m"):
            ControlSettings(follow_distance_m=0.5)

    def test_follow_distance_above_max(self) -> None:
        with pytest.raises(Exception, match="follow_distance_m"):
            ControlSettings(follow_distance_m=100.0)

    def test_valid_values_within_limits(self) -> None:
        c = ControlSettings(max_vx=2.0, max_vz=1.0, max_yaw_rate=1.0, follow_distance_m=6.0)
        assert c.max_vx == 2.0
