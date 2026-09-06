"""The safety filter switch: yaml block, defaults, and the run flag.

The filter is opt-in. The shipped controller.yaml keeps it off, the yaml
block turns it on for every run, and --safety-filter / --no-safety-filter
beat the yaml for one run in either direction.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from robot_control.controller.config import (
    DEFAULT_CONTROLLER_YAML,
    ControllerConfigs,
    SafetyFilterConfig,
    load_controller_configs,
)
from robot_control.controller.safety_filter import SafetyFilter
from robot_control.core.types import WorkspaceConfig
from robot_control.runtime import build_safety_filter

REPO = Path(__file__).resolve().parents[1]
WS = WorkspaceConfig(width=40.0, height=60.0, car_width=7.0, car_height=7.0, offset_w=3.5, offset_h=3.5)


def _yaml(tmp_path: Path, body: str) -> Path:
    p = tmp_path / "controller.yaml"
    p.write_text(body)
    return p


def test_the_shipped_yaml_keeps_the_filter_off():
    cfg = load_controller_configs(DEFAULT_CONTROLLER_YAML)
    assert cfg.safety_filter.enabled is False
    assert cfg.safety_filter.robot_static_margin_cm == 1.5


def test_a_yaml_without_the_block_defaults_to_off(tmp_path):
    cfg = ControllerConfigs.from_yaml(_yaml(tmp_path, "navigation: {}\npush: {}\n"))
    assert cfg.safety_filter == SafetyFilterConfig()
    assert cfg.safety_filter.enabled is False


def test_the_yaml_block_loads(tmp_path):
    cfg = ControllerConfigs.from_yaml(_yaml(
        tmp_path, "navigation: {}\npush: {}\nsafety_filter:\n  enabled: true\n  robot_static_margin_cm: 2.25\n"
    ))
    assert cfg.safety_filter.enabled is True
    assert cfg.safety_filter.robot_static_margin_cm == 2.25


def test_defaults_carry_the_block():
    assert ControllerConfigs.defaults().safety_filter == SafetyFilterConfig()


@pytest.mark.parametrize("yaml_enabled,override,expect_filter", [
    (False, None, False),   # yaml decides: off
    (True, None, True),     # yaml decides: on
    (False, True, True),    # --safety-filter beats an off yaml
    (True, False, False),   # --no-safety-filter beats an on yaml
])
def test_the_flag_beats_the_yaml_in_both_directions(yaml_enabled, override, expect_filter):
    cfg = ControllerConfigs.defaults()
    cfg.safety_filter = SafetyFilterConfig(enabled=yaml_enabled, robot_static_margin_cm=1.75)
    f = build_safety_filter(cfg, WS, override)
    if expect_filter:
        assert isinstance(f, SafetyFilter)
        assert f.margin_cm == 1.75
    else:
        assert f is None


@pytest.mark.parametrize("script", ["scripts/run_namo.py", "scripts/execute_real_push.py"])
def test_both_launchers_expose_the_flag_pair(script):
    out = subprocess.run(
        [sys.executable, script, "--help"],
        cwd=REPO, capture_output=True, text=True, timeout=120,
    )
    assert out.returncode == 0, out.stderr[-2000:]
    assert "--safety-filter" in out.stdout
    assert "--no-safety-filter" in out.stdout
