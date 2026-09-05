"""The code fallback for a missing wavefront_inflation.yaml must equal the
yaml value on both sides of the bridge. A drift here is BUG-001: the two
wavefront grids disagree about which cells are blocked."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

from robot_control.utils.wavefront import WavefrontConfig
from robot_control.utils.wavefront_inflation_config import (
    DEFAULT_TIER1_BASE_INFLATION_MARGIN_M,
    WavefrontInflationConfig,
    load_wavefront_inflation_config,
)

REPO = Path(__file__).resolve().parent.parent
SIDECAR = REPO / "config" / "wavefront_inflation.yaml"
NAMO_CPP = REPO.parent / "namo_cpp"
NAMO_SIDECAR = NAMO_CPP / "config" / "wavefront_inflation.yaml"
NAMO_HEADER = NAMO_CPP / "include" / "wavefront" / "goal_tolerance_utils.hpp"
NAMO_SNAPSHOT = NAMO_CPP / "python" / "namo" / "visualization" / "wavefront_snapshot.py"


def _yaml_margin(path: Path) -> float:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return float(data["tier1"]["base_inflation_margin_m"])


def _regex_float(path: Path, pattern: str) -> float:
    match = re.search(pattern, path.read_text(encoding="utf-8"))
    assert match, f"{pattern!r} not found in {path}"
    return float(match.group(1))


def test_python_fallbacks_match_sidecar():
    sidecar = _yaml_margin(SIDECAR)
    assert DEFAULT_TIER1_BASE_INFLATION_MARGIN_M == sidecar
    assert WavefrontInflationConfig().tier1_base_inflation_margin_m == sidecar
    assert WavefrontConfig().inflation_margin == sidecar


def test_loader_falls_back_to_constant_when_key_missing(tmp_path: Path):
    empty = tmp_path / "wavefront_inflation.yaml"
    empty.write_text("tier1: {}\n", encoding="utf-8")
    cfg = load_wavefront_inflation_config(empty)
    assert cfg.tier1_base_inflation_margin_m == DEFAULT_TIER1_BASE_INFLATION_MARGIN_M


@pytest.mark.skipif(not NAMO_CPP.is_dir(), reason="sibling namo_cpp checkout not present")
def test_namo_cpp_sidecar_and_fallbacks_match():
    sidecar = _yaml_margin(SIDECAR)
    assert _yaml_margin(NAMO_SIDECAR) == sidecar
    header = _regex_float(
        NAMO_HEADER, r"kDefaultWavefrontTier1MarginM\s*=\s*([0-9.]+);"
    )
    assert header == sidecar
    snapshot = _regex_float(
        NAMO_SNAPSHOT, r"DEFAULT_TIER1_INFLATION_MARGIN_M:\s*float\s*=\s*([0-9.]+)"
    )
    assert snapshot == sidecar
