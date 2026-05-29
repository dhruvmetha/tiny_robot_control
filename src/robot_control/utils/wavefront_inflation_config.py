"""Centralized wavefront inflation config loader."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class WavefrontInflationConfig:
    """Inflation values in meters."""

    tier1_base_inflation_margin_m: float = 0.002
    navigation_additional_margin_m: float = 0.0
    push_approach_additional_margin_m: float = 0.003
    xml_min_separation_m: float = 0.005
    xml_collision_additional_margin_m: float = 0.08


DEFAULT_WAVEFRONT_INFLATION_YAML = (
    Path(__file__).parent.parent.parent.parent / "config" / "wavefront_inflation.yaml"
)

_CACHED_CONFIG: WavefrontInflationConfig | None = None


def _to_float(data: dict, section: str, key: str, default: float) -> float:
    section_data = data.get(section, {})
    try:
        return float(section_data.get(key, default))
    except (TypeError, ValueError):
        return default


def load_wavefront_inflation_config(
    yaml_path: Path = DEFAULT_WAVEFRONT_INFLATION_YAML,
) -> WavefrontInflationConfig:
    """Load wavefront inflation config, falling back to defaults."""
    if not yaml_path.exists():
        return WavefrontInflationConfig()

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    return WavefrontInflationConfig(
        tier1_base_inflation_margin_m=_to_float(
            data, "tier1", "base_inflation_margin_m", 0.002
        ),
        navigation_additional_margin_m=_to_float(
            data, "navigation", "additional_margin_m", 0.0
        ),
        push_approach_additional_margin_m=_to_float(
            data, "push_approach", "additional_margin_m", 0.003
        ),
        xml_min_separation_m=_to_float(
            data, "xml_collision_resolution", "min_separation_m", 0.005
        ),
        xml_collision_additional_margin_m=_to_float(
            data, "xml_collision_resolution", "additional_margin_m", 0.08
        ),
    )


def get_wavefront_inflation_config() -> WavefrontInflationConfig:
    """Get cached wavefront inflation config."""
    global _CACHED_CONFIG
    if _CACHED_CONFIG is None:
        _CACHED_CONFIG = load_wavefront_inflation_config()
    return _CACHED_CONFIG
