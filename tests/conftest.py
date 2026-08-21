"""Shared fixtures.

The scene helper lives in robot_control.utils.scene_xml, since tests are not
its only caller.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from robot_control.utils.scene_xml import portable_scene


@pytest.fixture
def scene_loader(tmp_path):
    """Load a captured scene by repo-relative path, portable on any box."""

    def _load(relative_scene: str) -> Path:
        scene = Path(__file__).resolve().parents[1] / "real_test_envs" / relative_scene
        if not scene.is_file():
            pytest.skip(f"missing captured scene: {scene}")
        return portable_scene(scene, tmp_path)

    return _load
