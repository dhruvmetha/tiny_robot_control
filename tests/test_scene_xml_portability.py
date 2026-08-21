"""Cross-machine loading for captured MuJoCo scenes."""

from __future__ import annotations

import importlib
import os
import sys
from contextlib import contextmanager
from pathlib import Path

from robot_control.utils import scene_xml


STALE_SCENE = (
    '<mujoco><include file="/old/box/namo_cpp/test_xml/car/little_car.xml"/>'
    "</mujoco>"
)


def test_portable_copy_repoints_the_include_and_leaves_capture_untouched(
    monkeypatch, tmp_path
):
    source = tmp_path / "capture" / "env.xml"
    source.parent.mkdir()
    source.write_text(STALE_SCENE)
    current_namo = tmp_path / "checkout_named_namo"
    monkeypatch.setattr(scene_xml, "resolve_namo_cpp_dir", lambda _anchor: current_namo)

    with scene_xml.portable_scene_path(source) as loadable:
        assert loadable.exists()
        assert loadable != source
        assert (
            f'<include file="{current_namo}/test_xml/car/little_car.xml"/>'
            in loadable.read_text()
        )
        temporary_copy = loadable

    assert source.read_text() == STALE_SCENE
    assert not temporary_copy.exists()


def test_scene_without_the_captured_car_include_stays_at_its_original_path(
    tmp_path,
):
    source = tmp_path / "scene" / "env.xml"
    source.parent.mkdir()
    source.write_text('<mujoco><include file="assets/local.xml"/></mujoco>')

    with scene_xml.portable_scene_path(source) as loadable:
        assert loadable == source


def test_replay_keeps_the_portable_copy_for_the_whole_process(monkeypatch, tmp_path):
    source = tmp_path / "captured.xml"
    source.write_text(STALE_SCENE)
    loadable = tmp_path / "portable.xml"
    loadable.write_text("<mujoco/>")
    received = {}

    @contextmanager
    def fake_portable_scene_path(scene_path):
        received["source"] = Path(scene_path)
        received["inside"] = True
        yield loadable
        received["inside"] = False

    monkeypatch.setattr(scene_xml, "portable_scene_path", fake_portable_scene_path)

    module_name = "robot_control.diagnostics.sim_replay_subprocess"
    prior_qpos_dump = os.environ.get("NAMO_QPOS_DUMP")
    replay = importlib.import_module(module_name)
    try:
        def fake_main():
            received["xml_arg"] = Path(sys.argv[1])
            received["copy_exists"] = Path(sys.argv[1]).exists()
            received["context_open"] = received["inside"]
            return 17

        monkeypatch.setattr(replay, "_main", fake_main)
        monkeypatch.setattr(
            sys,
            "argv",
            ["sim_replay_subprocess.py", str(source), "chain.json", "out.mp4"],
        )
        result = replay.main()
    finally:
        Path(replay._QPOS_PATH).unlink(missing_ok=True)
        if prior_qpos_dump is None:
            os.environ.pop("NAMO_QPOS_DUMP", None)
        else:
            os.environ["NAMO_QPOS_DUMP"] = prior_qpos_dump
        sys.modules.pop(module_name, None)

    assert result == 17
    assert received["source"] == source
    assert received["xml_arg"] == loadable
    assert received["copy_exists"] is True
    assert received["context_open"] is True
    assert received["inside"] is False
    assert sys.argv[1] == str(source)
