"""Canonical namo_rl binding bootstrap for robot_control.

Strict policy:
- Only namo_cpp/build_python is allowed.
- No fallback to alternate build directories.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType
from typing import Tuple


def _build_instructions() -> str:
    return (
        "  cmake -S . -B build_python -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON\n"
        "  cmake --build build_python --target namo_rl -j$(nproc)"
    )


def resolve_namo_root(anchor_file: Path) -> Path:
    """Resolve <...>/namo root by walking parents and finding namo_cpp/python."""
    anchor = anchor_file.resolve()
    for parent in [anchor, *anchor.parents]:
        candidate = parent / "namo_cpp" / "python"
        if candidate.is_dir():
            return parent
    raise RuntimeError(
        f"Could not resolve namo root from: {anchor_file}. Expected to find namo_cpp/python in a parent directory."
    )


def ensure_namo_cpp_paths(anchor_file: Path) -> Tuple[Path, Path, Path]:
    """Ensure canonical namo_cpp python and build paths are on sys.path."""
    namo_root = resolve_namo_root(anchor_file)
    namo_cpp_python = (namo_root / "namo_cpp" / "python").resolve()
    canonical_build = (namo_root / "namo_cpp" / "build_python").resolve()

    if str(namo_cpp_python) not in sys.path:
        sys.path.insert(0, str(namo_cpp_python))

    if not canonical_build.is_dir() or not any(canonical_build.glob("namo_rl*.so")):
        raise RuntimeError(
            "Canonical namo_rl build missing at namo_cpp/build_python.\n"
            "Build with:\n"
            f"{_build_instructions()}"
        )

    if str(canonical_build) not in sys.path:
        sys.path.insert(0, str(canonical_build))

    return namo_root, namo_cpp_python, canonical_build


def assert_canonical_namo_rl(module: object, canonical_build: Path) -> Path:
    """Assert imported namo_rl module was loaded from canonical build path."""
    module_path = Path(getattr(module, "__file__", "")).resolve()
    if canonical_build not in module_path.parents:
        raise RuntimeError(
            "Loaded namo_rl from non-canonical path.\n"
            f"  loaded:   {module_path}\n"
            f"  expected: {canonical_build}\n"
            "Fix PYTHONPATH so namo_cpp/build_python is first."
        )
    return module_path


def load_canonical_namo_rl(anchor_file: Path) -> Tuple[ModuleType, Path, Path]:
    """Load and validate namo_rl from canonical namo_cpp/build_python."""
    _, _, canonical_build = ensure_namo_cpp_paths(anchor_file)
    namo_rl = importlib.import_module("namo_rl")
    module_path = assert_canonical_namo_rl(namo_rl, canonical_build)
    return namo_rl, module_path, canonical_build
