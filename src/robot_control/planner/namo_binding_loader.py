"""Canonical namo_rl binding bootstrap for robot_control.

Strict policy:
- Only namo_cpp/build_python is allowed.
- No fallback to alternate build directories.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from types import ModuleType
from typing import Tuple


def _build_instructions() -> str:
    return (
        "  cmake -S . -B build_python -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON\n"
        "  cmake --build build_python --target namo_rl -j$(nproc)"
    )


# What identifies a namo_cpp checkout regardless of what the directory is called.
# The bindings source is the marker: it exists in that repo and nowhere else.
_NAMO_CPP_MARKER = Path("python") / "namo" / "cpp_bindings"

# Checked first, in order. env.<machine>.sh already exports NAMO_REPO.
_NAMO_CPP_ENV_VARS = ("NAMO_CPP_DIR", "NAMO_REPO")


def _looks_like_namo_cpp(candidate: Path) -> bool:
    return (candidate / _NAMO_CPP_MARKER).is_dir()


def resolve_namo_cpp_dir(anchor_file: Path) -> Path:
    """Resolve the namo_cpp checkout, whatever the directory happens to be named.

    Callers used to build this as ``<parent>/namo_cpp``, which only works where
    the checkout carries that name. It is called ``namo`` on at least one box,
    so the name is not something to hardcode. An env var wins if set; otherwise
    walk up from the anchor and take the first directory that holds the bindings
    source, preferring a child literally named namo_cpp when several match so
    the answer does not depend on directory ordering.
    """
    for var in _NAMO_CPP_ENV_VARS:
        raw = os.environ.get(var)
        if not raw:
            continue
        candidate = Path(raw).expanduser().resolve()
        if _looks_like_namo_cpp(candidate):
            return candidate
        raise RuntimeError(
            f"${var} is set to {candidate}, which does not hold {_NAMO_CPP_MARKER}. "
            f"Point it at the namo_cpp checkout or unset it."
        )

    anchor = anchor_file.resolve()
    for parent in [anchor, *anchor.parents]:
        if not parent.is_dir():
            continue
        if _looks_like_namo_cpp(parent):
            return parent
        preferred = parent / "namo_cpp"
        if _looks_like_namo_cpp(preferred):
            return preferred
        for child in sorted(parent.iterdir()):
            if child.is_dir() and _looks_like_namo_cpp(child):
                return child

    raise RuntimeError(
        f"Could not find a namo_cpp checkout from: {anchor_file}. Looked for a "
        f"directory holding {_NAMO_CPP_MARKER} in every parent, and at "
        f"{' / '.join(_NAMO_CPP_ENV_VARS)}."
    )


def resolve_namo_root(anchor_file: Path) -> Path:
    """The directory the namo_cpp checkout sits in.

    Kept because callers use it to reach sibling checkouts. Where the checkout
    itself lives is resolve_namo_cpp_dir's job.
    """
    return resolve_namo_cpp_dir(anchor_file).parent


def ensure_namo_cpp_paths(anchor_file: Path) -> Tuple[Path, Path, Path]:
    """Ensure canonical namo_cpp python and build paths are on sys.path."""
    namo_cpp_dir = resolve_namo_cpp_dir(anchor_file)
    namo_root = namo_cpp_dir.parent
    namo_cpp_python = (namo_cpp_dir / "python").resolve()
    canonical_build = (namo_cpp_dir / "build_python").resolve()

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
