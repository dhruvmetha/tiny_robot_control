#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/scratch/$USER/namo_less_depth}"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/namo_cpp/build_python}"
JOBS="${JOBS:-${SLURM_CPUS_PER_TASK:-4}}"
MODULE_CMAKE="${MODULE_CMAKE:-cmake/3.26.5}"
CONDA_ENV_SPEC="${CONDA_ENV_SPEC:-${CONDA_ENV_NAME:-namo312}}"
CONDA_SH_PATH="${CONDA_SH_PATH:-}"
MJ_PATH="${MJ_PATH:-}"
GIT_EXECUTABLE="${GIT_EXECUTABLE:-}"
CC_BIN="${CC_BIN:-}"
CXX_BIN="${CXX_BIN:-}"
BUILD_SYSROOT="${BUILD_SYSROOT:-${CONDA_BUILD_SYSROOT:-}}"

if [[ -z "$BUILD_SYSROOT" ]]; then
  default_sysroot="/scratch/$USER/sysroot217/x86_64-conda-linux-gnu/sysroot"
  if [[ -d "$default_sysroot" ]]; then
    BUILD_SYSROOT="$default_sysroot"
  fi
fi

if [[ -z "$CONDA_SH_PATH" ]]; then
  for candidate in \
    "/scratch/$USER/miniforge3/etc/profile.d/conda.sh" \
    "$HOME/miniforge3/etc/profile.d/conda.sh" \
    "$HOME/miniconda3/etc/profile.d/conda.sh"; do
    if [[ -f "$candidate" ]]; then
      CONDA_SH_PATH="$candidate"
      break
    fi
  done
fi

if [[ -z "${PYTHON_BIN:-}" ]]; then
  if [[ -d "$CONDA_ENV_SPEC" ]]; then
    PYTHON_BIN="$CONDA_ENV_SPEC/bin/python"
  else
    PYTHON_BIN="$HOME/miniconda3/envs/${CONDA_ENV_SPEC}/bin/python"
  fi
fi

if ! type module >/dev/null 2>&1; then
  source /etc/profile.d/modules.sh 2>/dev/null || true
  source /usr/share/lmod/lmod/init/bash 2>/dev/null || true
fi

if [[ -n "$CONDA_SH_PATH" && -f "$CONDA_SH_PATH" ]]; then
  # shellcheck disable=SC1091
  source "$CONDA_SH_PATH"
  conda activate "$CONDA_ENV_SPEC"
fi
export PATH="/usr/bin:$(dirname "$PYTHON_BIN"):$PATH"

if type module >/dev/null 2>&1; then
  module load "$MODULE_CMAKE"
fi

if [[ -z "$MJ_PATH" ]]; then
  echo "MJ_PATH is required" >&2
  exit 1
fi
export MJ_PATH

if [[ -d "$MJ_PATH/lib" ]]; then
  export LD_LIBRARY_PATH="$MJ_PATH/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

if [[ -z "$GIT_EXECUTABLE" ]]; then
  GIT_EXECUTABLE="$(command -v git || true)"
fi
if [[ -z "$GIT_EXECUTABLE" ]]; then
  echo "git executable not found" >&2
  exit 1
fi

if [[ -n "$BUILD_SYSROOT" ]]; then
  if [[ ! -d "$BUILD_SYSROOT" ]]; then
    echo "BUILD_SYSROOT does not exist: $BUILD_SYSROOT" >&2
    exit 1
  fi
  export CONDA_BUILD_SYSROOT="$BUILD_SYSROOT"
  export CFLAGS="${CFLAGS:+$CFLAGS }--sysroot=$BUILD_SYSROOT"
  export CXXFLAGS="${CXXFLAGS:+$CXXFLAGS }--sysroot=$BUILD_SYSROOT"
  export CPPFLAGS="${CPPFLAGS:+$CPPFLAGS }--sysroot=$BUILD_SYSROOT"
  export LDFLAGS="${LDFLAGS:+$LDFLAGS }--sysroot=$BUILD_SYSROOT"
fi

if [[ -z "$CC_BIN" ]]; then
  for candidate in \
    "$(dirname "$PYTHON_BIN")/x86_64-conda-linux-gnu-gcc" \
    "$(dirname "$PYTHON_BIN")/gcc"; do
    if [[ -x "$candidate" ]]; then
      CC_BIN="$candidate"
      break
    fi
  done
fi
if [[ -z "$CXX_BIN" ]]; then
  for candidate in \
    "$(dirname "$PYTHON_BIN")/x86_64-conda-linux-gnu-g++" \
    "$(dirname "$PYTHON_BIN")/g++" \
    "$(dirname "$PYTHON_BIN")/c++"; do
    if [[ -x "$candidate" ]]; then
      CXX_BIN="$candidate"
      break
    fi
  done
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake_args=(
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_PYTHON_BINDINGS=ON
  -DPython_EXECUTABLE="$PYTHON_BIN"
  -DPython3_EXECUTABLE="$PYTHON_BIN"
  -DGIT_EXECUTABLE="$GIT_EXECUTABLE"
)
if [[ -n "$BUILD_SYSROOT" ]]; then
  cmake_args+=(-DCMAKE_SYSROOT="$BUILD_SYSROOT")
fi
if [[ -n "$CC_BIN" ]]; then
  cmake_args+=(-DCMAKE_C_COMPILER="$CC_BIN")
fi
if [[ -n "$CXX_BIN" ]]; then
  cmake_args+=(-DCMAKE_CXX_COMPILER="$CXX_BIN")
fi

cmake "${cmake_args[@]}" "$REPO_ROOT/namo_cpp"
cmake --build "$BUILD_DIR" --target namo_rl -- -j"$JOBS"

"$PYTHON_BIN" - <<'PY' "$REPO_ROOT/namo_cpp" "$BUILD_DIR"
from pathlib import Path
import hashlib
import json
import sys
import time

source_root = Path(sys.argv[1])
build_dir = Path(sys.argv[2])
skip_dirs = {"build", "build_python", ".git", "__pycache__", ".pytest_cache"}

h = hashlib.sha256()
for path in sorted(p for p in source_root.rglob("*") if p.is_file()):
    rel = path.relative_to(source_root)
    if any(part in skip_dirs for part in rel.parts):
        continue
    if path.suffix == ".pyc":
        continue
    h.update(str(rel).encode("utf-8"))
    h.update(b"\0")
    h.update(path.read_bytes())
    h.update(b"\0")

artifacts = sorted(
    p.name for p in build_dir.iterdir()
    if p.is_file() and (p.name.startswith("namo_rl") and p.suffix in {".so", ".pyd"})
)
manifest = {
    "source_hash": h.hexdigest(),
    "built_at_epoch": time.time(),
    "artifacts": artifacts,
}
(build_dir / "namo_rl_build_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
PY

echo "Built namo_rl in $BUILD_DIR"
