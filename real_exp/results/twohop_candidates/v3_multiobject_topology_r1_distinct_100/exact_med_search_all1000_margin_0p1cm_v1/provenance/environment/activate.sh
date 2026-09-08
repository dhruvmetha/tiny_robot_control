#!/bin/bash
# Canonical NAMO environment for Amarel compute nodes.
#
# Source this file inside an `srun --pty bash` shell (or a compute-node tmux)
# to get a working environment:
#
#   source scripts/amarel/activate.sh
#
# Every value below can be overridden by exporting the variable BEFORE
# sourcing, so this file stays a single source of truth without becoming
# a config registry. Idempotent — re-sourcing is safe.
#
# NOT for login nodes — modules + conda + LD_LIBRARY_PATH all expect a
# compute allocation.

if [ -z "${SLURM_JOB_ID:-}" ]; then
  echo "⚠️  No SLURM_JOB_ID set. You're either on a login node or in a stale shell."
  echo "    Grab a compute node first:"
  echo "      unset SLURM_JOB_ID"
  echo "      srun --partition=main --cpus-per-task=4 --mem=8G --time=2:00:00 --pty bash"
  echo "    then source this file."
  return 1 2>/dev/null || exit 1
fi

# ─── Workspace layout ──────────────────────────────────────────────────────
# NAMO_PARENT holds sibling clones (namo_cpp, sage_learning, mujoco_env_creator).
# Override to relocate the whole workspace; the other NAMO_* vars derive from it.
export NAMO_PARENT="${NAMO_PARENT:-/cache/home/dm1487/projects/namo}"
export NAMO_REPO="${NAMO_REPO:-$NAMO_PARENT/namo_cpp}"
export NAMO_SAGE="${NAMO_SAGE:-$NAMO_PARENT/sage_learning}"
export NAMO_ENV_CREATOR="${NAMO_ENV_CREATOR:-$NAMO_PARENT/mujoco_env_creator}"
# Canonical alias read by python (namo.paths) + env.<machine>.sh contract.
export SAGE_REPO="${SAGE_REPO:-$NAMO_SAGE}"

# ─── Data layout ───────────────────────────────────────────────────────────
# Everything large lives on /scratch (1 TB soft / 2 TB hard, NOT backed up,
# 90-day inactive purge — touch files periodically or stash a tar on /home).
export NAMO_DATA_ROOT="${NAMO_DATA_ROOT:-/scratch/dm1487}"
# NAMO_SCRATCH is the canonical base name read by python (namo.paths); keep it
# in lockstep with NAMO_DATA_ROOT.
export NAMO_SCRATCH="${NAMO_SCRATCH:-$NAMO_DATA_ROOT}"
export NAMO_DATASETS="${NAMO_DATASETS:-$NAMO_DATA_ROOT/datasets}"
export NAMO_MANIFESTS="${NAMO_MANIFESTS:-$NAMO_DATA_ROOT/manifests}"
export NAMO_OUTPUTS="${NAMO_OUTPUTS:-$NAMO_DATA_ROOT/outputs}"
export NAMO_LOGS="${NAMO_LOGS:-$NAMO_DATA_ROOT/logs}"
export NAMO_H5="${NAMO_H5:-$NAMO_DATA_ROOT/h5}"

# ─── Toolchain (compiler + CMake) — MODULE-FREE after the RHEL9 migration ───
# 2026-07 RHEL9 MIGRATION (OARC): compute nodes are now RHEL9 whose SYSTEM g++ is 11.5
# (C++17-ready — no compiler module needed), and lmod is BROKEN on them when you submit from
# the OLD CentOS7 login node (`ssh amarel` -> amarel1): `module load` dies with
# "module 'posix' not found". The legacy community tree also moved to /projects/community-old.
# So the build is now module-free:
#   compiler = the RHEL9 compute node's system g++ 11.5 (nothing to load)
#   cmake    = installed into the conda env (pip); on PATH after `conda activate`
# (Logging into the RHEL9 login node `amarel-new.hpc.rutgers.edu` should restore working
#  modules, but module-free is migration-proof. Old recipe was: module load gcc/12.3 cmake/3.26.5)

NAMO_CONDA_ENV="${NAMO_CONDA_ENV:-/scratch/dm1487/envs/namo}"
# Canonical interpreter name used by slurm/sh scripts.
export NAMO_PYTHON="${NAMO_PYTHON:-$NAMO_CONDA_ENV/bin/python}"
source "${NAMO_CONDA_PROFILE:-/cache/home/dm1487/miniforge3/etc/profile.d/conda.sh}"
conda activate "$NAMO_CONDA_ENV"
# CMake lives in the conda env (self-heal once; needed by ./build_python_bindings.sh).
command -v cmake >/dev/null 2>&1 || pip install -q cmake

export MJ_PATH="${MJ_PATH:-/scratch/dm1487/mujoco/mujoco-3.2.7}"
# conda env lib first so the g++-11-built .so finds a new-enough libstdc++ on any node.
export LD_LIBRARY_PATH="$MJ_PATH/lib:$NAMO_CONDA_ENV/lib:${LD_LIBRARY_PATH:-}"

# ─── Python bindings (canonical build_python/) ─────────────────────────────
# Built once via:  NAMO_MARCH=x86-64-v3 ./build_python_bindings.sh
# sage_learning is added so namo.visualization.mask_generation can import its
# NAMODataVisualizer (used by batch_collection.py for mask rendering).
export PYTHONPATH="$NAMO_REPO/build_python:$NAMO_REPO/python:$NAMO_SAGE:${PYTHONPATH:-}"

# ─── Hygiene ───────────────────────────────────────────────────────────────
# multiprocessing.Pool parallelizes our work; let BLAS run single-threaded
# so it doesn't oversubscribe the allocation.
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export PYTHONUNBUFFERED=1

cd "$NAMO_REPO"
echo "✓ NAMO env active on $(hostname). NAMO_REPO=$NAMO_REPO"
