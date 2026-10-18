#!/usr/bin/env bash
# Generate FK/collision-checking C++ code for vamp using cricket.
# Generates the X500 VAMP robot model (6 DOF: x, y, z, yaw, gimbal pan,
# gimbal tilt) from x500_spherized.urdf + x500.srdf and installs it in
# ext/ompl_vamp/include/vamp/robots/x500.hh (NOT inside the vamp submodule).
#
# The cricket config checks motions every 1/64 of configuration distance
# ("resolution": 64): 1.6 cm of translation, below the 1.8 cm diameter of
# the smallest sphere, so a straight edge cannot step over a point obstacle.
#
# WARNING: cricket's CppAD codegen is not bit-stable across runs — the
# emitted formulas are mathematically equivalent but use different
# intermediate variable orderings and slightly different floating-point
# rounding.  Regenerate only when the URDF or SRDF actually changes.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

CRICKET="$ROOT/third_party/cricket"
RESOURCES="$ROOT/drone_planning/resources/robot/x500"
CRICKET_CONFIGS="$SCRIPT_DIR/cricket_configs"
FK_DEST="$ROOT/ext/ompl_vamp/include/vamp/robots"

# 1. Distribute robot descriptions and cricket configs into the cricket
#    submodule (kept out of submodule git tracking on purpose — the
#    canonical copies live in this repo).
echo "[1/3] Distributing robot descriptions..."
CRICKET_RES="$CRICKET/resources/x500"
mkdir -p "$CRICKET_RES"
cp "$RESOURCES/x500_spherized.urdf" "$CRICKET_RES/"
cp "$RESOURCES/x500.srdf"           "$CRICKET_RES/"
cp "$CRICKET_CONFIGS/"*.json        "$CRICKET/resources/"
echo "  copied to $CRICKET_RES and $CRICKET/resources/"

# 2. Run cricket FK code generation
echo "[2/3] Generating FK code..."
mkdir -p "$FK_DEST"
(cd "$CRICKET" && "$CRICKET/build/fkcc_gen" "resources/x500.json")
cp "$CRICKET/x500_fk.hh" "$FK_DEST/x500.hh"
echo "    installed $FK_DEST/x500.hh"

echo "[3/3] Done! Rebuild the package to use the new FK code."
