#!/usr/bin/env bash
# Generate FK/collision-checking C++ code for vamp using cricket.
#
# Usage: generate_fk.sh [ROBOT]    ROBOT is x500 (the default) or drone.
#
# Generates the VAMP model of ROBOT (6 DOF: x, y, z, yaw, gimbal pan,
# gimbal tilt) from <robot>_spherized.urdf + <robot>.srdf in
# drone_planning/resources/robot/<robot>/ and installs it in
# ext/ompl_vamp/include/vamp/robots/<robot>.hh (NOT inside the vamp
# submodule).  Only the named robot's header is written.
#
# The cricket configs (scripts/cricket_configs/<robot>.json) check motions
# every 1/64 of configuration distance ("resolution": 64): 1.6 cm of
# translation, below the 1.8 cm diameter of the X500's smallest sphere, so
# a straight edge cannot step over a point obstacle.
#
# WARNING: cricket's CppAD codegen is not bit-stable across runs — the
# emitted formulas are mathematically equivalent but use different
# intermediate variable orderings and slightly different floating-point
# rounding.  Regenerate only when the URDF or SRDF actually changes.
set -euo pipefail

ROBOT="${1:-x500}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

CRICKET="$ROOT/third_party/cricket"
RESOURCES="$ROOT/drone_planning/resources/robot/$ROBOT"
CRICKET_CONFIGS="$SCRIPT_DIR/cricket_configs"
FK_DEST="$ROOT/ext/ompl_vamp/include/vamp/robots"

if [[ ! -f "$CRICKET_CONFIGS/$ROBOT.json" ]]; then
    echo "Unknown robot '$ROBOT': no $CRICKET_CONFIGS/$ROBOT.json" >&2
    exit 1
fi

# 1. Distribute robot descriptions and cricket configs into the cricket
#    submodule (kept out of submodule git tracking on purpose — the
#    canonical copies live in this repo).
echo "[1/3] Distributing robot descriptions..."
CRICKET_RES="$CRICKET/resources/$ROBOT"
mkdir -p "$CRICKET_RES"
cp "$RESOURCES/${ROBOT}_spherized.urdf" "$CRICKET_RES/"
cp "$RESOURCES/$ROBOT.srdf"             "$CRICKET_RES/"
cp "$CRICKET_CONFIGS/"*.json            "$CRICKET/resources/"
echo "  copied to $CRICKET_RES and $CRICKET/resources/"

# 2. Run cricket FK code generation
echo "[2/3] Generating FK code..."
mkdir -p "$FK_DEST"
(cd "$CRICKET" && "$CRICKET/build/fkcc_gen" "resources/$ROBOT.json")
cp "$CRICKET/${ROBOT}_fk.hh" "$FK_DEST/$ROBOT.hh"
echo "    installed $FK_DEST/$ROBOT.hh"

echo "[3/3] Done! Rebuild the package to use the new FK code."
