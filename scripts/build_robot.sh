#!/usr/bin/env bash
# Build the planning-ready X500 description from the official PX4 models.
# Fetches PX4-gazebo-models (x500_base, x500, gimbal, x500_gimbal) at a
# pinned commit into build/ and converts the x500_gimbal SDF to
# drone_planning/resources/robot/x500/x500.urdf + meshes/
# (see scripts/px4_sdf_to_urdf.py for what the conversion changes).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

PX4_MODELS_URL="https://github.com/PX4/PX4-gazebo-models.git"
PX4_MODELS_COMMIT="e5997f455e33ec071c7c677fc59a65781de496a4"
SRC="$ROOT/build/PX4-gazebo-models"
RESOURCES="$ROOT/drone_planning/resources/robot/x500"

echo "[1/2] Fetching PX4-gazebo-models@${PX4_MODELS_COMMIT:0:7}..."
rm -rf "$SRC"
git init -q "$SRC"
git -C "$SRC" remote add origin "$PX4_MODELS_URL"
git -C "$SRC" sparse-checkout set models/x500_base models/x500 models/gimbal models/x500_gimbal
git -C "$SRC" fetch -q --depth 1 --filter=blob:none origin "$PX4_MODELS_COMMIT"
git -C "$SRC" checkout -q FETCH_HEAD

echo "[2/2] Converting x500_gimbal SDF to URDF..."
mkdir -p "$RESOURCES"
python "$SCRIPT_DIR/px4_sdf_to_urdf.py" "$SRC/models" "$RESOURCES"
cp "$SRC/models/x500/LICENSE" "$RESOURCES/LICENSE"
echo "  wrote $RESOURCES/x500.urdf"
