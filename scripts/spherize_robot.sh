#!/usr/bin/env bash
# Spherize the X500 URDF using foam.
#
# foam gives each collision geometry min(0.7 * bounding-sphere volume /
# geometry volume, --branch) spheres.  The default cap of 8 covers the 40 cm
# frame with spheres up to 14 cm across its outline; --branch 16 halves the
# overreach.  The heuristic gives each flat rotor cylinder 3 spheres
# (10 cm radius, bulging that far above and below the propeller); the
# rotor factors raise that to 12 spheres of about 5.6 cm.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

# Relative paths: foam records the input path in the output URDF.
cd "$ROOT"
RESOURCES="drone_planning/resources/robot/x500"

python third_party/foam/scripts/generate_sphere_urdf.py \
    "$RESOURCES/x500.urdf" \
    --output "$RESOURCES/x500_spherized.urdf" \
    --database third_party/foam/sphere_database.json \
    --branch 16 \
    --rotor_0=4 --rotor_1=4 --rotor_2=4 --rotor_3=4 \
    "$@"
