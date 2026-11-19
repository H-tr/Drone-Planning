#!/usr/bin/env bash
# Spherize a robot's URDF using foam.
#
# Usage: spherize_robot.sh [ROBOT] [FOAM ARGS...]    ROBOT is x500 (the
# default) or drone.
#
# x500: foam gives each collision geometry min(0.7 * bounding-sphere volume /
# geometry volume, --branch) spheres.  The default cap of 8 covers the 40 cm
# frame with spheres up to 14 cm across its outline; --branch 16 halves the
# overreach.  The heuristic gives each flat rotor cylinder 3 spheres
# (10 cm radius, bulging that far above and below the propeller); the
# rotor factors raise that to 12 spheres of about 5.6 cm.
#
# drone: foam's medial fit of the body box with 9 spheres (--branch 9, one
# level).  The volume heuristic caps this box at int(0.7 * 5.97) = 4 spheres;
# --volume_heuristic_ratio 100 lifts the cap, so --branch sets the count.
# scripts/finish_drone_spheres.py then closes foam's small gaps and adds the
# camera's sphere (see its docstring) and writes drone_spherized.urdf.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"

ROBOT=x500
if [[ $# -gt 0 && "$1" != -* ]]; then
    ROBOT="$1"
    shift
fi

# Relative paths: foam records the input path in the output URDF.
cd "$ROOT"
RESOURCES="drone_planning/resources/robot/$ROBOT"

case "$ROBOT" in
x500)
    python third_party/foam/scripts/generate_sphere_urdf.py \
        "$RESOURCES/x500.urdf" \
        --output "$RESOURCES/x500_spherized.urdf" \
        --database third_party/foam/sphere_database.json \
        --branch 16 \
        --rotor_0=4 --rotor_1=4 --rotor_2=4 --rotor_3=4 \
        "$@"
    ;;
drone)
    mkdir -p build
    python third_party/foam/scripts/generate_sphere_urdf.py \
        "$RESOURCES/drone.urdf" \
        --output build/drone_foam.urdf \
        --database build/drone_sphere_database.json \
        --method medial --depth 1 --branch 9 \
        --volume_heuristic_ratio 100 \
        "$@"
    python scripts/finish_drone_spheres.py \
        build/drone_foam.urdf "$RESOURCES/drone.urdf" "$RESOURCES/drone_spherized.urdf"
    ;;
*)
    echo "Unknown robot '$ROBOT': x500 or drone" >&2
    exit 1
    ;;
esac
