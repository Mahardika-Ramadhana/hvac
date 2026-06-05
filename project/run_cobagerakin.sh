#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONTROLLER_BUILD="${PROJECT_DIR}/controller/build/lib/choreonoid-2.4/simplecontroller"
MOTION_DIR="${PROJECT_DIR}/motion"

if [[ ! -f "${CONTROLLER_BUILD}/RobotBesarWalkController.so" ]]; then
  echo "Building controllers..."
  cmake -B "${PROJECT_DIR}/controller/build" -DCMAKE_BUILD_TYPE=Release \
    -S "${PROJECT_DIR}/controller"
  cmake --build "${PROJECT_DIR}/controller/build"
fi

if [[ ! -f "${MOTION_DIR}/walk_in_place.seq" ]]; then
  echo "Generating motion files..."
  python3 "${PROJECT_DIR}/scripts/generate_walk_motion.py"
fi

export ROBOT_BESAR_MOTION_DIR="${MOTION_DIR}"
export LD_LIBRARY_PATH="${CONTROLLER_BUILD}:${LD_LIBRARY_PATH:-}"

cd "${PROJECT_DIR}"
# KinematicSimulator (no physics) + controller that drives joints AND
# root-link translation. Click "Start Simulation" (green ▶) – the robot
# will walk 5 s in place, 10 s forward, 8 s backward without falling.
exec choreonoid \
  --add-plugin-dir-as-prefix "${PROJECT_DIR}/controller/build" \
  --project "${PROJECT_DIR}/cobagerakin_nodes_robot_besar.cnoid" \
  --start-simulation \
  "$@"
