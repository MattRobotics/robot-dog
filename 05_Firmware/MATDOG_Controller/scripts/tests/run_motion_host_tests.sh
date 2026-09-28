#!/usr/bin/env bash
# G1/G2/G3 offline only: compile the production core, then use existing Python oracles.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
REPO_DIR="$(cd "$SKETCH_DIR/../.." && pwd)"
CXX="${CXX:-g++}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

python3 "$REPO_DIR/06_Software/Matdog_Core/kinematics/matdog_motion_geometry_export.py" --check
for TEST in test_motion_kinematics motion_oracle_driver; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -fno-exceptions -fno-rtti \
    -o "$OUT/$TEST" "$SCRIPT_DIR/$TEST.cpp" \
    "$SKETCH_DIR/src/motion/LegKinematics.cpp" \
    "$SKETCH_DIR/src/motion/LegInverseKinematics.cpp"
done
"$OUT/test_motion_kinematics"
python3 "$SCRIPT_DIR/test_motion_oracle.py" "$OUT/motion_oracle_driver"

python3 "$REPO_DIR/06_Software/Matdog_Core/kinematics/matdog_contact_stand_export.py" --check
for TEST in test_contact_stand contact_stand_oracle_driver; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -fno-exceptions -fno-rtti \
    -o "$OUT/$TEST" "$SCRIPT_DIR/$TEST.cpp" \
    "$SKETCH_DIR/src/motion/LegKinematics.cpp" \
    "$SKETCH_DIR/src/motion/LegInverseKinematics.cpp" \
    "$SKETCH_DIR/src/motion/FootContact.cpp" \
    "$SKETCH_DIR/src/motion/StandTrajectory.cpp" \
    "$SKETCH_DIR/src/motion/MotionState.cpp"
done
"$OUT/test_contact_stand"
python3 "$SCRIPT_DIR/test_contact_stand_oracle.py" "$OUT/contact_stand_oracle_driver"

for TEST in test_startup_timed_stand startup_timed_oracle_driver; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -fno-exceptions -fno-rtti \
    -o "$OUT/$TEST" "$SCRIPT_DIR/$TEST.cpp" \
    "$SKETCH_DIR/src/motion/LegKinematics.cpp" \
    "$SKETCH_DIR/src/motion/LegInverseKinematics.cpp" \
    "$SKETCH_DIR/src/motion/FootContact.cpp" \
    "$SKETCH_DIR/src/motion/StandTrajectory.cpp" \
    "$SKETCH_DIR/src/motion/MotionState.cpp" \
    "$SKETCH_DIR/src/motion/BodyPose.cpp" \
    "$SKETCH_DIR/src/motion/StartupAcquisition.cpp" \
    "$SKETCH_DIR/src/motion/TimedStand.cpp" \
    "$SKETCH_DIR/src/motion/StandTransition.cpp"
done
"$OUT/test_startup_timed_stand"
python3 "$SCRIPT_DIR/test_startup_timed_oracle.py" "$OUT/startup_timed_oracle_driver"
