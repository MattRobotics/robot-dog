#!/usr/bin/env bash
# Builds the two host bridges to the production firmware sources of this
# worktree (compiled unchanged): fk_reference (LegKinematics + FootContact) and
# export_sequences (SimpleMotion on the scripted rig, as export_walk_replay.cpp)
# and cosim (the same SimpleMotion fed by the MuJoCo twin, closed loop).
# Usage: fw_bridge/build.sh <build dir>   (then: <build dir>/export_sequences <out dir>)
# MATDOG_FW_TREE=<MATDOG_Controller of another checkout> compiles that tree instead
# (unchanged; its commit and dirty-file count go to <build dir>/SOURCE.txt).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; sk="$(cd "${MATDOG_FW_TREE:-$here/../../../05_Firmware/MATDOG_Controller}" && pwd)"; out="$(mkdir -p "$1" && cd "$1" && pwd)"
m="$sk/src/motion"
g++ -std=c++17 -O2 -Wall -I "$sk/src" "$here/fk_reference.cpp" "$m/LegKinematics.cpp" "$m/LegInverseKinematics.cpp" "$m/FootContact.cpp" -o "$out/fk_reference"
src=("$sk/src/simple/SimpleMotion.cpp" "$sk/src/core/MotionService.cpp" "$sk/src/core/MotionJson.cpp" "$sk/src/motion/MotionContract.cpp" "$sk/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$m/$n.cpp"); done
for tool in export_sequences cosim; do
  g++ -std=c++17 -O2 -w -DMATDOG_T2_SHADOW -I "$sk/scripts/tests/simple_host_stub" -I "$sk/scripts" -I "$here" "$here/$tool.cpp" "${src[@]}" -o "$out/$tool"
done
( cd "$sk" && { git rev-parse HEAD 2>/dev/null || echo "not a git checkout: $sk"; } && { git status --short -- . 2>/dev/null || true; } | wc -l ) > "$out/SOURCE.txt"
echo "built $out/fk_reference $out/export_sequences $out/cosim from $sk"
