#!/usr/bin/env bash
# STOP secant gate (F3 2026-11-16), offline: every sent reference, with one RAW
# tick of quantization, inside the StopPlan frontier of its drive, for every
# WALK mode, preset, joystick direction and transition and the suite's E -> F3
# sequence (flat, fixture tilt, slope, lag, late releases). Exit 0 = PASS.
# Usage: run_stop_secant_gate.sh [output-dir] [--verbose]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)"
out="${1:-$(mktemp -d /tmp/matdog-secant.XXXXXX)}";mkdir -p "$out"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sketch/scripts/tests/simple_host_stub" "$here/stop_secant_gate.cpp" "${src[@]}" -o "$out/gate"
"$out/gate" ${2:-} | tee "$out/stop_secant_gate.txt"
