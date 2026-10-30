#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)"
out="${MATDOG_STUDY_OUTPUT:?set an isolated artifact directory}"
mkdir -p "$out"
sources=();for n in BodyPose ContactMode FootContact LegKinematics LegInverseKinematics TiltCompensation TiltedContactIk ImuAttitude Gait GaitTiming StandTrajectory;do sources+=("$sketch/src/motion/$n.cpp");done
g++ -std=c++17 -O2 "$here/route_probe.cpp" "${sources[@]}" -o "$out/route_probe"
"$out/route_probe" > "$out/route.csv"
python3 "$here/analyze_route.py" "$out/route.csv" "$out" > "$out/route_analysis.log"
python3 "$here/rest_study.py" "$out/route.csv" "$out" > "$out/rest_analysis.log"
python3 "$here/check_study.py" "$out" > "$out/geometry_checks.log"
