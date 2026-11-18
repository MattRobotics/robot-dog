#!/usr/bin/env bash
# Motion FP32 host bench (18/11/2026, from the tolerance review bench): copies trees/<tree>/src to out/tol_<tree>/src, adds ONLY the bench numeric probes
# (rig/tol_probes.py; MotionTiming.h keeps esp_timer = the simulated clock), runs rig/tol_scenarios.h; writes stats and
# every RAW frame (frames_<scenario>.bin). Usage: run_tol.sh <tree>
set -euo pipefail
E="$(cd "$(dirname "$0")" && pwd)"; t=$1; R=$E/rig; out=$E/out/tol_$t${TOL_NOISE_K0:+_k$TOL_NOISE_K0}${TOL_EPS:+_eps}${TOL_KICK_K:+_kick$TOL_KICK_K}; rm -rf "$out"; mkdir -p "$out"
cp -r "$E/trees/$t/src" "$out/src"; python3 -I "$R/fp32_probes.py" "$out/src"
T=$out/src; Q=$E/trees/$t/scripts/offline_motion/qemu_motion
src=("$T/simple/SimpleMotion.cpp" "$T/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$T/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -w -I "$E/trees/$t/scripts/tests/simple_host_stub" -I "$R" -I "$T/simple" -I "$T/motion" -I "$Q" "$R/tol_host_main.cpp" "${src[@]}" -o "$out/host"
"$out/host" "$out" > "$out/tol.txt"; grep -E '^(SCENARIO|TOL)' "$out/tol.txt" | cut -c1-330
