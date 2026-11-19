#!/usr/bin/env bash
# P0 WALK NOMINAL bench: copies trees/<tree>/src to out/walk_<tree>/src, adds ONLY the bench probes (rig/bench_probes.sh),
# runs rig/walk_scenarios.h on the shared qemu_motion Sim: host (ns) and ESP32-S3 in QEMU (-icount shift=3, CCOUNT).
# Usage: [QSCEN_MASK=n] run_walk.sh <tree> [host]   (Motion FP32 bench 18/11: scenario mask, S = fixture STAND/HEIGHT/PRONE)
set -euo pipefail
E="$(cd "$(dirname "$0")" && pwd)"; t=$1; R=$E/rig; out=$E/out/walk_$t${QSCEN_MASK:+_m$QSCEN_MASK};mask=${QSCEN_MASK:+-DQSCEN_MASK=$QSCEN_MASK}; rm -rf "$out"; mkdir -p "$out"
cp -r "$E/trees/$t/src" "$out/src"; bash "$R/bench_probes.sh" "$out/src"
T=$out/src; Q=$E/trees/$t/scripts/offline_motion/qemu_motion
src=("$T/simple/SimpleMotion.cpp" "$T/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$T/motion/$n.cpp"); done
inc=(-I "$T/simple" -I "$T/motion" -I "$Q" -I "$R")
"${CXX:-g++}" -std=c++17 -O2 -w $mask -I "$E/trees/$t/scripts/tests/simple_host_stub" "${inc[@]}" "$R/host_main.cpp" "${src[@]}" -o "$out/host"
"$out/host" > "$out/host.txt"; grep -E '^(START|SCENARIO|HEAVY|BUDGET)' "$out/host.txt" | cut -c1-260
[ "${2:-}" = host ] && exit 0
sk=$out/walk; mkdir -p "$sk"; cp "$R/walk.ino" "$R/walk_scenarios.h" "$R/probe_clock.cpp" "$R/qprobe.h" "$Q/motion_scenarios.h" "$sk/"
k=0; for f in "${src[@]}"; do printf '#include "%s"\n' "$f" > "$sk/motion_src_$k.cpp"; k=$((k+1)); done
"${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}" compile --clean --fqbn esp32:esp32:esp32s3:CDCOnBoot=default,PSRAM=disabled,FlashMode=dio,FlashSize=4M \
  --build-property "compiler.cpp.extra_flags=-I$T/simple -I$T/motion -I$R $mask" --build-property "compiler.c.elf.extra_flags=-Wl,--wrap=esp_timer_get_time" \
  --output-dir "$out/build" "$sk" > "$out/compile.txt" 2>&1 || { tail -30 "$out/compile.txt"; exit 1; }
qemu="${MATDOG_QEMU:-$(ls -d "$HOME"/MATDOG/tools/qemu-esp/esp_develop_*/*/bin 2>/dev/null | tail -1)/qemu-system-xtensa}"
timeout 5400 "$qemu" -nographic -machine esp32s3 -icount shift=3 -drive file="$out/build/walk.ino.merged.bin",if=mtd,format=raw -serial mon:stdio > "$out/qemu.txt" 2>&1 &
pid=$!; while kill -0 $pid 2>/dev/null; do grep -q '^DONE' "$out/qemu.txt" 2>/dev/null && { kill $pid; break; }; sleep 3; done; wait $pid 2>/dev/null || true
tr -d '\r' < "$out/qemu.txt" | grep -E '^(PLATFORM|REL|START|SCENARIO|HEAVY|BUDGET|STACK|DONE)' > "$out/target.txt" || true
echo "== target"; grep -E '^(START|SCENARIO|HEAVY|BUDGET|DONE)' "$out/target.txt" | cut -c1-260
