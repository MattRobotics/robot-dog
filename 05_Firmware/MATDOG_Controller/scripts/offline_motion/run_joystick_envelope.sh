#!/usr/bin/env bash
# JOYSTICK envelope (dev.15), offline. Builds joystick_envelope.cpp on the
# production sources, runs fixed and seeded joystick scenarios and the URDF
# static-support check (offline_kinematics_dev5/walk_quality.py). PASS needs,
# in every scenario: no fault, end in STANDING, joint margin >= 0 to the WALK
# box, no IMU tilt-envelope ray outside the box/foot strip, static support
# margin > 0. Usage: run_joystick_envelope.sh [output-dir]. Geometry only.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)"
out="${1:-$(mktemp -d /tmp/matdog-joystick.XXXXXX)}";mkdir -p "$out"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sketch/scripts/tests/simple_host_stub" "$here/joystick_envelope.cpp" "${src[@]}" -o "$out/joy"
v=$(python3 -c 'print(.045/6.5)');h=$(python3 -c 'print(.015/6.5)')
declare -A sc=(
 [forward]="26:$v 1:0" [backward]="26:-$v 1:0" [slow_both]="20:$h 20:-$h 1:0"
 [reverse_twice]="8:$v 14:-$v 14:$v 1:0" [magnitude_steps]="6:$v 3:$h 3:$v 3:0.00001 6:$v 1:0"
 [silent_client]="15:$v 30:-" [release_in_start]="1:-$v 1:0" [silent_while_rising]="0.2:$v 30:-")
for seed in $(seq 1 12); do sc[random_$seed]="rand:$seed:160:$v"; done
fail=0;args=()
for name in $(printf '%s\n' "${!sc[@]}" | sort); do
  line=$("$out/joy" "$out/$name.txt" ${sc[$name]} || true);echo "$name $line" | tee -a "$out/envelope.log"
  [[ "$line" == *"end=STANDING"* && "$line" == *"tilt_fail=0 "* && "$line" != *"box_margin=-"* ]] || fail=1
  [[ -s "$out/$name.txt" ]] && args+=("$name" "$out/$name.txt")
done
(cd "$sketch/scripts/offline_kinematics_dev5" && python3 walk_quality.py "$out" "${args[@]}") | sed 's/ | COMx shift.*//' | tee "$out/support.log"
awk '{for(i=1;i<=NF;i++)if($i=="support"&&$(i+1)+0<=0)bad=1} END{exit bad}' "$out/support.log" || fail=1
echo "JOYSTICK envelope $([[ $fail == 0 ]] && echo PASS || echo FAIL): geometry and sequencing only, no hardware motion validation ($out)"
exit $fail
