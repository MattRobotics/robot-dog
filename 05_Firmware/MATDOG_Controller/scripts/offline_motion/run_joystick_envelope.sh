#!/usr/bin/env bash
# JOYSTICK envelope (dev.15), offline. Builds joystick_envelope.cpp on the
# production sources and runs the steered scenario set: six directions, the
# eight ellipsoid octants, a fore/aft reversal, a silent client, release in
# the start ramp and seeded random command sequences. PASS needs, in every
# scenario: no fault, end in STANDING, joint margin >= 0 to the walk box, no
# IMU tilt-envelope ray outside the box/fillet limit. foot_edge_contact.py then
# reports, per scenario, the static support margin with the real fillet
# contacts and the URDF COM (> 0 required, steady and random) and the non-foot
# ground clearance (>= 2 mm required); a scenario that never reaches WALK
# (release in start) is NOT_APPLICABLE there and still gated on sequencing.
# Usage: run_joystick_envelope.sh [output-dir]. Geometry only.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)"
out="${1:-$(mktemp -d /tmp/matdog-joystick.XXXXXX)}";mkdir -p "$out"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sketch/scripts/tests/simple_host_stub" "$here/joystick_envelope.cpp" "${src[@]}" -o "$out/joy"
mapfile -t lines < <(python3 - <<'PY'
import itertools,math
lim=[.045/6.5,.020/6.5,.20/6.5]
v=lambda a:','.join(repr(x) for x in a)
print('forward','26:'+v([lim[0],0,0]));print('backward','26:'+v([-lim[0],0,0]))
print('left','26:'+v([0,lim[1],0]));print('right','26:'+v([0,-lim[1],0]))
print('ccw','26:'+v([0,0,lim[2]]));print('cw','26:'+v([0,0,-lim[2]]))
for s in itertools.product((1,-1),repeat=3):
    print('octant_'+''.join('p' if x>0 else 'm' for x in s),'26:'+v([s[j]*lim[j]/math.sqrt(3) for j in range(3)]))
print('reverse_twice','8:'+v([lim[0],0,0])+' 14:'+v([-lim[0],0,0])+' 14:'+v([lim[0],lim[1]/2,0]))
print('silent_client','15:'+v([lim[0]/2,lim[1]/2,lim[2]/2])+' 30:-')
print('release_in_start','1:'+v([-lim[0],0,lim[2]/2]))
for seed in range(1,9):print(f'random_{seed}',f'rand:{seed}:160')
PY
)
fail=0;steady=();random=()
for line in "${lines[@]}"; do name=${line%% *};args=${line#* }
  [[ "$name" == random_* ]] && random+=("$name" "$out/$name.txt") || steady+=("$name" "$out/$name.txt")
  res=$("$out/joy" "$out/$name.txt" $args 1:0,0,0 || true);echo "$name $res" | tee -a "$out/envelope.log"
  [[ "$res" == *"end=STANDING"* && ( "$res" == *"walk=NOT_APPLICABLE"* || ( "$res" == *"tilt_fail=0 "* && "$res" != *"box_margin=-"* ) ) ]] || fail=1
done
contact=(env PYTHONDONTWRITEBYTECODE=1 python3 "$here/foot_edge_contact.py" frames)
"${contact[@]}" "$out/contact_steady.json" "${steady[@]}" | tee "$out/contact_steady.log"
"${contact[@]}" "$out/contact_random.json" "${random[@]}" | tee "$out/contact_random.log"
# Numeric values only: NOT_APPLICABLE carries none.
gate='{for(i=1;i<NF;i++)if($(i+1)~/^-?[0-9.]+$/){if($i=="support"&&$(i+1)+0<=0)bad=1;if($i=="clearance"&&$(i+1)+0<2)bad=1}} END{exit bad}'
awk "$gate" "$out/contact_steady.log" "$out/contact_random.log" || fail=1
summary=$(cat "$out/contact_steady.log" "$out/contact_random.log" | awk '$3~/^-?[0-9.]+$/{if(s==""||$3+0<s){s=$3+0;sn=$1}if(c==""||$9+0<c){c=$9+0;cn=$1}} END{printf "min support %.2f mm (%s), min non-foot clearance %.2f mm (%s)",s,sn,c,cn}')
tilt=$(grep -o 'compensated=[0-9.]*/[0-9.]*' "$out/envelope.log" | cut -d/ -f2 | sort -n | tail -1)
echo "summary: $summary, max compensated stance-foot tilt $tilt deg" | tee -a "$out/envelope.log"
echo "JOYSTICK envelope $([[ $fail == 0 ]] && echo PASS || echo FAIL): geometry and sequencing only, no hardware motion validation ($out)"
exit $fail
