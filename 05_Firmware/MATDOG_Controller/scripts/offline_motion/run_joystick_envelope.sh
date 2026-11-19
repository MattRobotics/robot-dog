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
# vNext audit (2026-11-11): every other scenario must walk (frames > 0; for
# release_in_start the walk is optional, SLOW walks a few frames there) and
# none may have a joystick refresh refused (WALK_STOPPING of a reversal aside);
# the tool's exit status is part of the verdict.
# Usage: [JOY_MODE=0|1|2] run_joystick_envelope.sh [output-dir]. Geometry only. 46 scenarios.
# JOY_MODE (vNext): the WALK mode whose own law the joystick walks (default SLOW);
# the scenario speeds are that mode's limits, printed by the tool (--limits).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";sketch="$(cd "$here/../.." && pwd)"
out="${1:-$(mktemp -d /tmp/matdog-joystick.XXXXXX)}";mkdir -p "$out"
src=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$sketch/src/motion/$n.cpp"); done
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$sketch/scripts/tests/simple_host_stub" "$here/joystick_envelope.cpp" "${src[@]}" -o "$out/joy"
limits=$("$out/joy" --limits)
mapfile -t lines < <(JOY_LIMITS="$limits" python3 - <<'PY'
import itertools,math,os
# SLOW: steered .045/.020/.16 per 5 s cycle, forward 100 mm, backward 50 mm (dev.5/dev.6);
# vNext: the selected mode's JoystickSpec per cycle of its period (joystick_envelope --limits).
f=[float(x) for x in os.environ['JOY_LIMITS'].split()]
lim=f[:3];pub,back=f[3],f[4]
v=lambda a:','.join(repr(x) for x in a)
print('forward','26:'+v([pub,0,0]));print('backward','26:'+v([-pub,0,0]))
print('forward_steered','6:'+v([lim[0],lim[1]/2,0])+' 20:'+v([lim[0],0,0]));print('backward_steered','6:'+v([-lim[0],lim[1]/2,0])+' 20:'+v([-lim[0],0,0]))
print('frontal_changes','6:'+v([.2*pub,0,0])+' 6:'+v([pub,0,0])+' 6:'+v([.1*pub,0,0])+' 8:'+v([.7*pub,0,0]))
# Ground HW 08/11: a forward stick carried 0.1-0.4 mm/s of lateral, alternating sides.
print('frontal_jitter',' '.join('0.48:'+v([.0069,s*1e-4,0]) for s in [1,2,-3,4,-1,3,-2,1]*7))
print('frontal_to_steer','10:'+v([pub,0,0])+' 20:'+v([pub/2,lim[1]/2,0]))
# dev.6 held-stick sequences, one press, no WALK STOP expected (only the reversal stops).
h=lambda f,l,y:v([f,l,y])
print('held_fwd_diag_lat_rot_backright_fwd','12:'+h(pub,0,0)+' 15:'+h(.7*pub,-.7*lim[1],0)+' 15:'+h(0,-lim[1],0)+' 15:'+h(0,0,lim[2])+' 15:'+h(-.5*lim[0],-.7*lim[1],0)+' 30:'+h(pub,0,0))
print('held_rev_lat_fwd','12:'+h(-back,0,0)+' 15:'+h(0,lim[1],0)+' 30:'+h(pub,0,0))
print('held_yaw_lat_fwd','15:'+h(.5*lim[0],.5*lim[1],.5*lim[2])+' 30:'+h(pub,0,0))
# Stride increases while walking (kJoystickFrontalRampCycles), at several phases, and seeded fore/aft sequences.
for k,t0 in enumerate((6.0,7.2,8.4)):print(f'frontal_accel{k}',f'{t0}:'+v([.1*pub,0,0])+' 12:'+v([pub,0,0]))
import random
for seed in range(1,4):
    random.seed(seed);steps=[];t=0
    while t<120:
        d=.5+random.random()*3.5;x=random.uniform(-1,1)*pub
        steps.append(f'{d:.2f}:'+v([x,random.uniform(-4e-4,4e-4),0]) if random.random()>=.15 else f'{d:.2f}:0,0,0');t+=d
    print(f'frontal_random_{seed}',' '.join(steps))
print('left','26:'+v([0,lim[1],0]));print('right','26:'+v([0,-lim[1],0]))
print('ccw','26:'+v([0,0,lim[2]]));print('cw','26:'+v([0,0,-lim[2]]))
for s in itertools.product((1,-1),repeat=3):
    print('octant_'+''.join('p' if x>0 else 'm' for x in s),'26:'+v([s[j]*lim[j]/math.sqrt(3) for j in range(3)]))
# Small steady lateral/yaw demands (motion-perf-next): 2.9 % stays in the low sway band
# (<= 5 mm), 3.3 % takes the full sway. A steady mid amplitude (~6 mm) sits on the
# fillet contact band and gave negative static support with a linear sway map.
for k,tag in ((.029,'small'),(.033,'mid')):
    print(f'lat_{tag}','20:'+v([.8*lim[0],k*lim[1],0]));print(f'yaw_{tag}','20:'+v([.8*lim[0],0,k*lim[2]]))
# Full turns (motion-perf-next review): the compensation envelope is bounded in the
# heading frame, so the hip/fillet loading must not depend on the accumulated heading.
turn=round(2*math.pi/lim[2]+2,1)
print('turn_cw',f'{turn}:'+v([0,0,-lim[2]]));print('turn_ccw',f'{turn}:'+v([0,0,lim[2]]))
print('reverse_twice','8:'+v([lim[0],0,0])+' 14:'+v([-lim[0],0,0])+' 14:'+v([lim[0],lim[1]/2,0]))
print('silent_client','15:'+v([lim[0]/2,lim[1]/2,lim[2]/2])+' 30:-')
print('release_in_start','1:'+v([-lim[0],0,lim[2]/2]))
for seed in range(1,9):print(f'random_{seed}',f'rand:{seed}:160')
PY
)
fail=0;steady=();random=()
for line in "${lines[@]}"; do name=${line%% *};args=${line#* }
  [[ "$name" == random_* ]] && random+=("$name" "$out/$name.txt") || steady+=("$name" "$out/$name.txt")
  expect=();[[ "$name" == release_in_start ]] && expect=(JOY_EXPECT_NOWALK=1)
  rc=0;res=$(env "${expect[@]}" "$out/joy" "$out/$name.txt" $args 1:0,0,0) || rc=$?;echo "$name $res" | tee -a "$out/envelope.log"
  [[ $rc == 0 && "$res" == *"end=STANDING"* && ( "$res" == *"walk=NOT_APPLICABLE"* || ( "$res" == *"tilt_fail=0 "* && "$res" != *"box_margin=-"* ) ) ]] || fail=1
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
echo "JOYSTICK envelope mode ${JOY_MODE:-0} $([[ $fail == 0 ]] && echo PASS || echo FAIL): geometry and sequencing only, no hardware motion validation ($out)"
exit $fail
