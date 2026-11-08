#!/usr/bin/env bash
# usage: run.sh <name> <lateralM> <yawRad> <rampCycles> [periodS]
set -e; S=$(dirname "$0"); N=$1; LAT=$2; YAW=$3; RAMP=$4; PER=${5:-6.5}; W=$S/$N; rm -rf $W; mkdir -p $W
R=~/MATDOG/worktrees/matdog-gait-redesign-20261108/05_Firmware/MATDOG_Controller
cp -r $R/src $W/src; cp -r $R/scripts $W/scripts
sed -i "s/constexpr double kJoystickStrideM=.045, kJoystickLateralM=.020, kJoystickYawRad=.20, kJoystickRampCycles=2;/constexpr double kJoystickStrideM=.045, kJoystickLateralM=$LAT, kJoystickYawRad=$YAW, kJoystickRampCycles=$RAMP;/" $W/src/simple/SimpleMotion.h
sed -i "s/constexpr double kJoystickPeriodS=6.5,/constexpr double kJoystickPeriodS=$PER,/" $W/src/simple/SimpleMotion.h
grep -q "kJoystickLateralM=$LAT" $W/src/simple/SimpleMotion.h
src=("$W/src/simple/SimpleMotion.cpp" "$W/src/servo/ServoPopulation.cpp"); for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$W/src/motion/$n.cpp"); done
g++ -std=c++17 -O2 -w -I $W/scripts/tests/simple_host_stub $W/scripts/offline_motion/joystick_envelope.cpp "${src[@]}" -o $W/joy
mapfile -t lines < <(LAT=$LAT YAW=$YAW PER=$PER python3 - <<'PY'
import itertools,math,os
lat,yaw,per=float(os.environ['LAT']),float(os.environ['YAW']),float(os.environ['PER'])
lim=[.045/per,lat/per,yaw/per];v=lambda a:','.join(repr(x) for x in a)
print('left','26:'+v([0,lim[1],0]));print('right','26:'+v([0,-lim[1],0]));print('ccw','26:'+v([0,0,lim[2]]));print('cw','26:'+v([0,0,-lim[2]]))
for s in itertools.product((1,-1),repeat=3):print('octant_'+''.join('p' if x>0 else 'm' for x in s),'26:'+v([s[j]*lim[j]/math.sqrt(3) for j in range(3)]))
turn=round(2*math.pi/yaw*per+2,1);print('turn_cw',f'{turn}:'+v([0,0,-lim[2]]))
print('reverse_twice','8:'+v([lim[0],lim[1]/2,0])+' 14:'+v([-lim[0],0,0])+' 14:'+v([lim[0],lim[1]/2,0]))
print('silent_client','15:'+v([lim[0]/2,lim[1]/2,lim[2]/2])+' 30:-')
for seed in range(1,5):print(f'random_{seed}',f'rand:{seed}:160')
PY
)
names=()
for line in "${lines[@]}"; do n=${line%% *};a=${line#* };res=$($W/joy $W/$n.txt $a 1:0,0,0 || true);echo "$N $n $(echo $res | grep -o 'box_margin=[-0-9.]* mrad\|tilt_margin=[-0-9.]* mrad\|tilt_fail=[0-9]*\|compensated=[0-9./]*\|max_target_step=[0-9.]*\|end=[A-Z_ ]*' | tr '\n' ' ')";names+=("$n" "$W/$n.txt");done > $W/envelope.log
cd $R/scripts/offline_motion && env PYTHONDONTWRITEBYTECODE=1 python3 foot_edge_contact.py frames $W/contact.json "${names[@]}" > $W/contact.log
awk -v N=$N '{if($3+0<s||s==""){s=$3+0;sn=$1} if($9+0<c||c==""){c=$9+0;cn=$1}} END{printf "%s min support %.2f (%s) min clearance %.2f (%s)\n",N,s,sn,c,cn}' $W/contact.log
grep -c "end=STANDING" $W/envelope.log | sed "s/^/$N scenarios STANDING: /"; grep -v "tilt_fail=0 " $W/envelope.log | head -3; grep -o "box_margin=[-0-9.]*" $W/envelope.log | sort -t= -k2 -n | head -1 | sed "s/^/$N min /"
