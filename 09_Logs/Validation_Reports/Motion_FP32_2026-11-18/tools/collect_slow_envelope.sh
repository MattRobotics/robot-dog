#!/usr/bin/env bash
# random_3 / kSteer re-certification (Motion FP32 bench, 18/11/2026): per-joint envelope of the steered SLOW law of the
# candidate (trees/p1fp32), with the 11/11 procedure of kSteerStandard/kSteerFast (Motion_vNext_Multidirectional_
# 2026-11-11/collect_steer_envelope.sh): the gate scenarios of run_joystick_envelope.sh plus a 52-direction grid
# (26 directions x 0.5 and 1.0 of the limits), on a SCRATCH copy whose steered walks run on the gen-1 limits
# (measurement only: no walk ends before its envelope is recorded). Never in the firmware.
set -uo pipefail
C="$(cd "$(dirname "$0")" && pwd)"; E="$(cd "$C/.." && pwd)"; S=$C/scratch; rm -rf "$S"; mkdir -p "$S"
cp -r "$E/trees/p1fp32/src" "$S/src"; cp -r "$E/trees/p1fp32/scripts" "$S/scripts"
python3 - "$S/src/simple" <<'PY'
import sys
from pathlib import Path
d=Path(sys.argv[1])
def rep(f,a,b):
    p=d/f;t=p.read_text();assert t.count(a)==1,(f,a);p.write_text(t.replace(a,b))
K='static constexpr collision::Box kCollect={{-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738},{.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981}};'
rep('SimpleMotion.h','  const collision::Box& walkBox() const { return steered_?steerBox(walkMode_):kWalkBox; }',
    '  // SCRATCH COLLECTION ONLY: steered walks on the gen-1 limits.\n  '+K+'\n  const collision::Box& walkBox() const { return steered_?kCollect:kWalkBox; }')
rep('SimpleMotion.cpp','  const collision::Box& box=toSteered?steerBox(walkMode_):kWalkBox;','  const collision::Box& box=toSteered?kCollect:kWalkBox;  // SCRATCH COLLECTION ONLY')
rep('SimpleMotion.cpp','         contains(collision::kSteerStandard,q)||contains(collision::kSteerFast,q);',
    '         contains(collision::kSteerStandard,q)||contains(collision::kSteerFast,q)||(phase_==Phase::WALK&&steered_&&contains(kCollect,q));  // SCRATCH COLLECTION ONLY')
PY
src=("$S/src/simple/SimpleMotion.cpp" "$S/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$S/src/motion/$n.cpp"); done
g++ -std=c++17 -O2 -w -I "$S/scripts/tests/simple_host_stub" "$S/scripts/offline_motion/joystick_envelope.cpp" "${src[@]}" -o "$C/joy_collect"
mode=0; out=$C/m$mode; rm -rf "$out"; mkdir -p "$out"
limits=$(JOY_MODE=$mode "$C/joy_collect" --limits)
gate=$(sed -n "/^mapfile -t lines < <(JOY_LIMITS/,/^PY$/p" "$S/scripts/offline_motion/run_joystick_envelope.sh" | sed '1d;$d')
mapfile -t lines < <(JOY_LIMITS="$limits" python3 -c "$gate"; JOY_LIMITS="$limits" python3 - <<'PY'
import itertools,math,os
f=[float(x) for x in os.environ['JOY_LIMITS'].split()];lim=f[:3]
v=lambda a:','.join(repr(x) for x in a)
for s in itertools.product((-1,0,1),repeat=3):
    if s==(0,0,0):continue
    n=math.sqrt(sum(x*x for x in s))
    for amp in (.5,1):print('g'+''.join('m' if x<0 else str(x) for x in s)+f'_{amp}','26:'+v([amp*s[j]/n*lim[j] for j in range(3)]))
PY
)
echo "${#lines[@]} scenarios, limits $limits"
for line in "${lines[@]}"; do name=${line%% *};args=${line#* }
  res=$(JOY_MODE=$mode JOY_ENVELOPE=$out/envelope.txt "$C/joy_collect" "$out/$name.txt" $args 1:0,0,0 2>/dev/null || true); echo "$name $res" >> "$out/collect.log"
done
echo "done $(wc -l < "$out/envelope.txt") envelope rows"
