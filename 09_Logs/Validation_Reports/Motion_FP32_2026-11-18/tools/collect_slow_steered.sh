#!/usr/bin/env bash
# random_3 / kSteer (18/11/2026): as collect_slow_envelope.sh, plus a SCRATCH measurement patch of the envelope tool that
# also writes, per scenario, the envelope of the frames walked in the steered regime only (walkBox() = kSteer in the
# firmware: steered_), nominal targets and IMU-ray targets as the tool computes them (JOY_ENVELOPE_STEERED).
set -uo pipefail
C="$(cd "$(dirname "$0")" && pwd)"; S=$C/scratch
[ -d "$S/src" ] || { echo "run collect_slow_envelope.sh first (scratch tree)"; exit 1; }
J=$S/scripts/offline_motion/joystick_envelope.cpp; cp "$J" "$C/joystick_envelope_steered.cpp"
python3 - "$C/joystick_envelope_steered.cpp" <<'PY'
import sys
from pathlib import Path
p=Path(sys.argv[1]);t=p.read_text()
def rep(a,b):
    global t
    assert t.count(a)==1,a[:60];t=t.replace(a,b)
rep('  static bool steeredEnvelope(const SimpleMotion& m){return m.steeredEnvelope();}',
    '  static bool steeredEnvelope(const SimpleMotion& m){return m.steeredEnvelope();}\n  static bool steeredBox(const SimpleMotion& m){return m.steered_;}  // SCRATCH MEASUREMENT')
rep('      const auto& g=simple::StopReferenceProbe::gait(m);const auto& box=simple::StopReferenceProbe::box(m);++r.frames;',
    '      const auto& g=simple::StopReferenceProbe::gait(m);const auto& box=simple::StopReferenceProbe::box(m);++r.frames;\n      const bool steeredBox=simple::StopReferenceProbe::steeredBox(m);gSteeredFrames+=steeredBox;')
rep('const double q=motion::jointSlot(f.target,i);r.joint(i,q);','const double q=motion::jointSlot(f.target,i);r.joint(i,q);if(steeredBox)steeredJoint(i,q);')
rep('const double q=std::lround(q3[j]/simple::kTickRad)*simple::kTickRad;r.joint(i,q);','const double q=std::lround(q3[j]/simple::kTickRad)*simple::kTickRad;r.joint(i,q);if(steeredBox)steeredJoint(i,q);')
rep('  if(const char* env=std::getenv("JOY_ENVELOPE")){',
    '  if(const char* env=std::getenv("JOY_ENVELOPE_STEERED"))if(gSteeredFrames){FILE* e=std::fopen(env,"a");if(e){for(unsigned i=0;i<12;++i)std::fprintf(e,"%s%.9f %.9f",i?" ":"",gSLo[i],gSHi[i]);std::fprintf(e,"\\n");std::fclose(e);}}\n  if(const char* env=std::getenv("JOY_ENVELOPE")){')
# accumulator definitions at file scope, after the includes
i=t.index('namespace matdog')
t=t[:t.rfind('\n',0,i)+1]+'// SCRATCH MEASUREMENT (random_3 / kSteer, 18/11): envelope of the steered-regime frames of one scenario.\nstatic double gSLo[12]={1e9,1e9,1e9,1e9,1e9,1e9,1e9,1e9,1e9,1e9,1e9,1e9},gSHi[12]={-1e9,-1e9,-1e9,-1e9,-1e9,-1e9,-1e9,-1e9,-1e9,-1e9,-1e9,-1e9};static unsigned long gSteeredFrames=0;\nstatic void steeredJoint(unsigned i,double q){if(q<gSLo[i])gSLo[i]=q;if(q>gSHi[i])gSHi[i]=q;}\n'+t[t.rfind('\n',0,i)+1:]
p.write_text(t)
PY
src=("$S/src/simple/SimpleMotion.cpp" "$S/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$S/src/motion/$n.cpp"); done
g++ -std=c++17 -O2 -w -I "$S/scripts/tests/simple_host_stub" -I "$S/scripts/offline_motion" "$C/joystick_envelope_steered.cpp" "${src[@]}" -o "$C/joy_collect_steered"
mode=0; out=$C/m${mode}_steered; rm -rf "$out"; mkdir -p "$out"
limits=$(JOY_MODE=$mode "$C/joy_collect_steered" --limits)
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
for line in "${lines[@]}"; do name=${line%% *};args=${line#* }
  res=$(JOY_MODE=$mode JOY_ENVELOPE=$out/envelope.txt JOY_ENVELOPE_STEERED=$out/envelope_steered.txt "$C/joy_collect_steered" "$out/$name.txt" $args 1:0,0,0 2>/dev/null || true); echo "$name $res" >> "$out/collect.log"
done
cmp -s "$out/envelope.txt" "$C/m0/envelope.txt" && echo "all-regime envelope identical to collect_slow_envelope.sh" || echo "all-regime envelope DIFFERS"
echo "done $(wc -l < "$out/envelope_steered.txt") steered rows of ${#lines[@]} scenarios"
