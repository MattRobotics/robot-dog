#!/usr/bin/env bash
# FAST 40 study: a generator variant through the firmware's own offline gates only (no twin build):
# export dev.7 (git archive), apply make_fast40_patch.py <args>, compile walk_support_check and
# walk_tilt_envelope_check with -Werror as run_walk_*_check.sh do, run the FAST presets 0/1/2 and
# reverse 50/58 mm, print one line against the selection rule (SimpleMotion.h: support >= 1.70 mm,
# capture >= 1.41 mm, HIP box >= 8.7 ticks, law vmax <= 2.393 rad/s, amax <= 27.0 rad/s2).
# Usage: offline_variant.sh <out dir> <generator args...>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; out="$(mkdir -p "$1" && cd "$1" && pwd)"; shift
repo="$(git -C "$here" rev-parse --show-toplevel)"; commit=b5610620d0cef40cdc34bd62bf6256857bc9cfde
FW=~/MATDOG/worktrees/matdog-dev7-b561062-src/05_Firmware/MATDOG_Controller
python3 "$here/make_fast40_patch.py" "$FW" "$out/variant.patch" "$@" >/dev/null
echo "$*" > "$out/ARGS.txt"
rm -rf "$out/src" && mkdir -p "$out/src"
git -C "$repo" archive "$commit" 05_Firmware/MATDOG_Controller/src 05_Firmware/MATDOG_Controller/scripts 03_CAD/URDF/matt_robodog_rev00 | tar -x -C "$out/src"
patch -s -d "$out/src" -p1 --no-backup-if-mismatch < "$out/variant.patch"
S="$out/src/05_Firmware/MATDOG_Controller"; src=("$S/src/simple/SimpleMotion.cpp" "$S/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$S/src/motion/$n.cpp"); done
for t in walk_tilt_envelope_check walk_support_check; do
  g++ -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I "$S/scripts/tests/simple_host_stub" "$S/scripts/offline_motion/$t.cpp" "${src[@]}" -o "$out/$t" & done; wait
cd "$out"
for p in 0 1 2; do (set +e; ./walk_tilt_envelope_check --mode 2 --stride $p > tilt_p$p.txt 2>&1; ./walk_support_check --mode 2 --stride $p > support_p$p.txt 2>&1) & done
for m in 50 58; do (set +e; ./walk_tilt_envelope_check --mode 2 --stride 2 --reverse 1 --stride-mm $m --swayy-mm 0.5 > tilt_rev$m.txt 2>&1; ./walk_support_check --mode 2 --stride 2 --reverse 1 --stride-mm $m --swayy-mm 0.5 > support_rev$m.txt 2>&1) & done; wait
python3 -I - "$out" <<'PY'
import re,sys,pathlib
d=pathlib.Path(sys.argv[1]);ok=True;parts=[]
for k in ("p0","p1","p2","rev50","rev58"):
    t=(d/f"tilt_{k}.txt").read_text() if (d/f"tilt_{k}.txt").exists() else "";s=(d/f"support_{k}.txt").read_text() if (d/f"support_{k}.txt").exists() else ""
    f=re.search(r"fail nominal/ik/box/stop=(\d+)/(\d+)/(\d+)/(\d+)",t);bm=re.search(r"box_margin_ticks nominal/compensated=([-\d.]+)/([-\d.]+)",t)
    vm=re.search(r"joint vmax=([\d.]+) rad/s (\S+)",t);am=re.search(r"amax=([\d.]+) rad/s2 (\S+)",t)
    sup=re.search(r"static support min ([-\d.]+) mm",s);cap=re.search(r"STOP capture min ([-\d.]+) mm",s);hip=re.search(r"box margin ticks HIP ([-\d.]+)",s)
    if not all((f,bm,vm,am,sup,cap,hip)):parts.append(f"{k}: PARSE/RUN FAILURE");ok=False;continue
    fails=sum(int(x) for x in f.groups());rule=fails==0 and float(sup[1])>=1.70 and float(cap[1])>=1.41 and float(hip[1])>=8.7 and float(vm[1])<=2.393 and float(am[1])<=27.0
    ok&=rule;parts.append(f"{k}: fail {f[1]}/{f[2]}/{f[3]}/{f[4]} sup {sup[1]} cap {cap[1]} hip {hip[1]} box {bm[2]} v {vm[1]} {vm[2]} a {am[1]} {am[2]}"+("" if rule else " RULE_FAIL"))
print(("RULE_PASS " if ok else "RULE_FAIL ")+d.name+" | "+" | ".join(parts))
PY
