#!/usr/bin/env bash
# Historical defects on the versions that shipped them (dev.6 consolidation),
# host only. Builds the rig of test_simple_motion.cpp AT THAT REVISION with the
# public-API probes of version_repro.inc (metrics, no asserts):
#   R1 forward stick with 3-13 % lateral touch jitter (dev.4 ground: 14 mm sway)
#   R2 steer, then forward with that jitter (review 09/11, dev.5)
#   R3 held FORWARD -> DIAGONAL -> FORWARD (law change, dev.5/dev.6)
#   R4 WALK STANDARD 100 for 160 s (T2 STOP_PLAN fault, dev.3..dev.6)
# Usage: run_version_repro.sh <git-rev> [label]   (e.g. 4358f13 dev4)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)";repo="$(git -C "$here" rev-parse --show-toplevel)";rev=$1;label=${2:-$1}
out="$(mktemp -d /tmp/matdog-repro.XXXXXX)";trap 'rm -rf "$out"' EXIT
git -C "$repo" archive "$rev" 05_Firmware/MATDOG_Controller/src 05_Firmware/MATDOG_Controller/scripts/tests | tar -x -C "$out"
s="$out/05_Firmware/MATDOG_Controller"
python3 -I - "$s/scripts/tests/test_simple_motion.cpp" "$here/version_repro.inc" <<'PY'
import sys;p,inc=sys.argv[1:];t=open(p).read();i=t.index("int main(){")
t=t[:i]+open(inc).read()+"int main(){repro::jitterForward();repro::jitterAfterSteer();repro::lawChange();repro::longWalk();return 0;}\nstatic int unused_main(){"+t[i+len("int main(){"):]
open(p,'w').write(t)
PY
src=("$s/src/simple/SimpleMotion.cpp" "$s/src/core/MotionService.cpp" "$s/src/core/MotionJson.cpp" "$s/src/motion/MotionContract.cpp" "$s/src/servo/ServoPopulation.cpp")
for n in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do src+=("$s/src/motion/$n.cpp"); done
"${CXX:-g++}" -w -I "$s/scripts/tests/simple_host_stub" -std=c++17 -O2 -DMATDOG_T2_SHADOW "$s/scripts/tests/test_simple_motion.cpp" "${src[@]}" -o "$out/repro"
"$out/repro" | sed "s/^/$label /"
