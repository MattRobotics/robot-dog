#!/usr/bin/env bash
# SIMPLE dashboard -> Controller command contract, offline (no device I/O).
# 1. real MotionService/SimpleMotion response bodies -> fixtures.json
# 2. the shipped page in jsdom, driven through its own controls -> envelopes.tsv
#    and the viewer's mobileMain.ts fed by that page (embedded) or by itself
# 3. those exact bodies through the real parser, origin, session/CSRF,
#    MotionService and SimpleMotion, plus the source-anchored handler order.
# Needs Node >= 20 with jsdom: NODE=/path/to/node NODE_PATH=/path/to/node_modules.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
sketch="$(cd "$here/../.." && pwd)"
out="$(mktemp -d /tmp/matdog-simple-dashboard.XXXXXX)"
trap 'rm -rf "$out"' EXIT
sources=("$sketch/src/simple/SimpleMotion.cpp" "$sketch/src/core/MotionService.cpp" "$sketch/src/core/MotionJson.cpp" "$sketch/src/motion/MotionContract.cpp" "$sketch/src/servo/ServoPopulation.cpp"
         "$sketch/src/network/PortalSecurity.cpp" "$sketch/src/network/NetworkConfig.cpp" "$sketch/src/update/Sha256.cpp" "$sketch/src/update/Hmac256.cpp")
for name in BodyPose ContactMode FootContact Gait GaitTiming ImuAttitude LegKinematics LegInverseKinematics StandTrajectory TiltCompensation TiltedContactIk; do sources+=("$sketch/src/motion/$name.cpp"); done
"${CXX:-g++}" -I "$here/simple_host_stub" -std=c++17 -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$here/test_simple_dashboard_contract.cpp" "${sources[@]}" -o "$out/contract"
"$out/contract" fixtures "$out/fixtures.json"
"${NODE:-node}" "$here/test_simple_dashboard_dom.cjs" "$out/fixtures.json" "$out/envelopes.tsv"
"${NODE:-node}" "$here/test_twin_feed_parent.mjs" "$out/fixtures.json"
"${NODE:-node}" "$here/test_portal_motion_budget_dom.cjs"
"$out/contract" envelopes "$out/envelopes.tsv" "$sketch/src"
