#!/usr/bin/env bash
set -euo pipefail
TESTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$TESTS/../.." && pwd)"
OUT="$(mktemp -d /tmp/matdog-profile-XXXXXX)"
trap 'rm -rf "$OUT"' EXIT
g++ -std=c++17 -O2 "$TESTS/test_validation_profile.cpp" "$FW/src/execution/ValidationProfile.cpp" "$FW/src/update/Sha256.cpp" "$FW/src/servo/ServoPopulation.cpp" "$FW"/src/motion/*.cpp -o "$OUT/test"
"$OUT/test"
