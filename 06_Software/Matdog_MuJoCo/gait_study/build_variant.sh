#!/usr/bin/env bash
# Offline gait study: a candidate patch on an EXPORTED copy (git archive, not a checkout) of a
# Motion tree, compiled into its own fw_bridge build. Nothing in any repository is modified.
# Usage: gait_study/build_variant.sh <commit> <patch | -> <out dir>
#   (then: MATDOG_FW_TREE=<out dir>/src/05_Firmware/MATDOG_Controller m6_dev7.py <out dir>/build/cosim ...)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; commit="$1"; patch="$2"; out="$(mkdir -p "$3" && cd "$3" && pwd)"
repo="$(git -C "$here" rev-parse --show-toplevel)"
rm -rf "$out/src" && mkdir -p "$out/src"
git -C "$repo" archive "$commit" 05_Firmware/MATDOG_Controller/src 05_Firmware/MATDOG_Controller/scripts | tar -x -C "$out/src"
if [ "$patch" != "-" ]; then patch -d "$out/src" -p1 --no-backup-if-mismatch < "$patch"; fi
MATDOG_FW_TREE="$out/src/05_Firmware/MATDOG_Controller" bash "$here/../fw_bridge/build.sh" "$out/build" >/dev/null
{ echo "commit $(git -C "$repo" rev-parse "$commit")"; echo "patch $patch $( [ "$patch" != "-" ] && sha256sum "$patch" | cut -c1-64)"; } > "$out/build/SOURCE.txt"
echo "built $out/build from $commit + $patch"
