#!/usr/bin/env bash
# FAST 40 targeted variants on the study tree (lead ramps from STUDY_FRONT_RAMP[_END]): walk_support_check
# and the StopPlan RCA probe (stop_rca: walk_tilt_envelope_check verdicts + one row per failure).
# Usage: variants40.sh <study bin dir> <rca binary> <out dir> "name|lead_r,lead_re|flags...|ENV=v ..." ...
# (ENV: study-only switches of the study tree, e.g. STUDY_SURGE_MM=1 STUDY_SURGE_C0=.25)
set -euo pipefail
bin="$1"; rca="$2"; out="$3"; shift 3; mkdir -p "$out"
base="--mode 2 --stride 2 --stride-mm 110 --period 2.75 --duty .80 --ramp .35 --ramp-end .30 --height-mm 105 --lift-mm 6 --swayx-mm 4.5 --com-mm 11 --swayy-mm .75"
for v in "$@"; do
  IFS='|' read -r name lead flags envs <<<"$v"
  ( export STUDY_FRONT_RAMP="${lead%,*}" STUDY_FRONT_RAMP_END="${lead#*,}"; for e in ${envs:-}; do export "$e"; done
    "$bin/walk_support_check" $base $flags > "$out/$name.support.txt" || true
    STOP_RCA_CSV="$out/$name.rca.csv" "$rca" $base $flags > "$out/$name.tilt.txt" || true ) &
done; wait
for v in "$@"; do name="${v%%|*}"
  printf '%-14s %s | %s | rca %s\n' "$name" "$(grep -oE 'static support min [0-9.-]+|STOP capture min [0-9.-]+|joint vmax [0-9.]+|amax [0-9.]+ rad/s2 [A-Z_]+|HIP [0-9.]+ UPPER [0-9.]+' "$out/$name.support.txt" | tr '\n' ' ')" \
    "$(grep -oE 'fail nominal/ik/box/stop=[0-9/]+|box_margin_ticks nominal/compensated=[0-9./]+' "$out/$name.tilt.txt" | tr '\n' ' ')" \
    "$(tail -n +2 "$out/$name.rca.csv" | awk -F, '{k=$1" "$14; c[k]++; if($15>m[k])m[k]=$15} END{for(k in c)printf "[%s x%d max %.1f] ", k, c[k], m[k]}')"
done
