#!/usr/bin/env bash
# 0.5 ms re-run of the four G2_NOMINAL lifecycles: confirms the 2 ms certified bounds (certified lower bound <= fine sampled minimum).
set -euo pipefail
cd "$(dirname "$0")"
PY="${PY:-/tmp/matdog-g35-env/bin/python}"
export OPENBLAS_NUM_THREADS=1
for c in WALK_357 WALK_287 TROT_61 TROT_309; do
  "$PY" lifecycle_v2.py --case "$c" --variant G2_NOMINAL --dt 0.0005 --tag _fine &
done
wait
