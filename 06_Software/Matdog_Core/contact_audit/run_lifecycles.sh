#!/usr/bin/env bash
# Run the eight lifecycle audits (4 cases x 2 contact configurations) at 2 ms, four at a time.
set -euo pipefail
cd "$(dirname "$0")"
PY="${PY:-/tmp/matdog-g35-env/bin/python}"
export OPENBLAS_NUM_THREADS=1
for variant in G2_NOMINAL G2_1_REGISTERED_CANDIDATE; do
  for c in WALK_357 WALK_287 TROT_61 TROT_309; do
    "$PY" lifecycle_v2.py --case "$c" --variant "$variant" --dt 0.002 &
  done
  wait
done
