#!/usr/bin/env bash
# FAST 40 study: walk each build (label, FAST law period:stride_mm:height_mm, build dir with cosim and
# src/) over the fast_* scenarios with the corrected twin (m6_dev7 variant dev7_ground).
# Usage: OUT=<dir> SC="fast_fwd ..." [EXTRA="--set k=v ..."] run_twin.sh "label law dir" ...   (dir "dev7": the M6 dev.7 build)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; sim="$here/../.."
M6=/home/matteo-manicardi/MATDOG/verification-artifacts/MATDOG_MUJOCO_DEV7_CORRELATION_20261112
DEV7_FW=/home/matteo-manicardi/MATDOG/worktrees/matdog-dev7-b561062-src/05_Firmware/MATDOG_Controller
SC="${SC:-fast_fwd fast_fwd_32 fast_fwd_stop fast_back fast_back_stop fast_left fast_right fast_yaw fast_yaw_stop fast_diag fast_diag_yaw fast_sweep fast_modes_cycle fast_stopcyc_0 fast_stopcyc_1 fast_stopcyc_2 fast_stopcyc_3 fast_stopcyc_4 fast_stopcyc_5 fast_stopcyc_6 fast_stopcyc_7}"
mkdir -p "$OUT"; jobs=""
for spec in "$@"; do
  read -r label law dir <<<"$spec"
  if [ "$dir" = dev7 ]; then bin=$M6/build/cosim; fw=$DEV7_FW; else bin=$dir/build/cosim; fw=$dir/src/05_Firmware/MATDOG_Controller; fi
  for s in $SC; do jobs="$jobs$label $law $bin $fw $s\n"; done
done
printf "$jobs" | xargs -P "${JOBS:-8}" -L1 sh -c 'cd '"$sim"' && MATDOG_FW_TREE=$3 ~/MATDOG/tools/mujoco-venv/bin/python m6_gait.py $2 '"$OUT"' $0 $4 --variant dev7_ground --fast-law $1 '"${EXTRA:-}"' > '"$OUT"'/$4__$0.log 2>&1 || echo "FAILED $0 $4"'
# a model perturbation (--set) must reach the twin: every JSON records its model in "definition"
if [[ "${EXTRA:-}" == *--set* ]]; then
  ~/MATDOG/tools/mujoco-venv/bin/python -c 'import glob,json,sys
sys.path.insert(0,sys.argv[2]);import m6_dev7
base=json.loads(json.dumps(m6_dev7.VARIANTS["dev7_ground"]))
same=[f for f in glob.glob(sys.argv[1]+"/*.json") if json.load(open(f))["definition"]==base]
sys.exit("EXTRA not applied: %d runs carry the unperturbed model" % len(same) if same else 0)' "$OUT" "$sim"
fi
echo TWIN_DONE
