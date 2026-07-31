#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE=LF_LOWER_M11_MAX
NORMACORE_HEAD=fff8e8989ca945bb56982ab5f626e3b45ba8b2dd
NORMACORE_BASE=32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
ROBOT_DOG_HEAD=01d4a43dee0585111b4fd337fdee9f59d769e510
GATE_BLOB=336992edfc4a049dee47deb490fd3b049419c7c0
RUNNER_BLOB=698610dff8aca9443e0e9943fe629fd143e9b432

NORMACORE_PRIMARY=${NORMACORE_PRIMARY:-$HOME/norma-core}
ROBOT_DOG_PRIMARY=${ROBOT_DOG_PRIMARY:-$HOME/MATDOG/github/robot-dog}
NORMACORE_WORKTREE=${NORMACORE_WORKTREE:-$HOME/MATDOG/worktrees/norma-core-lf-lower-m11-max-v30}
ARCHIVE_ROOT=${MATDOG_VERIFICATION_ARCHIVE:-$HOME/MATDOG/_archive/verification-artifacts}
STATION_CONFIG=${STATION_CONFIG:-$HOME/MATDOG/runtime/station/station.yaml}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
OUT="$ARCHIVE_ROOT/MATDOG_${PROFILE}_OFFLINE_EXECUTOR_V30_${STAMP}"
LOG="$OUT/executor.log"
SUMMARY="$OUT/SUMMARY.env"

GATE_PATH=06_Software/Matdog_Core/calibration/MATDOG_LF_LOWER_M11_MAX_OFFLINE_GATE_V29.sh
RUNNER_PATH=06_Software/Matdog_Core/calibration/MATDOG_LF_LOWER_M11_MAX_HARDWARE_RUNNER_V29.sh

fail() { printf 'HARD BLOCK: %s\n' "$*" >&2; exit 1; }
section() { printf '\n================================================================\n%s\n================================================================\n' "$1"; }
need() { command -v "$1" >/dev/null 2>&1 || fail "required command missing: $1"; }
clean_repo() { [[ -z "$(git -C "$1" status --porcelain)" ]] || fail "working tree is not clean: $1"; }

[[ ${1:-} == --apply ]] || {
  printf '%s\n' \
    "Usage:" \
    "  bash MATDOG_LF_LOWER_M11_MAX_OFFLINE_EXECUTOR_V30.sh --apply" \
    "" \
    "Offline only: no Station start, no serial open, no motor command."
  exit 64
}

mkdir -p "$OUT"
exec > >(tee -a "$LOG") 2>&1

section "PREFLIGHT — OFFLINE ONLY"
for cmd in git cargo npm sha256sum awk grep find sort tail cut tee pgrep; do
  need "$cmd"
done
[[ -d "$NORMACORE_PRIMARY/.git" ]] || fail "NormaCore primary clone missing"
[[ -d "$ROBOT_DOG_PRIMARY/.git" ]] || fail "robot-dog primary clone missing"
clean_repo "$NORMACORE_PRIMARY"
clean_repo "$ROBOT_DOG_PRIMARY"
[[ -f "$STATION_CONFIG" ]] || fail "Station configuration missing: $STATION_CONFIG"
grep -Fq 'st3215:' "$STATION_CONFIG" || fail "Station configuration has no st3215 section"
! pgrep -af '(^|/)station([[:space:]]|$)' >/dev/null || fail "Station is already running"

section "FETCH AND PIN REMOTE SOURCE"
git -C "$NORMACORE_PRIMARY" fetch matt \
  refs/heads/matdog/lf-lower-m11-min-v28r-alignment

git -C "$ROBOT_DOG_PRIMARY" fetch origin \
  refs/heads/matdog/lf-lower-m11-max-preparation

observed_normacore=$(git -C "$NORMACORE_PRIMARY" rev-parse FETCH_HEAD)
[[ "$observed_normacore" == "$NORMACORE_HEAD" ]] || fail "NormaCore remote head moved: $observed_normacore"

observed_robot_dog=$(git -C "$ROBOT_DOG_PRIMARY" ls-remote --exit-code --heads origin \
  refs/heads/matdog/lf-lower-m11-max-preparation | awk 'NR==1 {print $1}')
[[ "$observed_robot_dog" == "$ROBOT_DOG_HEAD" ]] || fail "robot-dog remote head moved: $observed_robot_dog"

git -C "$NORMACORE_PRIMARY" merge-base --is-ancestor "$NORMACORE_BASE" "$NORMACORE_HEAD" ||
  fail "NormaCore main base is not an ancestor of aligned source"

section "CREATE OR REUSE ISOLATED NORMACORE WORKTREE"
if [[ -e "$NORMACORE_WORKTREE" ]]; then
  [[ -e "$NORMACORE_WORKTREE/.git" ]] || fail "worktree path exists but is not a Git worktree"
  [[ "$(git -C "$NORMACORE_WORKTREE" rev-parse HEAD)" == "$NORMACORE_HEAD" ]] ||
    fail "existing worktree is at the wrong commit"
  clean_repo "$NORMACORE_WORKTREE"
  printf 'worktree_state=REUSED\n'
else
  mkdir -p "$(dirname "$NORMACORE_WORKTREE")"
  git -C "$NORMACORE_PRIMARY" worktree add --detach "$NORMACORE_WORKTREE" "$NORMACORE_HEAD"
  [[ "$(git -C "$NORMACORE_WORKTREE" rev-parse HEAD)" == "$NORMACORE_HEAD" ]] ||
    fail "new worktree head mismatch"
  clean_repo "$NORMACORE_WORKTREE"
  printf 'worktree_state=CREATED\n'
fi
printf 'normacore_worktree=%s\n' "$NORMACORE_WORKTREE"

section "EXTRACT EXACT REVIEWED GATE AND HARD-DISABLED RUNNER"
observed_gate_blob=$(git -C "$ROBOT_DOG_PRIMARY" rev-parse "$ROBOT_DOG_HEAD:$GATE_PATH")
observed_runner_blob=$(git -C "$ROBOT_DOG_PRIMARY" rev-parse "$ROBOT_DOG_HEAD:$RUNNER_PATH")
[[ "$observed_gate_blob" == "$GATE_BLOB" ]] || fail "offline gate blob mismatch"
[[ "$observed_runner_blob" == "$RUNNER_BLOB" ]] || fail "runner blob mismatch"

git -C "$ROBOT_DOG_PRIMARY" show "$ROBOT_DOG_HEAD:$GATE_PATH" > "$OUT/offline-gate-v29.sh"
git -C "$ROBOT_DOG_PRIMARY" show "$ROBOT_DOG_HEAD:$RUNNER_PATH" > "$OUT/hardware-runner-v29-hard-disabled.sh"
bash -n "$OUT/offline-gate-v29.sh"
bash -n "$OUT/hardware-runner-v29-hard-disabled.sh"
sha256sum "$OUT/offline-gate-v29.sh" "$OUT/hardware-runner-v29-hard-disabled.sh" > "$OUT/SCRIPT_SHA256SUMS"

section "RUN COMPLETE OFFLINE GATE"
NORMACORE_REPO="$NORMACORE_WORKTREE" \
MATDOG_VERIFICATION_ARCHIVE="$ARCHIVE_ROOT" \
bash "$OUT/offline-gate-v29.sh"

marker=$(find "$ARCHIVE_ROOT" -type f -name "OFFLINE_${PROFILE}_V29_PASS.env" \
  -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
[[ -n "$marker" && -f "$marker" ]] || fail "offline PASS marker not found after gate"
grep -qx 'result=PASS' "$marker" || fail "offline marker is not PASS"
grep -qx "profile=$PROFILE" "$marker" || fail "offline marker profile mismatch"
grep -qx "normacore_head=$NORMACORE_HEAD" "$marker" || fail "offline marker head mismatch"
grep -qx 'hardware_started=false' "$marker" || fail "offline marker hardware provenance invalid"
grep -qx 'serial_opened=false' "$marker" || fail "offline marker serial provenance invalid"

section "PREPARE HARD-DISABLED RUNNER CONTRACT"
set +e
NORMACORE_REPO="$NORMACORE_WORKTREE" \
STATION_CONFIG="$STATION_CONFIG" \
MATDOG_VERIFICATION_ARCHIVE="$ARCHIVE_ROOT" \
bash "$OUT/hardware-runner-v29-hard-disabled.sh" --prepare 2>&1 | tee "$OUT/runner-prepare.log"
runner_rc=${PIPESTATUS[0]}
set -e
[[ "$runner_rc" == 78 ]] || fail "hard-disabled runner returned unexpected status: $runner_rc"

prep_dir=$(find "$ARCHIVE_ROOT" -maxdepth 1 -type d -name "MATDOG_${PROFILE}_RUNNER_PREP_V29_*" \
  -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
[[ -n "$prep_dir" && -d "$prep_dir" ]] || fail "runner preparation directory not found"
[[ -f "$prep_dir/RUN_CONTRACT.env" ]] || fail "runner contract missing"
[[ -f "$prep_dir/STATION_LAUNCH_COMMAND.disabled" ]] || fail "disabled launch command missing"
grep -qx 'result=PREPARED_NOT_EXECUTED' "$prep_dir/RUN_CONTRACT.env" ||
  fail "runner contract state mismatch"
grep -qx 'hardware_started=false' "$prep_dir/RUN_CONTRACT.env" ||
  fail "runner contract hardware provenance invalid"
grep -qx 'serial_opened=false' "$prep_dir/RUN_CONTRACT.env" ||
  fail "runner contract serial provenance invalid"
[[ ! -x "$prep_dir/STATION_LAUNCH_COMMAND.disabled" ]] || fail "disabled command is executable"

section "WRITE EXECUTOR SUMMARY"
cat > "$SUMMARY" <<SUMMARY_EOF
result=PASS
profile=$PROFILE
normacore_head=$NORMACORE_HEAD
robot_dog_head=$ROBOT_DOG_HEAD
normacore_worktree=$NORMACORE_WORKTREE
offline_marker=$marker
runner_preparation=$prep_dir
station_config=$STATION_CONFIG
offline_tests=PASS
viewer_build=PASS
station_release_build=PASS
runner_state=PREPARED_NOT_EXECUTED
hardware_started=false
serial_opened=false
created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
SUMMARY_EOF
sha256sum "$SUMMARY" "$LOG" > "$OUT/SHA256SUMS"

cat "$SUMMARY"
printf '\nOffline executor artifact: %s\n' "$OUT"
printf 'No hardware was started. The generated Station command remains disabled.\n'
