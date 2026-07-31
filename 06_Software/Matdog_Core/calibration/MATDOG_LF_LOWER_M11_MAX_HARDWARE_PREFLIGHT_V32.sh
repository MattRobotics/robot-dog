#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE=LF_LOWER_M11_MAX
BUS_ID=5B14114953
NORMACORE_HEAD=fff8e8989ca945bb56982ab5f626e3b45ba8b2dd
NORMACORE_WORKTREE=${NORMACORE_WORKTREE:-$HOME/MATDOG/worktrees/norma-core-lf-lower-m11-max-v30}
ARCHIVE_ROOT=${MATDOG_VERIFICATION_ARCHIVE:-$HOME/MATDOG/_archive/verification-artifacts}
OFFLINE_DIR=${OFFLINE_DIR:-$ARCHIVE_ROOT/MATDOG_LF_LOWER_M11_MAX_OFFLINE_V29_20260731T182641Z}
OFFLINE_MARKER=${OFFLINE_MARKER:-$OFFLINE_DIR/OFFLINE_LF_LOWER_M11_MAX_V29_PASS.env}
EXECUTOR_SUMMARY=${EXECUTOR_SUMMARY:-$ARCHIVE_ROOT/MATDOG_LF_LOWER_M11_MAX_OFFLINE_EXECUTOR_V31_20260731T182640Z/SUMMARY.env}
RUNNER_PREP=${RUNNER_PREP:-$ARCHIVE_ROOT/MATDOG_LF_LOWER_M11_MAX_RUNNER_PREP_V29_20260731T183321Z}
RUN_CONTRACT=${RUN_CONTRACT:-$RUNNER_PREP/RUN_CONTRACT.env}
DISABLED_COMMAND=${DISABLED_COMMAND:-$RUNNER_PREP/STATION_LAUNCH_COMMAND.disabled}
SERIAL_DEVICE=${MATDOG_SERIAL_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114953-if00}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
OUT="$ARCHIVE_ROOT/MATDOG_${PROFILE}_HARDWARE_PREFLIGHT_V32_${STAMP}"
MARKER="$OUT/HARDWARE_READINESS_${PROFILE}_V32_PASS.env"
LOG="$OUT/preflight.log"

fail() { printf 'HARD BLOCK: %s\n' "$*" >&2; exit 1; }
section() { printf '\n================================================================\n%s\n================================================================\n' "$1"; }
need() { command -v "$1" >/dev/null 2>&1 || fail "required command missing: $1"; }
require_line() { grep -qx "$2" "$1" || fail "missing exact line in $1: $2"; }
port_busy() {
  ss -ltnH 2>/dev/null | awk '{print $4}' | grep -Eq "(^|:)$1$"
}

[[ ${1:-} == --apply ]] || {
  printf '%s\n' \
    "Usage:" \
    "  bash MATDOG_LF_LOWER_M11_MAX_HARDWARE_PREFLIGHT_V32.sh --apply" \
    "" \
    "Read-only readiness check. It does not start Station or open the serial device."
  exit 64
}

mkdir -p "$OUT"
exec > >(tee -a "$LOG") 2>&1

section "PREFLIGHT CONTRACT — READ ONLY"
for cmd in git sha256sum grep awk find sort tail cut pgrep ss id readlink stat tee; do
  need "$cmd"
done

# Validate the isolated source and build produced by the completed offline gate.
git -C "$NORMACORE_WORKTREE" rev-parse --is-inside-work-tree >/dev/null 2>&1 ||
  fail "NormaCore worktree missing: $NORMACORE_WORKTREE"
[[ "$(git -C "$NORMACORE_WORKTREE" rev-parse HEAD)" == "$NORMACORE_HEAD" ]] ||
  fail "unexpected NormaCore worktree head"
[[ -z "$(git -C "$NORMACORE_WORKTREE" status --porcelain)" ]] ||
  fail "NormaCore worktree is not clean"

[[ -f "$OFFLINE_MARKER" ]] || fail "offline marker missing"
[[ -f "$OFFLINE_MARKER.sha256" ]] || fail "offline marker checksum missing"
(
  cd "$(dirname "$OFFLINE_MARKER")"
  sha256sum -c "$(basename "$OFFLINE_MARKER").sha256"
) || fail "offline marker checksum failed"

[[ -f "$EXECUTOR_SUMMARY" ]] || fail "executor summary missing"
require_line "$EXECUTOR_SUMMARY" 'result=PASS'
require_line "$EXECUTOR_SUMMARY" "profile=$PROFILE"
require_line "$EXECUTOR_SUMMARY" "normacore_head=$NORMACORE_HEAD"
require_line "$EXECUTOR_SUMMARY" 'offline_tests=PASS'
require_line "$EXECUTOR_SUMMARY" 'viewer_build=PASS'
require_line "$EXECUTOR_SUMMARY" 'station_release_build=PASS'
require_line "$EXECUTOR_SUMMARY" 'runner_state=PREPARED_NOT_EXECUTED'
require_line "$EXECUTOR_SUMMARY" 'hardware_started=false'
require_line "$EXECUTOR_SUMMARY" 'serial_opened=false'

[[ -f "$RUN_CONTRACT" ]] || fail "runner contract missing"
[[ -f "$DISABLED_COMMAND" ]] || fail "disabled launch command missing"
[[ ! -x "$DISABLED_COMMAND" ]] || fail "disabled command is executable"
(
  cd "$RUNNER_PREP"
  sha256sum -c SHA256SUMS
) || fail "runner preparation checksums failed"

# shellcheck disable=SC1090
source "$OFFLINE_MARKER"
[[ ${result:-} == PASS ]] || fail "offline marker result mismatch"
[[ ${profile:-} == "$PROFILE" ]] || fail "offline marker profile mismatch"
[[ ${normacore_head:-} == "$NORMACORE_HEAD" ]] || fail "offline marker head mismatch"
[[ ${probe_motor_id:-} == 11 ]] || fail "probe motor mismatch"
[[ ${probe_sign:-} == -1 ]] || fail "probe direction mismatch"
[[ ${home_tick:-} == 2048 ]] || fail "home tick mismatch"
[[ ${baseline_target_tick:-} == 1984 ]] || fail "baseline mismatch"
[[ ${urdf_limit_tick:-} == 1621 ]] || fail "URDF limit mismatch"
[[ ${guard_tick:-} == 1557 ]] || fail "guard mismatch"
[[ ${contact_corridor_low:-} == 1557 ]] || fail "contact corridor low mismatch"
[[ ${contact_corridor_high:-} == 1685 ]] || fail "contact corridor high mismatch"
[[ ${prerequisite_m42_tick:-} == 2389 ]] || fail "M42 prerequisite mismatch"
[[ ${prerequisite_m13_tick:-} == 2048 ]] || fail "M13 prerequisite mismatch"
[[ ${prerequisite_m12_tick:-} == 3072 ]] || fail "M12 prerequisite mismatch"
[[ ${goal_position_unsigned:-} == true ]] || fail "unsigned GoalPosition contract missing"
[[ ${ram_only:-} == true ]] || fail "RAM-only contract missing"
[[ ${hip_profiles_blocked:-} == true ]] || fail "HIP block contract missing"
[[ ${hardware_started:-} == false ]] || fail "offline provenance says hardware started"
[[ ${serial_opened:-} == false ]] || fail "offline provenance says serial opened"

STATION_BIN=${station_binary:-}
STATION_SHA=${station_sha256:-}
[[ -n "$STATION_BIN" && -x "$STATION_BIN" ]] || fail "Station binary missing"
[[ -n "$STATION_SHA" ]] || fail "Station SHA missing"
[[ "$(sha256sum "$STATION_BIN" | awk '{print $1}')" == "$STATION_SHA" ]] ||
  fail "Station binary SHA mismatch"

# shellcheck disable=SC1090
source "$RUN_CONTRACT"
[[ ${result:-} == PREPARED_NOT_EXECUTED ]] || fail "runner contract state mismatch"
[[ ${profile:-} == "$PROFILE" ]] || fail "runner profile mismatch"
[[ ${normacore_head:-} == "$NORMACORE_HEAD" ]] || fail "runner head mismatch"
[[ ${station_binary:-} == "$STATION_BIN" ]] || fail "runner Station path mismatch"
[[ ${station_sha256:-} == "$STATION_SHA" ]] || fail "runner Station SHA mismatch"
STATION_CONFIG=${station_config:-}
[[ -n "$STATION_CONFIG" && -f "$STATION_CONFIG" ]] || fail "Station config missing"
grep -Fq 'st3215:' "$STATION_CONFIG" || fail "Station config has no st3215 section"

section "PROCESS, PORT AND SERIAL OWNERSHIP"
! pgrep -af '(^|/)station([[:space:]]|$)' >/dev/null || fail "Station is already running"
! port_busy 8888 || fail "TCP port 8888 is already listening"
! port_busy 8889 || fail "TCP port 8889 is already listening"

[[ -e "$SERIAL_DEVICE" ]] || fail "MATDOG serial by-id path is missing: $SERIAL_DEVICE"
SERIAL_REAL=$(readlink -f "$SERIAL_DEVICE")
[[ -n "$SERIAL_REAL" && -c "$SERIAL_REAL" ]] || fail "serial target is not a character device"
[[ -r "$SERIAL_DEVICE" && -w "$SERIAL_DEVICE" ]] || fail "current user lacks serial read/write access"
if command -v fuser >/dev/null 2>&1; then
  ! fuser "$SERIAL_DEVICE" >/dev/null 2>&1 || fail "serial device is busy"
  ! fuser "$SERIAL_REAL" >/dev/null 2>&1 || fail "resolved serial device is busy"
fi

section "WRITE READINESS MARKER"
cat > "$MARKER" <<EOF
result=PASS
profile=$PROFILE
bus_id=$BUS_ID
normacore_head=$NORMACORE_HEAD
normacore_worktree=$NORMACORE_WORKTREE
offline_marker=$OFFLINE_MARKER
executor_summary=$EXECUTOR_SUMMARY
runner_contract=$RUN_CONTRACT
station_binary=$STATION_BIN
station_sha256=$STATION_SHA
station_config=$STATION_CONFIG
serial_by_id=$SERIAL_DEVICE
serial_resolved=$SERIAL_REAL
serial_present=true
serial_free=true
ports_8888_8889_free=true
station_running=false
probe_motor_id=11
probe_sign=-1
contact_corridor=1557..1685
goal_position_unsigned=true
ram_only=true
hip_profiles_blocked=true
physical_setup_confirmation_required=true
operator_presence_required=true
master_disconnect_access_required=true
hardware_started=false
serial_opened=false
created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
sha256sum "$MARKER" > "$MARKER.sha256"
sha256sum "$MARKER" "$LOG" > "$OUT/SHA256SUMS"

cat "$MARKER"
printf '\nHARDWARE_READY_NOT_STARTED: %s\n' "$OUT"
printf 'No Station process was started and the serial device was not opened.\n'
