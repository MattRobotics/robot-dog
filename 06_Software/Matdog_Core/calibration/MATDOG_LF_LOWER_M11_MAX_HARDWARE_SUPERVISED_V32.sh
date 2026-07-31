#!/usr/bin/env bash
set -Eeuo pipefail

EXPECTED_PROFILE=LF_LOWER_M11_MAX
BUS_ID=5B14114953
NORMACORE_HEAD=fff8e8989ca945bb56982ab5f626e3b45ba8b2dd
ARCHIVE_ROOT=${MATDOG_VERIFICATION_ARCHIVE:-$HOME/MATDOG/_archive/verification-artifacts}
SERIAL_DEVICE=${MATDOG_SERIAL_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114953-if00}
MAX_PREFLIGHT_AGE_SECONDS=${MATDOG_MAX_PREFLIGHT_AGE_SECONDS:-1800}
MAX_SESSION_SECONDS=${MATDOG_MAX_SESSION_SECONDS:-1800}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
OUT="$ARCHIVE_ROOT/MATDOG_${EXPECTED_PROFILE}_HARDWARE_SUPERVISED_V32_${STAMP}"
STATION_LOG="$OUT/station_${EXPECTED_PROFILE}_${STAMP}.log"
SUMMARY="$OUT/SUMMARY.env"
HTTP_DIR="$OUT/http-verification"
STATION_PID=''
STATION_WAS_STARTED=false
SESSION_RESULT=NOT_STARTED
CALIBRATION_STARTED=false
FINALIZED=false

fail() { printf 'HARD BLOCK: %s\n' "$*" >&2; exit 1; }
section() { printf '\n================================================================\n%s\n================================================================\n' "$1"; }
need() { command -v "$1" >/dev/null 2>&1 || fail "required command missing: $1"; }
port_busy() { ss -ltnH 2>/dev/null | awk '{print $4}' | grep -Eq "(^|:)$1$"; }

stop_station() {
  [[ -n "$STATION_PID" ]] || return 0
  if kill -0 "$STATION_PID" >/dev/null 2>&1; then
    kill -INT "$STATION_PID" >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      kill -0 "$STATION_PID" >/dev/null 2>&1 || break
      sleep 1
    done
  fi
  if kill -0 "$STATION_PID" >/dev/null 2>&1; then
    kill -TERM "$STATION_PID" >/dev/null 2>&1 || true
    for _ in $(seq 1 10); do
      kill -0 "$STATION_PID" >/dev/null 2>&1 || break
      sleep 1
    done
  fi
  wait "$STATION_PID" 2>/dev/null || true
}

serial_is_free() {
  local resolved
  resolved=$(readlink -f "$SERIAL_DEVICE" 2>/dev/null || true)
  [[ -n "$resolved" ]] || return 1
  if command -v fuser >/dev/null 2>&1; then
    ! fuser "$SERIAL_DEVICE" >/dev/null 2>&1 || return 1
    ! fuser "$resolved" >/dev/null 2>&1 || return 1
  fi
  return 0
}

write_nonpass_summary() {
  mkdir -p "$OUT"
  [[ -f "$STATION_LOG" ]] || : > "$STATION_LOG"
  cat > "$SUMMARY" <<EOF
result=$SESSION_RESULT
profile=$EXPECTED_PROFILE
bus_id=$BUS_ID
calibration_started=$CALIBRATION_STARTED
hardware_phase_entered=$STATION_WAS_STARTED
station_started=$STATION_WAS_STARTED
serial_free_after_station_stop=$(serial_is_free && echo true || echo false)
completed_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
  sha256sum "$SUMMARY" "$STATION_LOG" 2>/dev/null > "$OUT/SHA256SUMS" || true
}

cleanup_on_exit() {
  local rc=$?
  if [[ -n "$STATION_PID" ]]; then
    stop_station
    STATION_PID=''
  fi
  if (( rc != 0 )) && [[ "$FINALIZED" != true ]]; then
    [[ "$SESSION_RESULT" != NOT_STARTED ]] || SESSION_RESULT=FAIL_PRECHECK_OR_LAUNCH
    write_nonpass_summary
    printf '\nControlled cleanup completed. Inspect %s\n' "$OUT" >&2
  fi
}

on_interrupt() {
  SESSION_RESULT=ABORTED_BY_OPERATOR
  exit 130
}
trap cleanup_on_exit EXIT
trap on_interrupt INT TERM HUP

[[ ${1:-} == --launch-supervised ]] || {
  printf '%s\n' \
    "Usage:" \
    "  bash MATDOG_LF_LOWER_M11_MAX_HARDWARE_SUPERVISED_V32.sh --launch-supervised" \
    "" \
    "The script launches Station only after operator confirmations." \
    "It never invokes Auto Calibrate automatically."
  exit 64
}

mkdir -p "$OUT" "$HTTP_DIR"
: > "$STATION_LOG"
exec > >(tee -a "$OUT/runner.log") 2>&1

section "LOCATE FRESH READ-ONLY PREFLIGHT"
for cmd in git sha256sum grep awk find sort tail cut pgrep ss curl readlink stat sed seq tee; do
  need "$cmd"
done

READINESS_MARKER=$(
  find "$ARCHIVE_ROOT" -maxdepth 2 -type f \
    -name "HARDWARE_READINESS_${EXPECTED_PROFILE}_V32_PASS.env" \
    -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-
)
[[ -n "$READINESS_MARKER" && -f "$READINESS_MARKER" ]] || fail "hardware readiness marker not found"
[[ -f "$READINESS_MARKER.sha256" ]] || fail "readiness marker checksum missing"
(
  cd "$(dirname "$READINESS_MARKER")"
  sha256sum -c "$(basename "$READINESS_MARKER").sha256"
) || fail "readiness marker checksum failed"

marker_mtime=$(stat -c %Y "$READINESS_MARKER")
marker_age=$(( $(date +%s) - marker_mtime ))
(( marker_age >= 0 && marker_age <= MAX_PREFLIGHT_AGE_SECONDS )) ||
  fail "readiness marker is too old (${marker_age}s); rerun preflight"

# shellcheck disable=SC1090
source "$READINESS_MARKER"
[[ ${result:-} == PASS ]] || fail "readiness marker is not PASS"
[[ ${profile:-} == "$EXPECTED_PROFILE" ]] || fail "profile mismatch"
[[ ${bus_id:-} == "$BUS_ID" ]] || fail "bus mismatch"
[[ ${normacore_head:-} == "$NORMACORE_HEAD" ]] || fail "NormaCore head mismatch"
[[ ${serial_present:-} == true && ${serial_free:-} == true ]] || fail "serial readiness mismatch"
[[ ${ports_8888_8889_free:-} == true ]] || fail "port readiness mismatch"
[[ ${station_running:-} == false ]] || fail "preflight reports Station running"
[[ ${goal_position_unsigned:-} == true ]] || fail "unsigned GoalPosition contract missing"
[[ ${ram_only:-} == true ]] || fail "RAM-only contract missing"
[[ ${hip_profiles_blocked:-} == true ]] || fail "HIP block contract missing"
[[ ${hardware_started:-} == false && ${serial_opened:-} == false ]] || fail "readiness provenance invalid"

NORMACORE_WORKTREE=${normacore_worktree:-}
STATION_BIN=${station_binary:-}
STATION_SHA=${station_sha256:-}
STATION_CONFIG=${station_config:-}
[[ -n "$NORMACORE_WORKTREE" ]] || fail "NormaCore worktree missing from readiness marker"
[[ -n "$STATION_BIN" && -x "$STATION_BIN" ]] || fail "Station binary missing"
[[ -n "$STATION_CONFIG" && -f "$STATION_CONFIG" ]] || fail "Station configuration missing"
[[ "$(sha256sum "$STATION_BIN" | awk '{print $1}')" == "$STATION_SHA" ]] || fail "Station SHA mismatch"
[[ "$(git -C "$NORMACORE_WORKTREE" rev-parse HEAD)" == "$NORMACORE_HEAD" ]] || fail "worktree head mismatch"
[[ -z "$(git -C "$NORMACORE_WORKTREE" status --porcelain)" ]] || fail "worktree is not clean"

section "LIVE OWNERSHIP RECHECK"
! pgrep -af '(^|/)station([[:space:]]|$)' >/dev/null || fail "Station is already running"
! port_busy 8888 || fail "TCP port 8888 is already listening"
! port_busy 8889 || fail "TCP port 8889 is already listening"
[[ -e "$SERIAL_DEVICE" ]] || fail "serial by-id path missing"
[[ -r "$SERIAL_DEVICE" && -w "$SERIAL_DEVICE" ]] || fail "serial read/write access missing"
serial_is_free || fail "serial device is busy"

section "PHYSICAL OPERATOR CONFIRMATIONS"
printf '%s\n' \
  "Profile: $EXPECTED_PROFILE" \
  "Expected M42 -> 2389" \
  "Expected M13 -> 2048" \
  "Expected M12 -> 3072 (upper horizontal)" \
  "M11 is the only probing joint; direction is decreasing ticks." \
  "Expected contact corridor: 1557..1685." \
  "The robot must be supported with base_link 180 mm above the table," \
  "all four legs free, operator present and master disconnect accessible."

read -r -p "Type ROBOT_SUPPORTED: " ACK1
[[ "$ACK1" == ROBOT_SUPPORTED ]] || fail "robot support not confirmed"
read -r -p "Type ALL_LEGS_FREE: " ACK2
[[ "$ACK2" == ALL_LEGS_FREE ]] || fail "free-leg condition not confirmed"
read -r -p "Type OPERATOR_PRESENT: " ACK3
[[ "$ACK3" == OPERATOR_PRESENT ]] || fail "operator presence not confirmed"
read -r -p "Type MASTER_DISCONNECT_READY: " ACK4
[[ "$ACK4" == MASTER_DISCONNECT_READY ]] || fail "master disconnect readiness not confirmed"
read -r -p "Type $EXPECTED_PROFILE to arm this single profile: " ACK5
[[ "$ACK5" == "$EXPECTED_PROFILE" ]] || fail "profile arming confirmation mismatch"

section "START STATION — NO AUTOMATIC CALIBRATION"
SESSION_RESULT=LAUNCHING_STATION
RUN_DATA="$HOME/MATDOG/runtime/station/station_data_${EXPECTED_PROFILE}_SUPERVISED_V32_${STAMP}"
mkdir -p "$RUN_DATA"
env MATDOG_NATIVE_CALIBRATOR_ARM="$EXPECTED_PROFILE" RUST_LOG=info \
  "$STATION_BIN" \
  --config "$STATION_CONFIG" \
  --normfs-base-folder "$RUN_DATA" \
  --tcp 127.0.0.1:8888 \
  --web 127.0.0.1:8889 \
  > >(tee -a "$STATION_LOG") 2>&1 &
STATION_PID=$!
STATION_WAS_STARTED=true
printf 'station_pid=%s\n' "$STATION_PID"

section "HTTP AND NATIVE UI HEALTH CHECK"
HTTP_INDEX="$HTTP_DIR/index.html"
for _ in $(seq 1 60); do
  if curl -fsS http://127.0.0.1:8889/ -o "$HTTP_INDEX"; then
    break
  fi
  kill -0 "$STATION_PID" >/dev/null 2>&1 || fail "Station exited before HTTP became ready"
  sleep 1
done
[[ -s "$HTTP_INDEX" ]] || fail "Station HTTP index not available"
if ! grep -Fq 'id="root"' "$HTTP_INDEX" && ! grep -Fq "id='root'" "$HTTP_INDEX"; then
  fail "served page is not the React Station viewer"
fi

VIEWER_DIST="$NORMACORE_WORKTREE/software/station/clients/station-viewer/dist"
[[ -d "$VIEWER_DIST/assets" ]] || fail "validated viewer dist is missing"
LOCAL_MATDOG_UI=$(grep -R -l -F --include='*.js' \
  'MATDOG native mode: Auto Calibrate only' "$VIEWER_DIST/assets" | head -1)
[[ -n "$LOCAL_MATDOG_UI" && -f "$LOCAL_MATDOG_UI" ]] || fail "local MATDOG UI asset not found"
UI_REL="assets/$(basename "$LOCAL_MATDOG_UI")"
SERVED_MATDOG_UI="$HTTP_DIR/$(basename "$LOCAL_MATDOG_UI")"
curl -fsS "http://127.0.0.1:8889/$UI_REL" -o "$SERVED_MATDOG_UI" || fail "served MATDOG UI asset not available"
LOCAL_UI_SHA=$(sha256sum "$LOCAL_MATDOG_UI" | awk '{print $1}')
SERVED_UI_SHA=$(sha256sum "$SERVED_MATDOG_UI" | awk '{print $1}')
[[ "$LOCAL_UI_SHA" == "$SERVED_UI_SHA" ]] || fail "served MATDOG UI differs from validated build"
printf 'http_matdog_ui_asset=%s\n' "$UI_REL"
printf 'http_matdog_ui_hash_match=true\n'

if command -v xdg-open >/dev/null 2>&1 && [[ -n ${DISPLAY:-} ]]; then
  xdg-open http://127.0.0.1:8889/ >/dev/null 2>&1 || true
fi

section "OPERATOR ACTION IN STATION"
printf '%s\n' \
  "1. Open http://127.0.0.1:8889/ if Chromium did not open automatically." \
  "2. Select bus $BUS_ID." \
  "3. Verify banner: MATDOG native mode: Auto Calibrate only." \
  "4. Verify Reset and Save are disabled." \
  "5. Press Auto Calibrate only when you are physically ready." \
  "6. Use Stop Calibration if necessary; use the master disconnect for an emergency." \
  "" \
  "This script does not click or invoke Auto Calibrate. It only monitors the Station log."

SESSION_RESULT=WAITING_FOR_OPERATOR
start_epoch=$(date +%s)
while kill -0 "$STATION_PID" >/dev/null 2>&1; do
  if grep -Fq "Starting auto-calibration sequence for bus $BUS_ID" "$STATION_LOG"; then
    CALIBRATION_STARTED=true
    SESSION_RESULT=CALIBRATING
  fi
  if grep -Fq "MATDOG $EXPECTED_PROFILE complete:" "$STATION_LOG" &&
     grep -Fq "status=Done, step=14/14, phase='$EXPECTED_PROFILE: completed'" "$STATION_LOG"; then
    SESSION_RESULT=PASS_CANDIDATE
    break
  fi
  if grep -Fq "status=Failed" "$STATION_LOG" && grep -Fq "$EXPECTED_PROFILE: failed" "$STATION_LOG"; then
    SESSION_RESULT=FAIL_PROFILE
    break
  fi
  now=$(date +%s)
  if (( now - start_epoch > MAX_SESSION_SECONDS )); then
    SESSION_RESULT=FAIL_TIMEOUT
    break
  fi
  sleep 1
done

section "CONTROLLED STATION STOP"
stop_station
STATION_PID=''
serial_is_free || fail "serial is still busy after Station stop"

section "VERIFY HARDWARE RESULT"
[[ "$SESSION_RESULT" == PASS_CANDIDATE ]] || {
  write_nonpass_summary
  FINALIZED=true
  fail "hardware session did not reach PASS: $SESSION_RESULT"
}
[[ "$CALIBRATION_STARTED" == true ]] || fail "calibration start was not observed"

grep -Fq "$EXPECTED_PROFILE: Final verified global torque OFF" "$STATION_LOG" ||
  fail "final verified global torque OFF was not observed"
grep -Fq "status=Done, step=14/14, phase='$EXPECTED_PROFILE: completed'" "$STATION_LOG" ||
  fail "Done 14/14 was not observed"

COMPLETE_LINE=$(grep -F "MATDOG $EXPECTED_PROFILE complete:" "$STATION_LOG" | tail -1)
PARSED=$(printf '%s\n' "$COMPLETE_LINE" | sed -n \
  's/.*complete: first=\([0-9][0-9]*\), second=\([0-9][0-9]*\), spread=\([0-9][0-9]*\), baseline_median=\(-\{0,1\}[0-9][0-9]*\), baseline_mad=\([0-9][0-9]*\).*/\1 \2 \3 \4 \5/p')
[[ -n "$PARSED" ]] || fail "could not parse contact summary"
read -r CONTACT_FIRST CONTACT_SECOND SPREAD BASELINE_MEDIAN BASELINE_MAD <<< "$PARSED"
(( CONTACT_FIRST >= 1557 && CONTACT_FIRST <= 1685 )) || fail "first contact outside corridor: $CONTACT_FIRST"
(( CONTACT_SECOND >= 1557 && CONTACT_SECOND <= 1685 )) || fail "second contact outside corridor: $CONTACT_SECOND"
ABS_DIFF=$(( CONTACT_FIRST - CONTACT_SECOND ))
(( ABS_DIFF < 0 )) && ABS_DIFF=$(( -ABS_DIFF ))
(( SPREAD == ABS_DIFF )) || fail "reported spread does not match contact difference"

SESSION_RESULT=PASS
cat > "$SUMMARY" <<EOF
result=PASS
profile=$EXPECTED_PROFILE
bus_id=$BUS_ID
runner=V32
normacore_head=$NORMACORE_HEAD
station_binary=$STATION_BIN
station_sha256=$STATION_SHA
readiness_marker=$READINESS_MARKER
http_matdog_ui_asset=$UI_REL
http_matdog_ui_local_sha256=$LOCAL_UI_SHA
http_matdog_ui_served_sha256=$SERVED_UI_SHA
http_matdog_ui_hash_match=true
calibration_started=true
contact_count=2
contact_first=$CONTACT_FIRST
contact_second=$CONTACT_SECOND
spread=$SPREAD
baseline_median=$BASELINE_MEDIAN
baseline_mad=$BASELINE_MAD
contact_corridor=1557..1685
done_14_of_14=true
final_verified_global_torque_off=true
serial_free_after_station_stop=true
goal_position_unsigned=true
ram_only=true
hip_profiles_blocked=true
completed_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
sha256sum "$SUMMARY" "$STATION_LOG" "$OUT/runner.log" > "$OUT/SHA256SUMS"
FINALIZED=true
cat "$SUMMARY"
printf '\nHARDWARE PASS artifact: %s\n' "$OUT"
