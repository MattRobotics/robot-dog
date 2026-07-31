#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE=LF_LOWER_M11_MAX
EXPECTED_HEAD=fff8e8989ca945bb56982ab5f626e3b45ba8b2dd
NORMACORE_REPO=${NORMACORE_REPO:-$HOME/norma-core}
STATION_BIN=${STATION_BIN:-$NORMACORE_REPO/target/release/station}
STATION_CONFIG=${STATION_CONFIG:-$HOME/MATDOG/runtime/station/station.yaml}
ARCHIVE_ROOT=${MATDOG_VERIFICATION_ARCHIVE:-$HOME/MATDOG/_archive/verification-artifacts}
SERIAL_DEVICE=${MATDOG_SERIAL_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114953-if00}

fail() { printf 'BLOCKED: %s\n' "$*" >&2; exit 1; }

[[ ${1:-} == --prepare ]] || fail "this checkpoint supports --prepare only"
[[ "$(git -C "$NORMACORE_REPO" rev-parse HEAD)" == "$EXPECTED_HEAD" ]] || fail "unexpected NormaCore HEAD"
[[ -z "$(git -C "$NORMACORE_REPO" status --porcelain)" ]] || fail "NormaCore working tree is not clean"
[[ -x "$STATION_BIN" ]] || fail "Station binary missing"
[[ -f "$STATION_CONFIG" ]] || fail "Station config missing"
grep -Fq 'st3215:' "$STATION_CONFIG" || fail "Station config does not contain the ST3215 section"
! pgrep -af '(^|/)station([[:space:]]|$)' >/dev/null || fail "Station is already running"
if [[ -e "$SERIAL_DEVICE" ]] && command -v fuser >/dev/null 2>&1; then
  ! fuser "$SERIAL_DEVICE" >/dev/null 2>&1 || fail "serial device is busy"
fi

mapfile -t MARKERS < <(find "$ARCHIVE_ROOT" -type f -name "OFFLINE_${PROFILE}_V29_PASS.env" -print 2>/dev/null | sort)
((${#MARKERS[@]} > 0)) || fail "offline PASS marker not found"
MARKER=${MARKERS[-1]}
[[ -f "$MARKER.sha256" ]] || fail "offline marker checksum is missing"
(
  cd "$(dirname "$MARKER")"
  sha256sum -c "$(basename "$MARKER").sha256"
) || fail "offline marker checksum failed"
# shellcheck disable=SC1090
source "$MARKER"
[[ ${result:-} == PASS ]] || fail "offline marker is not PASS"
[[ ${profile:-} == "$PROFILE" ]] || fail "offline marker profile mismatch"
[[ ${normacore_head:-} == "$EXPECTED_HEAD" ]] || fail "offline marker head mismatch"
[[ ${goal_position_unsigned:-} == true ]] || fail "unsigned GoalPosition contract missing"
[[ ${ram_only:-} == true ]] || fail "RAM-only contract missing"
[[ ${hip_profiles_blocked:-} == true ]] || fail "HIP block contract missing"
[[ ${hardware_started:-} == false && ${serial_opened:-} == false ]] || fail "offline marker provenance invalid"
[[ "$(sha256sum "$STATION_BIN" | awk '{print $1}')" == "${station_sha256:-}" ]] || fail "Station binary SHA mismatch"

STAMP=$(date -u +%Y%m%dT%H%M%SZ)
PREP_DIR="$ARCHIVE_ROOT/MATDOG_${PROFILE}_RUNNER_PREP_V29_${STAMP}"
mkdir -p "$PREP_DIR"
cp "$MARKER" "$MARKER.sha256" "$PREP_DIR/"

cat > "$PREP_DIR/RUN_CONTRACT.env" <<CONTRACT_EOF
result=PREPARED_NOT_EXECUTED
profile=$PROFILE
normacore_head=$EXPECTED_HEAD
station_binary=$STATION_BIN
station_sha256=$station_sha256
station_config=$STATION_CONFIG
probe_motor_id=11
probe_sign=-1
home_tick=2048
baseline_target_tick=1984
urdf_limit_tick=1621
guard_tick=1557
contact_corridor=1557..1685
prerequisite_m42_tick=2389
prerequisite_m13_tick=2048
prerequisite_m12_tick=3072
goal_position_unsigned=true
ram_only=true
hip_profiles_blocked=true
hardware_started=false
serial_opened=false
prepared_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
CONTRACT_EOF

cat > "$PREP_DIR/STATION_LAUNCH_COMMAND.disabled" <<COMMAND_EOF
env MATDOG_NATIVE_CALIBRATOR_ARM=$PROFILE RUST_LOG=info \\
  $STATION_BIN \\
  --config $STATION_CONFIG \\
  --normfs-base-folder $HOME/MATDOG/runtime/station/station_data_${PROFILE}_SUPERVISED \\
  --tcp 127.0.0.1:8888 \\
  --web 127.0.0.1:8889
COMMAND_EOF
chmod 600 "$PREP_DIR/STATION_LAUNCH_COMMAND.disabled"
sha256sum "$PREP_DIR/RUN_CONTRACT.env" "$PREP_DIR/STATION_LAUNCH_COMMAND.disabled" > "$PREP_DIR/SHA256SUMS"

printf 'PREPARED_NOT_EXECUTED: %s\n' "$PREP_DIR"
printf 'Hardware execution is intentionally disabled in this checkpoint.\n'
printf 'A later supervised authorization must review and activate the generated command.\n'
exit 78
