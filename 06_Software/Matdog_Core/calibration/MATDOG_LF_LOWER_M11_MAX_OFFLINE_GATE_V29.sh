#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE=LF_LOWER_M11_MAX
EXPECTED_HEAD=fff8e8989ca945bb56982ab5f626e3b45ba8b2dd
EXPECTED_BASE=32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
SOURCE_CI_RUN=30647874565
LEGACY_GATE_RUN=30647874515
VIEWER_CI_RUN=30647874525
NORMACORE_REPO=${NORMACORE_REPO:-$HOME/norma-core}
VIEWER="$NORMACORE_REPO/software/station/clients/station-viewer"
SOURCE="$NORMACORE_REPO/software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS="$NORMACORE_REPO/software/drivers/st3215/src/auto_calibrate/matdog_test.rs"
ARCHIVE_ROOT=${MATDOG_VERIFICATION_ARCHIVE:-$HOME/MATDOG/_archive/verification-artifacts}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
OUT="$ARCHIVE_ROOT/MATDOG_${PROFILE}_OFFLINE_V29_${STAMP}"
MARKER="$OUT/OFFLINE_${PROFILE}_V29_PASS.env"

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
require_token() { grep -Fq "$2" "$1" || fail "missing token in $1: $2"; }

mkdir -p "$OUT"
exec > >(tee "$OUT/offline-gate.log") 2>&1

printf '===== %s offline gate V29 =====\n' "$PROFILE"
git -C "$NORMACORE_REPO" rev-parse --is-inside-work-tree >/dev/null 2>&1 || fail "NormaCore repository/worktree not found: $NORMACORE_REPO"
[[ "$(git -C "$NORMACORE_REPO" rev-parse --is-inside-work-tree)" == true ]] || fail "NormaCore path is not a Git worktree"
[[ -f "$SOURCE" && -f "$TESTS" ]] || fail "aligned MATDOG source files are missing"
[[ "$(git -C "$NORMACORE_REPO" rev-parse HEAD)" == "$EXPECTED_HEAD" ]] || fail "unexpected NormaCore HEAD"
git -C "$NORMACORE_REPO" merge-base --is-ancestor "$EXPECTED_BASE" HEAD || fail "main base is not an ancestor"
[[ -z "$(git -C "$NORMACORE_REPO" status --porcelain)" ]] || fail "NormaCore working tree is not clean"
! pgrep -af '(^|/)station([[:space:]]|$)' >/dev/null || fail "Station is already running"

require_token "$SOURCE" 'const LOWER_MAX_DELTA: i16 = 427;'
require_token "$SOURCE" 'const UPPER_90_DELTA: i16 = 1024;'
require_token "$SOURCE" 'for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip]'
require_token "$TESTS" 'let maximum = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();'
require_token "$TESTS" 'assert_eq!(maximum.probe_sign, -1);'
require_token "$TESTS" 'assert_eq!(maximum.urdf_limit_tick, 1621);'
require_token "$TESTS" 'assert_eq!(maximum.guard_tick, 1557);'
require_token "$TESTS" 'assert_eq!(maximum.baseline_target_tick, 1984);'
require_token "$TESTS" 'target_tick: 2389'
require_token "$TESTS" 'target_tick: 3072'
require_token "$TESTS" 'canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path'

(
  cd "$NORMACORE_REPO"
  cargo test --package st3215 lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers -- --nocapture
  cargo test --package st3215 isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed -- --nocapture
  cargo test --package st3215
)

(
  cd "$VIEWER"
  npm ci
  npm run build
)

grep -R -Fq 'MATDOG native mode: Auto Calibrate only' "$VIEWER/dist/assets" || fail "MATDOG UI guard missing from built viewer"
(
  cd "$NORMACORE_REPO"
  cargo build --release --package station
)

STATION_BIN="$NORMACORE_REPO/target/release/station"
[[ -x "$STATION_BIN" ]] || fail "Station release binary missing"
STATION_SHA=$(sha256sum "$STATION_BIN" | awk '{print $1}')

cat > "$MARKER" <<MARKER_EOF
result=PASS
profile=$PROFILE
normacore_head=$EXPECTED_HEAD
normacore_base=$EXPECTED_BASE
source_ci_run=$SOURCE_CI_RUN
legacy_gate_run=$LEGACY_GATE_RUN
viewer_ci_run=$VIEWER_CI_RUN
station_binary=$STATION_BIN
station_sha256=$STATION_SHA
probe_motor_id=11
probe_sign=-1
home_tick=2048
baseline_target_tick=1984
urdf_limit_tick=1621
guard_tick=1557
contact_corridor_low=1557
contact_corridor_high=1685
prerequisite_m42_tick=2389
prerequisite_m13_tick=2048
prerequisite_m12_tick=3072
goal_position_unsigned=true
ram_only=true
hip_profiles_blocked=true
hardware_started=false
serial_opened=false
created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
MARKER_EOF
sha256sum "$MARKER" > "$MARKER.sha256"
printf 'PASS marker: %s\n' "$MARKER"
