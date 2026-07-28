#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE="LF_UPPER_M12_MAX"
BUS_SERIAL="5B14114953"
ROBOT_REPO="$HOME/MATDOG/github/robot-dog"
NORMA_REPO="$HOME/norma-core"
ROBOT_EXPECTED="4cdf440a2d37d1fe5e33c01f41687e460444a141"
NORMA_EXPECTED="32e3222c87016b7f5d7c1c1da497a4cea3e7b80a"
V11_ARCH="restart-safe-profile-entry-v11"
V13_ARCH="restart-safe-profile-entry-v13-distance-aware"
V11_PATCH_EXPECTED="120586219b6e2d05b727ffc7bbeda67c90ff99d8"
V11_BIN_EXPECTED_SHA256="aa21d2931acedd45db72478bf78d6f6163f6896998dbf1ad5b0c2d1560aab681"
V11_MARKER_EXPECTED_SHA256="c910ec6052e6277c0e57737c446a80fb821893ab13f519deb64368e7e365949e"
CONFIG="$HOME/MATDOG/runtime/station/station_m12_pilot_deadband2.yaml"
CONFIG_EXPECTED_SHA256="f648988540d22fb38aa9b66e19bdf059f05047025b6abd229b64d7eff5f20bd1"
SERIAL_BY_ID="/dev/serial/by-id/usb-1a86_USB_Single_Serial_${BUS_SERIAL}-if00"
ARTIFACT_ROOT="$HOME/MATDOG/_archive/verification-artifacts"

section() {
  printf '\n============================================================\n%s\n============================================================\n' "$1"
}

fail() {
  echo "HARD BLOCK: $*" >&2
  exit 1
}

marker_value_from() {
  local marker="$1" key="$2"
  sed -n "s/^${key}=//p" "$marker" | tail -n 1
}

serial_owners() {
  local dev="$1"
  {
    command -v lsof >/dev/null 2>&1 && lsof "$dev" 2>/dev/null || true
    command -v fuser >/dev/null 2>&1 && fuser -v "$dev" 2>/dev/null || true
  } | sed '/^[[:space:]]*$/d'
}

path_is_within() {
  local child parent
  child="$(realpath -m "$1")"
  parent="$(realpath -m "$2")"
  [[ "$child" == "$parent" || "$child" == "$parent/"* ]]
}

section "OFFLINE V13 — TIMEOUT DI MOVIMENTO PROPORZIONALE ALLA DISTANZA"
echo "Questa fase riusa il worktree V11 già validato e non avvia Station."
echo "Non apre la seriale e non invia alcun comando al robot."
echo "Corregge soltanto il budget temporale dei movimenti lunghi."
echo
read -r -p "Digita esattamente ${PROFILE} per autorizzare la validazione offline V13: " CONFIRM
[[ "$CONFIRM" == "$PROFILE" ]] || fail "conferma operatore non valida"

section "PREFLIGHT — NESSUN HARDWARE"
if pgrep -af '(^|/)(station)([[:space:]]|$)' >/tmp/matdog_v13_station.$$ 2>/dev/null; then
  cat /tmp/matdog_v13_station.$$
  rm -f /tmp/matdog_v13_station.$$
  fail "Station è già in esecuzione"
fi
rm -f /tmp/matdog_v13_station.$$ 2>/dev/null || true
[[ -e "$SERIAL_BY_ID" ]] || fail "seriale persistente assente: $SERIAL_BY_ID"
OWNERS="$(serial_owners "$SERIAL_BY_ID")"
[[ -z "$OWNERS" ]] || { echo "$OWNERS"; fail "seriale già posseduta"; }
if ss -ltn 2>/dev/null | grep -Eq '127\.0\.0\.1:(8888|8889)[[:space:]]'; then
  ss -ltnp 2>/dev/null | grep -E '127\.0\.0\.1:(8888|8889)' || true
  fail "porta 8888 o 8889 occupata"
fi
[[ -d "$ROBOT_REPO/.git" && -d "$NORMA_REPO/.git" ]] || fail "repository canonici assenti"
[[ "$(git -C "$ROBOT_REPO" branch --show-current)" == "main" ]] || fail "robot-dog non è su main"
[[ "$(git -C "$NORMA_REPO" branch --show-current)" == "main" ]] || fail "norma-core non è su main"
[[ -z "$(git -C "$ROBOT_REPO" status --porcelain)" ]] || fail "robot-dog non pulito"
[[ -z "$(git -C "$NORMA_REPO" status --porcelain)" ]] || fail "norma-core non pulito"
[[ "$(git -C "$ROBOT_REPO" rev-parse HEAD)" == "$ROBOT_EXPECTED" ]] || fail "robot-dog HEAD inatteso"
[[ "$(git -C "$NORMA_REPO" rev-parse HEAD)" == "$NORMA_EXPECTED" ]] || fail "norma-core HEAD inatteso"
[[ -f "$CONFIG" ]] || fail "configurazione validata assente"
[[ "$(sha256sum "$CONFIG" | awk '{print $1}')" == "$CONFIG_EXPECTED_SHA256" ]] || fail "configurazione modificata"
echo "PASS: Station assente, seriale libera, repository e configurazione coerenti"

section "INDIVIDUAZIONE ARTEFATTO V11 ESATTO"
mapfile -t CANDIDATES < <(
  find "$ARTIFACT_ROOT" -mindepth 2 -maxdepth 2 -type f -name 'OFFLINE_VALIDATION_PASS.env' \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | cut -d' ' -f2-
)
V11_MARKER=""
for candidate in "${CANDIDATES[@]}"; do
  [[ "$(marker_value_from "$candidate" result)" == "PASS" ]] || continue
  [[ "$(marker_value_from "$candidate" profile)" == "$PROFILE" ]] || continue
  [[ "$(marker_value_from "$candidate" bus)" == "$BUS_SERIAL" ]] || continue
  [[ "$(marker_value_from "$candidate" architecture)" == "$V11_ARCH" ]] || continue
  [[ "$(marker_value_from "$candidate" local_patch_commit)" == "$V11_PATCH_EXPECTED" ]] || continue
  V11_MARKER="$candidate"
  break
done
[[ -n "$V11_MARKER" ]] || fail "marker V11 esatto non trovato"
V11_MARKER_SHA256="$(sha256sum "$V11_MARKER" | awk '{print $1}')"
[[ "$V11_MARKER_SHA256" == "$V11_MARKER_EXPECTED_SHA256" ]] || fail "marker V11 modificato: $V11_MARKER_SHA256"

ARTIFACT="$(marker_value_from "$V11_MARKER" artifact)"
WORKTREE="$(marker_value_from "$V11_MARKER" worktree)"
OLD_BIN="$(marker_value_from "$V11_MARKER" station_binary)"
OLD_BIN_SHA="$(marker_value_from "$V11_MARKER" station_sha256)"
V11_REGRESSION="$(marker_value_from "$V11_MARKER" regression_log)"
V11_FULL_TEST="$(marker_value_from "$V11_MARKER" full_test_log)"
[[ -n "$ARTIFACT" && -n "$WORKTREE" && -n "$OLD_BIN" ]] || fail "marker V11 incompleto"
path_is_within "$WORKTREE" "$ARTIFACT" || fail "worktree esterno all'artefatto V11"
path_is_within "$OLD_BIN" "$ARTIFACT" || fail "binario esterno all'artefatto V11"
[[ "$(git -C "$WORKTREE" rev-parse HEAD)" == "$V11_PATCH_EXPECTED" ]] || fail "worktree V11 non è sul commit atteso"
[[ -z "$(git -C "$WORKTREE" status --porcelain)" ]] || fail "worktree V11 non pulito"
[[ "$(sha256sum "$OLD_BIN" | awk '{print $1}')" == "$V11_BIN_EXPECTED_SHA256" ]] || fail "binario V11 modificato"
[[ "$OLD_BIN_SHA" == "$V11_BIN_EXPECTED_SHA256" ]] || fail "hash binario nel marker V11 inatteso"
[[ -f "$V11_REGRESSION" && -f "$V11_FULL_TEST" ]] || fail "evidenze V11 mancanti"
grep -q "75 passed; 0 failed" "$V11_FULL_TEST" || fail "suite V11 non risulta 75/75 PASS"
echo "marker_v11=$V11_MARKER"
echo "worktree=$WORKTREE"
echo "PASS: artefatto V11 immutabile verificato"

SOURCE="$WORKTREE/software/drivers/st3215/src/auto_calibrate/matdog.rs"
TESTS="$WORKTREE/software/drivers/st3215/src/auto_calibrate/matdog_test.rs"
PORT="$WORKTREE/software/drivers/st3215/src/port.rs"
CARGO_TARGET="$ARTIFACT/cargo-target"
BIN="$CARGO_TARGET/release/station"
BUILD_RS="$WORKTREE/software/station/bin/station/build.rs"
DIST_SOURCE="$NORMA_REPO/software/station/clients/station-viewer/dist"
DIST_DEST="$WORKTREE/software/station/clients/station-viewer/dist"
[[ -f "$SOURCE" && -f "$TESTS" && -f "$PORT" ]] || fail "sorgenti V11 mancanti"
[[ -d "$CARGO_TARGET" ]] || fail "cargo target V11 assente"

section "APPLICAZIONE CORREZIONE V13 MINIMA"
python3 - "$SOURCE" "$TESTS" <<'PY'
from pathlib import Path
import sys

source_path = Path(sys.argv[1])
tests_path = Path(sys.argv[2])
source = source_path.read_text(encoding="utf-8")
tests = tests_path.read_text(encoding="utf-8")

def replace_once(text, old, new, label):
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: anchor count={count}, expected=1")
    return text.replace(old, new, 1)

def replace_function(text, start_sig, end_sig, transform, label):
    start = text.find(start_sig)
    if start < 0:
        raise SystemExit(f"{label}: start signature missing")
    end = text.find(end_sig, start)
    if end < 0:
        raise SystemExit(f"{label}: end signature missing")
    body = text[start:end]
    new_body = transform(body, label)
    return text[:start] + new_body + text[end:]

def patch_move(body, label):
    old = "        let deadline = Instant::now() + MOTION_TIMEOUT;\n"
    new = '''        let start_position = self.latest_observation(motor_id)?.position;
        let distance_ticks = circular_distance(start_position, target);
        let motion_timeout = motion_timeout_for_distance(distance_ticks);
        let deadline = Instant::now() + motion_timeout;
        info!(
            "MATDOG {} move plan: M{} start={} target={} distance={} timeout_ms={}",
            self.profile.label,
            motor_id,
            start_position,
            target,
            distance_ticks,
            motion_timeout.as_millis()
        );
'''
    if body.count(old) != 1:
        raise SystemExit(f"{label}: deadline anchor count={body.count(old)}")
    return body.replace(old, new, 1)

source = replace_once(
    source,
    "const MOTION_TIMEOUT: Duration = Duration::from_secs(12);\n",
    '''const MOTION_TIMEOUT: Duration = Duration::from_secs(12);
// Long MAX returns and +90-degree prerequisites can exceed the original fixed
// 12-second budget at GOAL_SPEED=80. Size the deadline from the commanded
// distance using a conservative half-speed floor, retaining 12 seconds
// as the minimum for short movements and telemetry/settling overhead.
const MIN_EXPECTED_MOTION_TICKS_PER_SECOND: u64 = 40;
const MOTION_SETTLE_MARGIN: Duration = Duration::from_secs(5);
''',
    "timeout constants",
)
source = replace_once(
    source,
    "fn passed_guard(value: u16, guard: u16, sign: i8) -> bool {\n",
    '''fn motion_timeout_for_distance(distance_ticks: u16) -> Duration {
    let travel_ms = u64::from(distance_ticks)
        .saturating_mul(1000)
        .saturating_add(MIN_EXPECTED_MOTION_TICKS_PER_SECOND - 1)
        / MIN_EXPECTED_MOTION_TICKS_PER_SECOND;
    Duration::from_millis(travel_ms)
        .saturating_add(MOTION_SETTLE_MARGIN)
        .max(MOTION_TIMEOUT)
}

fn passed_guard(value: u16, guard: u16, sign: i8) -> bool {
''',
    "distance helper",
)
source = replace_function(
    source,
    "    async fn move_profile_entry_motor_to_target(\n",
    "    async fn verify_profile_entry_holds_except(\n",
    patch_move,
    "profile-entry move",
)
source = replace_function(
    source,
    "    async fn move_motor_to(\n",
    "    async fn verify_profile_holds(&self)",
    patch_move,
    "ordinary move",
)

test_anchor = '''#[test]
fn robust_current_baseline_uses_median_and_mad() {
'''
new_tests = r'''#[test]
fn motion_timeout_covers_observed_m12_max_return() {
    let distance = circular_distance(3327, HOME_TICK);
    assert_eq!(distance, 1279);
    assert!(u64::from(distance) > u64::from(GOAL_SPEED) * MOTION_TIMEOUT.as_secs());
    let timeout = motion_timeout_for_distance(distance);
    assert!(timeout > MOTION_TIMEOUT);
    assert!(timeout >= Duration::from_secs(36));
}

#[test]
fn motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile() {
    assert_eq!(motion_timeout_for_distance(64), MOTION_TIMEOUT);
    for profile in all_profiles().unwrap() {
        let distance = circular_distance(profile.guard_tick, HOME_TICK);
        let ideal_ms = (u64::from(distance) * 1000 + u64::from(GOAL_SPEED) - 1)
            / u64::from(GOAL_SPEED);
        assert!(
            motion_timeout_for_distance(distance)
                >= Duration::from_millis(ideal_ms).saturating_add(MOTION_SETTLE_MARGIN)
        );
    }
}

'''
tests = replace_once(tests, test_anchor, new_tests + test_anchor, "timeout tests")
source_path.write_text(source, encoding="utf-8")
tests_path.write_text(tests, encoding="utf-8")
print("PASS: V13 applicata ai due movimenti e ai test")
PY

rustfmt --edition 2024 "$SOURCE" "$TESTS"
git -C "$WORKTREE" diff --check
CHANGED="$(git -C "$WORKTREE" diff --name-only | sort)"
EXPECTED_CHANGED=$'software/drivers/st3215/src/auto_calibrate/matdog.rs\nsoftware/drivers/st3215/src/auto_calibrate/matdog_test.rs'
[[ "$CHANGED" == "$EXPECTED_CHANGED" ]] || { printf '%s\n' "$CHANGED"; fail "scope V13 inatteso"; }
python3 - "$SOURCE" <<'PY'
from pathlib import Path
import re, sys
s = Path(sys.argv[1]).read_text()
assert s.count("let deadline = Instant::now() + motion_timeout;") == 2
assert "MIN_EXPECTED_MOTION_TICKS_PER_SECOND: u64 = 40" in s
assert "MOTION_SETTLE_MARGIN: Duration = Duration::from_secs(5)" in s
assert "move plan: M{} start={} target={} distance={} timeout_ms={}" in s
for start, end in [
    ("async fn move_profile_entry_motor_to_target(", "async fn verify_profile_entry_holds_except("),
    ("async fn move_motor_to(", "async fn verify_profile_holds(&self)"),
]:
    body = s[s.index(start):s.index(end, s.index(start))]
    assert "motion_timeout_for_distance(distance_ticks)" in body
    assert "Instant::now() + MOTION_TIMEOUT" not in body
print("PASS: audit statico V13")
PY

section "TEST MIRATI TIMEOUT — OFFLINE"
V13_TARGETED="$ARTIFACT/v13_motion_timeout_tests.log"
(
  cd "$WORKTREE"
  CARGO_TARGET_DIR="$CARGO_TARGET" CARGO_NET_OFFLINE=true \
    cargo test --package st3215 motion_timeout_ --offline -- --nocapture
) 2>&1 | tee "$V13_TARGETED"
grep -q "motion_timeout_covers_observed_m12_max_return ... ok" "$V13_TARGETED" || fail "test ritorno M12 MAX non PASS"
grep -q "motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile ... ok" "$V13_TARGETED" || fail "test timeout 24 profili non PASS"
grep -q "2 passed; 0 failed" "$V13_TARGETED" || fail "regressioni V13 non PASS"

section "SUITE ST3215 COMPLETA — OFFLINE"
V13_FULL="$ARTIFACT/v13_st3215_full_tests.log"
(
  cd "$WORKTREE"
  CARGO_TARGET_DIR="$CARGO_TARGET" CARGO_NET_OFFLINE=true \
    cargo test --package st3215 --offline
) 2>&1 | tee "$V13_FULL"
grep -q "77 passed; 0 failed" "$V13_FULL" || fail "suite V13 non risulta 77/77 PASS"
if grep -q '^warning:' "$V13_FULL"; then
  fail "warning Rust nella suite V13"
fi

section "COMMIT LOCALE V13"
git -C "$WORKTREE" -c user.name="MATDOG Local Validation" -c user.email="matdog-local@invalid" \
  add software/drivers/st3215/src/auto_calibrate/matdog.rs \
      software/drivers/st3215/src/auto_calibrate/matdog_test.rs
git -C "$WORKTREE" -c user.name="MATDOG Local Validation" -c user.email="matdog-local@invalid" \
  commit -m "fix(matdog): make movement deadline distance-aware"
V13_COMMIT="$(git -C "$WORKTREE" rev-parse HEAD)"
V13_SHORT="$(git -C "$WORKTREE" rev-parse --short=7 HEAD)"
[[ "$(git -C "$WORKTREE" rev-parse HEAD^)" == "$V11_PATCH_EXPECTED" ]] || fail "parent V13 inatteso"
[[ -z "$(git -C "$WORKTREE" status --porcelain)" ]] || fail "worktree non pulito dopo commit V13"

section "BUILD STATION OFFLINE DAL COMMIT V13 TESTATO"
if [[ ! -f "$DIST_DEST/index.html" ]]; then
  [[ -f "$DIST_SOURCE/index.html" ]] || fail "asset UI assenti"
  mkdir -p "$(dirname "$DIST_DEST")"
  rm -rf "$DIST_DEST"
  cp -a "$DIST_SOURCE" "$DIST_DEST"
fi
[[ -f "$BUILD_RS" ]] || fail "build.rs Station assente"
(
  cd "$WORKTREE"
  touch "$BUILD_RS"
  CARGO_TARGET_DIR="$CARGO_TARGET" CARGO_NET_OFFLINE=true cargo clean --release --package station
  CARGO_TARGET_DIR="$CARGO_TARGET" CARGO_NET_OFFLINE=true cargo build --release --package station --offline
)
[[ -x "$BIN" ]] || fail "binario V13 non prodotto"
VERSION="$($BIN --version 2>&1 || true)"
BIN_SHA256="$(sha256sum "$BIN" | awk '{print $1}')"
[[ "$VERSION" == *"(${V13_SHORT})"* ]] || fail "binario non marcato $V13_SHORT"
STRINGS_FILE="$ARTIFACT/v13_station_strings.txt"
strings "$BIN" > "$STRINGS_FILE"
for token in \
  "Inspect restart-safe profile entry" \
  "move plan: M" \
  "timeout_ms=" \
  "restart-safe profile entry refused before motion"; do
  grep -Fq "$token" "$STRINGS_FILE" || fail "binario V13 privo del token: $token"
done
[[ -z "$(git -C "$WORKTREE" status --porcelain)" ]] || fail "build V13 ha alterato file tracked"

V13_MARKER="$ARTIFACT/OFFLINE_DISTANCE_TIMEOUT_V13_PASS.env"
cat > "$V13_MARKER" <<EOF
result=PASS
profile=$PROFILE
bus=$BUS_SERIAL
robot_commit=$ROBOT_EXPECTED
norma_base_commit=$NORMA_EXPECTED
v11_parent_commit=$V11_PATCH_EXPECTED
local_patch_commit=$V13_COMMIT
station_version=$VERSION
station_sha256=$BIN_SHA256
station_binary=$BIN
worktree=$WORKTREE
artifact=$ARTIFACT
v11_marker=$V11_MARKER
v11_marker_sha256=$V11_MARKER_SHA256
targeted_test_log=$V13_TARGETED
full_test_log=$V13_FULL
validated_at=$(date --iso-8601=seconds)
hardware_started=false
serial_opened=false
architecture=$V13_ARCH
observed_contact_tick=3327
observed_return_distance_ticks=1279
minimum_expected_motion_ticks_per_second=40
motion_settle_margin_seconds=5
EOF
V13_MARKER_SHA="$(sha256sum "$V13_MARKER" | awk '{print $1}')"

section "RISULTATO OFFLINE V13"
echo "result=PASS"
echo "hardware_started=false"
echo "serial_opened=false"
echo "local_patch_commit=$V13_COMMIT"
echo "station_version=$VERSION"
echo "station_sha256=$BIN_SHA256"
echo "validation_marker=$V13_MARKER"
echo "validation_marker_sha256=$V13_MARKER_SHA"
echo
echo "V13 COMPLETATA. Station non è stata avviata e nessun servo è stato comandato."
