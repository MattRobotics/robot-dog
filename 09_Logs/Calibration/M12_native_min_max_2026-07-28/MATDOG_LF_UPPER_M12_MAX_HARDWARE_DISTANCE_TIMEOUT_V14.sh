#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE="LF_UPPER_M12_MAX"
BUS_SERIAL="5B14114953"
ROBOT_REPO="$HOME/MATDOG/github/robot-dog"
NORMA_REPO="$HOME/norma-core"
ROBOT_EXPECTED="4cdf440a2d37d1fe5e33c01f41687e460444a141"
NORMA_EXPECTED="32e3222c87016b7f5d7c1c1da497a4cea3e7b80a"
ARCHITECTURE_EXPECTED="restart-safe-profile-entry-v13-distance-aware"
V11_PARENT_EXPECTED="120586219b6e2d05b727ffc7bbeda67c90ff99d8"
V13_SCRIPT="$HOME/Downloads/MATDOG_LF_UPPER_M12_MAX_OFFLINE_DISTANCE_TIMEOUT_V13.sh"
V13_SCRIPT_EXPECTED_SHA256="de419febc9c03339170cf6adf73459c172b258d15e16a320f53f3bd07b7521e9"
CONFIG="$HOME/MATDOG/runtime/station/station_m12_pilot_deadband2.yaml"
CONFIG_EXPECTED_SHA256="f648988540d22fb38aa9b66e19bdf059f05047025b6abd229b64d7eff5f20bd1"
SERIAL_BY_ID="/dev/serial/by-id/usb-1a86_USB_Single_Serial_${BUS_SERIAL}-if00"
TCP_ADDR="127.0.0.1:8888"
WEB_ADDR="127.0.0.1:8889"
WEB_URL="http://${WEB_ADDR}/"
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

stop_tail() {
  if [[ -n "${TAIL_PID:-}" ]] && kill -0 "$TAIL_PID" 2>/dev/null; then
    kill "$TAIL_PID" 2>/dev/null || true
    wait "$TAIL_PID" 2>/dev/null || true
  fi
}

stop_station_controlled() {
  if [[ -n "${STATION_PID:-}" ]] && kill -0 "$STATION_PID" 2>/dev/null; then
    kill -INT "$STATION_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$STATION_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$STATION_PID" 2>/dev/null; then
      kill -TERM "$STATION_PID" 2>/dev/null || true
    fi
    wait "$STATION_PID" 2>/dev/null || true
  fi
}

STATION_PID=""
TAIL_PID=""
OUTCOME_SEEN=0
CALIBRATION_STARTED=0
RESULT="BLOCKED"

on_exit() {
  local rc=$?
  stop_tail
  if [[ $rc -ne 0 ]] && [[ -n "${STATION_PID:-}" ]] && kill -0 "$STATION_PID" 2>/dev/null; then
    if [[ "$CALIBRATION_STARTED" -eq 0 ]]; then
      stop_station_controlled
    elif [[ "$OUTCOME_SEEN" -eq 0 ]]; then
      echo >&2
      echo "ATTENZIONE: Station è ancora attiva durante una calibrazione senza esito finale." >&2
      echo "Usa Stop Calibration nella UI. In emergenza usa il master disconnect." >&2
    fi
  fi
}
trap on_exit EXIT

section "HARDWARE V14 — SOLO DAL V13 DISTANCE-AWARE VALIDATO"
echo "Questo runner non modifica sorgenti, non compila e non applica patch."
echo "Accetta soltanto un artefatto V13 PASS prodotto dallo script esatto verificato."
echo
read -r -p "Digita esattamente ${PROFILE} per autorizzare la prova hardware: " CONFIRM
[[ "$CONFIRM" == "$PROFILE" ]] || fail "conferma operatore non valida"

section "VERIFICA SCRIPT V13 E INDIVIDUAZIONE ARTEFATTO PASS"
[[ -f "$V13_SCRIPT" ]] || fail "script V13 validato assente: $V13_SCRIPT"
V13_SCRIPT_SHA256="$(sha256sum "$V13_SCRIPT" | awk '{print $1}')"
[[ "$V13_SCRIPT_SHA256" == "$V13_SCRIPT_EXPECTED_SHA256" ]] \
  || fail "script V13 diverso da quello revisionato: sha=$V13_SCRIPT_SHA256"

mapfile -t MARKER_CANDIDATES < <(
  find "$ARTIFACT_ROOT" -mindepth 2 -maxdepth 2 -type f \
    -name 'OFFLINE_DISTANCE_TIMEOUT_V13_PASS.env' \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | cut -d' ' -f2-
)
[[ ${#MARKER_CANDIDATES[@]} -gt 0 ]] || fail "nessun marker V13 PASS trovato; eseguire prima il validatore offline V13"

MARKER=""
for candidate in "${MARKER_CANDIDATES[@]}"; do
  [[ "$(marker_value_from "$candidate" result)" == "PASS" ]] || continue
  [[ "$(marker_value_from "$candidate" profile)" == "$PROFILE" ]] || continue
  [[ "$(marker_value_from "$candidate" bus)" == "$BUS_SERIAL" ]] || continue
  [[ "$(marker_value_from "$candidate" architecture)" == "$ARCHITECTURE_EXPECTED" ]] || continue
  [[ "$(marker_value_from "$candidate" hardware_started)" == "false" ]] || continue
  [[ "$(marker_value_from "$candidate" serial_opened)" == "false" ]] || continue
  MARKER="$candidate"
  break
done
[[ -n "$MARKER" ]] || fail "marker V13 presente ma nessuno soddisfa il contratto distance-aware"

MARKER_SHA256="$(sha256sum "$MARKER" | awk '{print $1}')"
ARTIFACT="$(marker_value_from "$MARKER" artifact)"
WORKTREE="$(marker_value_from "$MARKER" worktree)"
BIN="$(marker_value_from "$MARKER" station_binary)"
BIN_EXPECTED_SHA256="$(marker_value_from "$MARKER" station_sha256)"
PATCH_EXPECTED="$(marker_value_from "$MARKER" local_patch_commit)"
TARGETED_LOG="$(marker_value_from "$MARKER" targeted_test_log)"
FULL_TEST_LOG="$(marker_value_from "$MARKER" full_test_log)"
V11_PARENT="$(marker_value_from "$MARKER" v11_parent_commit)"
PATCH_SHORT="${PATCH_EXPECTED:0:7}"

[[ -n "$ARTIFACT" && -n "$WORKTREE" && -n "$BIN" && -n "$PATCH_EXPECTED" ]] \
  || fail "marker V13 incompleto"
[[ "$(realpath -m "$(dirname "$MARKER")")" == "$(realpath -m "$ARTIFACT")" ]] \
  || fail "marker non collocato nell'artefatto dichiarato"
path_is_within "$WORKTREE" "$ARTIFACT" || fail "worktree esterno all'artefatto V13"
path_is_within "$BIN" "$ARTIFACT" || fail "binario esterno all'artefatto V13"
path_is_within "$TARGETED_LOG" "$ARTIFACT" || fail "log mirato esterno all'artefatto V13"
path_is_within "$FULL_TEST_LOG" "$ARTIFACT" || fail "log suite esterno all'artefatto V13"

[[ "$(marker_value_from "$MARKER" robot_commit)" == "$ROBOT_EXPECTED" ]] || fail "robot commit marker inatteso"
[[ "$(marker_value_from "$MARKER" norma_base_commit)" == "$NORMA_EXPECTED" ]] || fail "norma base marker inattesa"
[[ "$V11_PARENT" == "$V11_PARENT_EXPECTED" ]] || fail "parent V11 inatteso nel marker V13"
[[ -f "$TARGETED_LOG" && -f "$FULL_TEST_LOG" ]] || fail "evidenze offline V13 mancanti"
grep -q "motion_timeout_covers_observed_m12_max_return ... ok" "$TARGETED_LOG" || fail "test ritorno M12 MAX non PASS"
grep -q "motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile ... ok" "$TARGETED_LOG" || fail "test timeout 24 profili non PASS"
grep -q "77 passed; 0 failed" "$FULL_TEST_LOG" || fail "suite completa V13 non 77/77 PASS"
echo "marker=$MARKER"
echo "marker_sha256=$MARKER_SHA256"
echo "PASS: artefatto V13 distance-aware individuato e coerente"

section "VERIFICA REPOSITORY, PATCH E BINARIO IMMUTABILE"
[[ -d "$ROBOT_REPO/.git" && -d "$NORMA_REPO/.git" ]] || fail "repository canonici assenti"
[[ "$(git -C "$ROBOT_REPO" branch --show-current)" == "main" ]] || fail "robot-dog non è su main"
[[ "$(git -C "$NORMA_REPO" branch --show-current)" == "main" ]] || fail "norma-core non è su main"
[[ -z "$(git -C "$ROBOT_REPO" status --porcelain)" ]] || fail "robot-dog non pulito"
[[ -z "$(git -C "$NORMA_REPO" status --porcelain)" ]] || fail "norma-core non pulito"
[[ "$(git -C "$ROBOT_REPO" rev-parse HEAD)" == "$ROBOT_EXPECTED" ]] || fail "robot-dog HEAD inatteso"
[[ "$(git -C "$NORMA_REPO" rev-parse HEAD)" == "$NORMA_EXPECTED" ]] || fail "norma-core HEAD inatteso"

[[ -d "$WORKTREE/.git" || -f "$WORKTREE/.git" ]] || fail "worktree V13 assente"
[[ "$(git -C "$WORKTREE" rev-parse HEAD)" == "$PATCH_EXPECTED" ]] || fail "worktree non è sul commit V13 dichiarato"
[[ "$(git -C "$WORKTREE" rev-parse HEAD^)" == "$V11_PARENT_EXPECTED" ]] || fail "patch V13 non discende dal parent V11 previsto"
[[ -z "$(git -C "$WORKTREE" status --porcelain)" ]] || fail "worktree V13 non pulito"
[[ "$(git -C "$WORKTREE" show -s --format=%s HEAD)" == "fix(matdog): make movement deadline distance-aware" ]] \
  || fail "subject commit V13 inatteso"
CHANGED="$(git -C "$WORKTREE" diff --name-only "$V11_PARENT_EXPECTED..$PATCH_EXPECTED" | sort)"
EXPECTED_CHANGED=$'software/drivers/st3215/src/auto_calibrate/matdog.rs\nsoftware/drivers/st3215/src/auto_calibrate/matdog_test.rs'
[[ "$CHANGED" == "$EXPECTED_CHANGED" ]] || { printf '%s\n' "$CHANGED"; fail "scope patch V13 inatteso"; }

[[ -x "$BIN" ]] || fail "binario V13 non eseguibile"
BIN_SHA256="$(sha256sum "$BIN" | awk '{print $1}')"
[[ "$BIN_SHA256" == "$BIN_EXPECTED_SHA256" ]] || fail "binario V13 modificato"
VERSION="$($BIN --version 2>&1 || true)"
[[ "$VERSION" == *"(${PATCH_SHORT})"* ]] || fail "binario non marcato $PATCH_SHORT"
BIN_STRINGS_TMP="$(mktemp)"
strings "$BIN" > "$BIN_STRINGS_TMP"
for token in \
  "Inspect restart-safe profile entry" \
  "Recover home-only joints to digital home" \
  "Establish geometry prerequisites from restart-safe state" \
  "restart-safe profile entry refused before motion" \
  "restart prerequisite inventory" \
  "establish prerequisite" \
  "move plan: M" \
  "timeout_ms="; do
  grep -Fq "$token" "$BIN_STRINGS_TMP" || { rm -f "$BIN_STRINGS_TMP"; fail "binario privo del token V13: $token"; }
done
rm -f "$BIN_STRINGS_TMP"

[[ -f "$CONFIG" ]] || fail "configurazione validata assente"
CONFIG_SHA256="$(sha256sum "$CONFIG" | awk '{print $1}')"
[[ "$CONFIG_SHA256" == "$CONFIG_EXPECTED_SHA256" ]] || fail "configurazione diversa dal pilot validato"
echo "local_patch_commit=$PATCH_EXPECTED"
echo "station_version=$VERSION"
echo "station_sha256=$BIN_SHA256"
echo "PASS: verrà eseguito esattamente il binario V13 validato"

section "PREFLIGHT HARDWARE FINALE"
if pgrep -af '(^|/)(station)([[:space:]]|$)' >/tmp/matdog_station_processes.$$ 2>/dev/null; then
  cat /tmp/matdog_station_processes.$$
  rm -f /tmp/matdog_station_processes.$$
  fail "Station è già in esecuzione"
fi
rm -f /tmp/matdog_station_processes.$$ 2>/dev/null || true
[[ -e "$SERIAL_BY_ID" ]] || fail "seriale persistente assente: $SERIAL_BY_ID"
OWNERS="$(serial_owners "$SERIAL_BY_ID")"
[[ -z "$OWNERS" ]] || { echo "$OWNERS"; fail "seriale già posseduta"; }
if ss -ltn 2>/dev/null | grep -Eq '127\.0\.0\.1:(8888|8889)[[:space:]]'; then
  ss -ltnp 2>/dev/null | grep -E '127\.0\.0\.1:(8888|8889)' || true
  fail "porta 8888 o 8889 occupata"
fi
echo "PASS: Station assente, seriale libera, porte libere"

STAMP="$(date +%Y%m%dT%H%M%S)"
RUN_DIR="$ARTIFACT/hardware_distance_timeout_v14_${STAMP}"
DATA_DIR="$RUN_DIR/station_data"
LOG="$RUN_DIR/station_${PROFILE}_${STAMP}.log"
META="$RUN_DIR/station_${PROFILE}_${STAMP}.meta"
SUMMARY="$RUN_DIR/SUMMARY.txt"
mkdir -p "$DATA_DIR"
cat > "$META" <<METAEOF
start_date=$(date --iso-8601=seconds)
profile=$PROFILE
bus=$BUS_SERIAL
architecture=$ARCHITECTURE_EXPECTED
robot_commit=$ROBOT_EXPECTED
norma_base_commit=$NORMA_EXPECTED
local_patch_commit=$PATCH_EXPECTED
station_version=$VERSION
station_sha256=$BIN_SHA256
validation_script=$V13_SCRIPT
validation_script_sha256=$V13_SCRIPT_SHA256
validation_marker=$MARKER
validation_marker_sha256=$MARKER_SHA256
config=$CONFIG
config_sha256=$CONFIG_SHA256
worktree=$WORKTREE
data_dir=$DATA_DIR
log=$LOG
METAEOF

section "AVVIO PROVA SUPERVISIONATA DISTANCE-AWARE"
echo "Robot completamente sostenuto e tutte le quattro zampe libere."
echo "M42 vicino a 2389 è riconosciuto come prerequisite valida, non come outlier."
echo "Solo i giunti home-only entro 64 tick vengono riportati a 2048."
echo "Poi M12 esegue MAX verso limite URDF 3442, guard 3506."
echo "Il ritorno 3327→2048 dispone ora di un timeout calcolato sulla distanza."
echo "Movimento inatteso: Stop Calibration. Emergenza: master disconnect."

setsid env MATDOG_NATIVE_CALIBRATOR_ARM="$PROFILE" RUST_LOG=info \
  "$BIN" --config "$CONFIG" --normfs-base-folder "$DATA_DIR" --tcp "$TCP_ADDR" --web "$WEB_ADDR" \
  >>"$LOG" 2>&1 &
STATION_PID=$!
echo "station_pid=$STATION_PID" | tee -a "$META"
trap 'echo; echo "CTRL-C ignorato durante la prova: usa Stop Calibration o master disconnect." >&2' INT

READY=0
for _ in $(seq 1 60); do
  if ! kill -0 "$STATION_PID" 2>/dev/null; then
    tail -n 180 "$LOG" || true
    fail "Station terminata durante startup"
  fi
  if grep -q "WebSocket server listening on ${WEB_ADDR}" "$LOG" \
     && grep -q "Successfully opened ST3215 port" "$LOG"; then
    READY=1
    break
  fi
  sleep 1
done
[[ "$READY" -eq 1 ]] || { tail -n 180 "$LOG" || true; fail "startup Station incompleto"; }

FILTER='MATDOG|Starting auto-calibration|Found [0-9]+ motor|contact:|move plan:|timeout_ms=|global torque|torque-OFF|Calibration|failed|complete|restart-safe|restart prerequisite|establish prerequisite|rejected'
tail -n 0 -F "$LOG" 2>/dev/null | grep --line-buffered -E "$FILTER" &
TAIL_PID=$!
command -v xdg-open >/dev/null 2>&1 && xdg-open "$WEB_URL" >/dev/null 2>&1 || true

section "AZIONE OPERATORE"
echo "1. Seleziona il bus $BUS_SERIAL."
echo "2. Premi soltanto Auto Calibrate."
echo "3. NON premere Save e NON premere Reset."

while true; do
  if grep -q "Starting auto-calibration sequence for bus ${BUS_SERIAL}" "$LOG"; then
    CALIBRATION_STARTED=1
  fi
  if grep -q "MATDOG ${PROFILE} complete:" "$LOG"; then
    RESULT="PASS"
    OUTCOME_SEEN=1
    break
  fi
  if grep -q "MATDOG native profile failed:" "$LOG" \
     || grep -q "torque-OFF cleanup failed" "$LOG"; then
    RESULT="FAIL"
    OUTCOME_SEEN=1
    break
  fi
  if ! kill -0 "$STATION_PID" 2>/dev/null; then
    RESULT="FAIL_STATION_EXIT"
    OUTCOME_SEEN=1
    break
  fi
  sleep 1
done

sleep 2
stop_tail
trap - INT
section "ARRESTO CONTROLLATO STATION"
stop_station_controlled

SERIAL_FREE=true
OWNERS_AFTER="$(serial_owners "$SERIAL_BY_ID")"
[[ -z "$OWNERS_AFTER" ]] || SERIAL_FREE=false
ENTRY_LINES="$(grep -E "MATDOG ${PROFILE} restart prerequisite inventory:|MATDOG ${PROFILE} establish prerequisite:" "$LOG" || true)"
RECOVERY_LINES="$(grep "MATDOG ${PROFILE} startup home recovery:" "$LOG" || true)"
CONTACT_LINES="$(grep "MATDOG ${PROFILE} contact:" "$LOG" || true)"
COMPLETE_LINES="$(grep "MATDOG ${PROFILE} complete:" "$LOG" || true)"
FAIL_LINES="$(grep -E "MATDOG native profile failed:|torque-OFF cleanup failed" "$LOG" || true)"
MOVE_PLAN_LINES="$(grep "MATDOG ${PROFILE} move plan:" "$LOG" || true)"
FINAL_RETURN_PLAN="$(grep -E "MATDOG ${PROFILE} move plan: M12 .*target=2048" "$LOG" | tail -n 1 || true)"
CONTACT_COUNT="$(grep -c "MATDOG ${PROFILE} contact:" "$LOG" || true)"

if [[ "$RESULT" == "PASS" && "$CONTACT_COUNT" -ne 2 ]]; then
  RESULT="FAIL_INCOMPLETE_CONTACT_EVIDENCE"
fi
if [[ "$RESULT" == "PASS" ]]; then
  [[ -n "$FINAL_RETURN_PLAN" ]] || RESULT="FAIL_MISSING_DISTANCE_AWARE_RETURN_PLAN"
fi
if [[ "$RESULT" == "PASS" ]]; then
  RETURN_DISTANCE="$(sed -n 's/.*distance=\([0-9][0-9]*\).*/\1/p' <<<"$FINAL_RETURN_PLAN")"
  RETURN_TIMEOUT_MS="$(sed -n 's/.*timeout_ms=\([0-9][0-9]*\).*/\1/p' <<<"$FINAL_RETURN_PLAN")"
  [[ "$RETURN_DISTANCE" =~ ^[0-9]+$ && "$RETURN_TIMEOUT_MS" =~ ^[0-9]+$ ]] \
    || RESULT="FAIL_INVALID_DISTANCE_AWARE_RETURN_PLAN"
fi
if [[ "$RESULT" == "PASS" ]]; then
  (( RETURN_DISTANCE > 1000 && RETURN_TIMEOUT_MS > 20000 )) \
    || RESULT="FAIL_RETURN_PLAN_BUDGET_TOO_SMALL"
fi
if [[ "$RESULT" == "PASS" && "$SERIAL_FREE" != true ]]; then
  RESULT="FAIL_SERIAL_NOT_RELEASED"
fi

cat > "$SUMMARY" <<SUMMARYEOF
MATDOG LF_UPPER_M12_MAX — hardware V14 from distance-aware V13
result=$RESULT
profile=$PROFILE
bus=$BUS_SERIAL
architecture=$ARCHITECTURE_EXPECTED
robot_commit=$ROBOT_EXPECTED
norma_base_commit=$NORMA_EXPECTED
local_patch_commit=$PATCH_EXPECTED
station_version=$VERSION
station_sha256=$BIN_SHA256
validation_script=$V13_SCRIPT
validation_script_sha256=$V13_SCRIPT_SHA256
validation_marker=$MARKER
validation_marker_sha256=$MARKER_SHA256
contact_count=$CONTACT_COUNT
serial_free_after_station_stop=$SERIAL_FREE

RESTART_SAFE_ENTRY_LOGS
$ENTRY_LINES

HOME_RECOVERY_LOGS
$RECOVERY_LINES

CONTACT_LOGS
$CONTACT_LINES

MOVE_PLAN_LOGS
$MOVE_PLAN_LINES

FINAL_RETURN_PLAN
$FINAL_RETURN_PLAN

COMPLETION_LOGS
$COMPLETE_LINES

FAILURE_LOGS
$FAIL_LINES

FILES
artifact=$ARTIFACT
worktree=$WORKTREE
targeted_tests=$TARGETED_LOG
full_tests=$FULL_TEST_LOG
log=$LOG
meta=$META
data_dir=$DATA_DIR
summary=$SUMMARY
SUMMARYEOF

{
  echo "end_date=$(date --iso-8601=seconds)"
  echo "result=$RESULT"
  echo "contact_count=$CONTACT_COUNT"
  echo "serial_free_after_station_stop=$SERIAL_FREE"
  echo "summary=$SUMMARY"
} >> "$META"

section "RISULTATO"
echo "result=$RESULT"
echo "contact_count=$CONTACT_COUNT"
echo "serial_free_after_station_stop=$SERIAL_FREE"
echo "local_patch_commit=$PATCH_EXPECTED"
echo "summary=$SUMMARY"
cat "$SUMMARY"
[[ "$RESULT" == "PASS" ]] || exit 1
