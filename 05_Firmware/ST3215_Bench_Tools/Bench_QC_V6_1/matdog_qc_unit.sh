#!/usr/bin/env bash
set -uo pipefail

if [ "$#" -ne 2 ]; then
    echo "USAGE: $0 LABEL ID"
    exit 2
fi

LABEL="$1"
ID="$2"

case "$LABEL" in
    *[!A-Za-z0-9_-]*|'')
        echo "ERROR: invalid LABEL"
        exit 2
        ;;
esac

if ! [[ "$ID" =~ ^[0-9]+$ ]] || [ "$ID" -gt 253 ]; then
    echo "ERROR: ID must be 0..253"
    exit 2
fi

BASE="$HOME/MATDOG/runtime/esp32"
RUNNER="$BASE/matdog_qc_runner.py"
OUT="$BASE/qc_campaign"
PORT='/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00'

mkdir -p "$OUT"

STAMP="$(date +%Y%m%d_%H%M%S)"
TMP="$(mktemp)"
LOG="$OUT/${LABEL}__id${ID}__${STAMP}.log"

SAFE_DONE=0

force_safe_off() {
python3 - "$ID" "$PORT" <<'PY'
import serial
import sys
import time

sid = int(sys.argv[1])
port = sys.argv[2]

try:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.25
    ser.write_timeout = 1
    ser.dtr = False
    ser.rts = False
    ser.open()

    time.sleep(1.5)
    ser.reset_input_buffer()

    ser.write(f"@SAFE_OFF {sid}\n".encode())
    ser.flush()

    deadline = time.monotonic() + 5.0
    text = ""

    while time.monotonic() < deadline:
        raw = ser.readline()

        if not raw:
            continue

        line = raw.decode(
            "utf-8",
            errors="replace"
        ).strip()

        print(line)
        text += line + "\n"

        if "SAFE_OFF_RESULT" in line:
            break

    ser.close()

    if (
        "SAFE_OFF_RESULT" not in text
        or "PASS" not in text
    ):
        print(
            "ERROR: final SAFE_OFF not verified",
            file=sys.stderr
        )
        sys.exit(1)

except Exception as exc:
    print(
        f"ERROR SAFE_OFF: {exc}",
        file=sys.stderr
    )
    sys.exit(1)
PY
}

cleanup() {
    if [ "$SAFE_DONE" -eq 0 ]; then
        echo
        echo "EMERGENCY/EXIT SAFE_OFF CHECK..."

        if ! force_safe_off; then
            echo
            echo "!!! CUT SERVO POWER MANUALLY !!!"
        fi
    fi

    rm -f "$TMP"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

if [ ! -f "$OUT/manifest.tsv" ]; then
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "timestamp" \
        "label" \
        "id" \
        "protocol" \
        "execution" \
        "checksum" \
        "performance_events" \
        "protective_stops" \
        "sha256" \
        "file" \
        > "$OUT/manifest.tsv"
fi

echo "================================================"
echo "MATDOG QC CAMPAIGN"
echo "LABEL : $LABEL"
echo "ID    : $ID"
echo "================================================"

set +e

python3 "$RUNNER" \
    --qc "$ID" \
    --confirm "QC_FAST_ID_${ID}" \
    2>&1 | tee "$TMP"

RC=${PIPESTATUS[0]}

set -e

cp "$TMP" "$LOG"

# Independent final torque-OFF after RAW is already saved.
echo
echo "FINAL INDEPENDENT SAFE_OFF..."

if force_safe_off; then
    SAFE_DONE=1
else
    echo
    echo "!!! CUT SERVO POWER MANUALLY !!!"
    exit 20
fi

BIN="$(
    sed -n \
        's/^FILE[[:space:]]*:[[:space:]]*//p' \
        "$TMP" |
    tail -n1
)"

if [ -z "$BIN" ] || [ ! -f "$BIN" ]; then
    echo "ERROR: RAW binary not found."
    echo "LOG preserved: $LOG"
    exit 21
fi

DEST="$OUT/${LABEL}__id${ID}__${STAMP}.bin"

mv "$BIN" "$DEST"

# Verify binary header belongs to frozen V6.1 (=61).
HEADER_INFO="$(
python3 - "$DEST" <<'PY'
import struct
import sys

p = sys.argv[1]

with open(p, "rb") as f:
    h = f.read(84)

if len(h) != 84:
    raise SystemExit("BAD_HEADER_SIZE")

vals = struct.unpack("<8s7I48s", h)

magic = vals[0]
version = vals[1]
sid = vals[2]
samples = vals[4]
sample_size = vals[5]
result = vals[7]

print(
    f"{magic!r}|{version}|{sid}|"
    f"{samples}|{sample_size}|{result}"
)
PY
)"

IFS='|' read -r \
    MAGIC VERSION BIN_ID SAMPLES SAMPLE_SIZE RESULT_CODE \
    <<< "$HEADER_INFO"

if [ "$VERSION" != "61" ]; then
    echo "ERROR: unexpected binary protocol: $VERSION"
    exit 22
fi

if [ "$BIN_ID" != "$ID" ]; then
    echo "ERROR: binary servo ID mismatch."
    exit 23
fi

SHA="$(
    sha256sum "$DEST" |
    awk '{print $1}'
)"

CHECKSUM="$(
    grep '^CHECKSUM' "$TMP" |
    tail -n1 |
    awk -F: '{
        gsub(/[[:space:]]/,"",$2);
        print $2
    }'
)"

EXECUTION="$(
    grep '^QC_EXECUTION' "$TMP" |
    tail -n1 |
    awk -F: '{
        gsub(/^[[:space:]]+|[[:space:]]+$/,"",$2);
        print $2
    }'
)"

PERF="$(
    grep '^PERFORMANCE_EVENTS' "$TMP" |
    tail -n1 |
    awk -F: '{
        gsub(/[[:space:]]/,"",$2);
        print $2
    }'
)"

PROTECT="$(
    grep '^PROTECTIVE_STOPS' "$TMP" |
    tail -n1 |
    awk -F: '{
        gsub(/[[:space:]]/,"",$2);
        print $2
    }'
)"

printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$STAMP" \
    "$LABEL" \
    "$ID" \
    "$VERSION" \
    "${EXECUTION:-UNKNOWN}" \
    "${CHECKSUM:-UNKNOWN}" \
    "${PERF:-UNKNOWN}" \
    "${PROTECT:-UNKNOWN}" \
    "$SHA" \
    "$DEST" \
    >> "$OUT/manifest.tsv"

echo
echo "===== UNIT SAVED ====="
echo "LABEL       : $LABEL"
echo "ID          : $ID"
echo "PROTOCOL    : V6.1 / binary 61"
echo "SAMPLES     : $SAMPLES"
echo "BIN         : $DEST"
echo "LOG         : $LOG"
echo "SHA256      : $SHA"
echo "EXECUTION   : ${EXECUTION:-UNKNOWN}"
echo "CHECKSUM    : ${CHECKSUM:-UNKNOWN}"
echo "PERF EVENTS : ${PERF:-UNKNOWN}"
echo "PROTECTIVE  : ${PROTECT:-UNKNOWN}"

exit "$RC"
