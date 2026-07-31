#!/usr/bin/env bash
set -Eeuo pipefail

EXPECTED_V32_BLOB=ffbc29dba0952cf241f605a8f4f7dfa881d003fa
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BASE="$SCRIPT_DIR/MATDOG_LF_LOWER_M11_MAX_HARDWARE_SUPERVISED_V32.sh"
ARCHIVE_ROOT=${MATDOG_VERIFICATION_ARCHIVE:-$HOME/MATDOG/_archive/verification-artifacts}
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
PATCH_DIR="$ARCHIVE_ROOT/MATDOG_LF_LOWER_M11_MAX_HTTP_DECODE_PATCH_V33_$STAMP"
PATCHED="$PATCH_DIR/MATDOG_LF_LOWER_M11_MAX_HARDWARE_SUPERVISED_V33_materialized.sh"

fail() { printf 'HARD BLOCK: %s\n' "$*" >&2; exit 1; }

[[ ${1:-} == --launch-supervised ]] || {
  printf '%s\n' \
    "Usage:" \
    "  bash MATDOG_LF_LOWER_M11_MAX_HARDWARE_SUPERVISED_V33.sh --launch-supervised" \
    "" \
    "V33 changes only HTTP content decoding and diagnostic headers." \
    "Auto Calibrate remains a manual UI action."
  exit 64
}

for cmd in git python3 bash sha256sum; do
  command -v "$cmd" >/dev/null 2>&1 || fail "required command missing: $cmd"
done
[[ -f "$BASE" ]] || fail "reviewed V32 launcher missing: $BASE"
[[ "$(git hash-object "$BASE")" == "$EXPECTED_V32_BLOB" ]] ||
  fail "reviewed V32 launcher blob mismatch"

mkdir -p "$PATCH_DIR"
python3 - "$BASE" "$PATCHED" <<'PY'
from pathlib import Path
import sys

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
text = src.read_text(encoding="utf-8")

replacements = {
    'if curl -fsS http://127.0.0.1:8889/ -o "$HTTP_INDEX"; then':
        'if curl --compressed -fsS -D "$HTTP_DIR/index.headers" http://127.0.0.1:8889/ -o "$HTTP_INDEX"; then',
    'curl -fsS "http://127.0.0.1:8889/$UI_REL" -o "$SERVED_MATDOG_UI" || fail "served MATDOG UI asset not available"':
        'curl --compressed -fsS -D "$HTTP_DIR/matdog-ui.headers" "http://127.0.0.1:8889/$UI_REL" -o "$SERVED_MATDOG_UI" || fail "served MATDOG UI asset not available"',
    'MATDOG_${EXPECTED_PROFILE}_HARDWARE_SUPERVISED_V32_${STAMP}':
        'MATDOG_${EXPECTED_PROFILE}_HARDWARE_SUPERVISED_V33_${STAMP}',
    'runner=V32': 'runner=V33_HTTP_DECODE',
}

for old, new in replacements.items():
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"expected exactly one occurrence, found {count}: {old}")
    text = text.replace(old, new, 1)

if 'curl --compressed -fsS -D "$HTTP_DIR/matdog-ui.headers"' not in text:
    raise SystemExit("decoded asset fetch was not materialized")
if 'Auto Calibrate only when you are physically ready' not in text:
    raise SystemExit("manual UI gate missing after materialization")
if '--execute' in text or 'MATDOG_OPERATOR_ACK' in text:
    raise SystemExit("unexpected automatic hardware execution path")

dst.write_text(text, encoding="utf-8")
PY

bash -n "$PATCHED"
chmod 700 "$PATCHED"
sha256sum "$BASE" "$PATCHED" > "$PATCH_DIR/SHA256SUMS"
cat > "$PATCH_DIR/PATCH.env" <<EOF
result=MATERIALIZED
profile=LF_LOWER_M11_MAX
base_launcher_blob=$EXPECTED_V32_BLOB
patched_launcher=$PATCHED
http_content_decoding=curl_compressed
http_headers_recorded=true
auto_calibrate_invocation=manual_ui_only
created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
sha256sum "$PATCH_DIR/PATCH.env" >> "$PATCH_DIR/SHA256SUMS"

exec bash "$PATCHED" --launch-supervised
