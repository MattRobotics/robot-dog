#!/usr/bin/env bash
# Packaged entry point. With no arguments it performs file-only checks.
set -euo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -eq 0 ]; then
  set -- --offline-check
fi
exec python3 "$PACKAGE_DIR/automation/matdog_release_session.py" calibrate --package "$PACKAGE_DIR" "$@"
