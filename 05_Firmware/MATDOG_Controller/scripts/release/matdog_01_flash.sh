#!/usr/bin/env bash
# Packaged entry point. With no arguments it performs file-only checks.
# Application-only flash never admits motion; pose qualification belongs to stage 02.
set -euo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -eq 0 ]; then
  set -- --offline-check
fi
exec python3 "$PACKAGE_DIR/automation/matdog_release_session.py" flash --package "$PACKAGE_DIR" "$@"
