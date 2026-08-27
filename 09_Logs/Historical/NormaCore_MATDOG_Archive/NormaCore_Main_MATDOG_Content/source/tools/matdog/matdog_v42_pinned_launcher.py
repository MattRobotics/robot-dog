#!/usr/bin/env python3
"""Immutable launcher for the reviewed MATDOG LF measurement/freeze artifact.

This module creates no register command. It verifies the reviewed headless
runner and native-authority observer byte-for-byte, installs the observer,
replaces only the Station executable SHA-256 pin, and delegates to the
fail-closed runner. EEPROM provisioning remains a separate post-measurement
transaction performed only after Station has stopped and released the serial
adapter.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "tools/matdog/matdog_headless_auto_calibrate.py"
OBSERVER_PATH = REPO_ROOT / "tools/matdog/matdog_native_observer_contract.py"

EXPECTED_RUNNER_SHA256 = (
    "9eccb4aa88c3496e6d4e986d9de2d5fea3d8185d1bd3ea4855d1d3aaa6945613"
)
EXPECTED_OBSERVER_SHA256 = (
    "b9521f97ed0a3cf4d7f39d8712c2fb7a060fa56bbbc2a10b8709742d6b0a5167"
)
PINNED_STATION_SHA256 = (
    "df4f6965d5c6b5eaecdc7f937391392dff0a1ca1cac166ab898d9a7c530f4651"
)
PINNED_STATION_SOURCE_COMMIT = "3f6a9099ea11f90da5981d9ea2cada1c7779878b"
PINNED_STATION_ARTIFACT_ID = 8869874935
PINNED_STATION_ARTIFACT_ZIP_SHA256 = (
    "ec7fd93805e73ea0691638cf863eda30dc0bbf5212db7bc25032ed6b270560d6"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_reviewed_file(path: Path, expected_sha256: str, label: str) -> None:
    actual = sha256_file(path)
    if actual != expected_sha256:
        raise RuntimeError(
            f"refusing to launch an unreviewed MATDOG {label}: "
            f"actual={actual}, expected={expected_sha256}"
        )


def load_reviewed_runner():
    require_reviewed_file(RUNNER_PATH, EXPECTED_RUNNER_SHA256, "runner")
    require_reviewed_file(OBSERVER_PATH, EXPECTED_OBSERVER_SHA256, "observer")

    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from tools.matdog import matdog_headless_auto_calibrate as runner
    from tools.matdog import matdog_native_observer_contract

    matdog_native_observer_contract.install(runner)
    runner.EXPECTED_STATION_SHA256 = PINNED_STATION_SHA256
    return runner


def main() -> int:
    runner = load_reviewed_runner()
    return runner.main()


if __name__ == "__main__":
    raise SystemExit(main())
