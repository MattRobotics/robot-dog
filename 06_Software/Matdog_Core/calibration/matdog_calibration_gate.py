"""MATDOG calibration gate — fail-closed authorization for live/hardware operation.

Single shared mechanism that decides whether the recorded calibration may be used to
touch hardware. Every live/hardware entry point calls :func:`require_hardware_authorized`
*before* acquiring a Station client, serial port or any other hardware handle.

Why this exists
---------------
`MATDOG_JOINT_CALIBRATION.yaml` carries legacy per-stage status strings
(``DIRECTION_MAPPING_COMPLETE_ZERO_PENDING`` -> ``VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION``
-> ``DIGITAL_ZERO_CALIBRATED_AND_VERIFIED``) and per-joint values such as
``zero_encoder_visual``. Different tools assert *different* strings, and some tools
assert nothing at all. After the 2026-08-27 reassembly every one of those values became
stale, but nothing in the code could tell.

The `calibration_reset` block is therefore the single authority. It overrides every
legacy status field. Historical values stay in the file for provenance; this gate makes
sure no live consumer can treat them as active.

See: 09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "MATDOG_JOINT_CALIBRATION.yaml"

#: The only state in which recorded calibration may authorize hardware.
STATE_CALIBRATED = "CALIBRATED_AND_VERIFIED"

#: Current state after the 2026-08-27 reassembly.
STATE_RESET_PENDING = "CALIBRATION_RESET_PENDING_FULL_RECALIBRATION"

#: Legacy per-stage strings. Recorded so the gate can explain itself; never trusted.
LEGACY_STAGE_STATUSES = (
    "DIRECTION_MAPPING_COMPLETE_ZERO_PENDING",
    "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION",
    "DIGITAL_ZERO_CALIBRATED_AND_VERIFIED",
)


class CalibrationGateError(RuntimeError):
    """Base class for calibration gate refusals."""


class CalibrationResetError(CalibrationGateError):
    """Raised when stale calibration is asked to authorize hardware."""


def load_calibration(config_path: Path | str | None = None) -> dict[str, Any]:
    """Load the calibration config. Pure read, no authorization implied."""
    path = Path(config_path) if config_path is not None else DEFAULT_CONFIG_PATH
    if not path.is_file():
        raise CalibrationGateError(f"calibration config not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise CalibrationGateError(f"calibration config is not a mapping: {path}")
    return dict(data)


def calibration_state(data: Mapping[str, Any]) -> str:
    """Return the authoritative calibration state.

    ``calibration_reset.state`` wins over every legacy status field. If the block is
    absent the state is unknown, which is treated as *not* authorized — a config
    predating the reset block cannot vouch for the current robot.
    """
    reset = data.get("calibration_reset")
    if isinstance(reset, Mapping):
        state = reset.get("state")
        if isinstance(state, str) and state:
            return state
        return "UNKNOWN_MISSING_STATE"
    return "UNKNOWN_NO_CALIBRATION_RESET_BLOCK"


def hardware_motion_authorized(data: Mapping[str, Any]) -> bool:
    """True only when the config explicitly authorizes hardware motion.

    Requires *both* an explicit ``hardware_motion_authorized: true`` and a calibrated
    state. Anything else — missing block, missing key, unknown state — is False.
    """
    reset = data.get("calibration_reset")
    if not isinstance(reset, Mapping):
        return False
    if reset.get("hardware_motion_authorized") is not True:
        return False
    return calibration_state(data) == STATE_CALIBRATED


def refusal_reason(data: Mapping[str, Any]) -> str:
    """Human-readable explanation of why the gate refuses. Empty if it does not."""
    if hardware_motion_authorized(data):
        return ""
    state = calibration_state(data)
    reset = data.get("calibration_reset")
    if not isinstance(reset, Mapping):
        return (
            "no calibration_reset block: this config predates the 2026-08-27 reset and "
            "cannot vouch for the current robot"
        )
    if state == STATE_RESET_PENDING:
        legacy = (data.get("robot") or {}).get("calibration_status")
        return (
            f"calibration state is {state}. All 17 servos were removed, provisioned to "
            "PositionOffset=0 and remounted on 2026-08-27; every joint zero, direction "
            "and limit on record describes an installation that no longer exists"
            + (
                f". The legacy robot.calibration_status ({legacy!r}) is preserved for "
                "provenance and is NOT active truth"
                if legacy
                else ""
            )
        )
    if reset.get("hardware_motion_authorized") is not True:
        return f"hardware_motion_authorized is not true (state {state})"
    return f"calibration state is {state}, expected {STATE_CALIBRATED}"


def require_hardware_authorized(
    tool: str,
    config_path: Path | str | None = None,
    *,
    data: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Fail closed unless the recorded calibration may drive hardware.

    Call this **before** acquiring a Station client, serial port or any hardware handle.

    Returns the loaded config on success so callers need not read the file twice.
    Raises :class:`CalibrationResetError` otherwise.
    """
    loaded = dict(data) if data is not None else load_calibration(config_path)
    if hardware_motion_authorized(loaded):
        return loaded

    raise CalibrationResetError(
        f"HARDWARE BLOCKED — {tool}\n"
        f"  {refusal_reason(loaded)}\n"
        "  Required: complete full recalibration on the new installation, then set\n"
        f"    calibration_reset.state: {STATE_CALIBRATED}\n"
        "    calibration_reset.hardware_motion_authorized: true\n"
        "  See 09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md"
    )


def load_for_historical_inspection(
    config_path: Path | str | None = None,
) -> tuple[dict[str, Any], bool]:
    """Read-only historical inspection. Never authorizes hardware.

    Returns ``(data, is_stale)``. Callers displaying these values **must** label them
    stale when ``is_stale`` is True.
    """
    data = load_calibration(config_path)
    return data, not hardware_motion_authorized(data)


def stale_banner(data: Mapping[str, Any]) -> str:
    """One-line banner for tools that display historical calibration values."""
    if hardware_motion_authorized(data):
        return ""
    return (
        "*** STALE CALIBRATION — HISTORICAL VALUES ONLY, NOT CURRENT ROBOT STATE *** "
        f"({calibration_state(data)})"
    )
