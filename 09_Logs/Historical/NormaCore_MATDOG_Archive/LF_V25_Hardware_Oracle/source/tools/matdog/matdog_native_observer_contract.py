#!/usr/bin/env python3
"""External safety observer for the native MATDOG LF calibrator.

The Rust calibrator is the sole authority for motion choreography: state,
active motor, priming target, operational target, torque role, geometric
corridor, contact search, holds, returns and cleanup.  Those values are
validated inside the driver against the same telemetry that drives the state
machine.

This observer deliberately validates only invariants that are globally true
and independently observable from outside the driver.  It must never infer an
internal command policy from a phase string, progress step or joint identity.
"""

from __future__ import annotations

from typing import Any


def build_native_authority_contract(runner: Any):
    """Return a FrameContract that does not duplicate native motion policy."""

    base = runner.FrameContract
    if getattr(base, "_matdog_native_authority_observer", False):
        return base

    class NativeAuthorityFrameContract(base):
        _matdog_native_authority_observer = True

        def validate_running(self, frame: Any) -> None:
            # parse_frame() already enforces the exact bus, exact motor set,
            # unsigned 12-bit GoalPosition and complete ST3215 RAM image.
            # validate_common() adds stable app identity, monotonic/fresh
            # telemetry, driver/status/current and thermal invariants.
            #
            # Per-joint torque roles, target corridors and prime->target
            # transitions stay exclusively in the Rust state machine.
            self.validate_common(frame)

    NativeAuthorityFrameContract.__name__ = "NativeAuthorityFrameContract"
    NativeAuthorityFrameContract.__qualname__ = "NativeAuthorityFrameContract"
    return NativeAuthorityFrameContract


def install(runner: Any):
    """Install the observer contract once and return the effective class."""

    runner.FrameContract = build_native_authority_contract(runner)
    return runner.FrameContract
