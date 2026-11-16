#!/usr/bin/env python3
"""Targeted WALK_SETTLE acquisition (P0 closure, 16/11/2026): steps D8-D11 of the qualified fixture
suite (yaw counter-clockwise 12 s, release, yaw clockwise 12 s, release), the two releases that gave the
WALK_SETTLE peaks of 16/11 (24.0 and 26.8 ms), between a STAND 120 from PRONE_REST and a PRONE, with a
since-boot @MOTION TIMING dump before and after.

Nothing new is sent to the robot: the Suite class, every command, check, limit and stop rule are those
of the pinned fixture_suite copy given as first argument (fixture_suite_82c1af9.py before the flash,
fixture_suite.py after it; each refuses any other build). Only the step list is shorter. The unchanged
http_watch.py runs on the same run directory, as for the suite.

Usage: fixture_settle.py <pinned fixture_suite copy> <run-dir> [--plan]"""
import importlib.util, sys
from pathlib import Path


def load(path):
    spec = importlib.util.spec_from_file_location('fixture_suite_pinned', str(Path(path).resolve()))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def targeted(S):
    std = next(w for w in S.caps['walk_modes'] if w['name'] == 'STANDARD')['joystick']
    yaw = std['yaw_rate_max_rad_s']
    return [
        ('S0 TIMING dump (since boot, Motion inactive)', lambda: S.timing_dump('S0')),
        ('S1 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        ('S2 MODE STANDARD', lambda: S.gait('STANDARD', ('WALK_MODE_SELECTED',))),
        # 16/11 t82 abort: S3 ran in the second of the S2 ACK, on an HTTP sample older than the selection, and the
        # suite's own check refused the STANDARD yaw against the SLOW ellipsoid (in the suite another step always
        # comes between). Read only: wait until the status publishes the selection; nothing is sent.
        ('S2b MODE STANDARD published (read only)', lambda: {'published_after_s': round(S.wait(
            'S2b MODE STANDARD published', lambda m: m['walk_mode'].get('selected') == 'STANDARD', 5, allowed=('STANDING',))[1], 1)}),
        ('S3 joystick yaw counter-clockwise full scale 12 s (as D8)', lambda: S.joystick(0.0, 0.0, yaw, 12, 'S3 yaw ccw')),
        ('S4 release (as D9)', lambda: S.joystick_release('S4 release')),
        ('S5 joystick yaw clockwise full scale 12 s (as D10)', lambda: S.joystick(0.0, 0.0, -yaw, 12, 'S5 yaw cw')),
        ('S6 release (as D11)', lambda: S.joystick_release('S6 release')),
        ('S7 MODE SLOW (default restored, as F9)', lambda: S.gait('SLOW', ('WALK_MODE_SELECTED',))),
        ('S8 PRONE to PRONE_REST', S.prone),
        ('S9 TIMING dump (since boot: PMU sums and heaviest release per class)', lambda: S.timing_dump('S9')),
    ]


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    suite = load(sys.argv[1])
    suite.sequence = targeted
    sys.argv = [sys.argv[1]] + sys.argv[2:]
    sys.exit(suite.main())
