#!/usr/bin/env python3
"""S9 of the 19/11 HW procedure (optional step, run on the operator's request): SLOW steered walking on
1314bc411c58, the build that carries the kSteer re-certification (SLOW steer box, RF UPPER max 0.963456 -> 1.004874
rad, certificate 06c38f8d...). Joystick SLOW lateral and yaw at the SLOW limits the device publishes, with a USB STOP.

Nothing new is sent to the robot: the Suite class, every command, check, limit and stop rule are those of the pinned
fixture_suite copy given as first argument (it refuses any build but 1314bc411c58). The steps are those of block D
(D4-D10: lateral left and right, yaw counter-clockwise and clockwise, 12 s each, with their releases) and of block B
(B4-B6: @STOP during the yaw walk, 4 s held, STAND resume). Only the amplitudes differ: the SLOW entry of the
device's capabilities instead of STANDARD's (the suite's own check holds every joystick command to the selected
mode's published ellipsoid). No backward walk (not part of S9). The SLOW selection is checked read-only at rest
before the STAND (SLOW is the boot default and the one restored by every runner; nothing is sent if it is not).
M0 memory gate first, as in every runner. The unchanged http_watch.py runs on the same run directory.

A steered SLOW frame outside kSteer would end in the firmware's own FAULT WALK_IK_OR_WORKSPACE (SimpleMotion.cpp,
walkBox() = steerBox(SLOW) = kSteer while steered), a STOP outside it in STOP_MEASURED_OUTSIDE_BOX.

Usage: fixture_s9_slow_steer.py <pinned fixture_suite copy> <run-dir> [--plan]"""
import importlib.util, sys
from pathlib import Path


def load(path):
    spec = importlib.util.spec_from_file_location('fixture_suite_pinned', str(Path(path).resolve()))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def s9(S):
    slow = next(w for w in S.caps['walk_modes'] if w['name'] == 'SLOW')['joystick']
    yaw, lat = slow['yaw_rate_max_rad_s'], slow['vy_max_m_s']
    return [
        ('K0 MODE SLOW selected (read only, at rest)', lambda: {'selected_after_s': round(S.wait(
            'K0 MODE SLOW selected', lambda m: m['walk_mode'].get('selected') == 'SLOW', 5, allowed=('READY',))[1], 1)}),
        ('K1 STAND 120 from PRONE_REST', lambda: S.stand(120, from_states=('READY',))),
        ('K2 joystick lateral left SLOW full scale 12 s (as D4)', lambda: S.joystick(0.0, lat, 0.0, 12, 'K2 left')),
        ('K3 release (as D5)', lambda: S.joystick_release('K3 release')),
        ('K4 joystick lateral right SLOW full scale 12 s (as D6)', lambda: S.joystick(0.0, -lat, 0.0, 12, 'K4 right')),
        ('K5 release (as D7)', lambda: S.joystick_release('K5 release')),
        ('K6 joystick yaw counter-clockwise SLOW full scale 12 s (as D8)', lambda: S.joystick(0.0, 0.0, yaw, 12, 'K6 yaw ccw')),
        ('K7 release (as D9)', lambda: S.joystick_release('K7 release')),
        ('K8 joystick yaw clockwise SLOW full scale 12 s (as D10)', lambda: S.joystick(0.0, 0.0, -yaw, 12, 'K8 yaw cw')),
        ('K9 STOP during the yaw walk (as B4)', lambda: S.stop_walk('K9 STOP')),
        ('K10 wait 4 s held (as B5)', lambda: S.held(4, 'K10 held')),
        ('K11 STAND 120 resume (as B6)', lambda: S.stand(120, resume=True)),
        ('K12 PRONE to PRONE_REST', S.prone),
        ('K13 TIMING dump (S9)', lambda: S.timing_dump('K13')),
    ]


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    suite = load(sys.argv[1])
    suite.sequence = s9
    _ms = importlib.util.spec_from_file_location('memgate', str(Path(__file__).resolve().parent / 'memgate.py'))
    memgate = importlib.util.module_from_spec(_ms)
    _ms.loader.exec_module(memgate)
    suite.sequence = memgate.with_gate(suite.sequence, suite.Abort)  # M0 before any Motion command (16/11 MEMSTOP)
    sys.argv = [sys.argv[1]] + sys.argv[2:]
    sys.exit(suite.main())
