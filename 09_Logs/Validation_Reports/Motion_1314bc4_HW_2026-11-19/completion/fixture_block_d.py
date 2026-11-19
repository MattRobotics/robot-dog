#!/usr/bin/env python3
"""Block D alone (19/11/2026, completion of the 1314bc411c58 qualification): steps D1-D23 of the qualified fixture
suite, never run on this build so far (run_ab_chain.sh runs settle and blocks A,B,E,F,G,H only): STAND 120, joystick
backward D2, lateral left D4 and right D6, yaw D8/D10, each 12 s with its release, WALK STOP in two gait phases with
STAND and WALK resumes, PRONE and the TIMING dump.

Nothing new is sent to the robot: the Suite class, every command, check, limit and stop rule are those of the pinned
fixture_suite copy given as first argument (it refuses any build but 1314bc411c58), and the D steps are taken by name
from its own sequence. In the full suite block D runs with STANDARD selected (B1) and its joystick amplitudes are
STANDARD's published maxima, which the suite's own check refuses against the SLOW ellipsoid: as fixture_settle.py does
for D8-D11, MODE STANDARD and the read-only wait for its publication come right after D1, and MODE SLOW (default
restored, as F9) right before D22 PRONE. M0 memory gate first, as in every runner. The unchanged http_watch.py runs on
the same run directory (run_acquisition.py).

Usage: fixture_block_d.py <pinned fixture_suite copy> <run-dir> [--plan]"""
import importlib.util, re, sys
from pathlib import Path


def load(path):
    spec = importlib.util.spec_from_file_location('fixture_suite_pinned', str(Path(path).resolve()))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def block_d(full):
    def sequence(S):
        d = [x for x in full(S) if re.fullmatch(r'D\d+', x[0].split()[0])]
        assert [x[0].split()[0] for x in d] == ['D%d' % i for i in range(1, 24)], 'block D of the pinned suite changed'
        standard = [
            ('Dm1 MODE STANDARD (as B1)', lambda: S.gait('STANDARD', ('WALK_MODE_SELECTED',))),
            ('Dm2 MODE STANDARD published (read only)', lambda: {'published_after_s': round(S.wait(
                'Dm2 MODE STANDARD published', lambda m: m['walk_mode'].get('selected') == 'STANDARD', 5, allowed=('STANDING',))[1], 1)}),
        ]
        slow = [('Dm3 MODE SLOW (default restored, as F9)', lambda: S.gait('SLOW', ('WALK_MODE_SELECTED',)))]
        return d[:1] + standard + d[1:21] + slow + d[21:]
    return sequence


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    suite = load(sys.argv[1])
    suite.sequence = block_d(suite.sequence)
    _ms = importlib.util.spec_from_file_location('memgate', str(Path(__file__).resolve().parent / 'memgate.py'))
    memgate = importlib.util.module_from_spec(_ms)
    _ms.loader.exec_module(memgate)
    suite.sequence = memgate.with_gate(suite.sequence, suite.Abort)  # M0 before any Motion command (16/11 MEMSTOP)
    sys.argv = [sys.argv[1]] + sys.argv[2:]
    sys.exit(suite.main())
