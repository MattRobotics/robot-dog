#!/usr/bin/env python3
"""Block selection of the qualified fixture suite (F3 + IRAM G2 validation, 16/11/2026 evening).

Runs only the chosen blocks of the pinned A-G sequence, in their original order: A,B for the same-session
Timing reference (a86c680 and cb9ed85, no FAST) and E,F,G for the FAST qualification on cb9ed85 (E -> F1-F9,
then G1 PRONE and the final TIMING dump).

Nothing new is sent to the robot: the Suite class, every command, check, limit and stop rule are those of the
pinned fixture_suite copy given as first argument (fixture_suite_a86c680.py before the flash, fixture_suite.py
after it; each refuses any other build). Only the step list is shorter, as in fixture_settle.py. The unchanged
http_watch.py runs on the same run directory, as for the suite. Every chosen block starts from PRONE_REST/READY
and ends in PRONE_REST (A ends at A8 STAND 120, which B continues: A is never run without B).

Usage: fixture_blocks.py <pinned fixture_suite copy> <run-dir> --blocks A,B|E,F,G [--plan]"""
import importlib.util, sys
from pathlib import Path

ALLOWED = ({'A', 'B'}, {'E', 'F', 'G'})


def load(path):
    spec = importlib.util.spec_from_file_location('fixture_suite_pinned', str(Path(path).resolve()))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


if __name__ == '__main__':
    if len(sys.argv) < 3 or '--blocks' not in sys.argv:
        print(__doc__)
        sys.exit(2)
    blocks = set(sys.argv[sys.argv.index('--blocks') + 1].split(','))
    if blocks not in ALLOWED:
        print('refused: --blocks must be A,B or E,F,G')
        sys.exit(2)
    suite = load(sys.argv[1])
    full = suite.sequence
    suite.sequence = lambda s: [x for x in full(s) if x[0].split()[0][0] in blocks]
    _ms = importlib.util.spec_from_file_location('memgate', str(Path(__file__).resolve().parent / 'memgate.py'))
    memgate = importlib.util.module_from_spec(_ms)
    _ms.loader.exec_module(memgate)
    suite.sequence = memgate.with_gate(suite.sequence, suite.Abort)  # M0 before any Motion command (16/11 MEMSTOP)
    sys.argv = [sys.argv[1]] + [a for a in sys.argv[2:] if a not in ('--blocks', sys.argv[sys.argv.index('--blocks') + 1])]
    sys.exit(suite.main())
