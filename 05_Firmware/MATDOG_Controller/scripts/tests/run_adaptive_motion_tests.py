#!/usr/bin/env python3
"""Compile pure G1/G2/G4 feedback integration and emit synthetic viewer telemetry."""
from pathlib import Path
import argparse
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
FW = ROOT / '05_Firmware/MATDOG_Controller'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trace', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='matdog-adaptive-') as temp:
        binary = Path(temp) / 'adaptive_motion_tests'
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O2',
                        '-I' + str(FW / 'src'),
                        str(FW / 'scripts/tests/test_adaptive_motion.cpp'),
                        *map(str, sorted((FW / 'src/motion').glob('*.cpp'))),
                        '-o', str(binary)], check=True)
        trace_temp = None
        try:
            if args.trace:
                args.trace.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(prefix='.' + args.trace.name + '.', suffix='.tmp',
                                                  dir=args.trace.parent, delete=False) as export:
                    trace_temp = Path(export.name)
            subprocess.run([str(binary), *([str(trace_temp)] if trace_temp else [])], check=True)
            if trace_temp:
                trace_temp.replace(args.trace)  # publish a complete successful export atomically
        finally:
            if trace_temp:
                trace_temp.unlink(missing_ok=True)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
