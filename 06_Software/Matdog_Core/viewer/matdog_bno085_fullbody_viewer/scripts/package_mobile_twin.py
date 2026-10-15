#!/usr/bin/env python3
"""Packages dist-mobile/ as the robot would serve it and reports exact bytes.

Adds matt_robodog_rev00.urdf and matdog_collision.mtw at the package root (served as /twin/...) (chosen grid),
writes a .gz sibling for every file (what the ESP32 would store and send with
Content-Encoding: gzip) and MOBILE_TWIN_PACKAGE.json with raw/gzip sizes and
SHA-256. Read-only on the canonical model; touches no firmware or hardware.
"""
import argparse, gzip, hashlib, json, shutil, sys
from pathlib import Path

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=Path, default=Path('dist-mobile'))
    ap.add_argument('--urdf', type=Path, required=True)
    ap.add_argument('--mtw', type=Path, required=True, help='uncompressed MTW1 pack')
    ap.add_argument('--app-free-bytes', type=int, default=5242880 - 3675520,
                    help='free bytes in the 5 MiB app slot (default: installed 83c3410 image)')
    a = ap.parse_args()
    twin = a.dist
    shutil.copyfile(a.urdf, twin / 'matt_robodog_rev00.urdf')
    shutil.copyfile(a.mtw, twin / 'matdog_collision.mtw')
    files = []
    for path in sorted(p for p in a.dist.rglob('*') if p.is_file() and p.suffix != '.gz' and p.name != 'MOBILE_TWIN_PACKAGE.json' and '.mtw-work' not in p.parts):
        raw = path.read_bytes(); gz = gzip.compress(raw, 9, mtime=0)
        path.with_name(path.name + '.gz').write_bytes(gz)
        files.append({'path': str(path.relative_to(a.dist)), 'raw_bytes': len(raw), 'gzip_bytes': len(gz),
                      'sha256': hashlib.sha256(raw).hexdigest()})
    total = sum(f['gzip_bytes'] for f in files)
    report = {'schema': 'MATDOG_MOBILE_TWIN_PACKAGE_V1', 'files': files, 'total_gzip_bytes': total,
              'total_raw_bytes': sum(f['raw_bytes'] for f in files), 'app_slot_free_bytes': a.app_free_bytes,
              'fraction_of_free_app_slot': round(total / a.app_free_bytes, 4),
              'note': 'Host build only. Not embedded in firmware, not flashed.'}
    (a.dist / 'MOBILE_TWIN_PACKAGE.json').write_text(json.dumps(report, indent=2) + '\n')
    for f in files: print(f"{f['path']:<44} raw={f['raw_bytes']:>8} gz={f['gzip_bytes']:>7}")
    print(f"TOTAL gzip={total} bytes ({100*report['fraction_of_free_app_slot']:.1f}% of {a.app_free_bytes} free app-slot bytes)")
    return 0

if __name__ == '__main__':
    sys.exit(main())
