#!/usr/bin/env python3
"""Motion FP32 (18/11/2026): refresh motion_final_release_sources.json for the PR-2 copied sources the change touched
(sha256 of the working tree file); never adds or removes a pinned path. Usage: update_pins.py <repo root>"""
import hashlib, json, subprocess, sys
from pathlib import Path
root = Path(sys.argv[1]); T = root / '05_Firmware/MATDOG_Controller/scripts/tests'
ext_path = T / 'motion_final_release_sources.json'; ext = json.loads(ext_path.read_text())
copied = {r['path']: r['sha256'] for r in json.loads((T / 'motion_integration_manifest_pr2.json').read_text())['copied_sources']}
changed = subprocess.run(['git', '-C', str(root), 'diff', '--name-only', 'HEAD'], capture_output=True, text=True, check=True).stdout.split()
for path in sorted(p for p in changed if p in copied):
    sha = hashlib.sha256((root / path).read_bytes()).hexdigest()
    old = ext['updated_copied_sources'].get(path, copied[path] + ' (PR-2)')
    ext['updated_copied_sources'][path] = sha
    print(f'{path}: {old[:16]} -> {sha[:16]}')
ext_path.write_text(json.dumps(ext, indent=2) + '\n' if ext_path.read_text().endswith('\n') else json.dumps(ext, indent=2))
