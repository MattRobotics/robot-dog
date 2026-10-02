"""Run every offline G4 validation gate on the CURRENT source and record validation_results.json.

Software-only: host compilers, Python and git. No serial port, no network transport, no hardware.
Run from anywhere with the pinned pose-audit virtual environment:  <venv>/bin/python validate.py
Order: validate.py -> write REPORT.md -> artifact_manifest.py -> artifact_manifest.py --check and
test_gait_audit.py again, because the manifest indexes validation_results.json and REPORT.md.
"""
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
FW = ROOT / '05_Firmware/MATDOG_Controller'
TESTS = FW / 'scripts/tests'
PY = sys.executable
CXX = os.environ.get('CXX', 'g++')
BASELINE = 'c4befbe90b3121d60ba1b9ba09dc1082364c8d88'
SUMMARY = re.compile(r'(GAIT_HOST|STATIC_AUDIT|REVALIDATE_SAVED|ARTIFACT_MANIFEST|Ran \d+ tests|^OK|PASS|passed|"status")', re.I)
REPORT_TOOLING = ('validate.py', 'artifact_manifest.py')


def sha_bytes(data):
    return hashlib.sha256(data).hexdigest()


def source_digest():
    audited = [p for p in HERE.glob('*.py') if p.name not in REPORT_TOOLING]
    paths = sorted(audited + list((FW / 'src/motion').glob('*')) + [p for p in TESTS.iterdir() if p.is_file()] + [FW / 'scripts/static_audit.py'])
    lines = [f'{p.relative_to(ROOT)} {sha_bytes(p.read_bytes())}' for p in paths]
    return sha_bytes('\n'.join(lines).encode()), len(lines)


def run(name, cmd, cwd=ROOT, env=None, timeout=7200, expect_stdout=None):
    start = time.time()
    proc = subprocess.run(cmd, cwd=cwd, env={**os.environ, **(env or {})}, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
    lines = [l for l in proc.stdout.splitlines() if l.strip()]
    ran = [l for l in lines if re.match(r'Ran \d+ tests?', l)]
    matches = [l for l in lines if SUMMARY.search(l)]
    summary = (ran[-1] + '; ' + lines[-1]) if ran else (matches or lines or [''])[-1].strip()
    passed = proc.returncode == 0 and (expect_stdout is None or expect_stdout in proc.stdout)
    gate = {'name': name, 'command': ' '.join(str(c) for c in cmd), 'returncode': proc.returncode, 'passed': passed,
            'seconds': round(time.time() - start, 1), 'summary': summary, 'output_sha256': sha_bytes(proc.stdout.encode()),
            'output_lines': len(lines)}
    print(f"{'PASS' if passed else 'FAIL'} {name} ({gate['seconds']}s): {summary}", flush=True)
    return gate


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, stdout=subprocess.PIPE, text=True, check=True).stdout


def main():
    tmp = Path(tempfile.mkdtemp(prefix='matdog-g4-validate-'))
    head = git('rev-parse', 'HEAD').strip()
    dirty = git('status', '--porcelain').strip()
    venv_bin = str(Path(PY).parent)
    env = {'OPENBLAS_NUM_THREADS': '1', 'PATH': venv_bin + os.pathsep + os.environ['PATH']}
    gates = []
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-fno-exceptions', '-fno-rtti']
    srcs = [str(TESTS / 'test_gait.cpp'), *map(str, sorted((FW / 'src/motion').glob('*.cpp')))]
    gates.append(run('g4_cpp_unit_build', [CXX, *flags, '-O1', *srcs, '-o', str(tmp / 'test_gait')]))
    gates.append(run('g4_cpp_unit_tests', [str(tmp / 'test_gait')], expect_stdout='GAIT_HOST = PASS'))
    gates.append(run('g4_independent_oracle', [PY, str(HERE / 'oracle.py'), '--output', 'oracle_results_final.json'], cwd=HERE, env=env, expect_stdout='"status": "PASS"'))
    gates.append(run('saved_frames_vs_current_core', [PY, str(HERE / 'revalidate_saved.py')], cwd=HERE, env=env, expect_stdout='REVALIDATE_SAVED PASS'))
    gates.append(run('g4_artifact_tests', [PY, str(HERE / 'test_gait_audit.py')], cwd=HERE, env=env))
    gates.append(run('g35_pose_audit_tests', [PY, str(ROOT / '06_Software/Matdog_Core/pose_audit/test_pose_audit.py')], cwd=ROOT / '06_Software/Matdog_Core/pose_audit', env=env))
    gates.append(run('g35_artifact_manifest_check', [PY, str(ROOT / '06_Software/Matdog_Core/pose_audit/artifact_manifest.py'), '--check'], env=env, expect_stdout='ARTIFACT_MANIFEST OK'))
    gates.append(run('host_motion_runner_g1_g2_g3_g35_g4', ['bash', str(TESTS / 'run_motion_host_tests.sh')], env=env))
    gates.append(run('static_audit', [PY, str(FW / 'scripts/static_audit.py')], env=env, expect_stdout='STATIC_AUDIT PASS'))
    san = [CXX, *flags, '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', *srcs, '-o', str(tmp / 'test_gait_san')]
    gates.append(run('g4_asan_ubsan_build', san))
    gates.append(run('g4_asan_ubsan_run', [str(tmp / 'test_gait_san')], env={'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1', 'UBSAN_OPTIONS': 'halt_on_error=1:print_stacktrace=1'}, expect_stdout='GAIT_HOST = PASS'))
    gates.append(run('git_diff_check_worktree', ['git', 'diff', '--check']))
    gates.append(run('git_diff_check_vs_baseline', ['git', 'diff', '--check', BASELINE + '..HEAD']))
    changed = git('diff', '--name-status', BASELINE + '..HEAD', '--', '05_Firmware/MATDOG_Controller/src/motion').split('\n')
    allowed = {'05_Firmware/MATDOG_Controller/src/motion/' + n for n in ('Gait.cpp', 'Gait.h', 'Locomotion.cpp', 'Locomotion.h')}
    added = {l.split('\t')[1] for l in changed if l.startswith('A\t')}
    other = [l for l in changed if l and not l.startswith('A\t')]
    gate = {'name': 'g1_g2_g3_motion_sources_unchanged_vs_baseline', 'command': 'git diff --name-status ' + BASELINE + '..HEAD -- src/motion',
            'returncode': 0, 'passed': added == allowed and not other, 'summary': 'only Gait.cpp/.h and Locomotion.cpp/.h added; no accepted motion source modified',
            'added': sorted(added), 'modified_or_deleted': other}
    print(('PASS ' if gate['passed'] else 'FAIL ') + gate['name'])
    gates.append(gate)
    digest, count = source_digest()
    results = {'all_passed': all(g['passed'] for g in gates), 'gates': gates,
               'git': {'head_at_validation': head, 'worktree_clean_before_validation': not dirty, 'baseline': BASELINE},
               'scope': 'offline software gates only: host compiler, Python, git; no hardware, serial device, ServoBus, servo command, flashing, EEPROM or calibration was used',
               'source_digest': digest, 'source_digest_file_count': count,
               'provenance': {'generator_sha256': sha_bytes(Path(__file__).read_bytes())}}
    (OUT / 'validation_results.json').write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
    print('ALL_PASSED' if results['all_passed'] else 'SOME_FAILED')
    return 0 if results['all_passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
