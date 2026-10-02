"""Run every offline G4.1 validation gate on the CURRENT source and record validation_results.json.

Software-only: host compilers, Python and git. No serial port, no network transport, no hardware.
Evidence regeneration order (before this script): geometry_provenance.py, root_cause.py, alternatives.py,
run_lifecycles.sh, run_convergence.sh. Then: validate.py -> build_report.py -> artifact_manifest.py -> artifact_manifest.py --check.
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
OUT = ROOT / '09_Logs/Validation_Reports/G41_Contact_Reconciliation'
FW = ROOT / '05_Firmware/MATDOG_Controller'
TESTS = FW / 'scripts/tests'
PY = sys.executable
CXX = os.environ.get('CXX', 'g++')
G4_HEAD = '3ce85f92e2b5381b65e327dd371b2f164308f326'
BASELINE = 'c4befbe90b3121d60ba1b9ba09dc1082364c8d88'
SUMMARY = re.compile(r'(GAIT_HOST|CONTACT_MODE_HOST|STATIC_AUDIT|REPLAY_COMPARE|ARTIFACT_MANIFEST|Ran \d+ tests|^OK|PASS|passed|"status")', re.I)
REPORT_TOOLING = ('validate.py', 'artifact_manifest.py', 'build_report.py')


def sha_bytes(data):
    return hashlib.sha256(data).hexdigest()


def source_digest():
    audited = [p for p in HERE.glob('*') if p.is_file() and p.suffix in ('.py', '.sh', '.md') and p.name not in REPORT_TOOLING]
    paths = sorted(audited + list((FW / 'src/motion').glob('*')) + [p for p in TESTS.iterdir() if p.is_file()] + [FW / 'scripts/static_audit.py'])
    lines = [f'{p.relative_to(ROOT)} {sha_bytes(p.read_bytes())}' for p in paths]
    return sha_bytes('\n'.join(lines).encode()), len(lines)


def run(name, cmd, cwd=ROOT, env=None, timeout=7200, expect=None):
    start = time.time()
    proc = subprocess.run(cmd, cwd=cwd, env={**os.environ, **(env or {})}, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
    lines = [l for l in proc.stdout.splitlines() if l.strip()]
    ran = [l for l in lines if re.match(r'Ran \d+ tests?', l)]
    matches = [l for l in lines if SUMMARY.search(l)]
    summary = (ran[-1] + '; ' + lines[-1]) if ran else (matches or lines or [''])[-1].strip()
    passed = proc.returncode == 0 and (expect is None or expect in proc.stdout)
    gate = {'name': name, 'command': ' '.join(str(c) for c in cmd), 'returncode': proc.returncode, 'passed': passed, 'seconds': round(time.time() - start, 1),
            'summary': summary[:300], 'output_sha256': sha_bytes(proc.stdout.encode()), 'output_lines': len(lines)}
    print(f"{'PASS' if passed else 'FAIL'} {name} ({gate['seconds']}s): {gate['summary']}", flush=True)
    return gate


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, stdout=subprocess.PIPE, text=True, check=True).stdout


def main():
    tmp = Path(tempfile.mkdtemp(prefix='matdog-g41-validate-'))
    head = git('rev-parse', 'HEAD').strip()
    dirty = git('status', '--porcelain').strip()
    venv_bin = str(Path(PY).parent)
    env = {'OPENBLAS_NUM_THREADS': '1', 'PATH': venv_bin + os.pathsep + os.environ['PATH']}
    gates = []
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-fno-exceptions', '-fno-rtti']
    motion = [str(p) for p in sorted((FW / 'src/motion').glob('*.cpp'))]
    for t in ('test_gait', 'test_contact_mode'):
        gates.append(run(f'{t}_build_strict_cpp17', [CXX, *flags, '-O1', str(TESTS / f'{t}.cpp'), *motion, '-o', str(tmp / t)]))
        gates.append(run(f'{t}_run', [str(tmp / t)], expect='= PASS'))
    gates.append(run('g4_independent_oracle_current_source', [PY, str(HERE / 'run_g4_oracle.py')], cwd=HERE, env=env, expect='"status": "PASS"'))
    gates.append(run('g4_artifact_tests', [PY, str(ROOT / '06_Software/Matdog_Core/gait_audit/test_gait_audit.py')], cwd=ROOT / '06_Software/Matdog_Core/gait_audit', env=env))
    gates.append(run('g4_artifact_manifest_check', [PY, str(ROOT / '06_Software/Matdog_Core/gait_audit/artifact_manifest.py'), '--check'], env=env, expect='ARTIFACT_MANIFEST OK'))
    gates.append(run('g35_pose_audit_tests', [PY, str(ROOT / '06_Software/Matdog_Core/pose_audit/test_pose_audit.py')], cwd=ROOT / '06_Software/Matdog_Core/pose_audit', env=env))
    gates.append(run('g35_artifact_manifest_check', [PY, str(ROOT / '06_Software/Matdog_Core/pose_audit/artifact_manifest.py'), '--check'], env=env, expect='ARTIFACT_MANIFEST OK'))
    gates.append(run('geometry_provenance_recheck', [PY, str(HERE / 'geometry_provenance.py')], cwd=HERE, env=env, expect='"geometry_consistent": true'))
    gates.append(run('g41_contact_tests', [PY, str(HERE / 'test_contact_audit.py')], cwd=HERE, env=env))
    gates.append(run('lifecycle_replay_bit_identical_2ms', [PY, str(HERE / 'replay_compare.py')], cwd=HERE, env=env, expect='REPLAY_COMPARE PASS'))
    gates.append(run('host_motion_runner_g1_g2_g3_g35_g4_g41', ['bash', str(TESTS / 'run_motion_host_tests.sh')], env=env))
    gates.append(run('static_audit', [PY, str(FW / 'scripts/static_audit.py')], env=env, expect='STATIC_AUDIT = PASS'))
    san = [CXX, *flags, '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    for t in ('test_gait', 'test_contact_mode'):
        gates.append(run(f'{t}_asan_ubsan_build', [*san, str(TESTS / f'{t}.cpp'), *motion, '-o', str(tmp / (t + '_san'))]))
        gates.append(run(f'{t}_asan_ubsan_run', [str(tmp / (t + '_san'))], env={'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1', 'UBSAN_OPTIONS': 'halt_on_error=1:print_stacktrace=1'}, expect='= PASS'))
    gates.append(run('git_diff_check_worktree', ['git', 'diff', '--check']))
    gates.append(run('git_diff_check_vs_g4_head', ['git', '-c', 'core.whitespace=cr-at-eol', 'diff', '--check', G4_HEAD + '..HEAD']))
    # contract preservation: only the intended additions relative to the accepted G4 head
    names = [l for l in git('diff', '--name-status', G4_HEAD + '..HEAD').split('\n') if l]
    allowed_added = re.compile(r'^(05_Firmware/MATDOG_Controller/(src/motion/ContactMode\.(h|cpp)|scripts/tests/test_contact_mode\.cpp)|06_Software/Matdog_Core/contact_audit/.*|09_Logs/Validation_Reports/G41_Contact_Reconciliation/.*)$')
    allowed_modified = {'05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh', '05_Firmware/MATDOG_Controller/scripts/tests/test_motion_oracle.py'}  # host oracle include allowlist gains ContactMode.h, as G4 did for Gait.h/Locomotion.h
    bad = []
    for l in names:
        status, path = l.split('\t', 1)
        if not ((status == 'A' and allowed_added.match(path)) or (status == 'M' and (path in allowed_modified or allowed_added.match(path)))):
            bad.append(l)
    gate = {'name': 'accepted_g1_g4_files_unchanged_vs_g4_head', 'command': f'git diff --name-status {G4_HEAD}..HEAD', 'returncode': 0, 'passed': not bad,
            'summary': 'only G4.1 additions, the host-runner line and the host-oracle include allowlist changed; G1/G2/G3/G3.5/G4 sources, geometry and evidence untouched', 'unexpected_changes': bad}
    print(('PASS ' if gate['passed'] else 'FAIL ') + gate['name'], bad[:5])
    gates.append(gate)
    digest, count = source_digest()
    results = {'all_passed': all(g['passed'] for g in gates), 'gates': gates,
               'git': {'head_at_validation': head, 'worktree_clean_before_validation': not dirty, 'g4_head': G4_HEAD, 'baseline': BASELINE},
               'scope': 'offline software gates only: host compiler, Python, git; no hardware, serial device, actuator bus, servo command, flashing, persistent-memory write or calibration change was used',
               'source_digest': digest, 'source_digest_file_count': count, 'provenance': {'generator_sha256': sha_bytes(Path(__file__).read_bytes())}}
    (OUT / 'validation_results.json').write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
    print('ALL_PASSED' if results['all_passed'] else 'SOME_FAILED')
    return 0 if results['all_passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
