"""Run every offline G5-A validation gate on the CURRENT source and record validation_results.json (software only: no hardware).

Evidence regeneration order (before this script, from committed sources): provenance.py, independent_checks.py, perturbation_sim.py,
stability_actuator.py (~9 min), joint_range.py, feasibility_table.py. Then: validate.py -> build_report.py -> artifact_manifest.py
-> artifact_manifest.py --check.
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
OUT = ROOT / '09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'
FW = ROOT / '05_Firmware/MATDOG_Controller'
TESTS = FW / 'scripts/tests'
PY = sys.executable
CXX = os.environ.get('CXX', 'g++')
G41_HEAD = '3974e9e213555285612cd29e373751d1dd862225'
SUMMARY = re.compile(r'(GAIT_HOST|CONTACT_MODE_HOST|STABILIZATION_HOST|STATIC_AUDIT|ARTIFACT_MANIFEST|Ran \d+ tests|^OK|PASS|passed|"status")', re.I)
REPORT_TOOLING = ('validate.py', 'artifact_manifest.py', 'build_report.py')


def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def source_digest():
    audited = [p for p in HERE.glob('*') if p.is_file() and p.suffix in ('.py', '.cpp', '.md') and p.name not in REPORT_TOOLING]
    paths = sorted(audited + list((FW / 'src/motion').glob('*')) + [p for p in TESTS.iterdir() if p.is_file()] + [FW / 'scripts/static_audit.py'])
    lines = [f'{p.relative_to(ROOT)} {sha_bytes(p.read_bytes())}' for p in paths]
    return sha_bytes('\n'.join(lines).encode()), len(lines)


def run(name, cmd, cwd=ROOT, env=None, expect=None, timeout=7200):
    start = time.time()
    p = subprocess.run(cmd, cwd=cwd, env={**os.environ, **(env or {})}, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
    lines = [l for l in p.stdout.splitlines() if l.strip()]
    ran = [l for l in lines if re.match(r'Ran \d+ tests?', l)]
    matches = [l for l in lines if SUMMARY.search(l)]
    summary = (ran[-1] + '; ' + lines[-1]) if ran else (matches or lines or [''])[-1].strip()
    ok = p.returncode == 0 and (expect is None or expect in p.stdout)
    gate = dict(name=name, command=' '.join(map(str, cmd)), returncode=p.returncode, passed=ok, seconds=round(time.time() - start, 1), summary=summary[:300], output_sha256=sha_bytes(p.stdout.encode()), output_lines=len(lines))
    print(('PASS ' if ok else 'FAIL ') + f"{name} ({gate['seconds']}s): {gate['summary']}", flush=True)
    return gate


def git(*a):
    return subprocess.run(['git', *a], cwd=ROOT, stdout=subprocess.PIPE, text=True, check=True).stdout


def main():
    tmp = Path(tempfile.mkdtemp(prefix='matdog-g5a-validate-'))
    head = git('rev-parse', 'HEAD').strip(); dirty = git('status', '--porcelain').strip()
    env = {'OPENBLAS_NUM_THREADS': '1', 'PATH': str(Path(PY).parent) + os.pathsep + os.environ['PATH']}
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-fno-exceptions', '-fno-rtti']
    motion = [str(p) for p in sorted((FW / 'src/motion').glob('*.cpp'))]
    gates = []
    tests = ('test_gait', 'test_contact_mode', 'test_stabilization')
    for t in tests:
        gates.append(run(f'{t}_build_strict_cpp17', [CXX, *flags, '-O1', str(TESTS / f'{t}.cpp'), *motion, '-o', str(tmp / t)]))
        gates.append(run(f'{t}_run', [str(tmp / t)], expect='= PASS'))
    san = [CXX, *flags, '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    for t in tests:
        gates.append(run(f'{t}_asan_ubsan_build', [*san, str(TESTS / f'{t}.cpp'), *motion, '-o', str(tmp / (t + '_san'))]))
        gates.append(run(f'{t}_asan_ubsan_run', [str(tmp / (t + '_san'))], env={'ASAN_OPTIONS': 'detect_leaks=1:halt_on_error=1', 'UBSAN_OPTIONS': 'halt_on_error=1:print_stacktrace=1'}, expect='= PASS'))
    gates.append(run('g5a_tests', [PY, str(HERE / 'test_stabilization_audit.py')], cwd=HERE, env=env))
    # deterministic regeneration: the committed oracle/simulation evidence must be reproduced byte for byte
    gates.append(run('regenerate_independent_oracles', [PY, str(HERE / 'independent_checks.py')], cwd=HERE, env=env))
    gates.append(run('regenerate_loop_simulations', [PY, str(HERE / 'perturbation_sim.py')], cwd=HERE, env=env))
    gates.append(run('regenerated_evidence_identical_to_committed', ['git', 'diff', '--exit-code', '--stat', '--', str(OUT / 'frames_oracle.json'), str(OUT / 'ik_oracle.json'), str(OUT / 'perturbation_results.json')]))
    gates.append(run('g4_independent_oracle_current_source', [PY, str(HERE / 'run_g4_oracle.py')], cwd=HERE, env=env, expect='"status": "PASS"'))
    gates.append(run('g4_artifact_tests', [PY, str(ROOT / '06_Software/Matdog_Core/gait_audit/test_gait_audit.py')], cwd=ROOT / '06_Software/Matdog_Core/gait_audit', env=env))
    gates.append(run('g4_artifact_manifest_check', [PY, str(ROOT / '06_Software/Matdog_Core/gait_audit/artifact_manifest.py'), '--check'], env=env, expect='ARTIFACT_MANIFEST OK'))
    gates.append(run('g41_contact_tests', [PY, str(ROOT / '06_Software/Matdog_Core/contact_audit/test_contact_audit.py')], cwd=ROOT / '06_Software/Matdog_Core/contact_audit', env=env))
    gates.append(run('g41_artifact_manifest_check', [PY, str(ROOT / '06_Software/Matdog_Core/contact_audit/artifact_manifest.py'), '--check'], env=env, expect='ARTIFACT_MANIFEST OK'))
    gates.append(run('g35_pose_audit_tests', [PY, str(ROOT / '06_Software/Matdog_Core/pose_audit/test_pose_audit.py')], cwd=ROOT / '06_Software/Matdog_Core/pose_audit', env=env))
    gates.append(run('g35_artifact_manifest_check', [PY, str(ROOT / '06_Software/Matdog_Core/pose_audit/artifact_manifest.py'), '--check'], env=env, expect='ARTIFACT_MANIFEST OK'))
    gates.append(run('host_motion_runner_g1_to_g5a', ['bash', str(TESTS / 'run_motion_host_tests.sh')], env=env))
    gates.append(run('static_audit', [PY, str(FW / 'scripts/static_audit.py')], env=env, expect='STATIC_AUDIT = PASS'))
    gates.append(run('git_diff_check_worktree', ['git', 'diff', '--check']))
    gates.append(run('git_diff_check_vs_g41_head', ['git', '-c', 'core.whitespace=cr-at-eol', 'diff', '--check', G41_HEAD + '..HEAD']))
    allowed_added = re.compile(r'^(05_Firmware/MATDOG_Controller/(src/motion/(ImuAttitude|BodyStabilizer|TiltedContactIk|TiltCompensation|ActuatorEnvelope)\.(h|cpp)|src/motion/STABILIZATION\.md|scripts/tests/test_stabilization\.cpp)|06_Software/Matdog_Core/stabilization_audit/.*|09_Logs/Validation_Reports/G5A_Stabilization_Feasibility/.*)$')
    allowed_modified = {'05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh', '05_Firmware/MATDOG_Controller/scripts/tests/test_motion_oracle.py'}
    bad = []
    for l in [x for x in git('diff', '--name-status', G41_HEAD + '..HEAD').split('\n') if x]:
        st, path = l.split('\t', 1)
        if not ((st == 'A' and allowed_added.match(path)) or (st == 'M' and (path in allowed_modified or allowed_added.match(path)))):
            bad.append(l)
    g = dict(name='accepted_g1_g4_1_files_unchanged_vs_g41_head', command=f'git diff --name-status {G41_HEAD}..HEAD', returncode=0, passed=not bad,
             summary='only G5-A additions, the host-runner line and the host-oracle include allowlist changed; G1-G4.1 sources, geometry, calibration interfaces and evidence untouched', unexpected_changes=bad)
    print(('PASS ' if g['passed'] else 'FAIL ') + g['name'], bad[:5])
    gates.append(g)
    digest, count = source_digest()
    res = dict(all_passed=all(x['passed'] for x in gates), gates=gates, git=dict(head_at_validation=head, worktree_clean_before_validation=not dirty, g41_head=G41_HEAD),
               scope='offline software gates only: host compiler, Python, git; no hardware, serial device, actuator bus, servo command, flashing, persistent-memory write or calibration change was used',
               source_digest=digest, source_digest_file_count=count, provenance=dict(generator_sha256=sha_bytes(Path(__file__).read_bytes())))
    (OUT / 'validation_results.json').write_text(json.dumps(res, indent=2, allow_nan=False) + '\n')
    print('ALL_PASSED' if res['all_passed'] else 'SOME_FAILED')
    return 0 if res['all_passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
