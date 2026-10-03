"""Index G5-A artifacts and cross-check provenance. Standard library only. `--check` fails on a stale index, a changed accepted
G4/G4.1 evidence file, or a changed canonical geometry source."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'
MANIFEST = OUT / 'artifact_manifest.json'
G41_HEAD = '3974e9e213555285612cd29e373751d1dd862225'
PRODUCER = {'provenance': 'provenance.py', 'frames_oracle': 'independent_checks.py', 'ik_oracle': 'independent_checks.py', 'perturbation_results': 'perturbation_sim.py',
            'stability_assessment': 'stability_actuator.py', 'actuator_requirements': 'stability_actuator.py', 'joint_range': 'joint_range.py', 'actuator_feasibility': 'feasibility_table.py',
            'oracle_results_g5a': 'run_g4_oracle.py', 'validation_results': 'validate.py', 'REPORT': 'build_report.py'}
ACCEPTED = ['09_Logs/Validation_Reports/G4_Gait_Envelope/REPORT.md', '09_Logs/Validation_Reports/G4_Gait_Envelope/artifact_manifest.json', '09_Logs/Validation_Reports/G41_Contact_Reconciliation/REPORT.md',
            '09_Logs/Validation_Reports/G41_Contact_Reconciliation/artifact_manifest.json', '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf', '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml',
            '05_Firmware/MATDOG_Controller/src/motion/Gait.cpp', '05_Firmware/MATDOG_Controller/src/motion/FootContact.cpp', '05_Firmware/MATDOG_Controller/src/motion/BodyPose.cpp']


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def head_sha(rel):
    o = subprocess.run(['git', 'show', f'{G41_HEAD}:{rel}'], cwd=ROOT, capture_output=True)
    return hashlib.sha256(o.stdout).hexdigest() if o.returncode == 0 else None


def build():
    files = sorted(p for p in OUT.rglob('*') if p.is_file() and p != MANIFEST)
    arts = [dict(path=str(p.relative_to(OUT)), sha256=sha(p), bytes=p.stat().st_size, producer=PRODUCER.get(p.stem)) for p in files]
    gen = {n: sha(HERE / n) for n in sorted({a['producer'] for a in arts if a['producer'] and (HERE / a['producer']).exists()})}
    acc = [dict(path=r, g41_head_sha256=head_sha(r), current_sha256=sha(ROOT / r), status='PRESERVED' if head_sha(r) == sha(ROOT / r) else 'CHANGED') for r in ACCEPTED]
    return dict(scope='G5-A offline stabilization and actuator feasibility; no hardware artifact, serial log or actuator command is indexed', excluded=['artifact_manifest.json (self)'],
                artifact_count=len(arts), artifacts=arts, generator_sha256=gen, accepted_preservation=acc, summary=dict(changed_accepted=[a['path'] for a in acc if a['status'] != 'PRESERVED']))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--check', action='store_true'); a = ap.parse_args()
    m = build(); bad = m['summary']['changed_accepted']
    if a.check:
        if not MANIFEST.exists(): print('ARTIFACT_MANIFEST missing'); return 1
        if json.loads(MANIFEST.read_text()) != m: print('ARTIFACT_MANIFEST stale or artifact changed'); return 1
        if bad: print('ARTIFACT_MANIFEST provenance failure', bad); return 1
        print('ARTIFACT_MANIFEST OK %d artifacts' % m['artifact_count']); return 0
    if bad: print('Refusing to write: accepted evidence changed', bad); return 1
    MANIFEST.write_text(json.dumps(m, indent=2, allow_nan=False) + '\n'); print('ARTIFACT_MANIFEST written: %d artifacts' % m['artifact_count']); return 0


if __name__ == '__main__':
    sys.exit(main())
