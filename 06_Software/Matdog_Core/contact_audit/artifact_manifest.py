"""Index G4.1 artifacts and cross-check provenance. Standard library only.

`--check` fails on a stale index, an artifact whose producer tool changed after the evidence was written is reported
(HISTORICAL_GENERATOR_REVISION) but does not fail, a changed canonical geometry source fails, and any difference in the
accepted G4 final evidence (REPORT.md, validation_results.json, artifact_manifest.json) fails.
"""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G41_Contact_Reconciliation'
G4 = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
MANIFEST = OUT / 'artifact_manifest.json'
G4_HEAD = '3ce85f92e2b5381b65e327dd371b2f164308f326'
PRODUCER = {'geometry_provenance': 'geometry_provenance.py', 'root_cause': 'root_cause.py', 'alternatives': 'alternatives.py', 'oracle_results_g41': 'run_g4_oracle.py',
            'validation_results': 'validate.py', 'REPORT': 'build_report.py'}
CANONICAL = ['03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf', '03_CAD/URDF/matt_robodog_rev00/meshes/collision/lf_foot_link.stl',
             '03_CAD/URDF/matt_robodog_rev00/meshes/collision/rf_foot_link.stl', '03_CAD/URDF/matt_robodog_rev00/meshes/lf_foot_link.stl',
             '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml', '05_Firmware/MATDOG_Controller/src/motion/FootContactData.h',
             '05_Firmware/MATDOG_Controller/src/motion/Gait.cpp', '05_Firmware/MATDOG_Controller/src/motion/Gait.h']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def producer(p):
    rel = p.relative_to(OUT)
    if p.name.startswith('lifecycle_'):
        return 'lifecycle_v2.py' if p.suffix in ('.json', '.csv') else None
    return PRODUCER.get(p.stem)


def g4_blob_sha(rel):
    out = subprocess.run(['git', 'show', f'{G4_HEAD}:{rel}'], cwd=ROOT, capture_output=True)
    return hashlib.sha256(out.stdout).hexdigest() if out.returncode == 0 else None


def build():
    files = sorted(p for p in OUT.rglob('*') if p.is_file() and p != MANIFEST)
    artifacts = [{'path': str(p.relative_to(OUT)), 'sha256': sha(p), 'bytes': p.stat().st_size, 'producer': producer(p)} for p in files]
    gen = {n: sha(HERE / n) for n in sorted(set(a['producer'] for a in artifacts if a['producer'])) if (HERE / n).exists()}
    g4 = []
    for name in ('REPORT.md', 'validation_results.json', 'artifact_manifest.json', 'full_cases.json', 'representatives.json'):
        rel = f'09_Logs/Validation_Reports/G4_Gait_Envelope/{name}'
        g4.append({'path': rel, 'g4_final_head_sha256': g4_blob_sha(rel), 'current_sha256': sha(ROOT / rel), 'status': 'PRESERVED' if g4_blob_sha(rel) == sha(ROOT / rel) else 'CHANGED'})
    prov = json.loads((OUT / 'geometry_provenance.json').read_text())
    recorded = {'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf': prov['urdf_sha256']}
    canon = []
    for rel in CANONICAL:
        now = sha(ROOT / rel)
        canon.append({'path': rel, 'current_sha256': now, 'g4_final_head_sha256': g4_blob_sha(rel), 'status': 'MATCH' if g4_blob_sha(rel) == now else 'CHANGED'})
    return {'scope': 'G4.1 offline contact reconciliation; no hardware artifact, serial log, or actuator command is indexed', 'excluded': ['artifact_manifest.json (self)'],
            'artifact_count': len(artifacts), 'artifacts': artifacts, 'generator_sha256': gen, 'g4_preservation': g4, 'canonical_source_checks': canon,
            'summary': {'changed_g4_evidence': [x['path'] for x in g4 if x['status'] != 'PRESERVED'], 'changed_canonical_sources': [x['path'] for x in canon if x['status'] != 'MATCH'],
                        'urdf_matches_provenance': recorded['03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'] == sha(ROOT / '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf')}}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--check', action='store_true'); args = ap.parse_args()
    manifest = build(); s = manifest['summary']
    bad = s['changed_g4_evidence'] or s['changed_canonical_sources'] or not s['urdf_matches_provenance']
    if args.check:
        if not MANIFEST.exists():
            print('ARTIFACT_MANIFEST missing'); return 1
        if json.loads(MANIFEST.read_text()) != manifest:
            print('ARTIFACT_MANIFEST stale or artifact changed'); return 1
        if bad:
            print('ARTIFACT_MANIFEST provenance failure', json.dumps(s)); return 1
        print('ARTIFACT_MANIFEST OK %d artifacts' % manifest['artifact_count']); return 0
    if bad:
        print('Refusing to write: provenance failure', json.dumps(s)); return 1
    MANIFEST.write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    print('ARTIFACT_MANIFEST written: %d artifacts' % manifest['artifact_count']); return 0


if __name__ == '__main__':
    sys.exit(main())
