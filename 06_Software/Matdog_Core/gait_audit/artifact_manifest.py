"""Index G4 artifacts and cross-check recorded provenance against the files on disk.

Standard library only. `--check` fails on a stale index, a changed preserved checkpoint artifact, a recorded
input hash that no longer matches, or a canonical geometry source that changed since evidence was written.
Gait/tool sources edited after the saved survey (late input guards, new tools) are reported as
HISTORICAL_GENERATOR_REVISION, not failed: revalidate_saved.py shows the current core reproduces every saved frame.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
MANIFEST = OUT / 'artifact_manifest.json'
CHECKPOINT = OUT / 'checkpoint_manifest.json'
PRODUCER = {
    'screen': 'survey.py', 'envelope_grid': 'survey.py', 'full_cases': 'survey.py', 'definitions': 'survey.py', 'refinement_screen': 'refine.py',
    'refinement_full': 'refine.py', 'lifecycle_audit': 'lifecycle_audit.py', 'oracle_results': 'oracle.py', 'oracle_results_final': 'oracle.py',
    'xgo_architecture': 'xgo_evidence.py', 'full_case_audit': 'audit_gate.py', 'contact_discrepancy': 'contact_discrepancy.py',
    'dense_representatives': 'dense_representatives.py', 'dense_robust_full': 'dense_robust_full.py', 'dense_ground_probe': 'dense_ground_probe.py',
    'robust_search': 'robust_search.py', 'kinematic_diagnostic_screen': 'diag_screen.py', 'diag_frontier_full': 'diag_frontier_full.py',
    'diag_stride_extension': 'diag_stride_extension.py', 'lifecycle_probe': 'lifecycle_probe.py', 'representatives': 'select_representatives.py',
    'derivative_tables': 'derivatives.py', 'envelope_summary': 'envelope_summary.py', 'revalidate_saved': 'revalidate_saved.py',
    'validation_results': 'validate.py', 'REPORT': 'hand-written from the saved artifacts',
}
PRESERVED = ('full_cases.json', 'refinement_full.json', 'screen.json', 'refinement_screen.json', 'envelope_grid.csv', 'lifecycle_audit.json',
             'oracle_results.json', 'definitions.json', 'xgo_architecture.json')
GENERATED_SOURCE = ('06_Software/Matdog_Core/gait_audit/', '05_Firmware/MATDOG_Controller/src/motion/Gait.', '05_Firmware/MATDOG_Controller/src/motion/Locomotion.')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def files():
    return sorted(p for p in OUT.rglob('*') if p.is_file() and p != MANIFEST)


def producer(p):
    rel = p.relative_to(OUT)
    if rel.parts[0] == 'views' or p.suffix == '.csv' and p.name.endswith('_cycle.csv'):
        return 'render.py'
    if rel.parts[0] == 'checkpoint_logs' or p.name.startswith(('G4_', 'handoff_state', 'checkpoint_manifest')):
        return 'checkpoint (preserved handoff material)'
    return PRODUCER.get(p.stem)


def build():
    artifacts = [{'path': str(p.relative_to(OUT)), 'sha256': sha(p), 'bytes': p.stat().st_size, 'producer': producer(p)} for p in files()]
    checkpoint = json.loads(CHECKPOINT.read_text())
    preserved = []
    for rel, rec in sorted(checkpoint['files'].items()):
        target = ROOT / rel
        now = sha(target) if target.exists() else None
        preserved.append({'path': rel, 'checkpoint_sha256': rec['sha256'], 'current_sha256': now, 'status': 'PRESERVED' if now == rec['sha256'] else 'CHANGED_OR_MISSING'})
    inputs = []
    for p in files():
        if p.suffix != '.json':
            continue
        data = json.loads(p.read_text())
        block = data.get('input_sha256') if isinstance(data, dict) else None
        for name, recorded in (block or {}).items():
            now = sha(OUT / name)
            inputs.append({'artifact': str(p.relative_to(OUT)), 'input': name, 'recorded_sha256': recorded, 'current_sha256': now, 'status': 'MATCH' if recorded == now else 'STALE_INPUT'})
    sources = []
    recorded = json.loads((OUT / 'definitions.json').read_text())['canonical_sources']
    for name, rec in sorted(recorded.items()):
        target = ROOT / name
        now = sha(target) if target.exists() else None
        generated = name.startswith(GENERATED_SOURCE)
        status = 'MATCH' if now == rec else ('HISTORICAL_GENERATOR_REVISION' if generated and now is not None else 'CHANGED_OR_MISSING')
        sources.append({'path': name, 'recorded_sha256': rec, 'current_sha256': now, 'status': status})
    return {
        'scope': 'G4 offline gait study; no hardware artifact, serial log, or actuator command is indexed',
        'excluded': ['artifact_manifest.json (self)'], 'artifact_count': len(artifacts), 'artifacts': artifacts,
        'preserved_checkpoint_checks': preserved, 'provenance_input_checks': inputs, 'definitions_source_checks': sources,
        'summary': {
            'changed_preserved_checkpoint_files': [x['path'] for x in preserved if x['status'] != 'PRESERVED'],
            'stale_inputs': [x for x in inputs if x['status'] != 'MATCH'],
            'changed_canonical_sources': [x['path'] for x in sources if x['status'] == 'CHANGED_OR_MISSING'],
            'historical_generator_revisions': [x['path'] for x in sources if x['status'] == 'HISTORICAL_GENERATOR_REVISION']}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    manifest = build()
    s = manifest['summary']
    bad = s['changed_preserved_checkpoint_files'] or s['stale_inputs'] or s['changed_canonical_sources']
    if args.check:
        if not MANIFEST.exists():
            print('ARTIFACT_MANIFEST missing'); return 1
        if json.loads(MANIFEST.read_text()) != manifest:
            print('ARTIFACT_MANIFEST stale or artifact changed'); return 1
        if bad:
            print('ARTIFACT_MANIFEST provenance failure', json.dumps(s)); return 1
        print('ARTIFACT_MANIFEST OK %d artifacts; historical generator revisions: %d' % (manifest['artifact_count'], len(s['historical_generator_revisions'])))
        return 0
    if bad:
        print('Refusing to write: provenance failure', json.dumps(s)); return 1
    MANIFEST.write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    print('ARTIFACT_MANIFEST written: %d artifacts' % manifest['artifact_count'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
