"""Select representatives only from cases that pass every recorded gate.

FULL_MESH_VALIDATED here is the strongest tier this study can offer, all required:
  R1  a new 400-interval full-mesh record passes the independent audit predicate (audit_gate.py);
  R2  no sub -1 um mesh height in the continuous stance-pitch sweep of that record;
  R3  the same parameters pass the ground/support screen at 1600 intervals (robust_search.json).
The 100 mm checkpoint WALK candidate (id287) passes R1 and R2 but fails the 800/1600-interval
screen with band-only (no penetration) failures; it is kept as a labelled, density-fragile case.
Raw artifacts are read, never edited. A failed candidate (e.g. TROT id1311) is never listed.
"""
import json
import sys
import audit_gate
from survey import OUT, save  # before contact_discrepancy: it prepends pose_audit/, which has its own survey.py
from contact_discrepancy import Model, Foot, stance_pitch_runs, pitch_interval_min_delta_um


ROBUST = 'FULL_MESH_VALIDATED'
FRAGILE = 'FULL_MESH_SAMPLED_PASS_DENSITY_FRAGILE'
# name -> (artifact holding the dense full-mesh record, case id in that artifact, expected tier, rationale)
CHOSEN = {
    'walk': ('dense_robust_full.json', 357, ROBUST, '80 mm WALK, +10 mm/cycle, 10 mm lift, duty 0.8, 4 mm X sway; passes the ground/support screen up to 1600 intervals.'),
    'walk_100mm': ('dense_representatives.json', 287, FRAGILE, 'Checkpoint 100 mm WALK candidate: full-mesh pass at 401 frames, but the ground screen fails at 800/1600 intervals (undeclared-contact band only, no penetration).'),
    'trot': ('dense_robust_full.json', 61, ROBUST, '80 mm TROT, +20 mm/cycle, 10 mm lift, duty 0.8; passes the ground/support screen up to 1600 intervals. Dynamics NOT YET PROVEN.'),
    'trot_120mm': ('dense_robust_full.json', 309, ROBUST, '120 mm TROT, +20 mm/cycle, 10 mm lift, duty 0.7; passes the ground/support screen up to 1600 intervals. Dynamics NOT YET PROVEN.'),
}


def main():
    model = Model(); foot = Foot(model)
    search = json.loads((OUT / 'robust_search.json').read_text())
    robust_ids = set(search['robust_ids'])
    reps = []
    for name, (src, i, tier, why) in CHOSEN.items():
        doc = json.loads((OUT / src).read_text())
        assert doc['complete']
        case = next(c for c in doc['cases'] if c['id'] == i)
        audit = audit_gate.audit_case(src, case)
        sweep = pitch_interval_min_delta_um(foot, stance_pitch_runs(model, case))
        r1 = audit['FULL_MESH_VALIDATED']; r2 = sweep >= -1.0
        r3 = src == 'dense_robust_full.json' and i in robust_ids
        assert r1 and r2, (name, audit['failures'][:3], sweep)
        assert r3 == (tier == ROBUST), (name, 'tier mismatch')
        reps.append(dict(name=name, id=i, type=audit['type'], status=tier, scope='SAMPLED_FINITE_SET', parameters=audit['parameters'],
                         frames_source=src, dense_intervals=case['resolution_intervals'], dense_frames=len(case['frames']), period_s=1.0,
                         gates=dict(R1_full_mesh_audit=r1, R2_pitch_sweep_min_delta_um=sweep, R3_ground_screen_1600=r3),
                         recomputed_metrics=audit['recomputed'], dynamic_stability=case['dynamic_stability'], rationale=why))
    save('representatives.json', dict(
        note='FULL_MESH_VALIDATED means a finite sampled set passed every recorded gate; it is not a continuous or physical limit. TROT dynamics are NOT YET PROVEN. TROT id1311 FAILED dense validation and is not a representative.',
        representatives=reps))
    print(json.dumps([(r['name'], r['id'], r['status']) for r in reps]))


if __name__ == '__main__':
    sys.exit(main())
