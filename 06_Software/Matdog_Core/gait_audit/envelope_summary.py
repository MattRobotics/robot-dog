"""Derive the sampled gait envelope tables from the saved artifacts. Maximum TESTED passing values only.

Four evidence tiers are kept apart and never merged:
  STRICT_SAVED   independent audit pass of the preserved raw full-mesh sets (audit_gate.py)
  STRICT_ROBUST  full-mesh pass at 400 intervals, ground screen pass at 1600 intervals (robust_search.json)
  DIAGNOSTIC     micrometre foot_link events quarantined (diag_screen.py, diag_stride_extension.py); no acceptance
  SCREEN         the saved strict screens (screen.json, refinement_screen.json), early exit, no self-collision
"""
import json
import sys
from collections import Counter, defaultdict
from survey import OUT, save

NAMES = {0: 'WALK', 1: 'TROT'}
HEIGHTS = (.06, .08, .09, .1, .11, .12, .14, .15, .16)


def load(name):
    return json.loads((OUT / name).read_text())


def span(values):
    return None if not values else [min(values) * 1000, max(values) * 1000]


def main():
    audit = load('full_case_audit.json')
    sweep = {s['id']: s for s in load('contact_discrepancy.json')['steady_cases']}
    search = load('robust_search.json')
    diag = load('kinematic_diagnostic_screen.json')['cases'] + load('diag_stride_extension.json')['cases']
    strict_screens = load('screen.json')['cases'] + load('refinement_screen.json')['cases']
    pure = lambda p: p['advance_y_m'] == 0 and p['yaw_rad'] == 0

    table = {}
    for t in (0, 1):
        for h in HEIGHTS:
            key = f'{NAMES[t]}_{round(h * 1000)}mm'
            row = dict(type=NAMES[t], height_mm=round(h * 1000))
            sa = [a for a in audit['cases'] if a['parameters']['type'] == t and a['parameters']['height_m'] == h]
            ok = [a for a in sa if a['FULL_MESH_VALIDATED']]
            row['strict_saved'] = dict(cases=len(sa), passing_ids=[a['id'] for a in ok],
                                       passing_x_mm=sorted(a['parameters']['advance_x_m'] * 1000 for a in ok),
                                       passing_and_pitch_sweep_ok_ids=[a['id'] for a in ok if sweep[a['id']]['pitch_interval_check'] == 'OK'])
            rb = [search['cases'][i] for i in search['robust_ids'] if search['cases'][i]['parameters']['type'] == t and abs(search['cases'][i]['parameters']['height_m'] - h) < 1e-9]
            row['strict_robust'] = dict(survivor_count=len(rb), x_mm=sorted(c['parameters']['advance_x_m'] * 1000 for c in rb),
                                        lifts_mm=sorted({c['parameters']['lift_m'] * 1000 for c in rb}), duties=sorted({c['parameters']['duty'] for c in rb}))
            sc = [r for r in strict_screens if r['parameters']['type'] == t and r['parameters']['height_m'] == h and pure(r['parameters'])]
            row['strict_screen'] = dict(cases=len(sc), passing=sum(r['complete'] for r in sc),
                                        passing_x_mm_span=span([r['parameters']['advance_x_m'] for r in sc if r['complete']]))
            dg = [r for r in diag if r['parameters']['type'] == t and r['parameters']['height_m'] == h and pure(r['parameters'])]
            for sign, label in ((1, 'forward'), (-1, 'backward')):
                grp = sorted((r for r in dg if sign * r['parameters']['advance_x_m'] > 0), key=lambda r: abs(r['parameters']['advance_x_m']))
                passing = [abs(r['parameters']['advance_x_m']) for r in grp if r['kinematic_diagnostic_pass']]
                ext = sorted((r for r in grp if r['id'] >= 5000), key=lambda r: abs(r['parameters']['advance_x_m']))
                last_ok = max((abs(r['parameters']['advance_x_m']) for r in ext if r['kinematic_diagnostic_pass']), default=None)
                first_block = next((r for r in ext if not r['kinematic_diagnostic_pass']), None)
                row.setdefault('diagnostic_' + label, dict(
                    max_tested_passing_abs_x_mm=max(passing) * 1000 if passing else None, cases=len(grp),
                    uniform_extension_last_pass_abs_x_mm=None if last_ok is None else last_ok * 1000,
                    uniform_extension_first_block_abs_x_mm=None if first_block is None else abs(first_block['parameters']['advance_x_m']) * 1000,
                    uniform_extension_first_block_categories=None if first_block is None else [c for c in first_block['categories'] if c != 'FOOT_MESH_MICROMETRE']))
            if sa or rb or sc or dg:
                table[key] = row

    strict_first = Counter()
    for r in strict_screens:
        if r['complete']:
            continue
        kinds = set()
        for e in r['first_failure']['errors']:
            kind, _, link = e.partition(':')
            kinds.add(('FOOT_LINK_' if link.endswith('_foot_link') else 'NON_FOOT_' if link else '') + kind)
        strict_first[' + '.join(sorted(kinds))] += 1
    lateral = [r for r in diag if not pure(r['parameters']) and r['id'] < 5000]
    lat_cats = Counter(c for r in lateral for c in r['categories'])
    result = dict(
        scope='Maximum TESTED passing values on finite sampled grids. Not physical maxima, not global limits, not a continuous valid box.',
        tiers=__doc__.strip().splitlines()[2:6],
        by_type_height=table,
        strict_screen_first_failure_composition=dict(strict_first),
        strict_screen_total=len(strict_screens), strict_screen_passing=sum(r['complete'] for r in strict_screens),
        diagnostic_screen_total=len([r for r in diag if r['id'] < 5000]), diagnostic_screen_passing=sum(r['kinematic_diagnostic_pass'] for r in diag if r['id'] < 5000),
        max_pure_fore_aft_quarantined_foot_penetration_um=max(r['max_foot_penetration_um'] for r in diag if pure(r['parameters'])),
        lateral_yaw=dict(
            cases=len(lateral), strict_screen_passing_nonzero_lateral_or_yaw=sum(1 for r in strict_screens if not pure(r['parameters']) and r['complete']),
            diagnostic_passing=sum(r['kinematic_diagnostic_pass'] for r in lateral),
            diagnostic_category_counts=dict(lat_cats),
            g2_contact_status_note='all 255 diagnostic CONTACT_INVALID-only lateral/yaw cases are solveGait CONTACT_INVALID, i.e. the unchanged G2 nominal-strip contact tilt validity'))
    save('envelope_summary.json', result)
    for k, v in table.items():
        print(k, '| strict saved ok', v['strict_saved']['passing_ids'], '| robust', v['strict_robust']['survivor_count'],
              '| diag fwd/back max', v['diagnostic_forward']['max_tested_passing_abs_x_mm'], v['diagnostic_backward']['max_tested_passing_abs_x_mm'])


if __name__ == '__main__':
    sys.exit(main())
