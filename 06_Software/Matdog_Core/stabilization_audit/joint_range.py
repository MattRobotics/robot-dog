"""Required joint ranges of STAND, tilt-compensated STAND and the G4 WALK/TROT lifecycles against the measured mechanical contacts
of the TRUE Full Calibration (hardware, 2026-10-01) and the URDF limits.

The calibration contacts are measurements of the installed hardware (RAM-only calibration; no operational envelope approved).
A positive margin means the semantic joint angle stays inside the measured contact range; it does not approve an envelope.
"""
import json
import sys
import numpy as np
from core import Lib, OUT, ROOT, LEGS
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/gait_audit'))
import importlib.util
_spec = importlib.util.spec_from_file_location('gait_core', ROOT / '06_Software/Matdog_Core/gait_audit/core.py')
gait_core = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(gait_core)

DEG = np.pi / 180
JOINTS = ('hip', 'upper', 'lower')
CASES = {
    'WALK_357': gait_core.params(kind=0, height=.08, lift=.01, duty=.8, x=.01, sway=.004),
    'WALK_287': gait_core.params(kind=0, height=.10, lift=.01, duty=.8, x=.01, sway=.004),
    'TROT_61': gait_core.params(kind=1, height=.08, lift=.01, duty=.8, x=.02),
    'TROT_309': gait_core.params(kind=1, height=.12, lift=.01, duty=.7, x=.02),
}


def main():
    prov = json.loads((OUT / 'provenance.json').read_text())
    meas = {(c['leg'], c['joint'], c['side']): c for c in prov['full_calibration']['contacts']}
    lib = Lib(); core = gait_core.Core()
    contacts, seeds, h, low = lib.stand()
    ranges = {}

    def add(name, qs):
        qs = np.asarray(qs).reshape(-1, 12) / DEG
        ranges[name] = dict(min_deg=qs.min(axis=0).tolist(), max_deg=qs.max(axis=0).tolist())

    # STAND rise (G3 contact-locked path 0.100 -> 0.150 m)
    s, qs = seeds.copy(), []
    for p in np.linspace(1, 0, 201):
        st, j, _ = lib.stand_path(p, s); assert st == 0; s = j; qs.append(j.ravel())
    add('STAND_RISE_0.100_to_0.150', qs)
    for amp in (3, 5):
        qs = []
        for r, p in [(a, b) for a in (-amp, 0, amp) for b in (-amp, 0, amp)]:
            c = lib.compensate(r * DEG, p * DEG)
            if c['status'] == 'OK': qs.append(c['joints'].ravel())
        add(f'STAND_TILT_COMPENSATED_+-{amp}deg', qs)
    for name, p in CASES.items():
        frames = gait_core.lifecycle(core, p, np.round(np.arange(0, 7.0 + 1e-9, 0.01), 9))
        add(f'LIFECYCLE_{name}', [f['q'] for f in frames])
    table = {}
    for name, r in ranges.items():
        rows = []
        for i, leg in enumerate(LEGS):
            for k, jn in enumerate(JOINTS):
                lo, hi = r['min_deg'][3 * i + k], r['max_deg'][3 * i + k]
                mn, mx = meas[(leg, jn, 'MIN')], meas[(leg, jn, 'MAX')]
                rows.append(dict(leg=leg, joint=jn, required_min_deg=lo, required_max_deg=hi, measured_contact_min_deg=mn['measured_contact_deg'], measured_contact_max_deg=mx['measured_contact_deg'],
                                 urdf_min_deg=mn['urdf_limit_deg'], urdf_max_deg=mx['urdf_limit_deg'],
                                 margin_to_measured_min_deg=lo - mn['measured_contact_deg'], margin_to_measured_max_deg=mx['measured_contact_deg'] - hi,
                                 margin_to_urdf_min_deg=lo - mn['urdf_limit_deg'], margin_to_urdf_max_deg=mx['urdf_limit_deg'] - hi))
        worst = min(rows, key=lambda x: min(x['margin_to_measured_min_deg'], x['margin_to_measured_max_deg']))
        table[name] = dict(rows=rows, worst_margin_to_measured_deg=min(worst['margin_to_measured_min_deg'], worst['margin_to_measured_max_deg']), worst_joint=f"{worst['leg']}_{worst['joint']}",
                           worst_margin_to_urdf_deg=min(min(x['margin_to_urdf_min_deg'], x['margin_to_urdf_max_deg']) for x in rows),
                           all_inside_measured_contacts=all(min(x['margin_to_measured_min_deg'], x['margin_to_measured_max_deg']) > 0 for x in rows))
    # tilt authority of the compensated STAND against measured contacts and against the URDF limits, versus body height
    lim_meas = np.array([[meas[(l, j, 'MIN')]['measured_contact_deg'], meas[(l, j, 'MAX')]['measured_contact_deg']] for l in LEGS for j in JOINTS])
    lim_urdf = np.array([[meas[(l, j, 'MIN')]['urdf_limit_deg'], meas[(l, j, 'MAX')]['urdf_limit_deg']] for l in LEGS for j in JOINTS])
    authority = {}
    for height in (0.15, 0.14, 0.13, 0.12, 0.10):
        row = {}
        for axis in ('roll', 'pitch'):
            for sign in (1, -1):
                best = {'measured': 0.0, 'urdf': 0.0, 'ik': 0.0}
                for a in np.arange(0.25, 20.01, 0.25):
                    r, p = (sign * a * DEG, 0.0) if axis == 'roll' else (0.0, sign * a * DEG)
                    c = lib.compensate(r, p, height=height)
                    if c['status'] != 'OK':
                        break
                    q = c['joints'].ravel() / DEG
                    best['ik'] = a
                    if np.all(q >= lim_urdf[:, 0]) and np.all(q <= lim_urdf[:, 1]): best['urdf'] = a
                    if np.all(q >= lim_meas[:, 0]) and np.all(q <= lim_meas[:, 1]): best['measured'] = a
                row[f'{axis}{"+" if sign > 0 else "-"}'] = best
        authority[f'{height:.2f}'] = row
    (OUT / 'joint_range.json').write_text(json.dumps(dict(
        tilt_authority_deg_by_body_height=authority,
        scope='Semantic joint angles (URDF degrees) against measured mechanical contacts of the installed robot; a positive margin is not an approved envelope',
        source_report=prov['full_calibration']['report_path'], source_ref=prov['full_calibration']['source_ref'], source_report_sha256=prov['full_calibration']['report_sha256'], cases=table), indent=2) + '\n')
    for hgt, row in authority.items():
        print('height', hgt, {k: (v['measured'], v['urdf'], v['ik']) for k, v in row.items()})
    for n, t in table.items():
        print('%-34s worst margin to measured contact %.2f deg (%s) | to URDF %.2f deg | inside %s' % (n, t['worst_margin_to_measured_deg'], t['worst_joint'], t['worst_margin_to_urdf_deg'], t['all_inside_measured_contacts']))


if __name__ == '__main__':
    sys.exit(main())
