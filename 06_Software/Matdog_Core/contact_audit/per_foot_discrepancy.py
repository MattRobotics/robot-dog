"""Per-foot analytic-reference versus collision-mesh discrepancy over the four G4.1 lifecycle motions (5 ms).

The four feet share one collision geometry in foot_link, so their discrepancy differs only through each foot's pitch:
front feet sit near -43..-21 deg and rear feet near -51..-33 deg during the lifecycle.
"""
import sys
import numpy as np
from common import UM, LEGS, save, delta_um
from core import lifecycle
from survey import FIELDS  # noqa: F401  gait_audit survey first
import lifecycle_v2 as lc
from alternatives import foot_pitches


def main():
    ctx = lc.Context()
    nom = ctx.nominal
    tread = ctx.treads['nominal'].all
    out = {}
    for name, p in lc.CASES.items():
        times = np.round(np.arange(0, 7.0 + 1e-9, 0.005), 9)
        frames = lifecycle(ctx.core, p, times)
        pit = foot_pitches(ctx, frames)
        stance = np.array([[f['leg_phase'][i][1] != 0 or f['leg_phase'][i][2] != 0 for i in range(4)] for f in frames])
        row = {}
        for i, leg in enumerate(LEGS):
            d = delta_um(tread, nom.center, nom.radius, pit[:, i])
            row[leg] = dict(pitch_deg=[float(pit[:, i].min()), float(pit[:, i].max())], mesh_minus_analytic_um_all=[float(d.min()), float(d.max())],
                            mesh_minus_analytic_um_stance=[float(d[stance[:, i]].min()), float(d[stance[:, i]].max())], samples_below_minus_1um=int((d < -1).sum()))
        out[name] = row
    save('per_foot_discrepancy.json', dict(
        purpose='Per-foot mesh-minus-analytic height over each G4.1 lifecycle (unchanged G2 reference, collision mesh)',
        note='identical collision geometry in foot_link; differences between feet are only the pitch they pass through', cases=out))
    for n, r in out.items():
        print(n, {l: (round(v['mesh_minus_analytic_um_stance'][0], 3), round(v['mesh_minus_analytic_um_stance'][1], 3)) for l, v in r.items()})


if __name__ == '__main__':
    sys.exit(main())
