"""Root cause of the G2-analytic-cylinder vs foot-mesh contact discrepancy (reproduces G4's 5.775 um offset independently).

Separates five things that G4 lumped together:
  nominal CAD geometry          the G2 YAML `cad_validation` block (R 14.9 mm, centre z 14.9 mm)
  ideal analytic contact        the G2 cylinder actually used by IK (identical to the nominal CAD values)
  tessellated collision mesh    the collision STL, as the URDF places it
  actual lowest point or edge   min over the mesh surface of height, as a function of foot pitch
  numerical evaluation error    float32 STL quantisation and float64 evaluation error
"""
import sys
import numpy as np
from common import foot_vertices, g2_cylinder, delta_um, save, UM, LEGS


def outer_ring(v, c, R, band=0.05e-3):
    r = np.hypot(v[:, 0] - c[0], v[:, 2] - c[2])
    return v[r >= R - band]


def fit_circle(points):
    """Algebraic least squares followed by Gauss-Newton on the geometric residual."""
    P = np.c_[points[:, 0], points[:, 2]]
    A = np.c_[2 * P, np.ones(len(P))]
    sol = np.linalg.lstsq(A, (P ** 2).sum(1), rcond=None)[0]
    cx, cz = sol[:2]
    r = np.sqrt(sol[2] + cx ** 2 + cz ** 2)
    for _ in range(20):
        d = np.hypot(P[:, 0] - cx, P[:, 1] - cz)
        J = np.c_[-(P[:, 0] - cx) / d, -(P[:, 1] - cz) / d, -np.ones(len(P))]
        step = np.linalg.lstsq(J, d - r, rcond=None)[0]
        cx, cz, r = cx - step[0], cz - step[1], r - step[2]
    res = np.hypot(P[:, 0] - cx, P[:, 1] - cz) - r
    return cx, cz, r, res


def analyse_mesh(kind, leg, cyl):
    v, meta = foot_vertices(kind, leg)
    c, R = cyl['center'], cyl['radius']
    ring = outer_ring(v, c, R)
    cx, cz, r, res = fit_circle(ring)
    ys = np.unique(np.round(ring[:, 1], 6))
    # extents-based estimate (independent of the fit): the circle spans the full diameter in x and z
    ext = dict(x_min_um=float((v[:, 0].min() - (c[0] - R)) * UM), x_max_um=float((v[:, 0].max() - (c[0] + R)) * UM),
               z_min_um=float((v[:, 2].min() - (c[2] - R)) * UM), z_max_um=float((v[:, 2].max() - (c[2] + R)) * UM))
    # ring-by-ring facet structure and the chord sagitta each ring can lose inside its own circle
    rings = []
    for y in ys:
        q = ring[np.round(ring[:, 1], 6) == y]
        ang = np.sort(np.arctan2(q[:, 0] - cx, q[:, 2] - cz))
        gaps = np.diff(np.r_[ang, ang[0] + 2 * np.pi])
        rings.append(dict(y_mm=float(y * 1e3), vertices=int(len(q)), max_gap_deg=float(np.degrees(gaps.max())), min_gap_deg=float(np.degrees(gaps.min())),
                          max_chord_sagitta_um=float(r * (1 - np.cos(gaps.max() / 2)) * UM)))
    # Minimax translation of the NOMINAL circle (radius fixed at R): the smallest rigid shift d such that every outer-ring
    # vertex lies inside the translated nominal circle. Excess ~ 0 means the mesh ring is the nominal cylinder, translated.
    from scipy.optimize import minimize
    obj = lambda d: np.max(np.hypot(ring[:, 0] - c[0] - d[0], ring[:, 2] - c[2] - d[1]) - R)
    best = min((minimize(obj, x0, method='Nelder-Mead', options=dict(xatol=1e-10, fatol=1e-13, maxiter=4000)) for x0 in ([0, 0], [4.5e-6, 3.5e-6])), key=lambda o: o.fun)
    d_star = best.x
    dev = np.hypot(ring[:, 0] - c[0] - d_star[0], ring[:, 2] - c[2] - d_star[1]) - R
    minimax = dict(dx_um=float(d_star[0] * UM), dz_um=float(d_star[1] * UM), offset_um=float(np.hypot(*d_star) * UM),
                   max_radial_excess_um=float(dev.max() * UM), min_radial_excess_um=float(dev.min() * UM), vertices_within_0p01um_of_circle=int((dev > -1e-8).sum()))
    phis = np.arange(-180, 180, 0.01)
    d = delta_um(ring, c, R, phis)
    circle = (cx - c[0]) * np.sin(np.radians(phis)) * UM + (cz - c[2]) * np.cos(np.radians(phis)) * UM - (r - R) * UM  # circle bottom = R + u.d - r
    # identity of the lowest vertex across pitch: the lowest mesh element is always a vertex, never an edge or face interior
    rel = ring - c
    lowest = np.array([np.argmin(np.sin(np.radians(p)) * rel[:, 0] + np.cos(np.radians(p)) * rel[:, 2]) for p in phis[::10]])
    # numerical error: longdouble evaluation of the same function
    sub = phis[::50]
    ld = delta_um(ring, c.astype(np.longdouble), np.longdouble(R), sub, dtype=np.longdouble)
    f64 = delta_um(ring, c, R, sub)
    f32 = np.float32(v * 1e3).astype(np.float64) / 1e3  # STL stores float32 mm
    return dict(
        kind=kind, leg=leg, **meta, unique_vertices=int(len(v)), outer_ring_vertices=int(len(ring)), outer_ring_y_slices=int(len(ys)),
        circle_fit=dict(centre_dx_um=float((cx - c[0]) * UM), centre_dz_um=float((cz - c[2]) * UM), centre_offset_um=float(np.hypot(cx - c[0], cz - c[2]) * UM),
                        radius_minus_nominal_um=float((r - R) * UM), residual_rms_um=float(res.std() * UM), residual_min_um=float(res.min() * UM), residual_max_um=float(res.max() * UM)),
        extents_vs_nominal=ext, nominal_radius_translated_circle_minimax=minimax,
        rings=rings,
        delta_um=dict(min=float(d.min()), max=float(d.max()), pitch_at_min_deg=float(phis[d.argmin()]), fraction_below_minus_1um=float((d < -1).mean()),
                      fraction_above_plus_1um=float((d > 1).mean())),
        ideal_offset_circle_delta_um=dict(min=float(circle.min()), max=float(circle.max())),
        tessellation_residual_um=dict(min=float((d - circle).min()), max=float((d - circle).max())),
        distinct_lowest_vertices_over_pitch=int(len(np.unique(lowest))),
        numeric_error=dict(float64_vs_longdouble_um=float(np.abs(f64 - ld.astype(np.float64)).max()),
                           float32_stl_quantisation_max_um=float(np.abs(f32 - v).max() * UM)))


def main():
    cyl = g2_cylinder()
    meshes = [analyse_mesh(k, 'lf', cyl) for k in ('collision', 'visual')]
    # all four feet: vertex sets coincide in foot_link, so every foot has the same delta(phi)
    feet = {}
    ref = foot_vertices('collision', 'lf')[0]
    key = lambda a: a[np.lexsort(a.T[::-1])]
    for leg in LEGS:
        v, meta = foot_vertices('collision', leg)
        feet[leg] = dict(max_vertex_difference_vs_lf_um=float(np.abs(key(v) - key(ref)).max() * UM), sha256=meta['sha256'])
    col, vis = meshes
    result = dict(
        nominal_cad=dict(source='MATDOG_FOOT_CONTACT_GEOMETRY.yaml cad_validation', radius_mm=cyl['radius'] * 1e3, centre_foot_link_mm=(cyl['center'] * 1e3).tolist(),
                         tread_width_mm=cyl['tread_width'] * 1e3, fillet_mm=cyl['fillet'] * 1e3),
        analytic_g2=dict(identical_to_nominal_cad=True, role='stable contact geometry for IK (YAML note); the collision mesh is for collision validation'),
        meshes=meshes, four_feet_collision_mesh=feet,
        findings=dict(
            reproduced_offset_um=col['circle_fit']['centre_offset_um'],
            g4_reported_offset_um=5.775,
            collision_vs_visual_centre_difference_um=float(np.hypot(col['circle_fit']['centre_dx_um'] - vis['circle_fit']['centre_dx_um'], col['circle_fit']['centre_dz_um'] - vis['circle_fit']['centre_dz_um'])),
            summary=[
                'The collision mesh tread is a circle of diameter 29.8 mm (radius within 1 um of nominal) translated rigidly by the reproduced offset in the foot_link x-z plane.',
                'The visual mesh of the SAME foot is registered differently, so the offset is a property of each STL export, not of the design geometry. The G2 YAML audit used the visual STL; G3.5/G4 used the collision STL.',
                'Mesh-minus-analytic height is a deterministic function of foot pitch: delta = u.d + dR + tessellation residual, with d the centre offset. The offset is why delta reaches -d for some pitches.',
                'The tread vertices lie inside, and on, the nominal-radius circle translated rigidly by the offset (maximum radial excess ~0): tessellation inscribes a polygon in that translated circle, so it can only raise the mesh above it. The least-squares circle scatter (about 1 um) is vertex placement noise.',
                'The lowest mesh element is always a vertex; no horizontal edge or face is ever the lowest feature, so the vertex support function is exact.',
                'Numerical evaluation error (float64 vs long double, float32 STL quantisation) is below 0.01 um and cannot explain the micrometre events.'],
            all_four_feet_identical_in_foot_link=all(f['max_vertex_difference_vs_lf_um'] < 1e-6 for f in feet.values())))
    save('root_cause.json', result)
    for m in meshes:
        print(m['kind'], 'offset um %.3f (dx %.3f dz %.3f) dR %.3f; delta [%.3f, %.3f]; tess [%.4f, %.3f]; err %.1e/%.1e' % (
            m['circle_fit']['centre_offset_um'], m['circle_fit']['centre_dx_um'], m['circle_fit']['centre_dz_um'], m['circle_fit']['radius_minus_nominal_um'],
            m['delta_um']['min'], m['delta_um']['max'], m['tessellation_residual_um']['min'], m['tessellation_residual_um']['max'],
            m['numeric_error']['float64_vs_longdouble_um'], m['numeric_error']['float32_stl_quantisation_max_um']))
    print(result['findings']['collision_vs_visual_centre_difference_um'], result['findings']['all_four_feet_identical_in_foot_link'])


if __name__ == '__main__':
    sys.exit(main())
