"""Stability decomposition and actuator-requirement analysis for STAND and WALK (offline, rigid-body, URDF masses).

Four different questions are kept apart:
  static equilibrium        COM over the support polygon of a stationary stance
  quasi-static WALK support the G4 margin: COM (no accelerations) against the stance polygon of the moving cycle
  dynamic behaviour         a ZMP check of the same cycle (COM accelerations included, angular momentum neglected) -
                            an approximation, NOT a certification
  actuator feasibility      velocity, acceleration and torque requirements against limits that carry PROVENANCE
Nothing here proves physical stability or authorises any motion.
"""
import csv
import json
import sys
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import ConvexHull
from core import Lib, OUT, ROOT, LEGS
from dynamics import Robot, com_world, strip_ends, distribute_vertical, G

G4 = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
G41 = ROOT / '09_Logs/Validation_Reports/G41_Contact_Reconciliation'
QC = ROOT / '09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/blind_audit'
DEG = np.pi / 180
TICK = 2 * np.pi / 4096


def jl(path):
    return json.loads(path.read_text())


def hull_margin(points_xy, p):
    pts = np.unique(np.round(np.array(points_xy), 12), axis=0)
    if len(pts) < 3:
        return None
    try:
        h = ConvexHull(pts)
    except Exception:
        return None
    return float(np.min(-(h.equations[:, :2] @ np.asarray(p) + h.equations[:, 2])))


def servo_limits():
    """Every actuator number used below, with its provenance and source."""
    units = list(csv.DictReader(open(QC / 'blind_metrics.csv')))
    healthy = [u for u in units if float(u['max_tps']) > 2000]        # M13/M23 carry a 50 percent output cap (documented configuration confound)
    col = lambda name, rows=healthy: np.array([float(u[name]) for u in rows])
    rows = list(csv.DictReader(open(QC / 'blind_move_metrics.csv')))
    ok = {u['scode'] for u in healthy}
    accel = [float(r['plateau_v']) / (float(r['accel_ms']) / 1000) for r in rows if r['phase_name'] in ('MAX_UP', 'MAX_DOWN') and r['scode'] in ok and r['accel_ms'] and float(r['accel_ms']) > 5]
    return dict(
        bench_max_speed_tick_s=dict(healthy_units=len(healthy), min=float(col('max_tps').min()), median=float(np.median(col('max_tps'))), max=float(col('max_tps').max()), source='blind_metrics.csv max_tps (span/duration over the whole cruise)'),
        bench_profile_accel_tick_s2=dict(median=float(np.median(accel)), p95=float(np.percentile(accel, 95)), setting='Acc=50 (register unit 100 tick/s^2 => 5000 nominal); a profile at ONE setting, not a capability limit'),
        bench_first_motion_ms=dict(median_of_unit_medians=float(np.median(col('med_first_motion'))), max_of_unit_medians=float(col('med_first_motion').max())),
        bench_settle_ms=dict(median_of_unit_medians=float(np.median(col('med_settle'))), max_of_unit_medians=float(col('med_settle').max())),
        bench_hysteresis_ticks=dict(median_of_unit_medians=float(np.median(col('hyst_med'))), max=float(col('hyst_max').max())),
        bench_bus_voltage_raw=dict(median=float(np.median(col('max_volt_med'))), unit='raw 0.1 V'),
        bench_conditions='QC V6.1 bench, servos not loaded by the robot, bus ~11.0-11.1 V, 500 Hz sampling, 18 units (16 healthy, 2 capped)',
        vendor=dict(no_load_speed_rad_s=2 * np.pi * 45 / 60, stall_torque_nm=30 * 0.0980665, rated_torque_nm=10 * 0.0980665, test_voltage_v=12.0, resolution_deg=360 / 4095),
        urdf_model_budget=dict(velocity_rad_s=3.03687289847, effort_nm=0.902244, source='URDF README: conservative MATDOG nominal-operation limits for the 3S point, not a hardware validation'))


def stability(lib, robot):
    contacts, seeds, h, _ = lib.stand()
    out = {}
    q = seeds.ravel(); I = np.eye(3); t = np.array([0, 0, h])
    com, mass = com_world(robot, q, I, t)
    pts = []
    for i, leg in enumerate(LEGS):
        a, b = strip_ends(robot, leg, q[3 * i:3 * i + 3], I, t); pts += [a[:2], b[:2]]
    out['total_mass_kg'] = mass
    out['canonical_stand'] = dict(com_world_m=com.tolist(), com_height_above_ground_m=float(com[2]), static_margin_m=hull_margin(pts, com[:2]))
    # static COM margin and load sharing for compensated tilts
    tilt = []
    for roll, pitch in [(0, 0), (2, 0), (-2, 0), (0, 2), (0, -2), (3, 3), (5, 0), (0, 5)]:
        c = lib.compensate(roll * DEG, pitch * DEG)
        if c['status'] != 'OK':
            tilt.append(dict(roll_deg=roll, pitch_deg=pitch, status=c['status'])); continue
        cr, sr, cp, sp = np.cos(roll * DEG), np.sin(roll * DEG), np.cos(pitch * DEG), np.sin(pitch * DEG)
        Rw = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])
        qq = c['joints'].ravel(); cm, _ = com_world(robot, qq, Rw, t)
        pp = []
        for i, leg in enumerate(LEGS):
            a, b = strip_ends(robot, leg, qq[3 * i:3 * i + 3], Rw, t); pp += [a[:2], b[:2]]
        F = distribute_vertical(contacts[:, :2], cm[:2], mass * G)
        tilt.append(dict(roll_deg=roll, pitch_deg=pitch, status='OK', com_xy_shift_vs_level_mm=((cm[:2] - com[:2]) * 1e3).tolist(), static_margin_mm=hull_margin(pp, cm[:2]) * 1e3,
                         foot_forces_n=None if F is None else F.tolist()))
    out['compensated_tilt_static'] = tilt
    return out, (contacts, seeds, h)


def stand_loads(lib, robot, stand, mass_scale):
    contacts, seeds, h = stand
    I = np.eye(3); out = []
    s = seeds.copy()
    for prog in np.linspace(0, 1, 41):
        st, j, hh = lib.stand_path(prog, s)
        if st: out.append(dict(progress=float(prog), status=int(st))); continue
        s = j; qq = j.ravel(); t = np.array([0, 0, hh]); cm, mass = com_world(robot, qq, I, t)
        cp = np.array([robot.contact_point(leg, qq[3 * i:3 * i + 3]) + t for i, leg in enumerate(LEGS)])
        F = distribute_vertical(cp[:, :2], cm[:2], mass * G)
        tau = np.array([robot.leg_torque(leg, qq[3 * i:3 * i + 3], np.zeros(3), np.zeros(3), [0, 0, 0], np.array([0, 0, F[i]])) for i, leg in enumerate(LEGS)]) if F is not None else None
        out.append(dict(progress=float(prog), body_height_m=hh, peak_abs_torque_nm=None if tau is None else np.abs(tau).max(axis=0).tolist(), per_foot_n=None if F is None else F.tolist()))
    ok = [o for o in out if o.get('peak_abs_torque_nm')]
    return dict(samples=len(out), worst_torque_nm_by_joint=np.max([o['peak_abs_torque_nm'] for o in ok], axis=0).tolist(), at_progress=[o['progress'] for o in ok if o['peak_abs_torque_nm'][1] == max(p['peak_abs_torque_nm'][1] for p in ok)][:1], mass_scale=mass_scale)


def stand_rise_derivatives(lib):
    """Joint rates of the G3 low->stand rise for a given duration, from the path and the quintic time law (chain rule)."""
    contacts, seeds, h, low = lib.stand()
    grid = np.linspace(0, 1, 2001); qs = []; s = seeds.copy()
    for p in grid[::-1]:
        st, j, _ = lib.stand_path(p, s)
        assert st == 0, (p, st); s = j; qs.append(j.ravel())
    q = np.array(qs[::-1]); dq = np.gradient(q, grid, axis=0); ddq = np.gradient(dq, grid, axis=0)
    u = np.linspace(0, 1, 2001)
    sp = 30 * u ** 2 * (1 - u) ** 2   # ds/du of s = 10u^3-15u^4+6u^5
    spp = 60 * u * (1 - u) * (1 - 2 * u)
    i = np.arange(len(grid))
    sidx = (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5 * 1.0)
    # velocity/acceleration with respect to progress s(u): interpolate q'(s), q''(s) at s(u)
    f1 = lambda col: np.interp(sidx, grid, col)
    rates = []
    for T in (1, 2, 3, 4, 6, 8):
        qd = np.array([f1(dq[:, k]) * sp / T for k in range(12)]).T
        qdd = np.array([f1(ddq[:, k]) * sp ** 2 / T ** 2 + f1(dq[:, k]) * spp / T ** 2 for k in range(12)]).T
        rates.append(dict(duration_s=T, peak_qdot_rad_s=float(np.abs(qd).max()), peak_qddot_rad_s2=float(np.abs(qdd).max()),
                          peak_qdot_by_class={c: float(np.abs(qd[:, k::3]).max()) for k, c in enumerate(('hip', 'upper', 'lower'))}))
    return dict(path_joint_travel_rad=float(np.abs(q[-1] - q[0]).max()), rates=rates, note='semantic requirement of the G3 contact-locked rise (0.100 -> 0.150 m) for a chosen duration; the duration is not set by any approved parameter')


def walk_cycle(robot, case, T, mass_scale=1.0):
    """Joint torques, velocity/acceleration and the quasi-static vs ZMP support margin over one steady WALK cycle at period T."""
    frames = case['frames']
    ph = np.array([f['phase'] for f in frames]); order = np.argsort(ph); frames = [frames[i] for i in order]; ph = ph[order]
    keep = np.r_[True, np.diff(ph) > 1e-9]; frames = [f for f, k in zip(frames, keep) if k]; ph = ph[keep]
    com = np.array([f['com_world_m'] for f in frames])
    # COM including the advance: spline of position vs phase, accelerations in time = d2/dphi2 / T^2
    cs = CubicSpline(ph, com, bc_type='natural')
    acc = cs(ph, 2) / T ** 2
    tau_rows, margins = [], []
    for f, a in zip(frames, acc):
        q = np.array(f['q']); qd = np.array(f['qdot']) / T; qdd = np.array(f['qddot']) / T ** 2
        R = np.array(f['body'])[:3, :3]; tb = np.array(f['body'])[:3, 3]
        active = [LEGS.index(l) for l in f['active_feet']]
        mass = robot.total_mass; W = mass * (G + a[2])
        cpts = np.array([np.array(f['contacts_world_m'][i]) for i in active])
        c = np.array(f['com_world_m']); zmp = c[:2] - (c[2] / (G + a[2])) * a[:2]
        F_dyn = distribute_vertical(cpts[:, :2], zmp, W); F_st = distribute_vertical(cpts[:, :2], c[:2], mass * G)
        poly = []
        for i in active:
            e = strip_ends(robot, LEGS[i], q[3 * i:3 * i + 3], R, tb); poly += [e[0][:2], e[1][:2]]
        margins.append(dict(phase=float(f['phase']), static=hull_margin(poly, c[:2]), zmp=hull_margin(poly, zmp), support=len(active), F_dyn_ok=F_dyn is not None))
        if F_dyn is None:
            continue
        tau = np.zeros(12)
        for i, leg in enumerate(LEGS):
            Fi = np.array([0, 0, F_dyn[active.index(i)]]) if i in active else None
            tau[3 * i:3 * i + 3] = robot.leg_torque(leg, q[3 * i:3 * i + 3], qd[3 * i:3 * i + 3], qdd[3 * i:3 * i + 3], [a[0] * 0, a[1] * 0, 0.0], Fi, R_wb=R)
        tau_rows.append(tau)
    tau_rows = np.array(tau_rows)
    qd_all = np.abs(np.array([f['qdot'] for f in frames])) / T; qdd_all = np.abs(np.array([f['qddot'] for f in frames])) / T ** 2
    ms = [m['static'] for m in margins if m['static'] is not None]; mz = [m['zmp'] for m in margins if m['zmp'] is not None]
    cls = lambda arr: {c: float(arr[:, k::3].max()) for k, c in enumerate(('hip', 'upper', 'lower'))}
    return dict(period_s=T, mass_scale=mass_scale, frames=len(frames), peak_qdot_by_class=cls(qd_all), peak_qddot_by_class=cls(qdd_all),
                peak_abs_torque_by_class=cls(np.abs(tau_rows)), rms_torque_by_class={c: float(np.sqrt((tau_rows[:, k::3] ** 2).mean())) for k, c in enumerate(('hip', 'upper', 'lower'))},
                min_static_margin_mm=min(ms) * 1e3, min_zmp_margin_mm=min(mz) * 1e3, zmp_outside_polygon_phases=int(sum(1 for m in margins if m['zmp'] is not None and m['zmp'] < 0)),
                frames_without_nonnegative_force_solution=int(sum(1 for m in margins if not m['F_dyn_ok'])),
                com_horizontal_accel_peak_m_s2=float(np.abs(acc[:, :2]).max()), com_vertical_accel_peak_m_s2=float(np.abs(acc[:, 2]).max()))


def main():
    lib = Lib(); robot = Robot()
    lim = servo_limits()
    stab, stand = stability(lib, robot)
    reps = {'WALK_357': jl(G4 / 'dense_robust_full.json')['cases'], 'WALK_287': jl(G4 / 'dense_representatives.json')['cases']}
    cases = {'WALK_357': next(c for c in reps['WALK_357'] if c['id'] == 357), 'WALK_287': next(c for c in reps['WALK_287'] if c['id'] == 287)}
    walk = {n: [walk_cycle(robot, c, T) for T in (1.0, 2.0, 4.0)] for n, c in cases.items()}
    heavy = Robot(mass_scale=3.3 / 2.48)
    walk_heavy = {n: [walk_cycle(heavy, c, T, 3.3 / 2.48) for T in (1.0, 4.0)] for n, c in cases.items()}
    loads = dict(urdf_mass=stand_loads(lib, robot, stand, 1.0), mass_x1p33=stand_loads(lib, heavy, stand, 3.3 / 2.48))
    rise = stand_rise_derivatives(lib)
    # lifecycle semantic peaks (G4.1 audits at period 1 s; scale 1/T and 1/T^2)
    life = {}
    for case in ('WALK_357', 'WALK_287', 'TROT_61', 'TROT_309'):
        d = jl(G41 / f'lifecycle_{case}_G2_NOMINAL.json')['verification']['derivatives']
        life[case] = dict(peak_qdot_rad_s_at_1s=d['max_abs_qdot_rad_s'], peak_qddot_rad_s2_at_1s=d['max_abs_qddot_rad_s2'])
    dyn_tables = {n: jl(G4 / 'derivative_tables.json')['cases'][k] for n, k in (('WALK_357', 'walk'), ('WALK_287', 'walk_100mm'))}
    json_default = lambda o: o.item() if hasattr(o, 'item') else float(o)
    (OUT / 'stability_assessment.json').write_text(json.dumps(dict(
        scope='Rigid-body analysis from CAD URDF masses/inertias. Static equilibrium, quasi-static support and ZMP are different statements; none proves physical stability.',
        mass_properties=stab, stand_static_loads=loads, walk_cycles=walk, walk_cycles_mass_x1p33=walk_heavy,
        notes=['URDF total mass 2.48 kg; the legacy power analysis cites a 3.3 kg robot (unreconciled): results are repeated with the masses scaled by 3.3/2.48.',
               'Load sharing among stance feet is a minimum-norm static ASSUMPTION; the real distribution depends on compliance and is unmeasured.',
               'ZMP neglects angular momentum and base rotation; swing-leg dynamics enter only through the COM trajectory and the leg joint torques.']),
        indent=2, default=json_default) + '\n')
    (OUT / 'actuator_requirements.json').write_text(json.dumps(dict(
        limits=lim, stand_rise=rise, g4_lifecycle_peaks=life, walk_cycles=walk, walk_cycles_mass_x1p33=walk_heavy, stand_static_loads=loads,
        g4_walk_derivative_tables={n: [dict(period_s=p['period_s'], peak_qdot_rad_s=max(p['peak_qdot_rad_s']), peak_qddot_rad_s2=max(p['peak_qddot_rad_s2'])) for p in t['periods']] for n, t in dyn_tables.items()}), indent=2, default=json_default) + '\n')
    s1 = walk['WALK_357'][0]
    print('stand COM', stab['canonical_stand'], 'mass', stab['total_mass_kg'])
    print('WALK357 T=1', {k: v for k, v in s1.items() if k.startswith(('peak', 'min', 'zmp', 'rms'))})
    print('WALK357 T=4 margins', walk['WALK_357'][2]['min_static_margin_mm'], walk['WALK_357'][2]['min_zmp_margin_mm'])
    print('stand worst torque (urdf mass)', loads['urdf_mass']['worst_torque_nm_by_joint'], 'x1.33', loads['mass_x1p33']['worst_torque_nm_by_joint'])
    print('rise', rise['rates'][0], lim['bench_max_speed_tick_s'], lim['bench_profile_accel_tick_s2'], lim['bench_first_motion_ms'], lim['bench_hysteresis_ticks'])


if __name__ == '__main__':
    sys.exit(main())
