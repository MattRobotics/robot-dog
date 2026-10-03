"""Classify every actuator-related requirement as VERIFIED / PROVISIONALLY_SUPPORTED / EXCEEDS_LIMIT / REQUIRES_MEASUREMENT.

Classification is done by the production C++ classifier (ActuatorEnvelope) through the bridge. No limit is invented: a limit that
has no measurement is passed as unset and the verdict is REQUIRES_MEASUREMENT. Nothing in the repository measures actuator
performance under the robot's load, so no row can be VERIFIED. All requirements are semantic study values, not operating settings.
"""
import json
import sys
import numpy as np
from core import Lib, OUT

TICK = 2 * np.pi / 4096
LEVELS = (1.0, 1.25, 1.5)


def main():
    lib = Lib()
    req = json.loads((OUT / 'actuator_requirements.json').read_text())
    lim = req['limits']
    v_bench = lim['bench_max_speed_tick_s']['min'] * TICK                     # slowest healthy unit, unloaded bench
    v_vendor = lim['vendor']['no_load_speed_rad_s']
    v_budget = lim['urdf_model_budget']['velocity_rad_s']
    t_rated, t_stall = lim['vendor']['rated_torque_nm'], lim['vendor']['stall_torque_nm']
    t_overload = 0.8 * t_stall                                                # vendor overload protection: 80 % of stall for 2 s (profile OverloadTorque 80)
    t_budget = lim['urdf_model_budget']['effort_nm']
    a_profile = lim['bench_profile_accel_tick_s2']['median'] * TICK
    rows = []

    def row(group, name, required, unit, limits):
        entry = dict(group=group, requirement=name, required=float(required), unit=unit, limits=[])
        for lname, value, prov in limits:
            entry['limits'].append(dict(limit=lname, value=value, provenance=prov, utilization=None if value is None else float(required / value),
                                        verdicts={str(m): lib.classify(float(required), value, prov, m) for m in LEVELS}))
        rows.append(entry)

    vel = lambda: [('bench no-load slowest healthy unit (2813 tick/s)', v_bench, 'BENCH_NO_LOAD'), ('vendor no-load 45 rpm @12 V', v_vendor, 'VENDOR_NOMINAL'),
                   ('loaded velocity limit on the assembled robot', None, 'UNMEASURED')]
    acc = lambda: [('acceleration capability (streamed targets, any Acc)', None, 'UNMEASURED')]
    tq_peak = lambda: [('vendor overload protection 80 % of stall (2.35 N m)', t_overload, 'VENDOR_NOMINAL'), ('loaded torque capability on the assembled robot', None, 'UNMEASURED')]
    tq_rms = lambda: [('vendor rated torque 10 kg cm (0.98 N m)', t_rated, 'VENDOR_NOMINAL'), ('loaded continuous torque on the assembled robot', None, 'UNMEASURED')]
    # ---- WALK steady cycles (4 periods) from the G4 derivative tables; torques from the rigid-body model
    for case, tables in req['g4_walk_derivative_tables'].items():
        for t in tables:
            row('WALK steady cycle', f"{case} T={t['period_s']:g} s peak joint speed", t['peak_qdot_rad_s'], 'rad/s', vel())
            row('WALK steady cycle', f"{case} T={t['period_s']:g} s peak joint acceleration", t['peak_qddot_rad_s2'], 'rad/s^2', acc())
    for case, cycles in req['walk_cycles'].items():
        for c in cycles:
            row('WALK torque (URDF masses, rigid-body, quasi-static load sharing)', f"{case} T={c['period_s']:g} s peak |torque| (lower leg)", c['peak_abs_torque_by_class']['lower'], 'N m', tq_peak())
            row('WALK torque (URDF masses, rigid-body, quasi-static load sharing)', f"{case} T={c['period_s']:g} s RMS torque (lower leg)", c['rms_torque_by_class']['lower'], 'N m', tq_rms())
    for case, cycles in req['walk_cycles_mass_x1p33'].items():
        for c in cycles:
            row('WALK torque (masses x1.33)', f"{case} T={c['period_s']:g} s peak |torque| (lower leg)", c['peak_abs_torque_by_class']['lower'], 'N m', tq_peak())
    # ---- STAND
    for r in req['stand_rise']['rates']:
        if r['duration_s'] in (1, 2, 4):
            row('STAND rise 0.100 -> 0.150 m', f"duration {r['duration_s']:g} s peak joint speed", r['peak_qdot_rad_s'], 'rad/s', vel())
            row('STAND rise 0.100 -> 0.150 m', f"duration {r['duration_s']:g} s peak joint acceleration", r['peak_qddot_rad_s2'], 'rad/s^2', acc())
    st = req['stand_static_loads']['urdf_mass']['worst_torque_nm_by_joint']
    row('STAND static load', 'worst static joint torque along the rise (lower leg)', st[2], 'N m', tq_rms())
    # ---- G4.1 lifecycle semantic peaks (period 1 s) and scaling to slower periods
    for case, d in req['g4_lifecycle_peaks'].items():
        for T in (1, 2, 4):
            row('G4.1 lifecycle semantic peaks', f"{case} lifecycle period {T} s peak joint speed", d['peak_qdot_rad_s_at_1s'] / T, 'rad/s', vel())
            row('G4.1 lifecycle semantic peaks', f"{case} lifecycle period {T} s peak joint acceleration", d['peak_qddot_rad_s2_at_1s'] / T ** 2, 'rad/s^2', acc())
    # ---- design budget (informational): the URDF's own nominal-operation limits
    budget = []
    for case, tables in req['g4_walk_derivative_tables'].items():
        for t in tables:
            budget.append(dict(case=case, period_s=t['period_s'], peak_qdot_rad_s=t['peak_qdot_rad_s'], velocity_budget_rad_s=v_budget, within_budget=bool(t['peak_qdot_rad_s'] <= v_budget)))
    # smallest WALK periods satisfying each speed limit (qdot scales 1/T, qddot 1/T^2)
    scale = {}
    for case, tables in req['g4_walk_derivative_tables'].items():
        t1 = next(t for t in tables if t['period_s'] == 1.0)
        scale[case] = dict(min_period_s_for_bench_speed=t1['peak_qdot_rad_s'] / v_bench, min_period_s_for_urdf_budget_speed=t1['peak_qdot_rad_s'] / v_budget,
                           period_s_where_accel_equals_bench_profile=float(np.sqrt(t1['peak_qddot_rad_s2'] / a_profile)),
                           bench_profile_accel_rad_s2=float(a_profile))
    # ---- command-rate / interpolation arithmetic (protocol arithmetic + measured bench latencies; not a measured 12-servo loop)
    bits = 10.0; baud = 1e6
    sync_bytes = lambda n, data: 8 + n * (1 + data)
    rates = []
    for hz in (25, 50, 100, 200, 500):
        Ts = 1 / hz
        for case, tables in req['g4_walk_derivative_tables'].items():
            for t in tables:
                if t['period_s'] in (1.0, 4.0):
                    rates.append(dict(update_hz=hz, case=case, period_s=t['period_s'], hold_error_ticks=float(t['peak_qdot_rad_s'] * Ts / TICK), linear_interpolation_error_ticks=float(t['peak_qddot_rad_s2'] * Ts ** 2 / 8 / TICK)))
    bus = dict(sync_write_12_servos_bytes=sync_bytes(12, 7), sync_write_12_servos_ms=sync_bytes(12, 7) * bits / baud * 1e3,
               status_read_one_servo_bytes_request_plus_response=8 + 6 + 15, status_read_one_servo_ms_wire_only=(8 + 6 + 15) * bits / baud * 1e3,
               qc_sample_interval_ms=2.0, qc_note='QC reads one servo at 500 Hz: a 12-servo read-all loop is not measured; sequential reads at the QC rate would give about 24 ms',
               first_motion_ms_median_of_unit_medians=lim['bench_first_motion_ms']['median_of_unit_medians'], settle_ms_median_of_unit_medians=lim['bench_settle_ms']['median_of_unit_medians'],
               hysteresis_ticks=lim['bench_hysteresis_ticks'], resolution_deg=lim['vendor']['resolution_deg'])
    delays = [dict(delay_steps=n, dt_s=0.02, max_stable_gain_per_s=lib.so.g5a_max_gain(0.02, n)) for n in range(0, 9)]
    summary = {}
    for r in rows:
        for l in r['limits']:
            summary.setdefault(l['limit'], {}).setdefault(l['verdicts']['1.0'], 0)
            summary[l['limit']][l['verdicts']['1.0']] += 1
    (OUT / 'actuator_feasibility.json').write_text(json.dumps(dict(
        scope='Classification of semantic requirements against limits with provenance. No row is VERIFIED: no loaded-hardware measurement exists. Study requirements, not operating settings.',
        limits_used=dict(bench_slowest_healthy_unit_rad_s=v_bench, vendor_no_load_rad_s=v_vendor, urdf_budget_rad_s=v_budget, vendor_rated_nm=t_rated, vendor_stall_nm=t_stall, vendor_overload_80pct_nm=t_overload,
                         urdf_effort_budget_nm=t_budget, bench_profile_accel_rad_s2=a_profile, tick_rad=TICK), rows=rows, verdict_counts_by_limit=summary,
        urdf_velocity_budget_check=budget, minimum_periods=scale, command_rate_error=rates, bus_and_latency=bus, stabilizer_gain_bound_vs_loop_delay=delays), indent=2) + '\n')
    for k, v in summary.items(): print(k, v)
    print(scale)


if __name__ == '__main__':
    sys.exit(main())
