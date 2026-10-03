"""Deterministic simulated-perturbation tests of the STAND stabilization chain (production C++ through the bridge).

IMPORTANT SCOPE. This simulates the loop EQUATIONS: measured tilt = commanded tilt delayed by N steps + an external
disturbance + bounded deterministic sensor noise. It is a numerical demonstration that the integral law, its limits,
its fail-safe behaviour and the compensated STAND IK behave as designed. It is NOT a model of the robot's dynamics,
compliance, friction or servo response and it does NOT show physical stability. Policy and controller values below are
STUDY VALUES chosen only to exercise the code; none is an approved or tuned hardware setting. Gains are expressed as fractions
of the analytically derived stability bound (maxStableIntegralGain), never as free tuning numbers.
"""
import json
import sys
import numpy as np
from core import Lib, Loop, OUT
from independent_checks import quat_from_matrix, rx, ry, rz

DEG = np.pi / 180
DT = 0.02                  # 50 Hz: the BNO085 acquisition rate recorded in G3.1
DELAY = 2                  # command-to-measurement delay in steps: STUDY VALUE, unmeasured on the robot
POLICY = [0.1, 0.1, 2, 1e-3, 0.35]   # maxAge s, maxAccuracy rad, minStatus, norm tol, max plausible tilt rad
STEPS = 500


def make_cfg(lib, fraction, **kw):
    bound = lib.so.g5a_max_gain(DT, DELAY)
    cfg = dict(ki=fraction * bound, max_corr=3 * DEG, max_rate=4 * DEG, deadband=0.0, hold=0.5, rref=0.0, pref=0.0)
    cfg.update(kw)
    return [cfg['ki'], cfg['max_corr'], cfg['max_rate'], cfg['deadband'], DT, cfg['hold'], cfg['rref'], cfg['pref']], bound


class Noise:
    def __init__(self, seed=1):
        self.s = seed

    def __call__(self, amp):
        self.s = (1103515245 * self.s + 12345) & 0x7fffffff
        return amp * (2 * (self.s / 0x7fffffff) - 1)


def snapshot(roll, pitch, yaw, t, seq, acc=0.05, status=3):
    q = quat_from_matrix(rz(yaw) @ ry(pitch) @ rx(roll))
    return [q[0], q[1], q[2], q[3], t, seq, acc, status]


def run(lib, name, disturbance, fraction=0.5, noise_deg=0.1, dropout=None, corrupt=None, yaw_rate=0.0, cfg_kw=None, limit_plant=None):
    cfg, bound = make_cfg(lib, fraction, **(cfg_kw or {}))
    loop = Loop(lib, POLICY, cfg, DELAY)
    assert loop.ok, name
    assert loop.enable(0.0)
    contacts, seeds, height, _ = lib.stand()
    noise = Noise(7)
    cmd_hist = [(0.0, 0.0)] * (DELAY + 1)
    seq = 0
    last_snap = None
    seeds_now = seeds.copy()
    log = dict(t=[], roll_meas=[], pitch_meas=[], roll_cmd=[], pitch_cmd=[], state=[], att=[], q_step=[])
    ik_fail = 0; max_drift = 0.0; min_margin = 1e9; max_cond = 0.0; prev_q = None; invalid_steps = 0
    for k in range(1, STEPS + 1):
        t = k * DT
        dr, dp = disturbance(t)
        cr, cp = cmd_hist[-1 - DELAY]
        r = cr + dr + noise(noise_deg * DEG); p = cp + dp + noise(noise_deg * DEG)
        stop = dropout is not None and dropout[0] <= t < dropout[1]
        if not stop:
            seq += 1
            snap = snapshot(r, p, yaw_rate * t, t, seq)
            if corrupt is not None and corrupt(k):
                kind = corrupt(k)
                if kind == 'nan': snap[1] = float('nan')
                elif kind == 'status': snap[7] = 0
                elif kind == 'norm': snap[0] *= 1.2
                elif kind == 'accuracy': snap[6] = 0.5
            last_snap = snap
        out = loop.step(last_snap, t)  # when the sensor stops, the last snapshot is offered again
        if out['attitude'] != 'OK': invalid_steps += 1
        cmd_hist.append((out['roll_cmd'], out['pitch_cmd']))
        c = lib.compensate(out['roll_cmd'], out['pitch_cmd'], seeds=seeds_now)
        if c['status'] != 'OK':
            ik_fail += 1
        else:
            seeds_now = c['joints']; max_drift = max(max_drift, c['drift']); min_margin = min(min_margin, c['margin']); max_cond = max(max_cond, c['condition'])
            if prev_q is not None: log['q_step'].append(float(np.abs(c['joints'] - prev_q).max()))
            prev_q = c['joints']
        log['t'].append(t); log['roll_meas'].append(r); log['pitch_meas'].append(p); log['roll_cmd'].append(out['roll_cmd']); log['pitch_cmd'].append(out['pitch_cmd']); log['state'].append(out['state']); log['att'].append(out['attitude'])
    t = np.array(log['t']); rm = np.array(log['roll_meas']); pm = np.array(log['pitch_meas']); rc = np.array(log['roll_cmd']); pc = np.array(log['pitch_cmd'])
    cmd_rate = max(np.abs(np.diff(rc, prepend=0)).max(), np.abs(np.diff(pc, prepend=0)).max()) / DT
    err = np.hypot(rm, pm)
    tail = slice(int(.8 * STEPS), STEPS)
    settle = next((float(t[i]) for i in range(STEPS) if np.all(err[i:] <= 0.02 * max(err.max(), 1e-9) + 0.3 * DEG * 0 + noise_deg * 2 * DEG)), None)
    states = {s: log['state'].count(s) for s in ('ACTIVE', 'RAMPING_DOWN', 'FAULT', 'DISABLED')}
    return dict(name=name, gain_fraction_of_bound=fraction, integral_gain_per_s=cfg[0], stability_bound_per_s=bound, steps=STEPS, dt_s=DT,
                peak_measured_tilt_deg=float(err.max() / DEG), mean_tail_measured_tilt_deg=float(err[tail].mean() / DEG), final_roll_cmd_deg=float(rc[-1] / DEG), final_pitch_cmd_deg=float(pc[-1] / DEG),
                peak_abs_roll_cmd_deg=float(np.abs(rc).max() / DEG), peak_abs_pitch_cmd_deg=float(np.abs(pc).max() / DEG),
                max_command_rate_deg_s=float(cmd_rate / DEG), rate_limit_deg_s=cfg[2] / DEG, correction_limit_deg=cfg[1] / DEG, settle_time_s=settle,
                samples_rejected=invalid_steps, state_counts=states, ik_failures=ik_fail, max_contact_drift_m=max_drift, min_joint_margin_rad=min_margin, max_condition=max_cond,
                max_joint_step_rad=max(log['q_step']) if log['q_step'] else 0.0,
                implied_joint_speed_rad_s=(max(log['q_step']) / DT) if log['q_step'] else 0.0,
                attitude_status_counts={a: log['att'].count(a) for a in sorted(set(log['att']))})


def scenarios(lib):
    step = lambda r, p, t0=0.5: (lambda t: (r * DEG, p * DEG) if t >= t0 else (0, 0))
    ramp = lambda r, p, rate: (lambda t: (min(r, rate * t) * DEG, min(p, rate * t) * DEG) if True else (0, 0))
    sine = lambda amp, f: (lambda t: (amp * DEG * np.sin(2 * np.pi * f * t), 0.0))
    runs = [
        run(lib, 'S1 roll step +2 deg', step(2, 0)),
        run(lib, 'S2 pitch step -2 deg (nose up)', step(0, -2)),
        run(lib, 'S3 combined roll +2 pitch +2 deg', step(2, 2)),
        run(lib, 'S4 slow roll ramp to 3 deg', lambda t: (min(3.0, 0.5 * t) * DEG, 0.0)),
        run(lib, 'S5 sinusoidal roll 1.5 deg at 0.2 Hz', sine(1.5, .2)),
        run(lib, 'S6 sensor dropout 3.0-4.5 s during a +2 deg disturbance', step(2, 0), dropout=(3.0, 4.5)),
        run(lib, 'S7 corrupted samples (NaN / low status / bad norm / accuracy)', step(2, 0), corrupt=lambda k: ['nan', 'status', 'norm', 'accuracy'][k % 4] if 100 <= k < 140 and k % 2 == 0 else None),
        run(lib, 'S8 disturbance beyond the correction limit (+5 deg)', step(5, 0)),
        run(lib, 'S9 heading drift 30 deg/s (yaw must not matter)', step(2, 0), yaw_rate=30 * DEG),
        run(lib, 'S10 unknown IMU mounting offset 0.5 deg, no disturbance', lambda t: (0.5 * DEG, 0.0), cfg_kw=dict(rref=0.0)),
        run(lib, 'S11 same offset with a calibrated level reference', lambda t: (0.5 * DEG, 0.0), cfg_kw=dict(rref=0.5 * DEG)),
        run(lib, 'S12 gain at 25 percent of the bound', step(2, 0), fraction=0.25),
        run(lib, 'S13 gain at 90 percent of the bound', step(2, 0), fraction=0.9),
    ]
    # configuration beyond the bound must be refused by the production code
    cfg, bound = make_cfg(lib, 1.05)
    over = Loop(lib, POLICY, cfg, DELAY)
    refused = not over.ok
    return runs, dict(gain_105_percent_of_bound_refused=refused)


def main():
    lib = Lib()
    runs, extra = scenarios(lib)
    # sensor-fault latch: the stabilizer cannot be re-enabled while latched, nor before the command has ramped to zero
    cfg, _ = make_cfg(lib, 0.5)
    loop = Loop(lib, POLICY, cfg, DELAY); loop.enable(0.0)
    snap = snapshot(0.03, 0, 0, 0.02, 1)
    loop.step(snap, 0.02)
    latched = None
    for k in range(2, 400):
        o = loop.step(snap, k * DT)  # the same stale sample forever
        if o['state'] == 'FAULT' and latched is None: latched = k * DT
    refuse_enable = not loop.enable(10.0)
    loop.reset_fault()
    extra.update(fault_latch_time_s=latched, enable_refused_while_latched=refuse_enable, final_state_after_reset=loop.step(snapshot(0, 0, 0, 8.0, 2), 8.0)['state'])
    result = dict(
        scope='Loop-equation simulation with STUDY values; not a dynamic model of the robot; does not demonstrate physical stability',
        study_values=dict(dt_s=DT, loop_delay_steps=DELAY, attitude_policy=dict(zip(['max_age_s', 'max_accuracy_rad', 'min_accuracy_status', 'norm_tolerance', 'max_plausible_tilt_rad'], POLICY)),
                          correction_limit_deg=3, rate_limit_deg_s=4, hold_before_fault_s=0.5, sensor_noise_amplitude_deg=0.1),
        runs=runs, checks=extra)
    (OUT / 'perturbation_results.json').write_text(json.dumps(result, indent=2, default=lambda o: o.item() if hasattr(o, 'item') else float(o)) + '\n')
    for r in runs:
        print('%-62s peak %.2f tail %.3f deg cmd(r/p) %.2f/%.2f rate %.1f sat-limit %.1f ik_fail %d rejected %d states %s' % (r['name'], r['peak_measured_tilt_deg'], r['mean_tail_measured_tilt_deg'], r['final_roll_cmd_deg'], r['final_pitch_cmd_deg'], r['max_command_rate_deg_s'], r['correction_limit_deg'], r['ik_failures'], r['samples_rejected'], r['state_counts']))
    print(extra)


if __name__ == '__main__':
    sys.exit(main())
