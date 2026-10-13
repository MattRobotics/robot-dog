#!/usr/bin/env python3
"""Create review cards offline. No pre-approved defaults or device access.
Review JSON and current record hashes are local operator evidence, never NVS.
"""
import argparse, json, hashlib, math
from pathlib import Path
FW = Path(__file__).resolve().parents[2]
ROOT = FW.parents[1]
REPORT = ROOT / '09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13'
CONFIRM = 'CONFIRM_INSTALLATION_WORKSPACE_MARGIN_SWEEP_TRACKING_TIMING_OUTPUT'
REQUIRED = ('installation', '12_joint_limits', 'fixed_fixture_free_volume', 'physical_power_cut', 'encoder_mapping_uncertainty', 'drive_units_and_ceiling', 'sequential_bus_deadlines', 'prime_holding_trial_envelope', 'tracking_trial_abort_and_stop')

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def nonzero(s):
    return isinstance(s, str) and len(s) == 64 and all((c in '0123456789abcdef' for c in s)) and (s != '0' * 64)

def create(d):
    if d.get('generation') != 1 or not d.get('operator_name') or (not d.get('review_utc')) or (not nonzero(d.get('current_record_sha256'))):
        raise ValueError('current gen-1 identity and explicit dated operator review required')
    scope = d.get('scope')
    if not scope in (1, 2, 3, 4):
        raise ValueError('review field outside qualified scope')
    for gate in REQUIRED:
        r = d.get('reviews', {}).get(gate, {})
        if not (r.get('approved') is True and nonzero(r.get('evidence_sha256'))):
            raise ValueError(gate + ' requires specific evidence review')
    if scope in (1, 2):
        for gate in ('external_bearing_support_observer', 'loaded_tracking') + (('loaded_stability_contact_compliance',) if scope == 2 else ()):
            r = d.get('reviews', {}).get(gate, {})
            if not (r.get('approved') is True and nonzero(r.get('evidence_sha256'))):
                raise ValueError(gate + ' requires physical measurements')
    route = d['route']
    if not route in ('CANONICAL', 'FIXTURE_LIMIT_COMPATIBLE'):
        raise ValueError('review field outside qualified scope')
    if not (route != 'FIXTURE_LIMIT_COMPATIBLE' or scope in (3, 4)):
        raise ValueError('review field outside qualified scope')
    proof_path = REPORT / ('fixture_limit_compatible_route_6mrad.json' if route == 'FIXTURE_LIMIT_COMPATIBLE' else 'stand_tracking_tube_6mrad.json')
    proof = json.loads(proof_path.read_text())
    if not proof['status'] in ('PASS_FIXTURE_ROUTE_TUBE', 'PASS_GEOMETRY_TUBE'):
        raise ValueError('review field outside qualified scope')
    first, last = d['corridor']
    if not (first >= proof['first_knot'] and last == proof['last_knot'] == 251):
        raise ValueError('review field outside qualified scope')
    e = d['execution']
    encoder, mapping = d['error_budget_rad']
    q = proof['tube_rad']
    drive = d['drive']
    blind = drive['speed_raw'] * drive['speed_rad_s_per_raw'] * e['feedback_age_ms'] / 1000
    if not (encoder >= math.pi / 4096 and mapping > 0 and (e['tracking_rad'] + encoder + mapping + d['max_joint_step_rad'] / 2 + blind < q)):
        raise ValueError('review field outside qualified scope')
    review_sha = hashlib.sha256(json.dumps(d, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    lines = [f'BEGIN 1 {scope}', f'REVIEW {review_sha}', f'RECORD {d['current_record_sha256']}', f'ROUTE {route}']
    joints = d['joint_limits']
    if not len(joints) == 12:
        raise ValueError('review field outside qualified scope')
    for i, j in enumerate(joints):
        if not j['reviewed'] is True:
            raise ValueError('review field outside qualified scope')
        lines.append(f'JOINT {i} {j['required_min_urad']} {j['required_max_urad']} {j['margin_urad']}')
    x = [e[k] for k in ('velocity_rad_s', 'acceleration_rad_s2', 'tracking_rad', 'feedback_age_ms', 'batch_timeout_ms', 'tracking_timeout_ms', 'imu_age_ms', 'temperature_max_c', 'voltage_min_dv', 'voltage_max_dv', 'output_ceiling', 'transaction_timeout_ms', 'contact_inset_ticks')]
    lines.append('EXEC ' + ' '.join(map(str, x)))
    drive = d['drive']
    if not drive['speed_raw'] * drive['speed_rad_s_per_raw'] <= e['velocity_rad_s']:
        raise ValueError('review field outside qualified scope')
    if not drive['acceleration_raw'] * drive['acceleration_rad_s2_per_raw'] <= e['acceleration_rad_s2']:
        raise ValueError('review field outside qualified scope')
    lines.append('DRIVE ' + ' '.join(map(str, [drive['speed_raw'], drive['acceleration_raw'], drive['speed_rad_s_per_raw'], drive['acceleration_rad_s2_per_raw'], d['reviews']['drive_units_and_ceiling']['evidence_sha256']])))
    lines.extend([f'CLOCK {d['stand_time_scale']}', f'STEP {d['max_joint_step_rad']}', 'ERROR_BUDGET ' + ' '.join(map(str, d['error_budget_rad'])), 'FIXTURE ' + ' '.join(map(str, d['measured_fixture_anchor_xyz_rpy'])) + ' ' + d['reviews']['fixed_fixture_free_volume']['evidence_sha256'], f'CORRIDOR {first} {last} {q} {d['fixture_position_tube_m']} {d['fixture_rotation_tube']} {digest(proof_path)}'])
    if scope in (1, 2):
        lines.append('SUPPORT ' + d['reviews']['external_bearing_support_observer']['evidence_sha256'] + ' ' + d['reviews']['loaded_tracking']['evidence_sha256'])
    if scope == 2:
        lines.append('DYNAMICS ' + d['reviews']['loaded_stability_contact_compliance']['evidence_sha256'])
    if scope in (2, 4):
        wpath = REPORT / 'first_stride_qualified_clock_swept_6mrad.json'
        w = json.loads(wpath.read_text())
        if not w['status'] == 'PASS_CONTINUOUS_TARGET_BOXES':
            raise ValueError('review field outside qualified scope')
        table = json.loads((REPORT / 'compiled_first_stride_receipt.json').read_text())
        scale = d['walk_time_scale']
        if not 9 <= scale <= 100:
            raise ValueError('review field outside qualified scope')
        if not e['tracking_rad'] + encoder + mapping + blind < table['runtime_error_tube_rad']:
            raise ValueError('review field outside qualified scope')
        if not d['walk_step_rad'] >= w['maximum_joint_target_step_rad']:
            raise ValueError('review field outside qualified scope')
        if not d['walk_step_rad'] <= 0.01:
            raise ValueError('review field outside qualified scope')
        lines.extend([f'WALK_CLOCK {scale} {d['walk_step_rad']}', f'WALK 90 90 .005 .8 .04 8 {0.005 / (90 * scale):.17g} 0 0 {digest(wpath)} {table['runtime_error_tube_rad']:.17g}'])
    sha = hashlib.sha256(('\n'.join(lines) + '\n').encode()).hexdigest()
    lines.append(f'SEAL {sha} {CONFIRM}')
    if not all((len(x) <= 240 for x in lines)):
        raise ValueError('card exceeds bounded USB parser')
    return '\n'.join(lines) + '\n'
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('review', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not not a.output.exists():
        raise ValueError('review field outside qualified scope')
    text = create(json.loads(a.review.read_text()))
    a.output.write_text(text)
    print('REVIEW_CARDS_CREATED_NO_PERMIT', digest(a.output))
