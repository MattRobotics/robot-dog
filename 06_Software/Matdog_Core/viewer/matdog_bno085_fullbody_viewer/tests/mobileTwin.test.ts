import { describe, expect, it } from 'vitest';
import { MtwFormatError, parseMtw } from '../src/urdf/mtwLoader';
import { describeTwin } from '../src/mobile/mobileModel';
import { parseMotionFrame } from '../src/motion/telemetry';

function pack(meshes: { name: string; v: number[]; i: number[] }[], trailing = 0): ArrayBuffer {
  const parts: number[] = [...'MTW1'].map((c) => c.charCodeAt(0));
  const u16 = (x: number) => parts.push(x & 255, x >> 8 & 255);
  const u32 = (x: number) => { u16(x & 0xffff); u16(x >>> 16); };
  const f32 = (x: number) => { const b = new Uint8Array(new Float32Array([x]).buffer); parts.push(...b); };
  u16(meshes.length);
  for (const m of meshes) {
    parts.push(m.name.length, ...[...m.name].map((c) => c.charCodeAt(0)));
    [0, 0, 0].forEach(f32); [1, 1, 1].forEach(f32); u32(m.v.length / 3); u32(m.i.length);
    m.v.forEach(u16); m.i.forEach(u16);
  }
  for (let t = 0; t < trailing; t++) parts.push(0);
  return new Uint8Array(parts).buffer;
}
const tri = { name: 'base_link', v: [0, 0, 0, 10, 0, 0, 0, 10, 0], i: [0, 1, 2] };

describe('MTW1 mobile mesh pack', () => {
  it('parses quantized vertices and indices into geometry (millimetres)', () => {
    const out = parseMtw(pack([tri, { ...tri, name: 'lf_foot_link' }]));
    expect([...out.keys()]).toEqual(['base_link', 'lf_foot_link']);
    const g = out.get('base_link')!;
    expect(Array.from(g.getAttribute('position').array)).toEqual([0, 0, 0, 10, 0, 0, 0, 10, 0]);
    expect(Array.from(g.getIndex()!.array)).toEqual([0, 1, 2]);
  });
  it('fails closed on malformed packs', () => {
    expect(() => parseMtw(new Uint8Array([1, 2, 3]).buffer)).toThrow(MtwFormatError);
    const bad = new Uint8Array(pack([tri])); bad[0] = 88; expect(() => parseMtw(bad.buffer)).toThrow('not an MTW1');
    expect(() => parseMtw(pack([tri]).slice(0, 30))).toThrow('truncated');
    expect(() => parseMtw(pack([{ ...tri, i: [0, 1, 7] }]))).toThrow('index out of range');
    expect(() => parseMtw(pack([tri], 2))).toThrow('trailing');
    expect(() => parseMtw(pack([tri, tri]))).toThrow('duplicate');
    expect(() => parseMtw(pack([{ ...tri, name: '../x' }]))).toThrow('bad');
  });
});

describe('phone twin presentation (14 Oct counterexample)', () => {
  const tel = { version: 1, now_ms: 892599, stale: true, reason: 'RESTORE_REQUIRED', executor_state: 'OFF', stand_state: 'IDLE',
    telemetry: { sampled_at_ms: 892247, actual_rad: null, raw_ticks: [2018, 3002, 3146, 1999, 1196, 944, 2063, 1126, 994, 2049, 3023, 3131],
      target_rad: null, quaternion: [0.12493896, 0.00524902, 0.00561523, -0.99212646] }, imu: { valid: true, stamp_ms: 892591 } };
  it('says UNMEASURED, keeps IMU and RAW fresh, prints null (never 0) per joint', () => {
    const f = describeTwin(parseMotionFrame(tel), 400, false, 'UNMEASURED', '', 0);
    expect(f.banner).toContain('UNMEASURED');
    expect(f.chips).toEqual([{ id: 'twin-status', text: 'DATI N/D', tone: 'warn' }]);
    expect(f.rows.find(([k]) => k === 'IMU')?.[1]).toBe('valida · 8 ms');
    expect(f.rows.find(([k]) => k === 'RAW')?.[1]).toBe('12/12');
    expect(f.jointLines[0]).toContain('raw 2018');
    expect(f.jointLines.every((l) => l.includes('null (non misurato)'))).toBe(true);
  });
  it('reports transport problems by their real names', () => {
    expect(describeTwin(null, null, true, null, 'HTTP_503 CONTROLLER_LOOP_LATE', 1).banner).toContain('HTTP_503');
    const stale = describeTwin(parseMotionFrame(tel), 4000, true, 'UNMEASURED', 'HTTP_TIMEOUT', 3);
    expect(stale.chips[0]).toMatchObject({ text: 'DATI SCADUTI', tone: 'bad' }); expect(stale.banner).toContain('scaduta');
  });
});

describe('single truthful telemetry badge', () => {
  const live = { version: 1, now_ms: 1000, stale: false, imu_valid: true, imu_age_ms: 10,
    telemetry: { sampled_at_ms: 990, source: 'CONTROLLER', actual_rad: Array(12).fill(0), quaternion: [1, 0, 0, 0] } };
  const facts = (body = live, age = 10, stale = false, error = '', failures = 0) =>
    describeTwin(parseMotionFrame(body), age, stale, 'MEASURED', error, failures);
  it('shows modern Controller state/phase ahead of obsolete legacy labels', () => {
    const frame = parseMotionFrame({ ...live, state: 'STOPPED', phase: 'STOP', executor_state: 'OLD_EXECUTOR', stand_state: 'OLD_STAND' });
    expect(frame?.executorState).toBe('STOPPED'); expect(frame?.standState).toBe('STOP');
    expect(describeTwin(frame, 10, false, 'MEASURED', '', 0).rows.find(([key]) => key === 'Executor')?.[1]).toBe('STOPPED / STOP');
  });
  it('keeps the legacy Controller state contract readable when modern fields are absent', () => {
    const frame = parseMotionFrame({ ...live, executor_state: 'OFF', stand_state: 'IDLE' });
    expect(frame?.executorState).toBe('OFF'); expect(frame?.standState).toBe('IDLE');
  });
  it('retains modern FAULT and its original reason in fresh and expired diagnostics', () => {
    const frame = parseMotionFrame({ ...live, state: 'FAULT', phase: 'IDLE', fault: true, reason: 'SERVO_COMMUNICATION_LOST' });
    for (const stale of [false, true]) {
      const f = describeTwin(frame, stale ? 1600 : 10, stale, 'MEASURED', '', 0);
      expect(f.rows.find(([key]) => key === 'Executor')?.[1]).toBe('FAULT / IDLE');
      expect(f.rows.find(([key]) => key === 'Motivo Controller')?.[1]).toBe('SERVO_COMMUNICATION_LOST');
      if (stale) expect(f.chips[0]?.tone).toBe('bad');
      else expect(f.banner).toContain('SERVO_COMMUNICATION_LOST');
    }
  });
  it('shows only LIVE for measured, fresh hardware data, with technical details kept in diagnostics', () => {
    const f = facts();
    expect(f.chips).toEqual([{ id: 'twin-status', text: 'LIVE', tone: 'ok' }]);
    expect(f.banner).toBe('');
    expect(f.rows.map(([k]) => k)).toEqual(expect.arrayContaining(['Fonte telemetria', 'Età telemetria', 'IMU', 'Giunti', 'RAW', 'Quaternion BNO085 (w,x,y,z)', 'Riferimento pavimento']));
  });
  it('never calls an offline fixture, trace or unknown source LIVE', () => {
    for (const source of ['OFFLINE_UI_FIXTURE', 'OFFLINE_SYNTHETIC', 'FAKE_AP_HOST_EMULATION']) {
      expect(facts({ ...live, telemetry: { ...live.telemetry, source } }).chips).toEqual([{ id: 'twin-status', text: 'OFFLINE', tone: 'warn' }]);
    }
    expect(facts({ ...live, owner: 'offline_trace' } as typeof live).chips[0]?.text).toBe('OFFLINE');
    expect(facts({ ...live, telemetry: { ...live.telemetry, source: '' } }).chips[0]?.text).toBe('DATI N/D');
  });
  it('drops LIVE on expiry, invalid IMU or receive error, and retains authentic faults', () => {
    expect(facts(live, 1600).chips[0]?.text).toBe('DATI SCADUTI');
    expect(facts(live, 10, true).chips[0]?.tone).toBe('bad');
    expect(facts(live, 10, false, 'TELEMETRY_REJECTED').chips[0]?.text).toBe('ERRORE');
    expect(facts(live, 10, false, '', 1).chips[0]?.tone).toBe('bad');
    expect(facts({ ...live, imu_valid: false }).chips[0]?.text).toBe('DATI N/D');
    expect(facts({ ...live, imu_age_ms: 201 }).chips[0]?.text).toBe('DATI N/D');
    const fault = { ...live, fault: true, reason: 'SERVO_COMMUNICATION_LOST', telemetry: { ...live.telemetry, source: 'OFFLINE_UI_FIXTURE' } };
    expect(facts(fault).banner).toContain('SERVO_COMMUNICATION_LOST');
  });
});
