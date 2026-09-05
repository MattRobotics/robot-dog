/**
 * Serial protocol parser: representative real firmware lines must parse, and
 * malformed / partial / unrelated input must be ignored without throwing.
 */

import { describe, expect, it } from 'vitest';
import {
  LineAssembler,
  parseChunk,
  parseSensorLine,
  type RvRecord,
} from '../src/serial/lineParser';

/** Verbatim samples of MATDOG BNO085 Phase C3 firmware output. */
const FIXTURE_BLOCK = [
  'MAG x=-21.688 y=-2.312 z=-42.812 |B|=48.048 status=3 count=13062',
  'RV w=0.642151 x=0.005371 y=0.016052 z=-0.766357 accuracy_rad=0.060547 status=3 count=13060',
  'GYR x=0.000000 y=0.000000 z=0.000000 |w|=0.000000 status=0',
  'SAVE_GATE ready=NO still_ms=0 ACC=3 GYR=0 MAG=3 RV=3 rv_accuracy_rad=0.060547 gyro_mag=0.000000 runtime_resets=0',
  'COUNTS acc=26124 gyr=26124 mag=13062 game=0 rv=13060 runtime_resets=0',
].join('\r\n');

describe('parseSensorLine - real firmware lines', () => {
  it('parses an RV line', () => {
    const record = parseSensorLine(
      'RV w=0.642151 x=0.005371 y=0.016052 z=-0.766357 accuracy_rad=0.060547 status=3 count=13060',
    ) as RvRecord;

    expect(record.kind).toBe('rv');
    expect(record.w).toBeCloseTo(0.642151, 6);
    expect(record.x).toBeCloseTo(0.005371, 6);
    expect(record.y).toBeCloseTo(0.016052, 6);
    expect(record.z).toBeCloseTo(-0.766357, 6);
    expect(record.accuracyRad).toBeCloseTo(0.060547, 6);
    expect(record.status).toBe(3);
    expect(record.count).toBe(13060);
  });

  it('parses a MAG line including the |B| magnitude key', () => {
    const record = parseSensorLine(
      'MAG x=-21.688 y=-2.312 z=-42.812 |B|=48.048 status=3 count=13062',
    );
    expect(record).toEqual({
      kind: 'mag',
      x: -21.688,
      y: -2.312,
      z: -42.812,
      magnitude: 48.048,
      status: 3,
      count: 13062,
    });
  });

  it('parses GYR, COUNTS, SAVE_GATE and runtime reset lines', () => {
    expect(parseSensorLine('GYR x=0.000000 y=0.000000 z=0.000000 |w|=0.000000 status=0')).toEqual({
      kind: 'gyr',
      magnitude: 0,
      status: 0,
    });
    expect(
      parseSensorLine('COUNTS acc=26124 gyr=26124 mag=13062 game=0 rv=13060 runtime_resets=2'),
    ).toEqual({ kind: 'counts', rv: 13060, runtimeResets: 2 });
    expect(
      parseSensorLine(
        'SAVE_GATE ready=YES still_ms=6000 ACC=3 GYR=3 MAG=3 RV=3 rv_accuracy_rad=0.01 gyro_mag=0.0 runtime_resets=1',
      ),
    ).toEqual({ kind: 'save_gate', ready: true, runtimeResets: 1 });
    expect(parseSensorLine('UNEXPECTED_RUNTIME_RESET count=4')).toEqual({
      kind: 'reset',
      runtimeResets: 4,
    });
  });

  it('ignores banners and unrelated firmware chatter', () => {
    const ignored = [
      '',
      '   ',
      '==========================================',
      'MATDOG BNO085 PHASE C3 DCD CONTROL',
      'PHASE_C3_MONITOR=START',
      'VALID_COMMANDS: STATUS | SAVE',
      'CAL_CONFIG requested=0x05 actual=0x05 ACC=1 GYR=0 MAG=1',
      'EXPECTED_STARTUP_RESET=YES',
      'DCD_SAVE=REJECTED_BY_INTERLOCK',
      'RVX w=1 x=0 y=0 z=0',
      ' garbled binary ÿ\x00\x01 ',
    ];
    for (const line of ignored) expect(parseSensorLine(line)).toBeNull();
  });
});

describe('parseSensorLine - malformed input is ignored safely', () => {
  const malformed = [
    'RV',
    'RV w=0.6 x=0.0 y=0.0',
    'RV w=nan x=0 y=0 z=0',
    'RV w=abc x=def y=ghi z=jkl',
    'RV w= x= y= z=',
    'RV w=0 x=0 y=0 z=0',
    'RV w=0.642151 x=0.005371 y=0.016052 z=-0.7663',
    'RV w=1 x=0 y=0 z=0 accuracy_rad=oops status=xx count=yy',
    'RV=====',
    'RV w=Infinity x=0 y=0 z=0',
  ];

  it('never throws and only accepts complete rotation vectors', () => {
    for (const line of malformed) {
      expect(() => parseSensorLine(line)).not.toThrow();
    }
    expect(parseSensorLine('RV')).toBeNull();
    expect(parseSensorLine('RV w=0.6 x=0.0 y=0.0')).toBeNull();
    expect(parseSensorLine('RV w=nan x=0 y=0 z=0')).toBeNull();
    expect(parseSensorLine('RV w=abc x=def y=ghi z=jkl')).toBeNull();
    expect(parseSensorLine('RV w= x= y= z=')).toBeNull();
    expect(parseSensorLine('RV w=0 x=0 y=0 z=0')).toBeNull();
    expect(parseSensorLine('RV w=Infinity x=0 y=0 z=0')).toBeNull();

    // A truncated metadata field still yields a usable quaternion; the
    // optional fields simply come back as null.
    const partial = parseSensorLine('RV w=1 x=0 y=0 z=0 accuracy_rad=oops status=xx count=yy');
    expect(partial).toEqual({
      kind: 'rv',
      w: 1,
      x: 0,
      y: 0,
      z: 0,
      accuracyRad: null,
      status: null,
      count: null,
    });
  });
});

describe('LineAssembler', () => {
  it('extracts every record from a full firmware block', () => {
    const assembler = new LineAssembler();
    const records = parseChunk(assembler, `${FIXTURE_BLOCK}\r\n\r\n`);
    expect(records.map((record) => record.kind)).toEqual([
      'mag',
      'rv',
      'gyr',
      'save_gate',
      'counts',
    ]);
  });

  it('reassembles lines split across arbitrary chunk boundaries', () => {
    const text = `${FIXTURE_BLOCK}\r\n`;
    for (const chunkSize of [1, 3, 7, 64]) {
      const assembler = new LineAssembler();
      const records = [];
      for (let index = 0; index < text.length; index += chunkSize) {
        records.push(...parseChunk(assembler, text.slice(index, index + chunkSize)));
      }
      const rv = records.filter((record) => record.kind === 'rv');
      expect(rv, `chunk size ${chunkSize}`).toHaveLength(1);
      expect((rv[0] as RvRecord).count).toBe(13060);
    }
  });

  it('holds back an incomplete trailing line until it is terminated', () => {
    const assembler = new LineAssembler();
    expect(parseChunk(assembler, 'RV w=1 x=0 y=0 ')).toHaveLength(0);
    expect(parseChunk(assembler, 'z=0 status=3\n')).toHaveLength(1);
  });

  it('does not grow without bound when the device never sends a terminator', () => {
    const assembler = new LineAssembler(1024);
    for (let index = 0; index < 100; index += 1) {
      expect(parseChunk(assembler, 'x'.repeat(1000))).toHaveLength(0);
    }
    expect((assembler.flush() ?? '').length).toBeLessThanOrEqual(1024);
  });
});
