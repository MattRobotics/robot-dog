/**
 * Streaming parser for the MATDOG BNO085 Phase C3 firmware text protocol.
 *
 * The firmware prints a block every 500 ms:
 *
 *   MAG x=-21.688 y=-2.312 z=-42.812 |B|=48.048 status=3 count=13062
 *   RV w=0.642151 x=0.005371 y=0.016052 z=-0.766357 accuracy_rad=0.060547 status=3 count=13060
 *   GYR x=0.000000 y=0.000000 z=0.000000 |w|=0.000000 status=0
 *   SAVE_GATE ready=NO still_ms=0 ACC=3 GYR=0 MAG=3 RV=3 rv_accuracy_rad=0.060547 gyro_mag=0.000000 runtime_resets=0
 *   COUNTS acc=... gyr=... mag=... game=... rv=... runtime_resets=0
 *
 * plus assorted banner / status lines. Only RV is used for orientation.
 * Everything unrecognised, truncated or malformed is ignored: the parser
 * never throws on input, it just returns `null` for a line it cannot use.
 *
 * This module is strictly read-only: it has no notion of sending anything.
 */

export interface RvRecord {
  readonly kind: 'rv';
  readonly w: number;
  readonly x: number;
  readonly y: number;
  readonly z: number;
  readonly accuracyRad: number | null;
  readonly status: number | null;
  readonly count: number | null;
}

export interface MagRecord {
  readonly kind: 'mag';
  readonly x: number | null;
  readonly y: number | null;
  readonly z: number | null;
  readonly magnitude: number | null;
  readonly status: number | null;
  readonly count: number | null;
}

export interface GyrRecord {
  readonly kind: 'gyr';
  readonly magnitude: number | null;
  readonly status: number | null;
}

export interface CountsRecord {
  readonly kind: 'counts';
  readonly rv: number | null;
  readonly runtimeResets: number | null;
}

export interface SaveGateRecord {
  readonly kind: 'save_gate';
  readonly ready: boolean | null;
  readonly runtimeResets: number | null;
}

export interface ResetRecord {
  readonly kind: 'reset';
  readonly runtimeResets: number | null;
}

export type SensorRecord =
  | RvRecord
  | MagRecord
  | GyrRecord
  | CountsRecord
  | SaveGateRecord
  | ResetRecord;

/** Splits a line body into its `key=value` tokens. Keys such as `|B|` are kept verbatim. */
function tokenize(body: string): Map<string, string> {
  const fields = new Map<string, string>();
  for (const token of body.trim().split(/\s+/)) {
    const separator = token.indexOf('=');
    if (separator <= 0) continue;
    fields.set(token.slice(0, separator), token.slice(separator + 1));
  }
  return fields;
}

function num(fields: Map<string, string>, key: string): number | null {
  const raw = fields.get(key);
  if (raw === undefined) return null;
  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}

function int(fields: Map<string, string>, key: string): number | null {
  const value = num(fields, key);
  if (value === null) return null;
  return Number.isInteger(value) ? value : null;
}

/**
 * Parses one already-split line. Returns `null` for banners, empty lines,
 * unknown prefixes and any record whose mandatory fields are missing or
 * non-numeric.
 */
export function parseSensorLine(line: string): SensorRecord | null {
  const trimmed = line.trim();
  if (trimmed.length === 0) return null;

  const separator = trimmed.search(/\s/);
  const prefix = separator === -1 ? trimmed : trimmed.slice(0, separator);
  const body = separator === -1 ? '' : trimmed.slice(separator + 1);

  switch (prefix) {
    case 'RV': {
      const fields = tokenize(body);
      const w = num(fields, 'w');
      const x = num(fields, 'x');
      const y = num(fields, 'y');
      const z = num(fields, 'z');
      // A rotation vector is only usable when all four components are present.
      if (w === null || x === null || y === null || z === null) return null;
      if (Math.hypot(w, x, y, z) < 1e-6) return null;
      return {
        kind: 'rv',
        w,
        x,
        y,
        z,
        accuracyRad: num(fields, 'accuracy_rad'),
        status: int(fields, 'status'),
        count: int(fields, 'count'),
      };
    }
    case 'MAG': {
      const fields = tokenize(body);
      return {
        kind: 'mag',
        x: num(fields, 'x'),
        y: num(fields, 'y'),
        z: num(fields, 'z'),
        magnitude: num(fields, '|B|'),
        status: int(fields, 'status'),
        count: int(fields, 'count'),
      };
    }
    case 'GYR': {
      const fields = tokenize(body);
      return { kind: 'gyr', magnitude: num(fields, '|w|'), status: int(fields, 'status') };
    }
    case 'COUNTS': {
      const fields = tokenize(body);
      return { kind: 'counts', rv: int(fields, 'rv'), runtimeResets: int(fields, 'runtime_resets') };
    }
    case 'SAVE_GATE': {
      const fields = tokenize(body);
      const ready = fields.get('ready');
      return {
        kind: 'save_gate',
        ready: ready === 'YES' ? true : ready === 'NO' ? false : null,
        runtimeResets: int(fields, 'runtime_resets'),
      };
    }
    case 'UNEXPECTED_RUNTIME_RESET': {
      const fields = tokenize(body);
      return { kind: 'reset', runtimeResets: int(fields, 'count') };
    }
    default:
      return null;
  }
}

/**
 * Accumulates arbitrary serial chunks and emits complete lines.
 * Handles CR, LF and CRLF, split multi-byte boundaries (via the caller's
 * TextDecoder stream) and pathological input without a terminator.
 */
export class LineAssembler {
  private buffer = '';

  constructor(private readonly maxBufferedChars = 64 * 1024) {}

  /** Feeds a chunk of decoded text and returns the complete lines it produced. */
  push(chunk: string): string[] {
    this.buffer += chunk;
    const lines: string[] = [];

    let start = 0;
    for (let index = 0; index < this.buffer.length; index += 1) {
      const character = this.buffer[index];
      if (character === '\n' || character === '\r') {
        if (index > start) lines.push(this.buffer.slice(start, index));
        start = index + 1;
      }
    }
    this.buffer = this.buffer.slice(start);

    // Defensive: a device that never sends a terminator must not grow memory.
    if (this.buffer.length > this.maxBufferedChars) {
      this.buffer = this.buffer.slice(-this.maxBufferedChars);
    }
    return lines;
  }

  /** Returns and clears whatever is still buffered (used on disconnect). */
  flush(): string | null {
    const remainder = this.buffer;
    this.buffer = '';
    return remainder.length > 0 ? remainder : null;
  }

  reset(): void {
    this.buffer = '';
  }
}

/** Convenience: feed raw text, get back only the records it contained. */
export function parseChunk(assembler: LineAssembler, chunk: string): SensorRecord[] {
  const records: SensorRecord[] = [];
  for (const line of assembler.push(chunk)) {
    const record = parseSensorLine(line);
    if (record !== null) records.push(record);
  }
  return records;
}
