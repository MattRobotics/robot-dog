/**
 * Read-only Web Serial transport for the MATDOG BNO085 ESP32-S3.
 *
 * SAFETY BOUNDARY — this module deliberately has NO write path:
 *   - `port.writable` is never touched, no writer is ever acquired;
 *   - no command string exists anywhere in this file;
 *   - the firmware's SAVE / STATUS commands are never sent, so the BNO085
 *     DCD can never be written by this viewer;
 *   - nothing here addresses the ST3215 bus, torque or battery systems.
 *
 * The operator picks the device through the browser's own port chooser; no
 * tty path is hard-coded. The port has exactly one owner: close the Arduino
 * Serial Monitor (or any other reader) before connecting.
 */

import { LineAssembler, parseSensorLine, type SensorRecord } from './lineParser';

export type SerialConnectionState = 'DISCONNECTED' | 'CONNECTING' | 'CONNECTED' | 'ERROR';

export interface SerialEvents {
  onState: (state: SerialConnectionState, detail?: string) => void;
  onRecord: (record: SensorRecord) => void;
  /** Raw lines, for the diagnostics log. */
  onLine?: (line: string) => void;
}

export const BNO085_BAUD_RATE = 115200;

export function isWebSerialAvailable(): boolean {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

export class Bno085SerialReader {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private readonly decoder = new TextDecoder('utf-8');
  private state: SerialConnectionState = 'DISCONNECTED';
  private readonly assembler = new LineAssembler();
  private closing = false;

  constructor(private readonly events: SerialEvents) {}

  get connectionState(): SerialConnectionState {
    return this.state;
  }

  private setState(state: SerialConnectionState, detail?: string): void {
    this.state = state;
    this.events.onState(state, detail);
  }

  /**
   * Prompts the operator for a port and starts reading.
   * Requires a user gesture and a secure context (http://localhost or https).
   */
  async connect(): Promise<void> {
    if (!isWebSerialAvailable()) {
      this.setState(
        'ERROR',
        'Web Serial is unavailable. Use Chrome/Edge over http://localhost or https.',
      );
      return;
    }
    if (this.state === 'CONNECTING' || this.state === 'CONNECTED') return;

    this.setState('CONNECTING');
    try {
      const port = await navigator.serial.requestPort();
      await port.open({
        baudRate: BNO085_BAUD_RATE,
        dataBits: 8,
        stopBits: 1,
        parity: 'none',
        flowControl: 'none',
      });
      this.port = port;
      this.assembler.reset();
      this.closing = false;
      this.setState('CONNECTED');
      void this.readLoop(port);
    } catch (error) {
      this.port = null;
      const message = error instanceof Error ? error.message : String(error);
      // The chooser being dismissed is a normal outcome, not a failure.
      if (error instanceof DOMException && error.name === 'NotFoundError') {
        this.setState('DISCONNECTED', 'No port selected.');
      } else {
        this.setState('ERROR', message);
      }
    }
  }

  private async readLoop(port: SerialPort): Promise<void> {
    try {
      if (!port.readable) throw new Error('Serial port is not readable.');
      const reader = port.readable.getReader();
      this.reader = reader;

      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value === undefined) continue;
        // stream: true keeps multi-byte sequences split across chunks intact.
        const text = this.decoder.decode(value, { stream: true });
        if (text.length === 0) continue;
        for (const line of this.assembler.push(text)) {
          this.events.onLine?.(line);
          const record = parseSensorLine(line);
          // Malformed or unrelated lines simply produce no record.
          if (record !== null) {
            try {
              this.events.onRecord(record);
            } catch {
              // A consumer fault must never kill the serial reader.
            }
          }
        }
      }
    } catch (error) {
      if (!this.closing) {
        this.setState('ERROR', error instanceof Error ? error.message : String(error));
      }
    } finally {
      try {
        this.reader?.releaseLock();
      } catch {
        /* already released */
      }
      this.reader = null;
      if (!this.closing) {
        this.closing = true;
        await this.closePort();
        this.setState('DISCONNECTED', 'Serial stream ended.');
      }
    }
  }

  private async closePort(): Promise<void> {
    const port = this.port;
    this.port = null;
    if (!port) return;
    try {
      await port.close();
    } catch {
      /* the port may already be gone (cable pulled) */
    }
  }

  async disconnect(): Promise<void> {
    if (this.state === 'DISCONNECTED') return;
    this.closing = true;
    try {
      await this.reader?.cancel();
    } catch {
      /* cancelling an already-broken stream is fine */
    }
    await this.closePort();
    this.assembler.reset();
    this.setState('DISCONNECTED');
    this.closing = false;
  }
}
