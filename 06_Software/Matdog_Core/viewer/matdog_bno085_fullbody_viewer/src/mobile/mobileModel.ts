/** Pure presentation model for the phone twin (unit-tested; no DOM). */
import { JOINT_NAMES, type MotionFrame } from '../motion/telemetry';
import type { ActualModelState } from '../motion/twinController';

export interface TwinChip { id: string; text: string; tone: 'ok' | 'warn' | 'bad' | 'info' | '' }
export interface TwinFacts { chips: TwinChip[]; banner: string; rows: [string, string][]; jointLines: string[] }

export function describeTwin(frame: MotionFrame | null, ageMs: number | null, stale: boolean, actualState: ActualModelState | null,
  error: string, failures: number): TwinFacts {
  const chips: TwinChip[] = [];
  if (!frame) {
    chips.push({ id: 'chip-link', text: error ? `LINK ${error.split(' ')[0]}` : 'LINK in attesa', tone: error ? 'bad' : '' });
    return { chips, banner: error ? `Nessuna telemetria: ${error}` : 'In attesa della prima telemetria…', rows: [], jointLines: [] };
  }
  const c = frame.channels;
  chips.push({ id: 'chip-link', text: stale ? `LINK STALE ${ageMs === null ? '' : Math.round(ageMs) + ' ms'}` : `LINK ${Math.round(ageMs ?? 0)} ms`, tone: stale ? 'bad' : failures ? 'warn' : 'ok' });
  chips.push({ id: 'chip-imu', text: c.imuValid ? `IMU ${c.imuAgeMs ?? '?'} ms` : c.imuValid === false ? 'IMU NON VALIDA' : 'IMU N/D', tone: c.imuValid ? 'ok' : 'bad' });
  const mapped = c.mappedCount ?? 0;
  chips.push({ id: 'chip-joints', text: actualState === 'MEASURED' ? 'GIUNTI MISURATI 12/12' : actualState === 'PARTIAL' ? `GIUNTI PARZIALI ${mapped}/12` : 'GIUNTI UNMEASURED', tone: actualState === 'MEASURED' ? 'ok' : 'warn' });
  chips.push({ id: 'chip-raw', text: c.rawCount === null ? 'RAW N/D' : `RAW ${c.rawCount}/12`, tone: c.rawCount === 12 ? 'info' : 'warn' });
  const banner = stale ? 'Telemetria STALE: il modello non rappresenta lo stato attuale.'
    : actualState === 'MEASURED' ? 'Postura misurata (12 angoli da encoder mappati) · orientamento BNO085.'
      : actualState === 'PARTIAL' ? 'Postura PARZIALE: i link grigi sono a URDF neutro, non misurati.'
        : 'UNMEASURED: modello URDF neutro, NON è la postura reale. Calibrazione non ripristinata in RAM.';
  const rows: [string, string][] = [
    ['Motivo Controller', frame.reason || '—'], ['Executor', `${frame.executorState} / ${frame.standState}`],
    ['RESTORE', frame.restore ? `${frame.restore.decision} · ${frame.restore.reason}` : 'N/D (firmware senza stato RESTORE)'],
    ['Errore rete', error || 'nessuno'],
  ];
  const jointLines = JOINT_NAMES.map((name, i) => {
    const q = frame.actualByJoint?.[name] ?? frame.actual?.[name] ?? null;
    const raw = frame.rawTicks ? String(frame.rawTicks[i]).padStart(4) : '   —';
    return `${name.padEnd(20)} raw ${raw}  q ${q === null ? 'null (non misurato)' : q.toFixed(4) + ' rad'}`;
  });
  return { chips, banner, rows, jointLines };
}
