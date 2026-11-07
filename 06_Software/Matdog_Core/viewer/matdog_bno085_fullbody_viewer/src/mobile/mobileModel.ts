/** Pure presentation model for the phone twin (unit-tested; no DOM). */
import { JOINT_NAMES, type MotionFrame } from '../motion/telemetry';
import type { ActualModelState, DisplayGrounding } from '../motion/twinController';

export interface TwinChip { id: string; text: string; tone: 'ok' | 'warn' | 'bad' | 'info' | '' }
export interface TwinFacts { chips: TwinChip[]; banner: string; rows: [string, string][]; jointLines: string[] }

export function describeTwin(frame: MotionFrame | null, ageMs: number | null, stale: boolean, actualState: ActualModelState | null,
  error: string, failures: number, ground?: DisplayGrounding): TwinFacts {
  const chips: TwinChip[] = [];
  if (!frame) {
    chips.push({ id: 'twin-status', text: error ? 'ERRORE' : 'IN ATTESA', tone: error ? 'bad' : '' });
    return { chips, banner: error ? `Nessuna telemetria: ${error}` : 'In attesa della prima telemetria…', rows: [], jointLines: [] };
  }
  const c = frame.channels;
  const mapped = c.mappedCount ?? 0;
  const demo = /OFFLINE|SYNTHETIC|FAKE|DEMO/i.test(frame.source) || /OFFLINE|SYNTHETIC|FAKE|DEMO/i.test(frame.owner) ||
    /^OFFLINE_|^SYNTHETIC_NOT_HARDWARE/.test(frame.reason);
  const expired = stale || ageMs === null || !Number.isFinite(ageMs) || ageMs > 1500;
  const imuFresh = frame.quaternion !== null && c.imuValid === true && c.imuAgeMs !== null && c.imuAgeMs <= 200;
  const measured = actualState === 'MEASURED' && !frame.jointsStale && imuFresh;
  const hardware = frame.source === 'CONTROLLER' || frame.source.startsWith('HARDWARE_');
  const failed = !!error || failures > 0;
  chips.push({ id: 'twin-status',
    text: expired ? (demo ? 'OFFLINE · SCADUTO' : 'DATI SCADUTI') : failed ? (demo ? 'OFFLINE · ERRORE' : 'ERRORE')
      : demo ? 'OFFLINE' : measured && hardware ? 'LIVE' : 'DATI N/D',
    tone: expired || failed ? 'bad' : !demo && measured && hardware ? 'ok' : 'warn' });
  const banner = expired ? 'Telemetria scaduta: il modello non rappresenta lo stato attuale.'
    : failed ? `Telemetria non disponibile: ${error || 'errore di ricezione'}`
      : frame.fault !== 'NONE' ? `Fault Controller: ${frame.reason || frame.fault}`
        : actualState !== 'MEASURED' ? `${actualState ?? 'UNMEASURED'}: i link grigi non rappresentano una postura misurata.`
          : !imuFresh ? 'Orientamento IMU non disponibile o non aggiornato.' : '';
  const rows: [string, string][] = [
    ['Fonte telemetria', frame.source || 'non dichiarata'],
    ['Età telemetria', ageMs === null ? 'N/D' : `${Math.round(ageMs)} ms${expired ? ' · STALE' : ''}`],
    ['IMU', c.imuValid ? `valida · ${c.imuAgeMs ?? '?'} ms` : c.imuValid === false ? 'NON VALIDA' : 'N/D'],
    ['Giunti', `${actualState ?? 'UNMEASURED'} · ${mapped}/12`],
    ['RAW', c.rawCount === null ? 'N/D' : `${c.rawCount}/12`],
    ['Quaternion BNO085 (w,x,y,z)', frame.quaternion ? [frame.quaternion.w, frame.quaternion.x, frame.quaternion.y, frame.quaternion.z].map((v) => v.toFixed(6)).join(', ') : 'N/D'],
    ['Motivo Controller', frame.reason || '—'], ['Executor', `${frame.executorState} / ${frame.standState}`],
    ['RESTORE', frame.restore ? `${frame.restore.decision} · ${frame.restore.reason}` : 'N/D (firmware senza stato RESTORE)'],
    ['Errore rete', error || 'nessuno'],
  ];
  const source = stale ? 'unavailable' : ground?.source;
  rows.push(['Riferimento pavimento', source === 'controller-estimate' ? 'Piano orizzontale · stima encoder + IMU'
    : source === 'visual-envelope' ? 'Appoggio grafico dai piedi del modello · contatti non misurati'
      : source === 'reported' ? 'Piano dichiarato dalla telemetria' : 'Non disponibile · posizione grafica non attuale']);
  if (!stale && ground?.baseHeightM !== null && ground?.baseHeightM !== undefined && source !== 'unavailable') {
    rows.push([source === 'visual-envelope' ? 'Quota base_link grafica' : 'Quota base_link stimata', `${(ground.baseHeightM * 1000).toFixed(1)} mm · non è una misura assoluta`]);
    if (ground.adjustmentM > .00005) rows.push(['Correzione grafica anti-penetrazione', `+${(ground.adjustmentM * 1000).toFixed(1)} mm`]);
  }
  const jointLines = JOINT_NAMES.map((name, i) => {
    const q = frame.actualByJoint?.[name] ?? frame.actual?.[name] ?? null;
    const raw = frame.rawTicks ? String(frame.rawTicks[i]).padStart(4) : '   —';
    return `${name.padEnd(20)} raw ${raw}  q ${q === null ? 'null (non misurato)' : q.toFixed(4) + ' rad'}`;
  });
  return { chips, banner, rows, jointLines };
}
