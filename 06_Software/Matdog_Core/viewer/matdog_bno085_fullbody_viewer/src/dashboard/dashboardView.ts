/** DOM rendering of the live bar and the mobile dashboard panel. Display only. */
import { RAD_TO_DEG, sh2ToRollPitchYaw } from '../frame/orientation';
import { DIAGNOSTIC_TEXT, type DiagnosticCode, type SourceClassification } from '../motion/liveStatus';
import { JOINT_NAMES, type MotionFrame } from '../motion/telemetry';
import type { StopResult } from '../motion/bridgeClient';
import { JOINT_COLUMNS, jointRows, type DashboardModel, type JointLimit, type JointRow } from './dashboardModel';

function el(id: string): HTMLElement {
  const found = document.getElementById(id);
  if (!found) throw new Error(`missing DOM element: #${id}`);
  return found;
}
function setText(id: string, value: string): void {
  const node = el(id);
  if (node.textContent !== value) node.textContent = value;
}
const yesNo = (value: boolean | null | undefined): string => value == null ? '?' : value ? 'yes' : 'no';
const or = (value: string | number | null | undefined): string => value == null || value === '' ? '—' : String(value);

let rowCells: HTMLTableCellElement[][] = [];
let rowElements: HTMLTableRowElement[] = [];

export function setUpJointTable(): void {
  const table = el('joint-table') as HTMLTableElement;
  const head = table.tHead!, body = table.tBodies[0]!;
  const headerRow = document.createElement('tr');
  for (const label of JOINT_COLUMNS) {
    const th = document.createElement('th');
    th.textContent = label;
    headerRow.append(th);
  }
  head.replaceChildren(headerRow);
  rowCells = []; rowElements = [];
  body.replaceChildren();
  for (const name of JOINT_NAMES) {
    const tr = document.createElement('tr');
    tr.dataset.joint = name;
    const cells = JOINT_COLUMNS.map(() => document.createElement('td'));
    tr.append(...cells);
    body.append(tr);
    rowCells.push(cells); rowElements.push(tr);
  }
}

function renderRows(rows: readonly JointRow[]): void {
  rows.forEach((row, index) => {
    const tr = rowElements[index], cells = rowCells[index];
    if (!tr || !cells) return;
    tr.classList.toggle('old', row.old);
    tr.classList.toggle('unmapped', row.unmapped);
    tr.classList.toggle('limit', row.outsideLimit);
    row.cells.forEach((value, column) => { if (cells[column] && cells[column].textContent !== value) cells[column].textContent = value; });
  });
}

export function renderLiveBar(source: SourceClassification, codes: readonly DiagnosticCode[]): void {
  const badge = el('source-badge');
  badge.className = `badge src-${source.badge.toLowerCase()}`;
  if (badge.textContent !== source.label) badge.textContent = source.label;
  const chips = el('diag-chips');
  const key = codes.join(',');
  if (chips.dataset.codes === key) return;
  chips.dataset.codes = key;
  chips.replaceChildren(...codes.map((code) => {
    const chip = document.createElement('span');
    chip.className = code === 'LEGACY_POLL' ? 'chip info' : 'chip';
    chip.textContent = code;
    chip.title = DIAGNOSTIC_TEXT[code];
    return chip;
  }));
}

export interface DashboardInput {
  readonly model: DashboardModel | null;
  readonly frame: MotionFrame | null;
  readonly limits: Readonly<Record<string, JointLimit>>;
  readonly transport: string;
  readonly sampleAgeMs: number | null;
  readonly codes: readonly DiagnosticCode[];
}

export function renderDashboard(input: DashboardInput): JointRow[] {
  const { model, frame } = input;
  const c = model?.commissioning ?? null;
  const readiness = model?.readiness ?? null;
  setText('dash-age', input.sampleAgeMs === null ? 'no sample' : `sample age ${Math.round(input.sampleAgeMs)} ms`);
  setText('dash-diagnostics', input.codes.length ? input.codes.map((code) => `${code}: ${DIAGNOSTIC_TEXT[code]}`).join('\n') : 'No diagnostic codes.');
  setText('dash-mode', or(c?.mode));
  const poseJoint = c?.poseJoint != null ? JOINT_NAMES[c.poseJoint] ?? `#${c.poseJoint}` : null;
  setText('dash-pose', c?.pose == null ? '—' : `${c.pose}${c.poseMaxDevRad != null ? ` · max ${(c.poseMaxDevRad * RAD_TO_DEG).toFixed(2)}°` : ''}${poseJoint ? ` @ ${poseJoint}` : ''}`);
  setText('dash-entry', or(c?.entry));
  setText('dash-limits', c?.limits == null && c?.approvedJoints == null ? '—'
    : `${or(c?.limits)} · reviewed ${or(c?.approvedJoints)}/12 · commissioning ${or(c?.commissioningJoints)}/12${c?.primeClampMask ? ` · LOWER inward prime 0x${c.primeClampMask.toString(16)}` : ''}${c?.reason ? ` · ${c.reason}` : ''}`);
  setText('dash-restore', c?.restore ?? (readiness?.restore == null ? '—' : readiness.restore ? 'RESTORED (readiness)' : 'NOT RESTORED (readiness)'));
  const session = model?.localSession;
  setText('dash-session', `${or(session?.state)}${session?.reason ? ` · ${session.reason}` : ''}${c?.session ? ` · commissioning ${c.session}` : ''}`);
  setText('dash-executor', `${or(model?.executorState)} / ${or(model?.standState)}`);
  setText('dash-reason', or(model?.reason));
  const tracking = model?.trackingErrorRad ?? frame?.trackingErrorRad ?? null;
  setText('dash-tracking', tracking === null ? '—' : `${(tracking * RAD_TO_DEG).toFixed(2)}° (${tracking.toFixed(4)} rad)`);
  setText('dash-segment', `${or(c?.segment ?? model?.validation?.segment)} / ${or(c?.phase)}${model?.validation?.scope ? ` · ${model.validation.scope}` : ''}`);
  setText('dash-readiness', readiness === null ? '—' : `restore ${yesNo(readiness.restore)} · calib ${yesNo(readiness.calibrationAccepted)} · limits ${yesNo(readiness.limitsApproved)} · permit ${yesNo(readiness.permit)} · support ${yesNo(readiness.supportObservable)} · hw ${yesNo(readiness.hardwareApproved)}`);
  const power = model?.power;
  setText('dash-power', power ? `${or(power.state)} · SoC ${power.socPct === null ? '—' : `${power.socPct.toFixed(1)} %`}${power.valid === false ? ' (invalid)' : ''}` : '—');

  const q = frame?.quaternion ?? null;
  if (q) {
    const { roll, pitch, yaw } = sh2ToRollPitchYaw(q);
    setText('dash-rpy', `${(roll * RAD_TO_DEG).toFixed(1)}° / ${(pitch * RAD_TO_DEG).toFixed(1)}° / ${(yaw * RAD_TO_DEG).toFixed(1)}° (sensor frame)`);
    setText('dash-quat', [q.w, q.x, q.y, q.z].map((v) => v.toFixed(4)).join(' '));
  } else {
    setText('dash-rpy', 'UNKNOWN (no quaternion reported)');
    setText('dash-quat', '—');
  }
  const imu = model?.imu;
  const imuAge = imu?.stampMs != null && model?.nowMs != null ? Math.max(0, model.nowMs - imu.stampMs) : null;
  setText('dash-imu', imu ? `${imu.valid === false ? 'INVALID' : imu.valid ? 'valid' : '?'} · ${imuAge === null ? '—' : `${imuAge} ms`} (Controller clock)` : '—');

  const bridge = model?.bridge ?? null;
  setText('dash-bridge-mode', bridge ? `${bridge.source} via ${input.transport}${bridge.fakeDevice ? ' · FAKE DEVICE, NOT HARDWARE' : ''}` : `no bridge · ${input.transport}`);
  setText('dash-bridge-rates', bridge ? `TEL ${or(bridge.telHz)} Hz · ${or(bridge.linesPerS)} lines/s${bridge.legacyCycleMs != null ? ` · legacy cycle ${Math.round(bridge.legacyCycleMs)} ms` : ''}` : '—');
  setText('dash-bridge-serial', bridge ? `${bridge.serialState}${bridge.serialDetail ? ` · ${bridge.serialDetail}` : ''}${bridge.consoleActive ? ' · console active' : ''}` : '—');
  setText('dash-bridge-boot', bridge ? `${or(bridge.bootNonce)} / ${bridge.rebootsSeen}` : '—');

  const rows = jointRows(model, frame, input.limits);
  renderRows(rows);
  return rows;
}

export function renderStopResult(result: StopResult): void {
  const node = el('dash-stop-result');
  node.hidden = false;
  const body = (result.body ?? {}) as { written?: unknown; replies?: unknown; error?: unknown; host_utc?: unknown };
  const written = Array.isArray(body.written) ? body.written.join(', ') : '';
  const replies = Array.isArray(body.replies) && body.replies.length ? body.replies.join(' | ') : 'no reply within 0.6 s';
  node.classList.toggle('error', !result.ok);
  node.textContent = result.ok
    ? `STOP written ${typeof body.host_utc === 'string' ? body.host_utc : ''}: ${written}. Controller: ${replies}`
    : `STOP FAILED (${result.error || String(body.error ?? 'unknown')})${written ? `; written: ${written}` : ''}. Use the robot AP dashboard STOP or the physical power cut.`;
}
