/** Offline file adapter only. The live Controller schema stays strict. */
import { LEGS, parseMotionFrame } from './telemetry';

function object(value: unknown): Record<string, unknown> | null {
  return value !== null && typeof value === 'object' && !Array.isArray(value) ? value as Record<string, unknown> : null;
}
export function parseOfflineTrace(text: string): Record<string, unknown>[] {
  if (new Blob([text]).size > 10 * 1024 * 1024) throw new Error('trace exceeds 10 MiB');
  let data: unknown;
  try { data = JSON.parse(text); }
  catch { data = text.split(/\r?\n/).filter((line) => line.trim()).map((line) => JSON.parse(line)); }
  const records = Array.isArray(data) ? data : object(data)?.frames;
  if (!Array.isArray(records) || records.length === 0 || records.length > 10000) throw new Error('expected 1–10000 frames');
  let previousNow = -1, previousSample = -1;
  return records.map((value, index) => {
    const status = object(value), telemetry = object(status?.telemetry);
    if (!status || !telemetry) throw new Error(`missing telemetry at frame ${index}`);
    const normalized = { ...telemetry };
    // The verified offline Motion Core export uses stamp_ms and keyed LF/RF/RH/LH feet.
    // Preserve every numeric sample. Contact remains source-declared simulation evidence.
    if (normalized.sampled_at_ms === undefined) normalized.sampled_at_ms = normalized.stamp_ms;
    const keyedFeet = object(normalized.feet);
    if (keyedFeet) normalized.feet = LEGS.map((leg) => {
      const foot = object(keyedFeet[leg]);
      return { ...foot, leg, contact: foot?.state === 'STANCE' ? 'support' : foot?.state === 'SWING' ? 'swing' : 'unknown' };
    });
    const frame = { ...status, telemetry: normalized,
      owner: status.owner ?? 'offline_trace', fault: status.fault ?? telemetry.fault ?? false,
      reason: `OFFLINE TRACE: ${typeof status.evidence === 'string' ? status.evidence : 'source qualification unspecified'}` };
    const parsed = parseMotionFrame(frame);
    if (!parsed || parsed.controllerNowMs < previousNow || parsed.sampledAtMs < previousSample) throw new Error(`invalid or reordered frame ${index}`);
    if (normalized.actual_rad != null && parsed.actual === null || normalized.target_rad != null && parsed.target === null) throw new Error(`invalid joint channel at frame ${index}`);
    previousNow = parsed.controllerNowMs; previousSample = parsed.sampledAtMs;
    return frame;
  });
}
