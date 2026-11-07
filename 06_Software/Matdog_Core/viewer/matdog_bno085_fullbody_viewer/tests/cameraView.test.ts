import { describe, expect, it } from 'vitest';
import { canonicalView } from '../src/viewer/canonicalView';

describe('canonical front camera', () => {
  it('looks from +X towards the centred robot with a stable floor reference', () => {
    const view = canonicalView(2);
    expect(view.target).toEqual([0, 0, .08]);
    expect(view.position[0]).toBeGreaterThan(0);
    expect(view.position[1]).toBe(0);
    expect(view.position[2]).toBeGreaterThan(view.target[2]);
  });
  it('keeps the front framing when the measured heading is -90 degrees', () => {
    const front = canonicalView(2), yawed = canonicalView(2, -Math.PI / 2);
    expect(yawed.position[0]).toBeCloseTo(0, 12);
    expect(yawed.position[1]).toBeCloseTo(-front.position[0], 12);
    expect(yawed.position[2]).toBe(front.position[2]);
    expect(yawed.target).toEqual(front.target);
  });
  it('reserves horizontal room on portrait screens and remains deterministic', () => {
    const desktop = canonicalView(2), phone = canonicalView(.65);
    expect(phone.position[0]).toBeGreaterThan(desktop.position[0]);
    expect(canonicalView(.65)).toEqual(phone);
    expect(canonicalView(0, NaN)).toEqual(canonicalView(1));
  });
});
