import { BufferGeometry, Group, Line, LineBasicMaterial, Mesh, MeshBasicMaterial, MeshStandardMaterial, SphereGeometry, Vector3 } from 'three';
import { sh2ToThree } from '../frame/orientation';
import type { MatdogRobot } from '../urdf/buildRobot';
import { JOINT_NAMES, LEGS, type MotionFrame } from './telemetry';

/**
 * Display state of one link of the ACTUAL model. Never upgraded by a target,
 * a synthetic feed or an interpolation: only a measured, in-range joint chain
 * renders as MEASURED.
 *  - UNMEASURED: no calibrated angle for a joint above this link (or, for
 *    base_link, no valid IMU). Drawn grey/translucent at URDF neutral. It is
 *    NOT a measured posture.
 *  - OUT_OF_RANGE: measured value beyond the canonical URDF limit (drawn at
 *    the limit, red; the true number is reported in the diagnostic).
 *  - IMPLAUSIBLE: measured value far outside any limit (> 0.35 rad); the
 *    joint is held at neutral and drawn red. Never silently accepted.
 */
export type LinkDisplayState = 'MEASURED' | 'UNMEASURED' | 'OUT_OF_RANGE' | 'IMPLAUSIBLE';
export type ActualModelState = 'MEASURED' | 'PARTIAL' | 'UNMEASURED';
const IMPLAUSIBLE_MARGIN_RAD = .35;
const STATE_RANK: Record<LinkDisplayState, number> = { MEASURED: 0, UNMEASURED: 1, OUT_OF_RANGE: 2, IMPLAUSIBLE: 3 };

/** Actual articulation and target articulation share IMU orientation, never each other's joint values. */
export class MotionTwinController {
  readonly overlay = new Group();
  readonly markers = new Map<string, Mesh>();
  readonly trails = new Map<string, Line>();
  /**
   * Visual-only joint interpolation window (0 = off). Displayed numbers always
   * come from the received sample; only the rendered pose eases towards it.
   */
  interpolationMs = 0;
  /** Per-link display state of the actual model after the last apply(). */
  readonly linkStates = new Map<string, LinkDisplayState>();
  actualState: ActualModelState = 'UNMEASURED';
  orientationMeasured = false;
  private readonly tweens = new Map<MatdogRobot, { from: Record<string, number>; to: Readonly<Record<string, number>>; t0: number }>();
  private rejectedChannel = '';
  private readonly measuredMaterials = new Map<string, MeshStandardMaterial>();
  private readonly stateMaterials = new Map<string, Map<LinkDisplayState, MeshStandardMaterial>>();
  /** Links whose pose depends on each joint (the joint's child and descendants). */
  private readonly downstream = new Map<string, string[]>();
  constructor(readonly actual: MatdogRobot, readonly target: MatdogRobot) {
    this.overlay.name = 'motion_feet_support_plane';
    this.target.rootObject.name = 'motion_target_ghost';
    this.target.rootObject.visible = false;
    this.target.rootObject.traverse((object) => {
      if (!(object instanceof Mesh)) return;
      const material = object.material as MeshStandardMaterial;
      material.color.setHex(0x36dcdf); material.transparent = true; material.opacity = .26;
      material.wireframe = true; material.depthWrite = false;
    });
    const geometry = new SphereGeometry(.006, 12, 8);
    for (const leg of LEGS) {
      const marker = new Mesh(geometry, new MeshBasicMaterial({ color: 0x8d99a6 }));
      marker.name = `contact_${leg}`; marker.visible = false;
      this.markers.set(leg, marker); this.overlay.add(marker);
      const line = new Line(new BufferGeometry(), new LineBasicMaterial({ color: 0x36dcdf }));
      line.name = `trajectory_${leg}`; this.trails.set(leg, line); this.overlay.add(line);
    }
    const children = new Map<string, string[]>();
    for (const joint of actual.model.joints) children.set(joint.parent, [...(children.get(joint.parent) ?? []), joint.child]);
    const below = (link: string): string[] => [link, ...(children.get(link) ?? []).flatMap(below)];
    for (const joint of actual.model.joints) this.downstream.set(joint.name, below(joint.child));
    for (const link of actual.collisionLinkNames) {
      const mesh = actual.collisionMesh(link)!;
      const base = mesh.material as MeshStandardMaterial;
      this.measuredMaterials.set(link, base);
      const variants = new Map<LinkDisplayState, MeshStandardMaterial>();
      const unmeasured = base.clone(); unmeasured.color.setHex(0x6b7480); unmeasured.transparent = true; unmeasured.opacity = .38; unmeasured.depthWrite = false;
      const bad = base.clone(); bad.color.setHex(0xe5484d); bad.emissive.setHex(0x3a0000);
      variants.set('UNMEASURED', unmeasured); variants.set('OUT_OF_RANGE', bad); variants.set('IMPLAUSIBLE', bad);
      this.stateMaterials.set(link, variants);
    }
    this.paint(new Map(actual.collisionLinkNames.map((link) => [link, 'UNMEASURED' as LinkDisplayState])));
  }
  private limitsOf(name: string): { lower: number | null; upper: number | null } | null {
    const joint = this.actual.model.joints.find((entry) => entry.name === name);
    return joint ? { lower: joint.lower, upper: joint.upper } : null;
  }
  private classify(name: string, value: number): LinkDisplayState {
    const limit = this.limitsOf(name);
    if (!limit || !Number.isFinite(value)) return 'IMPLAUSIBLE';
    const below = limit.lower !== null ? limit.lower - value : -Infinity, above = limit.upper !== null ? value - limit.upper : -Infinity;
    const excess = Math.max(below, above);
    if (excess > IMPLAUSIBLE_MARGIN_RAD) return 'IMPLAUSIBLE';
    if (excess > 1e-9) return 'OUT_OF_RANGE';
    return 'MEASURED';
  }
  private validJoints(values: Readonly<Record<string, number>>): boolean {
    return Object.entries(values).every(([name, value]) => this.classify(name, value) === 'MEASURED');
  }
  private paint(states: Map<string, LinkDisplayState>): void {
    this.linkStates.clear();
    for (const link of this.actual.collisionLinkNames) {
      const state = states.get(link) ?? 'MEASURED';
      this.linkStates.set(link, state);
      const mesh = this.actual.collisionMesh(link)!;
      mesh.material = state === 'MEASURED' ? this.measuredMaterials.get(link)! : this.stateMaterials.get(link)!.get(state)!;
    }
  }
  private setJoints(robot: MatdogRobot, values: Readonly<Record<string, number>>, nowMs: number | undefined): void {
    if (this.interpolationMs <= 0 || nowMs === undefined) { this.tweens.delete(robot); robot.setJointAngles(values); return; }
    const current = robot.jointSnapshot();
    const from: Record<string, number> = {};
    for (const name of Object.keys(values)) from[name] = current[name] ?? values[name]!;
    this.tweens.set(robot, { from, to: values, t0: nowMs });
  }
  /** Advances visual interpolation; called from the render loop only. */
  animate(nowMs: number): void {
    for (const [robot, tween] of this.tweens) {
      const alpha = Math.min(1, Math.max(0, (nowMs - tween.t0) / this.interpolationMs));
      const pose: Record<string, number> = {};
      for (const [name, to] of Object.entries(tween.to)) pose[name] = tween.from[name]! + (to - tween.from[name]!) * alpha;
      robot.setJointAngles(pose);
      if (alpha >= 1) this.tweens.delete(robot);
    }
  }
  apply(frame: MotionFrame, nowMs?: number): void {
    this.rejectedChannel = '';
    this.orientationMeasured = frame.quaternion !== null;
    if (frame.quaternion) {
      const q = sh2ToThree(frame.quaternion);
      this.actual.setRootOrientation(q); this.target.setRootOrientation(q);
    }
    if (frame.basePositionM !== null) {
      this.actual.rootObject.position.fromArray(frame.basePositionM);
      this.target.rootObject.position.fromArray(frame.basePositionM);
    } else if (frame.baseHeightM !== null) {
      // A height-only feed has no observed XY. Clear a prior replay's XY.
      this.actual.rootObject.position.x = this.target.rootObject.position.x = 0;
      this.actual.rootObject.position.y = this.target.rootObject.position.y = 0;
      this.actual.rootObject.position.z = frame.baseHeightM;
      this.target.rootObject.position.z = frame.baseHeightM;
    }
    // The actual model is ALWAYS shown. Each link carries what is known about it.
    this.actual.rootObject.visible = true;
    const measured = frame.actualByJoint ?? frame.actual;
    const pose: Record<string, number> = {}, states = new Map<string, LinkDisplayState>();
    const mark = (joint: string, state: LinkDisplayState): void => {
      for (const link of this.downstream.get(joint) ?? []) {
        const prior = states.get(link) ?? 'MEASURED';
        if (STATE_RANK[state] > STATE_RANK[prior]) states.set(link, state);
      }
    };
    const problems: string[] = [];
    let known = 0;
    for (const name of JOINT_NAMES) {
      const value = measured?.[name];
      if (value === null || value === undefined) { pose[name] = 0; mark(name, 'UNMEASURED'); continue; }
      const state = this.classify(name, value);
      if (state === 'IMPLAUSIBLE') { pose[name] = 0; mark(name, state); problems.push(`${name}=${value.toFixed(4)} implausible`); continue; }
      known += 1; pose[name] = value; mark(name, state);
      if (state === 'OUT_OF_RANGE') problems.push(`${name}=${value.toFixed(4)} outside canonical URDF limit`);
    }
    if (!this.orientationMeasured) states.set(this.actual.model.rootLink, 'UNMEASURED');
    this.actualState = known === 12 ? 'MEASURED' : known === 0 ? 'UNMEASURED' : 'PARTIAL';
    this.setJoints(this.actual, pose, known === 12 ? nowMs : undefined);
    this.paint(states);
    if (this.actualState !== 'MEASURED') this.rejectedChannel = this.actualState === 'UNMEASURED'
      ? 'actual joint channel unavailable: UNMEASURED URDF neutral (not a measured posture)'
      : `actual joints PARTIAL ${known}/12: unmeasured links grey at URDF neutral`;
    if (problems.length) this.rejectedChannel += (this.rejectedChannel ? ' · ' : '') + problems.join(' · ');
    this.target.rootObject.visible = frame.target !== null && this.validJoints(frame.target);
    if (this.target.rootObject.visible && frame.target) this.setJoints(this.target, frame.target, nowMs);
    else if (frame.target) this.rejectedChannel += ' target joints outside canonical URDF limits';
    for (const foot of frame.feet) {
      const marker = this.markers.get(foot.leg)!;
      marker.visible = foot.position !== null;
      if (foot.position) marker.position.fromArray(foot.position);
      (marker.material as MeshBasicMaterial).color.setHex(
        foot.contact === 'support' ? 0x4ade80 : foot.contact === 'swing' ? 0xfbbf24 : 0x8d99a6);
      const trail = this.trails.get(foot.leg)!;
      const prior = trail.geometry;
      trail.geometry = new BufferGeometry().setFromPoints(foot.trajectory.map((p) => new Vector3(...p)));
      prior.dispose(); trail.visible = foot.trajectory.length > 1;
    }
  }
  /**
   * Stale transport: overlays and the commanded ghost hide; the actual model
   * stays visible but every link is marked UNMEASURED (old data is not current).
   */
  setStale(stale: boolean): void {
    this.overlay.visible = !stale;
    this.target.rootObject.visible = this.target.rootObject.visible && !stale;
    if (stale) this.paint(new Map(this.actual.collisionLinkNames.map((link) => [link, 'UNMEASURED' as LinkDisplayState])));
  }
  get diagnostic(): string { return this.rejectedChannel; }
}
