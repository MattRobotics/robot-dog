import { BufferGeometry, Group, Line, LineBasicMaterial, Mesh, MeshBasicMaterial, MeshStandardMaterial, SphereGeometry, Vector3 } from 'three';
import { sh2ToThree } from '../frame/orientation';
import type { MatdogRobot } from '../urdf/buildRobot';
import { LEGS, type MotionFrame } from './telemetry';

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
  private readonly tweens = new Map<MatdogRobot, { from: Record<string, number>; to: Readonly<Record<string, number>>; t0: number }>();
  private rejectedChannel = '';
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
  }
  private validJoints(values: Readonly<Record<string, number>>): boolean {
    for (const [name, value] of Object.entries(values)) {
      const joint = this.actual.model.joints.find((entry) => entry.name === name);
      if (!joint || !Number.isFinite(value) || (joint.lower !== null && value < joint.lower - 1e-9) ||
        (joint.upper !== null && value > joint.upper + 1e-9)) return false;
    }
    return true;
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
    // Reject an entire invalid joint channel. Do not silently clamp measured states.
    this.actual.rootObject.visible = frame.actual !== null && this.validJoints(frame.actual);
    if (frame.actual && this.actual.rootObject.visible) this.setJoints(this.actual, frame.actual, nowMs);
    else this.rejectedChannel = frame.actual ? 'actual joints outside canonical URDF limits' : 'actual joint channel unavailable';
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
  setStale(stale: boolean): void {
    this.overlay.visible = !stale;
    this.target.rootObject.visible = this.target.rootObject.visible && !stale;
  }
  get diagnostic(): string { return this.rejectedChannel; }
}
