#!/usr/bin/env python3
"""Offline V5 collision scene audit of synthetic motion; writes only unsatisfied gates.

The demo's SYNTHETIC witnesses test the feedback protocol. They are not geometry
certificates. This separate audit tests actual canonical collision meshes and
always leaves swept-path/support-load/dynamic hardware qualification open.
"""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse
import hashlib
import json
import sys
import time
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/calibration'))
from matdog_geometry_scene_v5 import RobotSceneV5

LEGS = ('lf', 'rf', 'rh', 'lh')
NAMES = tuple(f'{leg}_{joint}_joint' for leg in LEGS for joint in ('hip', 'upper_leg', 'lower_leg'))
URDF = ROOT / '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
CONTACT = ROOT / '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml'
C4 = ROOT / '09_Logs/Validation_Reports/C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json'
WORKER_SCENE = None
WORKER_FOOT_BOUNDS = None


def base_rotation(quaternion):
    """Canonical telemetry order w,x,y,z; reject malformed orientation."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(float(q @ q) - 1) > 1e-6:
        raise ValueError('invalid telemetry quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def initialize_worker(foot_bounds):
    global WORKER_SCENE, WORKER_FOOT_BOUNDS
    WORKER_SCENE = RobotSceneV5.from_urdf(URDF)
    WORKER_FOOT_BOUNDS = foot_bounds


def audit_frame(item):
    index, row, self_collision_selected = item
    scene, foot_bounds = WORKER_SCENE, WORKER_FOOT_BOUNDS
    violations, raw_foot_mesh_penetrations = [], []
    pose = dict(zip(NAMES, row['telemetry']['actual_rad']))
    for name, angle in pose.items():
        joint = scene.model.joints[name]
        if not np.isfinite(angle) or not joint.lower_limit_rad <= angle <= joint.upper_limit_rad:
            violations.append(dict(frame=index, requirement='posterior_urdf_joint_limit', joint=name, angle_rad=angle))
    if self_collision_selected:
        collided, pair = scene.first_collision_at_pose(pose)
        if collided:
            violations.append(dict(frame=index, phase=row['stand_state'], requirement='discrete_self_collision_clear',
                                   reason='canonical V5 sampled collision', pair=pair))
    height = row['telemetry']['base_height_m']
    up_in_base = base_rotation(row['telemetry']['quaternion'])[2]
    for link in scene.model.collision_link_names:
        broad = scene.world_broad_phase(link, pose)
        # Compose measured body orientation and actual joint FK. Linear support
        # over the convex hull equals the minimum of the original mesh vertices.
        minimum = float((broad.hull_vertices_world @ up_in_base).min()) + height
        if minimum < 0 and link in foot_bounds:
            within_bound = minimum >= -foot_bounds[link] - 1e-9
            raw_foot_mesh_penetrations.append(dict(frame=index, phase=row['stand_state'], link=link,
                                                   minimum_z_m=minimum, derived_radial_bound_m=foot_bounds[link],
                                                   explained_by_contact_mesh_discretization=within_bound))
        tolerance = foot_bounds.get(link, 1e-6) + 1e-9
        if minimum < -tolerance:
            violations.append(dict(frame=index, phase=row['stand_state'], requirement='nonpenetrating_support_plane',
                                   reason='canonical mesh below support plane beyond derived model/discretization bound', link=link, minimum_z_m=minimum))
    return violations, raw_foot_mesh_penetrations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stride', type=int, default=80)
    parser.add_argument('--progress-every', type=int, default=100)
    parser.add_argument('--workers', type=int, default=1, help='independent offline V5 processes')
    args = parser.parse_args()
    if args.stride < 1:
        parser.error('--stride must be positive')
    if args.workers < 1:
        parser.error('--workers must be positive')
    frames = [json.loads(line) for line in args.trace.read_text().splitlines() if line.strip()]
    scene = RobotSceneV5.from_urdf(URDF)
    selected = list(range(0, len(frames), args.stride))
    selected.extend(i for i, row in enumerate(frames) if i == 0 or row['stand_state'] != frames[i - 1]['stand_state'])
    selected = sorted(set(selected + [len(frames) - 1]))
    contact = yaml.safe_load(CONTACT.read_text())['rigid_cylinder']
    center = np.array(contact['center_in_foot_link_m'])
    axis = np.array(contact['axis_in_foot_link_unit'])
    axis = axis / np.linalg.norm(axis)
    foot_bounds = {}
    for link in yaml.safe_load(CONTACT.read_text())['applies_to_links']:
        vertices = scene.mesh(link).triangles_local.reshape(-1, 3)
        relative = vertices - center
        radial = relative - np.outer(relative @ axis, axis)
        foot_bounds[link] = max(0., float(np.linalg.norm(radial, axis=1).max()) - contact['radius_m'])
    violations = []
    raw_foot_mesh_penetrations = []
    selected_set = set(selected)
    plane_checks = len(frames) * len(scene.model.collision_link_names)
    started = time.monotonic()
    items = [(index, row, index in selected_set) for index, row in enumerate(frames)]
    executor = None
    if args.workers == 1:
        global WORKER_SCENE, WORKER_FOOT_BOUNDS
        WORKER_SCENE, WORKER_FOOT_BOUNDS = scene, foot_bounds
        results = map(audit_frame, items)
    else:
        executor = ProcessPoolExecutor(max_workers=args.workers, initializer=initialize_worker, initargs=(foot_bounds,))
        results = executor.map(audit_frame, items, chunksize=8)
    try:
        for index, (frame_violations, frame_raw) in enumerate(results):
            violations.extend(frame_violations)
            raw_foot_mesh_penetrations.extend(frame_raw)
            if frame_violations:
                print(json.dumps(dict(frame=index, violations=frame_violations)), file=sys.stderr, flush=True)
            if args.progress_every > 0 and (index + 1) % args.progress_every == 0:
                print(f'V5 audited {index+1}/{len(frames)} frames; elapsed={time.monotonic()-started:.1f}s; violations={len(violations)}', file=sys.stderr, flush=True)
    finally:
        if executor:
            executor.shutdown(wait=True, cancel_futures=True)
    unsatisfied = [
        dict(requirement='autonomous_acquisition_support_transfer_deep_folded_REST_GROUND', reason='The candidate requires externalSupportRequired for a rigid fixture lift to65mm, Cartesian unfolding, rear-footprint touchdown and canonical placement. Fixture support is synthetic and required; autonomous belly-to-four-foot support/COM transfer is not implemented/qualified. G5-A tilted IK exists, but AdaptiveStand currently uses flat G2 and has no qualified rocking route. Mesh clearance does not prove loaded support.'),
        dict(requirement='physical_support_and_base_height_observation', reason='Demo inputs are synthetic; IMU and encoders do not measure contact or base_link height.'),
        dict(requirement='continuous_swept_collision_and_rate_bounds', reason=f'All {len(frames)} discrete support-plane frames and {len(selected)} discrete V5 self-collision frames audited; no continuous path certificate.'),
        dict(requirement='loaded_rate_tracking_and_operational_envelopes', reason='Demo envelopes are explicit synthetic test inputs; assembled-robot approval remains required.'),
        dict(requirement='walk_support_transfer_and_dynamic_stability', reason='G4 IK/contact trajectories do not prove physical load transfer or dynamic stability.'),
        dict(requirement='continuous_nonzero_velocity_transition', reason='Same-velocity refresh and zero stop are implemented. Changing nonzero vx/vy/yaw requests coordinated stop and returns INVALID_COMMAND; restart after STAND is required. Continuous transition remains unimplemented/unqualified.'),
        dict(requirement='hardware_GO', reason='Fixture, measured conditions, physical stop, local assistance and explicit operator GO required; no hardware operation attempted.'),
    ]
    if violations:
        unsatisfied.insert(0, dict(requirement='candidate_acquisition_collision_and_ground_clearance',
                                  reason='Synthetic full demo route is not mesh-qualified.', violations=violations))
    report = dict(schema='MATDOG_MOTION_UNSATISFIED_REQUIREMENTS_V1', offline_only=True,
                  hardware_access=False, physical_readiness=False,
                  trace_sha256=hashlib.sha256(args.trace.read_bytes()).hexdigest(),
                  urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
                  c4c_sha256=hashlib.sha256(C4.read_bytes()).hexdigest(),
                  contact_geometry_sha256=hashlib.sha256(CONTACT.read_bytes()).hexdigest(),
                  base_link_flat_belly_height_m=-float(scene.world_broad_phase('base_link', scene.home_pose()).aabb.min_xyz[2]),
                  foot_mesh_radial_overshoot_bounds_m=foot_bounds,
                  raw_foot_mesh_penetrations=raw_foot_mesh_penetrations,
                  continuous_collision_certificate=False, support_load_transfer_validated=False,
                  sampled_frames=len(selected), self_collision_sampled_frames=len(selected),
                  self_collision_stride=args.stride, offline_workers=args.workers,
                  support_plane_frames=len(frames), support_plane_link_checks=plane_checks,
                  raw_foot_penetration_count=len(raw_foot_mesh_penetrations),
                  unexplained_plane_violations=sum(v['requirement']=='nonpenetrating_support_plane' for v in violations),
                  sampled_self_collision_violations=sum(v['requirement']=='discrete_self_collision_clear' for v in violations),
                  posterior_joint_limit_violations=sum(v['requirement']=='posterior_urdf_joint_limit' for v in violations),
                  total_frames=len(frames),
                  unsatisfied_requirements=unsatisfied)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(sampled_self_collision_frames=len(selected), support_plane_frames=len(frames), link_plane_checks=plane_checks, violations=len(violations), report=str(args.output), physical_readiness=False)))
    return 1 if violations else 0

if __name__ == '__main__':
    raise SystemExit(main())
