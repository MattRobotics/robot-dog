#!/usr/bin/env python3
"""Original MATDOG offline figures from canonical URDF/STL and a semantic trace.

Visualization never supplies collision, contact, loaded stability or motion
permission. Full STL is audited separately by Geometry V5. Drawing alone uses
deterministic triangle subsampling and records that fact in its manifest.
"""
from pathlib import Path
import argparse
import base64
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/calibration'))
from matdog_geometry_scene_v5 import RobotSceneV5, _rotation_from_rpy

URDF = ROOT / '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
LEGS = ('LF', 'RF', 'RH', 'LH')
JOINTS = tuple(f'{leg.lower()}_{part}_joint' for leg in LEGS
               for part in ('hip', 'upper_leg', 'lower_leg'))
COLORS = ('#c55235', '#357db4', '#54853d', '#9465a5')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(q @ q - 1) > 1e-6:
        raise ValueError('invalid world/base quaternion')
    w, x, y, z = q
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def pose(row):
    values = row['telemetry']['actual_rad']
    if len(values) != 12 or not np.all(np.isfinite(values)):
        raise ValueError('trace needs 12 finite actual semantic radians')
    return dict(zip(JOINTS, values))


def body(row):
    t = row['telemetry']
    translation = np.asarray(t['base_position_m'], dtype=float)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)) or abs(translation[2] - t['base_height_m']) > 1e-9:
        raise ValueError('explicit finite world base position/height required; no invented XY')
    return rotation(t['quaternion']), translation


def link_point(scene, name, q, point):
    geometry = scene.model.collision_geometry(name)
    in_collision = _rotation_from_rpy(geometry.origin_rpy).T @ (np.asarray(point) - geometry.origin_xyz)
    transform = scene.collision_transform(name, q)
    return transform.rotation @ in_collision + transform.translation


def com(scene, q, row, inertials):
    r, t = body(row)
    total = sum(mass for _, mass, _ in inertials)
    value = sum(mass * link_point(scene, name, q, center) for name, mass, center in inertials) / total
    return r @ value + t


def anatomy(scene, row):
    q = pose(row)
    answer = {}
    for leg in LEGS:
        prefix = leg.lower()
        upper = scene.collision_transform(f'{prefix}_upper_leg_link', q)
        lower = scene.collision_transform(f'{prefix}_lower_leg_link', q)
        a = upper.rotation @ np.array((0., 0., -.09))
        # Canonical lower-to-foot translation, read rather than assumed q=0.
        foot_joint = scene.model.joints[f'{prefix}_foot_joint']
        b = lower.rotation @ np.asarray(foot_joint.origin_xyz)
        angle = math.degrees(math.acos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1)))
        answer[leg] = dict(upper_axis_world_base=a.tolist(), lower_axis_world_base=b.tolist(),
                           upper_inclination_deg=math.degrees(math.asin(a[2]/np.linalg.norm(a))),
                           lower_inclination_deg=math.degrees(math.asin(b[2]/np.linalg.norm(b))),
                           directed_upper_lower_angle_deg=angle,
                           fold_deviation_from_antiparallel_deg=180-angle)
    return answer


def save(fig, directory, name):
    fig.savefig(directory / f'{name}.png', dpi=150, bbox_inches='tight')
    if name in ('canonical_3d_sequence', 'projections_xz_yz'):
        # Keep canonical scientific renders portable without millions of SVG
        # polygons. The SVG embeds exactly the generated PNG, with no AI image.
        data=(directory/f'{name}.png').read_bytes()
        pixels=plt.imread(directory/f'{name}.png');height,width=pixels.shape[:2]
        encoded=base64.b64encode(data).decode('ascii')
        (directory/f'{name}.svg').write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}"><image width="{width}" height="{height}" href="data:image/png;base64,{encoded}"/></svg>\n')
    else:
        fig.savefig(directory / f'{name}.svg', bbox_inches='tight')
        vector=directory/f'{name}.svg'
        # Matplotlib writes trailing whitespace inside multiline path data.
        # XML whitespace is a token separator; compact it without changing
        # coordinates, glyphs or plotted data.
        vector.write_text(' '.join(line.strip() for line in vector.read_text().splitlines())+'\n')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--comparison', type=Path)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.trace.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError('empty trace')
    scene = RobotSceneV5.from_urdf(URDF)
    if len(scene.model.links) != 17 or len(scene.model.collision_link_names) != 17:
        raise ValueError('expected canonical 17 links and 17 collision meshes')
    args.output.mkdir(parents=True, exist_ok=True)
    inertials = []
    for link in ET.parse(URDF).getroot().findall('link'):
        node = link.find('inertial')
        if node is None:
            raise ValueError('missing CAD inertial: ' + link.get('name'))
        inertials.append((link.get('name'), float(node.find('mass').get('value')),
                          tuple(map(float, node.find('origin').get('xyz').split()))))
    qs = [pose(row) for row in rows]
    times = np.asarray([row['now_ms']/1000 for row in rows])
    heights = np.asarray([row['telemetry']['base_height_m'] for row in rows])
    rpy = []
    for row in rows:
        r, _ = body(row)
        rpy.append((math.atan2(r[2, 1], r[2, 2]), math.asin(np.clip(-r[2, 0], -1, 1)), math.atan2(r[1, 0], r[0, 0])))
    rpy = np.asarray(rpy)
    centers = np.asarray([com(scene, q, row, inertials) for q, row in zip(qs, rows)])
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(times, heights*1000, label='base_link Z / support plane')
    axes[0].axhline(150, ls='--', color='gray', label='150 mm target')
    axes[0].set_ylabel('height [mm]'); axes[0].legend()
    axes[1].plot(times, np.degrees(rpy[:, 0]), label='roll')
    axes[1].plot(times, np.degrees(rpy[:, 1]), label='pitch')
    axes[1].set_ylabel('angle [deg]'); axes[1].legend()
    for index, label in enumerate(('COM X', 'COM Y', 'COM Z')):
        axes[2].plot(times, centers[:, index]*1000, label=label)
    axes[2].set_ylabel('CAD COM [mm]');axes[2].set_xlabel('trace Controller time [s]');axes[2].legend()
    for axis in axes: axis.grid(alpha=.25)
    fig.suptitle('MATDOG canonical kinematics — synthetic inputs; CAD mass, no loaded support proof')
    save(fig, args.output, 'height_attitude_com')
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for index, leg in enumerate(LEGS):
        points = np.asarray([row['telemetry']['feet'][leg]['position_m'] for row in rows])
        for dimension, label in enumerate(('X', 'Y', 'Z')):
            axes[index].plot(times, points[:, dimension]*1000, label=label)
        stance = np.asarray([row['telemetry']['feet'][leg]['state']=='STANCE' for row in rows])
        axes[index].fill_between(times, 0, 1, where=stance, transform=axes[index].get_xaxis_transform(),
                                 color=COLORS[index], alpha=.12, label='reported stance (not measured load)')
        candidates=np.asarray([row['telemetry']['feet'][leg]['state']=='CANDIDATE_CONTACT' for row in rows])
        if candidates.any():
            axes[index].fill_between(times,0,1,where=candidates,transform=axes[index].get_xaxis_transform(),
                                    color=COLORS[index],alpha=.08,label='geometric candidate contact; load unknown')
        axes[index].set_ylabel(f'{leg} [mm]');axes[index].legend(loc='upper right', fontsize=8);axes[index].grid(alpha=.25)
    axes[-1].set_xlabel('trace Controller time [s]')
    fig.suptitle('MATDOG four-foot trajectories and declared contact transitions')
    save(fig, args.output, 'foot_trajectories_contacts')
    # Deterministic trace indices: begin, interior acquisition, rise, STAND,
    # WALK and intermediate restart when those phases are actually available.
    chosen = [0]
    for phase in ('ACQUIRE_SUPPORT', 'RISE', 'STAND', 'WALK', 'STOPPING'):
        indices = [i for i, row in enumerate(rows) if row['stand_state']==phase]
        if indices: chosen.append(indices[len(indices)//2])
    chosen = sorted(set(chosen + [len(rows)-1]))[:8]
    fig = plt.figure(figsize=(14, 4*math.ceil(len(chosen)/3)))
    projected, projection_axes = plt.subplots(2, 1, figsize=(12, 8))
    projection_axes[0].set(xlabel='X [m]', ylabel='Z [m]', title='XZ — actual canonical STL projections')
    projection_axes[1].set(xlabel='Y [m]', ylabel='Z [m]', title='YZ — actual canonical STL projections')
    rendered = {}
    for ordinal, index in enumerate(chosen):
        axis = fig.add_subplot(math.ceil(len(chosen)/3), 3, ordinal+1, projection='3d')
        row, q = rows[index], qs[index];r, t = body(row)
        for link in scene.model.collision_link_names:
            transform = scene.collision_transform(link, q)
            original = scene.mesh(link).triangles_local
            stride = max(1, math.ceil(len(original)/1200))
            triangles = original[::stride] @ transform.rotation.T + transform.translation
            triangles = triangles @ r.T + t
            color = '#6a7279' if link=='base_link' else COLORS[LEGS.index(link[:2].upper())]
            axis.add_collection3d(Poly3DCollection(triangles, facecolor=color, edgecolor='none', alpha=.72))
            if ordinal in (0, len(chosen)-1):
                for proj, dimensions in enumerate(((0, 2), (1, 2))):
                    projection_axes[proj].add_collection(PolyCollection(triangles[:, :, dimensions], facecolor=color,
                       edgecolor='none', alpha=.15 if ordinal==0 else .40))
            rendered[link] = dict(source_triangle_count=len(original), displayed_triangle_count=len(triangles), stride=stride)
        axis.plot([-0.37, .27], [0, 0], [0, 0], c='black', lw=1)
        axis.scatter(*centers[index], color='gold', s=22)
        axis.set(xlim=(-.37, .27), ylim=(-.2, .2), zlim=(-.04, .35), xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
        axis.set_box_aspect((.64, .4, .39));axis.view_init(elev=18, azim=-62)
        marker=' / FAILED POSE' if row.get('geometric_frame_accepted') is False else ''
        axis.set_title(f"frame {index} / {row['stand_state']}{marker}\nZ={heights[index]*1000:.1f} mm / t={times[index]:.2f}s", fontsize=9)
    fig.subplots_adjust(top=.80 if len(chosen)<=3 else .87, hspace=.40)
    fig.suptitle('Canonical URDF + all 17 actual collision STL — deterministic display decimation only', fontsize=11, y=.98)
    save(fig, args.output, 'canonical_3d_sequence')
    for axis in projection_axes:
        axis.axhline(0, color='black', lw=.8);axis.autoscale_view();axis.set_aspect('equal');axis.grid(alpha=.2)
    projected.suptitle('Initial and final meshes; body ground contact is intentional, not penetration permission')
    projected.subplots_adjust(hspace=.42,top=.87)
    save(projected, args.output, 'projections_xz_yz')
    result = dict(schema='MATDOG_MOTION_RECOVERY_FIGURES_V1', hardware_access=False,
       hardware_readiness=False, measured_initial_pose=False,
       initial_pose_provenance=rows[0].get('initial_pose_provenance', 'semantic trace synthetic candidate; no measured 12-joint starting pose'),
       missing_measurement='12 current-installation semantic joint observations in real belly rest, body orientation/support plane and contact/load witness',
       trace_sha256=digest(args.trace), trace_frames=len(rows), urdf_sha256=digest(URDF),
       initial_joint_order=list(JOINTS), initial_angles_rad=rows[0]['telemetry']['actual_rad'],
       initial_anatomy=anatomy(scene, rows[0]), urdf_mass_kg=sum(i[1] for i in inertials),
       initial_cad_com_world_m=centers[0].tolist(), com_uncertainty='CAD inertials only; as-built battery/electronics/mounting and load distribution unmeasured',
       base_link_height_method='canonical body underside vertex minimum against support plane; trace orientation and height supplied explicitly, not inferred from IMU altitude',
       meshes={name:dict(sha256=digest(scene.model.collision_mesh_path(name)), **rendered[name]) for name in rendered},
       selected_frames=chosen, final_base_link_height_m=float(heights[-1]),
       final_pose_geometry_accepted=rows[-1].get('geometric_frame_accepted'),
       visualization_is_collision_certificate=False, contact_position_is_load_proof=False)
    if args.audit:
        result['geometry_audit_sha256']=digest(args.audit)
    if args.comparison:
        before=[json.loads(line) for line in args.comparison.read_text().splitlines() if line.strip()]
        result['previous_candidate']=dict(trace_sha256=digest(args.comparison), trace_frames=len(before),
                                          initial_anatomy=anatomy(scene,before[0]))
    (args.output/'figures_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(frames=len(rows),rendered_links=len(rendered),initial_pose_measured=False,
                          urdf_mass_kg=result['urdf_mass_kg'],output=str(args.output))))


if __name__=='__main__':
    main()
