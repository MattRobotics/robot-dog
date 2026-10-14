#!/usr/bin/env python3
"""Offline certificate of the fixture-commissioning ENTRY from a MEASURED start.

Geometry only, no device access. The firmware entry (FixtureEntryPlanner) is
two straight joint-space stages from the armed pose to the fixture route head:
stage 0 moves the four LOWER joints to their head value, stage 1 moves HIP and
UPPER with LOWER held. A quintic time law stays on the same straight segment.

Proofs, each over the continuous segment inside a +/- tube on all 12 joints:
  A. the 120 non-adjacent canonical pairs keep >= 3 mm (self_proof_q);
  B. the ventral region of base_link (lowest 15 mm) keeps >= 3 mm + the
     ventral cover allowance (default 3 mm: the reprinted IMU belly may
     protrude ~3 mm below the STL) from every non-adjacent leg link: exact
     BVH distances at dense samples plus a Lipschitz bound (link radii x
     (half sample step + tube)); the rest of base_link is covered by A;
  C. each joint's adjacent pair is continuously clear over the joint domain
     swept by the entry +/- tube (certify_adjacent_axis.certify).
The fixture itself is NOT in the model: its free volume stays an operator
check. The certificate digest is recorded by the firmware ENTRY command; it is
traceability, never an approval. Start: --start-deg or a JOINT log capture.
"""
from pathlib import Path
import argparse,hashlib,json,math,os,re,sys,time,platform
HERE=Path(__file__).resolve().parent
from paired_prone_model import ROOT,JOINTS
FIRMWARE=ROOT/'05_Firmware/MATDOG_Controller/src/motion/PairedProneData.h'
ROUTE=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json'
URDF=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
SLOTS=tuple(f'{leg}_{part}_joint' for leg in ('lf','rf','rh','lh') for part in ('hip','upper_leg','lower_leg'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def from_log(path):
    """Last JOINT line per slot; requires mapped_valid=1, torque=0, speed 0."""
    q={}
    for line in Path(path).read_text(errors='replace').splitlines():
        m=re.search(r'JOINT slot=(\d+) .*?torque=(-?\d+) speed_raw=(-?\d+).*?mapped_valid=(\d) actual_rad=(-?[0-9.]+)',line)
        if not m:continue
        slot,torque,speed,mapped,rad=int(m[1]),int(m[2]),int(m[3]),int(m[4]),float(m[5])
        if mapped!=1 or torque!=0 or speed!=0:raise SystemExit(f'slot {slot}: needs mapped_valid=1, torque=0, speed 0 (RESTORE done, servos OFF, stationary)')
        q[slot]=rad
    if sorted(q)!=list(range(12)):raise SystemExit(f'log has slots {sorted(q)}, need all 12')
    return [q[i] for i in range(12)]
def head(front_floor,rear_floor):
    d=json.loads(ROUTE.read_bytes());p=dict(d['segments'][0]['start_parameters'])
    p['lf']=max(p['lf'],math.radians(front_floor));p['lr']=max(p['lr'],math.radians(rear_floor))
    q={}
    for leg in ('lf','rf','rh','lh'):
        front=leg in ('lf','rf');q[f'{leg}_hip_joint']=p['h']*(-1 if leg in ('lf','lh') else 1)
        q[f'{leg}_upper_leg_joint']=p['uf' if front else 'ur'];q[f'{leg}_lower_leg_joint']=p['lf' if front else 'lr']
    # Cross-check against the compiled firmware knot 0 (same frozen route).
    knot=re.search(r'kKnots\[\] = \{\s*\{\{([^}]*)\}\}',FIRMWARE.read_text())
    k=[float(x) for x in knot[1].split(',')]
    expect={'h':k[7],'ur':k[0],'lr':max(k[1],math.radians(rear_floor)),'uf':k[2],'lf':max(k[3],math.radians(front_floor))}
    for key,value in expect.items():
        mine=p[key]
        if abs(mine-value)>1e-12:raise SystemExit(f'route head {key} {mine} != firmware {value}')
    return q
def main():
    a=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    g=a.add_mutually_exclusive_group(required=True);g.add_argument('--start-deg');g.add_argument('--joint-log',type=Path)
    a.add_argument('--tube-rad',type=float,default=.01);a.add_argument('--ventral-extra-m',type=float,default=.003)
    a.add_argument('--front-floor-deg',type=float,default=-88.5);a.add_argument('--rear-floor-deg',type=float,default=-89.)
    a.add_argument('--adjacent-budget-s',type=float,default=180);a.add_argument('--ventral-samples',type=int,default=200);a.add_argument('--output',type=Path,required=True)
    args=a.parse_args()
    if args.output.exists():raise SystemExit('output exists; certificates are never overwritten')
    if not math.isfinite(args.tube_rad) or not 0<args.tube_rad<=.01:raise SystemExit('tube must be in (0, 0.01] rad (self_proof_q domain)')
    if not math.isfinite(args.ventral_extra_m) or args.ventral_extra_m<0:raise SystemExit('ventral-extra-m must be finite and >=0')
    if not math.isfinite(args.adjacent_budget_s) or args.adjacent_budget_s<=0 or args.ventral_samples<1:raise SystemExit('positive adjacent budget and at least one sample required')
    start=from_log(args.joint_log) if args.joint_log else [math.radians(float(x)) for x in args.start_deg.split(',')]
    if len(start)!=12 or not all(math.isfinite(x) for x in start):raise SystemExit('need 12 finite start angles')
    target=head(args.front_floor_deg,args.rear_floor_deg)
    q0={SLOTS[i]:start[i] for i in range(12)}
    if any(target[j]<q0[j] for j in SLOTS if j.endswith('lower_leg_joint')):raise SystemExit('LOWER entry must move away from the folded stop, increasing q')
    q1={j:(target[j] if j.endswith('lower_leg_joint') else q0[j]) for j in SLOTS}
    from continuous_pair_path import Continuous
    from certify_adjacent_axis import certify
    c=Continuous();stages=[('ENTRY_DISENGAGE',q0,q1),('ENTRY_ALIGN',q1,target)]
    # Geometry identity is complete: URDF alone does not identify STL bytes.
    meshes={n:{'path':str(c.s.model.collision_mesh_path(n).relative_to(ROOT)),
                 'sha256':sha(c.s.model.collision_mesh_path(n)),
                 'triangles':len(c.s.mesh(n).triangles_local)} for n in c.s.model.collision_link_names}
    if len(meshes)!=17 or len(c.s.model.clearance_pairs())!=120:raise SystemExit('unexpected canonical collision graph')
    # Verify each link-wide radius used by the continuous Lipschitz proof.
    # Distances upstream are bounded by the sum of fixed joint-origin lengths;
    # rotations preserve these lengths. Fixed foot origin is applied exactly.
    import numpy as np
    for n,radii in c.radii.items():
        if n=='base_link':continue
        leg=n[:2];part=n[3:];v=c.s.mesh(n).triangles_local.reshape(-1,3)
        if part=='foot_link':v=v+np.asarray(c.s.model.joints[leg+'_foot_joint'].origin_xyz)
        reach=float(np.linalg.norm(v,axis=1).max())
        if part in ('lower_leg_link','foot_link'):
            if radii[leg+'_lower_leg_joint']+1e-12<reach:raise SystemExit('invalid LOWER radius')
            reach+=float(np.linalg.norm(c.s.model.joints[leg+'_lower_leg_joint'].origin_xyz))
        if part!='hip_link':
            if radii[leg+'_upper_leg_joint']+1e-12<reach:raise SystemExit('invalid UPPER radius')
            reach+=float(np.linalg.norm(c.s.model.joints[leg+'_upper_leg_joint'].origin_xyz))
        if radii[leg+'_hip_joint']+1e-12<reach:raise SystemExit('invalid HIP radius')
    t0=time.monotonic();results={'non_adjacent_3mm':[],'base_link_ventral':[],'adjacent':[],'initial_adjacent_nesting':[]}
    for name,qa,qb in stages:
        ok,r=c.self_proof_q(qa,qb,.003,args.tube_rad);results['non_adjacent_3mm'].append(dict(stage=name,pass_=bool(ok),**r));print(name,'non_adjacent_3mm',ok,flush=True)
    # Proof B: ventral region of base_link (its lowest 15 mm, where the reprinted
    # IMU belly sits) against every non-adjacent leg link. Exact BVH distances
    # (python-fcl) at dense samples, then a Lipschitz bound: no leg point moves
    # more than sum_j radius_j * (half sample step_j + tube) between samples.
    import numpy as np,fcl
    tris=c.s.mesh('base_link').triangles_local;zmin=float(tris[:,:,2].min())
    # Include entire boundary-crossing faces conservatively. Selecting only
    # faces wholly below the cut would leave a hole in the claimed region.
    ventral=tris[tris[:,:,2].min(axis=1)<=zmin+.015];vv=ventral.reshape(-1,3)
    if not len(ventral):raise SystemExit('empty ventral region')
    def bvh(v):
        f=np.arange(len(v)).reshape(-1,3);b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();return b
    vmodel=bvh(vv);legs=[n for n in c.s.model.collision_link_names if n!='base_link' and not n.endswith('hip_link')]
    lmodel={n:bvh(c.s.mesh(n).triangles_local.reshape(-1,3)) for n in legs}
    def place(model,tf):return fcl.CollisionObject(model,fcl.Transform(np.asarray(tf.rotation),np.asarray(tf.translation)))
    samples=int(args.ventral_samples)
    for name,qa,qb in stages:
        worst=(math.inf,None,None)
        for k in range(samples+1):
            u=k/samples;q={j:qa[j]+u*(qb[j]-qa[j]) for j in SLOTS};bo=place(vmodel,c.s.collision_transform('base_link',q))
            for n in legs:
                d=fcl.distance(bo,place(lmodel[n],c.s.collision_transform(n,q)),fcl.DistanceRequest(),fcl.DistanceResult())
                margin=sum(r*(abs(qb[j]-qa[j])/(2*samples)+args.tube_rad) for j,r in c.radii[n].items())
                lower=d-margin-1e-9
                if lower<worst[0]:worst=(lower,n,dict(u=u,distance_m=d,lipschitz_margin_m=margin,roundoff_allowance_m=1e-9))
        need=.003+args.ventral_extra_m;ok=worst[0]>=need
        results['base_link_ventral'].append(dict(stage=name,pass_=bool(ok),method='fcl_exact_samples_plus_lipschitz',samples=samples,
          ventral_region='all base_link triangles touching z <= belly bottom + 15 mm; whole boundary faces included',ventral_triangles=int(len(ventral)),required_m=need,
          covered_links=legs,excluded_adjacent_hip_links=[n for n in c.s.model.collision_link_names if n.endswith('hip_link')],
          certified_lower_bound_m=worst[0],worst_link=worst[1],worst_sample=worst[2]))
        print(name,'base_link_ventral',ok,'lower bound %.2f mm'%(worst[0]*1e3),worst[1],flush=True)
    for j in SLOTS:
        values=[q0[j],q1[j],target[j]];lo=min(values)-args.tube_rad;hi=max(values)+args.tube_rad
        r=certify(c.s,j,lo,hi,args.adjacent_budget_s);results['adjacent'].append(dict(joint=j,domain_rad=[lo,hi],**{k:(list(v) if isinstance(v,tuple) else v) for k,v in r.items()}));print(j,r['status'],flush=True)
    # Surface separation alone cannot exclude pre-existing containment.
    # Check every closed mesh component at the actual entry start, without
    # removing hinge triangles or accepting intentional nesting exceptions.
    from check_assembly_nesting import components,inside
    representatives={};topology=[]
    for n in c.s.model.collision_link_names:
        representatives[n],count=components(c.s.mesh(n).triangles_local)
        topology.append(dict(link=n,closed=True,components=int(count)))
    for j in SLOTS:
        hinge=c.s.model.joints[j];parent,child=hinge.parent_link,hinge.child_link
        tf=c.s.collision_transform(parent,q0);child_tf=c.s.collision_transform(child,q0)
        points=lambda v:(child_tf.apply_points(v)-tf.translation)@tf.rotation
        a=c.s.mesh(parent).triangles_local;b=points(c.s.mesh(child).triangles_local.reshape(-1,3)).reshape(-1,3,3)
        rb=points(representatives[child])
        nested_parent=[int(i) for i,x in enumerate(representatives[parent]) if inside(x,b)]
        nested_child=[int(i) for i,x in enumerate(rb) if inside(x,a)]
        results['initial_adjacent_nesting'].append(dict(joint=j,pair=[parent,child],nested_parent_components=nested_parent,nested_child_components=nested_child,
          status='PASS_NO_NESTED_SURFACE' if not nested_parent and not nested_child else 'BLOCKED_NESTING_REVIEW_REQUIRED'))
    passed=all(x['pass_'] for x in results['non_adjacent_3mm']+results['base_link_ventral'])and all(x['status']=='PASS_CONTINUOUS' for x in results['adjacent']) and all(x['status']=='PASS_NO_NESTED_SURFACE' for x in results['initial_adjacent_nesting'])
    payload=dict(schema='MATDOG_COMMISSIONING_ENTRY_CERTIFICATE_V2',status='PASS_ENTRY_TUBE' if passed else 'BLOCKED',
      hardware_authorized=False,topology=topology,
      runtime_condition='full symmetric tube; no physical/command-limit clipping',
      modified_cover_adjacent_hip_verified=False,ventral_cover_all_pairs_certified=False,
      start_rad=start,start_deg=[math.degrees(x) for x in start],head_rad=[target[j] for j in SLOTS],slot_order=list(SLOTS),
      stages=['LOWER to head (HIP/UPPER held)','HIP+UPPER to head (LOWER held)'],tube_rad=args.tube_rad,
      non_adjacent_min_separation_m=.003,base_link_min_separation_m=.003+args.ventral_extra_m,ventral_cover_allowance_m=args.ventral_extra_m,
      lower_floor_deg={'front':args.front_floor_deg,'rear':args.rear_floor_deg},
      inputs={'urdf_sha256':sha(URDF),'route_json_sha256':sha(ROUTE),'firmware_route_data_sha256':sha(FIRMWARE),'tool_sha256':sha(__file__),
              'joint_log_sha256':sha(args.joint_log) if args.joint_log else None,
              'collision_meshes':meshes,
              'proof_sources_sha256':{p:sha(HERE/p) for p in ('continuous_pair_path.py','certify_adjacent_axis.py','check_assembly_nesting.py','paired_prone_model.py')},
              'mesh_kernel_sha256':sha(ROOT/'06_Software/Matdog_Core/calibration/matdog_geometry_mesh_kernel.py'),
              'geometry_scene_sha256':sha(ROOT/'06_Software/Matdog_Core/calibration/matdog_geometry_scene_v5.py'),
              'mesh_separation_bound_sha256':sha(ROOT/'06_Software/Matdog_Core/calibration/matdog_startup_geometry_v5.py'),
              'versions':{'python':platform.python_version(),'numpy':np.__version__,'python_fcl':getattr(fcl,'__version__','unknown')}},
      lipschitz_radius_m=c.radii,ventral_extra_interpretation='isotropic 3 mm dilation of selected original ventral surfaces for non-adjacent links; changed cover geometry and adjacent HIP mounts require separate physical/CAD inspection',
      results=results,elapsed_s=round(time.monotonic()-t0,1),
      scope='Rigid baseline-CAD self-clearance of both staged entry segments in the full +/- tube, plus 3 mm cover dilation against non-adjacent links only. Modified-cover-to-adjacent-HIP clearance, fixture/cable/battery free volume, mapping accuracy, loads, physical stop contact and actual tracking are NOT certified. Not a route, limit or hardware approval.')
    body=json.dumps(payload,sort_keys=True,separators=(',',':')).encode();digest=hashlib.sha256(body).hexdigest()
    os.umask(0o022);args.output.write_text(json.dumps(dict(certificate=payload,certificate_sha256=digest),indent=2)+'\n')
    urad=' '.join(str(round(x*1e6)) for x in start)
    print(payload['status'],digest)
    if passed:print(f'@MOTION COMMISSIONING ENTRY {urad} {round(args.tube_rad*1e6)} {digest}')
    return 0 if passed else 1
if __name__=='__main__':sys.exit(main())
