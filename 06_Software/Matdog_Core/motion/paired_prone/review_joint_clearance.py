#!/usr/bin/env python3
"""Read-only review of frozen route, calibrated contacts and surface witnesses."""
import json,math,re,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from paired_prone_model import PairModel,ROOT
BASE=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12'
OUT=ROOT/'09_Logs/Validation_Reports/Motion_Hardware_Readiness_2026-10-13'
raw=(BASE/'continuous_pair_path.json').read_bytes();d=json.loads(raw);m=PairModel();scene=m.scene
contacts=ROOT/'09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md'
lines=contacts.read_text().splitlines();joints=[]
# gen-1 was saved after the October 6 fresh calibration, not October 1.
# Historical reference below is review-only; runtime always reads accepted NVS.
gen1_q0={'lf':(1983,2095,2102),'rf':(2030,2096,1971),'rh':(2087,2061,2034),'lh':(2028,2092,2092)}
gen1_contacts={'lf':((2507,2507,1466,1464),(1455,1453,3478,3476),(3149,3149,1712,1712)),
 'rf':((2546,2547,1498,1497),(2730,2726,721,722),(937,936,2355,2355)),
 'rh':((1565,1564,2595,2595),(2667,2665,659,659),(992,991,2426,2424)),
 'lh':((1498,1499,2542,2543),(1481,1483,3484,3485),(3133,3134,1702,1705))}
for leg in ('lf','rf','rh','lh'):
 for part,label,field in [('hip','HIP','h'),('upper_leg','UPPER','uf' if leg in ('lf','rf') else 'ur'),('lower_leg','LOWER','lf' if leg in ('lf','rf') else 'lr')]:
  entries=[line.split('|')[1:-1] for line in lines if line.startswith('| '+leg.upper()+' | '+label+' | ')];entries=[list(map(str.strip,x)) for x in entries]
  entries=[x for x in entries if len(x)>=19 and x[2] in ("MIN","MAX")]
  assert len(entries)==2
  measured=[float(x[11]) for x in entries];fine_ticks=[int(x[9]) for x in entries];uncertainty=max(int(x[10].split()[0]) for x in entries)/2+1
  old_measured=measured
  kind={'hip':0,'upper_leg':1,'lower_leg':2}[part];fine=gen1_contacts[leg][kind];q0=gen1_q0[leg][kind];direction=int(entries[0][4])
  fine_ticks=[sum(fine[:2])/2,sum(fine[2:])/2];uncertainty=max(abs(fine[0]-fine[1]),abs(fine[2]-fine[3]))/2+1
  measured=[direction*(tick-q0)*360/4096 for tick in fine_ticks]
  name=leg+'_'+part+'_joint';g=scene.model.joints[name];sign=-1 if part=='hip' and leg in ('lf','lh') else 1
  vals=[sign*s[k][field] for s in d['segments'] for k in ('start_parameters','end_parameters')]
  # Explicit proposal only: 16 ticks rest/tracking precedent + fine repeatability
  # uncertainty. Does not qualify backlash, loaded deflection or installation.
  inset=(16+uncertainty)*360/4096
  candidate=[max(math.degrees(g.lower_limit_rad),min(measured)+inset),min(math.degrees(g.upper_limit_rad),max(measured)-inset)]
  lo,hi=map(math.degrees,(min(vals),max(vals)));margin=min(lo-candidate[0],candidate[1]-hi)
  byphase={}
  for s in d['segments']:
   values=[sign*s[k][field] for k in ('start_parameters','end_parameters')]
   row=byphase.setdefault(s['phase'],[math.inf,-math.inf]);row[0]=min(row[0],*values);row[1]=max(row[1],*values)
  joints.append(dict(joint=name,bus_id=int(entries[0][3].split('/')[1]),unit=entries[0][3].split('/')[0],direction=int(entries[0][4]),q0_historical_20261001=int(entries[0][5]),
    measured_contacts_deg=measured,october1_contacts_deg=old_measured,gen1_reference_q0_review_only=q0,measured_fine_ticks=fine_ticks,repeatability_uncertainty_ticks=uncertainty,
    proposal_inset_ticks=16+uncertainty,proposal_operational_deg=candidate,approved=False,
    route_range_deg=[lo,hi],route_margin_to_proposal_deg=margin,
    minimum_urdf_margin_deg=min(lo-math.degrees(g.lower_limit_rad),math.degrees(g.upper_limit_rad)-hi),phase_ranges_rad=byphase,
    status='ROUTE_INCOMPATIBLE_WITH_PROPOSAL' if margin<0 else 'CANDIDATE_REVIEW_REQUIRED',
    source='October 1 identity/direction + MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md gen-1 fresh q0/fine contacts; current installation must be independently matched',
    risk='initial LOWER at URDF stop and contact uncertainty' if part=='lower_leg' else 'LF UPPER MAX reduced spare travel beyond URDF; loaded margin unknown' if name=='lf_upper_leg_joint' else 'installation/zero/backlash/loaded tracking unknown',
    verification_required='current record integrity+generation+physical identity; neutral witness; supervised read-only encoder and measured linkage pose; rates/stop timing review'))
(OUT/'joint_limits_review.json').write_text(json.dumps(dict(schema='MATDOG_JOINT_REVIEW_V2',approved_count=0,no_historical_q0_runtime_import=True,route_sha256=hashlib.sha256(raw).hexdigest(),joints=joints),indent=2)+'\n')
# A pair of canonical surface vertices gives an UPPER bound on actual minimum
# distance. Below 3 mm is a proven counterexample, unlike a low lower bound.
witnesses=[]
for index in [0,len(d['segments'])//2,len(d['segments'])-1]:
 s=d['segments'][index];p=s['start_parameters'];q={}
 for leg in ('lf','rf','rh','lh'):
  q[leg+'_hip_joint']=p['h']*(-1 if leg in ('lf','lh') else 1);q[leg+'_upper_leg_joint']=p['uf' if leg in ('lf','rf') else 'ur'];q[leg+'_lower_leg_joint']=p['lf' if leg in ('lf','rf') else 'lr']
 points={n:scene.collision_transform(n,q).apply_points(scene.mesh(n).hull_vertices_local) for n in scene.model.collision_link_names}
 trees={n:cKDTree(v) for n,v in points.items()}
 best=None
 for a,b in scene.model.clearance_pairs():
  distance,other=trees[b].query(points[a]);i=int(np.argmin(distance));bound=float(distance[i])
  if best is None or bound<best['distance_upper_bound_m']:
   best=dict(segment=index,phase=s['phase'],pair=[a,b],distance_upper_bound_m=bound,canonical_surface_points_base_m=[points[a][i].tolist(),points[b][int(other[i])].tolist()])
 witnesses.append(best)
certificate_path=OUT/'nonadjacent_3mm.json'
certificate=json.loads(certificate_path.read_text()) if certificate_path.exists() else {}
qualified=certificate.get('policy_3mm_qualified') and certificate.get('source_certificate_sha256')==hashlib.sha256((BASE/'continuous_pair_path.json').read_bytes()).hexdigest()
lower=min(row['certificate']['minimum_swept_separation_lower_bound_m'] for row in certificate['segments']) if qualified else .000118303
(OUT/'operational_clearance.json').write_text(json.dumps(dict(schema='MATDOG_OPERATIONAL_CLEARANCE_V1',trajectory_changed=False,reference_m=.003,
 lower_bound_nonadjacent_m=lower,baseline_coarse_bound_m=.000118303,actual_minimum_exact_m=None,continuous_certificate='nonadjacent_3mm.json' if qualified else None,
 reference_status='PASS_CONTINUOUS_GEOMETRIC_REFERENCE' if qualified else 'FAIL_PROVEN_SURFACE_WITNESS' if min(x['distance_upper_bound_m'] for x in witnesses)<.003 else 'BLOCKED_EXACT_DISTANCE_UNDETERMINED',
 method='Continuous lower bound is separate from exact clearance; actual canonical surface vertex witness is a rigorous upper bound; convex-hull points are original STL vertices; no whole-link bound is mislabelled exact',
 adjacent_fit_is_separate_from_nonadjacent_operational_clearance=True,physical_loaded_clearance_qualified=False,witnesses=witnesses),indent=2)+'\n')
print('JOINT_REVIEW',len(joints),'approved=0');print('CLEARANCE_WITNESS',min(x['distance_upper_bound_m'] for x in witnesses))
