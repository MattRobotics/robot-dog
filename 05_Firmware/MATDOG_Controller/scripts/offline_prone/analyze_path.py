#!/usr/bin/env python3
"""Study a concrete reverse-route + inward-HIP landing candidate. OFFLINE ONLY."""
from geometry import *
from flat_bound import boxes,limits,floor,bound
from scipy.optimize import brentq
out=Path(sys.argv[1]);rows=list(csv.DictReader((out/'path.csv').open()));sampled=[]
for i,r in enumerate(rows):
 if i%10 and float(r['t'])!=0 and i+1<len(rows) and rows[i+1]['segment']==r['segment']:continue
 q=np.array([float(r[f'q{j}']) for j in range(12)]);s=rest3d(q);s.update(segment=int(r['segment']),t=float(r['t']),q=q.tolist());sampled.append(s)
final=rest3d(entry_q(.17));start=rest3d(entry_q(-math.pi/180));residuals=json.loads((Path(__file__).parent/'stand100_residuals.json').read_text());cases=[]
lo=np.array([j['limits_rad'][0] for j in limits]);hi=np.array([j['limits_rad'][1] for j in limits]);tick=2*math.pi/4096
for c in residuals['cases']:
 q=entry_q(.17)-np.array(residuals['directions'])*np.array(c['err'])*tick;s=rest3d(q)
 s['dev_ms']=c['dev_ms'];s['entry_margin_rad']=float(min((q-boxes['kEntry'][0]).min(),(boxes['kEntry'][1]-q).min()));s['joint_margin_rad']=float(min((q-lo).min(),(hi-q).min()));cases.append(s)
# Check the independent analytic floor expression against full URDF transforms,
# and interval upper bound against point evaluations. Non-hip leg subset only.
rng=np.random.default_rng(29);checks=0
for name,box in boxes.items():
 for leg,offset in [('lf',0),('rh',6)]:
  a=np.maximum(box[0,offset:offset+3],lo[offset:offset+3]);b=np.minimum(box[1,offset:offset+3],hi[offset:offset+3]);upper=bound(leg,a,b)
  for k in range(30):
   q=entry_q();v=rng.uniform(a,b);q[offset:offset+3]=v;V,_=geometry(q);expected=min(V[leg+'_'+part+'_link'][:,2].min() for part in ['upper_leg','lower_leg','foot'])
   assert abs(expected-floor(leg,v))<1e-12;assert expected<=upper+1e-12;checks+=1
summary={'classification':'IMPLEMENTED NOT VALIDATED: descent ends torque ON / no verified support','start':start,'final':final,'final_q_rad':entry_q(.17).tolist(),'segments':{},'residual_endpoint_cases':cases,'interval_expression_crosschecks':checks,'cad_mass_kg':sum(m for _,m,_ in inert),'assumed_mass_kg':2.8,'passive_settle_hypothesis':{'base_center_drop_mm':1000*final['z'],'rear_belly_gap_mm':1000*final['belly_max_gap'],'com_drop_fixed_q_mm':1000*(final['com'][2]-geometry(entry_q(.17))[1][2]),'potential_energy_fixed_q_J':2.8*9.80665*(final['com'][2]-geometry(entry_q(.17))[1][2]),'warning':'Flat pose at fixed q intersects the floor: these are scale estimates, NOT a physically possible passive trajectory.'}}
for group in range(3):
 rr=[r for r in rows if int(r['segment'])==group];ss=[r for r in sampled if r['segment']==group]
 summary['segments'][str(group)]={'duration_s':float(rr[-1]['t']),'samples':len(rr),'box_margin_rad':min(float(r['box_margin']) for r in rr),'joint_margin_rad':min(float(r['joint_margin']) for r in rr),'belly_minmax_mm':[1000*min(r['belly_gap'] for r in ss),1000*max(r['belly_gap'] for r in ss)],'minimum_support_margin_mm':1000*min(r['support_margin'] for r in ss),'minimum_nonfoot_nonbase_floor_mm':1000*min(v for r in ss for k,v in r['low'].items() if k!='base_link' and not k.endswith('foot_link'))}
# Reject foot-IK-only solutions which intersect the ground with other links.
locked=[]
for line in (out/'locked_contact.log').read_text().splitlines():
 if not line.startswith('CANDIDATE '):continue
 values=list(map(float,line.split()[1:]));pitch,xx,zz=values[:3];V,_=geometry(values[3:]);R=rot('0 1 0',pitch)
 lows={k:float((v@R.T)[:,2].min()+zz) for k,v in V.items()};link=min(lows,key=lows.get);locked.append({'pitch':pitch,'x':xx,'z':zz,'minimum_floor_m':lows[link],'limiting_link':link})
summary['locked_contact_alternative']={'kinematic_candidates':len(locked),'full_mesh_ground_clear':sum(r['minimum_floor_m']>=-1e-6 for r in locked),'best_minimum_floor_mm':1000*max(r['minimum_floor_m'] for r in locked),'warning':'Finite pose search, not a proof of impossibility of all contact-locked paths.'}
(out/'locked_contact_mesh_check.json').write_text(json.dumps(locked,indent=2)+'\n')
# These measured poses come from production SimpleMotion + flat-IMU host
# feedback scenarios. A resting plane is still a MODEL, not physical telemetry.
log=out/'tests/test_prone_descent.log'
if log.exists():
 ends=[rest3d(list(map(float,line.split(',')[1:]))) for line in log.read_text().splitlines() if line.startswith('ENDPOSE,')]
 summary['controller_endpoint_scenarios']={'count':len(ends),'belly_gap_mm_range':[1000*min(r['belly_gap'] for r in ends),1000*max(r['belly_gap'] for r in ends)],'support_margin_min_mm':1000*min(r['support_margin'] for r in ends)} if ends else {'count':0}
 (out/'controller_endpoint_rest.json').write_text(json.dumps(ends,indent=2)+'\n')
(out/'landing_analysis.json').write_text(json.dumps(summary,indent=2)+'\n');(out/'path_rest_samples.json').write_text(json.dumps(sampled,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='residual_endpoint_cases'},indent=2))
