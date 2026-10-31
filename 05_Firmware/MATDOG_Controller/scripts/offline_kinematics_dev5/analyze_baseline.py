"""Production sampled route, endpoint invariants and existing-box flat-belly proof."""
from common import *
from flat_bound import prove
import csv
out=Path(sys.argv[1]);lib=bridge(out);result={};rows=[]
for name,ks,box in [('DEPLOY',[0,30,42,43,50,58,65,72,80,100,107],boxes['kDeploy']),('LIFT',[107,114,115,125,134,146,160],boxes['kLift'])]:
 via=np.array([knot(lib,k) for k in ks]);v=np.zeros(16);assert lib.sample(via,len(via),0,v);duration=v[15];samples=[]
 for t in np.linspace(0,duration,int(math.ceil(duration/.01))+1):
  assert lib.sample(via,len(via),t,v);q=v[:12].copy();p=v[12:15].copy();f=feet(lib,q,p);rr=rest(q);rf=feet(lib,q,np.array([rr['pitch'],0,rr['z']]))
  row={'phase':name,'t_s':t,'pitch_rad':p[0],'body_x_m':p[1],'body_z_m':p[2],'margin_ticks':margin(q,box),'rigid_pitch_rad':rr['pitch'],'rigid_belly_gap_m':rr['belly_gap']}
  for i,a in enumerate(q):row[f'q{i}']=a
  for l in range(4):
   for i,a in enumerate('xyz'):row[f'foot{l}_{a}_m']=f[l,i];row[f'rigid_foot{l}_{a}_m']=rf[l,i]
  rows.append(row);samples.append((q,f,rf,rr))
 a,b=samples[0],samples[-1];# Body X cancels in pair separation; radius from immutable FootContactData.
 delta_sep=((b[1][2,0]-b[1][0,0])-(a[1][2,0]-a[1][0,0]))*1000
 angular_delta=(b[0][7]+b[0][8]-a[0][7]-a[0][8])-(b[0][1]+b[0][2]-a[0][1]-a[0][2])
 result[name]={'duration_s':duration,'min_margin_ticks':min(r['margin_ticks'] for r in rows if r['phase']==name),'rear_reference_dx_mm':1000*(b[1][2,0]-a[1][2,0]),'front_reference_dx_mm':1000*(b[1][0,0]-a[1][0,0]),'relative_rear_front_dx_mm':delta_sep,'relative_foot_rotation_rad':angular_delta,'sagittal_cylinder_relative_material_slip_mm':delta_sep-14.9*angular_delta,'cylinder_assumption':'continuous contact, sagittal zero hip approximation (initial hip ±1 degree); not measured slip; sign = rear relative to front','rigid_relative_rear_front_dx_mm':1000*((b[2][2,0]-b[2][0,0])-(a[2][2,0]-a[2][0,0])),'rigid_belly_gap_mm_range':[min(s[3]['belly_gap'] for s in samples)*1000,max(s[3]['belly_gap'] for s in samples)*1000]}
with (out/'baseline-route.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
# Independent bound from old study, regenerated on the exact dev.5 geometry.
# For every RH configuration in ENTRY/gen1, some leg vertex protrudes >=8mm
# below z=0. Does not require optimizer convergence or a contact sensor.
lo=np.maximum(LOW[6:9],boxes['kEntry'][0,6:9]);hi=np.minimum(HIGH[6:9],boxes['kEntry'][1,6:9]);cert=prove('rh',lo,hi,-.005)
result['flat_belly_entry_certificate']={'leg':'RH','domain_lo':lo,'domain_hi':hi,**cert};assert cert['proved'];dump(out/'baseline-analysis.json',result);print(json.dumps(result,indent=2,default=lambda a:a.tolist()))
