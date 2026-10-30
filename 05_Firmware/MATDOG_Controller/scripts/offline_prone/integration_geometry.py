#!/usr/bin/env python3
"""Delta-only geometric checks for dev.3; rigid static model, NOT landing validation."""
from geometry import *
import subprocess
here=Path(__file__).resolve().parent;fw=here.parents[1];out=Path(sys.argv[1]);out.mkdir(exist_ok=True,parents=True)
names=['BodyPose','ContactMode','FootContact','Gait','GaitTiming','ImuAttitude','LegKinematics','LegInverseKinematics','StandTrajectory','TiltCompensation','TiltedContactIk','MotionContract']
subprocess.run(['g++','-std=c++17','-O2',str(here/'path_probe.cpp'),*[str(fw/'src/motion'/(n+'.cpp')) for n in names],'-o',str(out/'path_probe')],check=True)
with (out/'path.csv').open('w') as f:subprocess.run([str(out/'path_probe')],stdout=f,check=True)
rows=list(csv.DictReader((out/'path.csv').open()));sample=[]
for i,r in enumerate(rows):
 if i%10==0 or i==len(rows)-1 or float(r['t'])==0:
  q=[float(r[f'q{j}']) for j in range(12)];s=rest3d(q);s.update(segment=r['segment'],t=r['t']);sample.append(s)
ends=[]
for l in (out.parent/'tests-corrected/test_prone_hardware_replay.log').read_text().splitlines():
 if l.startswith('REAL_END,'):
  a=l.split(',');s=rest3d(list(map(float,a[3:])));s.update(height=int(a[1]),ms=int(a[2]));ends.append(s)
final=rest3d(entry_q(.17,lf=-1.5246163880149817));summary={'scope':'static full-mesh rest and existing box proof; no contact/friction/dynamics validation','front_lower_delta_rad':.020,'final':final,'sample_count':len(sample),'minimum_support_margin_mm':1000*min(r['support_margin'] for r in sample),'minimum_nonfoot_nonbase_floor_mm':1000*min(v for r in sample for k,v in r['low'].items() if k!='base_link' and not k.endswith('foot_link')),'segments':{},'replay_endpoints':{}}
for g in range(3):
 a=[r for r in rows if int(r['segment'])==g];summary['segments'][g]={'duration_s':float(a[-1]['t']),'box_margin_rad':min(float(r['box_margin']) for r in a),'mechanical_margin_rad':min(float(r['joint_margin']) for r in a)}
for h in [50,100,150]:
 a=[r for r in ends if r['height']==h];summary['replay_endpoints'][h]={'count':len(a),'belly_min_gap_mm_range':[1000*min(r['belly_gap'] for r in a),1000*max(r['belly_gap'] for r in a)],'rear_gap_mm_range':[1000*min(r['belly_max_gap'] for r in a),1000*max(r['belly_max_gap'] for r in a)],'support_margin_min_mm':1000*min(r['support_margin'] for r in a)}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(out/'static_path_samples.json').write_text(json.dumps(sample,indent=2)+'\n');(out/'replay_endpoint_rest.json').write_text(json.dumps(ends,indent=2)+'\n');print(json.dumps(summary,indent=2))
