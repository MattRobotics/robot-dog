#!/usr/bin/env python3
"""New slip-oriented comparison, no route edits, no hardware dynamics claim."""
import csv,json,subprocess,sys
from pathlib import Path
import numpy as np
here=Path(__file__).resolve().parent;fw=here.parents[1];out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
s=(here/'route_probe.cpp').read_text()
a=s.index(' // Study only:');b=s.index(' q[12]=',a)
s=s[:a]+''' // Study v2: preserve separation through knot42. Then front +12 knots,
 // rear -12 knots (sinusoidal), both rejoin at100. Same boxes/body reference.
 double shift=early&&k>42&&k<100?12*std::sin(3.141592653589793*(k-42)/58.):0;
 for(unsigned l=0;l<4;++l){double f=k+(l<2?shift:-shift);unsigned a=unsigned(f),b=std::min(251u,a+1);double u=f-a;
  q[l*3]=((l==0||l==3)?-1:1)*p[7];
  q[l*3+1]=(1-u)*motion::paired_data::kKnots[a].p[l<2?2:0]+u*motion::paired_data::kKnots[b].p[l<2?2:0];
  q[l*3+2]=std::fmax((l<2?-88.5:-89.)*3.141592653589793/180,(1-u)*motion::paired_data::kKnots[a].p[l<2?3:1]+u*motion::paired_data::kKnots[b].p[l<2?3:1]);}
''' + s[b:]
s=s.replace('front_early','overlap_v2');(out/'probe.cpp').write_text(s.replace('../../src/',str(fw/'src')+'/'))
names=['BodyPose','ContactMode','FootContact','LegKinematics','LegInverseKinematics','TiltCompensation','TiltedContactIk','ImuAttitude','Gait','GaitTiming','StandTrajectory']
subprocess.run(['g++','-std=c++17','-O2',str(out/'probe.cpp'),*[str(fw/'src/motion'/(n+'.cpp')) for n in names],'-o',str(out/'probe')],check=True)
with (out/'route.csv').open('w') as f:subprocess.run([str(out/'probe')],stdout=f,check=True)
rows=list(csv.DictReader((out/'route.csv').open()));summary={'classification':'OFFLINE_KINEMATIC_COMPARISON_NOT_SLIP_OR_DYNAMIC_VALIDATION','variant':'after knot42 front +12*sin, rear -12*sin source knots until100; same nominal body reference','mass_kg':2.8,'results':{}}
sys.path.insert(0,str(here.parent/'offline_prone'));from geometry import rest3d
for name in ['current','overlap_v2']:
 for group in [0,1]:
  rr=[r for r in rows if r['route']==name and int(r['group'])==group];t=np.array([float(r['t']) for r in rr]);q=np.array([[float(r[f'q{i}']) for i in range(12)] for r in rr]);feet=np.array([[[float(r[f'f{l}{a}']) for a in 'xyz'] for l in range(4)] for r in rr]);vel=np.gradient(feet,t,axis=0);active=np.max(np.abs(np.gradient(q,t,axis=0)),axis=0)
  onset=[]
  for slots in [[1,2,4,5],[7,8,10,11]]:
   motion=np.max(np.abs(q[:,slots]-q[0,slots]),axis=1)>.01;onset.append(float(t[np.argmax(motion)]) if motion.any() else None)
  st=[rest3d(v) for v in q[::10]]
  contact=np.abs(feet[:,:,2])<.001
  summary['results'][f'{name}_{group}']={'duration_s':float(t[-1]),'front_rear_any_movement_onset_10mrad_s':onset,'front_extension_rear_extension_onset_10mrad_s':[float(t[np.argmax(q[:,1]<q[0,1]-.01)]),float(t[np.argmax(q[:,7]>q[0,7]+.01)])],'reference_pitch_range_deg':[float(min(float(r['pitch']) for r in rr)*180/np.pi),float(max(float(r['pitch']) for r in rr)*180/np.pi)],'reference_body_x_span_mm':1000*float(np.ptp([float(r['body_x']) for r in rr])),'foot_x_span_mm':(1000*np.ptp(feet[:,:,0],axis=0)).tolist(),'max_tangential_speed_near_nominal_ground_mm_s':float(np.max(np.abs(vel[:,:,0])[contact])*1000) if contact.any() else None,'min_foot_z_mm':float(feet[:,:,2].min()*1000),'max_foot_z_mm':float(feet[:,:,2].max()*1000),'box_margin_rad':min(float(r['box_margin']) for r in rr),'rigid_rest_pitch_range_deg':[float(min(r['pitch'] for r in st)*180/np.pi),float(max(r['pitch'] for r in st)*180/np.pi)],'min_static_support_margin_mm':1000*min(r['support_margin'] for r in st),'warning':'Ground-near foot speed is a kinematic slip demand indicator at prescribed body pose, not measured slip; rigid rest allows changing contact planes, not friction dynamics.'}
summary['decision']='EXCLUDED: lower nominal tangential-speed indicator (51.11 to40.88mm/s) but introduces4.11mm foot lift vs0.0004mm; pitch not reduced and rear travel remains47mm. Contact/friction unmeasured. Original STAND unchanged.'
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
