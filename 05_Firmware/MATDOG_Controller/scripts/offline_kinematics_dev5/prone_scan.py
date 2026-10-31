"""REV01-FIX rigid PRONE endpoint vs rear UPPER offset; dev.5 front LOWER floor. Not a dynamic or hardware result."""
from common import *
out=Path(sys.argv[1]);lib=bridge(out);floor=-1.5246163880149817;rows=[]
for hip in [.17]:
 for d in [0,-.02,-.05,-.075,-.10,-.125,-.15,-.175,-.195,.05,.10]:
  q=entry_q(hip,ur=math.pi/2+d,lr=-89*math.pi/180,lf=floor);r=rest3d(q);f=feet(lib,q,np.array([r['pitch'],0,r['z']]))
  non=min(v for n,v in r['low'].items() if n!='base_link' and not n.endswith('foot_link'))*1000
  ff=min(r['low'][n] for n in ('lf_foot_link','rf_foot_link'))*1000
  rows.append(dict(hip=hip,rear_upper_delta_rad=d,pitch=r['pitch'],gap_mm=r['belly_max_gap']*1000,nonfoot_mm=non,front_foot_mm=ff,sep_mm=(f[0,0]-f[2,0])*1000,support_mm=r['support_margin']*1000,belly_N=r['belly_force_N_minmax'],margin_ticks=margin(q,boxes['kEntry']),contacts=r['contacts']))
  print('d %+.3f pitch %.4f gap %.2f nonfoot %.2f frontfoot %.2f sep %.1f support %.1f belly %.1f margin %.1f %s'%(d,r['pitch'],r['belly_max_gap']*1000,non,ff,rows[-1]['sep_mm'],r['support_margin']*1000,r['belly_force_N_minmax'][0],rows[-1]['margin_ticks'],[c.replace('_link','') for c in r['contacts']]),flush=True)
dump(out/'prone-scan.json',rows)
