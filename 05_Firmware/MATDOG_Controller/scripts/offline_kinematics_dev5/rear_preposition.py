"""REV01-FIX rigid study: from a flat-belly hold, place the rear legs while unloaded, then close the HIPs.
Compares longitudinal/lateral foot travel under contact with the frozen DEPLOY. Rigid static CAD only."""
from common import *
out=Path(sys.argv[1]);lib=bridge(out);floor=-1.5246163880149817
def st(q):
 r=rest3d(q);f=feet(lib,q,np.array([r['pitch'],0,r['z']]));cl={k:v*1000 for k,v in r['low'].items()}
 return r,f,cl
nf=lambda cl:min(v for n,v in cl.items() if n!='base_link' and not n.endswith('foot_link'))
print('--- hip scan, rear upper pi/2, lower -89deg')
for h in [.17,.25,.30,.35,.38,.40,.42,.44,.46,.50]:
 q=entry_q(.17,lf=floor);q[6]=-h;q[9]=h;r,f,cl=st(q);print('h %.2f pitch %.4f gap %.2f rearfoot %.2f frontfoot %.2f nonfoot %.2f support %.1f belly %.1f'%(h,r['pitch'],r['belly_max_gap']*1000,min(cl['rh_foot_link'],cl['lh_foot_link']),min(cl['rf_foot_link'],cl['lf_foot_link']),nf(cl),r['support_margin']*1000,r['belly_force_N_minmax'][0]),[c[:7] for c in r['contacts']],flush=True)
for K in [50,65,80,107]:
 k=knot(lib,K)[:12];print('--- knot',K,'rear',np.round(k[6:9],4),'front',np.round(k[:3],4))
 for h in [.70,.60,.55,.50,.45,.40,.30,.20,.10,abs(k[6])]:
  q=k.copy();q[6]=-h if k[6]<=0 else h;q[9]=-q[6]
  try:r,f,cl=st(q)
  except AssertionError:print(' h %.3f NOFACE'%h);continue
  print(' h %.3f pitch %.4f gap %.2f rearfoot %.2f nonfoot %.2f sep %.1f rear_y %.1f rear_x %.1f belly %.1f'%(h,r['pitch'],r['belly_gap']*1000,min(cl['rh_foot_link'],cl['lh_foot_link']),nf(cl),(f[0,0]-f[2,0])*1000,f[3,1]*1000,f[3,0]*1000,r['belly_force_N_minmax'][0]),[c[:7] for c in r['contacts']],flush=True)
