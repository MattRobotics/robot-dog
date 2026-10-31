"""Dev.5 terminal residuals applied to counterfactual endpoints, not a motion replay."""
from common import *
import itertools
from flat_bound import floor
out=Path(sys.argv[1]);source=Path(sys.argv[2]);v=json.loads((out/'prone-variants.json').read_text());nom=np.array(v['dev5']['q']);actual=[]
for line in source.read_text().splitlines():
 s=json.loads(line)['status']
 if s['phase']=='PRONE_CONTACT_UNVERIFIED' and not s['stale'] and len(s['joints'])==12 and all(j['valid'] and j['age_ms']<=250 for j in s['joints']):actual.append((s['now_ms'],np.array(s['telemetry']['actual_rad'])))
assert len(actual)>=12
result={'classification':'COUNTERFACTUAL_STATIC_ENDPOINTS_NOT_HARDWARE_OR_DYNAMIC_REPLAY','samples':len(actual),'source':str(source),'variants':{}}
for name,var in v.items():
 target=np.array(var['q']);rr=[]
 for ms,q in actual:
  newq=q+target-nom;r=rest3d(newq);r.update(ms=ms,margin_ticks=margin(newq,boxes['kEntry']));rr.append(r)
 med=np.median([a[1] for a in actual],axis=0)+target-nom;corners=[]
 # 64 mirrored pair perturbations around median, not exhaustive 12D proof.
 for signs in itertools.product([-1,1],repeat=6):
  d=np.zeros(12)
  for leg in range(4):
   pair=0 if leg<2 else 1;sgn=1 if leg in [0,3] else -1
   d[leg*3:leg*3+3]=np.array(signs[pair*3:pair*3+3])*4*TICK;d[leg*3]*=sgn
  r=rest3d(med+d);corners.append(r)
 res={'gap_mm_range':[1000*min(r['belly_max_gap'] for r in rr),1000*max(r['belly_max_gap'] for r in rr)],'min_joint_box_margin_ticks':min(r['margin_ticks'] for r in rr),'margin_after_independent_plusminus4_ticks':min(r['margin_ticks'] for r in rr)-4,'min_nonfoot_floor_mm':1000*min(x for r in rr for n,x in r['low'].items() if n!='base_link' and not n.endswith('foot_link')),'min_support_mm':1000*min(r['support_margin'] for r in rr),'belly_N_range':[min(r['belly_force_N_minmax'][0] for r in rr),max(r['belly_force_N_minmax'][1] for r in rr)],'64corner_min_nonfoot_floor_mm':1000*min(x for r in corners for n,x in r['low'].items() if n!='base_link' and not n.endswith('foot_link')),'64corner_max_rear_gap_mm':1000*max(r['belly_max_gap'] for r in corners)};result['variants'][name]=res
 print(name,json.dumps(res),flush=True)
# Independently verify the reduced-coordinate RH floor formula used in the
# interval certificate against all transformed CAD upper/lower/foot vertices.
rng=np.random.default_rng(30);error=0
for _ in range(100):
 q=nom.copy();q[6:9]=rng.uniform(np.maximum(LOW,boxes['kEntry'][0])[6:9],np.minimum(HIGH,boxes['kEntry'][1])[6:9]);V,_=geometry(q);ref=min(float(V[n][:,2].min()) for n in ['rh_upper_leg_link','rh_lower_leg_link','rh_foot_link']);error=max(error,abs(ref-floor('rh',q[6:9])))
assert error<1e-12;result['floor_model_random_checks']=100;result['floor_model_max_error_m']=error;dump(out/'counterfactual-endpoints.json',result)
