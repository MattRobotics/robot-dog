"""Revalidate selected MATDOG research targets; no imported XGO joint geometry."""
import json,math
import numpy as np
from model import Model,LEGS,sha
from diagnostics import diagnostics
from survey import OUT,save

def build():
 m=Model();env=json.loads((OUT/'envelope.json').read_text());rest=json.loads((OUT/'rest_search.json').read_text());expanded=json.loads((OUT/'expanded_envelope.json').read_text());modes=json.loads((OUT/'contact_modes.json').read_text())
 candidates=[r for r in rest['candidates'] if r.get('valid') and r.get('regime')=='BODY_SUPPORT']
 balanced=max(candidates,key=lambda r:(r['joint_margin_rad'],r['min_separation']['distance_m'],r['q'][7]))
 selected=[('REST_GROUND',balanced,'rest_search.json: joint-margin-first body-only candidate'),('REST_GROUND_MAX_SEPARATION',rest['selected'],'rest_search.json:selected; reaches a joint limit')]
 for name in ('LOW_CROUCH','LOW_C4','STAND','STRETCH_CANDIDATE','ROLL_PREPARATION'):
  selected.append(({'STRETCH_CANDIDATE':'STRETCH','ROLL_PREPARATION':'ROLL_PREP_RESEARCH'}.get(name,name),env['poses'][name],'envelope.json:poses/'+name))
 seated=[r for r in expanded['seated_candidates'] if r['valid']];sit=min(seated,key=lambda r:math.atan2(-r['body'][2][0],r['body'][0][0]))
 selected.append(('PITCHED_CROUCH_RESEARCH',sit,'expanded_envelope.json:most negative valid pitch; not recovered XGO SIT'))
 selected.append(('XGO_HEIGHT_RATIO_RESEARCH',env['poses']['XGO_HEIGHT_LOW_INTENT'],'envelope.json:XGO_HEIGHT_LOW_INTENT; assumed 60/110 host-height ratio, not recovered ground datum'))
 for i,lift in enumerate(modes['lift_off_attempts']):selected.append(('BODY_FOUR_FEET_RESEARCH_'+str(i+1),lift['endpoint'],'contact_modes.json:lift_off_attempts/'+str(i)+'/endpoint'))
 records=[]
 for name,r,source in selected:
  v=m.evaluate(r['q'],np.array(r['body']),r['regime'],r['active_feet'])
  if not v['valid']:raise ValueError((name,v['errors']))
  d=diagnostics(m,v['q'],v['body']);L=.09+math.hypot(.107,.0499);t=m.fk(v['q'],np.array(v['body']));relative=[]
  for i,l in enumerate(LEGS):
   hip=np.array(v['body'])@np.r_[m.joints[l+'_hip_joint'].origin_xyz,1]
   relative.append((np.array(v['body'])[:3,:3].T@(np.array(v['contacts'][i])-hip[:3])/L).tolist())
  evidence='RESEARCH_ONLY' if 'RESEARCH' in name else 'VALIDATED_STATIC'
  records.append({'name':name,'evidence':evidence,'source':source,'scope':'canonical rigid CAD geometry and quasi-static URDF COM; not physical startup evidence','embedded_target':evidence=='VALIDATED_STATIC',**v,'diagnostics':d,'normalized':{'characteristic_length_m':L,'length_definition':'upper axis separation plus lower-to-foot-origin vector norm; foot cylinder excluded','body_translation_over_L':(np.array(v['body'])[:3,3]/L).tolist(),'world_from_body_rotation':np.array(v['body'])[:3,:3].tolist(),'contact_relative_to_hip_in_body_over_L':relative,'joint_order':'LF/RF/RH/LH hip/upper/lower URDF radians','support_regime':v['regime'],'active_feet':v['active_feet'],'transferable':'intent, orientation and dimensionless geometry after explicit frame binding; joint angles and contact mechanics require MATDOG re-solve'}})
 return {'provenance':{'canonical_sources':m.sources,'generator_sha256':sha(__file__),'model_sha256':sha(__import__('pathlib').Path(__file__).with_name('model.py')),'diagnostics_sha256':sha(__import__('pathlib').Path(__file__).with_name('diagnostics.py')),'inputs':{n:sha(OUT/n) for n in ['envelope.json','rest_search.json','expanded_envelope.json','contact_modes.json']}},'poses':records,'aliases':{'CRAWL_READY':'LOW_CROUCH','SQUAT':'LOW_CROUCH'},'startup_authority':False,'rejected_or_unknown':[{'name':'SIT','reason':'pitched crouch exists, but XGO SIT physical endpoint/support is unbound; initial -0.3 rad, 70 mm candidate IK failed'},{'name':'BEG','reason':'tested -0.6 rad, 70 mm candidate IK failed; two-foot static support not validated'},{'name':'MARK_TIME','reason':'preset 5 is completion-only in analyzed firmware; separate mark-time API geometry not recovered'},{'name':'CALIBRATION_REFERENCE','reason':'q=0 is a reference, not a validated ground-supported target'},{'name':'XGO_DIRECT_JOINT_TABLE','reason':'controller bytes have no proven physical sign/zero binding to MATDOG'}]}

def main():
 d=build();save('pose_library',d);print('library',[(x['name'],x['joint_margin_rad'],x['min_separation']['distance_m']) for x in d['poses']],flush=True)
if __name__=='__main__':main()
