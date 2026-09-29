"""Summarize sampled continuity separately from actuator rates and path proof."""
import json
import numpy as np
from model import sha,Model
from survey import OUT,save
from diagnostics import diagnostics

def summarize(frames):
 frames=[r for r in frames if 'q' in r];q=np.array([r['q'] for r in frames]);delta=np.diff(q,axis=0)
 if not frames:return {'sample_count':0}
 active0=set(frames[0]['active_feet']);active=set.intersection(*(set(r['active_feet']) for r in frames));c=np.array([r['contacts'] for r in frames]);reference=c[0];fixed=[('lf','rf','rh','lh').index(l) for l in active]
 return {'sample_count':len(frames),'per_frame_joint_delta_rad':delta.tolist(),'max_abs_joint_delta_rad_per_joint':np.max(abs(delta),axis=0).tolist() if len(delta) else [0]*12,'min_joint_margin_rad':min(r['joint_margin_rad'] for r in frames),'min_static_support_margin_m':min(r['support_margin_m'] for r in frames if r['support_margin_m'] is not None),'max_reference_xy_drift_m':float(np.max(np.linalg.norm(c[:,fixed,:2]-reference[fixed,:2],axis=2))) if fixed else None,'minimum_nonadjacent_separation_m':min(r['min_separation']['distance_m'] for r in frames),'all_recorded_samples_valid':all(r['valid'] for r in frames),'actuator_rate_limit':None,'timing':'untimed geometric samples; deltas are radians/sample, not hardware rate limits'}
def main():
 m=Model();di=json.loads((OUT/'candidate_diagnostics.json').read_text());lookup={(x['source'],x['pointer']):x for x in di['records']};rows=[];inputs={}
 groups=[('transitions','edges'),('contact_modes','contact_addition_attempts'),('contact_modes','lift_off_attempts'),('transfer_search','routes'),('support_transfer','routes'),('route_extension','original_footprint_rises'),('route_extension','body_supported_detours'),('equivalent_stand','routes')]
 for file,key in groups:
  p=OUT/(file+'.json');d=json.loads(p.read_text());inputs[p.name]=sha(p)
  # Retain original search naming when older attempts use another top-level key.
  if key not in d:
   if file=='transfer_search':key='attempts'
   else:continue
  for i,route in enumerate(d[key]):
   frames=route.get('frames',[]);summary=summarize(frames);branches=[];patch=[]
   for j,r in enumerate(frames):
    if 'q' not in r:continue
    record=lookup.get((p.name,'/'+key+'/'+str(i)+'/frames/'+str(j)))
    diag=record if record else diagnostics(m,r['q'],r['body'],r.get('collisions',[]))
    branches.append([diag['conditioning'][l]['jacobian_determinant'] for l in ('lf','rf','rh','lh')]);patch.append([diag['feet'][l]['lowest_patch_centroid_world_m'] for l in ('lf','rf','rh','lh')])
   if len(branches)>1:
    signs=np.sign(branches);summary['jacobian_orientation_sign_changes_per_leg']=np.count_nonzero(np.diff(signs,axis=0),axis=0).tolist();summary['max_patch_centroid_xy_migration_m']=float(np.max(np.linalg.norm(np.array(patch)[:,:,:2]-np.array(patch)[0,:,:2],axis=2)))
   summary['branch_scope']='offline optimizer uses previous valid q; determinant sign and joint deltas diagnose continuity, no analytic branch ID is assigned outside G2 domain'
   collisions=[]
   for j,r in enumerate(frames):
    if r.get('collisions'):collisions.append({'frame_index':j,'progress':j/50 if key=='contact_addition_attempts' else None,'pairs':r['collisions'],'details':diagnostics(m,r['q'],r['body'],r['collisions'])['collision_details']})
   rows.append({'source':p.name,'pointer':'/'+key+'/'+str(i),'from':route.get('from'),'to':route.get('to'),'search_result':route.get('status',route.get('all_samples_valid',route.get('valid'))),'summary':summary,'collisions':collisions,'continuous_collision_free_proved':False})
 rises=json.loads((OUT/'route_extension.json').read_text())['original_footprint_rises'];ends=json.loads((OUT/'equivalent_stand.json').read_text())['routes'];complete=[]
 for i,end in enumerate(ends):
  if not end['all_samples_valid']:continue
  first=rises[i]['frames'];last=end['frames'];assert first[-1]['q']==last[0]['q'];frames=first+last[1:]
  complete.append({'family':i,'frame_sources':['route_extension.json:/original_footprint_rises/'+str(i)+'/frames','equivalent_stand.json:/routes/'+str(i)+'/frames (omit duplicate first)'],'summary':summarize(frames),'final_body':frames[-1]['body'],'max_abs_mesh_foot_ground_residual_m':max(abs(r['ground_min_m'][l+'_foot_link']) for r in frames for l in ('lf','rf','rh','lh')),'contact_events':[{'frame':0,'regime':'BODY_AND_FOOT_SUPPORT','base_ground':True},{'frame':1,'regime':'FOOT_SUPPORT','base_ground':False}],'status':'VALID_SEQUENCE_CANDIDATE','continuous_collision_free_proved':False,'reference_xy_locked':True,'material_contact_locked_proved':False})
 save('transition_graph',{'provenance':{'inputs':inputs,'diagnostics_sha256':sha(OUT/'candidate_diagnostics.json'),'generator_sha256':sha(__file__)},'edges':rows,'complete_sampled_routes':complete,'contact_modes':['BODY_SUPPORT','BODY_AND_FOOT_SUPPORT','FOOT_SUPPORT','FREE_SPACE','TRANSITIONAL_SUPPORT'],'startup_reassessment':'STILL_REQUIRED','reason':'Body-only folded contact additions fail; broad body+feet path uses inclined edge contacts with patch migration outside G2 startup contract. Static endpoints and sampled mesh IK are not verified startup acquisition evidence.'})
 print('TRANSITION_SUMMARY',len(rows))
if __name__=='__main__':main()
