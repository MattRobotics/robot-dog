"""Validated support-mode connectivity graph, from saved artifacts only.

Edges exist only for sampled routes whose every recorded sample passed the full offline policy.
Missing edges mean "no validated route found by the bounded searches", never proof of absence.
"""
import json
import numpy as np
from model import Model,sha
from survey import OUT,save

LEG=('lf','rf','rh','lh')
def load(name):return json.loads((OUT/name).read_text())
def qsha(q):return sha_bytes(np.round(np.array(q),12).tobytes())
def sha_bytes(b):
 import hashlib;return hashlib.sha256(b).hexdigest()[:16]

class Union:
 def __init__(self,items):self.parent={i:i for i in items}
 def find(self,x):
  while self.parent[x]!=x:self.parent[x]=self.parent[self.parent[x]];x=self.parent[x]
  return x
 def join(self,a,b):self.parent[self.find(a)]=self.find(b)

def route_metrics(m,frames,diag_lookup,source_frames):
 q=np.array([f['q'] for f in frames]);delta=np.diff(q,axis=0);dets=[];conds=[];margins=[];sv=[]
 for k,src in enumerate(source_frames):
  d=diag_lookup[src];dets.append([d['conditioning'][l]['jacobian_determinant'] for l in LEG]);conds.append([d['conditioning'][l]['condition_number'] for l in LEG]);margins.append(d['joint_margins_rad']);sv.append([min(d['conditioning'][l]['singular_values_m_per_rad']) for l in LEG])
 dets=np.array(dets);conds=np.array(conds);margins=np.array(margins)
 jm=int(np.argmin(margins.min(axis=1)));jj=int(np.argmin(margins[jm]));name=m.joint_order[jj];side='lower' if q[jm][jj]-m.limits[jj,0]<m.limits[jj,1]-q[jm][jj] else 'upper'
 seps=[f['min_separation']['distance_m'] for f in frames];si=int(np.argmin(seps));supports=[f['support_margin_m'] for f in frames];pi=int(np.argmin(supports))
 residual=[max(abs(f['ground_min_m'][l+'_foot_link']) for l in LEG) for f in frames];ri=int(np.argmax(residual))
 body=np.array([f['body'] for f in frames]);com=np.array([f['com_world_m'] for f in frames])
 sign_changes=np.count_nonzero(np.diff(np.sign(dets),axis=0),axis=0).tolist()
 contact_sets=[];regimes=[]
 for f in frames:
  key=(f['regime'],tuple(f['active_feet']),abs(f['ground_min_m']['base_link'])<=1e-5)
  if not contact_sets or contact_sets[-1]['key']!=list(key):contact_sets.append({'key':list(key),'first_frame':len(regimes),'frames':0})
  contact_sets[-1]['frames']+=1;regimes.append(f['regime'])
 return {'sample_count':len(frames),'start':{'frame':0,'body_translation_m':body[0][:3,3].tolist(),'body_height_m':float(body[0][2,3]),'regime':frames[0]['regime'],'active_feet':frames[0]['active_feet'],'base_link_ground_contact':abs(frames[0]['ground_min_m']['base_link'])<=1e-5,'q_rad':frames[0]['q'],'q_order':list(m.joint_order)},
  'final':{'frame':len(frames)-1,'body_translation_m':body[-1][:3,3].tolist(),'body_height_m':float(body[-1][2,3]),'regime':frames[-1]['regime'],'active_feet':frames[-1]['active_feet'],'base_link_ground_contact':abs(frames[-1]['ground_min_m']['base_link'])<=1e-5,'q_rad':frames[-1]['q']},
  'body_translation_delta_m':(body[-1][:3,3]-body[0][:3,3]).tolist(),'body_rotation_max_deviation_from_identity':float(np.max(abs(body[:,:3,:3]-np.eye(3)))),
  'com_world_start_m':com[0].tolist(),'com_world_final_m':com[-1].tolist(),
  'contact_set_segments':[{'regime':c['key'][0],'active_feet':c['key'][1],'base_link_ground_contact':c['key'][2],'first_frame':c['first_frame'],'frames':c['frames']} for c in contact_sets],
  'min_joint_margin_rad':float(margins.min()),'min_joint_margin_at':{'frame':jm,'joint':name,'limit_side':side,'q_rad':float(q[jm][jj]),'limit_rad':float(m.limits[jj,0 if side=='lower' else 1])},
  'per_joint_min_margin_rad':dict(zip(m.joint_order,margins.min(axis=0).tolist())),'per_joint_min_margin_frame':dict(zip(m.joint_order,margins.argmin(axis=0).tolist())),
  'min_nonadjacent_separation_m':float(min(seps)),'min_separation_at':{'frame':si,'pair':frames[si]['min_separation']['pair']},
  'min_static_support_margin_m':float(min(supports)),'min_support_margin_at_frame':pi,
  'max_abs_mesh_foot_ground_residual_m':float(max(residual)),'max_residual_at_frame':ri,'patch_band_tolerance_m':1e-5,
  'max_reference_xy_drift_m':None,
  'max_abs_joint_delta_rad_per_sample':float(np.max(abs(delta))),'max_abs_joint_delta_rad_per_joint':np.max(abs(delta),axis=0).tolist(),
  'jacobian_determinant':{'per_leg_min_abs':np.min(abs(dets),axis=0).tolist(),'per_leg_max_abs':np.max(abs(dets),axis=0).tolist(),'global_min_abs':float(np.min(abs(dets))),'sign_changes_per_leg':sign_changes,'legs':list(LEG),'scope':'central-difference analytic G2 cross-section-reference FK Jacobian, not the nonsmooth mesh-edge IK Jacobian'},
  'condition_number':{'per_leg_max':np.max(conds,axis=0).tolist(),'global_max':float(np.max(conds)),'min_singular_value_m_per_rad':float(np.min(sv))},
  'branch_continuity':'no Jacobian determinant sign change on any leg and maximum per-sample joint step above; the optimizer used the previous sample as seed. This diagnoses continuity of the sampled branch; no analytic IK branch id is assigned outside the G2 domain' if not any(sign_changes) else 'JACOBIAN_SIGN_CHANGE_RECORDED',
  'all_samples_valid':all(f['valid'] for f in frames),'continuous_collision_free_proved':False,'material_contact_locked_proved':False}

def main():
 m=Model();lib=load('pose_library.json');poses={p['name']:p for p in lib['poses']};rest=load('rest_search.json');modes=load('contact_modes.json');ext=load('route_extension.json');eq=load('equivalent_stand.json');trans=load('transitions.json');rc=load('rest_connectivity.json');lower=load('lower_envelope.json')
 cd=load('candidate_diagnostics.json');ed=load('equivalent_stand_diagnostics.json');lookup={(x['source'],x['pointer']):x for x in cd['records']+ed['records']}
 rises=ext['original_footprint_rises'];ends=eq['routes'];family=1
 rise=rises[family]['frames'];tail=ends[family]['frames'];assert rise[-1]['q']==tail[0]['q']
 frames=rise+tail[1:];src=[('route_extension.json','/original_footprint_rises/%d/frames/%d'%(family,j)) for j in range(len(rise))]+[('equivalent_stand.json','/routes/%d/frames/%d'%(family,j)) for j in range(1,len(tail))]
 metrics=route_metrics(m,frames,lookup,src)
 c=np.array([f['contacts'] for f in frames]);metrics['max_reference_xy_drift_m']=float(np.max(np.linalg.norm(c[:,:,:2]-c[0,:,:2],axis=2)))
 assert metrics['sample_count']==251 and abs(metrics['final']['body_height_m']-.15)<1e-12
 bf2=poses['BODY_FOUR_FEET_RESEARCH_2'];assert bf2['q']==frames[0]['q']
 stand=poses['STAND'];final=frames[-1]
 footprint_delta=np.linalg.norm(np.array(stand['contacts'])[:,:2]-np.array(final['contacts'])[:,:2],axis=1)
 stand_gap={'canonical_STAND_body_height_m':stand['body'][2][3],'equivalent_final_body_height_m':final['body'][2][3],'max_abs_joint_difference_rad':float(np.max(abs(np.array(stand['q'])-np.array(final['q'])))),'per_joint_difference_rad':(np.array(final['q'])-np.array(stand['q'])).tolist(),'reference_contact_xy_difference_per_foot_m':dict(zip(LEG,footprint_delta.tolist())),'max_reference_contact_xy_difference_m':float(footprint_delta.max()),'body_xy_offset_of_equivalent_stand_m':final['body'][0][3],
  'validated_edge':None,'reason':'different foot footprint and body offset; changing footprint requires foot lift/support transfer or sliding, neither validated'}
 near=[]
 for fam in lower['families']:
  pure=[s for s in fam['samples'] if s['valid'] and s['regime']=='FOOT_SUPPORT'][:1]
  for s in pure:near.append({'family':fam['family'],'lowest_valid_pure_foot_height_above_body_ground_m':s['height_above_body_ground_m'],'max_abs_joint_difference_to_route_frame_1_rad':float(np.max(abs(np.array(s['q'])-np.array(rise[1]['q'])))) if fam['family']==family else None})
 nodes={};edges=[]
 def node(name,kind,record,regime=None,note=None):
  nodes.setdefault(name,{'id':name,'kind':kind,'regime':regime or record.get('regime'),'body_height_m':record['body'][2][3] if record.get('body') else None,'joint_hash':qsha(record['q']),'note':note})
 for n,p in poses.items():node(n,'LIBRARY_POSE',p)
 node('CRAWL_READY','ALIAS_OF_LOW_CROUCH',poses['LOW_CROUCH'],note='library alias; envelope.json CRAWL_READY differs from LOW_CROUCH by ~3.2e-5 rad')
 rest_ids={}
 for pointer in range(len(rest['candidates'])):
  v=rest['candidates'][pointer]
  if not (v.get('valid') and v.get('regime')=='BODY_SUPPORT'):continue
  match=[n for n in ('REST_GROUND','REST_GROUND_MAX_SEPARATION') if poses[n]['q']==v['q']]
  name=match[0] if match else 'BODY_ONLY_REST_CANDIDATE_%d'%pointer;rest_ids[pointer]=name
  if not match:node(name,'BODY_ONLY_CANDIDATE',v)
 node('FOUR_FEET_RISE0_ORIGINAL_FOOTPRINT_END','ROUTE_ENDPOINT',rises[0]['frames'][-1]);node('EQUIVALENT_STAND_150MM_X10MM','ROUTE_ENDPOINT',final)
 for i in (0,1):node('FOOT_SUPPORT_60MM_FROM_BODY_FOUR_FEET_RESEARCH_%d'%(i+1),'ROUTE_ENDPOINT',modes['lift_off_attempts'][i]['frames'][-1])
 def edge(a,b,status,source,sample_count,evidence,validated_directions=('forward',),note=None):edges.append({'from':a,'to':b,'status':status,'validated':status=='VALIDATED_SAMPLED_ROUTE','source':source,'sample_count':sample_count,'validated_directions':list(validated_directions) if status=='VALIDATED_SAMPLED_ROUTE' else [],'evidence':evidence,'note':note,'continuous_collision_free_proved':False})
 names={'STRETCH_CANDIDATE':'STRETCH','SIT_CANDIDATE':'SIT'}
 for i,e in enumerate(trans['edges']):
  a,b=e['from'],names.get(e['to'],e['to']);ok=e['status']=='VALID_SEQUENCE_CANDIDATE'
  if b not in nodes:b=e['to']
  edge(a,b,'VALIDATED_SAMPLED_ROUTE' if ok else 'UNPROVEN_OR_REJECTED','transitions.json:/edges/%d'%i,len(e.get('frames',[])),{'reason':e.get('reason')},('forward','reverse_geometric') if ok and e.get('reversible_geometrically') else ('forward',))
 for i,l in enumerate(modes['lift_off_attempts']):
  edge('BODY_FOUR_FEET_RESEARCH_%d'%(i+1),'FOOT_SUPPORT_60MM_FROM_BODY_FOUR_FEET_RESEARCH_%d'%(i+1),'VALIDATED_SAMPLED_ROUTE' if l['valid'] else 'REJECTED','contact_modes.json:/lift_off_attempts/%d'%i,len(l['frames']),{'body_height_range_m':[.001,.06],'contact_model':'original footprint reference XY, mesh Z'})
 edge('BODY_FOUR_FEET_RESEARCH_1','FOUR_FEET_RISE0_ORIGINAL_FOOTPRINT_END','VALIDATED_SAMPLED_ROUTE','route_extension.json:/original_footprint_rises/0',len(rises[0]['frames']),{'end_body_height_m':rises[0]['frames'][-1]['body'][2][3],'stop':rises[0]['failure'] or 'IK failure at next 1 mm step recorded by generator (no invalid sample stored)','complete_stand_route':False})
 edge('BODY_FOUR_FEET_RESEARCH_2','EQUIVALENT_STAND_150MM_X10MM','VALIDATED_SAMPLED_ROUTE','route_extension.json:/original_footprint_rises/1/frames + equivalent_stand.json:/routes/1/frames',metrics['sample_count'],{'metrics':'route_251'},note='original-footprint rise to about 149 mm then body-XY shift to +10 mm at 150 mm; mesh support patch migrates')
 edge('EQUIVALENT_STAND_150MM_X10MM','STAND','NO_VALIDATED_ROUTE','none',0,stand_gap)
 for r in rc['starts']:
  name=rest_ids[int(r['source_pointer'].rsplit('/',1)[1])]
  for d in r['direct_contact_additions']:edge(name,d['target']+'@'+'+'.join(d['target_active_feet']),'REJECTED_BOUNDED_SEARCH','rest_connectivity.json',d['frames_evaluated'],{'first_failure':d['first_failure']})
  for d in r['body_supported_detours']:edge(name,'BODY_FOUR_FEET_RESEARCH_%d'%(d['target_family']+1),'VALIDATED_SAMPLED_ROUTE' if d['all_samples_valid'] else 'REJECTED_BOUNDED_SEARCH','rest_connectivity.json',d['frames_evaluated'],{'failure':d['failure'],'attempts':d['attempts']})
 legacy=[('contact_modes.json','contact_addition_attempts',a) for a in range(5)]
 for src_file,key,i in legacy:
  d=load(src_file)[key][i];edge('REST_GROUND_MAX_SEPARATION',d['to'],'REJECTED_BOUNDED_SEARCH','%s:/%s/%d'%(src_file,key,i),len(d['frames']),{'first_failure_errors':d['frames'][-1]['errors'],'collisions':d['frames'][-1]['collisions']},note='original seeded start was REST_GROUND_MAX_SEPARATION')
 def pose_name(record,fallback):
  match=[n for n,p in poses.items() if p['q']==record['q']];return match[0] if match else fallback
 for file,label in (('transfer_search.json','SUPPORT_TRANSFER_TARGET'),('support_transfer.json','SUPPORT_TRANSFER_TARGET')):
  for i,d in enumerate(load(file)['routes']):edge(pose_name(d['start'],file+'_start_%d'%i),label+'_UNRESOLVED','REJECTED_BOUNDED_SEARCH','%s:/routes/%d'%(file,i),len(d['frames']),{'all_samples_valid':d['all_samples_valid'],'kind':'bounded support-transfer attempt; not required for the retained routes'})
 edge('FOUR_FEET_RISE0_ORIGINAL_FOOTPRINT_END','EQUIVALENT_STAND_150MM_FAMILY_0','REJECTED_BOUNDED_SEARCH','equivalent_stand.json:/routes/0',len(ends[0]['frames']),{'failure':ends[0]['failure'],'contact_model':ends[0]['contact_model']})
 uf=Union(nodes)
 for e in edges:
  if e['validated']:
   assert e['from'] in nodes and e['to'] in nodes,(e['from'],e['to']);uf.join(e['from'],e['to'])
 groups={}
 for n in nodes:groups.setdefault(uf.find(n),[]).append(n)
 comps=sorted(groups.values(),key=lambda g:(-len(g),g))
 components=[{'component':i,'members':sorted(g),'size':len(g),'contains_STAND':'STAND' in g,'contains_REST_GROUND':'REST_GROUND' in g} for i,g in enumerate(comps)]
 stand=[c for c in components if c['contains_STAND']][0]
 result={'provenance':{'inputs':{n:sha(OUT/n) for n in ('pose_library.json','transitions.json','contact_modes.json','route_extension.json','equivalent_stand.json','rest_connectivity.json','rest_search.json','lower_envelope.json','candidate_diagnostics.json','equivalent_stand_diagnostics.json','transfer_search.json')},'generator_sha256':sha(__file__)},
  'definition':'nodes are validated static poses or route endpoints; an edge exists only if every recorded sample of a saved sampled route passes the full offline policy. Reverse traversal is geometric only where transitions.json marks reversible_geometrically; timing, contact acquisition, friction and swept collision are unvalidated.',
  'nodes':list(nodes.values()),'edges':edges,'components':components,
  'summary':{'component_count':len(components),'nontrivial_component_count':sum(c['size']>1 for c in components),'singleton_component_count':sum(c['size']==1 for c in components),'stand_component':stand['component'],'stand_component_members':stand['members'],'REST_GROUND_in_stand_component':'REST_GROUND' in stand['members'],'any_body_only_rest_in_stand_component':any(n in stand['members'] for n in nodes if n.startswith('BODY_ONLY_REST_CANDIDATE_') or n in ('REST_GROUND','REST_GROUND_MAX_SEPARATION')),'rest_starts_with_valid_connection':rc['starts_with_any_valid_connection'],
   'disconnection_meaning':'no validated route found by the bounded searches; not a proof that no route exists','singleton_components_are_untested_not_proven_disconnected':[c['members'][0] for c in components if c['size']==1]},
  'route_251_sample':{'family':family,'source':'BODY_FOUR_FEET_RESEARCH_2 -> EQUIVALENT_STAND_150MM_X10MM','metrics':metrics,'rise_end_body_height_m':rise[-1]['body'][2][3],'rise_frames':len(rise),'equivalent_shift_frames_excluding_duplicate':len(tail)-1,'limiting_constraint':'joint limit margin: %s %s limit at frame %d (%.6f rad); nonadjacent separation and support margin are looser'%(metrics['min_joint_margin_at']['joint'],metrics['min_joint_margin_at']['limit_side'],metrics['min_joint_margin_at']['frame'],metrics['min_joint_margin_rad'])},
  'equivalent_stand_vs_canonical_STAND':stand_gap,'lower_envelope_continuity_to_route':near,
  'startup_reassessment':'G3 autonomous startup gate unchanged; route uses inclined-edge mesh support with patch migration, not the G2 strip-contact contract, and no acquisition contract exists for base+four-foot start'}
 save('connectivity',result)
 print('CONNECTIVITY components',len(components),'stand',stand['members'],'route',metrics['sample_count'],flush=True)
if __name__=='__main__':main()
