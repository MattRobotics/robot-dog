"""Independent FK/COM, mesh and evidence gates for G3.5 research."""
import unittest,json,copy,math,re,subprocess
from pathlib import Path
import numpy as np
from model import Model,LEGS,transform,sha
from survey import OUT,general_ik
from pose_export import check_sources,render,TARGET
from retarget import build,classify
from matdog_urdf_fk import forward_kinematics
from diagnostics import nearest_patch_point
class AuditTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.m=Model();cls.data=json.loads((OUT/'pose_library.json').read_text())
 def test_freshness(self):
  check_sources(self.data);self.assertEqual(render(self.data),TARGET.read_text());mut=copy.deepcopy(self.data);mut['provenance']['canonical_sources'][next(iter(mut['provenance']['canonical_sources']))]='invalid'
  with self.assertRaises(ValueError):check_sources(mut)
 def test_independent_fk_and_com(self):
  for p in self.data['poses']:
   q=dict(zip(self.m.joint_order,p['q']));body=np.array(p['body']);total=0.;weighted=np.zeros(3);tf=self.m.fk(p['q'],body)
   for name,(mass,offset) in self.m.inertial.items():
    t=body@np.array(forward_kinematics(self.m.urdf,'base_link',name,q).tip_transform)
    np.testing.assert_allclose(t,tf[name],rtol=0,atol=2e-15);weighted+=mass*(t@np.r_[offset,1])[:3];total+=mass
   np.testing.assert_allclose(weighted/total,p['com_world_m'],rtol=0,atol=2e-15);self.assertAlmostEqual(total,2.48,12)
 def test_accepted_geometry(self):
  for p in self.data['poses']:
   v=self.m.evaluate(p['q'],np.array(p['body']),p['regime'],p['active_feet']);self.assertTrue(v['valid'],(p['name'],v['errors']));self.assertGreaterEqual(v['joint_margin_rad'],-1e-12);self.assertGreater(v['support_margin_m'],0)
 def test_asymmetry(self):
  self.assertAlmostEqual(self.m.joints['lf_hip_joint'].origin_xyz[2]-self.m.joints['rh_hip_joint'].origin_xyz[2],.020,14)
  p=next(x for x in self.data['poses'] if x['name']=='LOW_C4');self.assertGreater(np.max(abs(np.array(p['q'][1:3])-p['q'][7:9])),.01)
 def test_six_promoted_poses_exact_contract(self):
  expected={'REST_GROUND':('BODY_SUPPORT',0.),'REST_GROUND_MAX_SEPARATION':('BODY_SUPPORT',0.),'LOW_CROUCH':('FOOT_SUPPORT',.06),'LOW_C4':('FOOT_SUPPORT',.10),'STAND':('FOOT_SUPPORT',.15),'STRETCH':('FOOT_SUPPORT',.10)}
  promoted={p['name']:p for p in self.data['poses'] if p['embedded_target']};self.assertEqual(set(promoted),set(expected))
  for name,(regime,height) in expected.items():
   p=promoted[name];self.assertEqual(p['evidence'],'VALIDATED_STATIC');self.assertEqual(p['regime'],regime);self.assertAlmostEqual(p['body'][2][3],height,12);self.assertTrue(p['valid']);self.assertEqual(p['collisions'],[])
   self.assertEqual(p['active_feet'],[] if regime=='BODY_SUPPORT' else list(LEGS));self.assertEqual(p['base_contact'],regime=='BODY_SUPPORT')
  for p in self.data['poses']:
   if p['name'] not in expected:self.assertEqual(p['evidence'],'RESEARCH_ONLY');self.assertFalse(p['embedded_target'])
 def test_support_policies(self):
  p=self.data['poses'][0];b=np.array(p['body']);q=p['q']
  self.assertTrue(self.m.evaluate(q,b,'BODY_SUPPORT')['valid'])
  v=self.m.evaluate(q,b,'FOOT_SUPPORT',LEGS);self.assertIn('UNDECLARED_GROUND_CONTACT:base_link',v['errors'])
  self.assertIn('INVALID_SUPPORT_DECLARATION',self.m.evaluate(q,b,'BODY_SUPPORT',['lf'])['errors'])
  self.assertTrue(self.m.evaluate(q,b,'TRANSITIONAL_SUPPORT',base_contact=True)['valid'])
  self.assertIn('INVALID_SUPPORT_DECLARATION',self.m.evaluate(q,b,'TRANSITIONAL_SUPPORT')['errors'])
  moved=b.copy();moved[2,3]-=.001;self.assertIn('GROUND_PENETRATION:base_link',self.m.evaluate(q,moved,'BODY_SUPPORT')['errors'])
  high=b.copy();high[2,3]=.5;self.assertTrue(self.m.evaluate(q,high,'FREE_SPACE')['valid'])
  with self.assertRaises(ValueError):self.m.evaluate([0]*11,b,'BODY_SUPPORT')
  bad=b.copy();bad[0,0]=2
  with self.assertRaises(ValueError):self.m.evaluate(q,bad,'BODY_SUPPORT')
 def test_deterministic_retarget_and_ik(self):
  self.assertEqual(build(),build());p=next(x for x in self.data['poses'] if x['name']=='STRETCH');targets=np.array(p['contacts']);targets[:,2]=0;b=np.array(p['body'])
  a=general_ik(self.m,targets,b,p['q']);c=general_ik(self.m,targets,b,p['q']);np.testing.assert_array_equal(a,c)
  for l in LEGS:
   t=self.m.fk(a,b)[l+'_foot_link'];self.assertLess(abs(np.min(self.m.meshes[l+'_foot_link'].vertices@t[2,:3]+t[2,3])),1e-8)
 def test_evidence_cannot_promote(self):
  cat=json.loads((OUT/'xgo_catalogue.json').read_text())
  for action in cat['actions']:
   self.assertIn(classify(action),['NOT_TRANSFERABLE','RETARGET_UNDERDETERMINED']);a=copy.deepcopy(action);a['evidence_class']='A';self.assertEqual(classify(a),'RETARGET_UNDERDETERMINED')
  self.assertFalse(self.data['startup_authority']);self.assertEqual(len([p for p in self.data['poses'] if p['embedded_target']]),6)
 def test_height_bound(self):
  e=json.loads((OUT/'expanded_envelope.json').read_text());rng=np.random.default_rng(71)
  for _ in range(50):
   q=rng.uniform(self.m.limits[:,0],self.m.limits[:,1]);tf=self.m.fk(q)
   for l in LEGS:
    t=tf[l+'_foot_link'];extent=-np.min(self.m.meshes[l+'_foot_link'].vertices@t[2,:3]+t[2,3]);self.assertLessEqual(extent,e['per_leg_upper_bounds_m'][l]+1e-12)
 def test_collision_exclusions(self):
  self.assertEqual(len(self.m.pairs),120);self.assertEqual(len(self.m.excluded),16);self.assertNotIn(tuple(sorted(('base_link','lf_foot_link'))),self.m.excluded)
 def test_contact_patch_distance(self):
  square=np.array([[-1.,-1.,0.],[-1.,1.,0.],[1.,-1.,0.],[1.,1.,0.]])
  np.testing.assert_allclose(nearest_patch_point(square,[0,0,2]),[0,0,0],atol=1e-14)
  np.testing.assert_allclose(nearest_patch_point(square,[2,0,0]),[1,0,0],atol=1e-14)
  np.testing.assert_allclose(nearest_patch_point(square[:2],[0,0,2]),[-1,0,0],atol=1e-14)
 def test_sampled_contact_route(self):
  graph=json.loads((OUT/'transition_graph.json').read_text());r=graph['complete_sampled_routes'][0]
  self.assertEqual(r['summary']['sample_count'],251);self.assertTrue(r['summary']['all_recorded_samples_valid'])
  self.assertEqual(r['contact_events'][0]['regime'],'BODY_AND_FOOT_SUPPORT');self.assertEqual(r['contact_events'][1]['regime'],'FOOT_SUPPORT')
  self.assertAlmostEqual(r['final_body'][2][3],.15);self.assertFalse(r['material_contact_locked_proved'])
  for e in graph['edges']:
   if e['summary'].get('all_recorded_samples_valid') and e['summary']['sample_count']>1:
    self.assertEqual(e['summary']['jacobian_orientation_sign_changes_per_leg'],[0]*4)
 def test_sources_and_saved_keyframes(self):
  table=json.loads((OUT/'xgo_table_semantics.json').read_text())
  for r in table['provenance']['manual_static_sources']:
   self.assertEqual(sha(OUT/'xgo_static_extracts'/(r['export_address']+'.txt')),r['sha256'])
  for t in table['tables']:
   self.assertEqual(len(t['controller_slots_u8']),12);self.assertEqual(len(t['logical_order']),12);self.assertIn('not URDF radians',t['scale'])
class ConnectivityTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.m=Model();load=lambda n:json.loads((OUT/(n+'.json')).read_text())
  cls.conn=load('connectivity');cls.rest=load('rest_connectivity');cls.lower=load('lower_envelope');cls.route=load('route_extension');cls.equiv=load('equivalent_stand');cls.lib=load('pose_library')
 def frames(self):
  return self.route['original_footprint_rises'][1]['frames']+self.equiv['routes'][1]['frames'][1:]
 def test_251_route_is_recomputed_not_copied(self):
  frames=self.frames();metrics=self.conn['route_251_sample']['metrics'];self.assertEqual(len(frames),251);self.assertTrue(all(f['valid'] for f in frames))
  q=np.array([f['q'] for f in frames]);margins=np.minimum(q-self.m.limits[:,0],self.m.limits[:,1]-q);self.assertAlmostEqual(margins.min(),metrics['min_joint_margin_rad'],12)
  self.assertEqual(int(np.argmin(margins.min(axis=1))),metrics['min_joint_margin_at']['frame']);self.assertAlmostEqual(np.max(abs(np.diff(q,axis=0))),metrics['max_abs_joint_delta_rad_per_sample'],12)
  self.assertAlmostEqual(min(f['support_margin_m'] for f in frames),metrics['min_static_support_margin_m'],12);self.assertAlmostEqual(min(f['min_separation']['distance_m'] for f in frames),metrics['min_nonadjacent_separation_m'],12)
  for k in (0,1,150,250):
   f=frames[k];v=self.m.evaluate(f['q'],np.array(f['body']),f['regime'],f['active_feet']);self.assertTrue(v['valid'],(k,v['errors']))
   self.assertAlmostEqual(v['min_separation']['distance_m'],f['min_separation']['distance_m'],9);self.assertAlmostEqual(v['support_margin_m'],f['support_margin_m'],9)
  self.assertEqual(frames[0]['regime'],'BODY_AND_FOOT_SUPPORT');self.assertEqual({f['regime'] for f in frames[1:]},{'FOOT_SUPPORT'})
  self.assertAlmostEqual(frames[250]['body'][2][3],.15,12);self.assertAlmostEqual(frames[250]['body'][0][3],.01,12);self.assertFalse(metrics['continuous_collision_free_proved']);self.assertFalse(metrics['material_contact_locked_proved'])
 def test_components_recomputed_from_validated_edges(self):
  parent={n['id']:n['id'] for n in self.conn['nodes']}
  def find(x):
   while parent[x]!=x:x=parent[x]
   return x
  for e in self.conn['edges']:
   if e['validated']:parent[find(e['from'])]=find(e['to'])
   else:self.assertNotEqual(e['status'],'VALIDATED_SAMPLED_ROUTE')
  groups={}
  for n in parent:groups.setdefault(find(n),set()).add(n)
  self.assertEqual(sorted(map(sorted,groups.values())),sorted(sorted(c['members']) for c in self.conn['components']));self.assertEqual(len(groups),self.conn['summary']['component_count'])
  stand=next(c for c in self.conn['components'] if c['contains_STAND']);self.assertNotIn('REST_GROUND',stand['members']);self.assertFalse(any(n.startswith('BODY_ONLY') for n in stand['members']))
  self.assertFalse(self.conn['summary']['REST_GROUND_in_stand_component']);self.assertIn('not a proof',self.conn['summary']['disconnection_meaning'])
 def test_body_only_connection_absence_is_bounded_not_proved(self):
  self.assertFalse(self.rest['global_absence_claim']);self.assertEqual(self.rest['starts_with_any_valid_connection'],[]);self.assertEqual(len(self.rest['starts']),6)
  self.assertTrue(any(s['is_REST_GROUND'] for s in self.rest['starts']));self.assertTrue(any(s['is_REST_GROUND_MAX_SEPARATION'] for s in self.rest['starts']))
  rest=next(p for p in self.lib['poses'] if p['name']=='REST_GROUND');self.assertEqual(next(s for s in self.rest['starts'] if s['is_REST_GROUND'])['q'],rest['q'])
  for s in self.rest['starts']:
   self.assertFalse(any(x['valid'] for x in s['direct_contact_additions']));self.assertFalse(any(x['all_samples_valid'] for x in s['body_supported_detours']))
   for d in s['direct_contact_additions']:self.assertLessEqual(d['frames_evaluated'],8);self.assertIsNotNone(d['first_failure'])
   for r in s['body_supported_detours']:self.assertEqual({a['seed'] for a in r['attempts']},{17,31,73})
 def test_lower_envelope_classification_is_recomputed(self):
  self.assertEqual(self.lower['limiting_constraint'],'BODY_GROUND_CONTACT');self.assertFalse(self.lower['physical_clearance_approval']);self.assertEqual(self.lower['geometric_infimum_height_above_body_ground_m'],0.0)
  for fam in self.lower['families']:
   by={s['height_above_body_ground_m']:s for s in fam['samples']}
   for h in (0.0,1e-6,1.001e-6):
    s=by[h];v=self.m.evaluate(s['q'],np.array(s['body']),s['regime'],s['active_feet']);self.assertEqual(v['valid'],s['valid'],(h,v['errors']))
   self.assertTrue(by[0.0]['valid'] and by[0.0]['regime']=='BODY_AND_FOOT_SUPPORT');self.assertIn('UNDECLARED_GROUND_CONTACT:base_link',by[1e-6]['errors']);self.assertTrue(by[1.001e-6]['valid'])
   for h,s in by.items():
    if 0<h<1.001e-6:self.assertFalse(s['valid'],h)
  self.assertEqual(self.lower['minimum_observed_pure_foot_height_above_body_ground_m'],1.001e-6);self.assertEqual(self.lower['numerical_undeclared_contact_boundary_m'],1e-6)
 def test_xgo_per_action_audit_fields(self):
  cat=json.loads((OUT/'xgo_catalogue.json').read_text());self.assertEqual(len(cat['actions']),30);self.assertEqual(cat['class_counts'],{'C':23,'D':1,'E':6})
  for a in cat['actions']:
   for key in ('action_id','aliases','canonical_name','documented_duration_s','handler_canonical_address','handler_export_address','handler_sha256','static_dynamic','evidence_class','recovered_controller_tables','contact_support_semantics','remaining_unknowns','retarget_eligibility','semantics'):self.assertIn(key,a,(a['action_id'],key))
   self.assertIn(a['evidence_class'],'ABCDE')
class CloseoutTests(unittest.TestCase):
 BOUNDARY='NO XGO PHYSICAL JOINT POSE WAS RECOVERED WITH SUFFICIENT SIGN/ZERO/SCALE BINDING TO BE COPIED DIRECTLY TO MATDOG.'
 POSES=('REST_GROUND','REST_GROUND_MAX_SEPARATION','LOW_CROUCH','LOW_C4','STAND','STRETCH')
 @classmethod
 def setUpClass(cls):
  load=lambda n:json.loads((OUT/(n+'.json')).read_text());cls.root=Path(__file__).resolve().parent.parents[2]
  cls.conn=load('connectivity');cls.close=load('closeout_classification');cls.lib=load('pose_library');cls.lower=load('lower_envelope');cls.m=Model();cls.text=(OUT/'REPORT.md').read_text()
 def components(self,classes):
  parent={n['id']:n['id'] for n in self.conn['nodes']}
  def find(x):
   while parent[x]!=x:x=parent[x]
   return x
  for e in self.conn['edges']:
   if e['edge_class'] in classes:parent[find(e['from'])]=find(e['to'])
  groups={}
  for n in parent:groups.setdefault(find(n),set()).add(n)
  return sorted(sorted(g) for g in groups.values())
 def test_edge_classes_are_explicit_and_consistent(self):
  edges=self.conn['edges'];nodes={n['id']:n for n in self.conn['nodes']};classes=[e['edge_class'] for e in edges]
  self.assertTrue(set(classes)<={'VALIDATED','CANDIDATE','FAILED','UNTESTED'});counts={k:classes.count(k) for k in set(classes)}
  self.assertEqual(counts,{'VALIDATED':4,'CANDIDATE':4,'FAILED':51,'UNTESTED':3});self.assertEqual(counts,self.conn['summary']['edge_class_counts'])
  for e in edges:
   self.assertFalse(e['authorizes_startup']);self.assertFalse(e['continuous_collision_free_proved'])
   if e['edge_class']=='VALIDATED':
    self.assertTrue(e['validated'] and e['all_recorded_samples_valid'] and nodes[e['from']]['promoted'] and nodes[e['to']]['promoted'],(e['from'],e['to']))
   else:self.assertFalse(e['validated'],(e['from'],e['to']))
   if e['edge_class']=='CANDIDATE':
    self.assertTrue(e['all_recorded_samples_valid'] and e['candidate_reasons'] and e['status']=='GEOMETRIC_PATH_CANDIDATE');self.assertIn('CONTACT_LOCK_NOT_PROVED',e['candidate_reasons'])
   if e['edge_class']=='UNTESTED':self.assertEqual(e['sample_count'],0)
  route=[e for e in edges if e['sample_count']==251];self.assertEqual(len(route),1);r=route[0]
  self.assertEqual((r['from'],r['to'],r['edge_class'],r['status'],r['validated']),('BODY_FOUR_FEET_RESEARCH_2','EQUIVALENT_STAND_150MM_X10MM','CANDIDATE','GEOMETRIC_PATH_CANDIDATE',False))
  for reason in ('ENDPOINT_IS_NOT_CANONICAL_STAND','NO_ACQUISITION_CONTRACT_FOR_BASE_PLUS_FOUR_FEET_START'):self.assertIn(reason,r['candidate_reasons'])
  eq=[e for e in edges if e['from']=='EQUIVALENT_STAND_150MM_X10MM' and e['to']=='STAND'];self.assertEqual([e['edge_class'] for e in eq],['UNTESTED'])
  self.assertFalse(any(e['edge_class']=='VALIDATED' and 'EQUIVALENT_STAND_150MM_X10MM' in (e['from'],e['to']) for e in edges))
 def test_components_by_edge_class_view(self):
  validated=self.components({'VALIDATED'});augmented=self.components({'VALIDATED','CANDIDATE'})
  self.assertEqual(validated,sorted(sorted(c['members']) for c in self.conn['components']));self.assertEqual(len(validated),self.conn['summary']['component_count'])
  self.assertEqual(len(augmented),self.conn['summary']['candidate_augmented_component_count']);self.assertGreater(len(validated),len(augmented))
  for view in (validated,augmented):
   stand=next(g for g in view if 'STAND' in g);self.assertEqual(stand,['CRAWL_READY','LOW_C4','LOW_CROUCH','STAND','STRETCH']);self.assertIn(['REST_GROUND'],view)
  self.assertFalse(self.conn['summary']['REST_GROUND_in_stand_component_including_candidate_edges'])
  self.assertIn('not the complete physical configuration space',self.close['transition_graph_semantics']['component_scope']);self.assertIn('not proof',self.close['transition_graph_semantics']['absence_of_validated_edge'])
 def test_four_foot_height_concepts_are_separate(self):
  h=self.close['four_foot_height_terminology'];a=h['A_BODY_AND_FOOT_SUPPORT_lower_geometric_boundary'];b=h['B_PURE_FOOT_SUPPORT_mathematical_search_boundary'];c=h['C_physically_robust_engineering_usable_pure_foot_stance']
  self.assertEqual((a['height_above_body_ground_m'],a['support_regime']),(0.0,'BODY_AND_FOOT_SUPPORT'));self.assertEqual(a['status'],'MODEL_GEOMETRIC_LIMIT_FOR_LEVEL_BODY')
  self.assertEqual((b['lowest_sampled_valid_height_m'],b['numerical_undeclared_contact_tolerance_m'],b['support_regime']),(1.001e-6,1e-6,'FOOT_SUPPORT'));self.assertFalse(b['physical_clearance_approval'])
  self.assertEqual(b['statement'],'THEORETICAL/SEARCH RESULT; NUMERICAL TOLERANCE DEPENDENCY; PHYSICAL ROBUSTNESS NOT ESTABLISHED');self.assertEqual(b['rejected_sampled_heights_m'],[1e-8,1e-7,5e-7,1e-6]);self.assertIn('physically meaningful clearance',b['must_not_be_described_as'])
  self.assertEqual((c['status'],c['height_m']),('NOT_ESTABLISHED',None));self.assertIn('no physical manufacturing tolerance is assumed',c['basis']);self.assertNotEqual(a['height_above_body_ground_m'],b['lowest_sampled_valid_height_m'])
  sens=h['clearance_sensitivity'];self.assertTrue(sens['no_acceptance_threshold_applied']);rows=sens['lower_envelope_samples'];self.assertEqual(len(rows),sum(len(f['samples']) for f in self.lower['families']))
  for row in rows:
   fam=next(f for f in self.lower['families'] if 'family %d'%f['family'] in row['source']);sample=next(x for x in fam['samples'] if x['height_above_body_ground_m']==row['height_above_body_ground_m'])
   self.assertEqual((row['valid'],row['regime']),(sample['valid'],sample['regime']))
   q=np.array(sample['q']);margin=float(np.minimum(q-self.m.limits[:,0],self.m.limits[:,1]-q).min());self.assertAlmostEqual(row['limiting_joint']['margin_rad'],margin,12)
   if sample['valid']:self.assertAlmostEqual(margin,sample['joint_margin_rad'],9)
  frames=[r['frame'] for r in sens['route_251_rise_frames']];self.assertEqual(frames,[0,1,2,3,5,10,20,50,100,150])
 def test_251_route_classification_preserves_metrics(self):
  r=self.close['route_251_sample'];self.assertEqual((r['classification'],r['edge_class'],r['sample_count']),('GEOMETRIC_PATH_CANDIDATE','CANDIDATE',251))
  for key in ('validated_transition_edge_to_STAND','authorized_startup_path','startup_authority','continuous_collision_free_proved','material_contact_locked_proved'):self.assertFalse(r[key],key)
  self.assertTrue(r['research_only'] and r['all_recorded_samples_valid']);metrics=self.conn['route_251_sample']['metrics']
  kept=r['metrics_preserved']
  for key in ('min_joint_margin_rad','min_nonadjacent_separation_m','min_static_support_margin_m','max_abs_mesh_foot_ground_residual_m','max_abs_joint_delta_rad_per_sample'):self.assertEqual(kept[key],metrics[key],key)
  self.assertEqual(kept['jacobian_global_min_abs_det'],metrics['jacobian_determinant']['global_min_abs']);self.assertEqual(kept['jacobian_sign_changes_per_leg'],[0,0,0,0]);self.assertEqual(kept['max_condition_number'],metrics['condition_number']['global_max'])
  self.assertAlmostEqual(kept['end_vs_canonical_STAND_max_joint_difference_rad'],0.3011,4);self.assertAlmostEqual(kept['end_vs_canonical_STAND_max_reference_contact_xy_difference_m']*1000,52.51,2)
  self.assertAlmostEqual(kept['min_joint_margin_rad'],0.003818655066258,12);self.assertAlmostEqual(kept['min_nonadjacent_separation_m']*1000,7.368754720,8)
 def test_g3_startup_and_motion_state_are_unchanged(self):
  g=self.close['g3_startup'];self.assertEqual(g['wording'],{'G3 architecture':'GENERALIZATION JUSTIFIED','G3 runtime startup gate':'STILL ACTIVE / NOT YET SUPERSEDED','BODY_ONLY REST_GROUND -> autonomous STAND':'NOT VALIDATED'})
  self.assertFalse(g['pose_library_startup_authority'] or g['route_251_authorized_as_startup_path']);header=(self.root/'05_Firmware/MATDOG_Controller/src/motion/MotionState.h').read_text()
  self.assertIn('enum class MotionState : uint8_t { OFF, IDLE, STAND_TRANSITION, STAND, STOPPING };',header)
  for rel,digest in g['runtime_gate_files_recorded'].items():
   self.assertEqual(sha(self.root/rel),digest,rel)
   try:baseline=subprocess.run(['git','show','%s:%s'%(g['g3_baseline_commit'],rel)],cwd=self.root,capture_output=True,timeout=60)
   except (OSError,subprocess.SubprocessError):continue
   if baseline.returncode==0 and 'Startup' in rel:import hashlib;self.assertEqual(hashlib.sha256(baseline.stdout).hexdigest(),digest,rel)
 def test_pose_roles_separate_targets_from_motion_states(self):
  roles={p['pose']:p for p in self.close['pose_library_roles']['promoted_static_poses']};self.assertEqual(sorted(roles),sorted(self.POSES));vocab=set(self.close['pose_library_roles']['vocabulary'])
  self.assertEqual(vocab,{'SEMANTIC_POSE_TARGET','MOTION_STATE_CANDIDATE','RESEARCH_VARIANT'})
  for name,p in roles.items():self.assertTrue(p['roles'] and set(p['roles'])<=vocab,name);self.assertTrue(p['embedded_static_reference_target'],name)
  self.assertEqual(roles['REST_GROUND_MAX_SEPARATION']['roles'],['RESEARCH_VARIANT']);self.assertEqual(roles['STAND']['motion_state_relation'],'EXISTING_STATE_STAND')
  self.assertEqual(roles['REST_GROUND']['motion_state_relation'],'FUTURE_CANDIDATE_BLOCKED');self.assertTrue(roles['REST_GROUND']['blockers']);self.assertFalse(roles['REST_GROUND']['in_stand_validated_component'])
  self.assertEqual([a['alias_of'] for a in self.close['pose_library_roles']['aliases']],['LOW_CROUCH','LOW_CROUCH'])
 def test_xgo_and_base_link_wording(self):
  x=self.close['xgo_retargeting_boundary'];self.assertEqual(x['boundary_statement'],self.BOUNDARY);self.assertTrue(x['no_new_xgo_research']);self.assertIn('not a numeric conversion rule',x['scaling_rule']);self.assertIn('reference evidence',x['use_of_xgo_evidence'])
  z=self.close['base_link_z'];rest=next(p for p in self.lib['poses'] if p['name']=='REST_GROUND');self.assertEqual(z['REST_GROUND_base_link_world_z_m'],rest['body'][2][3])
  self.assertIn('effectively zero within model/mesh numerical precision',z['nominal_cad_urdf_geometry']);self.assertIn('NOT ESTABLISHED',z['real_hardware_contact_height']);self.assertIn('cannot be positioned with 1e-16 m accuracy',z['not_a_positioning_accuracy'])
 def test_report_carries_the_closeout_wording(self):
  text=self.text;low=text.lower()
  for phrase in (self.BOUNDARY,'GEOMETRIC_PATH_CANDIDATE','GENERALIZATION JUSTIFIED','STILL ACTIVE / NOT YET SUPERSEDED','NOT VALIDATED','SEMANTIC_POSE_TARGET','MOTION_STATE_CANDIDATE','RESEARCH_VARIANT','NOT ESTABLISHED','VALIDATED','CANDIDATE','FAILED','UNTESTED'):self.assertIn(phrase,text,phrase)
  for phrase in ('body_and_foot_support lower geometric boundary','pure_foot_support mathematical/search boundary','not a physically meaningful clearance','numerical tolerance dependency','physical robustness not established','components of the validated graph','not a validated transition edge to stand','not an authorized or validated startup path','not a numeric conversion rule','effectively zero within model/mesh numerical precision','real hardware contact height'):self.assertIn(phrase,low,phrase)
  for stale in ('G3 startup is partially superseded','Partially superseded; the implemented gate is unchanged','-->|base lift-off, 251 samples|','lowest validated four-foot configuration sits at body height','no route joins them'):self.assertNotIn(stale,text,stale)
class ReportTests(unittest.TestCase):
 def test_report_answers_every_required_question_and_states_safety(self):
  text=(OUT/'REPORT.md').read_text()
  for n in range(1,23):self.assertRegex(text,r'(?m)^### Q%d\. '%n,'question %d'%n)
  for phrase in ('PROVEN LIMIT','LOWEST FOUND SO FAR','EXACT_LITE','CORROBORATED_XGO_FAMILY','GENERIC_TEMPLATE','UNKNOWN','No physical hardware was accessed','No push','No merge'):self.assertIn(phrase,text,phrase)
 def test_recorded_provenance_hashes_match_files(self):
  import artifact_manifest
  summary=artifact_manifest.build()['summary'];self.assertEqual(summary['stale_inputs'],[]);self.assertEqual(summary['changed_canonical_sources'],[])
if __name__=='__main__':unittest.main(verbosity=2)
