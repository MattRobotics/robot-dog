"""G3.5.1 closeout classification, derived from saved G3.5 artifacts only (standard library, deterministic).

No search, mesh model, optimizer or hardware access: this reads the committed JSON artifacts and the canonical
URDF, restates their status vocabulary and writes closeout_classification.json. Run after connectivity.py.
"""
import hashlib,json,math
import xml.etree.ElementTree as ET
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OUT=ROOT/'09_Logs/Validation_Reports/G35_Pose_Audit'
URDF='03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
MOTION='05_Firmware/MATDOG_Controller/src/motion/'
GATE_FILES=(MOTION+'MotionState.h',MOTION+'MotionState.cpp',MOTION+'StartupAcquisition.h',MOTION+'StartupAcquisition.cpp')
JSON_INPUTS=('pose_library.json','lower_envelope.json','connectivity.json','route_extension.json','equivalent_stand.json','rest_search.json','xgo_table_semantics.json','retarget_matrix.json')
ROUTE_FRAMES=(0,1,2,3,5,10,20,50,100,150)
G3_BASELINE='87c3e914e068260170e7c19c5d664f727f5b03ce'
XGO_BOUNDARY='NO XGO PHYSICAL JOINT POSE WAS RECOVERED WITH SUFFICIENT SIGN/ZERO/SCALE BINDING TO BE COPIED DIRECTLY TO MATDOG.'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(name):return json.loads((OUT/name).read_text())

def limits():
 root=ET.parse(ROOT/URDF).getroot()
 return {j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in root.findall('joint') if j.find('limit') is not None}

def limiting_joint(q,order,lim):
 rows=[(min(v-lim[n][0],lim[n][1]-v),n,'lower' if v-lim[n][0]<lim[n][1]-v else 'upper') for n,v in zip(order,q)]
 margin,name,side=min(rows);tied=sorted(n for m,n,_ in rows if m-margin<=1e-9)
 return {'joint':name,'limit_side':side,'margin_rad':margin,'margin_deg':math.degrees(margin),'joints_tied_within_1e-9_rad':tied}

def sample_row(source,sample,height,order,lim):
 lj=limiting_joint(sample['q'],order,lim);sep=sample['min_separation']['distance_m'] if sample.get('min_separation') else None
 return {'source':source,'height_above_body_ground_m':height,'regime':sample['regime'],'valid':sample['valid'],'errors':sample['errors'],'limiting_joint':lj,'stored_joint_margin_rad':sample['joint_margin_rad'],'min_nonadjacent_separation_m':sep,'support_margin_m':sample['support_margin_m']}

def main():
 lib=read('pose_library.json');lower=read('lower_envelope.json');conn=read('connectivity.json');ext=read('route_extension.json');rest_search=read('rest_search.json');tables=read('xgo_table_semantics.json');matrix=read('retarget_matrix.json')
 poses={p['name']:p for p in lib['poses']};lim=limits();route=conn['route_251_sample'];rm=route['metrics'];order=rm['start']['q_order'];summary=conn['summary']
 tol=lower['numerical_undeclared_contact_boundary_m'];pure=lower['minimum_observed_pure_foot_height_above_body_ground_m'];infimum=lower['geometric_infimum_height_above_body_ground_m']

 envelope_rows=[]
 for fam in lower['families']:
  for s in fam['samples']:
   row=sample_row('lower_envelope.json:family %d'%fam['family'],s,s['height_above_body_ground_m'],order,lim);row['family']=fam['family'];envelope_rows.append(row)
 rise=ext['original_footprint_rises'][route['family']]['frames'];route_rows=[]
 for k in ROUTE_FRAMES:
  f=rise[k];row=sample_row('route_extension.json:/original_footprint_rises/%d/frames/%d'%(route['family'],k),f,f['body'][2][3],order,lim);row['frame']=k;route_rows.append(row)
 by_family={fam['family']:{s['height_above_body_ground_m']:s for s in fam['samples']} for fam in lower['families']}
 zero=by_family[0][0.0];first_pure=by_family[0][pure];ten=by_family[0][1e-5]
 assert zero['valid'] and zero['regime']=='BODY_AND_FOOT_SUPPORT' and first_pure['valid'] and first_pure['regime']=='FOOT_SUPPORT'
 assert all(s['errors']==['UNDECLARED_GROUND_CONTACT:base_link'] and 0<h<=tol for h,s in by_family[0].items() if not s['valid'] and h>0)
 margin_span=abs(first_pure['joint_margin_rad']-zero['joint_margin_rad'])

 heights={
  'A_BODY_AND_FOOT_SUPPORT_lower_geometric_boundary':{'height_above_body_ground_m':infimum,'support_regime':'BODY_AND_FOOT_SUPPORT','contact_set':'base_link plus four foot links on the ground','status':'MODEL_GEOMETRIC_LIMIT_FOR_LEVEL_BODY','valid_pose_at_this_height':True,'basis':'below this height canonical base-mesh vertices are below the ground for a level body; the pose itself passes the full offline policy at exactly this height in both foot families','not_a_claim':'not a foot-supported stance: the base rests on the ground; it is a body-and-feet ground rest','limiting_constraint':lower['limiting_constraint']},
  'B_PURE_FOOT_SUPPORT_mathematical_search_boundary':{'lowest_sampled_valid_height_m':pure,'support_regime':'FOOT_SUPPORT','status':'LOWEST_FOUND_SO_FAR_THEORETICAL_SEARCH_RESULT','numerical_undeclared_contact_tolerance_m':tol,'rejected_sampled_heights_m':sorted(h for h,s in by_family[0].items() if not s['valid']),'rejection_code':'UNDECLARED_GROUND_CONTACT:base_link','tolerance_dependency':'the boundary is set by the configured 1e-6 m undeclared-contact tolerance: every sampled height in (0, 1e-6] m is rejected only as undeclared base contact, and the recorded joint margin changes by %.3e rad between height 0 and %.3e m, so no geometric constraint activates at the boundary'%(margin_span,pure),'physical_clearance_approval':lower['physical_clearance_approval'],'must_not_be_described_as':'a physically meaningful clearance, a hardware-achievable base-to-ground gap or a safe height','statement':'THEORETICAL/SEARCH RESULT; NUMERICAL TOLERANCE DEPENDENCY; PHYSICAL ROBUSTNESS NOT ESTABLISHED'},
  'C_physically_robust_engineering_usable_pure_foot_stance':{'status':'NOT_ESTABLISHED','height_m':None,'basis':'no hardware measurement, construction tolerance, servo/mechanical compliance or terrain model exists in this offline audit; no physical manufacturing tolerance is assumed and no acceptance threshold is introduced','what_is_available':'clearance_sensitivity below: saved sampled heights with the joint-limit, separation and support metrics at each','how_to_close':'requires reviewed hardware data outside this software-only audit'},
  'clearance_sensitivity':{'scope':'saved samples only; no search or optimization was rerun; joint margin is measured to the URDF limit of the limiting joint','no_acceptance_threshold_applied':True,'lower_envelope_samples':envelope_rows,'route_251_rise_frames':route_rows,
   'observations':{'joint_margin_rad_at_0_m':zero['joint_margin_rad'],'joint_margin_rad_at_first_pure_foot_height':first_pure['joint_margin_rad'],'joint_margin_rad_at_10_um':ten['joint_margin_rad'],'limiting_joint_at_0_m':limiting_joint(zero['q'],order,lim),'route_joint_margin_rad_at_frame_1':route_rows[1]['stored_joint_margin_rad'],'route_frame_1_height_m':route_rows[1]['height_above_body_ground_m'],'route_joint_margin_rad_at_frame_10':route_rows[5]['stored_joint_margin_rad'],'route_frame_10_height_m':route_rows[5]['height_above_body_ground_m'],'route_joint_margin_rad_at_frame_150':route_rows[9]['stored_joint_margin_rad'],'route_frame_150_height_m':route_rows[9]['height_above_body_ground_m']},
   'reading':'the pure-foot stance at ~1 um has essentially the joint-limit margin of the 0 m ground rest (a few micro-radians apart); along the saved route the minimum margin grows with body height and then falls again near 149 mm where a different joint limit (an upper limit) is the limiting one, so body height above ground alone is not a robustness measure'}}

 rest=poses['REST_GROUND'];body_z=rest['body'][2][3]
 stand_members=summary['stand_component_members']
 c=route['classification']
 route_block={'classification':c['status'],'edge_class':c['edge_class'],'research_only':c['research_only'],'validated_transition_edge_to_STAND':c['validated_transition_edge_to_STAND'],'authorized_startup_path':c['authorized_startup_path'],'startup_authority':c['startup_authority'],
  'meaning':'GEOMETRIC_PATH_CANDIDATE: a sampled geometric path with useful metrics (BODY_FOUR_FEET_RESEARCH_2 to an equivalent 150 mm stand at body X +10 mm); the endpoint is not canonical STAND, the path is not proven fully collision-free and is not proven contact-locked',
  'reasons':c['reasons'],'sample_count':rm['sample_count'],'all_recorded_samples_valid':rm['all_samples_valid'],'continuous_collision_free_proved':rm['continuous_collision_free_proved'],'material_contact_locked_proved':rm['material_contact_locked_proved'],
  'metrics_preserved':{'source':'connectivity.json:/route_251_sample/metrics','min_joint_margin_rad':rm['min_joint_margin_rad'],'min_joint_margin_at':rm['min_joint_margin_at'],'min_nonadjacent_separation_m':rm['min_nonadjacent_separation_m'],'min_separation_at':rm['min_separation_at'],'min_static_support_margin_m':rm['min_static_support_margin_m'],'max_abs_mesh_foot_ground_residual_m':rm['max_abs_mesh_foot_ground_residual_m'],'max_abs_joint_delta_rad_per_sample':rm['max_abs_joint_delta_rad_per_sample'],'jacobian_global_min_abs_det':rm['jacobian_determinant']['global_min_abs'],'jacobian_sign_changes_per_leg':rm['jacobian_determinant']['sign_changes_per_leg'],'max_condition_number':rm['condition_number']['global_max'],'end_vs_canonical_STAND_max_joint_difference_rad':conn['equivalent_stand_vs_canonical_STAND']['max_abs_joint_difference_rad'],'end_vs_canonical_STAND_max_reference_contact_xy_difference_m':conn['equivalent_stand_vs_canonical_STAND']['max_reference_contact_xy_difference_m']},
  'other_candidate_edges':[{'from':e['from'],'to':e['to'],'status':e['status'],'sample_count':e['sample_count']} for e in conn['edges'] if e['edge_class']=='CANDIDATE'],
  'transition_graph_json_note':'transition_graph.json keeps the search-level status VALID_SEQUENCE_CANDIDATE (every recorded sample valid) and startup_reassessment STILL_REQUIRED; it never marks the route validated or authorized. connectivity.json applies the edge classes.'}

 startup=dict(conn['startup_reassessment']);startup['g3_baseline_commit']=G3_BASELINE
 startup['runtime_gate_files_recorded']={f:sha(ROOT/f) for f in GATE_FILES}
 startup['pose_library_startup_authority']=lib['startup_authority']
 startup['wording']={'G3 architecture':'GENERALIZATION JUSTIFIED','G3 runtime startup gate':'STILL ACTIVE / NOT YET SUPERSEDED','BODY_ONLY REST_GROUND -> autonomous STAND':'NOT VALIDATED'}

 roles={
  'REST_GROUND':{'roles':['SEMANTIC_POSE_TARGET','MOTION_STATE_CANDIDATE'],'motion_state_relation':'FUTURE_CANDIDATE_BLOCKED','blockers':['singleton in the validated graph: no validated route to or from any foot-supported pose','no acquisition or contact contract for a body-supported start','the G3 runtime gate does not admit a body-supported start']},
  'REST_GROUND_MAX_SEPARATION':{'roles':['RESEARCH_VARIANT'],'motion_state_relation':'NOT_A_STATE','blockers':['alternative to REST_GROUND that sits exactly at a joint limit (joint margin %.1f rad); it belongs in the pose library as a reference variant and does not deserve its own state'%poses['REST_GROUND_MAX_SEPARATION']['joint_margin_rad']]},
  'LOW_CROUCH':{'roles':['SEMANTIC_POSE_TARGET','MOTION_STATE_CANDIDATE'],'motion_state_relation':'FUTURE_CANDIDATE_CONDITIONAL','aliases':['CRAWL_READY','SQUAT'],'blockers':['no production generator or acquisition contract reaches it yet; a state is justified only if a behaviour requires a stable low stance']},
  'LOW_C4':{'roles':['SEMANTIC_POSE_TARGET'],'motion_state_relation':'WAYPOINT_OF_EXISTING_STAND_TRANSITION','blockers':['G3 canonical startup entry pose; the existing STAND_TRANSITION state already covers the LOW_C4 to STAND rise, so no separate state is needed']},
  'STAND':{'roles':['SEMANTIC_POSE_TARGET'],'motion_state_relation':'EXISTING_STATE_STAND','blockers':[]},
  'STRETCH':{'roles':['SEMANTIC_POSE_TARGET'],'motion_state_relation':'NOT_A_STATE','blockers':['a posture gesture reached from STAND (validated sampled route STAND to STRETCH); a behaviour sequence may reference it, a state is not needed']}}
 pose_roles=[]
 for name,r in roles.items():
  p=poses[name];member=[x for x in conn['components'] if name in x['members']][0]
  pose_roles.append({'pose':name,**r,'regime':p['regime'],'embedded_static_reference_target':p['embedded_target'],'in_stand_validated_component':name in stand_members,'validated_component_size':member['size']})
 research=[{'pose':n,'roles':['RESEARCH_VARIANT'],'motion_state_relation':'NOT_A_STATE','embedded_static_reference_target':poses[n]['embedded_target'],'evidence':poses[n]['evidence']} for n in ('BODY_FOUR_FEET_RESEARCH_1','BODY_FOUR_FEET_RESEARCH_2','PITCHED_CROUCH_RESEARCH','ROLL_PREP_RESEARCH','XGO_HEIGHT_RATIO_RESEARCH')]
 pose_block={'basis':'DESIGN_CLASSIFICATION separating pose-library entries from motion states; not derived from measurements. The blockers are the only evidence-derived parts.','vocabulary':{'SEMANTIC_POSE_TARGET':'a named static geometric reference the pose library may serve to a planner','MOTION_STATE_CANDIDATE':'could justify a future MotionState if a behaviour requires it; none is created or authorized here','RESEARCH_VARIANT':'alternative or exploratory record kept for evidence; never a state'},'motion_state_enum':'OFF, IDLE, STAND_TRANSITION, STAND, STOPPING (unchanged)','motion_state_enum_file_sha256':sha(ROOT/MOTION/'MotionState.h'),'promoted_static_poses':pose_roles,'aliases':[{'alias':a,'alias_of':t,'role':'SEMANTIC_ALIAS'} for a,t in sorted(lib['aliases'].items())],'research_records':research}

 unique_tables=len({tuple(t['controller_slots_u8']) for t in tables['tables']})
 xgo={'boundary_statement':XGO_BOUNDARY,'recovered':'XGO action internals and normalized post-IK controller-domain tables (%d table references, %d unique 12-byte uint8 tables) were recovered from the firmware and static analysis'%(len(tables['tables']),unique_tables),'not_recovered':tables['physical_unknowns'],'use_of_xgo_evidence':'XGO behavior, topology and sequence semantics are reference evidence only; MATDOG joint geometry continues to come from the MATDOG URDF and the MATDOG IK','scaling_rule':'the ~1.5x similarity of two generic-template dimensions is not a numeric conversion rule and must not be used to convert any XGO value to MATDOG','physical_retarget_results':sorted({r['physical_retarget_result'] for r in matrix['preset_actions']+matrix['host_apis']}),'no_new_xgo_research':True}
 base_link={'REST_GROUND_base_link_world_z_m':body_z,'rest_search_body_patch_height_m':rest_search['body_patch']['height_m'],'nominal_cad_urdf_geometry':'effectively zero within model/mesh numerical precision (double-precision arithmetic on the canonical mesh); derived from the mesh minimum local Z, not assigned','real_hardware_contact_height':'NOT ESTABLISHED: subject to construction and compliance; no hardware measurement exists','not_a_positioning_accuracy':'a physical robot cannot be positioned with 1e-16 m accuracy; the value is a numerical artifact of the nominal model, not a positioning tolerance'}
 semantics={'edge_classes':conn['semantics']['edge_classes'],'component_scope':conn['semantics']['components'],'absence_of_validated_edge':conn['semantics']['absence_of_validated_edge'],'edge_class_counts':summary['edge_class_counts'],'node_pair_best_edge_class_counts':summary['node_pair_best_edge_class_counts'],'validated_component_count':summary['component_count'],'candidate_augmented_component_count':summary['candidate_augmented_component_count'],'previous_component_count_before_edge_classes':12,'previous_count_explanation':'the G3.5 count of 12 also joined four CANDIDATE research-route edges (two base lift-offs, the original-footprint rise and the 251-sample route); the STAND component and the REST_GROUND singleton are unchanged'}

 result={'provenance':{'inputs':{n:sha(OUT/n) for n in JSON_INPUTS},'canonical_sources':{**{f:sha(ROOT/f) for f in GATE_FILES},URDF:sha(ROOT/URDF)},'generator_sha256':sha(__file__)},
  'scope':'G3.5.1 classification and contract consistency; software-only, saved artifacts only, no search rerun, no hardware, serial, servo, firmware, q0, calibration or actuator change',
  'four_foot_height_terminology':heights,'route_251_sample':route_block,'g3_startup':startup,'pose_library_roles':pose_block,'xgo_retargeting_boundary':xgo,'base_link_z':base_link,'transition_graph_semantics':semantics}
 (OUT/'closeout_classification.json').write_text(json.dumps(result,sort_keys=True,indent=2,allow_nan=False)+'\n')
 print('CLOSEOUT written; validated components',summary['component_count'],'route class',c['status'])

if __name__=='__main__':main()
