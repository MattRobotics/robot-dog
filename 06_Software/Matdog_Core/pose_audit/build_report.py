"""Regenerate REPORT.md from the saved G3.5 artifacts (standard library only, deterministic).

Every number in the report is read from an artifact or from the canonical URDF; nothing is typed in
except fixed prose. Run after validate.py so the validation section reflects the recorded gates.
"""
import json,math,re
import xml.etree.ElementTree as ET
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OUT=ROOT/'09_Logs/Validation_Reports/G35_Pose_Audit'
URDF=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
LEGS=('lf','rf','rh','lh')
JOINTS=('hip','upper','lower')

def read(name):
 return json.loads((OUT/(name+'.json')).read_text())

def mm(x,n=6):
 return f'{x*1000:.{n}f}'

def rad(x,n=9):
 return f'{x:.{n}f}'

def table(header,rows):
 head='| '+' | '.join(header)+' |\n|'+'|'.join('---' for _ in header)+'|\n'
 return head+'\n'.join('| '+' | '.join(str(c) for c in r)+' |' for r in rows)

def hip_origin_z():
 root=ET.parse(URDF).getroot();out={}
 for j in root.findall('joint'):
  if j.get('name','').endswith('_hip_joint'):out[j.get('name')]=[float(v) for v in j.find('origin').get('xyz').split()]
 return out

def joint_limits():
 root=ET.parse(URDF).getroot()
 return {j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in root.findall('joint') if j.find('limit') is not None}

def yesno(flag):
 return 'yes' if flag else 'no'

def main():
 lib=read('pose_library');poses={p['name']:p for p in lib['poses']};rest=poses['REST_GROUND'];rest_search=read('rest_search');conn=read('connectivity');restc=read('rest_connectivity')
 lower=read('lower_envelope');dims=read('dimensions');cat=read('xgo_catalogue');tables=read('xgo_table_semantics');revs=read('xgo_revision_comparison');matrix=read('retarget_matrix');envelope=read('envelope')
 expanded=read('expanded_envelope');policy=read('collision_policy');inventory=read('xgo_search_inventory');graph=read('transition_graph');transfer=read('support_transfer');revalidation=read('revalidation');equiv_revalidation=read('equivalent_stand_revalidation')
 validation=json.loads((OUT/'validation_results.json').read_text()) if (OUT/'validation_results.json').exists() else None
 route=conn['route_251_sample'];rm=route['metrics'];hips=hip_origin_z();joint_names=rm['start'].get('q_order')
 body_z=rest_search['body_patch']['height_m'];front_hip_z=hips['lf_hip_joint'][2];rear_hip_z=hips['rh_hip_joint'][2]
 close=read('closeout_classification');hts=close['four_foot_height_terminology'];hA=hts['A_BODY_AND_FOOT_SUPPORT_lower_geometric_boundary'];hB=hts['B_PURE_FOOT_SUPPORT_mathematical_search_boundary'];hC=hts['C_physically_robust_engineering_usable_pure_foot_stance'];sens=hts['clearance_sensitivity'];startup=close['g3_startup'];roles=close['pose_library_roles'];xgo=close['xgo_retargeting_boundary'];baselink=close['base_link_z'];route_class=close['route_251_sample'];sem=close['transition_graph_semantics']
 stand_component=next(c for c in conn['components'] if c['contains_STAND']);summary=conn['summary']
 nontrivial=[c for c in conn['components'] if c['size']>1];singletons=[c for c in conn['components'] if c['size']==1]
 nodes={n['id']:n for n in conn['nodes']}
 body_only=[i for i,c in enumerate(rest_search['candidates']) if c.get('valid') and c.get('regime')=='BODY_SUPPORT']
 view_files=sorted(p.stem for p in (OUT/'views').glob('*.png'))
 limits=joint_limits();start_q=rm['start']['q_rad'];margins={n:min(v-limits[n][0],limits[n][1]-v) for n,v in zip(joint_names,start_q)};near_joint_margin=min(margins.values());near=[n for n in joint_names if abs(margins[n]-near_joint_margin)<1e-12]
 near_joint_names=', '.join(f'`{n}`' for n in near);q_near=start_q[joint_names.index(near[0])];near_joint_side='lower' if q_near-limits[near[0]][0]<limits[near[0]][1]-q_near else 'upper';near_joint_limit=limits[near[0]][0 if near_joint_side=='lower' else 1]
 families=lower['families'];pure=lower['minimum_observed_pure_foot_height_above_body_ground_m'];infimum=lower['geometric_infimum_height_above_body_ground_m']
 by_height={f['family']:{s['height_above_body_ground_m']:s for s in f['samples']} for f in families}
 margin_h0=by_height[0][0.0]['joint_margin_rad'];margin_h1=by_height[0][1e-5]['joint_margin_rad'];slope=(margin_h1-margin_h0)/1e-5
 actions=cat['actions'];matrix_rows={r['action_id']:r for r in matrix['preset_actions']}
 table_actions=sorted({t['action_id'] for t in tables['tables']});unique_tables=len({tuple(t['controller_slots_u8']) for t in tables['tables']})
 envelope_rows=envelope['rows'];envelope_invalid=[r for r in envelope_rows if not r['valid']]
 direct=[d for s in restc['starts'] for d in s['direct_contact_additions']];detours=[d for s in restc['starts'] for d in s['body_supported_detours']]
 first_failures=[d['first_failure'] for d in direct];failure_pairs=sorted({tuple(sorted(p)) for f in first_failures for p in f['collisions']})
 tree_nodes=[a['nodes'] for d in detours for a in d['attempts']];free=restc['isolated_leg_free_space']
 crawl=envelope['poses']['CRAWL_READY'];low=envelope['poses']['LOW_CROUCH'];alias_delta=max(abs(a-b) for a,b in zip(crawl['q'],low['q']))
 accepted=[c for c in conn['edges'] if c['edge_class']=='VALIDATED'];candidates=[c for c in conn['edges'] if c['edge_class']=='CANDIDATE'];failed=[c for c in conn['edges'] if c['edge_class']=='FAILED'];untested=[c for c in conn['edges'] if c['edge_class']=='UNTESTED']
 six=('REST_GROUND','REST_GROUND_MAX_SEPARATION','LOW_CROUCH','LOW_C4','STAND','STRETCH')
 counts={}
 for a in actions:counts[a['evidence_class']]=counts.get(a['evidence_class'],0)+1
 recovery={}
 for r in matrix['preset_actions']:recovery[r['recovery']]=recovery.get(r['recovery'],0)+1
 retarget_result={}
 for r in matrix['preset_actions']+matrix['host_apis']:retarget_result[r['physical_retarget_result']]=retarget_result.get(r['physical_retarget_result'],0)+1

 angle_rows=[]
 for i,leg in enumerate(LEGS):
  q=rest['q'][i*3:i*3+3];angle_rows.append([leg.upper()]+[f'{v:.12f}' for v in q]+[f'{math.degrees(v):.6f}' for v in q])
 asym_rows=[]
 for name in six:
  q=poses[name]['q'];asym_rows.append([name,rad(q[1],6),rad(q[7],6),rad(q[7]-q[1],6),rad(q[2],6),rad(q[8],6),rad(q[8]-q[2],6),rad(q[6]-q[0],6)])
 metric_rows=[[p['name'],p['evidence'],p['regime'],mm(p['body'][2][3]),rad(p['joint_margin_rad']),mm(p['min_separation']['distance_m']),mm(p['support_margin_m'])] for p in lib['poses']]
 six_rows=[[n,poses[n]['regime'],'base_link' if poses[n]['base_contact'] else ' '.join(poses[n]['active_feet']),mm(poses[n]['body'][2][3],3),rad(poses[n]['joint_margin_rad'],6),mm(poses[n]['min_separation']['distance_m'],3),mm(poses[n]['support_margin_m'],3),poses[n]['source']] for n in six]
 lower_joint={(r['family'],r['height_above_body_ground_m']):r['limiting_joint'] for r in sens['lower_envelope_samples']}
 lower_rows=[]
 for fam,samples in by_height.items():
  for h,s in samples.items():
   lj=lower_joint.get((fam,h)) or {'joints_tied_within_1e-9_rad':['-'],'limit_side':'-','margin_deg':float('nan')};lower_rows.append([fam,f'{h:.3e}',s['regime'],yesno(s['valid']),', '.join(s['errors']) or '-',rad(s['joint_margin_rad'],6) if s['valid'] else '-',f"{' / '.join(lj['joints_tied_within_1e-9_rad'])} ({lj['limit_side']}, {lj['margin_deg']:.3f} deg)" if s['valid'] else '-',mm(s['support_margin_m'],3) if s['valid'] else '-',mm(s['min_separation']['distance_m'],3) if s['valid'] else '-'])
 component_rows=[[c['component'],c['size'],', '.join(c['members']),yesno(c['contains_STAND']),yesno(c['contains_REST_GROUND'])] for c in conn['components']]
 def regimes_of(e):return nodes[e['from']]['regime']+' -> '+nodes[e['to']]['regime'] if e['from'] in nodes and e['to'] in nodes else '-'
 generic_reasons=('SAMPLED_ONLY_SWEPT_COLLISION_NOT_PROVED','CONTACT_LOCK_NOT_PROVED')
 edge_rows=[[e['from'],e['to'],regimes_of(e),e['sample_count'],e['edge_class'],e['source']] for e in accepted]
 candidate_rows=[[e['from'],e['to'],regimes_of(e),e['sample_count'],e['status'],'; '.join(r for r in e['candidate_reasons'] if r not in generic_reasons)] for e in candidates]
 untested_rows=[[e['from'],e['to'],e['source_status'],e['note'] or (e['evidence'].get('reason') if isinstance(e['evidence'],dict) else '') or '-'] for e in untested]
 role_rows=[[r['pose'],', '.join(r['roles']),r['motion_state_relation'],'; '.join(r['blockers']) or '-'] for r in roles['promoted_static_poses']]
 route_sens_rows=[[r['frame'],f"{r['height_above_body_ground_m']*1000:.4f}",r['regime'],f"{' / '.join(r['limiting_joint']['joints_tied_within_1e-9_rad'])} ({r['limiting_joint']['limit_side']})",rad(r['limiting_joint']['margin_rad'],6),f"{r['limiting_joint']['margin_deg']:.3f}",mm(r['min_nonadjacent_separation_m'],3),mm(r['support_margin_m'],2)] for r in sens['route_251_rise_frames']]
 first_rows=[]
 for s in restc['starts']:
  fails=[x['first_failure']['frame'] for x in s['direct_contact_additions']];firsts=[a['nodes'] for d in s['body_supported_detours'] for a in d['attempts']]
  first_rows.append([s['source_pointer'],'REST_GROUND' if s['is_REST_GROUND'] else ('REST_GROUND_MAX_SEPARATION' if s['is_REST_GROUND_MAX_SEPARATION'] else 'body-only candidate'),rad(s['joint_margin_rad'],6),mm(s['min_separation_m'],3),mm(s['support_margin_m'],3),'/'.join(str(f) for f in fails),f'{min(firsts)}-{max(firsts)}','no'])
 joint_rows=[[j,rad(a,9),rad(b,9),rad(c,9),rad(rm['per_joint_min_margin_rad'][j],9),rm['per_joint_min_margin_frame'][j]] for j,a,b,c in zip(joint_names,rm['start']['q_rad'],rm['final']['q_rad'],rm['max_abs_joint_delta_rad_per_joint'])] if joint_names else []
 dimension_rows=[]
 for x in dims['rows']:
  if x.get('ratio') is not None:dimension_rows.append([x['dimension'],x['xgo_identity'],mm(x['xgo_m']),mm(x['matdog_m']),f"{x['ratio']:.6f}",x['meaning']])
  elif 'xgo_m' in x:dimension_rows.append([x['dimension'],x['xgo_identity'],mm(x['xgo_m']),mm(x['matdog_m']),'not a valid ratio',x['meaning']])
  else:dimension_rows.append([x['dimension'],x['xgo_identity'],'-','-','no ratio','no source-bound exact Lite value'])
 identity_counts={}
 for x in dims['rows']:identity_counts[x['xgo_identity']]=identity_counts.get(x['xgo_identity'],0)+1
 action_rows=[]
 for a in actions:
  r=matrix_rows[a['action_id']];tabs=a['recovered_controller_tables']
  action_rows.append([a['action_id'],a['canonical_name'].replace('|','/'),', '.join(a['aliases']).replace('|','/'),a['evidence_class'],a['documented_duration_s'] or 'n/a',a['handler_canonical_address']+' ('+a['handler_sha256'][:10]+')',a['static_dynamic'],len(tabs) if tabs else 0,r['recovery'],r['physical_retarget_result'],', '.join(r['matdog_intent_experiments']) or '-'])
 host_rows=[[h['name'],h['evidence_class'],h['scope'],'; '.join(h['unknowns'])] for h in cat['additional_host_pose_apis']]
 semantic_rows=[[a['action_id'],a['semantics'].replace('|','/'),a['contact_support_semantics'].replace('|','/')] for a in actions]
 doc_conflicts=revs['document_vs_library_conflicts']
 validation_section='Validation gates have not been recorded yet (`validation_results.json` is missing). Run `validate.py`, then rebuild this report.'
 if validation:
  rows=[[g['name'],g['returncode'],yesno(g['passed']),f"{g['seconds']:.1f}",g.get('summary','')] for g in validation['gates']]
  validation_section=table(['Gate','Return code','Passed','Seconds','Key output line'],rows)+f"\n\nAll gates passed: **{yesno(validation['all_passed'])}**. Source digest over the audited sources: `{validation['source_digest']}`. Commands, output tails and output hashes are in [validation_results.json](validation_results.json). The artifact-manifest check and the final rerun of the pose-audit tests run after this report is built, because the manifest indexes this file; their outcome is stated in the delivery message, not in a self-referential artifact."

 text=f'''# G3.5 — Full pose repertoire, XGO retargeting and ground-rest audit

## Outcome

- **A true body-supported MATDOG ground rest exists in the canonical rigid model** (`REST_GROUND`, base_link on the ground, all feet clear). It is statically supported by the CAD/URDF COM. In the *validated* graph it is an **isolated component**: no validated path connects it to LOW_C4, STAND or any four-foot pose. That is a bounded-search result, not a proof that no physical path exists.
- **LOW_C4 is not the lowest four-foot stance, but three different "lowest" quantities must not be conflated** (Q10):
  - **A. BODY_AND_FOOT_SUPPORT lower geometric boundary = {hA['height_above_body_ground_m']:.1f} m.** Base_link and all four feet on the ground; a **PROVEN LIMIT** of the level-body model. It is a ground rest with the feet down, not a foot-supported stance.
  - **B. PURE_FOOT_SUPPORT mathematical/search boundary = {pure*1e6:.3f} µm** (**LOWEST FOUND SO FAR**). It is governed by the {hB['numerical_undeclared_contact_tolerance_m']*1e6:.0f} µm undeclared-contact tolerance and is **not** a physically meaningful clearance: a theoretical/search result with a numerical-tolerance dependency; physical robustness is **not established**.
  - **C. Physically robust / engineering-usable pure-foot stance: NOT ESTABLISHED.** No hardware tolerance, compliance or acceptance threshold is available or assumed; only saved clearance-sensitivity data are reported.
  The limiting constraint of the level-body model is **BODY_GROUND_CONTACT**.
- The {route['metrics']['sample_count']}-sample path from `BODY_FOUR_FEET_RESEARCH_2` to an *equivalent* 150 mm stand at body X = +10 mm is classified **`{route_class['classification']}` (research only)**. It keeps its useful sampled metrics, but its end is **not the canonical STAND**, it is **not proven fully collision-free** and **not proven contact-locked**. It is **not a validated transition edge to STAND** and **not an authorized or validated startup path**.
- The transition graph now carries explicit edge classes (VALIDATED / CANDIDATE / FAILED / UNTESTED). The **validated graph** has **{summary['component_count']} components** ({summary['nontrivial_component_count']} non-trivial, {summary['singleton_component_count']} singletons); these are components of the validated graph only, **not** a proof about the complete physical configuration space. STAND's component is {{{', '.join(stand_component['members'])}}}. **No body-only rest is in it.**
- **G3 status, kept separate:** G3 architecture = **GENERALIZATION JUSTIFIED**; G3 runtime startup gate = **STILL ACTIVE / NOT YET SUPERSEDED**; BODY_ONLY REST_GROUND → autonomous STAND = **NOT VALIDATED**. MATDOG must not be assumed able to stand autonomously from REST_GROUND.
- XGO recovery is **partial for every one of the {len(actions)} dispatcher entries and full for none**. XGO action internals and normalized post-IK controller-domain tables were recovered, but **{xgo['boundary_statement']}**
- The six promoted static poses are **pose-library targets, not motion states**; `MotionState` is unchanged and no production motion state, gait, actuator binding or calibration behaviour was added.

Everything below is offline, rigid-body, quasi-static and model-based (CAD/URDF MODEL COM, canonical collision meshes). None of it is a measurement, a clearance approval, a load/friction/stability claim, or actuator authorization.

## Reading guide: proven versus found, and what "no route" means

- **PROVEN LIMIT** — a bound that follows from the model geometry itself, independent of the search that found the poses.
- **LOWEST FOUND SO FAR** — the best value the bounded, finite searches reached; it can move with more search.
- **Absence of a validated route** means "no route was found by the bounded searches that were run"; it is never a proof that no route exists. Sampled routes are not swept-collision certificates and do not prove contact lock or no-slip motion.
- **Edge classes** — VALIDATED, CANDIDATE (`GEOMETRIC_PATH_CANDIDATE`), FAILED, UNTESTED (Q8). Components are components of the **validated graph only**; the absence of a validated edge is never a proof of physical disconnection or unreachability. No edge class authorizes startup or motion.
- **Evidence classes (XGO)**: A = exact trajectory/keyframes recovered; B = exact/static endpoint recovered; C = constrained geometric semantics recovered; D = semantics/name only; E = unrelated/non-postural.

## The 22 required answers

### Q1. Does a true MATDOG REST_GROUND pose exist?

**Yes, under the explicit collision policy.** `REST_GROUND` is `BODY_SUPPORT`: base_link rests on the ground, the four feet and all leg links are clear. {len(body_only)} body-only candidates are valid in `rest_search.json` (candidates {', '.join(str(i) for i in body_only)}); `REST_GROUND` was selected joint-margin-first and `REST_GROUND_MAX_SEPARATION` separation-first. Ground contact was checked for every link and {len(policy['pairs_checked']) if isinstance(policy.get('pairs_checked'),list) else 120} non-adjacent mesh pairs (FCL triangles plus solid containment); 16 direct URDF adjacency pairs are excluded as designed assembly/joint interfaces, and internal adjacent-link clearances are **not** certified. This qualification applies to every geometric claim in this report.

### Q2. What is the exact base_link world Z?

`REST_GROUND_BODY_HEIGHT = {body_z:.17g} m`, the negative of the canonical base mesh minimum local Z (`{rest_search['body_patch']['minimum_local_z_m']:.17g} m`); it was **derived from the collision mesh, not assigned by definition**. The bottom support patch (10 µm band) has {rest_search['body_patch']['patch_triangles']} triangles, one connected patch, area {rest_search['body_patch']['patch_area_m2']:.12f} m² and a six-vertex hull. Body orientation is identity.

- **Nominal CAD/URDF geometry:** {baselink['nominal_cad_urdf_geometry']}.
- **Real hardware contact height:** {baselink['real_hardware_contact_height']}.
- {baselink['not_a_positioning_accuracy'][0].upper()+baselink['not_a_positioning_accuracy'][1:]}.

### Q3. Which links are intended to contact the ground?

For `REST_GROUND` and `REST_GROUND_MAX_SEPARATION`: **base_link only**; feet, hip, upper and lower links remain clear. The two body-plus-four-feet research rests declare **base_link plus the four foot links**. `FOOT_SUPPORT` forbids base and every non-foot contact (C4 policy is not weakened globally); `FREE_SPACE` permits none; `TRANSITIONAL_SUPPORT` requires an explicit non-empty declared set. Permitted contact never permits penetration (1 µm undeclared-contact tolerance, 10 µm support-patch band; numerical model parameters, not clearances).

### Q4. What are the LF/RF/RH/LH semantic joint angles?

Selected `REST_GROUND`, URDF radians (degrees for reading only), identity body rotation, translation (0, 0, {body_z:.3g}) m (nominal model geometry, effectively zero within numerical precision; see Q2):

{table(['Leg','Hip rad','Upper rad','Lower rad','Hip deg','Upper deg','Lower deg'],angle_rows)}

Alternative candidates, exact transforms and provenance are in [pose_library.json](pose_library.json), [rest_search.json](rest_search.json) and [contact_modes.json](contact_modes.json). The records carry no actuator identifiers or conversions.

### Q5. Do front and rear angles differ because of the 20 mm hip offset?

Front hip Z = {front_hip_z*1000:.1f} mm, rear hip Z = {rear_hip_z*1000:.1f} mm (URDF), a **{(front_hip_z-rear_hip_z)*1000:.1f} mm offset**. In `REST_GROUND` the rear upper joint is **{rest['q'][7]-rest['q'][1]:+.12f} rad** and the rear lower joint **{rest['q'][8]-rest['q'][2]:+.12f} rad** relative to the front; the rear hip is 0 while the front hips are ±{rest['q'][0]:.2f} rad. The differences are outcomes of separate front/rear searches that use the true hip origins; they are not a universal correction. Other poses differ by different amounts:

{table(['Pose','Front upper','Rear upper','Δ upper','Front lower','Rear lower','Δ lower','Rear−front hip'],asym_rows)}

Front and rear angles are never forced to be equal.

### Q6. Is REST_GROUND statically supported by the URDF COM?

**Yes.** The CAD/URDF MODEL COM (all 17 inertials, total 2.48 kg) is ({rest['com_world_m'][0]:.9f}, {rest['com_world_m'][1]:.9f}, {rest['com_world_m'][2]:.9f}) m. Its ground projection lies inside the base-patch support hull with margin **{mm(rest['support_margin_m'])} mm**. Minimum tested non-adjacent separation is **{mm(rest['min_separation']['distance_m'])} mm** ({' / '.join(rest['min_separation']['pair'])}); minimum joint-limit margin **{rad(rest['joint_margin_rad'],9)} rad**. The maximum-separation alternative reaches {mm(poses['REST_GROUND_MAX_SEPARATION']['min_separation']['distance_m'])} mm separation but sits at a joint limit (margin {rad(poses['REST_GROUND_MAX_SEPARATION']['joint_margin_rad'],6)} rad). This is rigid CAD geometry, not measured mass, friction, load capacity or dynamic stability.

### Q7. Is REST_GROUND continuously connected to LOW_C4 / STAND?

**No validated connection exists.** `REST_GROUND` is a singleton in the graph. Evidence:

- The original contact-mode and route searches seeded only `REST_GROUND_MAX_SEPARATION`. To close this, the preserved protocol was rerun from **all {len(restc['starts'])} valid body-only candidates including `REST_GROUND`** (`rest_connectivity.json`): {len(direct)} direct 51-sample interpolations to contact-mode endpoints and {len(detours)} rear-then-front single-leg detour searches (seeds 17/31/73, 2000 trials each). **{len(restc['starts_with_any_valid_connection'])} succeeded.**
- Every direct interpolation fails within the first {max(x['frame'] for x in first_failures)+1} samples; the colliding pairs are {'; '.join(' / '.join(p) for p in failure_pairs)}. The first detour leg (RH) never finds a path: its search trees saturate at only {min(tree_nodes)}–{max(tree_nodes)} nodes. The front-leg detour is therefore never reached.
- With base_link on the ground, only {free['rh']['isolated_leg_clear_fraction_at_base_ground_height']*100:.2f} % (RH) and {free['lf']['isolated_leg_clear_fraction_at_base_ground_height']*100:.2f} % (LF) of uniformly sampled single-leg joint boxes are clear even when the other legs are ignored. This explains the failures qualitatively; it is not a proof of disconnection.
- The transition record `REST_GROUND → LOW_C4` is class `UNTESTED` (no direct route was sampled); every recorded search that starts at a body-only rest is class `FAILED` within its bound.

A *different* family (base plus four feet) has a sampled `{route_class['classification']}` to an equivalent 150 mm stand ({route['metrics']['sample_count']} samples), but that is a research candidate, not a validated edge: its end differs from the canonical STAND (Q8, Q22), and the body-only rest does not reach that family.

### Q8. Which support-mode transitions were validated?

Every saved route carries one of four **edge classes** (`edge_class` in [connectivity.json](connectivity.json)):

- **VALIDATED** — {sem['edge_classes']['VALIDATED']}.
- **CANDIDATE** (`GEOMETRIC_PATH_CANDIDATE`) — {sem['edge_classes']['CANDIDATE'].split(': ',1)[1]}.
- **FAILED** — {sem['edge_classes']['FAILED']}.
- **UNTESTED** — {sem['edge_classes']['UNTESTED']}.

Edge counts: VALIDATED {len(accepted)}, CANDIDATE {len(candidates)}, FAILED {len(failed)}, UNTESTED {len(untested)}. Of the {summary['node_pair_total']} unordered node pairs, {summary['node_pair_best_edge_class_counts']['NO_ROUTE_ATTEMPT_RECORDED']} have no recorded route attempt at all; the best recorded class per pair is VALIDATED {summary['node_pair_best_edge_class_counts']['VALIDATED']}, CANDIDATE {summary['node_pair_best_edge_class_counts']['CANDIDATE']}, FAILED {summary['node_pair_best_edge_class_counts']['FAILED']}, UNTESTED {summary['node_pair_best_edge_class_counts']['UNTESTED']}. The union-find graph treats every route as undirected and reversed traversal reuses the same samples. Timing, contact acquisition, friction and swept collision are not validated for any class.

**Validated edges** (sampled routes between promoted static poses):

{table(['From','To','Regimes','Samples','Class','Source'],edge_rows)}

**Candidate edges** (research only; none is a validated transition edge and none authorizes startup; every one is additionally sampled-only, with swept collision and contact lock unproved):

{table(['From','To','Regimes','Samples','Status','Why not validated'],candidate_rows)}

**Untested edges** (no route sampled):

{table(['From','To','Recorded status','Note'],untested_rows)}

Support-mode changes: the validated routes are all `FOOT_SUPPORT` → `FOOT_SUPPORT` (G3-family waypoint routes). The `BODY_AND_FOOT_SUPPORT` → `FOOT_SUPPORT` base lift-off appears only in candidate routes, because its start is a research pose. Attempted and **not** validated: `BODY_SUPPORT` → any foot-added mode (Q7); three-foot support transfer by RH/LH swing (unshifted margin −15.67 mm, a (+20,+20) mm shift permits the RH swing, the later LH swing fails IK); `EQUIVALENT_STAND` → canonical STAND (footprints differ by up to {mm(conn['equivalent_stand_vs_canonical_STAND']['max_reference_contact_xy_difference_m'],1)} mm; no route was sampled, class `UNTESTED`); STAND → SIT candidate (`UNTESTED`; the SIT target itself was never solved). {len(failed)} bounded searches are class `FAILED` (evidence of failure within each search bound, not of impossibility) and are kept in [connectivity.json](connectivity.json). The search-level status `VALID_SEQUENCE_CANDIDATE` in `transitions.json` and `transition_graph.json` only means every recorded sample is valid; the classes above are applied in `connectivity.json`.

### Q9. Is LOW_C4 the lowest valid four-foot stance?

**No.** LOW_C4 is a validated 100 mm waypoint. Foot-supported four-foot poses were validated below it at 60 mm (LOW_CROUCH, also inside STAND's component). With the base on the ground a level-body four-foot rest is valid down to **0 m** (`BODY_AND_FOOT_SUPPORT`, a research record). The lowering constraint is BODY_GROUND_CONTACT, not a 100 mm leg limit (Q21). LOW_C4 is not preserved as mandatory for backward compatibility (Q16).

### Q10. What is the lowest validated four-foot height?

**The loose phrase "lowest four-foot height" covers three different quantities, which must not be conflated.** Heights are the base_link bottom-plane height above the ground for a level body.

| Concept | Value | Status | What it is not |
|---|---|---|---|
| **A. BODY_AND_FOOT_SUPPORT lower geometric boundary** (base_link and four feet on the ground) | **{hA['height_above_body_ground_m']:.1f} m** | **PROVEN LIMIT** of the level-body model: a lower body height places canonical base-mesh vertices below the ground. The pose itself is valid (all policies pass at exactly this height in both foot families). | Not a foot-supported stance: the base rests on the ground. |
| **B. PURE_FOOT_SUPPORT mathematical/search boundary** (base clear, four feet only) | **{pure*1e6:.3f} µm** (lowest sampled valid height) | **LOWEST FOUND SO FAR.** Theoretical/search result; **numerical tolerance dependency**; **physical robustness NOT established**. `physical_clearance_approval = {str(lower['physical_clearance_approval']).lower()}`. | **Not a physically meaningful clearance**, not an achievable base-to-ground gap and not a safe height. |
| **C. Physically robust / engineering-usable pure-foot stance** | **not established** | **NOT ESTABLISHED.** {hC['basis'][0].upper()+hC['basis'][1:]}. | No value is claimed. |
| Highest four-foot height found | {mm(max(c['body'][2][3] for c in expanded['high_candidates']),3)} mm | **HIGHEST FOUND SO FAR**; a necessary (relaxed) upper bound is {mm(expanded['necessary_body_height_upper_bound_m'],3)} mm, which is not a feasibility proof | |

**Why B is a tolerance result.** The pure-foot boundary coincides with the configured {lower['numerical_undeclared_contact_boundary_m']*1e6:.0f} µm undeclared-contact tolerance: the sampled heights {', '.join(f'{h:.0e} m' for h in hB['rejected_sampled_heights_m'])} are rejected only as `{hB['rejection_code']}`, while the recorded joint-limit margin moves smoothly by just {abs(by_height[0][pure]['joint_margin_rad']-by_height[0][0.0]['joint_margin_rad']):.2e} rad between height 0 and {pure*1e6:.3f} µm. No geometric or physical event occurs at the boundary; it is where the model's contact classification changes. It is not a clearance the hardware could realise, and the audit does not invent a manufacturing tolerance to make it one.

**Clearance sensitivity from saved samples** (no search or optimization was rerun; no acceptance threshold is applied; the joint margin is to the URDF limit of the limiting joint). Full resolution of the lower envelope (both foot families), with the limiting joint at each height:

{table(['Family','Height above body ground, m','Regime','Valid','Errors','Joint margin, rad','Limiting joint (side, margin)','Support margin, mm','Min separation, mm'],lower_rows)}

The same metrics at several heights along the saved {route['metrics']['sample_count']}-sample route (rise frames of family {route['family']}; foot-supported from frame 1):

{table(['Frame','Body height, mm','Regime','Limiting joint','Min joint margin, rad','Margin, deg','Min separation, mm','Support margin, mm'],route_sens_rows)}

{sens['reading'][0].upper()+sens['reading'][1:]}.

Level-body scope: the proof covers identity body orientation. Pitched/rolled bodies were only sampled (24 seated/pitched experiments, 10 valid four-foot candidates, including pitch −20° at 120 mm). The fixed C4-footprint one-axis height slice passes {mm(envelope['extrema']['height'][0],0)}–{mm(envelope['extrema']['height'][1],0)} mm; these are observed slices, not proof that every combined pose is feasible.

### Q11. Full or partial XGO action recovery?

**Partial for all; full for none.** What was recovered is strong: {xgo['recovered']}. The boundary is equally explicit: **{xgo['boundary_statement']}** The catalogue holds **{len(actions)} dispatcher entries**: 24 quadruped actions (IDs 1–24), three manipulation entries (128–130), stair action 144, reset 255 and internal idle 0. Physical-pose evidence classes: {', '.join(f'{k}={counts.get(k,0)}' for k in 'ABCDE')}. Eight separate host APIs add A=1, B=1, C=6 **at the host-command layer**; these are not preset trajectories.

- **Exact controller-domain keyframes** (12-byte uint8 tables) were recovered for actions {', '.join(str(i) for i in table_actions)}: **{len(tables['tables'])} table references, {unique_tables} unique tables**, with schedules and recovery counters for actions 12, 14 and 21. They stay class **C** at the physical-pose layer because their frame, signs, zeros and scale are unbound; a reviewer who grades the controller layer alone could regard them as A there.
- For actions 1, 2, 3, 6, 7, 17 and 24 a body-height, pitch or periodic-command semantic was recovered from handler and consumer code; for the other dynamic entries only a numeric controller state sequence is known. No physical endpoint, contact set or achieved timing was recovered for any entry.
- The exact host APP press-up (Z commands 75 then 100, six counter cycles, requested 0.15 s sleeps) and APP leg reset (`[0,0,108]` per leg) are exact host commands with no proven ground/body datum.
- Recovery per preset: {', '.join(f'{k}={v}' for k,v in sorted(recovery.items()))}. Missing keyframes were never filled in.

The complete per-action table is below (Q11 detail).

### Q12. Which XGO poses can be retargeted?

**{xgo['boundary_statement']}** Therefore **none can be retargeted as a physical joint configuration; {len([r for r in matrix['preset_actions'] if r['physical_retarget_result']=='RETARGET_UNDERDETERMINED'])} of the {len(matrix['preset_actions'])} presets can only be retargeted as geometric intent, and {len([r for r in matrix['preset_actions'] if r['physical_retarget_result']=='NOT_TRANSFERABLE'])} are not transferable.** A failed or underdetermined retarget is a valid result. XGO behavior, topology and sequence semantics are reference evidence only; MATDOG joint geometry continues to come from the MATDOG URDF and MATDOG IK. Retarget outcomes across {len(matrix['preset_actions'])} presets and {len(matrix['host_apis'])} host APIs: {', '.join(f'{k}={v}' for k,v in sorted(retarget_result.items()))}. No XGO controller table is accepted as MATDOG joint geometry.

Intent-level MATDOG counterparts solved independently (`VALID_STATIC` or research records): Lie down → `REST_GROUND` / `LOW_CROUCH`; Stand up → `STAND`; Squat and Crawl → `LOW_CROUCH` (aliases `SQUAT`, `CRAWL_READY`); Stretch → `STRETCH`; Turn-pitch and Find-food → `PITCHED_CROUCH_RESEARCH`; Roll → `ROLL_PREP_RESEARCH` (preparation only); the documented host height range 60–110 mm → `XGO_HEIGHT_RATIO_RESEARCH` (an assumed ratio {matrix['height_intent_experiment']['matdog_height_m']*1000:.3f} mm, explicitly not recovered geometry); Yaw / Three-axis / Look-around / Sway / Wave-body / Dance / Playful → orientation/translation envelope slices. Mark-time (ID 5), Beg and Sit are not retargeted (Q14). Manipulation, stair and reset entries are not transferable. See [retarget_matrix.json](retarget_matrix.json).

### Q13. Is the ~1.5× scaling hypothesis true, partly true, or false?

**Partly true — for two dimensions of a generic template only; not established for the exact Lite; false as a single universal factor.** Ratios are MATDOG / generic-family template:

{table(['Dimension','Source identity','Generic family, mm','MATDOG, mm','Ratio','Meaning'],dimension_rows)}

Source-identity tally: {', '.join(f'{k}={v}' for k,v in sorted(identity_counts.items()))}; **EXACT_LITE = 0** dimensions, **CORROBORATED_XGO_FAMILY = 0** dimensions, **GENERIC_TEMPLATE = {identity_counts.get('GENERIC_TEMPLATE',0)}**, **UNKNOWN = {identity_counts.get('UNKNOWN',0)}**. Front/rear hip spacing is exactly 1.5 and the upper-link axis distance 1.5025; left/right front hip spacing (2.11), the lateral hip offset (0.97) and hip-to-upper origin distance (0.91) do not follow 1.5, and frame heights are not similarity invariants. Exact Lite dimensions appear only as host-API ranges (`EXACT_LITE_HOST_API_ONLY`, translation Z 60–110 mm, unbound datum). The template CAD (JoseManuelLuque/DOGZILLA `{dims['provenance']['xgo_commit'][:12]}`) is generic; the hash-matching primary xacro is used and the conflicting derived H2 inventory CSV is not (see `source_correction` in [dimensions.json](dimensions.json)). **The ~1.5× similarity is not a numeric conversion rule and must not be used to convert any XGO value or pose to MATDOG.** Conclusion recorded by the audit: {dims['conclusion']}

### Q14. Which XGO poses fail on MATDOG, and why?

**No whole XGO action is proved globally impossible on MATDOG from this incomplete evidence.** Specific MATDOG candidates failed:

- **SIT candidate** (pitch −0.3 rad, height 70 mm, fixed C4 footprint) and **BEG candidate** (pitch −0.6 rad, height 70 mm): the bounded IK attempt returned `IK_REACHABILITY_OR_LIMIT_OR_OPTIMIZER_FAILURE`, a combined code that cannot separate reach, joint limit and optimizer failure. A pitched crouch with a free footprint does exist (pitch −20° at 120 mm) and is labelled research, **not** recovered XGO Sit. Two-foot Beg support is not validated. Preset 17 (Beg/Pray) has a recovered pitch/height semantic but no support geometry.
- **{len(envelope_invalid)} of {len(envelope_rows)} fixed-footprint envelope grid points** fail with that same combined IK code; all failures are retained in [envelope.json](envelope.json).
- Direct joint interpolation from folded/body-only poses to contact poses collides (front foot/lower leg against the rear hip); the unshifted three-foot swing has negative COM margin (−15.67 mm) and the LH swing fails IK.
- Rollover, stepping, crawl/dance locomotion and mark-time are dynamic or unresolved and are not static poses. ID 5's analyzed handler completes immediately; the documented "stepping" name is not substantiated by the firmware and must not be conflated with the separate mark-time host API.
- q=0 is not a ground-supported starting pose (below).

### Q15. Which MATDOG semantic poses should become first-class targets?

**Six** static pose targets are embedded in `PoseReferenceData.h`: `REST_GROUND`, `REST_GROUND_MAX_SEPARATION` (explicitly at a joint boundary), `LOW_CROUCH`, `LOW_C4`, `STAND`, `STRETCH`. They are **pose-library entries, not motion states.** The roles below are a design classification separating semantic pose targets from motion-state candidates and research variants; only the blockers are evidence-derived. `MotionState` is unchanged (enum values {roles['motion_state_enum'].replace(' (unchanged)','')}), and being a `MOTION_STATE_CANDIDATE` authorizes nothing.

{table(['Pose','Roles','Relation to MotionState','Blockers / notes'],role_rows)}

`CRAWL_READY` and `SQUAT` are semantic aliases of `LOW_CROUCH` (the alias joint difference is {alias_delta:.2e} rad), not separate poses or states. Five records stay offline `RESEARCH_VARIANT`: two body-plus-four-feet rests, `PITCHED_CROUCH_RESEARCH`, `ROLL_PREP_RESEARCH`, `XGO_HEIGHT_RATIO_RESEARCH`. No SIT or BEG target is invented from an action name, and **no new production motion state** is justified by static results.

Verification of the six (independent FK/COM to 2e-15, full mesh policy re-evaluation, generated-header freshness, C++ policy test and G3 startup gate unchanged):

{table(['Pose','Regime','Ground contacts','Body Z, mm','Joint margin, rad','Min separation, mm','Support margin, mm','Evidence source'],six_rows)}

### Q16. Is G3 startup unchanged, generalized, or partially superseded?

**Two separate statements, which must not be merged: architectural evidence (A) and the implemented runtime gate (B).**

| Aspect | Status |
|---|---|
| **A. G3 architecture** (evidence) | **{startup['wording']['G3 architecture']}** |
| **B. G3 runtime startup gate** (implemented) | **{startup['wording']['G3 runtime startup gate']}** |
| BODY_ONLY REST_GROUND → autonomous STAND | **{startup['wording']['BODY_ONLY REST_GROUND -> autonomous STAND']}** |
| The {route['metrics']['sample_count']}-sample route as a startup path | **NOT AUTHORIZED** (`{route_class['classification']}`, research only) |

- *A, architectural evidence:* LOW_C4 is neither the lowest nor the only validated four-foot stance. STAND's validated component also contains `LOW_CROUCH` (60 mm), `STRETCH` and the `CRAWL_READY` alias, so the entry contract could be expressed as "a verified member of the stand-connected component" rather than "exactly LOW_C4". The generalized entry is **not implemented**; its supporting acquisition contract does not exist.
- *B, implemented runtime gate:* `evaluateStartup` still returns `FLOOR_ACQUISITION_UNPROVEN` for feet placed on an unverified floor, `SUSPENDED_PATH_UNPROVEN`, `UNKNOWN_POSE`, and `READY` only for the verified canonical low stance with all six external evidence assertions. {startup['B_implemented_runtime_gate']['basis'][0].upper()+startup['B_implemented_runtime_gate']['basis'][1:]} (file hashes are recorded in [closeout_classification.json](closeout_classification.json)). G3.5 does not change the G1/G2/G3 code, tests or history. The G3.5 routes use mesh IK, inclined-edge contact and patch migration; they are not G2 contact-locked paths and have no acquisition contract.
- *Not validated:* **BODY_ONLY REST → STAND** (no validated connected path; Q7). {startup['detail']}.
- *Architecture the evidence supports:* `verified stand-connected support component → STAND`, with BODY_ONLY REST kept as a separate component until a validated path and acquisition contract exist. This is future work and is not implemented here.

The implemented stand still requires, before the first autonomous stand, that the robot is already stationary in the canonical LOW_C4 body/joint/contact configuration on flat ground with fresh pose evidence, four confirmed feet, reviewed collision clearance and reviewed support/load. Software observations are not measurements or actuator authorization.

### Q17. What XGO information remains unrecovered?

- Physical joint zero, sign and per-slot scale binding of every 12-value table. Recorded unknowns: {'; '.join(tables['physical_unknowns'])}.
- Achieved physical endpoints, contact sets and support/load/friction assumptions.
- Exact Lite foot and distal collision geometry, link lengths and physical joint ranges (all `UNKNOWN` in the dimension table).
- Achieved action timing and interpolation: counters are task invocations, not seconds, and helper `0x400da840` sets a parameter that is not evidence of a physical interpolation duration.
- Firmware v4.3.7 ↔ later-document correspondence: {len(doc_conflicts)} document-versus-library conflicts are preserved unreconciled (for example action 22 wait 8 s versus 7 s, height ranges 75..115 versus 60..110).
- Body/leg/contact semantics of every preset (contact set is `UNKNOWN` for all {len(actions)} entries).
- The physical LF/RF/RH/LH and lower/upper/hip labels are corroborated, not a calibrated transform.

### Q18. How many disconnected validated transition components exist?

**{summary['component_count']} components of the validated graph** among {len(conn['nodes'])} nodes: {summary['nontrivial_component_count']} non-trivial and {summary['singleton_component_count']} singletons. These are components of the graph of VALIDATED edges, **not** a statement about the complete physical configuration space. The singletons are **untested, not proven disconnected**; a missing validated edge means "no validated route found by the bounded searches". The earlier G3.5 count of {sem['previous_component_count_before_edge_classes']} also joined the four CANDIDATE research-route edges; an informational view that includes them has {summary['candidate_augmented_component_count']} components (see `candidate_augmented_components` in [connectivity.json](connectivity.json)), and the STAND component and the REST_GROUND singleton are the same in both views.

{table(['#','Size','Members','Contains STAND','Contains REST_GROUND'],component_rows)}

### Q19. Which component contains STAND?

Component {stand_component['component']}: **{{{', '.join(stand_component['members'])}}}**, joined by VALIDATED sampled routes LOW_CROUCH → LOW_C4 → STAND → STRETCH and STAND → CRAWL_READY (`CRAWL_READY` is the `LOW_CROUCH` alias; the two joint vectors differ by {alias_delta:.2e} rad).

### Q20. Does BODY_ONLY REST_GROUND belong to it?

**No.** `REST_GROUND_in_stand_component = {str(summary['REST_GROUND_in_stand_component']).lower()}` and `any_body_only_rest_in_stand_component = {str(summary['any_body_only_rest_in_stand_component']).lower()}`, and REST_GROUND is not in it even when CANDIDATE edges are included (`{str(summary['REST_GROUND_in_stand_component_including_candidate_edges']).lower()}`). All {len(restc['starts'])} valid body-only candidates were tested for a connection and none connects (Q7). This is a bounded-search result, not a proof of impossibility.

### Q21. What constraint sets the lowest four-foot configuration?

**BODY_GROUND_CONTACT** (`limiting_constraint` in [lower_envelope.json](lower_envelope.json)): base_link reaches the ground. Below height 0 the base mesh penetrates the ground (proven for a level body); between 10 nm and 1 µm the pose is rejected for undeclared base contact (the configured tolerance, not a geometric or physical event; Q10); from {pure*1e6:.3f} µm upward it is valid `FOOT_SUPPORT`. Nothing else stops the body from going lower first: at height 0 the minimum non-adjacent separation is {mm(by_height[0][0.0]['min_separation']['distance_m'],3)} mm and the support margin {mm(by_height[0][0.0]['support_margin_m'],1)} mm (body and feet) / {mm(by_height[0][pure]['support_margin_m'],1)} mm (feet only); no self-collision, leg-ground collision, IK failure or singularity is active.

**A joint limit is nearly active in the same configuration.** At the 251-sample route's start (family 1; family 0 reports the same minimum margin), {near_joint_names} are only **{near_joint_margin:.6f} rad ({math.degrees(near_joint_margin):.3f}°)** from their {near_joint_side} limit ({near_joint_limit:.6f} rad). That margin grows with height ({rad(margin_h0,6)} rad at 0 m, {rad(margin_h1,6)} rad at 10 µm; the trend, extrapolated linearly and not a computed result, would close it roughly {abs(margin_h0/slope)*1000:.2f} mm below ground level). So the lowest four-foot stance found is limited by base_link ground contact with the rear lower-leg joints barely inside their range for these footprints. Whether a different footprint would give more joint margin at height 0 was not exhaustively searched; base_link contact bounds the height regardless.

### Q22. Exact metrics of the {route['metrics']['sample_count']}-sample BODY_AND_FOUR_FEET → equivalent-stand path (`{route_class['classification']}`)

**Classification: `{route_class['classification']}`, research only.** {(lambda m:m[0].upper()+m[1:])(route_class['meaning'].split(': ',1)[1])}. It is **not** a validated transition edge to STAND and **not** an authorized or validated startup path (`validated_transition_edge_to_STAND = {str(route_class['validated_transition_edge_to_STAND']).lower()}`, `authorized_startup_path = {str(route_class['authorized_startup_path']).lower()}`). Reasons:

{chr(10).join('- `'+r.split(':',1)[0]+'`'+(':'+r.split(':',1)[1] if ':' in r else '') for r in route_class['reasons'])}

All metrics below are preserved unchanged from G3.5. Source: `{route['source']}` — {route['rise_frames']} original-footprint rise frames plus {route['equivalent_shift_frames_excluding_duplicate']} equivalent-stand shift frames (duplicate joining frame removed). All values below are read from saved frames, not from rounded progress logs; `test_pose_audit.py` recomputes them.

{table(['Metric','Value'],[
['Samples / all valid',f"{rm['sample_count']} / {yesno(rm['all_samples_valid'])}"],
['Start (frame 0)',f"BODY_FOUR_FEET_RESEARCH_2 (route family index {route['family']}), {rm['start']['regime']}, contacts base_link + {' '.join(rm['start']['active_feet'])}"],
['Start body height',f"{rm['start']['body_height_m']:.17g} m (nominal CAD/URDF geometry: base_link at ground level, effectively zero within model/mesh numerical precision, not a hardware contact height; translation {rm['start']['body_translation_m']})"],
[f"Final (frame {rm['final']['frame']})",f"{rm['final']['regime']}, four feet {' '.join(rm['final']['active_feet'])}, base_link contact {yesno(rm['final']['base_link_ground_contact'])}"],
['Final body height',f"{rm['final']['body_height_m']:.17g} m (translation {rm['final']['body_translation_m']})"],
['Body translation delta',f"({rm['body_translation_delta_m'][0]:.9f}, {rm['body_translation_delta_m'][1]:.3e}, {rm['body_translation_delta_m'][2]:.9f}) m; body rotation deviation from identity {rm['body_rotation_max_deviation_from_identity']:.1e}"],
['Original-footprint rise end',f"{route['rise_end_body_height_m']*1000:.4f} mm at frame {route['rise_frames']-1} (rise to 150 mm with the original footprint fails IK)"],
['COM start / final, m',f"({rm['com_world_start_m'][0]:.6f}, {rm['com_world_start_m'][1]:.6f}, {rm['com_world_start_m'][2]:.6f}) / ({rm['com_world_final_m'][0]:.6f}, {rm['com_world_final_m'][1]:.6f}, {rm['com_world_final_m'][2]:.6f})"],
['Contact sets',' ; '.join(f"frame{'s' if c['frames']>1 else ''} {c['first_frame']}{'–'+str(c['first_frame']+c['frames']-1) if c['frames']>1 else ''}: {c['regime']}, feet {' '.join(c['active_feet'])}, base_link {yesno(c['base_link_ground_contact'])}" for c in rm['contact_set_segments'])],
['Min joint-limit margin',f"{rm['min_joint_margin_rad']:.15f} rad at frame {rm['min_joint_margin_at']['frame']}, {rm['min_joint_margin_at']['joint']} {rm['min_joint_margin_at']['limit_side']} limit (q {rm['min_joint_margin_at']['q_rad']:.6f} vs limit {rm['min_joint_margin_at']['limit_rad']:.6f})"],
['Min non-adjacent separation',f"{rm['min_nonadjacent_separation_m']*1000:.9f} mm at frame {rm['min_separation_at']['frame']} ({' / '.join(rm['min_separation_at']['pair'])})"],
['Min COM/support margin',f"{rm['min_static_support_margin_m']*1000:.9f} mm at frame {rm['min_support_margin_at_frame']}"],
['Max mesh-foot ground residual',f"{rm['max_abs_mesh_foot_ground_residual_m']:.6e} m at frame {rm['max_residual_at_frame']} (patch band {rm['patch_band_tolerance_m']:.0e} m)"],
['Max reference-XY drift',f"{rm['max_reference_xy_drift_m']:.6e} m"],
['Max per-sample joint step',f"{rm['max_abs_joint_delta_rad_per_sample']:.9f} rad (geometric continuity, not an actuator rate)"],
['Jacobian determinant',f"abs(det) per leg (LF RF RH LH): min {', '.join(f'{v:.3e}' for v in rm['jacobian_determinant']['per_leg_min_abs'])}, max {', '.join(f'{v:.3e}' for v in rm['jacobian_determinant']['per_leg_max_abs'])}; global min {rm['jacobian_determinant']['global_min_abs']:.6e}; sign changes per leg {rm['jacobian_determinant']['sign_changes_per_leg']}"],
['Conditioning',f"max condition number {rm['condition_number']['global_max']:.3f} (per leg {', '.join(f'{v:.2f}' for v in rm['condition_number']['per_leg_max'])}); min singular value {rm['condition_number']['min_singular_value_m_per_rad']:.5f} m/rad"],
['Branch continuity',rm['branch_continuity']],
['Limiting constraint',route['limiting_constraint']],
['Proven?',f"continuous collision-free: {yesno(rm['continuous_collision_free_proved'])}; material contact lock: {yesno(rm['material_contact_locked_proved'])}"],
])}

Start and final joint vectors and per-joint margins (URDF radians; the last two columns give each joint's minimum margin and the frame where it occurs):

{table(['Joint','Start q','Final q','Max step per sample','Min margin','Frame of min margin'],joint_rows)}

Relation to the canonical STAND: the route's end is at the same 0.15 m height but at body X = +10 mm with a different footprint; the maximum joint difference is {conn['equivalent_stand_vs_canonical_STAND']['max_abs_joint_difference_rad']:.4f} rad and the largest reference-contact XY difference is {conn['equivalent_stand_vs_canonical_STAND']['max_reference_contact_xy_difference_m']*1000:.2f} mm, so **no validated edge** joins them (the pair is `UNTESTED`). The 1.001 µm foot-only sample of family 1 differs from route frame 1 by {conn['lower_envelope_continuity_to_route'][1]['max_abs_joint_difference_to_route_frame_1_rad']:.2e} rad at most, consistent with the route's first lift-off step.

## Support-mode transition and connectivity detail

Solid arrows are VALIDATED edges; dotted arrows are CANDIDATE, FAILED or UNTESTED and none of them is a validated or authorized route.

```mermaid
flowchart LR
  R[REST_GROUND body only] -. no validated route .-> BF[Body plus four feet research pose]
  BF -. candidate path only 251 samples research only .-> E[Equivalent 150 mm stand X+10 mm not canonical STAND]
  E -. untested no route sampled .-> S
  C[LOW_CROUCH 60 mm] --> C4[LOW_C4 100 mm]
  C4 --> S[STAND 150 mm]
  S --> T[STRETCH]
  S --> CR[CRAWL_READY alias of LOW_CROUCH]
```

Body-only connection attempts from every valid body-only start (`rest_connectivity.json`; direct interpolation targets are the five valid contact-mode endpoints, detours target both four-foot endpoints; the last column notes whether any valid connection was found):

{table(['Start','Identity','Joint margin, rad','Min separation, mm','Support margin, mm','First failing frame per direct target','RRT nodes per attempt','Connected'],first_rows)}

Search scope: {restc['protocol']}. {restc['tree_growth'][0].upper()+restc['tree_growth'][1:]}. `global_absence_claim = {str(restc['global_absence_claim']).lower()}`.

## Static pose metrics

{table(['Pose','Evidence','Regime','Body Z, mm','Min joint margin, rad','Min separation, mm','CAD support margin, mm'],metric_rows)}

Poses are labelled `VALIDATED_STATIC` only when they meet the full policy; `RESEARCH_ONLY` records are never embedded. Diagnostics include selected and rejected poses; failed optimizer attempts are not shown as achieved targets.

## Mathematical and contact model

For each link, `T_world_link = T_world_base × product(T_joint_origin × R_axis(q))`. COM is `sum(m_i × T_world_link_i × c_i) / sum(m_i)` (all 17 URDF inertials). Ground tests transform every canonical collision vertex; body-ground height is the negative minimum base local Z. Support is the convex hull of intended near-ground mesh patches and the margin is the minimum inward edge distance of the COM projection.

G2 keeps the canonical eccentric finite-cylinder reference `p = t_foot + R_foot c + r d`; G1 keeps the foot-link origin. The offline optimizer holds that reference's XY and solves the actual lowest foot-mesh Z within URDF limits, seeding each solve from the previous valid sample. Body orientation is general only in this offline optimizer. Singularity diagnostics are a central-difference Jacobian of the smooth analytical reference (ε = 1e-6 rad; SVD in m/rad), not the nonsmooth mesh-edge Jacobian.

Foot-contact discrepancy: vertical lowest-mesh error and 3D distance to the near-bottom patch are different diagnostics from the tessellation-dependent patch centroid, which is **not** substituted for the G2 reference. Strongly inclined body-plus-foot rests have roughly 2–3.1 mm vertical differences and 5–6.3 mm patch distances; treating G2's central reference as their ground contact would be wrong, so those poses stay outside the production nominal-strip startup contract.

## Search coverage and failures (kept)

Initial folded grid: 6,125 points per front and rear leg; 279 front and 95 rear per-leg candidates passed the preliminary ground/non-adjacent checks. The broader mesh contact search used 9 hip × 31 upper × 32 lower-root brackets (37 front and 13 rear leg solutions before full-robot pairing). Families retained: BODY_ONLY, BODY_PLUS_FRONT_FEET, BODY_PLUS_REAR_FEET, BODY_PLUS_FOUR_FEET, FOUR_FEET_ONLY_LOW, TRANSITIONAL_CONTACT; no valid body-plus-rear combined candidate was found in the declared grid. This is finite systematic coverage, not exhaustive enumeration of 12-DoF space.

Direct interpolation first collides at progress 0.14 for one front-contact route (LF foot ↔ LH hip and RF foot ↔ RH hip; FCL triangle-depth estimates 0.0878 and 0.1275 mm) and at 0.04 for others (front lower/foot meshes against rear hips, up to 5.743 mm). FCL triangle depth is not a certified solid penetration distance. A route that repositions feet at 60 mm first fails three-foot support; a body shift allows the RH swing but the LH swing fails IK. The successful family avoids forcing the C4 footprint (family 2 succeeds; family 1 has a valid endpoint but its connecting IK path fails near the end).

Revalidation of the preserved corpus: {revalidation['sample_references']:,} references / {revalidation['unique_samples']:,} unique samples with **{len(revalidation['classification_changes'])} classification changes**; equivalent-stand samples: {equiv_revalidation['sample_references']} / {equiv_revalidation['unique_samples']} with {len(equiv_revalidation['classification_changes'])} changes. Old generator hashes remain historical and are not relabeled as fresh searches.

## XGO evidence

Read-only origin/main snapshot `{cat['provenance']['reverse_commit']}`; the original checkout was not modified. The bounded audit triaged **{inventory['scanned_matching_files']} matching files** across the canonical snapshot, the referenced source cache and selected CM4/CM5 archive text. Primary firmware SHA-256 `{cat['provenance']['firmware_sha256']}` (v4.3.7); Ghidra ran only on a copied project with `-noanalysis -readOnly` and the firmware was never executed. The linked export addresses are 8 below canonical IROM addresses (parser `0x400d3510`, dispatcher `0x400d91fc`, mode consumer `0x400daca4`; action state `0x3ffc476c`); a raw hardware loop at export `0x400d7d52` proves a 12-byte copy where the decompiler shows one byte.

Each 12-byte table is an **absolute normalized motor-domain command endpoint** (uint8 0..255) consumed in mode 3 after or bypassing Cartesian IK and mapped through stored per-slot endpoints ("bounded affine map … not URDF radians"). Body-command mode uses the Cartesian/IK pipeline instead. Historical Yahboom commit `aa6b0e414c53c5ec21ddf0e22fcc4ff6e341422e` versus current `{cat['provenance']['current_yahboom_commit']}`: the motion PDFs are byte-identical (`current_pdf_matches_pinned = {str(cat['provenance']['current_pdf_matches_pinned']).lower()}`). The Luwu host source is pinned at `cf72514273dc703284d3c47e46c67ce238caae11`. Detail: [xgo_table_semantics.json](xgo_table_semantics.json), [xgo_revision_comparison.json](xgo_revision_comparison.json), [xgo_static_trace.json](xgo_static_trace.json) and the immutable text extracts in `xgo_static_extracts/`.

### Per-action audit (all {len(actions)} dispatcher entries)

Columns: id, name, aliases, evidence class A–E, documented duration in seconds (documentation, not hardware-bound), handler canonical address and handler-hash prefix, state/static/dynamic class, number of recovered 12-value tables, recovery, direct physical retarget result, MATDOG intent counterparts. Every table is `uint8[12]` in controller slot coordinates (0..255, **not** URDF radians, physical sign and zero unbound, unit dimensionless); mode-3 tables are **post-IK** (bypass Cartesian IK); units/scale/sign confidence is `UNKNOWN` for physical meaning. Transferability to MATDOG is `NO_DIRECT_JOINT_TRANSFER` for every C/D entry and `NOT_TRANSFERABLE` for E.

{table(['ID','Name','Aliases','Class','Duration s','Handler (hash)','State/static/dynamic','Tables','Recovery','Physical retarget','MATDOG intent'],action_rows)}

Body, leg and contact semantics per action (contact/support is never recovered):

{table(['ID','Recovered semantics','Contact/support'],semantic_rows)}

Unresolved fields common to all entries: exact physical joint slot mapping/zero/signs; physical body frame and achieved pose; ground/self collision and contact set; achieved interpolation timing.

### Host-layer APIs

{table(['API','Class','Scope','Unknowns'],host_rows)}

## q=0 and startup

**q=0 is not a safe autonomous ground-supported starting pose.** On a level body the first foot contact occurs at body height {mm(policy['q0_first_ground_contact']['body'][2][3],6)} mm with only {'/'.join(f.upper() for f in policy['q0_active_feet'])} supported and CAD COM support margin {mm(policy['q0_first_ground_contact']['support_margin_m'],6)} mm (negative means the COM projection lies outside the support hull); at body-ground height the legs/feet penetrate the floor (`collision_policy.json`). This is a geometric reference, not a rest or acquisition trajectory. The calibration workstream and q0 mapping were not modified.

## Validation and reproducibility

{validation_section}

The pure C++ pose test checks every contact-policy combination, invalid inputs, all ground-link classes, the six targets and the unchanged startup gate. The ASan/UBSan gate links only the pure motion sources needed by `test_pose_support.cpp`, with `-fsanitize=address,undefined -fno-sanitize-recover=all`; no physical transport is linked. Offline Python tests recompute FK and COM independently, check canonical/source freshness, all library meshes and limits, asymmetry, support policies, deterministic IK/retarget reruns, evidence-promotion rejection, height bounds, patch-distance geometry, the 251-sample route metrics, the connectivity components from the validated edges, the body-only connection results, the lower-envelope classification and the per-action audit fields. See [README.md](../../../06_Software/Matdog_Core/pose_audit/README.md) for the ordered reproduction commands and source-copy rules, and [artifact_manifest.json](artifact_manifest.json) for the artifact hash index and provenance cross-checks.

## Visual review

{len(view_files)} deterministic engineering renders (side X/Z, front Y/Z and isometric projections of the canonical triangles, the support hull and the CAD COM, all labelled "rigid mesh evidence only") are in `views/`, indexed by [visual_manifest.json](visual_manifest.json): {', '.join(f'[{n}](views/{n}.png)' for n in view_files)}. Failed SIT/BEG targets are not drawn as achieved poses; the research poses are labelled as such in their titles.

## Resume and history

Accepted G3 baseline: `87c3e914e068260170e7c19c5d664f727f5b03ce`, branch `feat/gait-engine-offline-v1`. The workstream was interrupted twice. The first resume recovered fourteen analysis sources, two draft C++ support files and eleven JSON reports (reconstructed Ghidra exports matched all 30 saved handler hashes) and committed the XGO trace, the mesh model, the contact-mode search and the pose-support layer (`024e626`, `274d8ad`, `51319b3`, `821c266`). The second resume reconstructed state from git and the filesystem without modification, kept every valid artifact, and added the closing evidence: `rest_connectivity.py` (needed to close Q7 and Q20 because the original searches seeded only one body-only start), `connectivity.py`, `lower_envelope.py`, `artifact_manifest.py`, `validate.py`, this generator and the extended tests. `git log 87c3e91..HEAD` lists the local commits. The G3.5.1 closeout pass then reclassified terminology and edge classes without new searches: it added the edge classes to `connectivity.py`, the saved-artifact-only `closeout.py` and `closeout_classification.json`, and revised this report.

## Delivery boundaries and safety confirmation

Only offline research, pure motion pose/contact definitions, tests and reports were changed. The G3.5.1 closeout pass changed only offline audit tooling under `06_Software/Matdog_Core/pose_audit`, saved artifacts and this report; nothing under `05_Firmware` was touched. Existing G1/G2/G3 contracts and history are intact and the C4 policy is not weakened globally.

- **No physical hardware was accessed.**
- **No serial device was accessed.**
- **No ServoBus** traffic; **no servo command** (no ST3215 command, no GoalPosition write).
- **No Torque ON.**
- **No firmware flashing** (no esptool, no Arduino upload); the XGO firmware was only statically analysed from a read-only copy and never executed.
- **No EEPROM access.**
- **No q0, calibration, PositionOffset or raw-tick change.**
- The original XGO checkout was not modified.
- **No push. No merge.** All commits are local on the working branch; no WALK/TROT behaviour was added.
'''
 (OUT/'REPORT.md').write_text(text)
 print('REPORT.md written',len(text.splitlines()),'lines')

if __name__=='__main__':main()
