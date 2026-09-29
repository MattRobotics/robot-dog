"""Evidence-gated retarget matrix. Unknown source geometry stays unknown."""
import json
from model import sha
from survey import OUT,save

def classify(action):
 if action['evidence_class']=='E':return 'NOT_TRANSFERABLE'
 if action['recovered_joint_targets'] is None and action['recovered_leg_targets'] is None and action['recovered_body_pose'] is None:return 'RETARGET_UNDERDETERMINED'
 # This audit has no bound exact Lite physical targets. New evidence must first
 # acquire an explicit frame/scale/contact contract and a MATDOG solver result.
 return 'RETARGET_UNDERDETERMINED'
def build():
 cat=json.loads((OUT/'xgo_catalogue.json').read_text());lib=json.loads((OUT/'pose_library.json').read_text());rows=[]
 analogues={1:['REST_GROUND','LOW_CROUCH'],2:['STAND'],3:['LOW_CROUCH'],6:['LOW_CROUCH','STAND'],7:['ROLL_PREP_RESEARCH'],8:['PITCHED_CROUCH_RESEARCH'],9:['yaw envelope'],10:['roll/pitch/yaw envelope'],11:['support_transfer.json three-foot interval'],12:['PITCHED_CROUCH_RESEARCH','SIT_CANDIDATE failed'],13:['support_transfer.json three-foot interval'],14:['STRETCH'],15:['x/y envelope'],16:['roll envelope'],17:['BEG_CANDIDATE failed'],18:['PITCHED_CROUCH_RESEARCH'],19:['support_transfer.json three-foot interval'],20:['roll/pitch/yaw envelope'],21:['LOW_CROUCH','STAND'],22:['yaw envelope'],23:['x/y/roll/pitch envelope'],24:['x/y/roll/pitch envelope']}
 for a in cat['actions']:
  status=classify(a);dynamic=a['action_id'] in (3,4,5,11,13,19,23)
  rows.append({'action_id':a['action_id'],'name':a['canonical_name'],'evidence_class':a['evidence_class'],'source_semantics':a['semantics'],'recovery':'NOT_TRANSFERABLE' if status=='NOT_TRANSFERABLE' else ('SEMANTICS_ONLY' if a['evidence_class']=='D' else 'PARTIAL_GEOMETRY'),'exact_controller_tables_recovered':len(a['recovered_controller_tables']),'physical_retarget_result':status,'normalized_source':{'body_orientation':None,'height_over_leg_length':None,'contacts_relative_to_hip_over_leg_length':None,'physical_support_set':None,'reason':'no source-bound physical frame/length/contact set; controller unit interval is not spatial normalization'},'matdog_intent_experiments':analogues.get(a['action_id'],[]),'intent_experiment_scope':'independently specified MATDOG geometry, not recovered XGO endpoints','dynamic_behavior':'DYNAMIC_ONLY_NOT_STATIC' if dynamic else None,'direct_geometry_transfer_allowed':False})
 host=[]
 for a in cat['additional_host_pose_apis']:
  host.append({'name':a['name'],'host_evidence_class':a['evidence_class'],'recovery':'EXACT_KEYFRAMES_RECOVERED' if a['evidence_class']=='A' else ('ENDPOINT_RECOVERED' if a['evidence_class']=='B' else 'PARTIAL_GEOMETRY'),'recovery_scope':a['scope'],'physical_retarget_result':'RETARGET_UNDERDETERMINED','reason':'host coordinate datum/contact binding unknown','matdog_intent_experiments':['LOW_CROUCH','STAND'] if a['name']=='APP_PRESS_UP' else ['pose envelope parameter slices']})
 return {'provenance':{'generator_sha256':sha(__file__),'sources':{n:sha(OUT/n) for n in ['xgo_catalogue.json','pose_library.json','dimensions.json']}},'preset_actions':rows,'host_apis':host,'normalized_matdog_poses':[{'name':r['name'],'normalized':r['normalized'],'result':'VALID_STATIC','evidence':r['evidence'],'source':'pose_library.json'} for r in lib['poses']],'normalization_rule':'explicit characteristic length and body/hip frames; transfer orientation and ratios only after source geometry is known; solve all four canonical MATDOG legs independently','height_intent_experiment':{'source_host_range':[60,110],'source_units':'documented mm with unbound ground datum','matdog_height_m':.15*60/110,'status':'VALID_STATIC MATDOG research candidate','mapping_is_assumption':True,'scope':'endpoint-ratio experiment only; not evidence that XGO lie touches the body to ground'}}
def main():save('retarget_matrix',build());print('RETARGET_MATRIX = generated with physical-evidence gate')
if __name__=='__main__':main()
