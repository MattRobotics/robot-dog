"""Static evidence builder. Reads pinned sources/firmware, never imports vendor code."""
import argparse,csv,json,re,struct,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
from collections import Counter
from model import ROOT,sha,Model
from survey import OUT,save
HANDLERS={0:'7c28',1:'7da8',2:'7df4',3:'7e40',4:'7eac',5:'7ef8',6:'7f04',7:'7fa0',8:'7fec',9:'8038',10:'8084',11:'810c',12:'8214',13:'82a0',14:'83b0',15:'8460',16:'8560',17:'865c',18:'8714',19:'8930',20:'8a50',21:'8bf4',22:'8aec',23:'8b54',24:'8ba4',128:'8cdc',129:'8dac',130:'8e80',144:'8f54',255:'7d08'}
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--xgo',type=Path,default=Path('/tmp/matdog-g35-xgo'));ap.add_argument('--exports',type=Path,default=Path('/tmp/matdog-g35-ghidra-export'));ap.add_argument('--cache',type=Path,default=Path('/home/matteo-manicardi/robotics-reverse/work/milestone_h2_sources_20260728'));ap.add_argument('--firmware',type=Path,default=Path('/home/matteo-manicardi/robotics-reverse/work/ghidra/xgolite_g2_whole_program_20260725T075035Z/input/xgolite_app_v4.3.7.bin'));args=ap.parse_args()
    data=args.firmware.read_bytes();assert sha(args.firmware)=='71032255ffac656c75234c2dcf6a40b307aefc753a92a04c1d0f707d67db6b0e'
    # Export coordinates are the G2 linked/header convention. IROM bytes are +8
    # in canonical load coordinates. Never resolve a literal through a guessed map.
    segments=list(csv.DictReader((args.xgo/'evidence/firmware/segment_table.csv').open()))
    def physical_bytes(address,n):
        for s in segments:
            start=int(s['load_addr']);size=int(s['length'])
            if start<=address and address+n<=start+size:return data[int(s['file_offs'])+address-start:int(s['file_offs'])+address-start+n]
        raise ValueError('unmapped firmware address')
    def literal(export_address):return physical_bytes(export_address+8,4)
    catalogue=[];deep=[];sources=[]
    source=lambda name,path,layer: sources.append({'name':name,'sha256':sha(path),'evidence_layer':layer,'locator':str(path)})
    source('firmware',args.firmware,'FIRMWARE_PRIMARY')
    base=list(csv.DictReader((args.xgo/'evidence/h2/lite_action_catalog.csv').open()))
    base.insert(0,{'action_id':'0','course_pdf_name':'Idle / clear action','cm5_notebook_label':'internal idle','course_pdf_duration_seconds':'','portal_appendix_duration_seconds':''})
    notes={1:'Body height command minimum; no proof of underside floor support',2:'Body height command maximum',3:'Low body height plus forward motion command; dynamic crawl',5:'Analyzed handler immediately calls common completion; documented stepping is not implemented here as a distinct sequence',6:'Repeated height command changes',7:'Periodic roll command, not a proven full rollover',12:'Twelve-slot motor-domain endpoint then common recovery',14:'Three twelve-slot motor-domain keyframes then common recovery',17:'Pitch/height posture plus yaw modulation; support set unspecified',21:'Alternating two motor-domain keyframes then common recovery',24:'Periodic translation/rotation command fields'}
    for item in base:
        i=int(item['action_id']);addr=int('400d'+HANDLERS[i],16);path=args.exports/(f'{addr:x}.txt');text=path.read_text();code=text.split('\n\n\n')[0]
        source('handler_'+str(i),path,'FIRMWARE_PRIMARY_STATIC_EXPORT')
        constants=[];tables=[]
        for a in sorted(set(int(v,16) for v in re.findall(r'(?:PTR_)?DAT_([0-9a-f]{8})',code))):
            raw=literal(a);u=struct.unpack('<I',raw)[0];f=struct.unpack('<f',raw)[0]
            row={'export_literal':hex(a),'canonical_literal':hex(a+8),'file_offset':hex(a-0x400a0000),'u32':u,'bytes_hex':raw.hex()}
            if 0<=f<=10000:row['float32_interpretation']=f
            constants.append(row)
            if 0x3f400100<=u<0x3f400500 and 'FUN_400d7d48' in code:
                b=physical_bytes(u,12);tables.append({'literal':hex(a),'table_address':hex(u),'controller_slots_u8':list(b),'normalized_slots':[v/255 for v in b],'meaning':'12 normalized motor-domain command slots; physical leg/frame/zero binding remains unproved'})
        classification='E' if i in (0,128,129,130,144,255) else ('D' if i==5 else 'C')
        catalogue.append({'action_id':i,'canonical_name':item['course_pdf_name'],'aliases':[item['cm5_notebook_label']], 'documented_duration_s':item['course_pdf_duration_seconds'] or None,'alternate_documented_duration_s':item['portal_appendix_duration_seconds'] or None,'duration_is_hardware_bound':False,'evidence_class':classification,'confidence':'VERIFIED','confidence_scope':'source identity, action dispatch and numeric control writes; physical pose reconstruction remains qualified','static_dynamic':'CONTROL_OR_MANIPULATION' if classification=='E' else ('DYNAMIC_SEQUENCE' if i not in (1,2,12) else 'ENDPOINT_AND_RECOVERY'),'evidence_layer':['VENDOR_DOCUMENTATION','FIRMWARE_PRIMARY_STATIC_EXPORT'],'source':'xgo origin/main a1b34a8 evidence/h2/lite_action_catalog.csv plus fresh action handler export','handler_export_address':hex(addr),'handler_canonical_address':hex(addr+8),'handler_sha256':sha(path),'recovered_body_pose':None,'recovered_leg_targets':None,'recovered_joint_targets':None,'recovered_controller_tables':tables,'contact_support_semantics':'UNKNOWN: no force/contact sensing or declared support set recovered','semantics':notes.get(i,'Numeric action-specific controller state sequence recovered; physical endpoint/support not bound'),'remaining_unknowns':['exact physical joint slot mapping/zero/signs','physical body frame and achieved pose','ground/self collision and contact set','achieved interpolation timing'], 'retarget_eligibility':'NO_DIRECT_JOINT_TRANSFER' if classification!='E' else 'NOT_TRANSFERABLE'})
        deep.append({'action_id':i,'handler':hex(addr),'canonical_address':hex(addr+8),'constants':constants,'tables':tables,'calls':sorted(set(re.findall(r'FUN_([0-9a-f]{8})\(',code))),'entry_bytes':physical_bytes(addr+8,12).hex(),'export_sha256':sha(path)})
    for name in ['lite_action_catalog.csv','lite_course_source_manifest.csv','lite_numeric_contradictions.csv','lite_notebook_code_graph.csv','document_code_reference_graph.csv','app_ascii_to_lower_controller_map.csv','urdf_joint_inventory.csv','source_manifest.csv','rpi5_relevant_files.csv']:
        source(name,args.xgo/'evidence/h2'/name,'REPOSITORY_EVIDENCE')
    # Exhaustive textual triage across canonical exports/docs + relevant source-pinned host corpora.
    inventory=[];pattern=re.compile(r'action|preset|squat|lie.down|stretch|push.up|keyframe|cartesian',re.I)
    for root in [args.xgo,args.cache/'LuwuDynamics_xgo_doglib',args.cache/'HongyiHao-SXIT_DOGZILLALib',Path('/tmp/matdog-g35-vendor')]:
        for p in sorted(root.rglob('*')):
            if not p.is_file() or '.git' in p.parts or p.suffix not in ('.md','.csv','.tsv','.txt','.py','.ipynb','.c','.json'):continue
            if p.stat().st_size>8_000_000:continue
            text=p.read_text(errors='replace');hits=[i+1 for i,l in enumerate(text.splitlines()) if pattern.search(l)]
            if hits:inventory.append({'root':str(root),'path':str(p.relative_to(root)),'sha256':sha(p),'match_lines':hits,'inspection':'AUTOMATED_FULL_TEXT_TRIAGE; manual deep sources listed separately'})
    host=[{'name':'APP_PRESS_UP','evidence_class':'A','scope':'EXACT_HOST_CARTESIAN_COMMAND_SEQUENCE, not lower-firmware preset 21 geometry','source':'CM4 app_dogzilla.py task_press_up_handle','sequence':[{'counter':1,'translation_z':75},{'counter':5,'translation_z':100},{'counter':10,'reset_counter':0}],'loop_delay_requested_s':.15,'repetitions':6,'unknowns':['actual scheduling','body frame height datum','interaction with simultaneous preset 21']}, {'name':'APP_LEG_RESET','evidence_class':'B','scope':'EXACT_HOST_CARTESIAN_ENDPOINT','targets_host_leg_coordinates':[[0,0,108]]*4,'source':'CM4 app_dogzilla.py dogzilla_leg_reset','unknowns':['physical frame mapping to MATDOG']}]
    for name in ('body_translation','body_attitude','periodic_translation','periodic_rotation','single_leg_cartesian','mark_time_register'):
        host.append({'name':name,'evidence_class':'C','scope':'DOCUMENTED_PARAMETERIZED_API; not preset keyframes','source':'pinned Luwu xgolib/xgolib_dog.py changePara and matching methods','unknowns':['exact physical frame binding']})
    save('xgo_catalogue',{'provenance':{'reverse_commit':'a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298','firmware_sha256':sha(args.firmware),'current_yahboom_commit':'8a2cee0be163d7b095868699b360a1badfa2e852','current_preset_pdf_sha256':'a9d1531bd59fb5b55e942f8956d92b6f3ec9cd464f2edcd3f46b37921a6f8a3a','current_pdf_matches_pinned':True},'class_counts':dict(Counter(x['evidence_class'] for x in catalogue)),'actions':catalogue,'additional_host_pose_apis':host})
    save('xgo_static_trace',{'provenance':sources,'coordinate_rule':'G2 linked export IROM address +8 = canonical load address; literal resolution through firmware segment table','action_state_literal':{'export':'0x400d010c','canonical':'0x400d0114','destination':hex(struct.unpack('<I',literal(0x400d010c))[0])},'chain':['host ACTION 0x3e','parser export 0x400d3510 (canonical +8)','action state','dispatcher export 0x400d91fc (canonical +8)','per-action handlers','body command fields or 12-byte motor command tables','mode 1 Cartesian pipeline / mode 3 bounded affine motor target conversion','shared target interpolation/output boundary (never executed)'],'raw_loop_correction':'Ghidra decompiles 400d7d48 as one byte copy; raw loop instruction proves 12 iterations. Do not use decompiler alone.','records':deep})
    save('xgo_search_inventory',{'provenance':sources,'scanned_matching_files':len(inventory),'files':inventory})
    print('XGO catalogue',len(catalogue),Counter(x['evidence_class'] for x in catalogue),'triaged',len(inventory),flush=True)
if __name__=='__main__':main()
