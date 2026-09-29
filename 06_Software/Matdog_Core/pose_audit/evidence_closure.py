"""Close XGO source revisions and command-table semantics without executing vendor code."""
import json,csv,subprocess,urllib.request,urllib.parse,hashlib
from pathlib import Path
from model import sha
from survey import OUT,save
CACHE=Path('/home/matteo-manicardi/robotics-reverse/work/milestone_h2_sources_20260728')
XGO=Path('/tmp/matdog-g35-xgo');EXPORT=Path('/tmp/matdog-g35-ghidra-export')
OLD='aa6b0e414c53c5ec21ddf0e22fcc4ff6e341422e';NEW='8a2cee0be163d7b095868699b360a1badfa2e852'
def main():
 cat=json.loads((OUT/'xgo_catalogue.json').read_text());revisions=[]
 repo=CACHE/'YahboomTechnology_DOGZILLA-Lite'
 paths=subprocess.check_output(['git','-C',str(repo),'ls-tree','-r','--name-only',OLD],text=True).splitlines()
 for path in paths:
  if not path.startswith('3. Motion control') or not path.endswith('.pdf'):continue
  historical=subprocess.check_output(['git','-C',str(repo),'show',OLD+':'+path]);url='https://raw.githubusercontent.com/YahboomTechnology/DOGZILLA-Lite/'+NEW+'/'+urllib.parse.quote(path)
  current=urllib.request.urlopen(url,timeout=45).read();hs=hashlib.sha256(historical).hexdigest();cs=hashlib.sha256(current).hexdigest();same=hs==cs
  revisions.append({'source':path,'historical_revision':OLD,'current_revision':NEW,'historical_sha256':hs,'current_sha256':cs,'current_url':url,'classification':'IDENTICAL' if same else 'UNKNOWN','content_identical':same,**{k:False if same else None for k in ['action_ids_changed','names_changed','durations_changed','numeric_values_changed','implementation_semantics_changed']},'scope':'document bytes; firmware correspondence to later documentation is not established'})
  print(path,same,flush=True)
 revisions.append({'source':'LuwuDynamics/xgo_doglib','historical_revision':'cf72514273dc703284d3c47e46c67ce238caae11','current_revision':'cf72514273dc703284d3c47e46c67ce238caae11','classification':'IDENTICAL','content_identical':True,'scope':'same pinned Git tree; vendor code not executed'})
 tables=[]
 slots=list(csv.DictReader((XGO/'evidence/consolidation/milestone_h1_joint_slot_map.csv').open()))
 for a in cat['actions']:
  for index,t in enumerate(a['recovered_controller_tables']):
   tables.append({'action_id':a['action_id'],'table_index_in_address_order':index,**t,'data_type':'uint8[12]','unit':'dimensionless bounded controller code, 0..255','absolute_or_relative':'absolute endpoint in per-slot normalized target range','pre_or_post_IK':'mode 3 bypasses Cartesian IK, writes same downstream target slot domain','scale':'bounded affine map from [0,255] into stored slot endpoints; not URDF radians','sign':'physical direction/zero UNKNOWN; positive code traverses stored endpoint order','frame':'controller slot coordinates, no Cartesian frame','logical_order':[{'index':i,'logical_leg_slot':i//3,'physical_leg_documentary':['LF','RF','RH','LH'][i//3],'joint_documentary':['lower','upper','hip'][i%3],'physical_binding_confidence':'CORROBORATED, sign/zero/unit UNKNOWN'} for i in range(12)],'consumer_export':'0x400daca4 mode3 -> target base+0x10+2*i','consumer_canonical':'0x400dacac','keyframe_semantics':'ordered action endpoint/control state; address order does not itself establish temporal order','physical_recovery':'PARTIAL_GEOMETRY; exact controller keyframe bytes recovered, physical joint sequence not recovered'})
 schedules={12:[{'counter':0,'table':'0x3f40029c'},{'counter':60,'common_recovery':True},{'counter':70,'complete':True}],14:[{'counter':0,'table':'0x3f4002e4'},{'counter':80,'table':'0x3f4002f0'},{'counter':90,'table':'0x3f4002fc'},{'counter':160,'common_recovery':True},{'counter':170,'complete':True}],21:[{'counter':c,'table':('0x3f400380' if i%2==0 else '0x3f40038c')} for i,c in enumerate([0,40,56,72,88,104,120])]+[{'counter':130,'common_recovery':True},{'counter':140,'complete':True}]}
 manual=[]
 for name in ['400d7d48','400daca4','400da840','400db1f8','400db35c','400d91fc','400d7cdc','400d7c40']+[a['handler_export_address'][2:] for a in cat['actions']]:
  p=EXPORT/(name+'.txt');manual.append({'export_address':name,'canonical_address':hex(int(name,16)+8),'sha256':sha(p),'source':str(p)})
 # Preserve human-reviewable static extracts so /tmp lifetime is not an evidence dependency.
 dest=OUT/'xgo_static_extracts';dest.mkdir(exist_ok=True)
 for record in manual:
  p=EXPORT/(record['export_address']+'.txt');(dest/p.name).write_bytes(p.read_bytes())
 save('xgo_revision_comparison',{'provenance':{'generator_sha256':sha(__file__),'catalogue_sha256':sha(OUT/'xgo_catalogue.json')},'comparisons':revisions,'firmware_document_relation':'UNKNOWN: v4.3.7 image identity pinned independently; current documents are not substituted for its implementation','document_vs_library_conflicts':list(csv.DictReader((XGO/'evidence/h2/lite_numeric_contradictions.csv').open()))})
 save('xgo_table_semantics',{'provenance':{'generator_sha256':sha(__file__),'catalogue_sha256':sha(OUT/'xgo_catalogue.json'),'manual_static_sources':manual,'slot_evidence_sha256':sha(XGO/'evidence/consolidation/milestone_h1_joint_slot_map.csv')},'tables':tables,'selected_schedules':schedules,'timing':'counters are action-task invocations; 26 main-loop iterations with requested delay 2 do not prove wall-clock seconds','consumer_trace':['12-byte copy proven by hardware loop at export 0x400d7d52','mode3 bounded affine conversion at 0x400daca4 into common 16-bit target slots','0x400db1f8 reads target+0x10+2*i and parameter+0xc4 in output call; inspected statically only'],'interpolation_boundary':'For direct motor mode no Cartesian interpolation is proven. Helper 0x400da840 sets common output parameter +0xc4; naming it a recovered physical interpolation duration would overstate evidence. Body mode uses Cartesian pipeline including 0x400da40c before IK.','physical_unknowns':['metric/sign/zero binding of post-IK target slots','loaded per-slot physical range correspondence','achieved temporal interpolation','foot geometry and support contacts for firmware poses']})
 print('tables',len(tables),flush=True)
if __name__=='__main__':main()
