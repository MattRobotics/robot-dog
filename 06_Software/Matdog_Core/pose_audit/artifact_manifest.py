"""Index G3.5 artifacts and cross-check recorded provenance against the files on disk.

Standard library only. `--check` fails on a stale index, a changed artifact, a recorded input hash that no
longer matches, or canonical sources that changed since an artifact was written. Historical generators
that were later edited are reported (HISTORICAL_GENERATOR_REVISION), not failed: saved deterministic
searches are preserved and separately revalidated.
"""
import argparse,hashlib,json,sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OUT=ROOT/'09_Logs/Validation_Reports/G35_Pose_Audit'
MANIFEST=OUT/'artifact_manifest.json'
PRODUCER={'rest_search':'survey.py','envelope':'survey.py','transitions':'survey.py','contact_modes':'contact_modes.py','expanded_envelope':'expanded_survey.py','transfer_search':'transfer_search.py','support_transfer':'support_transfer.py','route_extension':'route_extension.py','equivalent_stand':'equivalent_stand.py','candidate_diagnostics':'diagnostics.py','equivalent_stand_diagnostics':'diagnostics.py','revalidation':'revalidate.py','equivalent_stand_revalidation':'revalidate.py','collision_policy':'policy_report.py','pose_library':'pose_library.py','retarget_matrix':'retarget.py','transition_graph':'transition_summary.py','lower_envelope':'lower_envelope.py','rest_connectivity':'rest_connectivity.py','connectivity':'connectivity.py','closeout_classification':'closeout.py','visual_manifest':'render.py','dimensions':'dimensions.py','xgo_catalogue':'xgo_audit.py','xgo_search_inventory':'xgo_audit.py','xgo_static_trace':'xgo_audit.py','xgo_table_semantics':'evidence_closure.py','xgo_revision_comparison':'evidence_closure.py','validation_results':'validate.py','REPORT':'build_report.py'}
INPUT_FIELDS={'source_sha256':None,'catalogue_sha256':'xgo_catalogue.json','library_sha256':'pose_library.json'}
TOOL_FIELDS={'model_sha256':'model.py','route_extension_sha256':'route_extension.py'}

def sha(path):
 return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def files():
 return sorted(p for p in OUT.rglob('*') if p.is_file() and p!=MANIFEST)

def provenance_of(path):
 if path.suffix!='.json':return None
 data=json.loads(path.read_text())
 prov=data.get('provenance') if isinstance(data,dict) else None
 return prov if isinstance(prov,dict) else None

def source_pointer(path,provenance):
 if path.stem=='equivalent_stand':return 'route_extension.json'
 if path.stem=='lower_envelope':return 'contact_modes.json'
 return None

def build():
 artifacts=[];inputs=[];generators=[];sources=[]
 canonical={}
 for p in files():
  rel=str(p.relative_to(OUT));artifacts.append({'path':rel,'sha256':sha(p),'bytes':p.stat().st_size,'producer':PRODUCER.get(p.stem) if p.parent==OUT else ('render.py' if p.parent.name=='views' else 'xgo_audit.py')})
  prov=provenance_of(p)
  if not prov:continue
  block=prov.get('inputs') or {}
  for name,recorded in block.items():
   target=OUT/name if name.endswith('.json') else None
   if target and target.exists():inputs.append({'artifact':rel,'input':name,'recorded_sha256':recorded,'current_sha256':sha(target),'status':'MATCH' if recorded==sha(target) else 'STALE_INPUT'})
  for field,target_name in INPUT_FIELDS.items():
   if field in prov:
    target_name=target_name or source_pointer(p,prov)
    if target_name:inputs.append({'artifact':rel,'input':target_name,'recorded_sha256':prov[field],'current_sha256':sha(OUT/target_name),'status':'MATCH' if prov[field]==sha(OUT/target_name) else 'STALE_INPUT'})
  if 'diagnostics_sha256' in prov:
   if p.stem=='transition_graph':inputs.append({'artifact':rel,'input':'candidate_diagnostics.json','recorded_sha256':prov['diagnostics_sha256'],'current_sha256':sha(OUT/'candidate_diagnostics.json'),'status':'MATCH' if prov['diagnostics_sha256']==sha(OUT/'candidate_diagnostics.json') else 'STALE_INPUT'})
   else:generators.append({'artifact':rel,'generator':'diagnostics.py','recorded_sha256':prov['diagnostics_sha256'],'current_sha256':sha(HERE/'diagnostics.py'),'status':'CURRENT' if prov['diagnostics_sha256']==sha(HERE/'diagnostics.py') else 'HISTORICAL_GENERATOR_REVISION'})
  for field,script in TOOL_FIELDS.items():
   if field in prov:generators.append({'artifact':rel,'generator':script,'recorded_sha256':prov[field],'current_sha256':sha(HERE/script),'status':'CURRENT' if prov[field]==sha(HERE/script) else 'HISTORICAL_GENERATOR_REVISION'})
  recorded=prov.get('generator_sha256');producer=PRODUCER.get(p.stem)
  if recorded and producer:
   current=sha(HERE/producer);generators.append({'artifact':rel,'generator':producer,'recorded_sha256':recorded,'current_sha256':current,'status':'CURRENT' if recorded==current else 'HISTORICAL_GENERATOR_REVISION'})
  for key in ('canonical_sources','sources','matdog_sources'):
   if isinstance(prov.get(key),dict):
    for name,recorded in prov[key].items():
     if '/' not in name and name.endswith('.json'):
      if (OUT/name).exists():inputs.append({'artifact':rel,'input':name,'recorded_sha256':recorded,'current_sha256':sha(OUT/name),'status':'MATCH' if recorded==sha(OUT/name) else 'STALE_INPUT'})
      continue
     canonical.setdefault(name,set()).add(recorded)
 for name,seen in sorted(canonical.items()):
  target=ROOT/name
  current=sha(target) if target.exists() else None
  sources.append({'path':name,'current_sha256':current,'recorded_variants':sorted(seen),'status':'MATCH' if current is not None and seen=={current} else 'CHANGED_OR_MISSING'})
 return {'scope':'G3.5 offline pose audit; no hardware artifact, serial log, or actuator command is indexed','excluded':['artifact_manifest.json (self)'],'artifact_count':len(artifacts),'artifacts':artifacts,'provenance_input_checks':inputs,'generator_checks':generators,'canonical_source_checks':sources,
  'summary':{'stale_inputs':[i for i in inputs if i['status']!='MATCH'],'historical_generator_revisions':[g['artifact'] for g in generators if g['status']!='CURRENT'],'changed_canonical_sources':[s['path'] for s in sources if s['status']!='MATCH']}}

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true');args=parser.parse_args();manifest=build()
 bad=manifest['summary']['stale_inputs'] or manifest['summary']['changed_canonical_sources']
 if args.check:
  if not MANIFEST.exists():print('ARTIFACT_MANIFEST missing');return 1
  saved=json.loads(MANIFEST.read_text())
  if saved!=manifest:
   print('ARTIFACT_MANIFEST stale or artifact changed');return 1
  if bad:print('ARTIFACT_MANIFEST recorded provenance does not match files',manifest['summary']);return 1
  print('ARTIFACT_MANIFEST OK',manifest['artifact_count'],'artifacts;','historical generator revisions:',len(manifest['summary']['historical_generator_revisions']));return 0
 MANIFEST.write_text(json.dumps(manifest,sort_keys=True,indent=2,allow_nan=False)+'\n')
 print('ARTIFACT_MANIFEST wrote',manifest['artifact_count'],'artifacts; stale inputs',len(manifest['summary']['stale_inputs']),'changed sources',len(manifest['summary']['changed_canonical_sources']),'historical generator revisions',len(manifest['summary']['historical_generator_revisions']))
 return 1 if bad else 0
if __name__=='__main__':sys.exit(main())
