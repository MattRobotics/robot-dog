#!/usr/bin/env python3
"""One offline integration campaign. No robot or device access.
Large geometry certificates are verified by identity/status, not regenerated.
"""
import argparse,concurrent.futures,datetime,hashlib,json,os,subprocess,sys
from pathlib import Path
FW=Path(__file__).resolve().parents[2];ROOT=FW.parents[1];TESTS=FW/'scripts/tests'
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--static-audit-log',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1'}
 tasks=[('restore',['bash',str(TESTS/'run_calibration_restore_tests.sh')]),('service_executor',['bash',str(TESTS/'run_motion_service_executor_tests.sh')]),('paired_route_session',[sys.executable,str(TESTS/'run_paired_runtime_tests.py')]),('supervised_planner',[sys.executable,str(TESTS/'run_supervised_runtime_tests.py')]),('quantized_fixture_pipeline',[sys.executable,str(TESTS/'run_supervised_runtime_tests.py')]),('profile',['bash',str(TESTS/'run_validation_profile_tests.sh')]),('review_cards',[sys.executable,str(TESTS/'test_profile_cards_integration.py')]),('stl_adapter',[sys.executable,str(TESTS/'test_stl_gait_adapter.py')]),('smooth_derivatives',[sys.executable,str(TESTS/'test_suspended_stl_gait.py')]),('lifecycle',['bash',str(TESTS/'run_motion_lifecycle_tests.sh')]),('adaptive',[sys.executable,str(TESTS/'run_adaptive_motion_tests.py'),'--trace',str(a.output/'adaptive.jsonl')]),('ap',[sys.executable,str(TESTS/'test_ap_only_transport.py')]),('http',[sys.executable,str(TESTS/'test_motion_http_transport.py')]),('radio_mode',[sys.executable,str(TESTS/'test_motion_transport_mode.py')]),('safety',[sys.executable,str(TESTS/'test_motion_safety_boundaries.py')]),('receipt_mutations',[sys.executable,str(TESTS/'test_motion_rc_receipt.py')]),('remote_preflight',[sys.executable,str(TESTS/'test_remote_preflight.py')]),('drive_readback',[sys.executable,str(TESTS/'test_validation_drive_readback.py')]),('portal_dom',['node',str(TESTS/'test_portal_dom.cjs')]),('motion_dom',['node',str(TESTS/'test_motion_dashboard_dom.cjs')])]
 tasks += [('fixture_commissioning',[sys.executable,str(TESTS/'run_fixture_commissioning_tests.py')]),
           ('build_identity',[sys.executable,str(TESTS/'test_dev_identity_build.py')])]
 def run(task):
  name,cmd=task;e=env.copy()
  if name=='quantized_fixture_pipeline':e['MATDOG_SUPERVISED_TEST_UNIT']='test_fixture_pipeline.cpp'
  log=a.output/(name+'.log')
  with log.open('w') as f:r=subprocess.run(cmd,cwd=ROOT,env=e,stdout=f,stderr=subprocess.STDOUT)
  print(name, 'PASS' if r.returncode==0 else 'FAIL',flush=True)
  return dict(name=name,status='PASS' if r.returncode==0 else 'FAIL',exit_code=r.returncode,log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest())
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,tasks))
 static=a.static_audit_log;results.append(dict(name='full_static_and_host_led_network_nvs_startup',status='PASS' if static.is_file() and 'STATIC_AUDIT = PASS' in static.read_text() else 'FAIL',log=str(static),log_sha256=hashlib.sha256(static.read_bytes()).hexdigest() if static.exists() else None))
 report=ROOT/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13'
 certs=[]
 for name,status in [('fixture_final_margin_route_6mrad.json','PASS_FIXTURE_ROUTE_TUBE'),('stand_tracking_tube_6mrad.json','PASS_GEOMETRY_TUBE'),('first_stride_30mm_qualified_clock_swept_6mrad.json','PASS_CONTINUOUS_TARGET_BOXES')]:
  path=report/name;d=json.loads(path.read_text());certs.append(dict(file=name,status=d['status'],identity_sha256=hashlib.sha256(path.read_bytes()).hexdigest()));results.append(dict(name='geometry_'+name,status='PASS' if d['status']==status else 'FAIL'))
 inventory={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for base in [FW/'src',FW/'scripts',ROOT/'06_Software/Matdog_Core/motion/paired_prone'] for f in base.rglob('*') if f.is_file() and f.suffix in ('.cpp','.h','.py','.sh','.cjs','.json') and f.name!='TlsIdentity.local.h' and '__pycache__' not in str(f)}
 receipt=dict(schema='MATDOG_FINAL_OFFLINE_CAMPAIGN_V1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),source_inventory_sha256=hashlib.sha256(json.dumps(inventory,sort_keys=True).encode()).hexdigest(),source_files=inventory,status='PASS' if all(r['status']=='PASS' for r in results) else 'FAIL',tests=results,certificates=certs,hardware_access=False,physical_motion_authorized=False,flash_authorized=False)
 (a.output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print('FINAL_OFFLINE_CAMPAIGN',receipt['status'],flush=True);return 0 if receipt['status']=='PASS' else 1
if __name__=='__main__':sys.exit(main())
