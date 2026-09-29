"""Run every offline G3.5 validation gate once and record the results in validation_results.json.

Software-only: host compilers and Python only. No serial port, no network transport, no hardware.
Run from anywhere with the pose-audit virtual environment:  <venv>/bin/python validate.py
The manifest check and the final rerun of the pose-audit tests happen after this script and after the
report is rebuilt, because the manifest indexes validation_results.json and REPORT.md.
"""
import hashlib,json,os,re,shutil,subprocess,sys,tempfile,time
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OUT=ROOT/'09_Logs/Validation_Reports/G35_Pose_Audit'
FW=ROOT/'05_Firmware/MATDOG_Controller'
TESTS=FW/'scripts/tests'
PY=sys.executable
CXX=os.environ.get('CXX','g++')
MOTION_SOURCES=('PoseSupport','LegKinematics','LegInverseKinematics','FootContact','StandTrajectory','BodyPose','StartupAcquisition')
SUMMARY=re.compile(r'(POSE_DATA_FRESHNESS|POSE_SUPPORT|REVALIDATION|Ran \d+ tests|^OK|STATIC_AUDIT|static audit|PASS|passed)',re.I)

def sha_bytes(data):
 return hashlib.sha256(data).hexdigest()

REPORT_TOOLING=('build_report.py','validate.py')

def source_digest():
 audited=[p for p in HERE.glob('*.py') if p.name not in REPORT_TOOLING]
 paths=sorted(audited+list((FW/'src/motion').glob('*'))+[p for p in TESTS.iterdir() if p.is_file()]+[FW/'scripts/static_audit.py'])
 lines=[f'{p.relative_to(ROOT)} {sha_bytes(p.read_bytes())}' for p in paths]
 return sha_bytes('\n'.join(lines).encode()),len(lines)

def run(name,cmd,cwd=ROOT,env=None,timeout=3600,extra=None):
 start=time.time();full={**os.environ,**(env or {})}
 proc=subprocess.run(cmd,cwd=cwd,env=full,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout)
 lines=[l for l in proc.stdout.splitlines() if l.strip()]
 matches=[l for l in lines if SUMMARY.search(l)];ran=[l for l in lines if re.match(r'Ran \d+ tests?',l)]
 summary=(ran[-1]+'; '+lines[-1]) if ran else (matches or lines or [''])[-1].strip()
 gate={'name':name,'command':' '.join(str(c) for c in cmd),'returncode':proc.returncode,'passed':proc.returncode==0,'seconds':round(time.time()-start,1),'summary':summary,'output_sha256':sha_bytes(proc.stdout.encode()),'output_line_count':len(lines),'output_tail':lines[-12:]}
 if extra:gate.update(extra)
 print(f"{'PASS' if gate['passed'] else 'FAIL'} {name} ({gate['seconds']}s): {gate['summary']}",flush=True)
 return gate

def strip_provenance(data):
 return {k:v for k,v in data.items() if k!='provenance'}

def revalidation_gate(name,source_args,saved_name,check_name):
 saved=OUT/(saved_name+'.json');backup=saved.read_bytes();check=OUT/(check_name+'.json')
 gate=run(name,[PY,str(HERE/'revalidate.py'),*source_args,'--output',check_name],cwd=HERE)
 identical=False
 if gate['passed'] and check.exists():
  old=json.loads(backup);new=json.loads(check.read_text());identical=strip_provenance(old)==strip_provenance(new)
  if identical:check.replace(saved)
  else:check.unlink()
 elif check.exists():check.unlink()
 gate['classification_and_sample_data_identical_to_saved']=identical
 gate['saved_artifact']=saved_name+'.json'
 gate['saved_artifact_provenance_refreshed_to_current_generator']=identical and saved.read_bytes()!=backup
 gate['passed']=gate['passed'] and identical
 return gate

def sanitizer_gate():
 tmp=tempfile.mkdtemp(prefix='matdog-g35-asan-')
 try:
  binary=str(Path(tmp)/'test_pose_support_asan')
  build=[CXX,'-std=c++17','-O1','-g','-fno-exceptions','-fno-rtti','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-no-pie','-o',binary,str(TESTS/'test_pose_support.cpp')]+[str(FW/'src/motion'/(n+'.cpp')) for n in MOTION_SOURCES]
  built=run('sanitizer_build_asan_ubsan',build)
  if not built['passed']:return [built]
  ran=run('sanitizer_run_asan_ubsan',[binary],env={'ASAN_OPTIONS':'detect_leaks=1:halt_on_error=1','UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'})
  return [built,ran]
 finally:shutil.rmtree(tmp,ignore_errors=True)

def main():
 digest,count=source_digest();gates=[]
 gates.append(revalidation_gate('revalidate_preserved_corpus',[],'revalidation','revalidation_check'))
 gates.append(revalidation_gate('revalidate_equivalent_stand',['--sources','equivalent_stand'],'equivalent_stand_revalidation','equivalent_stand_revalidation_check'))
 gates.append(run('pose_export_freshness',[PY,str(HERE/'pose_export.py'),'--check'],cwd=HERE))
 gates.append(run('pose_audit_tests',[PY,str(HERE/'test_pose_audit.py')],cwd=HERE))
 gates.append(run('host_motion_runner_g1_g2_g3_pose',['bash',str(TESTS/'run_motion_host_tests.sh')]))
 gates.append(run('full_host_suite',['bash',str(TESTS/'run_host_tests.sh')]))
 gates.append(run('static_audit',['python3',str(FW/'scripts/static_audit.py')]))
 gates.extend(sanitizer_gate())
 gates.append(run('git_diff_check',['git','diff','--check','HEAD']))
 result={'provenance':{'generator_sha256':sha_bytes(Path(__file__).read_bytes())},'scope':'offline software gates only: host compiler, Python, git; no hardware, serial device, ServoBus, servo command, Torque, flashing or EEPROM access','source_digest':digest,'source_digest_file_count':count,'gates':gates,'all_passed':all(g['passed'] for g in gates)}
 (OUT/'validation_results.json').write_text(json.dumps(result,sort_keys=True,indent=2,allow_nan=False)+'\n')
 print('VALIDATION all_passed',result['all_passed'],'gates',len(gates),flush=True)
 return 0 if result['all_passed'] else 1

if __name__=='__main__':sys.exit(main())
