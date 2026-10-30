#!/usr/bin/env python3
"""Offline provenance gate for the one designated app-only candidate.
A later documentation commit can refer to an earlier clean compiled source;
Firmware differences reject. Explicitly pinned host-only serial/ROM repairs
has a separate receipt; it never relabels the compiled source or image SHA.
"""
import argparse,hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()
HOST_SERIAL_PATCH_FILES=(
 '05_Firmware/MATDOG_Controller/scripts/release/motion_remote_preflight.py',
 '05_Firmware/MATDOG_Controller/scripts/release/review_preflash.py',
 '05_Firmware/MATDOG_Controller/scripts/tests/test_remote_preflight.py',
 '05_Firmware/MATDOG_Controller/scripts/verify_designated_candidate.py',
 '05_Firmware/MATDOG_Controller/scripts/flash_app_only.sh',
 '05_Firmware/MATDOG_Controller/scripts/verify_application_partition.py',
 '05_Firmware/MATDOG_Controller/scripts/tests/test_matdog_layout.py',
)
def verify_host_serial_patch(root,source,image_sha,changed):
 """Only the named host repair, with old/new exact byte identities, can pass."""
 if set(changed)!=set(HOST_SERIAL_PATCH_FILES):raise ValueError('compiled source differs outside the pinned host serial repair')
 receipt=root/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13/PREFLIGHT_SERIAL_FIX.json'
 r=json.loads(receipt.read_text())
 if r.get('purpose')!='PREFLIGHT_SERIAL_RESPONSE_FRAMING_FIX' or r.get('status')!='PASS' or r.get('compiled_source_commit')!=source or r.get('application_sha256')!=image_sha or set(r.get('files',{}))!=set(HOST_SERIAL_PATCH_FILES):raise ValueError('host serial repair receipt mismatch')
 for name in HOST_SERIAL_PATCH_FILES:
  original=subprocess.check_output(['git','-C',str(root),'show',source+':'+name])
  pair=r['files'][name]
  if pair.get('compiled_source_file_sha256')!=hashlib.sha256(original).hexdigest() or pair.get('current_file_sha256')!=hashlib.sha256((root/name).read_bytes()).hexdigest():raise ValueError('host serial repair byte identity mismatch: '+name)
def verify_final_receipt(path):
 """Bind the existing final DIRTY-build receipt to its exact compiled inputs.
 This is source provenance, not a runtime permission or a static-audit PASS.
 """
 import build_manifest as bm
 import matdog_layout as layout
 d=json.loads(path.read_text())
 if d.get('classification')=='TEMPORARY MEASUREMENT ONLY; NOT OPERATIONAL':
  return verify_timing_receipt(path)
 image=Path(d['app']).resolve();directory=image.parent
 def need(ok,why):
  if not ok:raise ValueError(why)
 digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 preenable=d.get('classification')=='VERIFIED OFFLINE (build/tests); IMPLEMENTED NOT VALIDATED (hardware)'
 need(preenable or d.get('classification')=='VERIFIED OFFLINE','final receipt classification mismatch')
 need(image.is_file() and digest(image)==d['sha256'] and image.stat().st_size==d['size'],'final image mismatch')
 manifest_text=(directory/'matdog_build_manifest.txt').read_text()
 need(manifest_text==d['manifest' if preenable else 'build_manifest'],'final manifest differs from reviewed receipt')
 if preenable:
  # This authentic build records an unchanged-inventory assertion, not a
  # second JSON snapshot. Preserve that distinction; verify its recorded bytes.
  evidence={name.removeprefix('./'):sha for sha,name in
            (line.split(maxsplit=1) for line in (path.parent/'SHA256SUMS.txt').read_text().splitlines())}
  for name in (path.name,'head.txt','source-hashes-before-build.json','source-hashes-after-build.check'):
   need(digest(path.parent/name)==evidence.get(name),'receipt evidence hash mismatch: '+name)
  before=json.loads((path.parent/'source-hashes-before-build.json').read_text())
  need(bool(before) and (path.parent/'source-hashes-after-build.check').read_text().strip()=='SOURCES_CHANGED_DURING_BUILD []',
       'compiled inventory changed during build')
  source=(path.parent/'head.txt').read_text().strip()
  need(d['source_base'].startswith(source+' + '),'receipt source base differs')
 else:
  before=json.loads((path.parent/'build-source-hashes.json').read_text())
  after=json.loads((path.parent/'source-hashes-after-build.json').read_text())
  need(before==after and bool(before),'compiled inventory changed during build')
  source=d['git_head']
 fw=ROOT/'05_Firmware/MATDOG_Controller'
 actual={p.relative_to(ROOT).as_posix() for p in (fw/'src').rglob('*') if p.is_file() and p.suffix not in ('.md','.example')}
 required=actual|{p.relative_to(ROOT).as_posix() for p in (fw/'MATDOG_Controller.ino',fw/'partitions.csv')}
 need({n for n in before if n.startswith('05_Firmware/MATDOG_Controller/src/')}==actual and required<=set(before),'compiled file set differs')
 for name in required:
  need(digest(ROOT/name)==before[name],'compiled source changed: '+name)
 manifest=bm.parse_manifest(manifest_text)
 need(git('rev-parse','HEAD')==source,'baseline HEAD differs from final receipt')
 data=image.read_bytes()
 table_hash,_=layout.check_table_bytes((directory/'MATDOG_Controller.ino.partitions.bin').read_bytes())
 verdict=bm.verify_manifest(manifest,head_commit=source,expected_fqbn=bm_value_fqbn(),
  tree_state='DIRTY' if git('status','--porcelain') else 'CLEAN',binary_exists=True,
  binary_size=len(data),binary_sha256=digest(image),requested_profile='ROBOT_POWERED',
  requested_ota_ingest='0',build_partition_table_sha256=table_hash,
  binary_embeds_layout_id=layout.binary_embeds_layout_id(data),source_inventory_verified=True)
 need(verdict.ok,'final manifest refused: '+str(verdict))
 return source,directory

def timing_manifest(path):
 """Translate the pinned diagnostic receipt schema; never change build artifacts."""
 import build_manifest as bm
 d=json.loads(path.read_text())
 base=json.loads((path.parent.parent/'FINAL_RECEIPT.json').read_text())
 m=bm.parse_manifest(base['build_manifest'])
 m.update(BUILD_ID='TIMING_ONLY_NOT_OPERATIONAL', APPLICATION_SIZE=str(d['size']),
          APPLICATION_SHA256=d['sha256'], APP_SHA256=d['sha256'], BUILD_UTC='2026-10-25T16:38:42Z')
 return m

def verify_timing_receipt(path):
 """Exact authorized diagnostic overlay on the already verified corrective APP."""
 import difflib
 import build_manifest as bm
 import matdog_layout as layout
 sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 def need(ok,why):
  if not ok:raise ValueError(why)
 need(sha(path)=='de093dcd793bad8c48f079c689d83c10f77ce32c721bcf5435af0a094d6abee2','diagnostic receipt identity mismatch')
 d=json.loads(path.read_text());image=Path(d['app']).resolve();directory=image.parent
 need(d['sha256']=='d15d4711598b638992dc4472d79fe723c1770d28adbd832450f005e3b52988b5' and
      sha(image)==d['sha256'] and image.stat().st_size==d['size']==1614656 and d['exit']==0,'diagnostic image mismatch')
 base_path=path.parent.parent/'FINAL_RECEIPT.json'
 source,base_dir=verify_final_receipt(base_path)
 need(d['operational_candidate_unmodified']==json.loads(base_path.read_text())['sha256']==
      '520c1e87c8cb2349e2a9868c12c4973a36de57b9ab37d693da6b6abf0b999eb6','corrective base mismatch')
 sketch=path.parent/'MATDOG_Controller';inventory=d['source_sha256']
 actual={p.relative_to(sketch).as_posix() for p in sketch.rglob('*') if p.is_file() and
         (p.suffix in ('.cpp','.h','.ino') or p.name=='partitions.csv')}
 need(actual==set(inventory),'diagnostic compiled file set differs')
 for name,h in inventory.items():need(sha(sketch/name)==h,'diagnostic source mismatch: '+name)
 patch=path.parent/'diagnostic-only.patch'
 need(sha(patch)=='44de4832ef6e9d0e76a0ec768e720ca5bf40f7903978bbcb19fbc57e44c1969b','reviewed diagnostic patch mismatch')
 order=[line[6:] for line in patch.read_text().splitlines() if line.startswith('--- a/')]
 baseline=ROOT/'05_Firmware/MATDOG_Controller'
 changed={n for n in actual if n.startswith('src/') and
          (not (baseline/n).exists() or (baseline/n).read_bytes()!=(sketch/n).read_bytes())}
 need(changed==set(order),'diagnostic changes outside reviewed patch')
 diff=''
 for n in order:
  p=baseline/n
  diff+=''.join(difflib.unified_diff(p.read_text().splitlines(True) if p.exists() else [],
       (sketch/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
 need(diff==patch.read_text(),'diagnostic patch differs from corrective base')
 for n in ('MATDOG_Controller.ino','partitions.csv'):need(sha(sketch/n)==sha(baseline/n),'non-diagnostic sketch/layout change')
 compile_dir=directory.parent/'compile'
 opts=json.loads((compile_dir/'build.options.json').read_text());cmd=d['command']
 need(opts['fqbn']==bm_value_fqbn()==cmd[cmd.index('--fqbn')+1] and opts['sketchLocation']==str(sketch), 'diagnostic compile options mismatch')
 flags=cmd[cmd.index('compiler.cpp.extra_flags='+opts['customBuildProperties'].split('compiler.cpp.extra_flags=',1)[1])]
 need('MATDOG_BUILD_ID=\"TIMING_ONLY_NOT_OPERATIONAL\"' in flags and 'HardwareProfile::ROBOT_POWERED' in flags,'diagnostic profile mismatch')
 for n in ('sdkconfig','MATDOG_Controller.ino.partitions.bin','MATDOG_Controller.ino.bootloader.bin'):
  p=compile_dir/n if n=='sdkconfig' else directory/n
  need(sha(p)==sha(base_dir/n),'diagnostic layout/config differs: '+n)
 m=timing_manifest(path);table_hash,_=layout.check_table_bytes((directory/'MATDOG_Controller.ino.partitions.bin').read_bytes())
 verdict=bm.verify_manifest(m,head_commit=source,expected_fqbn=bm_value_fqbn(),tree_state='DIRTY',
  binary_exists=True,binary_size=image.stat().st_size,binary_sha256=sha(image),requested_profile='ROBOT_POWERED',
  requested_ota_ingest='0',build_partition_table_sha256=table_hash,
  binary_embeds_layout_id=layout.binary_embeds_layout_id(image.read_bytes()),source_inventory_verified=True)
 need(verdict.ok,'diagnostic metadata refused: '+str(verdict))
 return source,directory

def bm_value_fqbn():
 # Same board contract as flash_app_only.sh; never infer it from the receipt.
 return 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi'

def verify(path):
 if 'app' in json.loads(path.read_text()):return verify_final_receipt(path)
 if json.loads(path.read_text()).get('schema')==3:
  from release_candidate_gate import verify as verify_v3
  return verify_v3(path,ROOT)
 if git('status','--porcelain'):raise ValueError('current worktree must be clean')
 d=json.loads(path.read_text());source=d['source_commit'];head=git('rev-parse','HEAD')
 subprocess.run(['git','-C',str(ROOT),'merge-base','--is-ancestor',source,head],check=True)
 changed=git('diff','--name-only',source,head,'--','05_Firmware/MATDOG_Controller','06_Software/Matdog_Core/motion/paired_prone').splitlines()
 if changed:verify_host_serial_patch(ROOT,source,d['application_sha256'],changed)
 root=Path(d['artifact_directory']);image=root/'MATDOG_Controller.ino.bin';manifest={}
 for line in (root/'matdog_build_manifest.txt').read_text().splitlines():
  if '=' in line:k,v=line.split('=',1);manifest[k]=v
 digest=hashlib.sha256(image.read_bytes()).hexdigest()
 if manifest['SOURCE_COMMIT']!=source or manifest['SOURCE_STATE']!='CLEAN' or manifest['APPLICATION_SHA256']!=digest or digest!=d['application_sha256']:raise ValueError('candidate source/image receipt mismatch')
 if manifest.get('SUPERVISED_VALIDATION_CAPABLE')!='1' or manifest.get('BOOT_MOTION_AUTHORIZED')!='0' or manifest['OTA_INGEST_ENABLED']!='0' or manifest['HARDWARE_PROFILE']!='ROBOT_POWERED':raise ValueError('candidate capability/profile mismatch')
 if d.get('release_ready')!='PASS':raise ValueError('designated release review is not PASS')
 return source,root
def verify_frozen_qualification(root,source):
 """Reuse completed suites only with exact compiled inventory and log identities.
 No test process is launched. The separately pinned host repairs must carry
 their own successful regression and APP-ONLY audit receipts.
 """
 report=root/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13'
 def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
 build=json.loads((report/'BUILD_RECEIPT.json').read_text())
 tests_path=report/'TEST_RECEIPT.json';tests=json.loads(tests_path.read_text())
 if build.get('status')!='PASS' or build.get('source_commit')!=source or build.get('test_receipt_sha256')!=sha(tests_path) or tests.get('status')!='PASS':raise ValueError('frozen qualification identity/status mismatch')
 inventory=tests.get('source_files',{})
 if not inventory:raise ValueError('frozen qualification has no source inventory')
 for name,expected in inventory.items():
  path=root/name
  if not path.resolve().is_relative_to(root.resolve()):raise ValueError('qualification inventory path escapes repository')
  original=subprocess.check_output(['git','-C',str(root),'show',source+':'+name])
  if hashlib.sha256(original).hexdigest()!=expected:raise ValueError('qualification differs from compiled source: '+name)
  if name not in HOST_SERIAL_PATCH_FILES and sha(path)!=expected:raise ValueError('qualified source changed: '+name)
 def logs(entries):
  if not entries:raise ValueError('qualification logs absent')
  for entry in entries:
   if entry.get('status',entry.get('result'))!='PASS' or entry.get('exit_code',0)!=0:raise ValueError('qualification test not PASS')
   if 'log' not in entry:
    name=entry.get('name','').removeprefix('geometry_')
    expected=build.get('certificates_sha256',{}).get(name)
    if not expected or sha(report/name)!=expected:raise ValueError('qualified certificate identity mismatch')
    continue
   path=report/entry['log']
   if not path.resolve().is_relative_to(report.resolve()) or sha(path)!=entry['log_sha256']:raise ValueError('qualification log identity mismatch')
 logs(tests.get('tests',[]))
 full=next((x for x in tests['tests'] if x['name']=='full_static_and_host_led_network_nvs_startup'),None)
 if full is None or 'STATIC_AUDIT = PASS' not in (report/full['log']).read_text():raise ValueError('completed static audit PASS absent')
 repair=json.loads((report/'PREFLIGHT_SERIAL_FIX.json').read_text())
 host=repair['tests'];audit=host['targeted_app_only_audit']
 if host.get('result')!='PASS' or audit.get('result')!='PASS' or audit.get('checks')!=4:raise ValueError('host repair qualification incomplete')
 logs([dict(status=host['result'],log=host['log'],log_sha256=host['log_sha256']),dict(status=audit['result'],log=audit['log'],log_sha256=audit['log_sha256'])])
 reuse=host.get('frozen_qualification_gate',{})
 if reuse.get('result')!='PASS':raise ValueError('frozen qualification gate regression incomplete')
 logs([dict(status=reuse['result'],log=reuse['log'],log_sha256=reuse['log_sha256'])])
 return 'STATIC_AUDIT = PASS (frozen source/log identities; separately qualified host repairs; no repeated firmware suites)'
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('designation',type=Path);p.add_argument('--field',choices=('source','directory','qualification','sdkconfig'),required=True);a=p.parse_args();source,root=verify(a.designation)
 if a.field=='sdkconfig':
  d=json.loads(a.designation.read_text());print(root.parent/'compile/sdkconfig' if d.get('classification')=='TEMPORARY MEASUREMENT ONLY; NOT OPERATIONAL' else root/'sdkconfig');raise SystemExit(0)
 if 'app' in json.loads(a.designation.read_text()):
  print('FINAL_RECEIPT = VERIFIED OFFLINE (compiled source/image identity; historical static findings are not relabelled PASS)' if a.field=='qualification' else source if a.field=='source' else root)
  raise SystemExit(0)
 modern=json.loads(a.designation.read_text()).get('schema')==3
 print(('STATIC_AUDIT_QUALIFICATION = PASS (V3 inventories; FULL or strictly resolved COMPOSED campaign, see designation)' if modern else verify_frozen_qualification(ROOT,source)) if a.field=='qualification' else source if a.field=='source' else root)
