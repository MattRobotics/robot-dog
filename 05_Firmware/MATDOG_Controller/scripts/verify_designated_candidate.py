#!/usr/bin/env python3
"""Offline provenance gate for the one designated app-only candidate.
A later documentation commit can refer to an earlier clean compiled source;
Firmware differences reject. One explicitly pinned host-only serial repair
has a separate receipt; it never relabels the compiled source or image SHA.
"""
import argparse,hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()
HOST_SERIAL_PATCH_FILES=(
 '05_Firmware/MATDOG_Controller/scripts/release/motion_remote_preflight.py',
 '05_Firmware/MATDOG_Controller/scripts/tests/test_remote_preflight.py',
 '05_Firmware/MATDOG_Controller/scripts/verify_designated_candidate.py',
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
def verify(path):
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
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('designation',type=Path);p.add_argument('--field',choices=('source','directory'),required=True);a=p.parse_args();source,root=verify(a.designation);print(source if a.field=='source' else root)
