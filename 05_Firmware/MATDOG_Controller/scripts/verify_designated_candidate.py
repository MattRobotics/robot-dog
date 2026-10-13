#!/usr/bin/env python3
"""Offline provenance gate for the one designated app-only candidate.
A later documentation commit can refer to an earlier clean compiled source;
any firmware/tool/test difference rejects. This never relabels the source SHA.
"""
import argparse,hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()
def verify(path):
 if git('status','--porcelain'):raise ValueError('current worktree must be clean')
 d=json.loads(path.read_text());source=d['source_commit'];head=git('rev-parse','HEAD')
 subprocess.run(['git','-C',str(ROOT),'merge-base','--is-ancestor',source,head],check=True)
 if git('diff','--name-only',source,head,'--','05_Firmware/MATDOG_Controller','06_Software/Matdog_Core/motion/paired_prone'):raise ValueError('compiled source differs from current firmware/tools')
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
