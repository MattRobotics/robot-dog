#!/usr/bin/env python3
"""Exercise the REAL flasher with a file-only esptool double, never a tty.

Requires a committed V3 designation. The fake executable implements four
commands against local fixture files. No hardware, socket or serial imports.
"""
import argparse,json,os,subprocess,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parents[1];ROOT=HERE.parents[2]
FAKE='''#!/usr/bin/env python3
import sys,os,json,struct,zlib
from pathlib import Path
args=sys.argv[1:];root=Path(os.environ['FAKE_APP_ROOT']);mode=os.environ.get('FAKE_APP_MODE','PASS')
with (root/'calls.jsonl').open('a') as f:f.write(json.dumps(args)+'\\n')
def tail(command):return args[args.index(command)+1:]
if 'read-mac' in args:print('MAC: '+('00:00:00:00:00:00' if mode=='MAC' else '14:c1:9f:22:75:94'))
elif 'read-flash' in args:
 off,size,out=tail('read-flash');off=int(off,0);size=int(size,0)
 if off==0x8000:
  data=Path(os.environ['FAKE_TABLE']).read_bytes().ljust(size,b'\\xff')
  if mode=='TABLE':data=b'\\0'*size
 elif off==0xe000:
  entry=struct.pack('<I',1)+b'\\xff'*20+struct.pack('<II',2,zlib.crc32(struct.pack('<I',1),0xffffffff)&0xffffffff)
  data=entry.ljust(4096,b'\\xff')+b'\\xff'*4096
  if mode=='OTA':data=b'\\0'*size
 else:raise SystemExit('unexpected read offset')
 Path(out).write_bytes(data)
elif 'write-flash' in args:
 rest=tail('write-flash');assert len(rest)==2 and int(rest[0],0)==0x10000
 (root/'simulated_app.bin').write_bytes(Path(rest[1]).read_bytes());print('FILE-ONLY simulated write')
elif 'verify-flash' in args:
 off,path=tail('verify-flash');assert int(off,0)==0x10000
 if mode=='VERIFY' or (root/'simulated_app.bin').read_bytes()!=Path(path).read_bytes():raise SystemExit(1)
 print('Verification successful (FILE-ONLY digest matched).')
else:raise SystemExit('unexpected operation')
'''
def main():
 p=argparse.ArgumentParser();p.add_argument('designation',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 d=json.loads(a.designation.read_text());a.output.mkdir(parents=True,exist_ok=True)
 with tempfile.TemporaryDirectory() as td:
  td=Path(td);fake=td/'esptool-file-only';fake.write_text(FAKE);fake.chmod(0o700)
  port=td/'NOT_A_DEVICE';port.write_text('regular file, never opened by a driver')
  env=dict(os.environ,ESPTOOL_BIN=str(fake),MATDOG_ESP32_PORT=str(port),MATDOG_FLASH_PROFILE='ROBOT_POWERED',
   MATDOG_FLASH_OTA_INGEST='0',MATDOG_FLASH_CANDIDATE_DESIGNATION=str(a.designation.resolve()),
   MATDOG_FLASH_BACKUP=str(Path.home()/'MATDOG/private-preflight/20261013T180138Z/fullflash_A.bin'),
   MATDOG_FLASH_BACKUP_SHA256='9a2bf4bd25b581abc50eabadb70c5c4f54c802c789fdbd908723040939e76c83',
   FAKE_APP_ROOT=str(td),FAKE_TABLE=str(Path(d['artifact_directory'])/'MATDOG_Controller.ino.partitions.bin'))
  for mode in ('PASS','MAC','TABLE','OTA','VERIFY','BACKUP'):
   calls=td/'calls.jsonl';calls.unlink(missing_ok=True);e=dict(env,FAKE_APP_MODE=mode)
   if mode=='BACKUP':e['MATDOG_FLASH_BACKUP_SHA256']='0'*64
   result=subprocess.run(['bash',str(HERE/'flash_app_only.sh')],cwd=ROOT,env=e,capture_output=True,text=True,timeout=180)
   (a.output/(mode+'.log')).write_text(result.stdout+result.stderr)
   invocations=[json.loads(x) for x in calls.read_text().splitlines()] if calls.exists() else []
   writes=[x for x in invocations if 'write-flash' in x];verifies=[x for x in invocations if 'verify-flash' in x]
   assert (result.returncode==0)==(mode=='PASS'),(mode,result.returncode,result.stderr[-1000:])
   assert len(writes)==(1 if mode in ('PASS','VERIFY') else 0),mode
   assert len(verifies)==(1 if mode in ('PASS','VERIFY') else 0),mode
   for call in invocations:
    assert call[call.index('--before')+1]=='no-reset' and call[call.index('--after')+1]=='no-reset'
    assert '--no-stub' in call
   print(mode,'PASS',len(writes),'simulated app writes',len(verifies),'simulated readback checks',flush=True)
 print('APP_ONLY_FROZEN_WORKFLOW = PASS (file-only double; no hardware access)')
if __name__=='__main__':main()
