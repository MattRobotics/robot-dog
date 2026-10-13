#!/usr/bin/env python3
"""PLAN by default. Explicit execution performs USB reads + full backups only.
Run ON ASUS over a PC maintenance connection independent of robot Wi-Fi.
No flash writer, Motion grant, calibration, BMS write or NVS writer is present.
"""
import argparse,datetime,hashlib,json,os,re,shutil,struct,subprocess,sys,time,zlib
from pathlib import Path
HERE=Path(__file__).resolve().parent;FW=HERE.parents[1];REPO=FW.parents[1]
sys.path.insert(0,str(FW/'scripts'))
import matdog_layout as layout
PORT='/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00'
MAC='14:c1:9f:22:75:94';SIZE=16*1024*1024
IDS=(11,12,13,21,22,23,31,32,33,41,42,43,51)
READS=('@STATUS','@SYSTEM SOURCE_SIGNATURE','@AUTHORITY STATUS','@CALIBRATION STATUS','@CALIBRATION PERSIST STATUS','@IMU STATUS','@BMS STATUS','@WIFI STATUS','@LED STATUS','@SERVO PREFLIGHT')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def check(b,s):
 if not b:raise RuntimeError(s)
def candidate(path):
 designation=json.loads(path.read_text());root=Path(designation['artifact_directory']);manifest={}
 for line in (root/'matdog_build_manifest.txt').read_text().splitlines():
  if '=' in line:k,v=line.split('=',1);manifest[k]=v
 image=root/'MATDOG_Controller.ino.bin'
 check(manifest.get('SOURCE_STATE')=='CLEAN','candidate source not CLEAN')
 check(manifest.get('HARDWARE_PROFILE')=='ROBOT_POWERED' and manifest.get('OTA_INGEST_ENABLED')=='0','candidate profile/OTA')
 check(manifest.get('SUPERVISED_VALIDATION_CAPABLE')=='1' and manifest.get('BOOT_MOTION_AUTHORIZED')=='0','candidate capability/startup flags')
 check(sha(image)==manifest['APPLICATION_SHA256']==designation['application_sha256'],'candidate SHA mismatch')
 check(image.stat().st_size<int(manifest['APP_PARTITION_SIZE']),'candidate does not fit')
 return designation,manifest

def nvs_records(full):
 data=full[0xfe0000:0xff0000];valid=[];at=0
 while True:
  at=data.find(b'MDCR',at)
  if at<0:break
  blob=data[at:at+1088];at+=4
  if len(blob)!=1088 or zlib.crc32(blob[:-4])!=struct.unpack_from('<I',blob,1088-4)[0]:continue
  schema=struct.unpack_from('<H',blob,4)[0];generation=struct.unpack_from('<I',blob,12)[0]
  valid.append(dict(schema=schema,generation=generation,sha256=hashlib.sha256(blob).hexdigest(),offset=0xfe0000+at-4))
 return valid

def application_image(slot):
 check(len(slot)>=24 and slot[0]==0xe9 and 1<=slot[1]<=16,'active ESP image header invalid')
 check(struct.unpack_from('<H',slot,12)[0]==9,'active image chip mismatch')
 at=24;checksum=0xef
 for _ in range(slot[1]):
  check(at+8<=len(slot),'truncated image segment header');length=struct.unpack_from('<I',slot,at+4)[0];at+=8
  check(0<length<=len(slot)-at,'invalid image segment length')
  for value in slot[at:at+length]:checksum^=value
  at+=length
 end=(at+16)&~15;check(end<=len(slot) and slot[end-1]==checksum,'active image checksum failed')
 if slot[23]==1:
  check(slot[end:end+32]==hashlib.sha256(slot[:end]).digest(),'active image appended SHA failed');end+=32
 else:check(slot[23]==0,'unknown image digest flag')
 return slot[:end]

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute-readonly',action='store_true');p.add_argument('--confirm-asus-independent-maintenance-link',action='store_true');p.add_argument('--port',default=PORT);p.add_argument('--esptool',type=Path,default=Path.home()/'.arduino15/packages/esp32/tools/esptool_py/5.3.1/esptool');p.add_argument('--designation',type=Path,default=REPO/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13/designated_candidate.json');p.add_argument('--session-root',type=Path,default=Path.home()/'MATDOG/private-preflight');p.add_argument('--independent-copy-dir',type=Path);a=p.parse_args()
 plan={'mode':'PLAN_ONLY','port':a.port,'expected_mac':MAC,'reads':[*READS,*[f'@SERVO READ {i}' for i in IDS]],'backup':'two complete 16MiB ROM reads; no erase/write; remain in ROM after capture','required':'independent ASUS link, SAFE_OFF readbacks, current gen-1, external recoverable backup copy, candidate release review','flash':'SEPARATE_EXPLICIT_GO; this program never invokes flash'}
 if not a.execute_readonly:print(json.dumps(plan,indent=2));return 0
 check(a.confirm_asus_independent_maintenance_link,'independent ASUS link must be explicitly confirmed')
 check(a.esptool.is_file(),'pinned esptool 5.3.1 unavailable')
 os.umask(0o077);root=a.session_root/datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ');root.mkdir(parents=True,exist_ok=False)
 report={'mode':'EXECUTED_READONLY','flash_executed':False,'motion_executed':False,'result':'NO_GO','session_directory':str(root),'gates':{}};log=root/'usb_readonly.log'
 def save(): (root/'preflight_report.json').write_text(json.dumps(report,indent=2)+'\n')
 try:
  import serial,fcntl
  from compile_record_decoder import compile_decoder
  decoder=root/'record_decoder';compile_decoder(decoder)
  designation,manifest=candidate(a.designation)
  check(designation.get('release_ready')=='PASS','release review not PASS')
  from verify_designated_candidate import verify
  verify(a.designation)
  report['candidate']=designation
  report['tool_identity_sha256']={str(f.relative_to(FW)):sha(f) for f in [Path(__file__).resolve(),FW/'scripts/flash_app_only.sh',FW/'scripts/build_manifest.py',FW/'scripts/matdog_layout.py',FW/'scripts/ota_partition_logic.py',FW/'scripts/release/validate_record_offline.cpp']}
  report['esptool_sha256']=sha(a.esptool)
  lock=open(str(a.session_root/'serial.lock'),'a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  s=serial.Serial();s.port=a.port;s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False;s.open()
  def read(command,seconds=3,terminal=None):
   check(command in plan['reads'],'not in read-only whitelist');s.reset_input_buffer();s.write((command+'\n').encode());start=time.monotonic();data=bytearray();last=start
   while time.monotonic()-start<seconds:
    b=s.read(8192)
    if b:data.extend(b);last=time.monotonic()
    if data and time.monotonic()-start>.6 and time.monotonic()-last>.3 and (terminal is None or terminal in data.decode('utf-8','replace')):break
   text=data.decode('utf-8','replace');log.open('a').write(command+'\n'+text+'\n');return text
  observed={command:read(command,60,'NOTE present_position') if command=='@SERVO PREFLIGHT' else read(command) for command in READS};report['gates']['diagnostics_received']=all(observed.values())
  check(re.search(r'(?:authority|AUTHORITY|owner)=NONE(?:\s|$)',observed['@STATUS']+' '+observed['@AUTHORITY STATUS']) is not None,'authority not independently observed NONE')
  check('SERVO_PREFLIGHT=PASS' in observed['@SERVO PREFLIGHT'] and 'evaluated=12 pass=12' in observed['@SERVO PREFLIGHT'],'12-servo persistent profile/model/mapping preflight required; no writes performed')
  report['gates']['servo_persistent_profile']=True
  persist=observed['@CALIBRATION PERSIST STATUS'];check('VALID_ACKNOWLEDGED' in persist and 'ACKNOWLEDGED_GENERATION=1 ' in persist and 'MARKER_STATE=IDLE' in persist and 'ACKNOWLEDGED_RECORD_INTACT=1' in persist,'current acknowledged gen-1 is not proven by production persistence reader')
  installed_source=re.search(r'GIT_SHA=([0-9a-f]{40})',observed['@SYSTEM SOURCE_SIGNATURE']);check(installed_source is not None,'installed source identity unavailable');report['installed_source_commit']=installed_source[1]
  for i in IDS:
   text=read(f'@SERVO READ {i}');m=re.search(rf'SERVO_READ id={i} position=(\d+).*?torque=(\d+)',text)
   check(m is not None and int(m[1])<4096 and m[2]=='0',f'fresh valid torque-OFF required for ID {i}; no ROM reset performed')
  report['gates']['13_servo_readbacks_off']=True;s.close();save()
  def tool(operation,args,before='no-reset'):
   check(operation in ('read-mac','flash-id','read-flash'),'ROM operation not read-only')
   command=[str(a.esptool),'--chip','esp32s3','--port',a.port,'--baud','921600','--no-stub','--before',before,'--after','no-reset',operation,*args]
   result=subprocess.run(command,capture_output=True,text=True,timeout=900);(root/(operation+'_'+str(time.time_ns())+'.log')).write_text(result.stdout+result.stderr);check(result.returncode==0,operation+' failed');return result.stdout
  text=tool('read-mac',[],before='usb-reset');check(MAC in text.lower(),'MAC mismatch')
  text=tool('flash-id',[]);check('16MB' in text or '16 MB' in text,'16MiB flash not identified')
  paths=[root/'fullflash_A.bin',root/'fullflash_B.bin']
  for dest in paths:tool('read-flash',['0x0',str(SIZE),str(dest)]);check(dest.stat().st_size==SIZE,'full backup length');os.chmod(dest,0o600)
  digest=sha(paths[0]);check(digest==sha(paths[1]),'independent full reads differ');full=paths[0].read_bytes()
  table=full[0x8000:0x8c00];report['backup_sha256']=digest;report['partition_sha256']=hashlib.sha256(table).hexdigest()
  (root/'partition_table.bin').write_bytes(table);(root/'default_nvs.bin').write_bytes(full[0x9000:0xe000]);(root/'matdog_nvs.bin').write_bytes(full[0xfe0000:0xff0000])
  entries=layout.parse_partition_table(table);layout.check_entries(entries);report['gates']['layout_preserves_nvs']=True
  records=nvs_records(full);check(any(r['schema']==1 and r['generation']==1 for r in records),'no intact gen-1 record envelope found');report['gen1_record_envelopes']=records
  current=[r for r in records if r['schema']==1 and r['generation']==1];check(len({r['sha256'] for r in current})==1,'gen-1 backup records ambiguous; require NVS key extraction before GO')
  blob=current[0];record=root/'current_gen1.bin';record.write_bytes(full[blob['offset']:blob['offset']+1088]);result=subprocess.run([str(decoder),str(record)],capture_output=True,text=True);check(result.returncode==0,'production gen-1 semantic validation failed');report['gen1_semantics']=json.loads(result.stdout);report['gates']['gen1_semantic_validation']=True
  report['gates']['installed_currentness']='OPERATOR_INSTALLATION_REVIEW_REQUIRED_BEFORE_MOTION; never inferred from record CRC'

  check(manifest['PARTITION_TABLE_SHA256']==report['partition_sha256'],'installed/candidate layouts differ')
  from ota_partition_logic import resolve_application_partition,parse_sdkconfig_ota_flags
  rollback,anti=parse_sdkconfig_ota_flags((Path(designation['artifact_directory'])/'sdkconfig').read_text());active=resolve_application_partition(table,full[0xe000:0x10000],rollback=rollback,anti_rollback=anti)
  image=application_image(full[active.offset:active.offset+active.size]);check(report['installed_source_commit'].encode() in image,'live source identity absent from active backup image')
  report['installed_application']={'offset':active.offset,'slot':active.label,'size':len(image),'sha256':hashlib.sha256(image).hexdigest(),'source_commit':report['installed_source_commit']}
  report['gates']['active_application_identity']=True
  report['gates']['candidate_file_identity']=True
  if a.independent_copy_dir:
   target=a.independent_copy_dir;check(target.exists() and target.stat().st_dev!=root.stat().st_dev,'independent mounted medium required; same filesystem rejected')
   copy=target/('MATDOG_'+root.name+'.bin');check(not copy.exists(),'preserve existing backup');shutil.copyfile(paths[0],copy);check(sha(copy)==digest and copy.stat().st_size==SIZE,'external copy verification failed');report['recovery_copy']=str(copy);report['gates']['independent_recovery_copy']='COPY_SHA_VERIFIED_PHYSICAL_INDEPENDENCE_REVIEW_REQUIRED'
  else:report['gates']['independent_recovery_copy']='BLOCKED: verify encrypted off-PC recovery copy or independent medium'
  report['gates']['release_ready']=designation.get('release_ready','BLOCKED')
  report['gates']['installed_source_identity']=True
  (root/'fullflash_A.bin.manifest.txt').write_text(f'BACKUP_SHA256={digest}\nBACKUP_SIZE={SIZE}\nMAC={MAC}\nPARTITION_TABLE_SHA256={report["partition_sha256"]}\n')
  report['usb_diagnostics_sha256']=sha(log)
  report['result']='NO_GO_CURRENT_POWER_SENSORS_AND_INDEPENDENT_RECOVERY_REVIEW_REQUIRED';report['flash_authorized']=False
 except Exception as e:report['error']=str(e)
 finally:save()
 print(json.dumps({'result':report['result'],'report':str(root/'preflight_report.json'),'flash_executed':False},indent=2));return 0 if 'error' not in report else 1
if __name__=='__main__':sys.exit(main())
