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
def response_rules(command):
 """Ordered response records from the installed b08322d CommandRouter.
 The final record is the semantic boundary, never an inter-byte silence.
 """
 availability=lambda name:(rf'^{name}\s+init=', 'init detected expected result')
 rules={
  '@STATUS': [('^SYSTEM health=', 'health power_state mode authority uptime_ms profile'), availability('BNO085'), availability('DALY'), availability('SERVO'), availability('LED'), ('^SERVO_POP ', 'canonical expected_now absent_by_design last_census'), (r'^WIFI\s+state=', 'state connected ip rssi_dbm fault'), ('^heap_free=', 'heap_free heap_min_free')],
  '@SYSTEM SOURCE_SIGNATURE': [('^SOURCE_SIGNATURE ', 'build_id firmware version profile board'), ('^FW_VERSION=', 'FW_VERSION GIT_SHA GIT_DIRTY HARDWARE_PROFILE'), ('^FLASH_LAYOUT=', 'FLASH_LAYOUT CAL_RECORD_SCHEMA CAL_MARKER_SCHEMA OTA_INGEST MOTION_STACK MOTION_AUTHORIZED'), ('^BUILD_UTC=', 'BUILD_UTC APP_SHA256 RESTORE'), ('^ota_running_build_id=', 'ota_running_build_id ota_running_image_state reset_reason'), ('^partition=', 'partition address size')],
  '@AUTHORITY STATUS': [('^AUTHORITY owner=', 'owner generation last_result'), ('^AUTHORITY_INHIBIT ', 'active reason'), ('^AUTHORITY_MODE ', 'operating_mode motion_allowed service_allowed'), ('^AUTHORITY_LAST_CLEAR ', ''), ('^AUTHORITY_COUNTERS ', 'grants already_owned releases rejections stale force_clears inhibits'), ('^AUTHORITY_NOTE SAFE_OFF is outside arbitration and always reachable$', '')],
  '@CALIBRATION STATUS': [('^CALIBRATION_CURRENT ', 'state hardware_motion'), ('^CALIBRATION_NOTE ', 'source_of_truth'), ('^CALIBRATION_SESSION ', 'state origin leg last_result'), ('^CALIBRATION_AUTHORITY ', 'held generation'), ('^CALIBRATION_MOTION_PERMIT_STATE ', 'active operator_authorized token_valid'), ('^CALIBRATION_POPULATION ', 'verdict observed mask'), ('^CALIBRATION_PHASE ', 'reported last contacts'), ('^CALIBRATION_RESTORE ', 'required torque_off_required cause'), ('^CALIBRATION_COUNTERS ', 'started completed aborted failed'), ('^CALIBRATION_LF_V25=HISTORICAL_HARDWARE_ORACLE', '')],
  '@CALIBRATION PERSIST STATUS': [('^CALIBRATION_PERSIST=STATUS$', ''), ('^NVS=', 'NVS ESP_ERROR PARTITION'), ('^LOAD_COUNT=', 'LOAD_COUNT LOAD_STATUS BOOT_VERDICT LAST_LOAD_VERDICT DEGRADED'), ('^MARKER=', 'MARKER MARKER_STATE'), ('^CLASS=', 'CLASS ACKNOWLEDGED_GENERATION PENDING_GENERATION AWAITING_ACK_GENERATION ACKNOWLEDGED_RECORD_INTACT'), ('^SLOT_A=', 'SLOT_A GENERATION_HINT DETAIL'), ('^SLOT_B=', 'SLOT_B GENERATION_HINT DETAIL'), ('^STORAGE_SAVE_ALLOWED=', 'STORAGE_SAVE_ALLOWED ACK_ALLOWED RECONCILIATION_REQUIRED ALLOWED_ACTIONS'), ('^WRITE_STATE=', 'WRITE_STATE ACK_UNCERTAIN RECONCILE_UNCERTAIN WRITES_BLOCKED'), ('^CALIBRATION_AVAILABLE=', 'CALIBRATION_AVAILABLE MOTION_AUTHORIZED RESTORE'), ('^CALIBRATION_PERSIST_NOTE ', '')],
  '@IMU STATUS': [availability('BNO085'), ('^stream=', 'stream rv_count runtime_resets')],
  '@BMS STATUS': [availability('DALY'), ('^comm=', 'comm age_ms'), ('^pack_v=', 'pack_v current_a soc cells'), ('^cell_max_mv=', 'cell_max_mv cell_min_mv delta_mv'), ('^charge_mos=', 'charge_mos discharge_mos state alarms')],
  '@WIFI STATUS': [(rf'^{name}', fields) for name,fields in [('WIFI_STATE=', 'WIFI_STATE fault'), ('WIFI_CONFIG ', 'enabled credentials ssid'), ('WIFI_LINK ', 'connected ip rssi_dbm channel'), ('WIFI_RADIO ', 'bssid mode profile source disconnect_reason bandwidth'), ('WIFI_AP ', 'active ssid ip clients provisioned admin'), ('WIFI_SLEEP ', 'configured effective apply_ok'), ('WIFI_ADMIN ', 'phase driver_ap_started driver_sta_started ap_profile_verified portal web_manual_stop'), ('WIFI_FAULT ', 'site event armed count last'), ('WIFI_CONFIG_TX ', 'phase busy error seq kind result'), ('WIFI_REQUEST_LAST ', 'post seq'), ('WIFI_POLICY ', 'mode role sta_profile recovery_level retry_in_ms'), ('WIFI_STA_TEST ', 'seq outcome associated reason attempts ip rssi_dbm elapsed_ms rollback'), ('WIFI_SCAN ', 'running count starts failures inhibited roam'), ('WIFI_WORKER ', 'max_us stack_free'), ('WIFI_PERFORMANCE ', 'roam_enabled threshold hysteresis scan_ms dwell_ms bandwidth_configured bandwidth_apply_ok'), ('WIFI_AP_POLICY ', 'always timeout_ms reload_pending'), ('WIFI_PROFILE index=0 ', 'index enabled ssid dhcp ip mask gateway dns'), ('WIFI_PROFILE index=1 ', 'index enabled ssid dhcp ip mask gateway dns'), ('WIFI_RETRY ', 'backoff_ms state_since_ms'), ('WIFI_COUNTERS ', 'radio_starts attempts connects reconnects timeouts link_losses'), ('WIFI_TICK ', 'last_us max_us')]],
  '@LED STATUS': [availability('LED'), ('^pixels=', 'pixels pin brightness_max test_running data_pin_driven'), ('^presentation=', 'presentation soc_valid soc_percent soc_segments charging charging_fault charge_complete_verified battery_warning battery_critical diagnostic')],
  '@SERVO PREFLIGHT': [('^SERVO_PREFLIGHT=PASS ', 'profile source_sha256'), ('^evaluated=', 'evaluated pass no_response mismatch incomplete'), ('^expected_unit joint expected_id observed_id model offset profile torque position result$', '')] + [('^JOINT ', 'expected_physical_unit joint expected_bus_id observed_bus_id model position_offset persistent_profile torque_enable present_position result')]*12 + [('^NOTE present_position is a raw liveness tick, NOT q0$', ''), availability('SERVO')],
 }
 if command.startswith('@SERVO READ '):
  check(command in [f'@SERVO READ {i}' for i in IDS], 'not in read-only whitelist')
  return [('^SERVO_READ ', 'id position speed load voltage temp torque current')]
 check(command in rules, 'not in read-only whitelist')
 return rules[command]

def line_fields(line):
 matches=list(re.finditer(r'\b([A-Za-z_][A-Za-z_0-9]*)=',line));out={}
 for i,m in enumerate(matches):
  check(m[1] not in out, 'ambiguous duplicate field '+m[1])
  out[m[1]]=line[m.end():matches[i+1].start() if i+1<len(matches) else len(line)].strip()
 return out

def validate_response(command,lines):
 f=[line_fields(line) for line in lines]
 if command=='@SYSTEM SOURCE_SIGNATURE':
  a,b,c,d,e,g=f;git=b['GIT_SHA']
  check(re.fullmatch('[0-9a-f]{40}',git) is not None, 'installed source identity unavailable or malformed')
  check(a['build_id']==git[:12] and e['ota_running_build_id']==a['build_id'], 'discordant installed build/source identity')
  check(a['firmware']=='MATDOG Controller' and a['board']=='YD-ESP32-S3 N16R8' and a['version']==b['FW_VERSION'] and a['profile']==b['HARDWARE_PROFILE']=='ROBOT_POWERED' and b['GIT_DIRTY']=='NO', 'discordant installed source signature fields')
  check(c['FLASH_LAYOUT']==layout.LAYOUT_ID and c['CAL_RECORD_SCHEMA']=='1' and c['CAL_MARKER_SCHEMA']=='2' and c['OTA_INGEST']==c['MOTION_AUTHORIZED']=='0', 'installed source layout/schema/authority mismatch')
  check(d['APP_SHA256']=='ASSOCIATED_MANIFEST' and re.fullmatch(r'0x[0-9a-fA-F]+',g['address']) and int(g['address'],16) in (0x10000,0x510000) and int(g['size'],16)==0x500000, 'invalid running partition identity')
 elif command=='@STATUS':check(f[0]['authority']=='NONE' and f[0]['mode']=='MAINTENANCE' and f[0]['profile']=='ROBOT_POWERED', 'status authority/mode/profile not safe for read-only preflight')
 elif command=='@AUTHORITY STATUS':check(f[0]['owner']=='NONE' and f[2]['operating_mode']=='MAINTENANCE' and f[2]['motion_allowed']=='NO', 'authority response not independently NONE/MAINTENANCE')
 elif command=='@CALIBRATION STATUS':check(f[3]['held']=='NO' and f[4]['active']==f[4]['operator_authorized']==f[4]['token_valid']=='NO', 'calibration authority or permit active')
 elif command=='@CALIBRATION PERSIST STATUS':
  check(f[1]['NVS']=='READY' and f[1]['ESP_ERROR']=='0' and f[1]['PARTITION']=='matdog_nvs' and f[2]['DEGRADED']=='0' and f[2]['LAST_LOAD_VERDICT']=='VALID_ACKNOWLEDGED' and f[3]['MARKER_STATE']=='IDLE' and f[4]['CLASS']=='CONSISTENT' and f[4]['ACKNOWLEDGED_GENERATION']=='1' and f[4]['PENDING_GENERATION']==f[4]['AWAITING_ACK_GENERATION']=='0' and f[4]['ACKNOWLEDGED_RECORD_INTACT']=='1' and f[8]['WRITE_STATE']=='OPEN' and f[8]['ACK_UNCERTAIN']==f[8]['RECONCILE_UNCERTAIN']==f[8]['WRITES_BLOCKED']=='0', 'current acknowledged gen-1 not proven by complete persistence response')
 elif command=='@SERVO PREFLIGHT':
  check(re.fullmatch('[0-9a-f]{64}',f[0]['source_sha256']) and f[1]['evaluated']==f[1]['pass']=='12' and f[1]['no_response']==f[1]['mismatch']==f[1]['incomplete']=='0', '12-servo complete preflight PASS required')
  ids=[]
  for row in f[3:15]:
   i=int(row['expected_bus_id']);ids.append(i)
   check(row['observed_bus_id']==str(i) and row['result']=='PASS' and row['persistent_profile']=='MATCH' and row['torque_enable']=='0' and 0<=int(row['present_position'])<4096 and int(row['model'])==777, 'preflight joint identity/profile/OFF mismatch')
  check(len(set(ids))==12 and set(ids)==set(IDS[:-1]), 'preflight duplicate/missing/unexpected joint ID')
 elif command.startswith('@SERVO READ '):
  row=f[0];check(row['id']==command.split()[-1] and 0<=int(row['position'])<4096 and row['torque']=='0', 'current requested servo ID/valid position/OFF not proven')

class SerialResponseReader:
 """Single outstanding request; preserve raw bytes and reject mixed replies."""
 def __init__(self,port,audit,clock=time.monotonic):
  self.port=port;self.audit=audit;self.clock=clock;self.pending=bytearray()
  self.known=[re.compile(pattern) for c in (*READS,*(f'@SERVO READ {i}' for i in IDS)) for pattern,_ in response_rules(c)]+[re.compile('^SERVO_PREFLIGHT=')]
 def read(self,command,timeout=None):
  rules=response_rules(command);timeout=timeout if timeout is not None else (60 if command=='@SERVO PREFLIGHT' else 20)
  check(0<timeout<=60, 'finite serial timeout required')
  self.audit(command+'\n');lines=[];stage=0;started=None;total=0;deadline=self.clock()+timeout
  patterns=[re.compile(p) for p,_ in rules]
  def consume(chunk,requested):
   nonlocal stage,started,total
   self.audit(chunk.decode('utf-8','backslashreplace'))
   total+=len(chunk);check(total<=1024*1024, 'serial response byte limit exceeded');self.pending.extend(chunk)
   while b'\n' in self.pending:
    raw,_,tail=self.pending.partition(b'\n');self.pending=bytearray(tail);check(len(raw)<=4096,'serial line too long');line=raw.decode('utf-8').strip()
    if not line or line==command:continue
    check(not re.match(r'^(?:ERROR(?:[=: ]|$)|REASON(?:[=: ]|$)|SERVO_READ .*result=|SERVO_PREFLIGHT=(?:FAIL|BLOCKED)|CALIBRATION_PERSIST=(?:REFUSED|BLOCKED))',line), 'serial command error: '+line.split()[0])
    if requested and command=='@SERVO PREFLIGHT' and line.startswith('SERVO_PREFLIGHT=STARTED '):
     check(stage==0 and started is None, 'ambiguous preflight start');started=line_fields(line);check(started.get('joints')=='12' and started.get('profile'), 'invalid preflight start');continue
    if requested and stage<len(rules) and patterns[stage].match(line):
     fields=line_fields(line);empty=('ssid','bssid') if command=='@WIFI STATUS' else ();check(all(k in fields and (fields[k] or k in empty) for k in rules[stage][1].split()), 'incomplete response fields for '+command)
     lines.append(line);stage+=1;continue
    check(not any(p.match(line) for p in self.known), 'unexpected/duplicate response while collecting '+command)
    # Boot and periodic MAG/RV/GYR/SAVE_GATE/COUNTS remain in the raw audit,
    # never enter a response or satisfy a safety predicate.
  # Do not reset_input_buffer: stale complete responses are ambiguity, not
  # evidence for the request about to be sent. Partial lines are retained.
  while self.port.in_waiting or self.pending:
   check(self.clock()<deadline,'serial pre-request drain timeout')
   consume(self.port.read(min(8192,self.port.in_waiting) if self.port.in_waiting else 8192),False)
  check(not self.pending,'partial unsolicited line before request')
  packet=(command+'\n').encode();check(self.port.write(packet)==len(packet),'serial command short write')
  while self.clock()<deadline:
   consume(self.port.read(8192),True)
   if stage==len(rules) and not self.pending and self.clock()<=deadline:
    if command=='@SERVO PREFLIGHT':check(started is not None and started['profile']==line_fields(lines[0])['profile'],'missing/discordant preflight start/profile')
    validate_response(command,lines);return '\n'.join(lines)+'\n'
  raise RuntimeError('serial response timeout for '+command+'; missing semantic record '+str(stage+1)+'/'+str(len(rules)))
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
 s=None
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
  with log.open('a') as audit:
   reader=SerialResponseReader(s,lambda text:(audit.write(text),audit.flush()))
   observed={command:reader.read(command) for command in READS}
  report['gates']['diagnostics_received']=all(observed.values())
  check(re.search(r'(?:authority|AUTHORITY|owner)=NONE(?:\s|$)',observed['@STATUS']+' '+observed['@AUTHORITY STATUS']) is not None,'authority not independently observed NONE')
  check('SERVO_PREFLIGHT=PASS' in observed['@SERVO PREFLIGHT'] and 'evaluated=12 pass=12' in observed['@SERVO PREFLIGHT'],'12-servo persistent profile/model/mapping preflight required; no writes performed')
  report['gates']['servo_persistent_profile']=True
  persist=observed['@CALIBRATION PERSIST STATUS'];check('VALID_ACKNOWLEDGED' in persist and 'ACKNOWLEDGED_GENERATION=1 ' in persist and 'MARKER_STATE=IDLE' in persist and 'ACKNOWLEDGED_RECORD_INTACT=1' in persist,'current acknowledged gen-1 is not proven by production persistence reader')
  installed_source=re.search(r'GIT_SHA=([0-9a-f]{40})',observed['@SYSTEM SOURCE_SIGNATURE']);check(installed_source is not None,'installed source identity unavailable');report['installed_source_commit']=installed_source[1]
  for i in IDS:
   with log.open('a') as audit:text=SerialResponseReader(s,lambda text:(audit.write(text),audit.flush())).read(f'@SERVO READ {i}')
   m=re.search(rf'SERVO_READ id={i} position=(\d+).*?torque=(\d+)',text)
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
 finally:
  if s is not None and s.is_open:s.close()
  save()
 print(json.dumps({'result':report['result'],'report':str(root/'preflight_report.json'),'flash_executed':False},indent=2));return 0 if 'error' not in report else 1
if __name__=='__main__':sys.exit(main())
