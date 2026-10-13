#!/usr/bin/env python3
"""No serial port is opened. Verify plan, native gen-1 decoder and image parser."""
import importlib.util,json,subprocess,sys,tempfile,unittest,hashlib,struct,zlib,time,threading
from pathlib import Path
FW=Path(__file__).resolve().parents[2];sys.path.insert(0,str(FW/'scripts/release'));sys.path.insert(0,str(FW/'scripts'))
import motion_remote_preflight as p
import review_preflash
import verify_designated_candidate as provenance
import verify_application_partition as partition_reader
from unittest.mock import patch
from compile_record_decoder import compile_decoder
class Tests(unittest.TestCase):
 def test_plan_has_no_execution(self):
  r=subprocess.run([sys.executable,str(FW/'scripts/release/motion_remote_preflight.py')],capture_output=True,text=True,check=True);d=json.loads(r.stdout);self.assertEqual(d['mode'],'PLAN_ONLY');self.assertNotIn('GRANT',' '.join(d['reads']));self.assertEqual(len(d['reads']),23)
 def image(self):
  h=bytearray(24);h[0]=0xe9;h[1]=1;struct.pack_into('<H',h,12,9);h[23]=1;data=bytes([1,2,3,4]);h+=struct.pack('<II',0x3c000020,len(data))+data;end=(len(h)+16)&~15;h+=bytes(end-len(h));h[-1]=0xef^1^2^3^4;return bytes(h)+hashlib.sha256(h).digest()
 def test_active_image_integrity_and_exact_length(self):
  image=self.image();self.assertEqual(p.application_image(image+bytes([255])*4096),image)
  damaged=bytearray(image);damaged[34]^=1
  with self.assertRaises(RuntimeError):p.application_image(damaged)
  damaged=bytearray(image);damaged[-1]^=1
  with self.assertRaises(RuntimeError):p.application_image(damaged)
  with self.assertRaises(RuntimeError):p.application_image(image[:25])
 def test_native_gen1_semantics_and_crc(self):
  with tempfile.TemporaryDirectory(prefix='matdog-preflight-test-') as temp:
   root=Path(temp);unit=root/'golden.cpp';unit.write_text('#include "'+str(FW/'scripts/tests/calibration_record_golden.h')+'"\n#include <fstream>\nint main(int argc,char** argv){if(argc!=2)return 1;auto r=golden::goldenRecord();uint8_t bytes[1088];size_t n=0;if(matdog::calibration::encodeCalibrationRecord(r,bytes,sizeof(bytes),&n)!=matdog::calibration::CalibrationRecordStatus::OK)return 2;std::ofstream f(argv[1],std::ios::binary);f.write(reinterpret_cast<char*>(bytes),n);}\n')
   compile_decoder(root/'decoder');compile_decoder(root/'golden',unit);record=root/'record.bin';subprocess.run([str(root/'golden'),str(record)],check=True)
   result=subprocess.run([str(root/'decoder'),str(record)],capture_output=True,text=True);self.assertEqual(result.returncode,0);self.assertEqual(json.loads(result.stdout)['status'],'OK')
   full=bytearray([255])*(16*1024*1024);full[0xfe0200:0xfe0200+1088]=record.read_bytes();self.assertEqual(p.nvs_records(full)[0]['generation'],1)
   for offset in (20,300,1086):
    data=bytearray(record.read_bytes());data[offset]^=1;(root/'bad.bin').write_bytes(data);r=subprocess.run([str(root/'decoder'),str(root/'bad.bin')],capture_output=True,text=True);self.assertNotEqual(r.returncode,0)
   full[0xfe0200+300]^=1;self.assertEqual(p.nvs_records(full),[])
 def test_no_rom_writer_exists(self):
  text=(FW/'scripts/release/motion_remote_preflight.py').read_text();self.assertIn("operation in ('read-mac','flash-id','read-flash')",text);self.assertNotIn("'write-flash'",text);self.assertNotIn("'erase-flash'",text);self.assertNotIn('nvs_set',text)
 def test_partial_review_cannot_grant_go(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);report=root/'report.json';review=root/'review.json';report.write_text(json.dumps({'mode':'EXECUTED_READONLY','error':'no sensor snapshot'}));review.write_text('{}')
   with self.assertRaises(RuntimeError):review_preflash.review(report,review)
   report.write_text(json.dumps({'mode':'EXECUTED_READONLY'}));review.write_text(json.dumps({'preflight_report_sha256':'0'*64}))
   with self.assertRaises(RuntimeError):review_preflash.review(report,review)
   self.assertIn('NO_GO_CURRENT_POWER_SENSORS_AND_INDEPENDENT_RECOVERY_REVIEW_REQUIRED',(FW/'scripts/release/motion_remote_preflight.py').read_text())

# Synthetic wire replies use the exact installed router record names/order.
# They contain no copied credentials or private calibration/encoder records.
def wire_reply(command):
 values=dict(health='READY',power_state='SAFE_OFF',mode='MAINTENANCE',authority='NONE',profile='ROBOT_POWERED',init='OK',detected='PRESENT',expected='POWERED',result='PASS',build_id='b08322dedc55',firmware='MATDOG Controller',version='0.2.0-dev.3',board='YD-ESP32-S3 N16R8',FW_VERSION='0.2.0-dev.3',GIT_SHA='b08322dedc5564c080f691dc98047511ef2b953f',GIT_DIRTY='NO',HARDWARE_PROFILE='ROBOT_POWERED',FLASH_LAYOUT=p.layout.LAYOUT_ID,CAL_RECORD_SCHEMA='1',CAL_MARKER_SCHEMA='2',OTA_INGEST='0',MOTION_STACK='TEST_ONLY',MOTION_AUTHORIZED='0',BUILD_UTC='2026-10-11T12:00:00Z',APP_SHA256='ASSOCIATED_MANIFEST',RESTORE='NOT_IMPLEMENTED',ota_running_build_id='b08322dedc55',ota_running_image_state='VALID',reset_reason='POWERON',partition='app0',address='0x010000',size='0x500000',owner='NONE',operating_mode='MAINTENANCE',motion_allowed='NO',service_allowed='YES',held='NO',active='NO',operator_authorized='NO',token_valid='NO',hardware_motion='BLOCKED',source_of_truth='MATDOG_JOINT_CALIBRATION.yaml',NVS='READY',ESP_ERROR='0',PARTITION='matdog_nvs',LOAD_COUNT='1',LOAD_STATUS='OK',BOOT_VERDICT='VALID_ACKNOWLEDGED',LAST_LOAD_VERDICT='VALID_ACKNOWLEDGED',DEGRADED='0',MARKER='VALID',MARKER_STATE='IDLE',CLASS='CONSISTENT',ACKNOWLEDGED_GENERATION='1',PENDING_GENERATION='0',AWAITING_ACK_GENERATION='0',ACKNOWLEDGED_RECORD_INTACT='1',SLOT_A='VALID',SLOT_B='EMPTY',ALLOWED_ACTIONS='NONE',WRITE_STATE='OPEN',source_sha256='a'*64,evaluated='12',**{'pass':'12'},ssid='',bssid='',stream='OFF',comm='OK',pack_v='12.0',current_a='0.0',soc='50%',cells='3',charge_mos='OFF',discharge_mos='OFF',alarms='0000 0000 0000 0000',pixels='12',pin='47',brightness_max='60',presentation='UNKNOWN',soc_percent='UNKNOWN',state='IDLE',id=command.split()[-1],position='100',torque='0')
 lines=[];joint=0
 for pattern,required in p.response_rules(command):
  prefix=pattern.removeprefix('^').removesuffix('$').replace(r'\s+',' ')
  keys=required.split();d=values.copy()
  if prefix.startswith('WIFI_PROFILE index='):
   d['index']=prefix.split('=')[1].strip();prefix='WIFI_PROFILE '
  if prefix.startswith('JOINT '):
   i=p.IDS[joint];joint+=1;d.update(expected_physical_unit='TEST_ONLY',joint='test_joint',expected_bus_id=str(i),observed_bus_id=str(i),model='777',position_offset='0',persistent_profile='MATCH',torque_enable='0',present_position='100',result='PASS')
  if keys and prefix.endswith(keys[0]+'='):prefix=prefix[:-len(keys[0])-1]
  if prefix=='AUTHORITY_LAST_CLEAR ':prefix+='NONE'
  if prefix=='CALIBRATION_PERSIST_NOTE ':prefix+='available is evidence, not a motion permit; no JointTransform is admitted from storage'
  lines.append(prefix+' '.join(k+'='+str(d.get(k,'0')) for k in keys))
 if command=='@SERVO PREFLIGHT':lines.insert(0,'SERVO_PREFLIGHT=STARTED joints=12 profile=ROBOT_POWERED')
 return '\n'.join(lines)+'\n'

class Wire:
 def __init__(self,replies,initial=()):self.now=0.;self.replies=replies;self.events=list(initial);self.writes=[]
 @property
 def in_waiting(self):return sum(len(b) for at,b in self.events if at<=self.now)
 def write(self,data):
  cmd=data.decode().strip();self.writes.append((self.now,cmd));self.events.extend((self.now+dt,chunk) for dt,chunk in self.replies[cmd]);self.events.sort(key=lambda x:x[0]);return len(data)
 def read(self,n):
  self.now+=.001 if self.in_waiting else .05
  if not self.events or self.events[0][0]>self.now:return b''
  at,data=self.events.pop(0);part=data[:n]
  if len(data)>n:self.events.insert(0,(at,data[n:]))
  return part

def reader(replies,initial=()):
 w=Wire(replies,initial);audit=[];return p.SerialResponseReader(w,audit.append,lambda:w.now),w,audit

class SerialTests(unittest.TestCase):
 def test_delayed_git_sha_is_not_assigned_to_authority(self):
  source=wire_reply('@SYSTEM SOURCE_SIGNATURE').splitlines(keepends=True)
  r,w,a=reader({'@SYSTEM SOURCE_SIGNATURE':[(.1,source[0].encode()),(1.2,source[1].encode()),(1.4,''.join(source[2:]).encode())],'@AUTHORITY STATUS':[(.05,wire_reply('@AUTHORITY STATUS').encode())]})
  text=r.read('@SYSTEM SOURCE_SIGNATURE');self.assertIn('GIT_SHA=b08322dedc5564c080f691dc98047511ef2b953f',text);self.assertGreaterEqual(w.now,1.4)
  authority=r.read('@AUTHORITY STATUS');self.assertNotIn('GIT_SHA=',authority);self.assertGreaterEqual(w.writes[1][0],1.4);self.assertIn(source[1],''.join(a))
 def test_boot_delay_does_not_complete_status(self):
  r,w,a=reader({'@STATUS':[(.1,b'load:0x403c8700,len:0xc2c\nentry 0x403c88f8\n'),(2.,wire_reply('@STATUS').encode())]});text=r.read('@STATUS');self.assertTrue(text.startswith('SYSTEM health='));self.assertGreaterEqual(w.now,2.);self.assertIn('load:', ''.join(a));self.assertNotIn('load:',text)
 def test_all_23_contracts_fragmented_multiline_with_telemetry(self):
  for command in (*p.READS,*(f'@SERVO READ {i}' for i in p.IDS)):
   with self.subTest(command=command):
    data=wire_reply(command).replace('\n','\r\n').encode();chunks=[(.1+j*.01,data[i:i+7]) for j,i in enumerate(range(0,len(data),7))]
    r,w,a=reader({command:chunks});text=r.read(command);self.assertEqual(text,wire_reply(command).replace('SERVO_PREFLIGHT=STARTED joints=12 profile=ROBOT_POWERED\n','').strip()+'\n')
  r,w,a=reader({'@IMU STATUS':[(.1,b'GYR x=0 y=0 z=0\n'+wire_reply('@IMU STATUS').encode()+b'COUNTS n=1\n')]});self.assertNotIn('GYR',r.read('@IMU STATUS'));self.assertIn('GYR',''.join(a))
 def test_timeout_does_not_send_next_request(self):
  text=wire_reply('@SYSTEM SOURCE_SIGNATURE').splitlines(keepends=True)[0].encode();r,w,a=reader({'@SYSTEM SOURCE_SIGNATURE':[(.1,text)]})
  with self.assertRaisesRegex(RuntimeError,'timeout'):r.read('@SYSTEM SOURCE_SIGNATURE',2)
  self.assertEqual(len(w.writes),1);self.assertLess(w.now,2.1);self.assertIn(text.decode(),''.join(a))
 def test_signature_missing_or_discordant_fields_fail_closed(self):
  source=wire_reply('@SYSTEM SOURCE_SIGNATURE')
  for bad in [source.replace('GIT_SHA=b08322dedc5564c080f691dc98047511ef2b953f','GIT_SHA=b08322dedc55'),source.replace('build_id=b08322dedc55','build_id=0123456789ab'),source.replace('FW_VERSION=0.2.0-dev.3','FW_VERSION=0.2.0-dev.4'),source.replace('MOTION_AUTHORIZED=0','MOTION_AUTHORIZED=1'),source.replace(' GIT_DIRTY=NO',''),source.replace(' GIT_DIRTY=NO',' GIT_DIRTY=NO GIT_DIRTY=YES')]:
   r,_,_=reader({'@SYSTEM SOURCE_SIGNATURE':[(.1,bad.encode())]})
   with self.assertRaises(RuntimeError):r.read('@SYSTEM SOURCE_SIGNATURE',2)
 def test_foreign_duplicate_or_stale_response_is_never_reused(self):
  reply=wire_reply('@AUTHORITY STATUS')
  for bad in [wire_reply('@SYSTEM SOURCE_SIGNATURE')+reply,reply.splitlines()[0]+'\n'+reply,reply+reply.splitlines()[0]+'\n']:
   r,_,_=reader({'@AUTHORITY STATUS':[(.1,bad.encode())]})
   with self.assertRaises(RuntimeError):r.read('@AUTHORITY STATUS')
  r,w,a=reader({'@AUTHORITY STATUS':[]},[(0,reply.encode())])
  with self.assertRaises(RuntimeError):r.read('@AUTHORITY STATUS')
  self.assertFalse(w.writes);self.assertIn(reply,''.join(a))
 def test_authority_and_calibration_mutations_are_denied(self):
  cases=[('@AUTHORITY STATUS','owner=NONE','owner=MOTION'),('@CALIBRATION STATUS','held=NO','held=YES'),('@CALIBRATION STATUS','active=NO','active=YES'),('@CALIBRATION PERSIST STATUS','ACKNOWLEDGED_GENERATION=1','ACKNOWLEDGED_GENERATION=2'),('@CALIBRATION PERSIST STATUS','ACKNOWLEDGED_RECORD_INTACT=1','ACKNOWLEDGED_RECORD_INTACT=0'),('@CALIBRATION PERSIST STATUS','WRITE_STATE=OPEN','WRITE_STATE=CLOSED'),('@CALIBRATION PERSIST STATUS','ACK_UNCERTAIN=0','ACK_UNCERTAIN=1'),('@CALIBRATION PERSIST STATUS','NVS=READY','NVS=ERROR'),('@CALIBRATION PERSIST STATUS','DEGRADED=0','DEGRADED=1')]
  for c,a,b in cases:
   with self.subTest(command=c,field=a):
    r,_,_=reader({c:[(.1,wire_reply(c).replace(a,b).encode())]})
    with self.assertRaises(RuntimeError):r.read(c)
 def test_preflight_requires_start_all_12_rows_and_final_availability(self):
  text=wire_reply('@SERVO PREFLIGHT')
  for bad in [text.replace('SERVO_PREFLIGHT=STARTED joints=12 profile=ROBOT_POWERED\n',''),text.replace('observed_bus_id=11','observed_bus_id=12'),text.replace('torque_enable=0','torque_enable=1',1),text.replace('expected_bus_id=12','expected_bus_id=11').replace('observed_bus_id=12','observed_bus_id=11'),text.replace('evaluated=12','evaluated=11'),'\n'.join(text.splitlines()[:-1])+'\n']:
   r,_,_=reader({'@SERVO PREFLIGHT':[(.1,bad.encode())]})
   with self.assertRaises(RuntimeError):r.read('@SERVO PREFLIGHT',2)
 def test_thirteen_servo_reads_enforce_requested_id_off_and_all_fields(self):
  for i in p.IDS:
   c=f'@SERVO READ {i}';text=wire_reply(c)
   for bad in [text.replace(f'id={i}',f'id={i+1}'),text.replace('torque=0','torque=1'),text.replace('position=100','position=4096'),text.replace(' current=0',''),f'SERVO_READ id={i} result=NO_RESPONSE\n']:
    r,_,_=reader({c:[(.1,bad.encode())]})
    with self.assertRaises(RuntimeError):r.read(c,2)
 def test_no_reset_input_or_unbounded_wait_and_raw_error_retained(self):
  r,w,a=reader({'@STATUS':[(.1,b'ERROR=USAGE\n')]})
  with self.assertRaises(RuntimeError):r.read('@STATUS')
  self.assertIn('ERROR=USAGE',''.join(a));self.assertFalse(hasattr(w,'reset_input_buffer'))
  for timeout in (0,61):
   with self.assertRaises(RuntimeError):r.read('@STATUS',timeout)
  with self.assertRaises(RuntimeError):r.read('@CALIBRATION RESTORE 1')

class RomBackupTests(unittest.TestCase):
 def test_canonical_flasher_has_one_rom_write_and_independent_rom_verify(self):
  text=(FW/'scripts/flash_app_only.sh').read_text();invocations=[line for line in text.splitlines() if '"$ESPTOOL" --chip' in line]
  self.assertEqual(len(invocations),3);self.assertEqual(sum(' write-flash ' in line for line in invocations),1);self.assertEqual(sum(' verify-flash ' in line for line in invocations),1)
  for line in invocations:
   for flags in ('--no-stub','--baud 115200','--connect-attempts 1','--before no-reset','--after no-reset'):self.assertIn(flags,line)
  self.assertNotIn('sleep 2',text);self.assertIn('POST_FLASH_STATE = ROM',text)
 def test_partition_reader_rom_flags_deadline_and_exact_length(self):
  with tempfile.TemporaryDirectory() as temp:
   output=Path(temp)/'partition.bin';output.write_bytes(bytes(4096))
   with patch.object(partition_reader.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'','')) as run:
    partition_reader.esptool_read('not-executed','esp32s3','not-opened',0x8000,4096,output)
    command=run.call_args.args[0]
    for flag in ('--no-stub','--baud','--connect-attempts','--before','--after'):self.assertIn(flag,command)
    self.assertEqual(command[command.index('--baud')+1],'115200');self.assertEqual(command[command.index('--before')+1],'no-reset');self.assertEqual(command[command.index('--after')+1],'no-reset');self.assertEqual(run.call_args.kwargs['timeout'],120)
    output.write_bytes(bytes(4095))
    with self.assertRaises(SystemExit):partition_reader.esptool_read('not-executed','esp32s3','not-opened',0x8000,4096,output)
 def test_rom_method_deadline_and_single_connect_are_pinned(self):
  command=p.rom_command('/not/executed/esptool','/not/opened/port','read-flash',['0x0',str(p.SIZE),'new.bin'])
  self.assertEqual(command[command.index('--baud')+1],'115200');self.assertIn('--no-stub',command)
  self.assertEqual(command[command.index('--connect-attempts')+1],'1');self.assertEqual(command[command.index('--before')+1],'no-reset');self.assertEqual(command[command.index('--after')+1],'no-reset');self.assertGreaterEqual(p.ROM_READ_TIMEOUT,2700)
  for operation in ('write-flash','erase-flash','run'):
   with self.assertRaises(RuntimeError):p.rom_command('tool','port',operation,[])
  with self.assertRaises(RuntimeError):p.rom_command('tool','port','read-flash',[],'usb-reset')
 def test_progress_is_live_and_existing_log_is_preserved(self):
  with tempfile.TemporaryDirectory() as temp:
   log=Path(temp)/'read.log';errors=[]
   def run():
    try:p.run_logged([sys.executable,'-c','import time; print("READ_PROGRESS",flush=True); time.sleep(.5); print("DONE",flush=True)'],log,2)
    except BaseException as e:errors.append(e)
   thread=threading.Thread(target=run);thread.start();deadline=time.monotonic()+1
   while time.monotonic()<deadline and (not log.exists() or b'READ_PROGRESS' not in log.read_bytes()):time.sleep(.01)
   self.assertIn(b'READ_PROGRESS',log.read_bytes());self.assertTrue(thread.is_alive());thread.join(3);self.assertFalse(thread.is_alive());self.assertFalse(errors);self.assertIn(b'DONE',log.read_bytes())
   before=log.read_bytes()
   with self.assertRaises(FileExistsError):p.run_logged([sys.executable,'-c','raise SystemExit(99)'],log,1)
   self.assertEqual(log.read_bytes(),before)
 def test_failed_or_timed_out_read_preserves_partial_log_without_retry(self):
  with tempfile.TemporaryDirectory() as temp:
   for name,code,timeout in [('failed','print("READ_6_PERCENT",flush=True); raise SystemExit(7)',2),('timeout','import time; print("READ_PARTIAL",flush=True); time.sleep(5)',.2)]:
    log=Path(temp)/(name+'.log')
    with self.assertRaises(RuntimeError):p.run_logged([sys.executable,'-c',code],log,timeout)
    self.assertIn(b'READ_',log.read_bytes());self.assertIn(b'ABORTED_NO_RETRY',log.read_bytes());self.assertEqual(log.read_bytes().count(b'"started_utc"'),1)
 def test_rom_resume_requires_complete_old_diagnostics_without_claiming_freshness(self):
  with tempfile.TemporaryDirectory() as temp:
   root=Path(temp);report=root/'preflight_report.json';log=root/'usb_readonly.log';commands=[*p.READS,*[f'@SERVO READ {i}' for i in p.IDS]]
   transcript=''.join(c+'\n'+wire_reply(c) for c in commands);log.write_text(transcript)
   designation={'application_sha256':'a'*64,'installed_historical_source_commit':'b08322dedc5564c080f691dc98047511ef2b953f'}
   data={'mode':'EXECUTED_READONLY','session_directory':str(root),'flash_executed':False,'motion_executed':False,'gates':{'diagnostics_received':True,'servo_persistent_profile':True,'13_servo_readbacks_off':True},'candidate':designation,'installed_source_commit':designation['installed_historical_source_commit'],'error':'read-flash timed out'};report.write_text(json.dumps(data))
   previous,path=p.resume_diagnostics(report,designation);self.assertEqual(path,log);self.assertEqual(previous,data)
   for bad in [transcript.replace('torque=0','torque=1',1),transcript.replace('ACKNOWLEDGED_GENERATION=1','ACKNOWLEDGED_GENERATION=2'),transcript.replace('@SERVO READ 51\n',''),transcript+wire_reply('@AUTHORITY STATUS')]:
    log.write_text(bad)
    with self.assertRaises(RuntimeError):p.resume_diagnostics(report,designation)
 def test_physical_supervision_and_continuity_cannot_be_inferred(self):
  resumed={'diagnostic_evidence':{'origin':'PREVIOUS_PRE_ROM_CAPTURE_NOT_FRESH'}}
  for report,review in [({},{}),(resumed,{'operator_physically_present_with_emergency_stop':True}),(resumed,{'operator_physically_present_with_emergency_stop':True,'hardware_continuity_since_diagnostics_verified':False})]:
   with self.assertRaises(RuntimeError):review_preflash.review_physical_conditions(report,review)
  review_preflash.review_physical_conditions(resumed,{'operator_physically_present_with_emergency_stop':True,'hardware_continuity_since_diagnostics_verified':True})

class ProvenanceTests(unittest.TestCase):
 def test_frozen_qualification_rejects_failed_receipts_or_changed_source_and_logs(self):
  with tempfile.TemporaryDirectory(prefix='matdog-frozen-qualification-') as temp:
   root=Path(temp);report=root/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13';report.mkdir(parents=True)
   name='05_Firmware/MATDOG_Controller/src/core/qualified.cpp';path=root/name;path.parent.mkdir(parents=True);path.write_text('qualified source\n')
   subprocess.run(['git','-C',str(root),'init','-q'],check=True);subprocess.run(['git','-C',str(root),'add','.'],check=True);subprocess.run(['git','-C',str(root),'-c','user.name=Offline test','-c','user.email=offline@example.invalid','commit','-qm','fixture'],check=True)
   source=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
   log=report/'audit.log';log.write_text('STATIC_AUDIT = PASS\n');digest=hashlib.sha256(log.read_bytes()).hexdigest()
   tests={'status':'PASS','source_files':{name:hashlib.sha256(path.read_bytes()).hexdigest()},'tests':[{'name':'full_static_and_host_led_network_nvs_startup','status':'PASS','log':log.name,'log_sha256':digest}]}
   def receipts():
    (report/'TEST_RECEIPT.json').write_text(json.dumps(tests));(report/'BUILD_RECEIPT.json').write_text(json.dumps({'status':'PASS','source_commit':source,'test_receipt_sha256':hashlib.sha256((report/'TEST_RECEIPT.json').read_bytes()).hexdigest()}))
   receipts()
   entry={'result':'PASS','log':log.name,'log_sha256':digest}
   repair={'tests':{**entry,'targeted_app_only_audit':{**entry,'checks':4},'frozen_qualification_gate':entry}}
   def host(): (report/'PREFLIGHT_SERIAL_FIX.json').write_text(json.dumps(repair))
   host();self.assertIn('STATIC_AUDIT = PASS',provenance.verify_frozen_qualification(root,source))
   for changed in ['qualified source modified\n','']:
    path.write_text(changed)
    with self.assertRaises(ValueError):provenance.verify_frozen_qualification(root,source)
   path.write_text('qualified source\n');log.write_text('STATIC_AUDIT = FAIL\n')
   with self.assertRaises(ValueError):provenance.verify_frozen_qualification(root,source)
   log.write_text('STATIC_AUDIT = PASS\n');tests['tests'][0]['status']='FAIL';receipts()
   with self.assertRaises(ValueError):provenance.verify_frozen_qualification(root,source)
   tests['tests'][0]['status']='PASS';receipts();repair['tests']['frozen_qualification_gate']['result']='BLOCKED';host()
   with self.assertRaises(ValueError):provenance.verify_frozen_qualification(root,source)
   repair['tests']['frozen_qualification_gate']['result']='PASS';host();tests['source_files'][name]='0'*64;receipts()
   with self.assertRaises(ValueError):provenance.verify_frozen_qualification(root,source)
 def test_host_patch_receipt_is_scoped_to_exact_old_new_bytes_and_image(self):
  with tempfile.TemporaryDirectory(prefix='matdog-host-receipt-') as temp:
   root=Path(temp);names=provenance.HOST_SERIAL_PATCH_FILES;image='a'*64
   def git(*args):return subprocess.check_output(['git','-C',str(root),*args],text=True).strip()
   git('init','-q')
   for name in names:
    path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'original host tool\n')
   git('add','.')
   git('-c','user.name=Offline test','-c','user.email=offline@example.invalid','commit','-qm','fixture')
   source=git('rev-parse','HEAD');files={}
   for name in names:
    (root/name).write_bytes(b'repaired host tool\n')
    files[name]={'compiled_source_file_sha256':hashlib.sha256(b'original host tool\n').hexdigest(),'current_file_sha256':hashlib.sha256((root/name).read_bytes()).hexdigest()}
   receipt=root/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13/PREFLIGHT_SERIAL_FIX.json';receipt.parent.mkdir(parents=True)
   data={'purpose':'PREFLIGHT_SERIAL_RESPONSE_FRAMING_FIX','status':'PASS','compiled_source_commit':source,'application_sha256':image,'files':files}
   def write(d):receipt.write_text(json.dumps(d))
   write(data);provenance.verify_host_serial_patch(root,source,image,names)
   for key,value in [('status','BLOCKED'),('compiled_source_commit','0'*40),('application_sha256','b'*64),('purpose','GENERIC_OVERRIDE')]:
    bad=json.loads(json.dumps(data));bad[key]=value;write(bad)
    with self.assertRaises(ValueError):provenance.verify_host_serial_patch(root,source,image,names)
   write(data)
   for changed in [names[:-1],(*names,'05_Firmware/MATDOG_Controller/src/core/CommandRouter.cpp'),(*names,'06_Software/Matdog_Core/motion/paired_prone/route.cpp')]:
    with self.assertRaises(ValueError):provenance.verify_host_serial_patch(root,source,image,changed)
   for key in ('compiled_source_file_sha256','current_file_sha256'):
    bad=json.loads(json.dumps(data));bad['files'][names[0]][key]='0'*64;write(bad)
    with self.assertRaises(ValueError):provenance.verify_host_serial_patch(root,source,image,names)
   write(data);(root/names[0]).write_bytes(b'additional unreviewed tool change\n')
   with self.assertRaises(ValueError):provenance.verify_host_serial_patch(root,source,image,names)

if __name__=='__main__':unittest.main()
