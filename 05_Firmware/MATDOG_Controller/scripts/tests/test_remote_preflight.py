#!/usr/bin/env python3
"""No serial port is opened. Verify plan, native gen-1 decoder and image parser."""
import importlib.util,json,subprocess,sys,tempfile,unittest,hashlib,struct,zlib
from pathlib import Path
FW=Path(__file__).resolve().parents[2];sys.path.insert(0,str(FW/'scripts/release'));sys.path.insert(0,str(FW/'scripts'))
import motion_remote_preflight as p
import review_preflash
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
if __name__=='__main__':unittest.main()
