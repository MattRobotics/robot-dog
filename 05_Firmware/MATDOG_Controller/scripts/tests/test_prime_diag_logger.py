#!/usr/bin/env python3
"""Offline USB simulator and syscall doubles. Never opens a serial device."""
import base64
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import subprocess
import unittest
from unittest.mock import patch

here = Path(__file__).resolve().parent
module_path = here / 'prime_diag_logger.py'
if not module_path.exists():
    module_path = here.parent / 'release/prime_diag_logger.py'
spec = importlib.util.spec_from_file_location('prime_diag_logger', module_path)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def record(slot=0, attempt=1, field=41, invalid=None):
    target = 2018 + slot
    payload = bytearray([0, 5, target & 255, target >> 8, 0, 0, 20, 0, 200, 0])
    payload[field-40] ^= 1
    p = bytearray([255, 255, m.IDS[slot], 12, 0]) + payload
    p += bytes([(~sum(p[2:])) & 255])
    if invalid == 'BAD_CHECKSUM': p[-1] ^= 1
    if invalid == 'WRONG_ID': p[2] = 99
    if invalid == 'WRONG_LENGTH': p[3] = 11
    if invalid == 'SERVO_STATUS': p[4] = 1
    if invalid == 'BAD_HEADER': p[0] = 0
    if invalid == 'SHORT_PACKET': p = p[:-1]
    if invalid == 'NO_RESPONSE': p = bytearray()
    values = (int.from_bytes(p[7:9], 'little'), int.from_bytes(p[11:13], 'little'), p[6]) if len(p) >= 15 else (-1, -1, -1)
    valid = invalid is None
    d = dict(index=0,slot=slot,id=m.IDS[slot],attempt=attempt,reason='BACKEND_REJECTED' if valid else 'UNCERTAIN_REQUIRES_SAFE_OFF',session_ms=100,
             target_position=target,capture=1,target_speed=20,target_acc=5,ack=1,
             write_start_ms=100,write_done_ms=101,read_start_ms=101,read_done_ms=102,done_ms=102,
             address=40,width=10,rx_size=len(p),drained=0,validity=invalid or 'VALID',decoded_valid=int(valid),
             position_read=values[0] if valid else -1,speed_read=values[1] if valid else -1,acc_read=values[2] if valid else -1,
             raw_payload=int(len(p)>=15),raw_position=values[0],raw_speed=values[1],raw_acc=values[2],
             mismatch=hex({42:1,46:2,41:4}[field] if valid else 0),rx=p.hex().upper())
    return d


def line(d): return 'PRIME_RECORD ' + ' '.join(k+'='+str(v) for k,v in d.items())


def frame():
    return dict(boot_nonce=123,now_ms=100,executor_state='OFF',fault=False,
        local_session={'state':'OFF'},restore={'generation':1,'fingerprint':'be8d5902bb4e37da'},
        commissioning={'restore':'RESTORED'},operational={'system_health':'READY'},
        channels={'raw':{'count':12},'mapped':{'count':12},'imu':{'valid':True}},
        servos=[dict(id=id,mapped=True,energized=0,speed=0,status=0,age_ms=10) for id in m.IDS],
        runtime=dict(heap_free=50000,heap_min_free=45000,loop_stack_min_free=8000))


class USB:
    def __init__(self, records=None, fault=None):
        self.records = records if records is not None else [record()]
        for i,r in enumerate(self.records):r['index']=i
        self.fault=fault; self.queue=[];self.now=0;self.commands=[];self.started=False
        self.stopped=False;self.closed=False;self.polls=0;self.chunks=[];self.counter=0
        self.add(' MATDOG Controller 0.2.0-dev.5\n')
    def add(self, text):
        data=text.encode();self.queue.extend([data[:3],data[3:17],data[17:]])
        self.queue=[x for x in self.queue if x]
    def clock(self): return self.now
    def read(self, timeout):
        self.now += min(timeout,.05)
        if self.fault=='interrupt' and self.started and not self.stopped:
            self.fault=None;raise KeyboardInterrupt('simulated operator STOP')
        if self.queue:
            chunk=self.queue.pop(0);self.chunks.append(chunk);return chunk
        return b''
    def write(self, data):
        c=data.decode().rstrip('\n');self.commands.append(c)
        if self.fault=='short_write' and ' START ' in c:return 5
        if c in ('@MOTION TELEMETRY 0','@IMU STREAM OFF','@BMS STREAM OFF'):
            self.add({'@MOTION TELEMETRY 0':'TELEMETRY=OFF','@IMU STREAM OFF':'IMU_STREAM=OFF','@BMS STREAM OFF':'BMS_STREAM=OFF'}[c]+'\n')
        elif c=='@SYSTEM SOURCE_SIGNATURE':
            source='0'*40 if self.fault=='source' else m.SOURCE
            self.add(f'SOURCE_SIGNATURE build_id={m.SOURCE[:12]} profile=ROBOT_POWERED\n GIT_SHA={source} GIT_DIRTY=NO HARDWARE_PROFILE=ROBOT_POWERED\n FLASH_LAYOUT=MATDOG_16M_2x5M_NVS_V1 OTA_INGEST=0\n partition=app0 address=0x010000 size=0x500000\n')
        elif c=='@STATUS':
            self.add('SERVO_POP canonical=17 expected_now=13 absent_by_design=4 last_census='+('FAIL' if self.fault=='census' else 'PASS')+'\n')
        elif c=='@MOTION TELEMETRY 2':
            f=frame();self.counter+=1;f['now_ms']+=self.counter
            if self.fault=='torque':f['servos'][0]['energized']=1
            if self.fault=='stale':f['servos'][11]['age_ms']=401
            if self.fault=='restore':f['restore']['fingerprint']='bad'
            if self.fault=='reset_after' and self.started:f['boot_nonce']=999
            self.add('TELEMETRY=ON\n@TEL '+json.dumps(f)+'\n')
        elif c=='@MOTION VALIDATION SENSORS':
            self.add('SENSORS uptime_ms=100 imu_valid=1 imu_stamp_ms=99 bms_valid=1 bms_stamp_ms=90 power=RUN fault=0 boot_safe_off_pending=0\n')
        elif c==m.STATUS:
            self.polls+=1
            session='OFF';reason='IDLE';used=0;count=0
            if self.started:
                session='PRIMING' if not self.stopped and (self.polls<4 or self.fault=='safeoff_timeout') else 'OFF'
                reason='SAFE_OFF_VERIFIED' if session=='OFF' else 'PRIMING_PRESENT'
                used=1;count=len(self.records)
            if self.fault=='energize' and self.started:session='ENERGIZING'
            nonce='bad' if self.fault=='nonce' else '42'
            overflow='1' if self.fault=='overflow' and self.started else '0'
            revoke='PRIME_BACKEND_NOT_APPLIED' if self.records else 'PRIME_DIAGNOSTIC_COMPLETE_NO_ENERGIZE'
            self.add(f'PRIME_DIAG mode=1 used={used} records={count} capacity=36 overflow={overflow} session={session} reason={reason} last_revoke={revoke} torque_inhibit=1 nonce={nonce}\n')
        elif ' PRIME_DIAG START ' in c:
            self.started=True
            if self.fault=='start_timeout':pass
            elif self.fault=='start_refused':self.add('VALIDATION=REFUSED REASON=STALE_FEEDBACK\n')
            else:self.add('VALIDATION=PASS REASON=PRIME_DIAGNOSTIC_STARTED_NO_ENERGIZE\n')
        elif ' PRIME_DIAG READ ' in c:
            i=int(c.split()[-1]);r=copy.deepcopy(self.records[i])
            if self.fault=='record_index':r['index']=99
            if self.fault=='record_decode':r['position_read']=0
            if self.fault=='record_raw':r['rx']='FF'
            text=line(r)+'\n'
            if self.fault=='partial':text=text[:-7]
            self.add(text)
        elif c==m.STOP:
            self.stopped=True;self.add('VALIDATION=PASS REASON=SAFE_OFF_REQUESTED\n')
        else:raise AssertionError('unexpected command '+c)
        return len(data)
    def close(self):self.closed=True


class Tests(unittest.TestCase):
    def run_case(self, usb, fail=False):
        with tempfile.TemporaryDirectory() as d:
            journal=m.Journal(Path(d)/'session');acq=m.Acquisition(usb,journal,usb.clock)
            if fail:
                with self.assertRaises(m.Refusal):acq.run()
            else:self.assertEqual(acq.run()['status'],'PASS_ACQUISITION')
            saved=json.loads((Path(d)/'session/receipt.json').read_text())
            events=[json.loads(x) for x in journal.path.read_text().splitlines()]
            rx=b''.join(base64.b64decode(e['base64']) for e in events if e['kind']=='RX')
            self.assertEqual(rx,b''.join(usb.chunks))
            self.assertTrue(all(e['utc'] and e['monotonic_ns'] for e in events))
            digest=json.loads((Path(d)/'session/log-digest.json').read_text())['sha256']
            self.assertEqual(digest,m.hashlib.sha256(journal.path.read_bytes()).hexdigest())
            self.assertTrue(usb.closed)
            self.assertLessEqual(sum(' START ' in c for c in usb.commands),1)
            self.assertLessEqual(usb.commands.count(m.STOP),1)
            self.assertFalse(any('CLEAR' in c or 'STAND' in c or 'WALK' in c for c in usb.commands))
            return saved
    def test_complete_prime_without_failure(self):
        self.assertEqual(self.run_case(USB([]))['prime_result'],'COMPLETE_NO_ENERGIZE')
    def test_status_polling_is_paced_with_immediate_replies(self):
        class Immediate(USB):
            def read(self, timeout):
                if self.queue:
                    chunk=self.queue.pop(0);self.chunks.append(chunk);return chunk
                self.now+=timeout;return b''
        usb=Immediate([]);self.run_case(usb)
        self.assertGreaterEqual(usb.now,.2)
    def test_every_slot_field_and_attempt(self):
        for slot in range(12):
            for field in (41,42,46):
                with self.subTest(slot=slot,field=field):
                    r=self.run_case(USB([record(slot,i,field) for i in (1,2,3)]))
                    self.assertEqual(len(r['records']),3)
    def test_all_twelve_transient_failures(self):
        self.assertEqual(len(self.run_case(USB([record(i) for i in range(12)]))['records']),12)
    def test_original_invalid_packets_preserved(self):
        for invalid in ('NO_RESPONSE','SHORT_PACKET','BAD_HEADER','WRONG_ID','WRONG_LENGTH','SERVO_STATUS','BAD_CHECKSUM'):
            with self.subTest(invalid=invalid):
                r=self.run_case(USB([record(invalid=invalid)]))
                self.assertEqual(r['records'][0]['validity'],invalid)
    def test_gate_refusals_never_start(self):
        for fault in ('source','nonce','torque','stale','restore','census'):
            with self.subTest(fault=fault):
                usb=USB(fault=fault);r=self.run_case(usb,True)
                self.assertEqual(r['start_commands'],0)
    def test_stop_and_persist_on_timeout_or_interrupt(self):
        for fault in ('start_timeout','start_refused','safeoff_timeout','interrupt'):
            with self.subTest(fault=fault):
                usb=USB(fault=fault);r=self.run_case(usb,True)
                self.assertEqual(usb.commands.count(m.STOP),1)
                self.assertEqual(len(r['records']),1)
    def test_incomplete_or_corrupt_export_is_failure_no_retry(self):
        for fault in ('partial','record_index','record_decode','record_raw'):
            with self.subTest(fault=fault):
                usb=USB(fault=fault);r=self.run_case(usb,True)
                self.assertEqual(sum(' READ ' in c for c in usb.commands),1)
                self.assertEqual(r['status'],'FAIL')
    def test_reset_overflow_energize_short_write(self):
        for fault in ('reset_after','overflow','energize','short_write'):
            with self.subTest(fault=fault):self.run_case(USB(fault=fault),True)
    def test_whole_record_capacity(self):
        records=[record(slot,attempt) for slot in range(12) for attempt in (1,2,3)]
        self.assertEqual(len(self.run_case(USB(records))['records']),36)
    def test_duplicate_attempt_refused(self):
        self.run_case(USB([record(),record()]),True)
    def test_command_allowlist(self):
        with tempfile.TemporaryDirectory() as d:
            j=m.Journal(Path(d)/'session');u=USB();a=m.Acquisition(u,j,u.clock)
            for c in ('@MOTION STAND','@MOTION WALK','@BMS KEY OFF','@MOTION VALIDATION PRIME_DIAG CLEAR 42 CONFIRM_CLEAR_RAM_EVIDENCE'):
                with self.assertRaises(m.Refusal):a.send(c)
            self.assertFalse(u.commands);j.close()
    def test_raw_usb_single_open_and_no_modem_operations(self):
        import termios
        attrs=[0,0,termios.HUPCL,0,0,0,[0]*32]
        with patch('os.open',return_value=7) as op,patch('os.close') as close,patch('fcntl.ioctl') as ioctl,patch('termios.tcgetattr',return_value=attrs),patch('termios.tcsetattr') as setattrs:
            usb=m.RawUSB('/offline/fixture');usb.close();usb.close()
            op.assert_called_once();close.assert_called_once_with(7)
            ioctl.assert_called_once_with(7,termios.TIOCEXCL)
            self.assertFalse(setattrs.call_args.args[2][2]&termios.HUPCL)
    def test_transport_eof_and_partial_tx_fail(self):
        with patch('select.select',return_value=([7],[],[])),patch('os.read',return_value=b''):
            usb=object.__new__(m.RawUSB);usb.fd=7
            with self.assertRaises(m.Refusal):usb.read(.1)
    def test_invalid_physical_setup_never_opens_usb(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'physical.json'
            for flags in ({},dict(fixture=True,operator_present=True,key_on=False,cutoff_accessible=True),
                          dict(fixture=True,operator_present=True,key_on=True,cutoff_accessible=True,utc='2000-01-01T00:00:00+00:00')):
                p.write_text(json.dumps(flags))
                with patch('sys.argv',['logger','--output',td+'/session','--physical-check',str(p)]),patch.object(m,'RawUSB') as raw:
                    with self.assertRaises(m.Refusal):m.main()
                    raw.assert_not_called()
    def test_record_required_fields_and_integrity(self):
        base=record()
        for field in list(base):
            bad=copy.deepcopy(base);del bad[field]
            with self.subTest(field=field),self.assertRaises(m.Refusal):m.validate_record(line(bad),0)
    def test_actual_production_formatter_all_slots(self):
        fw=Path(os.environ.get('MATDOG_FW',str(here.parents[1])))
        source=r'''
#include "diagnostics/PrimeDiagnostic.h"
#include <cstdio>
using namespace matdog::diagnostics;
int main(){unsigned ids[]={13,12,11,23,22,21,33,32,31,43,42,41};
for(unsigned slot=0;slot<12;++slot)for(unsigned attempt=1;attempt<=3;++attempt){
 PrimeRejection r;r.slot=slot;r.id=ids[slot];r.attempt=attempt;r.reason="BACKEND_REJECTED";r.target_position=2018+slot;
 auto& g=r.goal;g.attempted=true;g.id=r.id;g.position=r.target_position;g.speed=20;g.acc=5;g.ack=1;
 auto& p=g.response;p.id=r.id;p.address=40;p.width=10;p.size=16;p.validity=ReadValidity::VALID;
 p.bytes[0]=p.bytes[1]=255;p.bytes[2]=r.id;p.bytes[3]=12;p.bytes[6]=4;p.bytes[7]=g.position&255;p.bytes[8]=g.position>>8;p.bytes[11]=20;
 unsigned sum=0;for(unsigned i=2;i<15;++i)sum+=p.bytes[i];p.bytes[15]=~sum;
 g.position_read=g.position;g.speed_read=20;g.acc_read=4;g.mismatch=4;
 char out[768];if(!formatPrimeRejection(r,slot*3+attempt-1,out,sizeof(out)))return 1;std::puts(out);
}}
'''
        with tempfile.TemporaryDirectory() as d:
            cpp=Path(d)/'fixture.cpp';exe=Path(d)/'fixture';cpp.write_text(source)
            result=subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(fw/'src'),str(cpp),str(fw/'src/diagnostics/PrimeDiagnostic.cpp'),'-o',str(exe)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            lines=subprocess.check_output([str(exe)],text=True).splitlines()
            self.assertEqual(len(lines),36)
            for i,text in enumerate(lines):self.assertEqual(m.validate_record(text,i)['mismatch'],'0x04')
        base=record()
        with self.assertRaises(m.Refusal):m.validate_record(line(base)+' id=13',0)
        for field,value in [('drained',65),('write_start_ms',-1),('slot',12),('attempt',4),('decoded_valid',0),('raw_acc',999)]:
            bad=copy.deepcopy(base);bad[field]=value
            with self.subTest(field=field),self.assertRaises(m.Refusal):m.validate_record(line(bad),0)


if __name__=='__main__':unittest.main(verbosity=2)
