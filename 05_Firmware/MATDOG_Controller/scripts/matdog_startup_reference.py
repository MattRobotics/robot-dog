#!/usr/bin/env python3
"""Verify one historical acquisition. Data only; no device or motion interface."""
import argparse
import hashlib
import json
from pathlib import Path
import re

Q0 = {11:2102,12:2107,13:1975,21:1997,22:2106,23:2024,
      31:2036,32:2058,33:2086,41:2079,42:2076,43:2026}
UNITS = {11:'M33',12:'ELR01',13:'M22',21:'NEW03',22:'ELR03',23:'NEW01',
         31:'NEW05',32:'ELR02',33:'NEW06',41:'M41',42:'M42',43:'M43'}
GEOMETRY = '3713f4ddc43b204e'
DIRECTION = {11:-1,12:1,13:-1,21:1,22:-1,23:-1,31:1,32:-1,33:1,41:-1,42:1,43:1}


def classify_positions(positions):
    """Compatibility only. Firmware fresh population proof still required."""
    if set(positions)!=set(Q0): raise ValueError('STARTUP_READBACK_INCOMPLETE')
    if any(type(p) is not int or not 0<=p<4096 for p in positions.values()):
        raise ValueError('STARTUP_READBACK_INVALID')
    q={b:(positions[b]-Q0[b])*DIRECTION[b] for b in Q0}
    if all(-10<=value<=10 for value in q.values()): return 'NOMINAL'
    bands={21:(-10,367),22:(1014,1034),32:(388,408)}
    if all(bands.get(b,(-10,10))[0]<=value<=bands.get(b,(-10,10))[1] for b,value in q.items()):
        return 'RF_LOWER_MAX_RETURN'
    raise ValueError('STARTUP_POSE_UNRECOGNIZED')


def verify(directory):
    directory = Path(directory)
    names = ['hw_session_all_20261003_134852.log','q0_promoted.json',
             'evidence_export_20261003_134852_after_failure.txt']
    raw = {name:(directory/name).read_bytes() for name in names}
    log = raw[names[0]].decode('utf-8')
    export = raw[names[2]].decode('utf-8')
    def require(ok, reason):
        if not ok: raise ValueError(reason)
    def single(pattern):
        matches = list(re.finditer(pattern,log,re.M))
        require(len(matches)==1,'ambiguous/missing event: '+pattern)
        return matches[0]
    stamps = re.findall(r'^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}) ',log,re.M)
    require(stamps == sorted(stamps),'log timestamps are not monotonic')
    require(log.count(' RX SYSTEM_BOOT_COMPLETE ')==1,'boot continuity is ambiguous')
    require(log.count(' SYS PORT_OPEN ')==1 and log.count(' SYS PORT_CLOSED')==1,
            'acquisition connection continuity is ambiguous')
    require(re.search(r'RX SOURCE_SIGNATURE build_id=be0c12979e5b .* profile=ROBOT_POWERED board=YD-ESP32-S3 N16R8',log),
            'source firmware/profile mismatch')
    require('reset_reason : POWERON' in log,'source boot not established')
    start=single(r'TX @CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE')
    complete=single(r'RX CALIBRATION_Q0 state=COMPLETE failure=NONE session=1 sample_passes=9/9 next_joint=0 candidates=12/12')
    promote=single(r'RX CALIBRATION_Q0_PROMOTE=OK admitted=12/12 source=CURRENT_BOOT_CAPTURE capture_session=1')
    lf=single(r'RX CALIBRATION_FULL_LEG_RESULT leg=LF verdict=HARDWARE_CONTACT_CALIBRATED failure=NONE')
    rf=single(r'RX CALIBRATION_FULL_LEG_RESULT leg=RF verdict=FAILED failure=EXECUTOR_FAILED')
    require(start.start()<complete.start()<promote.start()<lf.start()<rf.start(),'acquisition event order mismatch')
    require('RX CALIBRATION_SESSION=ACTIVE leg=LF session=1 ' in log and
            'RX CALIBRATION_SESSION=ACTIVE leg=RF session=2 ' in log,'leg session identity mismatch')
    rows = re.findall(r'RX   Q0 bus=(\d+) leg=(\w+) joint=(\w+) unit=(\w+) tick=(\d+) spread=(\d+) samples=(\d+) state=CANDIDATE estimator=MANUAL_ZERO_POSE',log)
    require(len(rows)==12,'capture does not contain exactly twelve candidates')
    observed={}
    for bus,leg,joint,unit,tick,spread,samples in rows:
        bus=int(bus)
        require(bus not in observed and bus in Q0,'duplicate/unknown servo')
        require(leg=={1:'LF',2:'RF',3:'RH',4:'LH'}[bus//10] and
                joint=={1:'LOWER',2:'UPPER',3:'HIP'}[bus%10] and unit==UNITS[bus],
                'canonical identity mismatch')
        require(int(spread)==0 and int(samples)==9,'capture quality mismatch')
        observed[bus]=int(tick)
    def unique_pairs(pairs):
        out={}
        for key,value in pairs:
            require(key not in out,'duplicate JSON key');out[key]=value
        return out
    original=json.loads(raw[names[1]],object_pairs_hook=unique_pairs)
    require(all(type(value) is int for value in original.values()),'invalid Q0 type')
    require(observed==Q0 and original=={str(k):v for k,v in Q0.items()},'Q0 JSON/capture/acquisition mismatch')
    erows=re.findall(r'CALIBRATION_EVIDENCE_Q0 leg=(\w+) joint=(\w+) unit=(\w+) bus=(\d+) present=1 q0_tick=(\d+) state=PROMOTED origin=LIVE_SESSION geometry=(\w+)',export)
    require(len(erows)==6,'LF/RF export Q0 population mismatch')
    require({int(row[3]) for row in erows}=={11,12,13,21,22,23},'export identity population mismatch')
    for leg,joint,unit,bus,tick,geometry in erows:
        bus=int(bus)
        require(Q0[bus]==int(tick) and UNITS[bus]==unit and geometry==GEOMETRY and
                leg=={1:'LF',2:'RF'}[bus//10] and joint=={1:'LOWER',2:'UPPER',3:'HIP'}[bus%10],
                'export provenance mismatch')
    require('total_contacts_accepted=6 all_contact_calibrated=0' in export,'export completion mismatch')
    require(re.search(r'CALIBRATION_EVIDENCE_LEG leg=LF .* session=1 geometry='+GEOMETRY,export) and
            re.search(r'CALIBRATION_EVIDENCE_LEG leg=RF .* session=2 geometry='+GEOMETRY,export),
            'export session mismatch')
    require('decision=CONFIRMED samples=95,117,34 count=3 published=117 limit=70' in log and
            'failed_phase=LOWER_MAX probe_phase=SAFE_OFF_REQUIRED probe_failure=OVER_TEMPERATURE' in log,
            'RF interruption mismatch')
    require('CALIBRATION_PERSISTENCE_BOOT nvs=READY verdict=NO_RECORD available=0 motion_authorized=0' in log,
            'persistence source mismatch')
    return dict(schema='MATDOG_STARTUP_REFERENCE_V1',reference_status='VERIFIED_12_OF_12',
                source_directory=str(directory.resolve()),source_firmware='be0c12979e5b',geometry=GEOMETRY,
                capture_session=1,lf_session=1,rf_session=2,same_boot=True,
                q0={str(k):v for k,v in Q0.items()},units={str(k):v for k,v in UNITS.items()},
                source_sha256={name:hashlib.sha256(value).hexdigest() for name,value in raw.items()},
                persisted_in_controller=False,automatic_motion_authority=False,hardware_io=False,
                integrity_scope='Cross-correlated local originals; hashes computed offline, not a historical digital signature')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory');ap.add_argument('--output',required=True)
    args=ap.parse_args();report=verify(args.directory)
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print('STARTUP_REFERENCE=VERIFIED_12_OF_12')
