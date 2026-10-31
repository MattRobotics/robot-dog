#!/usr/bin/env python3
"""Passive Motion recorder. GET only; NEVER sends a command, STOP or flash.
Use instead of an additional dashboard poll. Ctrl-C stops recording only.
--replay-json converts a downloaded dashboard trace entirely offline.
"""
import argparse,csv,hashlib,json,math,re,time,urllib.request
from pathlib import Path
FW=Path(__file__).resolve().parents[2]
GEN=FW/'scripts/offline_prone/gen1_geometry.json'
COLLISION=FW/'src/simple/CollisionData.h'
TICK=2*math.pi/4096
PHASES={'PRONE_HEIGHT':'kHeight','PRONE_LIFT':'kLift','PRONE_DEPLOY':'kDeploy','PRONE_LAND':'kEntry','PRONE_CONTACT_UNVERIFIED':'kEntry','HEIGHT':'kHeight','STAND_LIFT':'kLift','STAND_DEPLOY':'kDeploy','WALK':'kWalk','WALK_SETTLE':'kWalk'}

def limits():
 gen=json.loads(GEN.read_text())['joints'];boxes={}
 for name,low,high in re.findall(r'constexpr Box (\w+)=\{\{([^}]+)\},\{([^}]+)\}\};',COLLISION.read_text()):
  boxes[name]=([float(x) for x in low.split(',')],[float(x) for x in high.split(',')])
 return [j['limits_rad'] for j in gen],boxes

def margin_rows(s,received_ms,rtt_ms):
 bounds,boxes=limits();phase=s.get('phase','');box=boxes.get(PHASES.get(phase))
 if phase=='HOLD':box=boxes.get({'WALK':'kWalk','STAND_HEIGHT':'kHeight','STAND_LIFT':'kLift','STAND_DEPLOY':'kDeploy','PRONE_ENTRY':'kEntry'}.get(s.get('pose_region')))
 angles=s.get('telemetry',{}).get('actual_rad')
 for j in s.get('joints',[]):
  i=j['slot'];fresh=not s.get('stale',True) and j.get('valid') and j.get('age_ms',1e9)+rtt_ms<=250
  q=angles[i] if fresh and isinstance(angles,list) and len(angles)==12 else None
  yield {'received_ms':received_ms,'roundtrip_ms':rtt_ms,'device_ms':s.get('now_ms'),'phase':phase,'state':s.get('state'),'command_id':s.get('active_command',{}).get('id'),'outcome':s.get('active_command',{}).get('outcome'),'slot':i,'id':j.get('id'),'raw':j.get('raw'),'commanded_raw':j.get('commanded_raw'),'error_raw_ticks':j['commanded_raw']-j['raw'] if 'commanded_raw' in j else None,'q_rad':q,'gen1_margin_ticks':min(q-bounds[i][0],bounds[i][1]-q)/TICK if q is not None else None,'box_margin_ticks':min(q-box[0][i],box[1][i]-q)/TICK if q is not None and box else None,'fresh':bool(fresh),'age_ms':j.get('age_ms'),'load_word':j.get('load'),'temperature':j.get('temperature'),'status':j.get('status'),'voltage_dv':j.get('voltage_dv'),'torque_dated':j.get('torque'),'control_age_ms':j.get('control_age_ms')}

def main():
 ap=argparse.ArgumentParser(description=__doc__);source=ap.add_mutually_exclusive_group(required=True);source.add_argument('--url',help='Controller base URL, explicit; future supervised use only');source.add_argument('--replay-json',type=Path);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--duration',type=float,default=0,help='seconds; 0 until Ctrl-C. NEVER sends STOP');a=ap.parse_args()
 a.out.mkdir(parents=True,exist_ok=False)
 (a.out/'metadata.json').write_text(json.dumps({'mode':'replay' if a.replay_json else 'passive_GET','started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'gen1_sha256':hashlib.sha256(GEN.read_bytes()).hexdigest(),'boxes_sha256':hashlib.sha256(COLLISION.read_bytes()).hexdigest(),'period_s':.35,'never_sends_stop':True},indent=2)+'\n')
 writer=None;start=time.monotonic()
 with (a.out/'samples.jsonl').open('w') as log,(a.out/'joints.csv').open('w') as cf:
  def write(sample):
   nonlocal writer
   log.write(json.dumps(sample)+'\n');log.flush()
   for row in margin_rows(sample['status'],sample['received_ms'],sample.get('roundtrip_ms',0)):
    if writer is None:writer=csv.DictWriter(cf,fieldnames=row);writer.writeheader()
    writer.writerow(row)
   cf.flush()
  if a.replay_json:
   for sample in json.loads(a.replay_json.read_text())['samples']:write(sample)
  else:
   print('PASSIVE GET only. No STOP timer. Ctrl-C stops logging, NOT Motion. Use the independent STOP control.')
   url=a.url.rstrip('/')+'/api/motion/status'
   if not url.startswith(('http://','https://')):ap.error('HTTP(S) base URL required')
   try:
    while not a.duration or time.monotonic()-start<a.duration:
     t=time.monotonic()
     try:
      with urllib.request.urlopen(urllib.request.Request(url,headers={'Cache-Control':'no-store'}),timeout=.8) as response:
       status=json.loads(response.read(16384));header=response.headers.get('Server-Timing')
      now=time.monotonic();write({'received_ms':1000*(now-start),'roundtrip_ms':1000*(now-t),'server_timing':header,'status':status})
     except Exception as e:log.write(json.dumps({'error':type(e).__name__,'elapsed_s':time.monotonic()-start})+'\n');log.flush()
     time.sleep(max(0,.35-(time.monotonic()-t)))
   except KeyboardInterrupt:pass
if __name__=='__main__':main()
