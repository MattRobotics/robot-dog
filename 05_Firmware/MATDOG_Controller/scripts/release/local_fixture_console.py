#!/usr/bin/env python3
"""PLAN by default. Future physically-present fixture operator console.
No autonomous segment sequence. SPACE held -> presence; F1 explicit grant,
F2 STAND, F3 ONE NEXT, F4 qualified first stride, F5 explicit finite renewal.
ESC/release/exit -> STOP. No NVS/EEPROM/BMS/flash command exists.
"""
import argparse,os,json,sys,time,select,struct,re,subprocess
from pathlib import Path
PORT='/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00'
KEY_SPACE=57;F1,F2,F3,F4,F5,ESC=59,60,61,62,63,1
EVENT=struct.Struct('llHHI')
def action(code,nonce,segment,vx):
 prefix='@MOTION VALIDATION '
 return {F1:f'{prefix}GRANT {nonce} 60000 CONFIRM_LOCAL_SUPERVISED_PRIME_HOLDING_OUTPUT_CEILING',F2:f'{prefix}STAND {nonce}',F3:f'{prefix}NEXT {nonce} {segment} CONFIRM_TRACKING_DIRECTIONS_CLEARANCE_POWER_STOP',F4:f'{prefix}WALK_START {nonce} {vx:.17g} 0 0 CONFIRM_SUSPENDED_WALK_PROOF_AND_CLEARANCE',F5:f'{prefix}RENEW {nonce} 60000 CONFIRM_CONTINUED_LOCAL_SUPERVISION_AND_GATES'} .get(code)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute-local',action='store_true');p.add_argument('--confirm-present-fixture-power-cut-reviewed-profile',action='store_true');p.add_argument('--port',default=PORT);p.add_argument('--keyboard-event',type=Path);p.add_argument('--log',type=Path);a=p.parse_args()
 if not a.execute_local:print(json.dumps(dict(mode='PLAN_ONLY',requires='physical ASUS USB keyboard, held SPACE, previously loaded reviewed RAM profile + measured fixture POSE + RUN stable',keys='F1 GRANT; F2 STAND; F3 ONE NEXT; F4 WALK; F5 RENEW; ESC/release STOP',no_auto_grant=True,no_flash=True)));return 0
 if not a.confirm_present_fixture_power_cut_reviewed_profile or not sys.stdin.isatty() or os.getenv('SSH_CONNECTION') or not a.keyboard_event or not a.log:raise SystemExit('Physical console, explicit fixture review, real USB keyboard and private log required; SSH refused')
 facts=subprocess.check_output(['udevadm','info','--query=property','--name',str(a.keyboard_event)],text=True)
 if 'ID_INPUT_KEYBOARD=1' not in facts or 'ID_BUS=usb' not in facts:raise SystemExit('USB keyboard identity not proven')
 import serial
 os.umask(0o077);a.log.parent.mkdir(parents=True,exist_ok=True);log=a.log.open('x');s=serial.Serial();s.port=a.port;s.baudrate=115200;s.timeout=0;s.dtr=s.rts=False;s.open();fd=os.open(a.keyboard_event,os.O_RDONLY|os.O_NONBLOCK)
 held=walk=walk_seen=False;nonce=segment=uptime=0;clock_seen=0;last_presence=last_walk=last_status=0;buf='';vx=.005/(90*9)
 def send(c):
  prefix='@MOTION VALIDATION '
  if c.startswith(prefix) and c[len(prefix):].split()[0] not in ('STATUS','STOP'):
   if not clock_seen or time.monotonic()-clock_seen>.35:return
   c=prefix+'AT '+str(int(uptime+1000*(time.monotonic()-clock_seen)))+' '+c[len(prefix):]
  s.write((c+'\n').encode());log.write(f'{time.monotonic():.6f} TX {c}\n');log.flush()
 print('No grant sent. Hold SPACE and deliberately press F1 only after live gate review. F3 approves one segment. Release SPACE -> STOP.')
 try:
  while True:
   now=time.monotonic()
   if now-last_status>.2:send('@MOTION VALIDATION STATUS');last_status=now
   data=s.read(8192).decode('utf-8','replace')
   if data:
    log.write(f'{now:.6f} RX '+data);log.flush();buf=(buf+data)[-8192:]
    while '\n' in buf:
     line,buf=buf.split('\n',1);m=re.search(r'SEGMENT=(\d+) NONCE=(\d+)',line)
     if m:segment,nonce=map(int,m.groups())
     stamp=re.search(r'UPTIME_MS=(\d+)',line)
     if stamp:uptime=int(stamp[1]);clock_seen=now
     speed=re.search(r'WALK_VX=([0-9eE.+-]+)',line)
     if speed:vx=float(speed[1])
     planner=re.search(r'PLANNER=([A-Z_]+)',line)
     if planner and walk:
      if planner[1] in ('WALK_STARTING','WALK'):walk_seen=True
      elif planner[1]=='STAND' and walk_seen:walk=False;walk_seen=False;send('@MOTION VALIDATION STOP')
     if 'REFUSED' in line or 'SAFE_OFF' in line:walk=False
     if 'SESSION=OFF' in line:walk=False
     if 'VALIDATION=' in line:print(line)
   if select.select([fd],[],[],.01)[0]:
    events=os.read(fd,EVENT.size*64)
    for i in range(0,len(events)-EVENT.size+1,EVENT.size):
     _,_,kind,code,value=EVENT.unpack(events[i:i+EVENT.size])
     if kind!=1:continue
     if code==ESC and value==1:send('@MOTION VALIDATION STOP');return 0
     if code==KEY_SPACE:
      held=value!=0
      if not held:walk=False;send('@MOTION VALIDATION STOP')
     elif value==1 and held and nonce:
      command=action(code,nonce,segment,vx)
      if command:
       send(command)
       if code==F4:walk=True;walk_seen=False
   if held and nonce and now-last_presence>.05:send(f'@MOTION VALIDATION PRESENT {nonce} CONFIRM_PRESENT_FIXTURE_POWER_CUT_FIXED_POSE_UNCHANGED');last_presence=now
   if held and walk and nonce and now-last_walk>.05:send(action(F4,nonce,segment,vx).replace('WALK_START ','WALK '));last_walk=now
 finally:
  try:send('@MOTION VALIDATION STOP')
  finally:s.close();os.close(fd);log.close()
if __name__=='__main__':sys.exit(main())
