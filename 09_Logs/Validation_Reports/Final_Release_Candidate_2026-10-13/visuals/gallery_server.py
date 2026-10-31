#!/usr/bin/env python3
"""Owned localhost-only visual fixture; no devices, authority or hardware feed."""
import argparse,json,sys
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer,SimpleHTTPRequestHandler
ROOT=Path(__file__).resolve().parents[4];FW=ROOT/'05_Firmware/MATDOG_Controller';sys.path.insert(0,str(FW/'scripts'));import motion_dashboard_demo as demo
p=argparse.ArgumentParser();p.add_argument('--port',type=int,required=True);p.add_argument('--role',choices=('LAN','AP','MOTION','VIEWER'),required=True);p.add_argument('--trace',type=Path);a=p.parse_args()
html=demo.portal_html().replace('<body>','<body><p style="padding:12px;background:#542;font-weight:bold">OFFLINE VISUAL FIXTURE · nessun robot collegato · '+a.role+'</p>')
profile=dict(ssid='',enabled=False,dhcp=True,ip='0.0.0.0',mask='0.0.0.0',gateway='0.0.0.0',dns='0.0.0.0')
w=dict(network_phase='CLIENT_STA' if a.role=='LAN' else 'ADMIN_AP',admin_generation=1,build_id='OFFLINE_VISUAL_FIXTURE',profiles=[profile,profile],scan=[],active_profile=0,sta_test=dict(seq=0),policy=dict(role='MAINTENANCE_STA_READONLY' if a.role=='LAN' else 'MAINTENANCE_AP',run=False),ap_ssid='',ap_ip='192.168.4.1',ap_active=a.role!='LAN',ap_clients=0,connected=a.role=='LAN',sta_credentials=False,ip='127.0.0.1',read_only=a.role=='LAN',ota_ingest=0,tls=False)
class Handler(BaseHTTPRequestHandler):
 def send(self,value,status=200,content='application/json'):
  data=value.encode() if isinstance(value,str) else json.dumps(value).encode();self.send_response(status);self.send_header('Content-Type',content);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
 def do_GET(self):
  path=self.path.split('?')[0]
  if path=='/':return self.send(html,content='text/html; charset=utf-8')
  if path=='/status':return self.send('build_id=OFFLINE_VISUAL_FIXTURE\nmode=MAINTENANCE\nauthority=NONE\ncalibration_hardware_motion_authorized=NO\nactuator_readiness_torque_enable=BLOCKED\nactuator_readiness_position_command=BLOCKED\nbms_valid=NO\nimu_rv_count=0\n',content='text/plain')
  if path=='/wifi/status':return self.send(w)
  if path=='/api/motion/capabilities':
   c=demo.capabilities();c['transport_readonly']=a.role=='LAN';return self.send(c)
  if path=='/api/motion/status':return self.send(dict(version=1,now_ms=1000,last_sequence=0,owner='none',executor_state='OFF',stand_state='OBSERVE',fault=False,reason='OFFLINE_DISPLAY_ONLY',stale=True,transport_readonly=a.role=='LAN',readiness=dict(restore=False,limits_approved=False,permit=False,hardware_approved=False),validation=dict(scope='UNQUALIFIED',capable=True,foot_load_proven=False),telemetry=dict(source='OFFLINE_SYNTHETIC',actual_rad=None,target_rad=None,raw_ticks=None,quaternion=None)))
  return self.send(dict(reason='NOT_FOUND'),404)
 def do_POST(self):self.send(dict(reason='OFFLINE_FIXTURE_DENIED_NO_HARDWARE'),403)
 def log_message(self,*args):pass
class Viewer(SimpleHTTPRequestHandler):
 def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(ROOT/'06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer/dist'),**kwargs)
 def do_GET(self):
  if self.path.split('?')[0]=='/motion/adaptive_motion_synthetic.jsonl':
   data=a.trace.read_bytes();self.send_response(200);self.send_header('Content-Type','application/jsonl');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
  else:super().do_GET()
 def log_message(self,*args):pass
ThreadingHTTPServer(('127.0.0.1',a.port),Viewer if a.role=='VIEWER' else Handler).serve_forever()
