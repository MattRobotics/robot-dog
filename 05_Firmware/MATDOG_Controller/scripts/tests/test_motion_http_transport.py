#!/usr/bin/env python3
"""Extracted-production handleMotion, service admission and cases; host-only doubles.

Socket ABI/HTTPD delivery, driver observations and mailbox scheduling are fakes.
PortalSession, JSON parser, semantic MotionService and its catalog are real code.
No devices are opened; no actuator is bound and no hardware writes can occur.
"""
import ast
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent / 'src'
c = (SRC/'network/HttpTransport.cpp').read_text()
h = (SRC/'network/HttpTransport.h').read_text()
w = (SRC/'network/WifiManager.cpp').read_text()
def between(text, start, end):
    assert text.count(start) == 1 and text.count(end) == 1, (start, end)
    return text.split(start, 1)[1].split(end, 1)[0]
tree = ast.parse((HERE/'test_ap_transport.py').read_text())
fixture = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'fixture' for t in n.targets))
fixture = fixture[:fixture.index('int main(int argc,char**)')]
fixture = fixture.replace('#include <map>', '#include <map>\n#include <mutex>\n#include <memory>\n#include <vector>\n#include <WiFi.h>\n#include <freertos/semphr.h>\n#include "network/AdminAccess.h"\n#include "network/NetworkPolicy.h"\n#include "network/NetworkPolicy.h"\n#include "core/MotionService.h"\n#include "core/MotionJson.h"')
fixture = fixture.replace('constexpr int ESP_OK=0, HTTP_GET=0', 'constexpr int HTTP_GET=0')
fixture = fixture.replace('memset(p,0x42,n)', 'static unsigned seed=0;memset(p,++seed,n)')
fixture = fixture.replace('struct FakeService {', '''class WifiManager { public: static bool verifyAdminRadio(); };
struct FakeService {
 AdminAccess gate;
 struct {bool run_mode=false,connected=false;uint32_t ipv4=0;uint8_t role_reason=0;} motion_wifi;
 const auto& wifiStatus() const {return motion_wifi;}
 core::MotionService motion;
 unsigned command_calls=0,status_calls=0,catalog_calls=0;
 FakeService() { gate.begin(); gate.phase.store(NetworkPhase::ADMIN_AP); gate.open(); motion.begin(nullptr,nullptr); }
 AdminAccess &networkAdminAccess() { return gate; }
 bool motionStatus(char* out,size_t cap,uint32_t now) { ++status_calls;return motion.statusJson(out,cap,now); }
 bool motionCapabilities(char* out,size_t cap) { ++catalog_calls;return motion.capabilitiesJson(out,cap); }
 motion::MotionAck motionCommand(const motion::MotionCommand& command,uint32_t now) { ++command_calls;return motion.command(command,now); }
 bool motionAck(const motion::MotionAck& ack,char* out,size_t cap) { return motion.ackJson(ack,out,cap); }
''')
fixture = fixture.replace(' static esp_err_t handleWifi(httpd_req_t*);', ''' bool portal_sta_=false;
 bool admitReadOnly(HttpTransportRequest&);
 bool readOnlyAdmitted(const HttpTransportRequest&) const;
 bool admitMotion(HttpTransportRequest&);
 bool motionStaAdmitted(const HttpTransportRequest&) const;
 uint32_t portal_generation_=0;
 bool busy=false,revoke_before_service=false;
 char motion_capabilities_[kMotionCapabilitiesBytes]{};
 bool admitAdmin(HttpTransportRequest&);
 static esp_err_t handleMotion(httpd_req_t*);
 static esp_err_t handleWifi(httpd_req_t*);''')
fixture = fixture.replace('bool dispatch(const HttpTransportRequest&req,HttpTransportResponse*out){', '''bool dispatch(const HttpTransportRequest&req,HttpTransportResponse*out,char* large_text=nullptr,size_t large_cap=0){
  if(busy)return false;
  if(revoke_before_service)service.gate.revoke();''')
fixture = fixture.replace('  switch(req.kind){', '''  auto serviceRequest = [&]() {
  // PRODUCTION_SERVICE_ADMISSION
  switch(req.kind){''')
fixture = fixture.replace('   default: return false;', '   default: return;')
fixture = fixture.replace('  *out=resp;return true;', '''  }; serviceRequest(); *out=resp;
  [[maybe_unused]] const auto& request=req;[[maybe_unused]] auto* response=out;
  // PRODUCTION_CATALOG_COPY
  return true;''')
fixture = fixture.replace('sha.finish(s.service.digest);', 'sha.finish(s.service.digest);s.service.gate.setDigest(s.service.digest);')
# Clock and short reads are controllable, while the shipped bounded receive loop remains intact.
fixture = fixture.replace('struct httpd_req_t {', 'size_t recv_chunk=4096;uint32_t recv_delay_ms=0;\nstruct httpd_req_t {')
fixture = fixture.replace('size_t z=std::min(n,r->body.size()-r->consumed);', 'clock_ms+=recv_delay_ms;size_t z=std::min({n,r->body.size()-r->consumed,recv_chunk});')
platform = r'''
WifiStub WiFi;
esp_err_t esp_wifi_get_mode(wifi_mode_t *m) { if(!WiFi.mode_read_ok)return -1;*m=static_cast<wifi_mode_t>(WiFi.mode_value);return ESP_OK; }
esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t*) { if(WiFi.connected)return ESP_OK;return WiFi.STA.started()?ESP_ERR_WIFI_NOT_CONNECT:ESP_ERR_WIFI_CONN; }
SemaphoreHandle_t xSemaphoreCreateMutex() { static std::vector<std::unique_ptr<std::mutex>> m;m.emplace_back(new std::mutex);return m.back().get(); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t h,TickType_t t) { if(t)abort();return static_cast<std::mutex*>(h)->try_lock(); }
BaseType_t xSemaphoreGive(SemaphoreHandle_t h) {static_cast<std::mutex*>(h)->unlock();return 1;}
'''
fixture = fixture.replace('namespace matdog { namespace network {', platform+'\nnamespace matdog { namespace network {', 1)
fixture += r'''
void adminRadio() { WiFi.mode(WIFI_AP);WiFi.ap=true;WiFi.AP.running=true;WiFi.STA.running=false; }
void login(HttpTransport& s) { clock_ms+=1000;auto r=request(s,"/wifi/login","token="+token);run(r);check(r.status==200,"production login");check(r.response_headers["Set-Cookie"].find("HttpOnly; SameSite=Strict; Path=/; Max-Age=300")!=std::string::npos,"session cookie reaches /api/motion and remains HttpOnly"); }
httpd_req_t motionRequest(HttpTransport& s,const std::string& body,bool authenticated=true) {
 auto r=request(s,"/api/motion/command",body);r.headers["Content-Type"]="application/json";
 if(authenticated){r.headers["Cookie"]="md_session="+std::string(s.portal_.cookie());r.headers["X-Csrf-Token"]=s.portal_.csrf();}
 return r;
}
void runMotion(httpd_req_t& r) {r.uri=r.uri_store.c_str();HttpTransport::handleMotion(&r);}
std::string command(const char* type,const char* params="{}",const char* owner="dashboard",unsigned sequence=1,int age=0) {
 return std::string("{\"version\":1,\"id\":\"fixture-")+std::to_string(sequence)+"\",\"sequence\":"+std::to_string(sequence)+",\"issued_at_ms\":"+std::to_string(static_cast<uint32_t>(clock_ms-age))+",\"ttl_ms\":1000,\"owner\":\""+owner+"\",\"type\":\""+type+"\",\"params\":"+params+"}";
}
void expectReject(HttpTransport& s,const char* type,const char* params,const char* reason,unsigned sequence) {
 auto r=motionRequest(s,command(type,params,"dashboard",sequence));runMotion(r);
 check(r.status==200&&r.response.find("\"outcome\":\"REJECTED\"")!=std::string::npos&&r.response.find(reason)!=std::string::npos,reason);
}
int main() {
 adminRadio();endpoints();HttpTransport s;provision(s);
 auto status=request(s,"/api/motion/status","");status.method=HTTP_GET;runMotion(status);
 check(status.status==200&&status.response.find("\"version\":1")!=std::string::npos,"production GET status");
 check(status.response_headers["Content-Type"]=="application/json"&&status.response_headers["Cache-Control"]=="no-store","JSON/no-store response");
 auto caps=request(s,"/api/motion/capabilities","");caps.method=HTTP_GET;runMotion(caps);
 size_t count=0,pos=0;while((pos=caps.response.find("\"id\":",pos))!=std::string::npos){count++;pos+=5;}
 check(caps.status==200&&count==48,"production 48 action catalog delivered via heap snapshot");
 check(caps.response.size()>4096&&caps.response.size()<kMotionCapabilitiesBytes,"catalog exceeds small status buffer and fits heap response");
 check(caps.response.find("\"teleoperation\":false")!=std::string::npos,"Controller refuses unqualified teleoperation");
 auto unauth=motionRequest(s,command("MOTION.STOP"),false);runMotion(unauth);check(unauth.status==403&&s.service.command_calls==0,"cookie and CSRF required even for semantic writes");
 login(s);
 auto stop=motionRequest(s,command("MOTION.STOP","{\"reason\":\"fixture\"}"));runMotion(stop);check(stop.status==200&&stop.response.find("COMPLETED")!=std::string::npos,"authorized semantic STOP acknowledges without actuator binding");
 expectReject(s,"POSTURE.STAND","{\"target_height_m\":0.15}","RESTORE_REQUIRED",2);
 expectReject(s,"GAIT.SELECT","{\"gait\":\"TROT\"}","GAIT_NOT_VALIDATED",3);
 expectReject(s,"ACTION.EXECUTE","{\"action_id\":\"action.wave\"}","ACTION_NOT_VALIDATED",4);
 expectReject(s,"BASE_VELOCITY","{\"vx\":0.1,\"vy\":0,\"yaw_rate\":0}","VELOCITY_OUTSIDE_SOFTWARE_ENVELOPE",5);
 expectReject(s,"BODY_POSE","{\"x\":0.01}","ACTION_NOT_VALIDATED",6);
 auto expired=motionRequest(s,command("MOTION.STATUS","{}","dashboard",7,1001));runMotion(expired);check(expired.status==200&&expired.response.find("EXPIRED")!=std::string::npos,"backend TTL rejection delivered as ack");
 const unsigned calls=s.service.command_calls;
 for(const char* owner:{"jetson","local"}){auto r=motionRequest(s,command("MOTION.STOP","{}",owner,8));runMotion(r);check(r.status==403,"browser cannot select Jetson/local owner");}
 for(const char* key:{"Host","Origin"}){auto r=motionRequest(s,command("MOTION.STOP"));r.headers[key]="evil.invalid";runMotion(r);check(r.status==400,"untrusted write origin denied before service");}
 for(const char* bad:{"application/x-www-form-urlencoded","application/json; charset=utf-8",""}){auto r=motionRequest(s,command("MOTION.STOP"));r.headers["Content-Type"]=bad;runMotion(r);check(r.status==400,"only specified JSON content type");}
 for(const char* body:{"", "{bad}", "{\"version\":1,\"version\":1}"}){auto r=motionRequest(s,body);runMotion(r);check(r.status==400,"empty malformed duplicate JSON denied");}
 {auto r=motionRequest(s,std::string(1025,'x'));runMotion(r);check(r.status==400&&r.consumed==0,"body >1024 denied without reading");}
 {auto r=motionRequest(s,command("MOTION.STOP"));r.content_len++;runMotion(r);check(r.status==400,"truncated bounded body denied");}
 {auto r=motionRequest(s,command("MOTION.STOP"));r.body.push_back('\0');r.content_len++;runMotion(r);check(r.status==400,"embedded NUL denied");}
 {auto r=motionRequest(s,command("MOTION.STOP"));r.headers["X-Csrf-Token"]=std::string(64,'0');runMotion(r);check(r.status==403,"CSRF mismatch denied");}
 {auto r=motionRequest(s,command("MOTION.STOP"));r.headers["Cookie"]="md_session="+std::string(64,'0');runMotion(r);check(r.status==403,"cookie mismatch denied");}
 {auto r=motionRequest(s,command("MOTION.STOP"));r.headers["Cookie"]="md_device="+std::string(s.portal_.cookie());runMotion(r);check(r.status==403,"device cookie alone grants no motion session");}
 check(s.service.command_calls==calls,"all rejected transport envelopes bypass semantic service");
 recv_chunk=1;recv_delay_ms=1000;auto timeout=motionRequest(s,command("MOTION.STOP"));runMotion(timeout);check(timeout.status==400&&timeout.response=="Body timeout","bounded production receive timeout");recv_chunk=4096;recv_delay_ms=0;
 login(s);
 endpoints(true,0xc0a80401u,0xc0a80403u);auto peer=motionRequest(s,command("MOTION.STOP"));runMotion(peer);check(peer.status==403,"session remains peer-bound");endpoints();
 for(auto phase:{NetworkPhase::CLIENT_STA,NetworkPhase::ENTER_ADMIN,NetworkPhase::LEAVE_ADMIN,NetworkPhase::CLOSED,NetworkPhase::FAULT}){
  s.service.gate.phase.store(phase);auto r=request(s,"/api/motion/status","");r.method=HTTP_GET;runMotion(r);check(r.status==403,"motion GET phase fence");auto p=motionRequest(s,command("MOTION.STOP"));runMotion(p);check(p.status==403,"motion POST phase fence");
 }
 s.service.gate.phase.store(NetworkPhase::ADMIN_AP);
 for(int mode:{WIFI_STA,WIFI_AP_STA}){WiFi.mode_value=mode;auto r=request(s,"/api/motion/capabilities","");r.method=HTTP_GET;runMotion(r);check(r.status==403,"STA/APSTA driver state rejects motion read");}
 adminRadio();WiFi.mode_read_ok=false;auto unverifiable=request(s,"/api/motion/status","");unverifiable.method=HTTP_GET;runMotion(unverifiable);check(unverifiable.status==403,"unverifiable radio rejects");WiFi.mode_read_ok=true;
 s.service.gate.phase.store(NetworkPhase::CLIENT_STA);WiFi.mode_value=WIFI_STA;endpoint(local_addr,local_len,0xc0a80110u,true,443);auto sta=request(s,"/api/motion/status","");sta.method=HTTP_GET;runMotion(sta);check(sta.status==403,"STA TLS cannot access motion API");
 // Real RUN_STA TLS route, independent authenticated session and no authority.
 s.service.motion_wifi.run_mode=true;s.service.motion_wifi.connected=true;
 s.service.motion_wifi.role_reason=static_cast<uint8_t>(RoleReason::RUN_STA);
 s.service.motion_wifi.ipv4=htonl(0xc0a80110u);WiFi.connected=true;WiFi.STA.running=true;
 s.service.gate.closing.store(false);
 clock_ms+=1000;auto tlsLogin=request(s,"/api/motion/login","token="+token,"192.168.1.16","https://192.168.1.16");run(tlsLogin);
 check(tlsLogin.status==200,"RUN_STA TLS credential login");
 check(tlsLogin.response_headers["Set-Cookie"].find("; Secure")!=std::string::npos,"TLS session Secure cookie");
 auto tlsStatus=request(s,"/api/motion/status","","192.168.1.16","https://192.168.1.16");tlsStatus.method=HTTP_GET;
 runMotion(tlsStatus);check(tlsStatus.status==403,"STA GET requires session");
 tlsStatus.status=200;tlsStatus.headers["Cookie"]="md_session="+std::string(s.portal_.cookie());runMotion(tlsStatus);
 check(tlsStatus.status==200,"authenticated STA actual/status snapshot");
 auto tlsStop=motionRequest(s,command("MOTION.STOP"));tlsStop.headers["Host"]="192.168.1.16";tlsStop.headers["Origin"]="https://192.168.1.16";
 runMotion(tlsStop);check(tlsStop.status==200,"authenticated STA semantic STOP");
 auto insecure=tlsStop;endpoint(local_addr,local_len,0xc0a80110u,true,80);runMotion(insecure);check(insecure.status==403,"plain STA HTTP denied");
 endpoint(local_addr,local_len,0xc0a80110u,true,443);s.service.motion_wifi.connected=false;auto lost=tlsStop;runMotion(lost);check(lost.status==403,"STA link loss fences command");
 s.service.motion_wifi.run_mode=false;adminRadio();endpoints();s.service.gate.phase.store(NetworkPhase::ADMIN_AP);
 login(s);
 s.revoke_before_service=true;auto pending=motionRequest(s,command("MOTION.STOP"));runMotion(pending);check(pending.status==403,"revocation between handler and Controller service fences request");s.revoke_before_service=false;s.service.gate.open();
 auto old=motionRequest(s,command("MOTION.STOP"));runMotion(old);check(old.status==403,"generation change resets portal session");login(s);
 s.busy=true;auto busy=request(s,"/api/motion/status","");busy.method=HTTP_GET;runMotion(busy);check(busy.status==500&&busy.response=="Controller busy","mailbox failure explicit");s.busy=false;
 // MAINTENANCE LAN has a separate GET-only contract; no session or command.
 s.service.gate.phase.store(NetworkPhase::CLIENT_STA);s.service.gate.radio_fault.store(false);
 s.service.motion_wifi.run_mode=false;s.service.motion_wifi.connected=true;s.service.motion_wifi.role_reason=static_cast<uint8_t>(RoleReason::MAINTENANCE_STA_READONLY);
 s.service.gate.closing.store(false);s.busy=false;s.revoke_before_service=false;WiFi.connected=true;WiFi.mode_value=WIFI_STA;
 endpoint(local_addr,local_len,0xc0a80110u,true,80);
 auto ro=request(s,"/api/motion/status","");ro.method=HTTP_GET;runMotion(ro);check(ro.status==200,"LAN maintenance status allowed without grant");
 auto rocap=request(s,"/api/motion/capabilities","");rocap.method=HTTP_GET;runMotion(rocap);check(rocap.status==200,"LAN maintenance catalog allowed");
 const unsigned before=s.service.command_calls;auto denied=motionRequest(s,command("MOTION.STAND"));runMotion(denied);check(denied.status==403&&s.service.command_calls==before,"LAN command denied before service");
 auto rologin=request(s,"/api/motion/login","token="+token);run(rologin);check(rologin.status==403,"LAN cannot create operational session");
 s.service.gate.closing.store(true);auto closed=ro;runMotion(closed);check(closed.status==403,"LAN read revoked during radio transition");
 s.service.gate.closing.store(false);WiFi.mode_value=WIFI_AP_STA;auto dual=ro;runMotion(dual);check(dual.status==403,"LAN APSTA never admitted");
 check(s.service.mutations==0,"no configuration or actuator writes from motion API harness");
 printf("MOTION_HTTP_TRANSPORT = PASS (%u production-handler/security/service checks; HTTPD/socket/mailbox/driver facts are doubles)\n",checks);
}
'''
helpers = between(c, '\nbool readHexHeader', '\nvoid jsonString');helpers = helpers[helpers.index('\n}')+2:]
types = 'enum class HttpRequestKind'+between(h, 'enum class HttpRequestKind', '// Two optional listeners')
handler = 'esp_err_t HttpTransport::handleMotion'+between(c, 'esp_err_t HttpTransport::handleMotion', 'esp_err_t HttpTransport::handleWifi')
wifi_handler = 'esp_err_t HttpTransport::handleWifi'+between(c, 'esp_err_t HttpTransport::handleWifi', '\n\n} // namespace network')
admit = 'bool HttpTransport::admitAdmin('+between(c, 'bool HttpTransport::admitAdmin(', 'bool HttpTransport::dispatch(')
verify = 'bool WifiManager::verifyAdminRadio()'+between(w, 'bool WifiManager::verifyAdminRadio()', '// Arduino delivers role events asynchronously.')
admission = between(c, '  resp.sequence = req.sequence;\n', '  switch (req.kind) {')
cases = 'case HttpRequestKind::MOTION_STATUS:'+between(c, 'case HttpRequestKind::MOTION_STATUS:', 'case HttpRequestKind::WIFI_STATUS:')
login_case = 'case HttpRequestKind::MOTION_LOGIN: case HttpRequestKind::WIFI_LOGIN:'+between(c, 'case HttpRequestKind::WIFI_LOGIN:', 'case HttpRequestKind::WIFI_SESSION:')
copy_start = c.index('      if(request.kind==HttpRequestKind::MOTION_CAPABILITIES')
copy = c[copy_start:c.index('      xSemaphoreGive(slot_free_);', copy_start)]
admit += 'bool HttpTransport::readOnlyAdmitted'+between(c,'bool HttpTransport::readOnlyAdmitted','void HttpTransport::lifecycle()')
unit = fixture.replace('// PRODUCTION_TYPES',types).replace('// PRODUCTION_HELPERS',helpers+admit+verify).replace('// PRODUCTION_HANDLER',handler+wifi_handler).replace('// PRODUCTION_SERVICE_CASES',cases+login_case).replace('// PRODUCTION_SERVICE_ADMISSION',admission).replace('// PRODUCTION_CATALOG_COPY',copy)
with tempfile.TemporaryDirectory(prefix='matdog-motion-http-') as temp:
    cpp, exe = Path(temp)/'probe.cpp', Path(temp)/'probe';cpp.write_text(unit)
    sources = ['network/AdminAccess.cpp','network/PortalSecurity.cpp','network/NetworkConfig.cpp','update/Sha256.cpp','update/Hmac256.cpp','core/MotionService.cpp','core/MotionJson.cpp','execution/MotionExecutor.cpp','actuator/ActuatorRuntime.cpp','actuator/ActuatorWritePolicy.cpp','actuator/CalibrationGeometryProfile.cpp','actuator/CalibrationTargetResolver.cpp','actuator/CalibrationSequencePlan.cpp','calibration/StartupRecoveryReference.cpp','calibration/CalibrationDomain.cpp','servo/ServoPopulation.cpp','core/ActuatorAuthority.cpp','core/OperatingMode.cpp']
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-DDISABLED=0x00','-O1','-pthread','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(SRC),'-I',str(HERE/'network_stubs'),'-I',str(HERE/'nvs_stub'),str(cpp),*[str(SRC/p) for p in sources],*[str(SRC/'motion'/p) for p in ['FirstStrideReference.cpp','PairedProneRoute.cpp','StlContactGait.cpp','MotionContract.cpp','MotionFeedback.cpp','AdaptiveStand.cpp','FeedbackWalk.cpp','BodyPose.cpp','PoseSupport.cpp','Gait.cpp','Locomotion.cpp','TimedStand.cpp','StandTransition.cpp','StandTrajectory.cpp','FootContact.cpp','ContactMode.cpp','TiltedContactIk.cpp','LegKinematics.cpp','LegInverseKinematics.cpp','MotionState.cpp','StartupAcquisition.cpp']],'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
