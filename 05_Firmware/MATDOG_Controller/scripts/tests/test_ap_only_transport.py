#!/usr/bin/env python3
"""AP-only acceptance on extracted production HTTP paths and AdminAccess.

HTTPD/TLS, socket ABI, Controller operations, driver observations and scheduling
are doubles. Includes unmodified receive/admission, Controller
service admission and lifecycle code; WifiManager runtime separately exercises
queued intent and actual production NVS transaction calls with storage doubles.
Historical F01 AP+STA tests remain pinned to baseline and deliberately fail.
"""
import ast
from pathlib import Path
import subprocess
import tempfile
here = Path(__file__).resolve().parent
src = here.parent.parent / 'src'
c = (src/'network/HttpTransport.cpp').read_text()
h = (src/'network/HttpTransport.h').read_text()
w = (src/'network/WifiManager.cpp').read_text()
def between(s,a,b):
    assert s.count(a)==1 and s.count(b)==1,(a,b)
    return s.split(a,1)[1].split(b,1)[0]
tree=ast.parse((here/'test_ap_transport.py').read_text())
fixture=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='fixture' for t in n.targets))
fixture=fixture.replace('#include <map>', '#include <map>\n#include <mutex>\n#include <memory>\n#include <vector>\n#include <atomic>\n#include <thread>\n#include <WiFi.h>\n#include <freertos/semphr.h>\n#include "network/AdminAccess.h"')
fixture=fixture.replace('memset(p,0x42,n)', 'static unsigned seed=0;memset(p,++seed,n)')
fixture=fixture.replace('constexpr int ESP_OK=0, HTTP_GET=0','constexpr int HTTP_GET=0')
# The portal now answers with two Set-Cookie headers; the HTTPD double keeps both.
fixture=fixture.replace('{ r->response_headers[k]=v;return 0; }','{ auto &h=r->response_headers[k]; if(std::string(k)=="Set-Cookie"&&!h.empty())h+="\\n"; else h.clear(); h+=v; return 0; }')
assert 'h+="\\n"' in fixture
fixture=fixture.replace('struct FakeService {', '''class WifiManager { public: static bool verifyAdminRadio(); };
struct FakeService {
 AdminAccess gate;
 FakeService() { gate.begin(); gate.phase.store(NetworkPhase::ADMIN_AP); gate.open(); }
 AdminAccess &networkAdminAccess() {return gate;}
''')
fixture=fixture.replace('configureNetwork(const ConfigPatch&)', 'configureNetwork(const ConfigPatch&,uint32_t)').replace('accessPoint(bool)', 'accessPoint(bool,uint32_t)')
fixture=fixture.replace(' static esp_err_t handleWifi', ''' uint32_t portal_generation_=0;
 bool revoke_before_service=false;
 bool admitAdmin(HttpTransportRequest&);
 void lifecycleTick();
 void *server_=nullptr,*tls_server_=nullptr;
 std::atomic<bool> desired_{false},tls_desired_{false},listening_{false},tls_listening_{false},manual_stop_{false};
 bool registerHandlers(void*,bool=true);
 static esp_err_t handlePage(httpd_req_t*);
 static esp_err_t handleStatus(httpd_req_t*) {return 0;}
 static esp_err_t handleOtaChallenge(httpd_req_t*) {return 0;}
 static esp_err_t handleOtaUpdate(httpd_req_t*) {return 0;}
 static esp_err_t handleWifi''')
fixture=fixture.replace('#include \"network/AdminAccess.h\"', '#include \"network/AdminAccess.h\"\n#include \"network/PortalPage.h\"')
fixture=fixture.replace('sha.finish(s.service.digest);', 'sha.finish(s.service.digest);s.service.gate.setDigest(s.service.digest);')
fixture=fixture.replace('  switch(req.kind){','''  if(revoke_before_service) service.gate.revoke();
  // PRODUCTION_SERVICE_ADMISSION
  switch(req.kind){''')
# Prefix includes guard and generation-reset exactly as shipped, using response local.
admission=between(c,'  resp.sequence = req.sequence;\n','  switch (req.kind) {')
fixture=fixture.replace('// PRODUCTION_SERVICE_ADMISSION',admission)
fixture=fixture.replace('  if(revoke_before_service)', '  auto serviceRequest = [&]() {\n  if(revoke_before_service)').replace('   default: return false;', '   default: return;').replace('  *out=resp;return true;', '  }; serviceRequest(); *out=resp;return true;')
platform=r'''
WifiStub WiFi;
esp_err_t esp_wifi_get_mode(wifi_mode_t *m) { if(!WiFi.mode_read_ok)return -1; *m=static_cast<wifi_mode_t>(WiFi.mode_value); return ESP_OK; }
esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t*) { if(WiFi.connected)return ESP_OK; return WiFi.STA.started()?ESP_ERR_WIFI_NOT_CONNECT:ESP_ERR_WIFI_CONN; }
SemaphoreHandle_t xSemaphoreCreateMutex() { static std::vector<std::unique_ptr<std::mutex>> m; m.emplace_back(new std::mutex); return m.back().get(); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t h,TickType_t t) { if(t)abort(); return static_cast<std::mutex*>(h)->try_lock(); }
BaseType_t xSemaphoreGive(SemaphoreHandle_t h) {static_cast<std::mutex*>(h)->unlock(); return 1;}
namespace matdog { namespace config { constexpr bool kTlsIdentityPresent=true; const char kTlsCertificate[]="cert",kTlsPrivateKey[]="key"; } }
constexpr int MALLOC_CAP_INTERNAL=1;
size_t heap_caps_get_largest_free_block(int){return 200000;}
struct httpd_config_t {int stack_size,max_uri_handlers,max_open_sockets,recv_wait_timeout,send_wait_timeout,task_priority;bool lru_purge_enable;};
struct httpd_ssl_config_t {httpd_config_t httpd;int tls_handshake_timeout_ms;const uint8_t *servercert,*prvtkey_pem;size_t servercert_len,prvtkey_len;};
#define HTTPD_DEFAULT_CONFIG() httpd_config_t{}
#define HTTPD_SSL_CONFIG_DEFAULT() httpd_ssl_config_t{}
using httpd_handle_t=void*;
struct httpd_uri_t {const char *uri;int method;esp_err_t (*handler)(httpd_req_t*);void *user_ctx;};
std::vector<std::string> registered;
int httpd_register_uri_handler(void*,const httpd_uri_t *u){registered.emplace_back(u->uri);return ESP_OK;}
bool stop_fails=false,tls_stop_fails=false;
unsigned stop_calls=0,tls_stop_calls=0;
int httpd_start(void **h,httpd_config_t*){*h=reinterpret_cast<void*>(2);return 0;}
int httpd_ssl_start(void **h,httpd_ssl_config_t*){*h=reinterpret_cast<void*>(3);return 0;}
int httpd_stop(void*){++stop_calls;return stop_fails?-1:0;}
int httpd_ssl_stop(void*){++tls_stop_calls;return tls_stop_fails?-1:0;}
'''
fixture=fixture.replace('namespace matdog { namespace network {',platform+'\nnamespace matdog { namespace network {',1)
# status bits/AP mode: explicit host observations, never a radio simulation.
fixture=fixture.replace('void endpoints(bool mapped=', 'void endpoints(bool mapped=')
fixture=fixture[:fixture.index('int main(int argc,char**)')]+r'''
void adminRadio() {WiFi.mode(WIFI_AP);WiFi.ap=true;WiFi.AP.running=true;}
void loginOk(HttpTransport &s) {clock_ms+=1000;auto r=request(s,"/wifi/login","token="+token);run(r);if(r.status!=200)fprintf(stderr,"login status=%d message=%s check=%u phase=%d mode=%d\n",r.status,r.response.c_str(),checks,int(s.service.gate.phase.load()),WiFi.mode_value);check(r.status==200,"AP login");}
int action(HttpTransport &s,const char*path="/wifi/ap",const char*body="on=1",const char*cookie=nullptr,const char*csrf=nullptr) {
 auto r=request(s,path,body);r.headers["Cookie"]="md_session="+std::string(cookie?cookie:s.portal_.cookie());
 r.headers["X-Csrf-Token"]=csrf?csrf:s.portal_.csrf();run(r);return r.status;
}
int main() {
 adminRadio();endpoints();HttpTransport s;provision(s);memset(s.service.digest,0,32);loginOk(s); // delayed Controller snapshot cannot replace the fenced digest
 check(action(s)==200&&s.service.mutations==1,"valid POST ADMIN_AP");
 check(action(s,"/wifi/scan","")==403,"manual scan explicit rejection");
 check(WiFi.mode_value==WIFI_AP && WiFi.scan_calls==0,"scan never enables STA");
 check(action(s,"/wifi/config","usb_sleep=0")==200,"config intent dispatch");
 auto writes=s.service.mutations;
 for(auto phase:{NetworkPhase::CLIENT_STA,NetworkPhase::ENTER_ADMIN,NetworkPhase::LEAVE_ADMIN,NetworkPhase::CLOSED,NetworkPhase::FAULT}) {
  s.service.gate.phase.store(phase);auto r=request(s,"/wifi/login","token="+token);run(r);check(r.status==403,"phase fail closed");check(action(s)==403,"phase POST deny");
 }
 s.service.gate.phase.store(NetworkPhase::CLIENT_STA);
 for(const char *path:{"/wifi/status","/wifi/config"}) {auto r=request(s,path,"");r.method=HTTP_GET;run(r);check(r.status==403,"all admin GET denied in CLIENT_STA");}
 auto page=request(s,"/","");page.method=HTTP_GET;check(HttpTransport::handlePage(&page)==0&&page.status==403,"page denied in CLIENT_STA");
 s.service.gate.phase.store(NetworkPhase::ADMIN_AP);
 registered.clear();check(s.registerHandlers(nullptr,false),"register HTTPS role");
 check(registered.size()==3&&std::none_of(registered.begin(),registered.end(),[](auto &u){return u=="/"||u.find("/wifi/")==0;}),"HTTPS has only non-admin routes");
 registered.clear();check(s.registerHandlers(nullptr,true)&&registered.size()==12,"HTTP admin route inventory");
 for(int mode:{WIFI_STA,WIFI_AP_STA}) {WiFi.mode_value=mode;check(action(s)==403,"driver STA/APSTA deny");}
 adminRadio();WiFi.mode_read_ok=false;check(action(s)==403,"driver unverifiable deny");WiFi.mode_read_ok=true;
 WiFi.STA.running=true;check(action(s)==403,"operational STA bits deny");adminRadio();
 check(s.service.mutations==writes,"denied requests no writes");
 // Old credentials cannot authorize a fresh generation even with deterministic
 // random doubles: generation reset happens before authorize. Test replay first.
 std::string cookie=s.portal_.cookie(),csrf=s.portal_.csrf(); auto generation=s.service.gate.generation();
 s.service.gate.revoke();check(action(s)==403,"revoke denies at reception");
 s.service.gate.open();check(!s.service.gate.allowed(generation),"old generation denied");
 check(action(s,"/wifi/ap","on=1",cookie.c_str(),csrf.c_str())==403,"old cookie/CSRF rejected after restart before new login");
 loginOk(s);check(action(s)==200,"fresh login required");
 check(action(s,"/wifi/ap","on=1",cookie.c_str(),csrf.c_str())==403,"old tokens rejected even after fresh login");
 // A request received under AP, then revoked before Controller service.
 writes=s.service.mutations;s.revoke_before_service=true;check(action(s)==403,"pending HTTP denied after revoke");
 check(s.service.mutations==writes,"pending HTTP zero service mutation");s.revoke_before_service=false;s.service.gate.open();loginOk(s);
 for(bool mapped:{false,true}) {endpoints(mapped);check(action(s)==200,"IPv4 and mapped IPv6 POST");}
 endpoints();for(const char*k:{"Host","Origin"}){auto r=request(s,"/wifi/login","token="+token);r.headers[k]="evil.invalid";run(r);check(r.status==403,"Host/Origin preserved");}
 check(action(s,"/wifi/ap","on=1",nullptr,std::string(64,'0').c_str())==403,"CSRF deny");
 check(action(s,"/wifi/ap","on=1",std::string(64,'0').c_str())==403,"cookie deny");
 endpoints(true,0xc0a80401u,0xc0a80403u);check(action(s)==403,"peer binding");endpoints();
 // HTTPS shares exact same admin guard even if routes accidentally registered.
 endpoint(local_addr,local_len,0xc0a80110u,true,443);check(action(s)==403,"HTTPS STA cannot admin");endpoints();
 clock_ms=(uint64_t{1}<<32)-1000;HttpTransport uptime;provision(uptime);loginOk(uptime);clock_ms+=2000;check(action(uptime)==200,"F02 wrap live");clock_ms+=300000;check(action(uptime)==403,"F02 expiry");clock_ms=(uint64_t{2}<<32)-1000;check(action(uptime)==403,"F02 no replay");
 HttpTransport throttle;provision(throttle);for(int i=0;i<5;++i){clock_ms+=1000;auto r=request(throttle,"/wifi/login","token="+std::string(64,'b'));run(r);check(r.status==403,"bad auth");}clock_ms+=1000;auto r=request(throttle,"/wifi/login","token="+token);run(r);check(r.status==403,"throttle");clock_ms+=60000;loginOk(throttle);
 // Production lifecycle: stop success is the only acknowledgement. Failure
 // retains handles and no fresh start can replace the unresolved old listener.
 HttpTransport life;life.lifecycleTick();check(life.server_!=nullptr,"AP listener opened");
 life.tls_server_=reinterpret_cast<void*>(3);life.tls_listening_.store(true);
 life.service.gate.revoke();life.service.gate.phase.store(NetworkPhase::LEAVE_ADMIN);
 stop_fails=tls_stop_fails=true;life.lifecycleTick();check(life.server_!=nullptr&&!life.service.gate.listeners_closed.load(),"HTTP stop failure retains handle, denies transition");
 stop_fails=false;tls_stop_fails=true;life.lifecycleTick();check(life.server_==nullptr&&life.tls_server_!=nullptr&&!life.service.gate.listeners_closed.load(),"HTTPS stop failure denies transition");
 tls_stop_fails=false;life.lifecycleTick();check(life.tls_server_==nullptr&&life.service.gate.listeners_closed.load(),"both stops confirmed");
 life.service.gate.phase.store(NetworkPhase::FAULT);life.lifecycleTick();check(life.server_==nullptr,"unexpected restart does not reopen");
 // Drain refuses acknowledgement while a production lease owns the fence.
 HttpTransport drain;drain.lifecycleTick();{AdminLease lease(drain.service.gate);check(bool(lease),"lease acquired");drain.service.gate.revoke();std::thread closer([&]{drain.lifecycleTick();});closer.join();check(!drain.service.gate.listeners_closed.load(),"cannot acknowledge before lease drained");}
 drain.lifecycleTick();check(drain.service.gate.listeners_closed.load(),"drained closure acknowledges");
 // ---- Remembered device: persistent administrator recognition ----------------
 // Synthetic token only. The credential is stateless on the controller, so
 // "reboot" is a brand-new transport object holding the same admin digest.
 const auto npos=std::string::npos;
 adminRadio();endpoints();HttpTransport dev;provision(dev);clock_ms+=5000;
 auto plain=request(dev,"/wifi/login","token="+token);run(plain);
 check(plain.status==200&&plain.response_headers["Set-Cookie"].find("md_device=")==npos,"plain login issues no device credential");
 for(const char*bad:{"&remember=0","&remember=1x","&other=1","&remember=1&remember=1"}){clock_ms+=1000;auto r=request(dev,"/wifi/login","token="+token+bad);run(r);check(r.status==400,"malformed remember flag");}
 clock_ms+=1000;auto rem=request(dev,"/wifi/login","token="+token+"&remember=1");run(rem);check(rem.status==200,"remember login");
 std::string set=rem.response_headers["Set-Cookie"];size_t at=set.find("md_device=");
 check(set.find("md_session=")==0&&set.find("HttpOnly; SameSite=Strict; Path=/wifi/; Max-Age=300")!=npos,"session cookie unchanged by remember");
 check(at!=npos&&set.find("HttpOnly; SameSite=Strict; Path=/wifi/session; Max-Age=34560000",at)!=npos,"device cookie protected and scoped to the session endpoint");
 std::string device=set.substr(at+10,64);check(device.size()==64&&device!=dev.portal_.cookie()&&device!=token,"device credential is neither the token nor the session");
 clock_ms+=1000;auto wrongrem=request(dev,"/wifi/login","token="+std::string(64,'b')+"&remember=1");run(wrongrem);
 check(wrongrem.status==403&&wrongrem.response_headers.count("Set-Cookie")==0,"failed login issues nothing");
 clock_ms+=1000;loginOk(dev);
 // It opens a session; it never is one.
 writes=dev.service.mutations;
 check(action(dev,"/wifi/ap","on=1",device.c_str(),device.c_str())==403,"device credential is not a session cookie");
 {auto r=request(dev,"/wifi/ap","on=1");r.headers["Cookie"]="md_device="+device;r.headers["X-Csrf-Token"]=dev.portal_.csrf();run(r);check(r.status==403,"device cookie alone authorizes no write");}
 check(dev.service.mutations==writes,"no mutation from a device credential");
 auto resume=[&](HttpTransport &s,const std::string &cookies){auto r=request(s,"/wifi/session","");if(!cookies.empty())r.headers["Cookie"]=cookies;run(r);return r;};
 // Live session, page reload: its own cookie returns its own CSRF, same peer only.
 std::string live=dev.portal_.cookie(),live_csrf=dev.portal_.csrf();
 auto reload=resume(dev,"md_session="+live);
 check(reload.status==200&&reload.response==live_csrf&&live==dev.portal_.cookie(),"live session re-reads its CSRF, not replaced");
 check(reload.response_headers["Set-Cookie"].find("md_device=")==npos,"no device credential without proof");
 endpoints(true,0xc0a80401u,0xc0a80403u);check(resume(dev,"md_session="+live).status==403,"another peer cannot read the CSRF");endpoints();
 check(resume(dev,"").status==403,"no cookie, no CSRF");
 check(resume(dev,"md_session="+std::string(64,'0')).status==403,"forged session cookie gets no CSRF");
 check(resume(dev,"md_session="+live).status==200,"absent credentials are not throttled attempts");
 // New AP generation (after an STA test): the session is gone, the device is recognised.
 clock_ms+=2000;dev.service.gate.revoke();dev.service.gate.open();
 check(action(dev,"/wifi/ap","on=1",live.c_str(),live_csrf.c_str())==403,"old session dead in a new generation");
 auto back=resume(dev,"md_session="+live+"; md_device="+device);
 check(back.status==200&&back.response.size()==64&&back.response==dev.portal_.csrf(),"device recognised without the token");
 check(std::string(dev.portal_.cookie())!=live&&back.response!=live_csrf,"fresh session, fresh CSRF");
 check(back.response_headers["Set-Cookie"].find("md_device="+device)!=npos,"device cookie lifetime renewed");
 check(action(dev)==200&&dev.service.mutations==writes+1,"write authorized after resume");
 // Session expiry: silent renewal instead of a new login.
 clock_ms+=300000;check(action(dev)==403,"session still expires after 5 minutes");
 check(resume(dev,"md_device="+device).status==200&&action(dev)==200,"expired session renewed from the device");
 // Forgery and throttling.
 std::string forged=device;forged[63]=forged[63]=='0'?'1':'0';std::string other=device;other[0]=other[0]=='0'?'1':'0';
 dev.portal_.reset();clock_ms+=2000;
 check(resume(dev,"md_device="+forged).status==403,"forged tag refused");
 check(resume(dev,"md_device="+device).status==403,"valid credential throttled right after a failure");
 clock_ms+=1000;check(resume(dev,"md_device="+other).status==403,"forged id refused");
 // The throttle is over here, so these refusals are about the cookie itself.
 // Malformed cookies carry no credential at all: refused, and not counted.
 clock_ms+=1000;
 check(resume(dev,"md_device="+device.substr(0,63)).status==403,"short credential refused");
 check(resume(dev,"md_device="+device+"0").status==403,"long credential refused");
 check(resume(dev,"xmd_device="+device).status==403,"cookie name must match exactly");
 check(resume(dev,"md_device_old="+device).status==403,"cookie name is not a prefix match");
 check(resume(dev,"other=1; md_device="+device+"; z=2").status==200,"credential found among other cookies; malformed ones were not attempts");
 dev.portal_.reset();
 for(const std::string &bad:{forged,other,token,forged,other}){clock_ms+=1000;check(resume(dev,"md_device="+bad).status==403,"forged credential or token refused");}
 clock_ms+=1000;check(resume(dev,"md_device="+device).status==403,"five failures: long throttle");
 clock_ms+=58999;check(resume(dev,"md_device="+device).status==403,"throttle 60 s minus one");
 clock_ms+=1;check(resume(dev,"md_device="+device).status==200,"throttle releases");
 // Same origin, method and shape rules as every other write.
 for(const char*k:{"Host","Origin"}){auto r=request(dev,"/wifi/session","");r.headers["Cookie"]="md_device="+device;r.headers[k]="evil.invalid";run(r);check(r.status==403,"resume requires AP Host/Origin");}
 for(const char*path:{"/wifi/session?x=1","/wifi/sessions","/wifi/logout/","/wifi/other"}){auto r=request(dev,path,"");r.headers["Cookie"]="md_device="+device;run(r);check(r.status==400,"only the exact paths are actions");}
 {auto r=request(dev,"/wifi/session","x=1");r.headers["Cookie"]="md_device="+device;run(r);check(r.status==400,"resume takes no body");}
 {auto r=request(dev,"/wifi/session","");r.headers["Cookie"]="md_device="+device;r.method=HTTP_GET;run(r);check(r.response.size()!=64,"GET never returns a CSRF token");}
 endpoint(local_addr,local_len,0xc0a80110u,true,443);check(resume(dev,"md_device="+device).status==403,"no resume over the STA/TLS socket");endpoints();
 for(auto phase:{NetworkPhase::CLIENT_STA,NetworkPhase::ENTER_ADMIN,NetworkPhase::LEAVE_ADMIN,NetworkPhase::CLOSED,NetworkPhase::FAULT}){dev.service.gate.phase.store(phase);check(resume(dev,"md_device="+device).status==403,"resume only in ADMIN_AP");}
 dev.service.gate.phase.store(NetworkPhase::ADMIN_AP);
 for(int mode:{WIFI_STA,WIFI_AP_STA}){WiFi.mode_value=mode;check(resume(dev,"md_device="+device).status==403,"resume needs the verified AP radio");}
 adminRadio();
 // Reboot: nothing was stored, the same digest recognises the same device.
 HttpTransport reboot;provision(reboot);clock_ms+=1000;
 check(resume(reboot,"md_device="+device).status==200&&action(reboot)==200,"device survives a controller restart");
 // A new administrator token revokes every remembered device.
 {uint8_t next[32];matdog::update::Sha256 sha;std::string t2(64,'c');sha.update(reinterpret_cast<const uint8_t*>(t2.data()),64);sha.finish(next);reboot.service.gate.setDigest(next);}
 reboot.portal_.reset();clock_ms+=2000;check(resume(reboot,"md_device="+device).status==403,"token change revokes the device");
 {uint8_t zero[32]{};HttpTransport blank;blank.service.gate.setDigest(zero);check(resume(blank,"md_device="+device).status==403,"no administrator: nothing is remembered");}
 // Explicit logout.
 clock_ms+=2000;check(resume(dev,"md_device="+device).status==200,"session before logout");
 live=dev.portal_.cookie();live_csrf=dev.portal_.csrf();
 auto logout=[&](const std::string &cookies){auto r=request(dev,"/wifi/logout","");if(!cookies.empty())r.headers["Cookie"]=cookies;run(r);return r;};
 check(logout("md_session="+std::string(64,'0')).status==200&&action(dev)==200,"another client cannot end the session");
 endpoints(true,0xc0a80401u,0xc0a80403u);logout("md_session="+live);endpoints();check(action(dev)==200,"another peer cannot end the session");
 auto out=logout("md_session="+live);
 check(out.status==200&&out.response=="LOGGED_OUT","logout accepted");
 check(out.response_headers["Set-Cookie"].find("md_session=; HttpOnly; SameSite=Strict; Path=/wifi/; Max-Age=0")!=npos&&out.response_headers["Set-Cookie"].find("md_device=; HttpOnly; SameSite=Strict; Path=/wifi/session; Max-Age=0")!=npos,"logout clears both cookies");
 writes=dev.service.mutations;check(action(dev,"/wifi/ap","on=1",live.c_str(),live_csrf.c_str())==403&&dev.service.mutations==writes,"session ended by logout");
 {auto r=request(dev,"/wifi/logout","");r.headers["Origin"]="http://evil.invalid";run(r);check(r.status==403,"logout requires the AP origin");}
 printf("AP_ONLY_TRANSPORT_PASS: %u checks (production paths; HTTPD/TLS/driver doubles)\n",checks);
}
'''
helpers=between(c,'\nbool readHexHeader','\nvoid jsonString');helpers=helpers[helpers.index('\n}')+2:]
types='enum class HttpRequestKind'+between(h,'enum class HttpRequestKind','// Two optional listeners')
handler='esp_err_t HttpTransport::handleWifi'+between(c,'esp_err_t HttpTransport::handleWifi','\n\n} // namespace network')
cases='case HttpRequestKind::WIFI_STATUS:'+between(c,'case HttpRequestKind::WIFI_STATUS:','case HttpRequestKind::OTA_CHALLENGE:')
lifecycle='void HttpTransport::lifecycleTick()'+between(c,'void HttpTransport::lifecycleTick()','bool HttpTransport::admitAdmin(')
admit='bool HttpTransport::admitAdmin('+between(c,'bool HttpTransport::admitAdmin(','bool HttpTransport::dispatch(')
register='bool HttpTransport::registerHandlers'+between(c,'bool HttpTransport::registerHandlers','void HttpTransport::lifecycle()')
page='esp_err_t HttpTransport::handlePage'+between(c,'esp_err_t HttpTransport::handlePage','esp_err_t HttpTransport::handleWifi')
verify='bool WifiManager::verifyAdminRadio()'+between(w,'bool WifiManager::verifyAdminRadio()','// Arduino delivers role events asynchronously.')
unit=fixture.replace('// PRODUCTION_TYPES',types).replace('// PRODUCTION_HELPERS',helpers+register+page+lifecycle+admit+verify).replace('// PRODUCTION_HANDLER',handler).replace('// PRODUCTION_SERVICE_CASES',cases)
with tempfile.TemporaryDirectory(prefix='matdog-ap-only-') as d:
    cpp,exe=Path(d)/'probe.cpp',Path(d)/'probe';cpp.write_text(unit)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1','-pthread','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(src),'-I',str(here/'network_stubs'),'-I',str(here/'nvs_stub'),str(cpp),str(src/'network/AdminAccess.cpp'),str(src/'network/PortalSecurity.cpp'),str(src/'network/NetworkConfig.cpp'),str(src/'update/Sha256.cpp'),str(src/'update/Hmac256.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
