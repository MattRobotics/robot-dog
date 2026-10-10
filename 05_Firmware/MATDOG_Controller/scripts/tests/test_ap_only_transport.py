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
struct httpd_config_t {int stack_size,max_uri_handlers,max_open_sockets,recv_wait_timeout,send_wait_timeout,task_priority;};
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
 registered.clear();check(s.registerHandlers(nullptr,true)&&registered.size()==10,"HTTP admin route inventory");
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
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1','-pthread','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(src),'-I',str(here/'network_stubs'),'-I',str(here/'nvs_stub'),str(cpp),str(src/'network/AdminAccess.cpp'),str(src/'network/PortalSecurity.cpp'),str(src/'network/NetworkConfig.cpp'),str(src/'update/Sha256.cpp'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
