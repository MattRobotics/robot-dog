#!/usr/bin/env python3
"""Admission fence contention on the PRODUCTION mailbox path (RCA 2026-10-21).

Extracted unmodified from src/network/HttpTransport.cpp: dispatch() (httpd side),
the request-serving tail of update(), the serviceRequest admission block, its
MOTION_STATUS and MOTION_COMMAND cases and serviceStopLane(). Real AdminAccess
and HttpMailbox. FreeRTOS semaphores are thread-backed doubles, the md_http_life
fence holder, the httpd reader and the Controller loop are real threads (TSan).
Motion execution, portal session and radio facts are doubles. No device is opened.
"""
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent / 'src'
c = (SRC/'network/HttpTransport.cpp').read_text()
def between(text, start, end):
    assert text.count(start) == 1 and text.count(end) == 1, (start, end)
    return text.split(start, 1)[1].split(end, 1)[0]
dispatch = 'bool HttpTransport::dispatch(' + between(c, 'bool HttpTransport::dispatch(', 'void HttpTransport::appendTransportStats')
tail = '  if (xSemaphoreTake(request_ready_, 0) != pdTRUE)\n    return;\n' + between(c, '  if (xSemaphoreTake(request_ready_, 0) != pdTRUE)\n    return;\n', '\nvoid HttpTransport::serviceRequest')
admission = between(c, 'void HttpTransport::serviceRequest(uint32_t now_ms) {\n', '  switch (req.kind) {')
status_case = 'case HttpRequestKind::MOTION_STATUS:' + between(c, 'case HttpRequestKind::MOTION_STATUS:', 'case HttpRequestKind::MOTION_CAPABILITIES:')
command_case = 'case HttpRequestKind::MOTION_COMMAND: {' + between(c, 'case HttpRequestKind::MOTION_COMMAND: {', 'case HttpRequestKind::CALIBRATION_RESTORE: {')
stop_lane = 'bool HttpTransport::serviceStopLane(' + between(c, 'bool HttpTransport::serviceStopLane(', 'esp_err_t HttpTransport::handleCalibrationRestore')
fixture = r'''
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <freertos/semphr.h>
#include "network/AdminAccess.h"
#include "network/HttpMailbox.h"
#include "network/NetworkPolicy.h"
#include "motion/MotionContract.h"
// ---- platform doubles: thread-backed FreeRTOS semaphores, monotonic clock ----
static const auto t0 = std::chrono::steady_clock::now();
uint32_t millis() { return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count()); }
int64_t esp_timer_get_time() { return static_cast<int64_t>(millis())*1000; }
struct Sem { std::mutex m; std::condition_variable cv; bool available; };
SemaphoreHandle_t xSemaphoreCreateMutex() { return new Sem{{},{},true}; }   // AdminAccess: non-recursive, take(...,0)
SemaphoreHandle_t xSemaphoreCreateBinary() { return new Sem{{},{},false}; }
void vSemaphoreDelete(SemaphoreHandle_t h) { delete static_cast<Sem*>(h); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t h, TickType_t ms) {
  auto* s = static_cast<Sem*>(h); std::unique_lock<std::mutex> l(s->m);
  if (!s->cv.wait_for(l, std::chrono::milliseconds(ms), [&]{ return s->available; })) return 0;
  s->available = false; return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t h) { auto* s = static_cast<Sem*>(h); { std::lock_guard<std::mutex> l(s->m); s->available = true; } s->cv.notify_all(); return pdTRUE; }
void vTaskDelay(TickType_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
namespace matdog { namespace network {
std::atomic<bool> g_radio_ok{true};
class WifiManager { public: static bool verifyAdminRadio(const char** failure=nullptr) { if(!g_radio_ok.load()&&failure)*failure="AP_NOT_EXCLUSIVE"; return g_radio_ok.load(); } };
enum class HttpRequestKind : uint8_t { NONE, WEB_STATUS, OTA_CHALLENGE, OTA_BEGIN, OTA_CHUNK, OTA_FINISH, OTA_ABORT, WIFI_STATUS, WIFI_LOGIN,
  WIFI_CONFIG, WIFI_SCAN, WIFI_AP, WIFI_SESSION, WIFI_LOGOUT, MOTION_STATUS, MOTION_CAPABILITIES, MOTION_COMMAND, MOTION_LOGIN, MOTION_SESSION,
  MOTION_LOGOUT, CALIBRATION_RESTORE };
struct HttpTransportRequest { HttpRequestKind kind=HttpRequestKind::NONE; uint32_t sequence=0; bool local_authorized=false,tls=false,ap_socket=false;
  uint32_t peer=0,admin_generation=0,local_ipv4=0; char cookie[65]{},csrf[65]{}; motion::MotionCommand motion_command{}; uint32_t stop_epoch=0; };
struct HttpTransportResponse { bool ok=false; const char* message=""; char text[512]{}; size_t text_len=0; uint32_t sequence=0; };
constexpr size_t kMotionCapabilitiesBytes=64;
struct FakeService {
  AdminAccess gate;
  struct { bool run_mode=false,connected=false; uint32_t ipv4=0; uint8_t role_reason=0; } wifi;
  std::atomic<unsigned> statuses{0},commands{0},stops{0};
  FakeService() { gate.begin(); gate.open(); }
  AdminAccess& networkAdminAccess() { return gate; }
  const decltype(wifi)& wifiStatus() const { return wifi; }
  void operatorHeartbeat(uint32_t) {}
  bool motionStatus(char* out,size_t cap,uint32_t) { ++statuses; return std::snprintf(out,cap,"{\"version\":1}")>0; }
  motion::MotionAck motionCommand(const motion::MotionCommand& m,uint32_t) { (m.intent==motion::Intent::STOP?stops:commands)++; return motion::MotionAck{}; }
  bool motionAck(const motion::MotionAck&,char* out,size_t cap) { return std::snprintf(out,cap,"{\"ack\":true}")>0; }
};
struct FakePortal {   // the AP admin session: valid or not, decided by the test
  std::atomic<bool> valid{true};
  void reset() {}
  bool current(uint64_t,uint32_t,const char*) const { return valid.load(); }
  bool authorize(uint64_t,uint32_t,const char*,const char*) { return valid.load(); }
};
class HttpTransport {
 public:
  FakeService service; FakeService* service_=&service; FakePortal portal_;
  uint32_t portal_generation_=0; bool portal_sta_=false;
  SemaphoreHandle_t request_ready_=xSemaphoreCreateBinary(), response_ready_=xSemaphoreCreateBinary(), slot_free_=xSemaphoreCreateBinary();
  HttpMailbox mailbox_{};
  bool deferred_=false;
  HttpTransportRequest pending_request_{}; HttpTransportResponse pending_response_{};
  std::atomic<uint32_t> sequence_{0},ready_sequence_{0},dispatch_last_ms_{0},dispatch_max_ms_{0},dispatch_late_{0},dispatch_busy_{0};
  char motion_capabilities_[kMotionCapabilitiesBytes]{};
  std::atomic<bool> stop_lock_{false},stop_pending_{false},stop_ack_ok_{false};
  HttpTransportRequest stop_request_{}; char stop_ack_[128]{};
  std::atomic<uint32_t> stop_epoch_{0},stop_done_{0},stop_ticket_{0};
  static constexpr uint32_t kDispatchTimeoutMs=3000;
  HttpTransport() { xSemaphoreGive(slot_free_); }
  ~HttpTransport() { vSemaphoreDelete(request_ready_); vSemaphoreDelete(response_ready_); vSemaphoreDelete(slot_free_); }
  bool readOnlyAdmitted(const HttpTransportRequest&) const { return false; }   // AP admin path only
  bool motionStaAdmitted(const HttpTransportRequest&) const { return false; }
  bool motionStaAdmitted(bool,uint32_t,uint32_t,const char** =nullptr) const { return false; }
  void appendTransportStats(char*,size_t) {}
  bool dispatch(const HttpTransportRequest&,HttpTransportResponse*,char*,size_t,uint32_t,const char**);
  void updateTail(uint32_t now_ms);
  void serviceRequest(uint32_t now_ms);
  bool serviceStopLane(uint32_t now_ms);
};
// PRODUCTION_DISPATCH
void HttpTransport::updateTail(uint32_t now_ms) {
  (void)now_ms;
// PRODUCTION_UPDATE_TAIL
void HttpTransport::serviceRequest(uint32_t now_ms) {
// PRODUCTION_ADMISSION
  switch (req.kind) {
// PRODUCTION_STATUS_CASE
// PRODUCTION_COMMAND_CASE
  default: resp.message="UNSUPPORTED_IN_FIXTURE"; break;
  }
}
// PRODUCTION_STOP_LANE
} }
using namespace matdog;
using namespace matdog::network;
int checks=0,failures=0;
void check(bool ok,const char* what){++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL %s\n",what);}}
// The Controller thread: STOP lane first, then the mailbox - as Controller::update().
struct Loop {
  HttpTransport& t; std::atomic<bool> run{true}; std::atomic<unsigned> passes{0}; std::thread th;
  explicit Loop(HttpTransport& x):t(x),th([this]{while(run.load()){t.serviceStopLane(millis());t.updateTail(millis());++passes;std::this_thread::sleep_for(std::chrono::milliseconds(1));}}){}
  void waitPasses(unsigned n){const unsigned s=passes.load();while(passes.load()<s+n)std::this_thread::sleep_for(std::chrono::microseconds(200));}
  ~Loop(){run.store(false);th.join();}
};
// md_http_life: holds the admission fence until released.
struct Holder {
  AdminAccess& g; std::mutex m; std::condition_variable cv; bool held=false,release=false; std::thread th;
  explicit Holder(AdminAccess& a):g(a),th([this]{AdminLease lease(g);std::unique_lock<std::mutex> l(m);held=bool(lease);cv.notify_all();cv.wait(l,[&]{return release;});}){
    std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return held;});}
  void drop(){{std::lock_guard<std::mutex> l(m);release=true;}cv.notify_all();if(th.joinable())th.join();}
  ~Holder(){drop();}
};
HttpTransportRequest apRequest(HttpTransport& t,HttpRequestKind k,motion::Intent intent=motion::Intent::STAND){
  HttpTransportRequest r;r.kind=k;r.ap_socket=true;r.local_authorized=true;r.peer=0x0204a8c0u;r.admin_generation=t.service.gate.generation();
  r.motion_command.intent=intent;r.stop_epoch=t.stop_epoch_.load();return r;
}
struct Result { bool ok=false; HttpTransportResponse resp{}; const char* failure=""; };
std::thread reader(HttpTransport& t,HttpTransportRequest r,uint32_t budget,Result* out){
  return std::thread([&t,r,budget,out]{out->ok=t.dispatch(r,&out->resp,nullptr,0,budget,&out->failure);});
}
uint32_t publishStop(HttpTransport& t){   // as handleMotion's STOP lane writer
  auto r=apRequest(t,HttpRequestKind::MOTION_COMMAND,motion::Intent::STOP);
  const uint32_t ticket=t.stop_ticket_.fetch_add(1)+1;r.sequence=ticket;
  while(t.stop_lock_.exchange(true)){}
  t.stop_request_=r;t.stop_pending_.store(true);t.stop_lock_.store(false);return ticket;
}
int main(){
  // 1. Valid Motion GET while md_http_life holds the fence: served when it is released, never a false 403.
  { HttpTransport t;Loop loop(t);Holder h(t.service.gate);Result r;auto th=reader(t,apRequest(t,HttpRequestKind::MOTION_STATUS),1200,&r);
    loop.waitPasses(20);check(t.service.statuses.load()==0,"GET not served while the fence is held");
    h.drop();th.join();
    check(r.ok&&r.resp.ok&&!std::strcmp(r.resp.message,"OK"),"GET recovered after contention (no ADMIN_AP_REQUIRED)");
    check(t.service.statuses.load()==1,"GET executed exactly once");
    std::printf("CONTENTION_GET ok=%d message=%s passes_held>=20\n",r.resp.ok,r.resp.message); }
  // 2. Contention longer than the reader budget: bounded 503, released unexecuted, mailbox reusable.
  { HttpTransport t;Loop loop(t);Result r;{Holder h(t.service.gate);auto th=reader(t,apRequest(t,HttpRequestKind::MOTION_STATUS),80,&r);th.join();
      check(!r.ok&&!std::strcmp(r.failure,"CONTROLLER_LOOP_LATE"),"held past the budget: CONTROLLER_LOOP_LATE, not 403");}
    loop.waitPasses(10);check(t.service.statuses.load()==0,"abandoned deferred request never executed late");
    Result again;auto th=reader(t,apRequest(t,HttpRequestKind::MOTION_STATUS),1200,&again);th.join();
    check(again.ok&&again.resp.ok&&t.service.statuses.load()==1,"slot released: next request served once"); }
  // 3. Generation change while deferred: genuine refusal, no wait to the budget, not executed.
  { HttpTransport t;Loop loop(t);Result r;{Holder h(t.service.gate);auto th=reader(t,apRequest(t,HttpRequestKind::MOTION_STATUS),3000,&r);
      loop.waitPasses(5);t.service.gate.revoke();t.service.gate.open();   // a new admin generation
      const uint32_t started=millis();th.join();
      check(r.ok&&!r.resp.ok&&!std::strcmp(r.resp.message,"ADMIN_AP_REQUIRED"),"stale generation still refused ADMIN_AP_REQUIRED");
      check(millis()-started<1000,"refused promptly, not at the budget");}
    check(t.service.statuses.load()==0,"refused request not executed"); }
  // 4. Radio not verifiable during contention: genuine refusal kept.
  { HttpTransport t;Loop loop(t);Result r;{Holder h(t.service.gate);g_radio_ok.store(false);
      auto th=reader(t,apRequest(t,HttpRequestKind::MOTION_STATUS),3000,&r);th.join();g_radio_ok.store(true);}
    check(!r.ok||(!r.resp.ok&&!std::strcmp(r.resp.message,"ADMIN_AP_REQUIRED")),"radio fault refused");check(t.service.statuses.load()==0,"radio fault never served"); }
  // 5. STOP during contention: stays pending (no ACK, not done), applied once the fence is free.
  { HttpTransport t;Loop loop(t);Holder h(t.service.gate);const uint32_t ticket=publishStop(t);
    loop.waitPasses(20);
    check(t.stop_pending_.load()&&t.stop_done_.load()!=ticket&&t.service.stops.load()==0,"STOP not consumed nor acknowledged while the fence is held");
    h.drop();while(t.stop_done_.load()!=ticket)std::this_thread::sleep_for(std::chrono::microseconds(200));
    check(t.stop_ack_ok_.load()&&t.service.stops.load()==1&&t.stop_epoch_.load()==1,"STOP applied (ACK only after application)");
    std::printf("CONTENTION_STOP applied=%u ack=%d epoch=%u\n",t.service.stops.load(),t.stop_ack_ok_.load(),t.stop_epoch_.load()); }
  // 6. STOP racing a Motion command under contention: STOP first, the command superseded, never executed.
  { HttpTransport t;Loop loop(t);Result cmd;std::thread th;
    {Holder h(t.service.gate);th=reader(t,apRequest(t,HttpRequestKind::MOTION_COMMAND),3000,&cmd);loop.waitPasses(5);publishStop(t);loop.waitPasses(5);
     check(t.service.commands.load()==0&&t.service.stops.load()==0,"neither runs while the fence is held");}
    th.join();
    check(t.service.stops.load()==1&&t.service.commands.load()==0,"STOP applied, earlier command never executed");
    check(cmd.ok&&!cmd.resp.ok&&!std::strcmp(cmd.resp.message,"SUPERSEDED_BY_STOP"),"command answered SUPERSEDED_BY_STOP"); }
  // 7. STOP precedence without fence contention: a command meets a STOP still in its lane (writer mid-copy).
  { HttpTransport t;Loop loop(t);Result cmd;const auto req=apRequest(t,HttpRequestKind::MOTION_COMMAND);
    t.stop_lock_.store(true);t.stop_pending_.store(true);   // the STOP writer holds its lane lock
    auto th=reader(t,req,3000,&cmd);loop.waitPasses(20);
    check(t.service.commands.load()==0,"command waits behind the pending STOP");
    t.stop_request_=apRequest(t,HttpRequestKind::MOTION_COMMAND,motion::Intent::STOP);t.stop_request_.sequence=t.stop_ticket_.fetch_add(1)+1;
    t.stop_lock_.store(false);th.join();
    check(t.service.stops.load()==1&&t.service.commands.load()==0&&!std::strcmp(cmd.resp.message,"SUPERSEDED_BY_STOP"),"STOP first, command superseded"); }
  // 8. Invalid session under contention: genuine refusals preserved for commands and STOP.
  { HttpTransport t;Loop loop(t);t.portal_.valid.store(false);Result cmd;std::thread th;uint32_t ticket=0;
    {Holder h(t.service.gate);th=reader(t,apRequest(t,HttpRequestKind::MOTION_COMMAND),3000,&cmd);ticket=publishStop(t);loop.waitPasses(5);}
    th.join();while(t.stop_done_.load()!=ticket)std::this_thread::sleep_for(std::chrono::microseconds(200));
    check(cmd.ok&&!cmd.resp.ok&&!std::strcmp(cmd.resp.message,"AUTH_FAILED"),"invalid session: command AUTH_FAILED");
    check(!t.stop_ack_ok_.load()&&t.service.stops.load()==0&&t.stop_epoch_.load()==0,"invalid session: STOP refused (handler 403), not applied");
    check(t.service.commands.load()==0,"nothing executed"); }
  // 9. No contention: ordinary request served on the first pass, once.
  { HttpTransport t;Loop loop(t);Result r;auto th=reader(t,apRequest(t,HttpRequestKind::MOTION_STATUS),1200,&r);th.join();
    check(r.ok&&r.resp.ok&&t.service.statuses.load()==1,"uncontended GET served once"); }
  std::printf("HTTP_FENCE_CONTENTION %s checks=%d failures=%d (production dispatch/update/admission/STOP lane; threads; no hardware)\n",failures?"FAIL":"PASS",checks,failures);
  return failures?1:0;
}
'''
fixture = (fixture.replace('// PRODUCTION_DISPATCH', dispatch).replace('// PRODUCTION_UPDATE_TAIL', tail)
           .replace('// PRODUCTION_ADMISSION', admission).replace('// PRODUCTION_STATUS_CASE', status_case)
           .replace('// PRODUCTION_COMMAND_CASE', command_case).replace('// PRODUCTION_STOP_LANE', stop_lane))
with tempfile.TemporaryDirectory(prefix='matdog-fence-') as d:
    cpp = Path(d)/'fence.cpp'; cpp.write_text(fixture); exe = Path(d)/'fence'
    import platform
    for sanitizer in ('thread','address,undefined'):
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-O1','-g','-pthread','-fsanitize='+sanitizer,
                        '-I',str(SRC),'-I',str(HERE/'network_stubs'),str(cpp),str(SRC/'network/AdminAccess.cpp'),'-o',str(exe)],check=True)
        # TSan needs the low-entropy layout on this kernel (as run_imu_feed_tests.sh).
        # AdminAccess's FreeRTOS mutex lives for the firmware lifetime (no destructor): one per scenario here.
        subprocess.run(['setarch',platform.machine(),'-R',str(exe)],check=True,timeout=120,
                       env={**__import__('os').environ,'ASAN_OPTIONS':'detect_leaks=0'})
