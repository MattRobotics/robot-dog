// Core Motion V5: bounded servo UART transactions, STOP priority, ownership.
// PRODUCTION ServoBus + installed SCServo SDK against a virtual-time UART model
// (scripts/tests/servo_timed_stub). Every duration printed is SIMULATED virtual
// time of the model, not an ESP32 WCET and not a hardware measurement.
#include <cinttypes>
#include <cstring>
#include "../../src/servo/ServoBus.h"
#include "../../src/servo/ServoBusActuatorBackend.h"
using namespace matdog;
using namespace matdog::servo;
static int checks=0,failures=0;
#define CHECK(c) do{++checks;if(!(c)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#c);}}while(0)
namespace {
constexpr uint8_t kId=11;
struct Rig {
  ServoBus bus;HardwareSerial* uart;
  explicit Rig(uint32_t timeout_ms=8){CHECK(bus.begin());bus.setValidationDrive(20,5);bus.setValidationTimeout(timeout_ms);uart=HardwareSerial::bus;
    for(unsigned i=0;i<71;++i)uart->regs[kId][i]=static_cast<uint8_t>(i);
    uart->regs[kId][40]=1;host_us=1000000;}
  uint64_t timeIt(bool (*op)(Rig&),bool* result){const uint64_t t0=host_us;*result=op(*this);return host_us-t0;}
};
bool fastRead(Rig& r){ServoBus::FastFeedback fb;return r.bus.readFastFeedback(kId,&fb,8);}
// ---- stop preemption model -------------------------------------------------
struct StopAt { uint64_t at_us; unsigned polls=0; };
bool stopPreempt(void* ctx){auto* s=static_cast<StopAt*>(ctx);++s->polls;return host_us>=s->at_us;}
bool reentrantProbe(void* ctx){auto* r=static_cast<Rig*>(ctx);r->bus.ping(kId);return false;}   // a second bus user inside a wait
struct Rng{uint64_t s;uint32_t next(){s=s*6364136223846793005ull+1442695040888963407ull;return static_cast<uint32_t>(s>>33);}};
}
// Stock SDK, exactly as before V5: SCSerial::readSCS inter-byte timeout.
struct Stock : SMS_STS {};
int main(){
  // ===== 1. AS-IS: the SDK timeout is inter-byte (the V4 counterexample) =====
  {
    HardwareSerial uart;uart.plan.latency_us=60;uart.plan.gap_us=7000;               // 7 ms between bytes, IOTimeOut 8 ms
    for(unsigned i=0;i<71;++i)uart.regs[kId][i]=static_cast<uint8_t>(i);
    Stock stock;stock.pSerial=&uart;stock.IOTimeOut=8;uint8_t data[15];host_us=1000000;
    const uint64_t t0=host_us;const int n=stock.Read(kId,56,data,15);const uint64_t elapsed=host_us-t0;
    std::printf("BUS_AS_IS stock_sdk read15 gap_ms=7 IOTimeOut_ms=8 result=%d elapsed_ms=%.1f (inter-byte timeout, not a deadline) simulated\n",n,elapsed/1000.0);
    CHECK(elapsed>100000);   // the stock driver occupies the caller far beyond its 8 ms setting
  }
  // ===== 2. TO-BE: same stimulus, absolute deadline =====
  {
    Rig r;r.uart->plan.gap_us=7000;bool ok;const uint64_t elapsed=r.timeIt(fastRead,&ok);
    std::printf("BUS_TO_BE slow_progressive gap_ms=7 budget_ms=8 ok=%d outcome=%s elapsed_ms=%.3f simulated\n",ok,toString(r.bus.lastBusOutcome()),elapsed/1000.0);
    CHECK(!ok);CHECK(elapsed<=8000+100);CHECK(r.bus.lastBusOutcome()==BusOutcome::INCOMPLETE);
  }
  // ===== 3. gap sweep: success iff the whole reply fits the budget =====
  for(uint32_t gap:{10u,100u,300u,350u,400u,1000u,4000u,7000u,12000u}){
    Rig r;r.uart->plan.gap_us=gap;bool ok;const uint64_t elapsed=r.timeIt(fastRead,&ok);
    const uint64_t reply_us=60+20ull*gap+gap;   // 21 bytes
    CHECK(elapsed<=8000+100);
    if(reply_us+200<8000)CHECK(ok);
    if(reply_us>8000+200)CHECK(!ok);
    std::printf("BUS_GAP gap_us=%u ok=%d outcome=%s elapsed_us=%llu budget_us=8000\n",gap,ok,toString(r.bus.lastBusOutcome()),(unsigned long long)elapsed);
  }
  // ===== 4. silent servo, truncated, corrupt, wrong id =====
  {
    Rig r;r.uart->plan.silent=true;bool ok;const uint64_t e=r.timeIt(fastRead,&ok);
    CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::NO_RESPONSE&&e<=8100&&e>=7900);
    std::printf("BUS_SILENT outcome=%s elapsed_us=%llu\n",toString(r.bus.lastBusOutcome()),(unsigned long long)e);
    // recovery: next read is clean
    r.uart->plan=HardwareSerial::ReplyPlan{};CHECK(fastRead(r));CHECK(r.bus.lastBusOutcome()==BusOutcome::OK);
  }
  {Rig r;r.uart->plan.truncate_to=12;bool ok;r.timeIt(fastRead,&ok);CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::INCOMPLETE);
   r.uart->plan=HardwareSerial::ReplyPlan{};CHECK(fastRead(r));}                       // stale RX drained, next read valid
  {Rig r;r.uart->plan.corrupt_checksum=true;bool ok;const uint64_t e=r.timeIt(fastRead,&ok);
   CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::CORRUPT&&e<2000);}
  {Rig r;r.uart->plan.wrong_id=true;bool ok;r.timeIt(fastRead,&ok);CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::CORRUPT);}
  // ===== 5. total budget across several replies; no TX after expiry =====
  {
    Rig r;r.uart->plan.latency_us=60000;                                               // every reply takes 60 ms (< 100 ms phase)
    ServoBus::RuntimeState st;const uint64_t t0=host_us;const bool ok=r.bus.readRuntimeState(kId,&st);const uint64_t e=host_us-t0;
    const size_t sent=r.uart->requests.size();
    std::printf("BUS_TOTAL_BUDGET composite_read replies_ms=60 phase_ms=100 total_ms=300 ok=%d outcome=%s elapsed_ms=%.1f requests_sent=%zu of 7 simulated\n",ok,toString(r.bus.lastBusOutcome()),e/1000.0,sent);
    CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::TOTAL_DEADLINE);CHECK(e<=300000+300);CHECK(sent<7);
    CHECK(r.bus.busStats().suppressed_tx>0);
  }
  // ===== 6. STOP pending: preempts non-safety waits, never SAFE_OFF =====
  {
    Rig r;r.uart->plan.silent=true;StopAt stop{host_us+1000};stop.at_us=host_us+1000;r.bus.setStopPreempt(&stopPreempt,&stop);
    stop.at_us=host_us+1000;bool ok;const uint64_t e=r.timeIt(fastRead,&ok);
    std::printf("BUS_STOP_DURING_READ stop_after_us=1000 outcome=%s elapsed_us=%llu polls=%u (budget 8000) simulated\n",toString(r.bus.lastBusOutcome()),(unsigned long long)e,stop.polls);
    CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::ABORTED_STOP&&e<=1000+100);
  }
  {
    Rig r;StopAt stop{0};r.bus.setStopPreempt(&stopPreempt,&stop);                    // STOP already pending
    const size_t before=r.uart->requests.size();
    CHECK(!fastRead(r));CHECK(r.uart->requests.size()==before);                       // no request leaves the UART
    CHECK(r.bus.enableTorqueOn(kId)==ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE);  // never energize after STOP
    CHECK(r.uart->requests.size()==before);
    CHECK(r.bus.writeGoalPosition(kId,2048,GoalMotionProfile::BOUNDED)==ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE);
    CHECK(r.uart->requests.size()==before);
    // SAFE_OFF is NOT preemptible: torque goes to zero and is read back.
    r.uart->regs[kId][40]=1;
    CHECK(r.bus.safeOff(kId)==SafeOffResult::VERIFIED_OFF);CHECK(r.uart->regs[kId][40]==0);
    r.uart->regs[kId][40]=1;CHECK(r.bus.verifyTorqueOff(kId,8)==SafeOffResult::VERIFY_FAILED);   // readback runs under a pending STOP
    std::printf("BUS_STOP_PENDING reads_refused=1 torque_on_refused=1 goal_write_refused=1 safe_off=VERIFIED_OFF (not preempted)\n");
  }
  // ===== 7. a slow servo during SAFE_OFF is still bounded =====
  {
    Rig r;r.uart->plan.silent=true;StopAt stop{0};r.bus.setStopPreempt(&stopPreempt,&stop);
    const uint64_t t0=host_us;const auto res=r.bus.safeOff(kId);const uint64_t e=host_us-t0;
    std::printf("BUS_SAFE_OFF_SILENT result=%s elapsed_us=%llu budget_us=16000 simulated\n",toString(res),(unsigned long long)e);
    CHECK(res==SafeOffResult::UNVERIFIED_NO_RESPONSE&&e<=16000+300);
    // ...and a progressive reply cannot stretch the readback either
    Rig q;q.uart->plan.gap_us=7000;const uint64_t t1=host_us;(void)q.bus.safeOff(kId);CHECK(host_us-t1<=16000+300);
  }
  // ===== 8. 32-bit microsecond clock wrap =====
  {
    Rig r;r.uart->plan.gap_us=7000;host_us=0x100000000ull-3000;bool ok;const uint64_t e=r.timeIt(fastRead,&ok);
    CHECK(!ok&&e<=8100&&r.bus.lastBusOutcome()==BusOutcome::INCOMPLETE);
    Rig n;host_us=0x100000000ull-100;CHECK(fastRead(n)&&n.bus.lastBusOutcome()==BusOutcome::OK);   // nominal read straddling the wrap
    std::printf("BUS_CLOCK_WRAP slow_elapsed_us=%llu nominal_ok=1\n",(unsigned long long)e);
  }
  // ===== 9. ownership: no overlapping / foreign user =====
  {
    Rig r;r.uart->plan.silent=true;r.bus.setStopPreempt(&reentrantProbe,&r);
    const size_t before=r.uart->requests.size();bool ok;r.timeIt(fastRead,&ok);
    CHECK(r.bus.busStats().refused_reentrant>=1);CHECK(r.uart->requests.size()==before+1);   // only the owner's request was sent
    std::printf("BUS_OWNERSHIP nested_user_refused=%u requests=%zu\n",r.bus.busStats().refused_reentrant,r.uart->requests.size()-before);
    BusTransaction t;static uintptr_t token=1;t.bindClock(+[]()->uint32_t{return static_cast<uint32_t>(host_us);});
    t.bindOwner(+[]()->uintptr_t{return token;});CHECK(t.begin(1000,1000,false));t.end(true);
    token=2;CHECK(!t.begin(1000,1000,false));CHECK(t.outcome()==BusOutcome::REFUSED_FOREIGN_OWNER&&t.stats().refused_foreign==1);
    CHECK(!t.idleAndOwned());
    token=1;CHECK(t.begin(1000,1000,false));CHECK(!t.begin(1000,1000,false));CHECK(t.stats().refused_reentrant==1);t.end(true);
    // SCServo traffic outside a transaction is refused at the hook
    CHECK(!t.allowTx());CHECK(t.stats().unowned_io==1);
  }
  // ===== 10. babbling bus =====
  {
    Rig r;r.uart->flood=true;const size_t before=r.uart->requests.size();bool ok;const uint64_t e=r.timeIt(fastRead,&ok);
    CHECK(!ok&&r.bus.lastBusOutcome()==BusOutcome::RX_FLOOD&&r.uart->requests.size()==before&&e<=1000);
    ServoBus::RuntimeState st;CHECK(!r.bus.readRuntimeState(kId,&st));CHECK(r.uart->requests.size()==before);
    std::printf("BUS_RX_FLOOD outcome=RX_FLOOD requests_sent=0 elapsed_us=%llu\n",(unsigned long long)e);
  }
  // ===== 11. nominal 12-joint poll round and randomized invariants =====
  {
    Rig r;const uint64_t t0=host_us;unsigned ok=0;for(unsigned i=0;i<12;++i){ok+=fastRead(r)?1:0;}
    const uint64_t e=host_us-t0;std::printf("BUS_NOMINAL 12x fast read ok=%u elapsed_us=%llu per_read_us=%.1f model: latency 60us + 21 bytes x 10us simulated\n",ok,(unsigned long long)e,e/12.0);
    CHECK(ok==12);
  }
  {
    Rng rng{12345};uint64_t worst_over=0,sum=0;std::vector<uint64_t> el;unsigned okn=0,bad_accept=0;
    for(unsigned i=0;i<4000;++i){
      Rig r;auto& p=r.uart->plan;
      p.silent=rng.next()%11==0;p.latency_us=rng.next()%12000;p.gap_us=10+rng.next()%(rng.next()%2?200:3000);
      if(rng.next()%4==0){p.slow_after=rng.next()%21;p.slow_gap_us=rng.next()%15000;}
      if(rng.next()%6==0)p.truncate_to=rng.next()%21;
      if(rng.next()%8==0)p.corrupt_checksum=true;
      ServoBus::FastFeedback fb;const uint64_t t0=host_us;const bool ok=r.bus.readFastFeedback(kId,&fb,8);const uint64_t e=host_us-t0;
      el.push_back(e);sum+=e;if(e>8000)worst_over=std::max<uint64_t>(worst_over,e-8000);
      CHECK(e<=8000+100);
      if(ok){++okn;CHECK(!p.silent&&!p.corrupt_checksum&&p.truncate_to<0&&fb.present_position==(56|(57<<8))
          && r.bus.lastBusOutcome()==BusOutcome::OK);if(fb.present_position!=(56|(57<<8)))++bad_accept;}
      else CHECK(r.bus.lastBusOutcome()!=BusOutcome::OK);
    }
    std::sort(el.begin(),el.end());
    std::printf("BUS_RANDOM runs=4000 budget_us=8000 accepted=%u false_accepts=%u elapsed_p50_us=%llu p99_us=%llu max_us=%llu max_overrun_us=%llu simulated\n",
      okn,bad_accept,(unsigned long long)el[2000],(unsigned long long)el[3960],(unsigned long long)el.back(),(unsigned long long)worst_over);
    CHECK(bad_accept==0);
  }
  std::printf("SERVO_BUS_DEADLINE %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
  return failures?1:0;
}
