#include <cstdio>
#include <vector>
#include "../../src/calibration/ThermalConfirmation.h"
using namespace matdog::calibration;
static int checks=0, failures=0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #x); } } while(0)
struct Port : ThermalReadPort {
  std::vector<int32_t> values;
  unsigned next=0, reads=0, waited=0;
  uint8_t expected=21;
  bool readPresentTemperatureDirect(uint8_t bus, int32_t* out) override {
    ++reads; CHECK(bus==expected);
    if (next==values.size()) return false;
    *out=values[next++]; return *out!=-1000;
  }
  void delayMs(uint32_t ms) override { waited+=ms; CHECK(ms==50); }
};
// One block-read value over the limit followed by the given direct reads,
// 50 ms apart, through the production (non-blocking) state.
static ThermalConfirmation incident(ThermalConfirmationState& state, Port& p, int bulk,
                                    std::vector<int32_t> direct, uint32_t start) {
  p.values=direct;p.next=0;
  ThermalConfirmation r=state.update(&p,21,bulk,start);
  for (unsigned i=1;state.pending() && i<=8;++i) r=state.update(&p,21,34,start+i*50);
  return r;
}
int main() {
  // G: the limit and the confirmation majority are unchanged.
  CHECK(kThermalLimitC==70 && kThermalConfirmationReads==5 && kThermalConfirmedOverLimit==3);
  CHECK(kThermalDirectNormalToClear==3 && kThermalConfirmationDelayMs==50);
  // F: a normal block-read value is published as is and costs no direct read.
  for (int t : {0,32,35,69,70}) {
    Port p; auto r=confirmPresentTemperature(&p,21,t);
    CHECK(r.decision==ThermalDecision::NORMAL && r.published_c==t && p.reads==0);
  }
  struct Case { int trigger; std::vector<int32_t> samples; ThermalDecision verdict; int published; unsigned reads; };
  for (const auto& c : std::vector<Case>{
      // A, B: the 2026-10-06 bus 12 records. A refuted block-read value is no fault.
      {95,{32,32,32},ThermalDecision::BULK_ARTIFACT,32,3},
      {78,{32,33,32},ThermalDecision::BULK_ARTIFACT,32,3},
      {77,{34,32,32},ThermalDecision::BULK_ARTIFACT,32,3},
      {150,{34,34,34},ThermalDecision::BULK_ARTIFACT,34,3},
      // One direct sample can be an artifact too (2026-10-03: 95,117,34).
      {95,{117,34,34,34},ThermalDecision::BULK_ARTIFACT,34,4},
      // D: over-temperature needs three DIRECT samples over the limit.
      {71,{71,71,71},ThermalDecision::CONFIRMED,71,3},
      {95,{75,34,72,80},ThermalDecision::CONFIRMED,80,4},
      {75,{75,75,75},ThermalDecision::CONFIRMED,75,3},
      // The block-read value is never counted: two direct hot samples do not confirm...
      {255,{71,71,34,34},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,4},
      // ...and direct samples that stay incoherent fail closed, they do not clear.
      {95,{34,75,34,80},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,4},
      // E: a direct read that fails or is invalid fails closed.
      {95,{34,-1000},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,2},
      {95,{34,-1},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,2},
      {95,{34,256},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,2},
      {95,{},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,1},
      {95,{117,34,34},ThermalDecision::THERMAL_TELEMETRY_FAULT,71,4},
  }) {
    Port p;p.values=c.samples;auto r=confirmPresentTemperature(&p,21,c.trigger);
    CHECK(r.decision==c.verdict && r.published_c==c.published);
    CHECK(p.reads==c.reads && p.waited==50*p.reads && r.sample_count<=5);
    CHECK(r.samples[0]==c.trigger);  // the raw block-read value is preserved as evidence
    // Every verdict that is not a clean pass publishes a value over the limit.
    if (c.verdict!=ThermalDecision::BULK_ARTIFACT) CHECK(r.published_c>kThermalLimitC);
    else CHECK(r.published_c<=kThermalLimitC && r.bulk_artifacts==1);
  }
  for (int t : {-1,256}) { Port p;auto r=confirmPresentTemperature(&p,21,t);CHECK(r.decision==ThermalDecision::THERMAL_TELEMETRY_FAULT && r.published_c<0 && p.reads==0); }
  CHECK(confirmPresentTemperature(nullptr,21,95).decision==ThermalDecision::THERMAL_TELEMETRY_FAULT);
  CHECK(confirmPresentTemperature(nullptr,21,95).published_c>kThermalLimitC);

  // Non-blocking production path: no read before 50 ms, no sleep, one read per call.
  ThermalConfirmationState state;
  Port p;p.values={117,34,34,34};
  auto r=state.update(&p,21,95,1000); CHECK(state.pending() && p.reads==0 && p.waited==0);
  r=state.update(&p,21,34,1049);CHECK(state.pending() && p.reads==0);
  for(unsigned t : {1050,1100,1150,1200}) r=state.update(&p,21,34,t);
  CHECK(r.decision==ThermalDecision::BULK_ARTIFACT && r.sample_count==5 && p.waited==0 && p.reads==4);

  // C: ten refuted block-read spikes inside 30 s never become a safety verdict.
  {
    ThermalConfirmationState dense; Port q;
    const int spikes[10]={95,78,77,76,126,91,150,111,84,72};
    const int direct[4]={32,33,34,35};
    for (unsigned i=0;i<10;++i) {
      const int d=direct[i%4];
      r=incident(dense,q,spikes[i],{d,d,d},i*2900);
      CHECK(r.decision==ThermalDecision::BULK_ARTIFACT && r.published_c==d);
      CHECK(r.bulk_artifacts==i+1 && dense.bulkArtifacts()==i+1);
      // Straight after each one the servo is plainly normal again.
      r=dense.update(&q,21,d,i*2900+200);
      CHECK(r.decision==ThermalDecision::NORMAL && r.published_c==d && !dense.pending());
    }
    // ...and a real overheat after all of them is still caught.
    r=incident(dense,q,75,{75,76,75},40000);
    CHECK(r.decision==ThermalDecision::CONFIRMED && r.published_c==76);
  }
  // Far more than the former eight-per-boot bound, spread over a long run: still diagnostic.
  {
    ThermalConfirmationState sparse; Port q;
    for (unsigned i=0;i<40;++i) {
      r=incident(sparse,q,95,{34,34,34},i*31000);
      CHECK(r.decision==ThermalDecision::BULK_ARTIFACT);
    }
    CHECK(sparse.bulkArtifacts()==40);
  }
  // A confirmation is bound to its servo and to its 300 ms deadline.
  ThermalConfirmationState separate;
  p.values={34,34,34};p.next=0;r=separate.update(&p,21,95,0);
  r=separate.update(&p,22,34,50);
  CHECK(r.decision==ThermalDecision::THERMAL_TELEMETRY_FAULT && r.published_c>kThermalLimitC);
  ThermalConfirmationState deadline;
  p.next=0;auto reads=p.reads;
  r=deadline.update(&p,21,95,100);
  r=deadline.update(&p,21,34,400);
  CHECK(r.decision==ThermalDecision::THERMAL_TELEMETRY_FAULT && r.published_c>kThermalLimitC && p.reads==reads);
  // A fault is not sticky state either: the next sample is judged on its own.
  r=deadline.update(&p,21,34,500);
  CHECK(r.decision==ThermalDecision::NORMAL && r.published_c==34);
  std::printf("test_thermal_confirmation: %d checks, %d failures\n",checks,failures);
  return failures ? 1:0;
}
