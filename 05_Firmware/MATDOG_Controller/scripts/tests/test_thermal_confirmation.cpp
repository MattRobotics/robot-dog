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
int main() {
  CHECK(kThermalLimitC==70 && kThermalConfirmationReads==5 && kThermalConfirmedOverLimit==3);
  for (int t : {0,34,69,70}) {
    Port p; auto r=confirmPresentTemperature(&p,21,t);
    CHECK(r.decision==ThermalDecision::NORMAL && r.published_c==t && p.reads==0);
  }
  struct Case { int trigger; std::vector<int32_t> samples; ThermalDecision verdict; int published; unsigned reads; };
  for (const auto& c : std::vector<Case>{
      {95,{117,34,34,34},ThermalDecision::TRANSIENT,34,4},
      {150,{34,34,34},ThermalDecision::TRANSIENT,34,3},
      {71,{71,71},ThermalDecision::CONFIRMED,71,2},
      {95,{117,34,72},ThermalDecision::CONFIRMED,117,3},
      {95,{34,75,34,80},ThermalDecision::CONFIRMED,95,4},
      {95,{34,-1000},ThermalDecision::CONFIRMATION_READ_FAILED,95,2},
      {95,{34,-1},ThermalDecision::CONFIRMATION_READ_FAILED,95,2},
      {95,{34,256},ThermalDecision::CONFIRMATION_READ_FAILED,95,2},
      {95,{117,34,34},ThermalDecision::CONFIRMATION_READ_FAILED,95,4},
  }) {
    Port p;p.values=c.samples;auto r=confirmPresentTemperature(&p,21,c.trigger);
    CHECK(r.decision==c.verdict && r.published_c==c.published);
    CHECK(p.reads==c.reads && p.waited==50*p.reads && r.sample_count<=5);
  }
  for (int t : {-1,256}) { Port p;auto r=confirmPresentTemperature(&p,21,t);CHECK(r.decision==ThermalDecision::CONFIRMATION_READ_FAILED && r.published_c<0 && p.reads==0); }
  CHECK(confirmPresentTemperature(nullptr,21,95).decision==ThermalDecision::CONFIRMATION_READ_FAILED);
  ThermalConfirmationState state;
  Port p;p.values={117,34,34,34};
  auto r=state.update(&p,21,95,1000); CHECK(state.pending() && p.reads==0 && p.waited==0);
  r=state.update(&p,21,34,1049);CHECK(state.pending() && p.reads==0);
  for(unsigned t : {1050,1100,1150,1200}) r=state.update(&p,21,34,t);
  CHECK(r.decision==ThermalDecision::TRANSIENT && r.sample_count==5 && p.waited==0);
  for(unsigned attempt=0;attempt<2;++attempt) {
    p.values={34,34,34};p.next=0;
    r=state.update(&p,21,95,2000+attempt*1000);
    for(unsigned i=1;i<=3;++i) r=state.update(&p,21,34,2000+attempt*1000+i*50);
  }
  CHECK(r.decision==ThermalDecision::REPEATED_ANOMALY && r.published_c>70);
  auto reads=p.reads;r=state.update(&p,21,34,60000);CHECK(r.decision==ThermalDecision::REPEATED_ANOMALY && p.reads==reads);
  ThermalConfirmationState separate;
  p.values={34,34,34};p.next=0;r=separate.update(&p,21,95,0);
  r=separate.update(&p,22,34,50);CHECK(r.decision==ThermalDecision::CONFIRMATION_READ_FAILED);
  ThermalConfirmationState deadline;
  p.next=0;reads=p.reads;
  r=deadline.update(&p,21,95,100);
  r=deadline.update(&p,21,34,400);
  CHECK(r.decision==ThermalDecision::CONFIRMATION_READ_FAILED && r.published_c==95 && p.reads==reads);
  ThermalConfirmationState sparse;
  for (unsigned incident=0;incident<8;++incident) {
    p.next=0;p.values={34,34,34};const unsigned start=incident*31000;
    r=sparse.update(&p,21,95,start);
    for(unsigned i=1;i<=3;++i) r=sparse.update(&p,21,34,start+i*50);
    CHECK(r.decision==(incident==7 ? ThermalDecision::REPEATED_ANOMALY : ThermalDecision::TRANSIENT));
    if(incident<7) CHECK(sparse.update(&p,21,34,start+200).decision==ThermalDecision::NORMAL);
  }
  std::printf("test_thermal_confirmation: %d checks, %d failures\n",checks,failures);
  return failures ? 1:0;
}
