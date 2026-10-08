#include "../../src/calibration/StartupRecoveryQualification.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include <cassert>
#include <cstdio>
using namespace matdog;
using namespace matdog::calibration;

static void prepare(StartupRecoveryQualification& q) {
  assert(q.phase()==StartupQualificationPhase::IDLE);assert(!q.ready(100));assert(!q.nominal());
  assert(q.start(100));q.censusStarted();q.censusComplete();q.preflightStarted();
  PopulationEvidenceBuildResult pop{};pop.status=PopulationEvidenceBuildStatus::PASS;
  pop.evidence.evaluated=true;pop.evidence.origin=CalibrationOrigin::LIVE_SESSION;pop.evidence.observed_mask=0xFFF;
  q.population(pop);assert(q.phase()==StartupQualificationPhase::OBSERVING);
}
static void samples(StartupRecoveryQualification& q,int fault,bool nominal=false) {
  for (int i=0;i<36 && q.phase()==StartupQualificationPhase::OBSERVING;++i) {
    const auto* ref=startupReference(q.bus());
    actuator::TelemetrySample s{};s.read_ok=true;s.sampled_at_ms=200+i*15;
    s.present_position=ref->q0;s.torque_enable=0;s.present_speed=0;s.present_current=1;s.present_temperature=34;s.servo_status=0;
    if (!nominal) {if(ref->bus==21)s.present_position=2348;if(ref->bus==22)s.present_position=1080;if(ref->bus==32)s.present_position=1665;}
    if (i==14) {
      if(fault==1)s.read_ok=false;
      if(fault==2)s.present_position=4096;
      if(fault==3)s.present_temperature=-1;
      if(fault==4)s.present_temperature=95;
      if(fault==5)s.present_current=200;
      if(fault==6)s.torque_enable=1;
      if(fault==7)s.present_position+=15;
      if(fault==8)s.sampled_at_ms=0;
    }
    if(fault==9 && ref->bus==22)s.present_position=2106;
    q.observe(s,fault==8 ? 5000:s.sampled_at_ms);
  }
}
int main() {
  actuator::CalibrationGeometryProfile geometry;
  assert(!startupReferenceMatches(geometry));
  geometry.bind(&actuator::geometry_data::kProvenance,actuator::geometry_data::kJoints,12,
                actuator::geometry_data::kEndpoints,24);
  assert(startupReferenceMatches(geometry));
  assert(startupReference(22)->q0==2106 && startupReference(21)->direction==1);
  for (int fault=0;fault<=9;++fault) {
    StartupRecoveryQualification q;prepare(q);samples(q,fault);
    if(!fault) {assert(q.ready(800));assert(!q.nominal());assert(!q.ready(5000));assert(q.consumeForExecution(800));}
    else {assert(q.phase()==StartupQualificationPhase::REFUSED);assert(!q.ready(800));assert(!q.consumeForExecution(800));}
    assert(!q.start(1000));
  }
  StartupRecoveryQualification nominal;prepare(nominal);samples(nominal,0,true);
  assert(nominal.nominal() && nominal.ready(800));assert(!nominal.consumeForExecution(800));
  assert(!startupPositionInBand(32,1649,CalibrationPhase::RETURN_LOWER_HELD));
  assert(startupPositionInBand(32,1665,CalibrationPhase::RETURN_LOWER_HELD));
  assert(!startupPositionInBand(22,2106,CalibrationPhase::RETURN_LOWER_HELD));
  assert(startupPositionInBand(22,2106,CalibrationPhase::RESTORE_PARKING));
  std::puts("STARTUP_QUALIFICATION_TESTS=PASS (fresh 3x12, 9 refusals, nominal/no torque, scoped reference)");
}
