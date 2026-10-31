// Restores the EXISTING gen-1 blob into read-only host storage, never a device.
#define main original_restore_suite_main
#include "../tests/test_calibration_runtime_restore.cpp"
#undef main
#include <fstream>
#include <iterator>
#include <cmath>
#include "../../src/actuator/CalibrationTargetResolver.h"
int main(int argc,char** argv){
 if(argc!=2)return 2;
 Scenario s;std::ifstream f(argv[1],std::ios::binary);
 s.storage.slot[0]=std::vector<uint8_t>((std::istreambuf_iterator<char>(f)),{});
 s.persistence.begin(NvsInitStatus::READY,0);
 CHECK(s.run()==RuntimeRestoreStatus::RESTORED);
 CalibrationRecord record;CHECK(decodeCalibrationRecord(s.storage.slot[0].data(),s.storage.slot[0].size(),&record)==CalibrationRecordStatus::OK);
 const auto fp=calibrationRecordFingerprint(record);CHECK(fp==0xbe8d5902bb4e37daULL);
 CHECK(s.restore.qualifyCommissioningLimits(s.profile,geometry_data::kProvenance,5,facts(),s.transforms,s.limits)==OperationalAdmissionStatus::APPROVED);
 CHECK(s.storage.writes==0);
 std::printf("{\"generation\":1,\"fingerprint\":\"%016llx\",\"writes\":%d,\"joints\":[",static_cast<unsigned long long>(fp),s.storage.writes);
 for(unsigned i=0;i<12;++i){const auto& j=record.joint[i];const auto id=canonicalIdentity(i);const auto* t=s.transforms.find(id,s.profile.provenanceTag());const auto* l=s.limits.find(id,s.profile.provenanceTag());MicroRad a=0,b=0;
  CHECK(t&&l);
  const uint8_t expected_id[12]={13,12,11,23,22,21,33,32,31,43,42,41};
  const int8_t expected_dir[12]={-1,1,-1,-1,-1,1,1,-1,1,1,1,-1};
  CHECK(slotServo(i)&&slotServo(i)->bus_id==expected_id[i]);CHECK(jointDirection(s.profile,id)==expected_dir[i]);
  for(uint16_t tick:{l->min_tick,t->q0_tick,l->max_tick}){
    double q=0;uint16_t back=0;
    CHECK(resolveRawToRadians(s.profile,geometry_data::kProvenance,*t,tick,&q)==TargetResolveStatus::OK);
    CHECK(resolveRadiansToRaw(s.profile,geometry_data::kProvenance,*t,q,&back)==TargetResolveStatus::OK&&back==tick);
  }
  MicroRad observed=0;CHECK(resolveRawObservation(s.profile,geometry_data::kProvenance,*t,t->q0_tick,&observed)==TargetResolveStatus::OK&&observed==0);
  CHECK(resolveRawToUrdfQ(s.profile,geometry_data::kProvenance,*t,l->min_tick,&a)==TargetResolveStatus::OK);
  CHECK(resolveRawToUrdfQ(s.profile,geometry_data::kProvenance,*t,l->max_tick,&b)==TargetResolveStatus::OK);
  std::printf("%s{\"slot\":%u,\"q0_tick\":%u,\"min_tick\":%u,\"max_tick\":%u,\"limits_rad\":[%.6f,%.6f]}",i?",":"",i,j.q0_tick,l->min_tick,l->max_tick,std::fmin(a,b)*1e-6,std::fmax(a,b)*1e-6);
 }
 std::printf("],\"pass\":%s}\n",failures?"false":"true");return failures?1:0;
}
