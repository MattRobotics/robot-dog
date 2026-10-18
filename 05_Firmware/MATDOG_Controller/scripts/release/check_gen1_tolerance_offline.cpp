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
  CHECK(t&&l);CHECK(resolveRawToUrdfQ(s.profile,geometry_data::kProvenance,*t,l->min_tick,&a)==TargetResolveStatus::OK);
  CHECK(resolveRawToUrdfQ(s.profile,geometry_data::kProvenance,*t,l->max_tick,&b)==TargetResolveStatus::OK);
  std::printf("%s{\"slot\":%u,\"q0_tick\":%u,\"min_tick\":%u,\"max_tick\":%u,\"limits_rad\":[%.6f,%.6f]}",i?",":"",i,j.q0_tick,l->min_tick,l->max_tick,std::fmin(a,b)*1e-6,std::fmax(a,b)*1e-6);
 }
 std::printf("],\"pass\":%s}\n",failures?"false":"true");return failures?1:0;
}
