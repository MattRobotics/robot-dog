// Offline-only decoder. The input is sensitive; output omits q0/contact ticks.
#include "../../src/calibration/CalibrationRecord.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"
#include <fstream>
#include <vector>
#include <cstdio>
using namespace matdog;
int main(int argc,char** argv){if(argc!=2)return 2;std::ifstream f(argv[1],std::ios::binary);std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)),{});calibration::CalibrationRecord r;auto status=calibration::decodeCalibrationRecord(bytes.data(),bytes.size(),&r);actuator::CalibrationGeometryProfile g;g.bind(&actuator::geometry_data::kProvenance,actuator::geometry_data::kJoints,actuator::geometry_data::kJointCount,actuator::geometry_data::kEndpoints,actuator::geometry_data::kEndpointCount);if(status==calibration::CalibrationRecordStatus::OK)status=calibration::validateCalibrationRecord(r,g);printf("{\"status\":\"%s\",\"generation\":%lu,\"joints\":12,\"authorizes_motion\":false}\n",calibration::toString(status),(unsigned long)r.generation);return status==calibration::CalibrationRecordStatus::OK&&r.generation==1?0:1;}
