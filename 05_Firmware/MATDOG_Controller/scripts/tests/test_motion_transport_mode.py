#!/usr/bin/env python3
"""Production Wi-Fi role/admission versus MotionService mode, no hardware.

Runs actual WifiManager, AdminAccess, NetworkPolicy, WifiPolicy, config owner and
MotionService. Driver/event/scheduler/NVS facts are the existing declared doubles.
One OperatingMode feeds both subsystems; no phase or admission is forced by tests.
Other motion qualification facts are isolated hypothetical inputs; no executor or
actuator is bound, and every tested motion command is refused.
"""
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent / 'src'
controller = (SRC/'core/Controller.cpp').read_text()
critical = (SRC/'core/ControllerService.h').read_text()
assert 'wifi_.setRunMode(operating_mode_.mode() == OperatingMode::RUN);' in controller
assert 'operatingMode() != OperatingMode::MAINTENANCE ||' in critical
fixture = (HERE/'test_wifi_mode_policy.cpp').read_text().split('int main() {', 1)[0]
# Resolve the existing production headers from a temporary extracted unit.
fixture = fixture.replace('"../../src/network/WifiManager.h"', '"network/WifiManager.h"')
source = (SRC/'network/HttpTransport.cpp').read_text()
start = source.index('bool HttpTransport::admitAdmin(')
end = source.index('bool HttpTransport::dispatch(', start)
admit = source[start:end].replace('HttpTransport::', 'TransportProbe::')
unit = fixture + r'''
#include "core/MotionService.h"
using matdog::core::OperatingMode;
struct HttpTransportRequest { uint32_t admin_generation=0; };
struct ServiceNetworkView { WifiManager* wifi; AdminAccess& networkAdminAccess() { return wifi->adminAccess(); } };
struct TransportProbe { ServiceNetworkView* service_; bool admitAdmin(HttpTransportRequest&); };
''' + admit + r'''
unsigned semantic_checks=0;
#define ASSERT_MODE(x) do { ++semantic_checks; CHECK(x); } while(0)
matdog::core::MotionReadiness isolatedReadiness(OperatingMode mode) {
 matdog::core::MotionReadiness r;
 r.calibration_restored=r.calibration_accepted=r.limits_approved=true;
 r.hardware_approved=r.operational_permit=r.support_observable=true;
 r.power_ok=r.bms_ok=r.authority_live=true;r.mode=mode;return r;
}
void applyControllerMode(WifiManager& wifi,matdog::core::MotionService& motion,OperatingMode mode) {
 // Production Controller wires both subsystems from this same enum. In RUN
 // networkCritical's mode clause alone is true, preserving config inhibit.
 wifi.setContext(mode!=OperatingMode::MAINTENANCE,false);
 wifi.setRunMode(mode==OperatingMode::RUN);
 motion.refresh(isolatedReadiness(mode),clock_ms);
}
bool admitted(WifiManager& wifi) {
 ServiceNetworkView service{&wifi};TransportProbe transport{&service};HttpTransportRequest request;
 return transport.admitAdmin(request);
}
void rejectedMotion(matdog::core::MotionService& service,const char* reason) {
 static unsigned sequence=0;
 matdog::motion::MotionCommand c;c.sequence=++sequence;std::snprintf(c.id,sizeof(c.id),"mode-probe-%u",sequence);
 c.issued_at_ms=clock_ms;c.ttl_ms=1000;c.owner=matdog::motion::ClientOwner::DASHBOARD;
 c.intent=matdog::motion::Intent::STAND;c.target_height_m=.150;
 const auto ack=service.command(c,clock_ms);
 ASSERT_MODE(ack.outcome==matdog::motion::CommandOutcome::REJECTED);
 ASSERT_MODE(!strcmp(ack.reason,reason));
}
int main() {
 using R=RoleReason;
 {
  store(true,false);WifiManager wifi;matdog::core::MotionService motion;motion.begin(nullptr,nullptr);
  applyControllerMode(wifi,motion,OperatingMode::MAINTENANCE);WifiRuntimeTest::initialize(wifi);
  ASSERT_MODE(until(wifi,Phase::ADMIN_AP,3000)!=UINT32_MAX);
  ASSERT_MODE(verifiedAp(wifi)&&admitted(wifi));
  ASSERT_MODE(role(wifi)==R::MAINTENANCE_AP);
  ASSERT_MODE(!strcmp(motion.refusal(),"MODE_RUN_REQUIRED"));
  rejectedMotion(motion,"MODE_RUN_REQUIRED");
  const auto old_generation=wifi.adminAccess().generation();
  applyControllerMode(wifi,motion,OperatingMode::RUN);
  ASSERT_MODE(until(wifi,Phase::CLIENT_STA,3000)!=UINT32_MAX);
  advance(wifi,500);association("home-router");advance(wifi,500);
  ASSERT_MODE(role(wifi)==R::RUN_STA&&wifi.status().connected);
  ASSERT_MODE(!wifi.adminAccess().allowed(old_generation)&&!admitted(wifi));
  ASSERT_MODE(!strcmp(motion.refusal(),"EXECUTOR_NOT_BOUND"));
  ASSERT_MODE(!WifiManager::verifyAdminRadio());
  rejectedMotion(motion,"EXECUTOR_NOT_BOUND");
  ASSERT_MODE(!wifi.configure(patch("another-router")));
  ASSERT_MODE(sets==0&&commits==0);
 }
 {
  store(false,false);WifiManager wifi;matdog::core::MotionService motion;motion.begin(nullptr,nullptr);
  applyControllerMode(wifi,motion,OperatingMode::RUN);WifiRuntimeTest::initialize(wifi);
  ASSERT_MODE(until(wifi,Phase::ADMIN_AP,3000)!=UINT32_MAX);
  ASSERT_MODE(role(wifi)==R::RUN_NO_PROFILE_AP&&wifi.status().run_mode);
  ASSERT_MODE(verifiedAp(wifi)&&admitted(wifi)); // disproves universal exclusivity
  ASSERT_MODE(!strcmp(motion.refusal(),"EXECUTOR_NOT_BOUND"));
  rejectedMotion(motion,"EXECUTOR_NOT_BOUND");
  ASSERT_MODE(!wifi.configure(patch("another-router")));
  ASSERT_MODE(sets==0&&commits==0);
 }
 {
  store(true,false);WifiManager wifi;matdog::core::MotionService motion;motion.begin(nullptr,nullptr);
  applyControllerMode(wifi,motion,OperatingMode::RUN);WifiRuntimeTest::initialize(wifi);
  ASSERT_MODE(until(wifi,Phase::ADMIN_AP,60000)!=UINT32_MAX); // real policy timed-out router episode
  ASSERT_MODE(role(wifi)==R::RUN_RECOVERY_AP&&wifi.status().run_mode);
  ASSERT_MODE(verifiedAp(wifi)&&admitted(wifi));
  ASSERT_MODE(!strcmp(motion.refusal(),"EXECUTOR_NOT_BOUND"));
  rejectedMotion(motion,"EXECUTOR_NOT_BOUND");
  ASSERT_MODE(!wifi.configure(patch("another-router")));
  ASSERT_MODE(sets==0&&commits==0);
 }
 printf("MOTION_TRANSPORT_MODE = %s (%u semantic checks; %d total including driver/queue/fence assertions). RUN_STA lacks motion admission; RUN recovery AP is compatible.\n",failures?"FAIL":"PASS",semantic_checks,checks);
 return failures?1:0;
}
'''
network = ['network/WifiManager.cpp','network/AdminAccess.cpp','network/WifiPolicy.cpp','network/NetworkConfig.cpp','network/NetworkConfigNvs.cpp','update/Sha256.cpp']
service = ['core/MotionService.cpp','execution/MotionExecutor.cpp','actuator/ActuatorRuntime.cpp','actuator/ActuatorWritePolicy.cpp','actuator/CalibrationGeometryProfile.cpp','actuator/CalibrationTargetResolver.cpp','actuator/CalibrationSequencePlan.cpp','calibration/StartupRecoveryReference.cpp','calibration/CalibrationDomain.cpp','servo/ServoPopulation.cpp','core/ActuatorAuthority.cpp','core/OperatingMode.cpp','motion/MotionContract.cpp']
with tempfile.TemporaryDirectory(prefix='matdog-motion-mode-') as temp:
    cpp,exe=Path(temp)/'probe.cpp',Path(temp)/'probe';cpp.write_text(unit)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1','-pthread','-DDISABLED=0x00','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(SRC),'-I',str(HERE/'network_stubs'),'-I',str(HERE/'nvs_stub'),str(cpp),*[str(SRC/p) for p in network+service],'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
