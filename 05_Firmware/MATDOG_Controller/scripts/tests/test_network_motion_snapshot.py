#!/usr/bin/env python3
"""Offline Controller Wi-Fi snapshot regression; real WifiManager, stubbed radio.

Extract the production Controller call site and SimpleMotion engagement predicate.
The existing radio/NVS double supplies the platform; worker publication and mutex
contention are controlled here. No real device, radio or NVS is accessed.
"""
from pathlib import Path
import re
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent / 'src'
controller = (SRC / 'core/Controller.cpp').read_text()
call = re.findall(r'^.*wifi_\.update\(now_ms\);.*$', controller, re.M)
assert len(call) == 1
engaged = re.search(r'bool engaged\(\) const \{[^}]+\}',
                    (SRC / 'simple/SimpleMotion.h').read_text()).group()
fixture = (HERE / 'test_wifi_runtime_v3.cpp').read_text().split('int main() {', 1)[0]
fixture = fixture.replace('#include "../../src/network/WifiManager.h"',
                          '#include "network/WifiManager.h"')
fixture = fixture.replace('class WifiRuntimeTest {\npublic:', '''class WifiRuntimeTest {
public:
  static void publishOnly(WifiManager& m, unsigned seq) {
    m.shared_status_.disconnect_reason = seq;
    m.shared_status_.connected = seq % 2;
    m.shared_admin_digest_[0] = seq;
    m.config_busy_.store(seq % 2);
  }
  static void lockSnapshot(WifiManager& m) {
    CHECK(xSemaphoreTake(m.mutex_, 0) == pdTRUE);
  }
  static void unlockSnapshot(WifiManager& m) { xSemaphoreGive(m.mutex_); }
''')
fixture += '''
struct MotionState {
  enum class State { READY, MOVING, STANDING };
  bool energized_=false, offPending_=false;
  State state_=State::READY;
  ENGAGED
};
struct ControllerCallSite {
  WifiManager& wifi_;
  MotionState simple_motion_;
  void tick(uint32_t now_ms) { CALL_SITE }
};
int main() {
  seed(); WifiManager wifi; WifiRuntimeTest::initialize(wifi); enter(wifi, 100);
  ControllerCallSite c{wifi,{}};
  unsigned seq=1;
  const auto generation=wifi.adminAccess().generation();
  // Hundreds of worker publications during MOVING and indefinite energized HOLD.
  for (unsigned phase=0; phase<4; ++phase) {
    c.simple_motion_.state_=phase==1?MotionState::State::MOVING:
      phase==2?MotionState::State::STANDING:MotionState::State::READY;
    c.simple_motion_.energized_=phase==2;
    c.simple_motion_.offPending_=phase==3;
    for(unsigned i=0;i<50;++i,++seq) {
      WifiRuntimeTest::publishOnly(wifi,seq); c.tick(now+=50);
      CHECK(wifi.status().disconnect_reason==seq);
      CHECK(wifi.status().connected==bool(seq%2));
      CHECK(wifi.adminDigest()[0]==seq);
      CHECK(wifi.status().config_busy==bool(seq%2));
    }
  }
  // A busy snapshot mutex must return without waiting; next pass catches up.
  const auto before=wifi.status().disconnect_reason;
  WifiRuntimeTest::publishOnly(wifi,seq);
  WifiRuntimeTest::lockSnapshot(wifi); c.tick(now+=50);
  CHECK(wifi.status().disconnect_reason==before);
  WifiRuntimeTest::unlockSnapshot(wifi); c.tick(now+=50);
  CHECK(wifi.status().disconnect_reason==seq);
  CHECK(wifi.adminAccess().generation()==generation);
  CHECK(wifi.adminAccess().allowed(generation));
  CHECK(WifiManager::verifyAdminRadio());
  CHECK(sets==0&&commits==0);
  printf("NETWORK_MOTION_SNAPSHOT checks=%d failures=%d\\n",checks,failures);
  return failures?1:0;
}
'''.replace('ENGAGED', engaged).replace('CALL_SITE', call[0])
with tempfile.TemporaryDirectory(prefix='matdog-network-snapshot-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    cpp.write_text(fixture)
    exe = Path(tmp) / 'test'
    sources = ['network/WifiManager.cpp', 'network/AdminAccess.cpp',
               'network/WifiPolicy.cpp', 'network/NetworkConfig.cpp',
               'network/NetworkConfigNvs.cpp', 'update/Sha256.cpp']
    subprocess.run(['g++', '-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror',
                    '-I', str(SRC), '-I', str(HERE / 'network_stubs'),
                    '-I', str(HERE / 'nvs_stub'), str(cpp),
                    *[str(SRC / s) for s in sources], '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=15)
