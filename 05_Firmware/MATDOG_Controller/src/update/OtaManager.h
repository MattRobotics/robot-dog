#ifndef MATDOG_UPDATE_OTA_MANAGER_H
#define MATDOG_UPDATE_OTA_MANAGER_H

#include <stdint.h>

#include "OtaBootGuard.h"
#include "OtaEspBackend.h"
#include "OtaPolicy.h"

// ---------------------------------------------------------------------------
// OTA-A INGEST GATE — the "no open firmware upload endpoint" rule, as a
// compile-time fact rather than a promise.
// ---------------------------------------------------------------------------
// OTA-A implements the update core. It ships NO transport and NO
// authentication. Until authentication exists, the byte-ingest entry points
// must not be reachable in a production image, and "we just haven't wired a
// transport yet" is not a guarantee — the next person to add one would flip
// nothing and get an unauthenticated writer.
//
// So ingest is compiled out by default. A transport bring-up session opts in
// explicitly, for one build, exactly the way a ROBOT_POWERED session opts in
// to the hardware profile:
//
//     -DMATDOG_OTA_INGEST_ENABLED=1
//
// scripts/static_audit.py FAILS the build if the SOURCE default is anything
// but 0, so an unauthenticated firmware writer cannot be produced by an
// unreviewed edit or by accident.
#ifndef MATDOG_OTA_INGEST_ENABLED
#define MATDOG_OTA_INGEST_ENABLED 0
#endif

namespace matdog {
namespace update {

// What the Controller knows and the OTA layer does not. Passed in rather
// than reached for, so the self-check criteria stay host-testable.
struct OtaHostFacts {
  bool controller_initialized = false;
  bool command_router_bound = false;
  uint32_t uptime_ms = 0;
};

struct OtaManagerStatus {
  OtaStatus policy{};

  OtaBootState boot_state = OtaBootState::UNEVALUATED;
  OtaSelfCheckFault self_check_fault = OtaSelfCheckFault::NONE;
  bool rollback_armed = false;   // the bootloader would still roll this image back
  uint32_t self_check_ticks = 0;
  uint32_t confirmed_at_ms = 0;

  bool ingest_enabled = false;   // compile-time; see MATDOG_OTA_INGEST_ENABLED
  bool identity_readable = false;
  bool fatal_reset_reason = false;
  char running_build_id[kOtaBuildIdBytes] = {0};
  const char* reset_reason = "UNKNOWN";

  // Measured blocking cost of the flash operations, in microseconds.
  uint32_t max_open_us = 0;
  uint32_t max_write_us = 0;
  uint32_t max_end_us = 0;
  // Cost of OtaManager::update() itself, which runs every Controller tick.
  uint32_t last_update_us = 0;
  uint32_t max_update_us = 0;

  int32_t last_backend_error = 0;
};

// Controller-facing owner of the OTA subsystem.
//
//   transport (none in OTA-A)  ->  OtaManager  ->  OtaPolicy  ->  OtaEspBackend
//                                      |
//                                      +-------->  OtaBootGuard
//
// Owns no task and no callback, exactly like WifiManager. Everything happens
// on the Controller thread.
class OtaManager {
 public:
  // The image must survive this long, and keep looping, before it is allowed
  // to confirm itself. A boot-loop crash happens far inside 15 s, so a
  // firmware that cannot run never reaches confirmation and the bootloader
  // rolls it back on the next boot with no code of ours involved.
  static constexpr uint32_t kSelfCheckMinUptimeMs = 15000;
  static constexpr uint32_t kSelfCheckMinLoopTicks = 2000;

  void begin(uint32_t now_ms);

  // Bounded and cheap. Once the boot lifecycle has settled it does almost
  // nothing; it never touches flash.
  void update(uint32_t now_ms, const OtaHostFacts& facts);

  const OtaManagerStatus& status() const { return status_; }

  // ---- Transport-facing ingest API -------------------------------------
  // This is the substitution point: any transport (USB, HTTP, a custom TCP
  // framing) drives these five calls and nothing else. All of them refuse
  // unless MATDOG_OTA_INGEST_ENABLED is 1.
  bool prepare(const OtaImageMetadata& metadata);
  bool openStream();
  bool writeChunk(const uint8_t* data, uint32_t len);
  bool finishStream();
  bool commitBootTarget();
  void abort();
  bool resetStateMachine();

  static constexpr bool ingestEnabled() { return MATDOG_OTA_INGEST_ENABLED != 0; }

 private:
  void refreshStatus();
  bool ingestAllowed();

  OtaEspBackend backend_{};
  OtaStageAGate gate_{};   // OTA-B: replace with an ActuatorAuthority-backed gate
  OtaPolicy policy_{};
  OtaBootGuard boot_guard_{};
  OtaManagerStatus status_{};
};

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_MANAGER_H
