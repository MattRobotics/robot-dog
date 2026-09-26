#include "OtaManager.h"

#include <Arduino.h>
#include <esp_system.h>

#include "../config/BuildConfig.h"

namespace matdog {
namespace update {
namespace {

// A boot that followed one of these is evidence about the image that just
// ran, not noise. Confirming a firmware that panicked its way here would
// throw away the bootloader's rollback for exactly the case it exists for.
bool isFatalResetReason(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_BROWNOUT:
      return true;
    default:
      return false;
  }
}

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

void copyBounded(char* dst, size_t dst_size, const char* src) {
  size_t i = 0;
  if (src != nullptr) {
    while (i + 1 < dst_size && src[i] != '\0') { dst[i] = src[i]; ++i; }
  }
  dst[i] = '\0';
}

}  // namespace

void OtaManager::begin(uint32_t now_ms, core::ActuatorAuthorityArbiter* arbiter) {
  (void)now_ms;

  gate_.bind(arbiter);
  policy_.begin(&backend_, &gate_);

  const OtaSelfCheckConfig config{kSelfCheckMinUptimeMs, kSelfCheckMinLoopTicks};
  boot_guard_.begin(&backend_, config);

  status_ = OtaManagerStatus{};
  status_.ingest_enabled = ingestEnabled();

  const esp_reset_reason_t reason = esp_reset_reason();
  status_.fatal_reset_reason = isFatalResetReason(reason);
  status_.reset_reason = resetReasonName(reason);

  // Firmware identity, reusing the one canonical scheme: build::kBuildId is
  // the git short SHA injected by scripts/build.sh. Deliberately NOT
  // esp_app_desc_t, whose version/project_name fields describe
  // arduino-lib-builder in an Arduino-ESP32 build, not MATDOG.
  copyBounded(status_.running_build_id, sizeof(status_.running_build_id),
              build::kBuildId);

  refreshStatus();
}

void OtaManager::update(uint32_t now_ms, const OtaHostFacts& facts) {
  (void)now_ms;
  const uint32_t started_us = micros();

  if (!boot_guard_.settled()) {
    // identity_readable is a real check, not a constant: the running slot
    // must be resolvable AND be an OTA application slot AND the build id
    // must be a usable string. If we cannot say which firmware this is, we
    // must not confirm it.
    const OtaPartitionInfo running = status_.policy.running;
    const bool id_ok = running.isOtaAppSlot() && status_.running_build_id[0] != '\0';
    status_.identity_readable = id_ok;

    OtaSelfCheckInputs inputs{};
    inputs.controller_initialized = facts.controller_initialized;
    inputs.command_router_bound = facts.command_router_bound;
    inputs.identity_readable = id_ok;
    inputs.fatal_reset_reason = status_.fatal_reset_reason;
    inputs.uptime_ms = facts.uptime_ms;

    boot_guard_.update(inputs);
    refreshStatus();
  }

  const uint32_t elapsed_us = micros() - started_us;
  status_.last_update_us = elapsed_us;
  if (elapsed_us > status_.max_update_us) status_.max_update_us = elapsed_us;
}

void OtaManager::refreshStatus() {
  status_.policy = policy_.status();
  status_.boot_state = boot_guard_.state();
  status_.self_check_fault = boot_guard_.fault();
  status_.rollback_armed = boot_guard_.rollbackStillArmed();
  status_.self_check_ticks = boot_guard_.loopTicks();
  status_.confirmed_at_ms = boot_guard_.confirmedAtMs();
  status_.max_open_us = backend_.maxOpenUs();
  status_.max_write_us = backend_.maxWriteUs();
  status_.max_end_us = backend_.maxEndUs();
  status_.last_backend_error = backend_.lastError();
  status_.holds_actuator_inhibit = gate_.holdsExclusive();
  // Read from the central arbiter through the gate's query, never cached:
  // there is exactly one authority state and OTA is not allowed a copy of it.
  status_.policy.last_gate_verdict = policy_.status().last_gate_verdict;
}

bool OtaManager::ingestAllowed() {
  if (ingestEnabled()) return true;
  // Not an oversight and not a silent no-op: the refusal is published so a
  // transport author sees exactly why nothing happened.
  status_.policy.fault = OtaFault::NOT_AUTHORIZED;
  return false;
}

bool OtaManager::prepare(const OtaImageMetadata& metadata) {
  if (!ingestAllowed()) return false;
  const bool ok = policy_.prepare(metadata);
  refreshStatus();
  return ok;
}

bool OtaManager::openStream() {
  if (!ingestAllowed()) return false;
  const bool ok = policy_.openStream();
  refreshStatus();
  return ok;
}

bool OtaManager::writeChunk(const uint8_t* data, uint32_t len) {
  if (!ingestAllowed()) return false;
  const bool ok = policy_.writeChunk(data, len);
  // Deliberately NOT refreshing the whole status per chunk: this runs in the
  // hot streaming path. Byte counters live in the policy and are published
  // at the next boundary.
  if (!ok) refreshStatus();
  return ok;
}

bool OtaManager::finishStream() {
  if (!ingestAllowed()) return false;
  const bool ok = policy_.finishStream();
  refreshStatus();
  return ok;
}

bool OtaManager::commitBootTarget() {
  if (!ingestAllowed()) return false;
  const bool ok = policy_.commitBootTarget();
  refreshStatus();
  return ok;
}

void OtaManager::abort() {
  policy_.abort();
  refreshStatus();
}

bool OtaManager::resetStateMachine() {
  const bool ok = policy_.reset();
  refreshStatus();
  return ok;
}

}  // namespace update
}  // namespace matdog
