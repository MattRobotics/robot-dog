#ifndef MATDOG_NETWORK_HTTP_TRANSPORT_H
#define MATDOG_NETWORK_HTTP_TRANSPORT_H

#include <Arduino.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "../core/ControllerService.h"
#include "../update/OtaManager.h"
#include "../update/OtaSession.h"

// The network transport adapter — I7/I8, per the operator's 2026-09-25
// correction: firmware bytes flow through the one existing
// OtaManager/OtaPolicy/OtaEspBackend writer; this class is the CONTROL/DATA
// plane in front of it, not a second writer. It also carries the read-only
// Web status endpoint (I8), reusing the same server and the same
// cross-thread handoff.
//
//   esp_http_server (its own FreeRTOS task)
//        |  every handler below does: fill request_, Give(request_ready_),
//        |  Take(response_ready_, timeout), read response_
//        v
//   HttpTransport::update()   <- called from Controller::update(), the ONE
//        |                       thread that ever touches ControllerService,
//        |                       OtaSession or OtaManager
//        v
//   ControllerService (I8 read) / OtaSession + OtaManager (I7 write)
//
// WHY A CROSS-THREAD HANDOFF AT ALL
// ------------------------------------
// esp_http_server always runs registered URI handlers on its own task,
// never on the caller of httpd_start(). Every other module in this
// firmware is deliberately single-threaded — "MATDOG's Controller is
// single-threaded... every subsystem runs cooperatively inside
// Controller::update()" (core/ActuatorAuthority.h, update/OtaPolicy.h) — a
// guarantee this class must not break. So the httpd task is never allowed
// to call ControllerService/OtaSession/OtaManager directly; it can only
// hand a request to the Controller thread and wait for a response.
//
// WHY A SINGLE-SLOT MAILBOX, NOT A QUEUE
// ------------------------------------------
// "Keep responses bounded. No unbounded queues" (I8 requirement). One
// pending request at a time, full stop: request_ready_/response_ready_ are
// binary semaphores strictly alternated (Give one, Take the other, never
// both outstanding at once), and the request/response structs are plain
// data protected entirely by that alternation — no separate mutex is
// needed because by construction only one side ever touches them at a
// time. esp_http_server itself serialises concurrent client connections
// onto its own task by default, so this is not a new bottleneck; it is the
// same one-request-at-a-time behaviour the server already has, made
// explicit at the Controller boundary too.
//
// WHAT THIS FILE DOES NOT PROVE
// --------------------------------
// The request/response TYPES and the STATE they carry are exercised by
// OtaSession's own offline suite (test_ota_session.cpp) and by
// ControllerService's read-only accessors (already Arduino-linked
// elsewhere). The FreeRTOS semaphore handoff itself — whether it is
// genuinely deadlock-free and correctly timed under real concurrent HTTP
// load — cannot be exercised by a host test; there is no FreeRTOS scheduler
// on the host. That is HARDWARE_TO_TEST, the same classification this
// repository already gives Wi-Fi association timing and OTA flash-write
// costs. Every blocking wait below carries a bounded timeout for exactly
// this reason: a stuck Controller thread must fail a pending HTTP request
// closed (503), never hang the httpd task forever.
//
// SERVER LIFECYCLE — disabled at boot, always compiled in
// ------------------------------------------------------------
// start()/stop() are never called from Controller::begin() — the server
// does not listen at boot in the frozen candidate. They are reachable only
// from a MAINTENANCE-gated command (@WEB SERVER START|STOP), the same
// "physical/USB access is the trust boundary" pattern already used for the
// DALY KEY write. Firmware ingest additionally requires
// MATDOG_OTA_INGEST_ENABLED=1 at compile time (unchanged, existing gate) —
// starting the server alone can never make ingest reachable.

namespace matdog {
namespace network {

enum class HttpRequestKind : uint8_t {
  NONE            = 0,
  WEB_STATUS      = 1,
  OTA_CHALLENGE   = 2,
  OTA_BEGIN       = 3,
  OTA_CHUNK       = 4,
  OTA_FINISH      = 5,
  OTA_ABORT       = 6,
};

constexpr size_t kHttpChunkBufferBytes = 1024;
// Sized for the full I8 read-only field list (system/source identity,
// module health, Wi-Fi state, BMS cached telemetry, IMU status, LED
// presentation, actuator/calibration readiness, OTA provenance) with
// headroom — still a fixed, bounded buffer, never a growing one.
constexpr size_t kHttpStatusTextBytes = 2048;

struct HttpTransportRequest {
  HttpRequestKind kind = HttpRequestKind::NONE;

  // OTA_BEGIN
  uint8_t nonce[update::kOtaNonceBytes] = {0};
  update::OtaImageMetadata metadata{};
  uint8_t signature[update::kHmac256DigestBytes] = {0};

  // OTA_CHUNK
  uint8_t chunk_data[kHttpChunkBufferBytes] = {0};
  uint32_t chunk_len = 0;
};

struct HttpTransportResponse {
  bool ok = false;
  const char* message = "";  // always a static string literal — never freed, never a pointer
                             // into request-scoped memory

  // WEB_STATUS
  char text[kHttpStatusTextBytes] = {0};
  size_t text_len = 0;

  // OTA_CHALLENGE
  uint8_t nonce[update::kOtaNonceBytes] = {0};

  // OTA_BEGIN / OTA_CHUNK / OTA_FINISH
  update::OtaAuthResult auth_result = update::OtaAuthResult::REJECTED_NO_SECRET;
};

class HttpTransport {
 public:
  // service/ota are not owned; both must outlive this object, the same
  // pointer-holding convention every other module in this codebase uses.
  void begin(core::ControllerService* service, update::OtaManager* ota,
            const uint8_t* ota_secret, size_t ota_secret_len);

  // Opens the listening socket and registers every handler. False if
  // already started or if httpd_start() itself fails. Never called from
  // Controller::begin() — see the class comment.
  bool start();
  // Idempotent; safe to call when not started.
  void stop();
  bool started() const { return server_ != nullptr; }

  // Drains at most one pending request per call — bounded, the same
  // "advance a little every tick, never block" contract every other
  // Controller-thread module follows. Must be called from
  // Controller::update(), and ONLY from there: this is the one place
  // ControllerService/OtaSession/OtaManager are touched on behalf of the
  // HTTP transport.
  void update(uint32_t now_ms);

 private:
  static esp_err_t handleStatus(httpd_req_t* req);
  static esp_err_t handleOtaChallenge(httpd_req_t* req);
  static esp_err_t handleOtaUpdate(httpd_req_t* req);

  // Blocking (bounded) round trip from the httpd task: fills `req`, wakes
  // the Controller thread, waits for the response. Returns false only on
  // timeout — the Controller thread never stalls under normal operation
  // (G3.1's non-blocking guarantee), so a timeout here means something is
  // already wrong and the honest answer is a failed request, not a hang.
  bool dispatch(const HttpTransportRequest& req, HttpTransportResponse* out_resp);

  void serviceRequest(uint32_t now_ms);

  core::ControllerService* service_ = nullptr;
  update::OtaManager* ota_ = nullptr;
  update::OtaSession session_{};

  httpd_handle_t server_ = nullptr;

  SemaphoreHandle_t request_ready_ = nullptr;
  SemaphoreHandle_t response_ready_ = nullptr;
  HttpTransportRequest pending_request_{};
  HttpTransportResponse pending_response_{};

  static constexpr uint32_t kDispatchTimeoutMs = 3000;
};

}  // namespace network
}  // namespace matdog

#endif  // MATDOG_NETWORK_HTTP_TRANSPORT_H
