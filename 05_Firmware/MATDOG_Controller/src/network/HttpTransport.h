#ifndef MATDOG_NETWORK_HTTP_TRANSPORT_H
#define MATDOG_NETWORK_HTTP_TRANSPORT_H

#include <Arduino.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "../core/ControllerService.h"
#include "../update/OtaManager.h"
#include "../update/OtaSession.h"
#include "HttpMailbox.h"

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
// pending request at a time, full stop, enforced by a THIRD semaphore,
// slot_free_ — see HttpMailbox.h for why a fix found during I7/I8
// hardening made this necessary:
//
//   httpd task:        Take(slot_free_, bounded)   <- claims exclusive use
//                       mailbox_.beginDispatch()       of the shared slot
//                       write pending_request_
//                       Give(request_ready_)
//                       Take(response_ready_, bounded)
//                         -> timeout: mailbox_.abandon(); return false
//                         -> got it: read pending_response_; return true
//
//   Controller thread: Take(request_ready_, 0)   <- non-blocking poll
//                       compute pending_response_
//                       if (mailbox_.awaitingResponse()) Give(response_ready_)
//                       mailbox_.delivered()
//                       Give(slot_free_)              <- always, last
//
// slot_free_ is what makes the request/response structs safe with only one
// slot: a NEW dispatch() cannot write into pending_request_/
// pending_response_ until the Controller thread has unconditionally
// finished with the PREVIOUS one (Give(slot_free_) is always its last act),
// so a timed-out dispatch's abandoned request can never be overwritten
// mid-read. mailbox_ (HttpMailbox, pure and host-tested — see
// scripts/tests/test_http_mailbox.cpp) is what prevents the OTHER half of
// the bug: a late Give(response_ready_) for that same abandoned request
// leaking forward and being wrongly consumed by a LATER, unrelated
// dispatch()'s Take(response_ready_) — the Controller thread only signals
// response_ready_ when mailbox_ says somebody is still listening.
//
// WHAT THIS FILE DOES NOT PROVE
// --------------------------------
// The request/response TYPES and the STATE they carry are exercised by
// OtaSession's own offline suite (test_ota_session.cpp), the correlation
// logic above by HttpMailbox's own suite (test_http_mailbox.cpp), and
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
//
// A bounded START -> STOP -> START -> STOP cycle must not leak resources:
// stop() deletes every semaphore start() created, and start() itself
// releases anything it allocated on its own partial-failure paths before
// returning false. See start()/stop() below.
//
// GET /status THREAT MODEL — explicit decision (I7/I8 hardening, 2026-09-25)
// -----------------------------------------------------------------------------
// /status is unauthenticated. This is a deliberate choice, not an oversight:
// it is documented HERE as the intentional design, per the operator's
// instruction to make the decision explicit rather than leave it implied.
//
//   - Reaching /status at all already requires the operator to have issued
//     @WEB SERVER START over USB CDC while in MAINTENANCE mode — the same
//     "physical/USB access is the trust boundary" gate the DALY KEY write
//     and the servo scan/census/preflight commands already use. Once
//     started, any client on the same Wi-Fi network can call it — this
//     class does not additionally authenticate the read.
//   - /status carries no secret: build/source identity, module
//     availability, Wi-Fi link state (never the SSID's passphrase — see
//     network/WifiPolicy.h's own "must have no field that could ever hold
//     a secret" rule, which this reuses via WifiStatus), BMS telemetry,
//     IMU/LED presentation state, actuator/calibration readiness
//     classifications, and OTA provenance. None of it is a credential, a
//     key, or information that grants any capability by itself.
//   - Adding authentication here was considered and rejected as security
//     theatre for this candidate: reusing the OTA HMAC scheme would put a
//     read-only status page behind the SAME credential that authorizes
//     firmware writes — raising the read surface to the write surface's
//     trust level, backwards from "write authorization must remain
//     strictly stronger than read-only status access" (true today only
//     because read requires nothing and write requires a valid HMAC
//     signature over a fresh challenge). Inventing a SEPARATE, weaker
//     read-only credential would be a second credential/auth system for a
//     page that discloses no secret — complexity with no matching threat.
//   - This is intentionally a MAINTENANCE-mode, operator-initiated,
//     local-LAN observability surface, not a public or always-on one: it
//     cannot be reached unless an operator with USB access chose to expose
//     it, and it stops being reachable the moment @WEB SERVER STOP runs or
//     the device reboots (never auto-started — see above).
//
// SECURITY MODEL — what the OTA transport DOES and DOES NOT protect
// -----------------------------------------------------------------------------
// This is plain HTTP (no TLS) with an HMAC-SHA256 pre-shared-secret
// challenge/response layer (update/OtaSession.h) in front of the one
// existing OtaManager/OtaPolicy/OtaEspBackend writer. Stated plainly, so
// no TLS guarantee is ever implied where none exists:
//
//   DOES protect:
//     - Writer authentication: only a caller who can compute a valid
//       HMAC-SHA256 over the signed payload (proving possession of the
//       shared secret, which is never transmitted) can begin a write.
//     - Metadata authenticity: schema_version, image_size, sha256 and
//       build_id are all part of the signed payload — none can be altered
//       in transit without invalidating the signature.
//     - Image integrity: the declared, signed SHA256 is checked against
//       the actually-streamed bytes by OtaPolicy/OtaManager (unchanged,
//       pre-existing OTA-A logic) — a network attacker who tampers with
//       the firmware bytes in transit is DETECTED (the update is refused),
//       even though the tampering itself is not prevented at the transport
//       layer.
//     - Challenge/replay resistance: a single-use, server-issued nonce,
//       hardened (2026-09-25) so a repeated unauthenticated challenge call
//       cannot displace a legitimate client's still-valid nonce — see
//       update/OtaSession.h and update/OtaSession.cpp's issueChallenge().
//
//   Does NOT protect:
//     - Confidentiality of the firmware bytes, the request/response
//       metadata, or the /status content: this is plain HTTP, readable by
//       anyone who can observe the traffic. The shared secret itself is
//       never sent over the wire (only an HMAC computed from it is), so
//       eavesdropping cannot recover the secret — only the content.
//     - Availability against a network attacker capable of dropping or
//       blocking traffic: nothing at this layer defends against that: it
//       is a plain TCP/HTTP connection with no redundant path.
//
// Firmware confidentiality is NOT a requirement for MATDOG today: this is
// engineering firmware for a private robot on a private LAN, not software
// being distributed to untrusted parties, and no requirement demanding
// transport confidentiality was found during this review. HMAC-authenticated
// plain HTTP is therefore judged acceptable for this candidate, with this
// threat model documented rather than a TLS guarantee implied. If a concrete
// confidentiality requirement emerges later, esp_https_server is already
// confirmed available (see README.md's transport evaluation table) and
// OtaSession's authentication layer underneath would not need to change.

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
  // Claims exclusive use of pending_request_/pending_response_ before a new
  // dispatch() may write into them — see the class comment above and
  // HttpMailbox.h. Starts "given" (free) each time start() creates it.
  SemaphoreHandle_t slot_free_ = nullptr;
  // Pure correlation logic deciding whether a computed response is still
  // wanted — see HttpMailbox.h. Reset implicitly by start() recreating this
  // object fresh (default-constructed) each time.
  HttpMailbox mailbox_{};
  HttpTransportRequest pending_request_{};
  HttpTransportResponse pending_response_{};

  static constexpr uint32_t kDispatchTimeoutMs = 3000;
};

}  // namespace network
}  // namespace matdog

#endif  // MATDOG_NETWORK_HTTP_TRANSPORT_H
