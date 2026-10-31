#ifndef MATDOG_NETWORK_HTTP_TRANSPORT_H
#define MATDOG_NETWORK_HTTP_TRANSPORT_H
#include <Arduino.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>

#include "../core/ControllerService.h"
#include "../update/OtaManager.h"
#include "../update/OtaSession.h"
#include "HttpMailbox.h"
#include "PortalSecurity.h"
namespace matdog {
namespace network {
enum class HttpRequestKind : uint8_t {
  NONE,
  WEB_STATUS,
  OTA_CHALLENGE,
  OTA_BEGIN,
  OTA_CHUNK,
  OTA_FINISH,
  OTA_ABORT,
  WIFI_STATUS,
  WIFI_LOGIN,
  WIFI_CONFIG,
  WIFI_SCAN,
  WIFI_AP,
  WIFI_SESSION,
  WIFI_LOGOUT,
  MOTION_STATUS,
  MOTION_CAPABILITIES,
  MOTION_COMMAND,
  MOTION_LOGIN, MOTION_SESSION, MOTION_LOGOUT,
  // Issue #48: AP admin portal only (handler refuses any non-AP socket).
  CALIBRATION_RESTORE
};
constexpr size_t kMotionCapabilitiesBytes=20480;
constexpr size_t kHttpChunkBufferBytes = 1024, kHttpStatusTextBytes = core::kMotionStatusJsonBytes;
struct HttpTransportRequest {
  HttpRequestKind kind = HttpRequestKind::NONE;
  uint32_t sequence = 0;
  uint8_t nonce[update::kOtaNonceBytes]{};
  update::OtaImageMetadata metadata{};
  uint8_t signature[update::kHmac256DigestBytes]{};
  uint8_t chunk_data[kHttpChunkBufferBytes]{};
  uint32_t chunk_len = 0;
  bool local_authorized = false, tls = false, ap_socket = false, ap_on = false,
       remember = false;
  uint32_t peer = 0, admin_generation = 0, local_ipv4=0;
  char cookie[65]{}, csrf[65]{}, login[65]{}, device[65]{};
  ConfigPatch patch{};
  motion::MotionCommand motion_command{};
  uint32_t restore_generation = 0;
  bool restore_confirmed = false;
  uint32_t stop_epoch = 0;   // STOP epoch at receipt (Issue #52 ordering)
};
struct HttpTransportResponse {
  bool ok = false;
  const char *message = "";
  char text[kHttpStatusTextBytes]{};
  core::MotionStatusSnapshot motion_snapshot{};
  bool motion_session=false,motion_readonly=false;
  uint32_t loop_stack_free=0;
  size_t text_len = 0;
  uint32_t sequence = 0;
  char cookie[65]{}, device[65]{};
  uint8_t nonce[update::kOtaNonceBytes]{};
  update::OtaAuthResult auth_result = update::OtaAuthResult::REJECTED_NO_SECRET;
};
// Two optional listeners (HTTP status + AP-local portal, HTTPS remote OTA),
// ONE mailbox, ONE OtaSession, ONE writer. Dedicated lifecycle task executes
// server start/stop so the Controller continues servicing in-flight handlers.
// Semaphores exist for the lifetime of Controller, never freed under handlers.
class HttpTransport {
public:
  void begin(core::ControllerService *service, update::OtaManager *ota,
             const uint8_t *secret, size_t len);
  bool start();
  void stop();
  bool started() const { return listening_.load(); }
  bool tlsStarted() const { return tls_listening_.load(); }
  bool manuallyStopped() const { return manual_stop_.load(); }
  void update(uint32_t now_ms);
  // Issue #52 STOP lane: called FIRST in every Controller loop. Validates the
  // pending STOP with the same session+CSRF proof as any Motion command, applies
  // it through the ControllerService, and advances the STOP epoch that makes
  // every motion command received before it SUPERSEDED_BY_STOP. True = applied.
  bool serviceStopLane(uint32_t now_ms);
  // Wake-up hint only: the Controller bus hook must call serviceStopLane()
  // and preempt ONLY when it authenticated and applied the request.
  bool stopPending() const { return stop_pending_.load(std::memory_order_acquire); }

private:
  static esp_err_t handleStatus(httpd_req_t *req);
  static esp_err_t handleOtaChallenge(httpd_req_t *req);
  static esp_err_t handleOtaUpdate(httpd_req_t *req);
  static esp_err_t handlePage(httpd_req_t *req);
  static esp_err_t handleWifi(httpd_req_t *req);
  static esp_err_t handleMotion(httpd_req_t *req);
  static esp_err_t handleCalibrationRestore(httpd_req_t *req);
  // Phone Digital Twin: static gzip assets from flash, no Controller dispatch.
  static esp_err_t handleTwin(httpd_req_t *req);
  // Every Motion/RESTORE refusal is JSON with the true HTTP status.
  static esp_err_t sendJsonError(httpd_req_t* req,const char* status,const char* code,uint32_t elapsed_ms=0);
  static bool readBoundedBody(httpd_req_t* req,char* body,size_t cap,size_t* used);
  static void lifecycleEntry(void *self);
  void lifecycle();
  void lifecycleTick();
  bool admitAdmin(HttpTransportRequest &request,const char** failure=nullptr);
  bool admitReadOnly(HttpTransportRequest&);
  bool readOnlyAdmitted(const HttpTransportRequest&,bool verify_radio=true) const;
  bool registerReadOnlyHandlers(httpd_handle_t);
  bool http_readonly_=false;
  uint32_t portal_generation_ = 0;
  bool portal_sta_=false;
  bool admitMotion(HttpTransportRequest&,const char** failure=nullptr);
  bool motionStaAdmitted(const HttpTransportRequest&,bool verify_radio=true) const;
  bool motionStaAdmitted(bool tls,uint32_t local_ipv4,uint32_t generation,const char** failure=nullptr,bool verify_radio=true) const;
  bool registerHandlers(httpd_handle_t server, bool admin = true);
  // failure (optional) names why false was returned: MAILBOX_BUSY,
  // ADMIN_GATE_CLOSED or CONTROLLER_LOOP_LATE. timeout_ms bounds the wait.
  bool dispatch(const HttpTransportRequest &request,
                HttpTransportResponse *response, char* large_text=nullptr, size_t large_cap=0,
                uint32_t timeout_ms=kDispatchTimeoutMs, const char** failure=nullptr);
  void appendTransportStats(char* text,size_t cap,uint32_t loop_stack_free);
  bool formatMotionResponse(HttpTransportResponse&);
  bool motionReadResponseAdmitted(const HttpTransportRequest&) const;
  void serviceRequest(uint32_t now_ms);
  void wifiResponse(HttpTransportResponse &response,bool redact_network_names=false);
  void statusExtension(HttpTransportResponse &response,
                       const power::DalySample &bms,
                       const calibration::CalibrationSessionStatus &cal);
  core::ControllerService *service_ = nullptr;
  update::OtaManager *ota_ = nullptr;
  update::OtaSession session_{};
  PortalSession portal_{};
  httpd_handle_t server_ = nullptr, tls_server_ = nullptr;
  std::atomic<bool> listening_{false}, tls_listening_{false}, desired_{false},
      tls_desired_{false}, manual_stop_{false};
  std::atomic<uint32_t> sequence_{0}, ready_sequence_{0};
  uint8_t previous_admin_digest_[32]{};
  SemaphoreHandle_t request_ready_ = nullptr, response_ready_ = nullptr,
                    slot_free_ = nullptr;
  HttpMailbox mailbox_{};
  bool deferred_ = false;   // pending request kept for a later Controller pass (Controller thread only)
  HttpTransportRequest pending_request_{};
  HttpTransportResponse pending_response_{};
  char motion_capabilities_[kMotionCapabilitiesBytes]{};
  static constexpr uint32_t kDispatchTimeoutMs = 3000;
  // Motion READS answer within this budget or return a structured 503, so a
  // browser never has to abort first and retry behind a still-busy handler.
  static constexpr uint32_t kMotionReadBudgetMs = 1200;
  // Requests needing Motion slack while Motion is engaged: 5 periods, then 503.
  static constexpr uint32_t kMotionBusyBudgetMs = 100;
  // Instrumentation (no behaviour): Controller-loop gap seen by update() and
  // HTTP dispatch latency. Max values are windowed: reset when reported.
  std::atomic<uint32_t> loop_last_ms_{0}, loop_max_ms_{0}, dispatch_last_ms_{0},
      dispatch_max_ms_{0}, dispatch_late_{0}, dispatch_busy_{0};
  std::atomic<uint32_t> read_service_last_us_{0},read_service_max_us_{0};
  uint32_t last_update_ms_ = 0;
  // Set by the Controller thread each loop: a Motion torque sequence or motion
  // is in progress. The single httpd task then starts no large Twin download
  // and aborts one in progress, so a STOP or a status poll is not queued behind
  // a ~140 KB send (304 still served; every body is time-bounded anyway).
  std::atomic<bool> twin_deferred_{false};
  // One-slot STOP lane (httpd task -> Controller), independent of slot_free_.
  std::atomic<bool> stop_lock_{false};   // tiny spin lock around stop_request_ copies
  HttpTransportRequest stop_request_{};
  std::atomic<bool> stop_pending_{false};
  std::atomic<uint32_t> stop_epoch_{0}, stop_done_{0}, stop_ticket_{0};
  char stop_ack_[512]{};
  // applied: the STOP passed admission and reached MotionService; ack_ok: its
  // ACK was serialized. Applied without ACK is an unknown outcome, not AUTH_FAILED.
  std::atomic<bool> stop_applied_{false},stop_ack_ok_{false};
  static constexpr uint32_t kStopLaneBudgetMs = 600;
  static constexpr size_t kTwinDeferBytes = 8192;
};
} // namespace network
} // namespace matdog
#endif
