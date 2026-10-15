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
constexpr size_t kHttpChunkBufferBytes = 1024, kHttpStatusTextBytes = 4096;
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
};
struct HttpTransportResponse {
  bool ok = false;
  const char *message = "";
  char text[kHttpStatusTextBytes]{};
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

private:
  static esp_err_t handleStatus(httpd_req_t *req);
  static esp_err_t handleOtaChallenge(httpd_req_t *req);
  static esp_err_t handleOtaUpdate(httpd_req_t *req);
  static esp_err_t handlePage(httpd_req_t *req);
  static esp_err_t handleWifi(httpd_req_t *req);
  static esp_err_t handleMotion(httpd_req_t *req);
  static esp_err_t handleCalibrationRestore(httpd_req_t *req);
  // Every Motion/RESTORE refusal is JSON with the true HTTP status.
  static esp_err_t sendJsonError(httpd_req_t* req,const char* status,const char* code,uint32_t elapsed_ms=0);
  static bool readBoundedBody(httpd_req_t* req,char* body,size_t cap,size_t* used);
  static void lifecycleEntry(void *self);
  void lifecycle();
  void lifecycleTick();
  bool admitAdmin(HttpTransportRequest &request);
  bool admitReadOnly(HttpTransportRequest&);
  bool readOnlyAdmitted(const HttpTransportRequest&) const;
  bool registerReadOnlyHandlers(httpd_handle_t);
  bool http_readonly_=false;
  uint32_t portal_generation_ = 0;
  bool portal_sta_=false;
  bool admitMotion(HttpTransportRequest&);
  bool motionStaAdmitted(const HttpTransportRequest&) const;
  bool registerHandlers(httpd_handle_t server, bool admin = true);
  // failure (optional) names why false was returned: MAILBOX_BUSY,
  // ADMIN_GATE_CLOSED or CONTROLLER_LOOP_LATE. timeout_ms bounds the wait.
  bool dispatch(const HttpTransportRequest &request,
                HttpTransportResponse *response, char* large_text=nullptr, size_t large_cap=0,
                uint32_t timeout_ms=kDispatchTimeoutMs, const char** failure=nullptr);
  void appendTransportStats(char* text,size_t cap);
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
  HttpTransportRequest pending_request_{};
  HttpTransportResponse pending_response_{};
  char motion_capabilities_[kMotionCapabilitiesBytes]{};
  static constexpr uint32_t kDispatchTimeoutMs = 3000;
  // Motion READS answer within this budget or return a structured 503, so a
  // browser never has to abort first and retry behind a still-busy handler.
  static constexpr uint32_t kMotionReadBudgetMs = 1200;
  // Instrumentation (no behaviour): Controller-loop gap seen by update() and
  // HTTP dispatch latency. Max values are windowed: reset when reported.
  std::atomic<uint32_t> loop_last_ms_{0}, loop_max_ms_{0}, dispatch_last_ms_{0},
      dispatch_max_ms_{0}, dispatch_late_{0}, dispatch_busy_{0};
  uint32_t last_update_ms_ = 0;
};
} // namespace network
} // namespace matdog
#endif
