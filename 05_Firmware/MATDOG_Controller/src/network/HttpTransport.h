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
  WIFI_AP
};
constexpr size_t kHttpChunkBufferBytes = 1024, kHttpStatusTextBytes = 4096;
struct HttpTransportRequest {
  HttpRequestKind kind = HttpRequestKind::NONE;
  uint32_t sequence = 0;
  uint8_t nonce[update::kOtaNonceBytes]{};
  update::OtaImageMetadata metadata{};
  uint8_t signature[update::kHmac256DigestBytes]{};
  uint8_t chunk_data[kHttpChunkBufferBytes]{};
  uint32_t chunk_len = 0;
  bool local_authorized = false, tls = false, ap_socket = false, ap_on = false;
  uint32_t peer = 0;
  char cookie[65]{}, csrf[65]{}, login[65]{};
  ConfigPatch patch{};
};
struct HttpTransportResponse {
  bool ok = false;
  const char *message = "";
  char text[kHttpStatusTextBytes]{};
  size_t text_len = 0;
  uint32_t sequence = 0;
  char cookie[65]{};
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
  void update(uint32_t now_ms);

private:
  static esp_err_t handleStatus(httpd_req_t *req);
  static esp_err_t handleOtaChallenge(httpd_req_t *req);
  static esp_err_t handleOtaUpdate(httpd_req_t *req);
  static esp_err_t handlePage(httpd_req_t *req);
  static esp_err_t handleWifi(httpd_req_t *req);
  static void lifecycleEntry(void *self);
  void lifecycle();
  bool registerHandlers(httpd_handle_t server);
  bool dispatch(const HttpTransportRequest &request,
                HttpTransportResponse *response);
  void serviceRequest(uint32_t now_ms);
  void wifiResponse(HttpTransportResponse &response);
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
  static constexpr uint32_t kDispatchTimeoutMs = 3000;
};
} // namespace network
} // namespace matdog
#endif
