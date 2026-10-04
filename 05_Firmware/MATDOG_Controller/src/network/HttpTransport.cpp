#include "HttpTransport.h"

#include <esp_heap_caps.h>
#include <esp_https_server.h>
#include <esp_random.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#include "../config/TlsIdentity.h"
#include "../update/Sha256.h"
#include "PortalPage.h"

namespace matdog {
namespace network {

namespace {

// Reads exactly two hex characters worth of bytes from a request header
// into a fixed buffer. Returns false on any missing header, wrong length,
// or non-hex character — metadata arriving over a transport is never
// trusted, the same rule Sha256.h's parseHex() already states.
bool readHexHeader(httpd_req_t *req, const char *field, uint8_t *out,
                   size_t out_bytes) {
  char buf[2 * update::kHmac256DigestBytes +
           1]; // large enough for every field used here
  const size_t need = out_bytes * 2;
  if (need >= sizeof(buf))
    return false;
  if (httpd_req_get_hdr_value_str(req, field, buf, sizeof(buf)) != ESP_OK)
    return false;
  if (strlen(buf) != need)
    return false;
  return update::parseHex(buf, need, out);
}

bool socketFacts(httpd_req_t *req, HttpTransportRequest &r) {
  sockaddr_in local{}, peer{};
  socklen_t n = sizeof(local);
  int fd = httpd_req_to_sockfd(req);
  if (getsockname(fd, reinterpret_cast<sockaddr *>(&local), &n) != 0)
    return false;
  n = sizeof(peer);
  if (getpeername(fd, reinterpret_cast<sockaddr *>(&peer), &n) != 0)
    return false;
  r.ap_socket = ntohl(local.sin_addr.s_addr) == 0xc0a80401u;
  r.peer = peer.sin_addr.s_addr;
  r.tls = ntohs(local.sin_port) == 443;
  char host[64]{}, origin[64]{};
  httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host));
  httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin));
  r.local_authorized = portalOrigin(r.ap_socket, host, origin);
  return true;
}
void jsonString(const char *in, char *out, size_t cap) {
  size_t n = 0;
  for (size_t i = 0; in[i] && n + 2 < cap; ++i) {
    unsigned char c = in[i];
    if (c == '"' || c == '\\') {
      out[n++] = '\\';
      out[n++] = c;
    } else
      out[n++] = c < 32 ? '?' : c;
  }
  out[n] = 0;
}
} // namespace

void HttpTransport::begin(core::ControllerService *service,
                          update::OtaManager *ota, const uint8_t *ota_secret,
                          size_t ota_secret_len) {
  service_ = service;
  ota_ = ota;
  update::OtaSessionConfig config;
  session_.begin(config, ota_secret, ota_secret_len);
  request_ready_ = xSemaphoreCreateBinary();
  response_ready_ = xSemaphoreCreateBinary();
  slot_free_ = xSemaphoreCreateBinary();
  if (!request_ready_ || !response_ready_ || !slot_free_ ||
      xTaskCreatePinnedToCore(lifecycleEntry, "md_http_life", 4096, this, 1,
                              nullptr, 0) != pdPASS) {
    if (request_ready_)
      vSemaphoreDelete(request_ready_);
    if (response_ready_)
      vSemaphoreDelete(response_ready_);
    if (slot_free_)
      vSemaphoreDelete(slot_free_);
    request_ready_ = response_ready_ = slot_free_ = nullptr;
    return;
  }
  xSemaphoreGive(slot_free_);
}

bool HttpTransport::start() {
  if (!request_ready_ || desired_.load())
    return false;
  manual_stop_.store(false);
  desired_.store(true);
  return true;
}
void HttpTransport::stop() {
  manual_stop_.store(true);
  desired_.store(false);
  tls_desired_.store(false);
}
void HttpTransport::lifecycleEntry(void *self) {
  static_cast<HttpTransport *>(self)->lifecycle();
}
bool HttpTransport::registerHandlers(httpd_handle_t server) {
  const httpd_uri_t uris[] = {
      {"/status", HTTP_GET, &HttpTransport::handleStatus, this},
      {"/ota/challenge", HTTP_GET, &HttpTransport::handleOtaChallenge, this},
      {"/ota/update", HTTP_POST, &HttpTransport::handleOtaUpdate, this},
      {"/", HTTP_GET, &HttpTransport::handlePage, this},
      {"/wifi/status", HTTP_GET, &HttpTransport::handleWifi, this},
      {"/wifi/config", HTTP_GET, &HttpTransport::handleWifi, this},
      {"/wifi/login", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/wifi/config", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/wifi/scan", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/wifi/ap", HTTP_POST, &HttpTransport::handleWifi, this}};
  for (const auto &uri : uris)
    if (httpd_register_uri_handler(server, &uri) != ESP_OK)
      return false;
  return true;
}
void HttpTransport::lifecycle() {
  for (;;) {
    if (desired_.load() && !server_) {
      httpd_config_t c = HTTPD_DEFAULT_CONFIG();
      c.stack_size = 24576;
      c.max_uri_handlers = 12;
      c.max_open_sockets = 2;
      c.recv_wait_timeout = 3;
      c.send_wait_timeout = 3;
      c.task_priority = 2;
      if (httpd_start(&server_, &c) == ESP_OK && !registerHandlers(server_)) {
        httpd_stop(server_);
        server_ = nullptr;
      }
      listening_.store(server_ != nullptr);
    }
    if (tls_desired_.load() && !tls_server_ && config::kTlsIdentityPresent &&
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >= 100000) {
      httpd_ssl_config_t c = HTTPD_SSL_CONFIG_DEFAULT();
      c.httpd.stack_size = 28672;
      c.httpd.max_uri_handlers = 12;
      c.httpd.max_open_sockets = 1;
      c.httpd.task_priority = 2;
      c.httpd.recv_wait_timeout = 3;
      c.httpd.send_wait_timeout = 3;
      c.tls_handshake_timeout_ms = 5000;
      c.servercert = reinterpret_cast<const uint8_t *>(config::kTlsCertificate);
      c.servercert_len = sizeof(config::kTlsCertificate);
      c.prvtkey_pem = reinterpret_cast<const uint8_t *>(config::kTlsPrivateKey);
      c.prvtkey_len = sizeof(config::kTlsPrivateKey);
      if (httpd_ssl_start(&tls_server_, &c) == ESP_OK &&
          !registerHandlers(tls_server_)) {
        httpd_ssl_stop(tls_server_);
        tls_server_ = nullptr;
      }
      tls_listening_.store(tls_server_ != nullptr);
    }
    if (!desired_.load() && server_) {
      httpd_stop(server_);
      server_ = nullptr;
      listening_.store(false);
    }
    if (!tls_desired_.load() && tls_server_) {
      httpd_ssl_stop(tls_server_);
      tls_server_ = nullptr;
      tls_listening_.store(false);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
bool HttpTransport::dispatch(const HttpTransportRequest &request,
                             HttpTransportResponse *response) {
  if (xSemaphoreTake(slot_free_, pdMS_TO_TICKS(kDispatchTimeoutMs)) != pdTRUE)
    return false;
  xSemaphoreTake(response_ready_,
                 0); // drain prior late wake; IDs remain authoritative
  pending_request_ = request;
  const uint32_t id = sequence_.fetch_add(1) + 1;
  pending_request_.sequence = id;
  mailbox_.beginDispatch();
  xSemaphoreGive(request_ready_);
  const uint32_t since = millis();
  do {
    const uint32_t elapsed = millis() - since;
    if (elapsed >= kDispatchTimeoutMs)
      break;
    if (xSemaphoreTake(response_ready_,
                       pdMS_TO_TICKS(kDispatchTimeoutMs - elapsed)) != pdTRUE)
      break;
    if (ready_sequence_.load() == id) {
      *response = pending_response_;
      xSemaphoreGive(slot_free_);
      return true;
    }
  } while (millis() - since < kDispatchTimeoutMs);
  // If delivery won the atomic race, response bytes are already complete and
  // this reader owns release. Otherwise Controller releases after processing.
  if (!mailbox_.abandon())
    xSemaphoreGive(slot_free_);
  return false;
}
void HttpTransport::update(uint32_t now_ms) {
  if (!request_ready_)
    return;
  if (memcmp(previous_admin_digest_, service_->networkAdminDigest(), 32) != 0) {
    portal_.reset();
    memcpy(previous_admin_digest_, service_->networkAdminDigest(), 32);
  }
  if (!manual_stop_.load() && (service_->wifiStatus().ap_active &&
                               service_->wifiStatus().admin_provisioned))
    desired_.store(true);
  tls_desired_.store(
      !manual_stop_.load() && config::kTlsIdentityPresent &&
      (service_->remoteUpdateAllowed() ||
       (session_.sessionAuthenticated() && service_->wifiStatus().connected)));
  if (session_.sessionAuthenticated() &&
      (session_.timedOut(now_ms) || !service_->wifiStatus().connected ||
       manual_stop_.load())) {
    ota_->abort();
    session_.reset();
  }
  if (xSemaphoreTake(request_ready_, 0) != pdTRUE)
    return;
  serviceRequest(now_ms);
  ready_sequence_.store(pending_response_.sequence);
  if (mailbox_.claimDelivery())
    xSemaphoreGive(response_ready_);
  else
    xSemaphoreGive(slot_free_);
}

void HttpTransport::serviceRequest(uint32_t now_ms) {
  HttpTransportResponse &resp = pending_response_;
  resp = HttpTransportResponse{};
  const HttpTransportRequest &req = pending_request_;
  resp.sequence = req.sequence;

  switch (req.kind) {
  case HttpRequestKind::WEB_STATUS: {
    // Read-only. Every field below is a ControllerService accessor — see
    // ControllerService.h — never a direct hardware read. Field coverage
    // is deliberately the operator's I8 list in full: system/source
    // identity, module health, Wi-Fi state, BMS cached telemetry,
    // IMU/status summary, LED presentation state, actuator readiness/
    // fail-closed state, calibration readiness/state, OTA status/
    // provenance (2026-09-25 correction).
    const power::DalySample &bms = service_->bmsSample();
    const calibration::CalibrationSessionStatus &cal =
        service_->calibrationStatus();
    const update::OtaManagerStatus &ota = service_->otaStatus();
    int n = snprintf(
        resp.text, sizeof(resp.text),
        // system / source identity
        "build_id=%s\nsystem_health=%s\npower_state=%s\nmode=%s\nauthority=%s\n"
        // module health
        "imu_availability=%s\nbms_availability=%s\nservo_availability=%s\n"
        "led_availability=%s\n"
        // IMU status summary
        "imu_stream=%s\nimu_rv_count=%lu\n"
        // BMS cached telemetry
        "bms_comm=%s\nbms_age_ms=%lu\nbms_valid=%s\n"
        "bms_pack_v=%.1f\nbms_soc_percent=%.1f\n"
        // Wi-Fi state
        "wifi_state=%s\nwifi_connected=%s\nwifi_fault=%s\n"
        // LED presentation state
        "led_presentation=%s\n"
        // actuator readiness / fail-closed state
        "actuator_policy_epoch=%lu\nactuator_geometry_bound=%s\n"
        "actuator_readiness_torque_enable=%s\n"
        "actuator_readiness_position_command=%s\n"
        // calibration readiness / state
        "calibration_session_state=%s\ncalibration_hardware_motion_authorized=%"
        "s\n"
        "calibration_restore_required=%s\n"
        // OTA status / provenance
        "ota_state=%s\nota_running_build_id=%s\nota_ingest_compiled=%s\n"
        "ota_rollback_possible=%s\n",
        build::kBuildId, core::toString(service_->systemHealth()),
        core::toString(service_->powerState()),
        core::toString(service_->operatingMode()),
        core::toString(service_->authorityOwner()),
        core::toString(core::classify(service_->imuAvailability())),
        core::toString(core::classify(service_->bmsAvailability())),
        core::toString(core::classify(service_->servoAvailability())),
        core::toString(core::classify(service_->ledAvailability())),
        service_->imuStreamEnabled() ? "ON" : "OFF",
        (unsigned long)service_->imuRvCount(),
        power::toString(service_->bmsLastCommResult()),
        (unsigned long)service_->bmsLastResultAgeMs(now_ms),
        service_->bmsHasValidSample() ? "YES" : "NO",
        service_->bmsHasValidSample() ? bms.pack_voltage_v : 0.0f,
        service_->bmsHasValidSample() ? bms.soc_percent : 0.0f,
        network::toString(service_->wifiStatus().state),
        service_->wifiStatus().connected ? "YES" : "NO",
        network::toString(service_->wifiStatus().fault),
        status::toString(service_->ledPresentationState()),
        (unsigned long)service_->actuatorPolicyEpoch(),
        service_->actuatorPolicyGeometryBound() ? "YES" : "NO",
        core::toString(service_->readiness(
            core::ServiceCapability::ACTUATOR_TORQUE_ENABLE)),
        core::toString(service_->readiness(
            core::ServiceCapability::ACTUATOR_POSITION_COMMAND)),
        calibration::toString(cal.state),
        calibration::CalibrationManager::hardwareMotionAuthorized() ? "YES"
                                                                    : "NO",
        cal.restore.required ? "YES" : "NO", update::toString(ota.policy.state),
        ota.running_build_id,
        update::OtaManager::ingestEnabled() ? "YES" : "NO",
        ota.policy.rollback_possible ? "YES" : "NO");
    resp.text_len = (n > 0 && static_cast<size_t>(n) < sizeof(resp.text))
                        ? static_cast<size_t>(n)
                        : sizeof(resp.text) - 1;
    resp.ok = true;
    resp.message = "OK";
    break;
  }

  case HttpRequestKind::WIFI_STATUS: {
    wifiResponse(resp);
    break;
  }
  case HttpRequestKind::WIFI_LOGIN: {
    uint8_t random[64];
    esp_fill_random(random, sizeof(random));
    resp.ok = req.local_authorized &&
              portal_.login(now_ms, req.peer, req.login,
                            service_->networkAdminDigest(), random);
    resp.message = resp.ok ? "OK" : "AUTH_FAILED";
    if (resp.ok) {
      strcpy(resp.cookie, portal_.cookie());
      strcpy(resp.text, portal_.csrf());
      resp.text_len = 64;
    }
    break;
  }
  case HttpRequestKind::WIFI_CONFIG:
  case HttpRequestKind::WIFI_SCAN:
  case HttpRequestKind::WIFI_AP: {
    if (!req.local_authorized ||
        !portal_.authorize(now_ms, req.peer, req.cookie, req.csrf)) {
      resp.message = "AUTH_FAILED";
      break;
    }
    if (session_.sessionAuthenticated() ||
        service_->otaStatus().holds_actuator_inhibit) {
      resp.message = "OTA_BUSY";
      break;
    }
    if (req.kind == HttpRequestKind::WIFI_CONFIG)
      resp.ok = service_->configureNetwork(req.patch);
    if (req.kind == HttpRequestKind::WIFI_SCAN)
      resp.ok = service_->scanNetwork();
    if (req.kind == HttpRequestKind::WIFI_AP)
      resp.ok = service_->accessPoint(req.ap_on);
    resp.message =
        resp.ok ? (req.kind == HttpRequestKind::WIFI_CONFIG ? "CONFIG_PENDING"
                                                            : "QUEUED")
                : "REFUSED_BUSY_OR_RECOVERY";
    break;
  }
  case HttpRequestKind::OTA_CHALLENGE: {
    if (!req.tls || req.ap_socket || !service_->remoteUpdateAllowed() ||
        !update::OtaManager::ingestEnabled()) {
      resp.message = "OTA_SECURITY_GATE";
      break;
    }
    uint8_t random_bytes[update::kOtaNonceBytes];
    esp_fill_random(random_bytes, sizeof(random_bytes));
    session_.issueChallenge(now_ms, random_bytes, resp.nonce);
    resp.ok = true;
    resp.message = "OK";
    break;
  }

  case HttpRequestKind::OTA_BEGIN: {
    if (!req.tls || req.ap_socket || !service_->remoteUpdateAllowed() ||
        session_.sessionAuthenticated()) {
      resp.message = "OTA_SECURITY_GATE";
      break;
    }
    const update::OtaAuthResult auth =
        session_.authenticate(now_ms, req.nonce, req.metadata, req.signature);
    resp.auth_result = auth;
    if (auth != update::OtaAuthResult::OK) {
      resp.ok = false;
      resp.message = "AUTH_FAILED";
      break;
    }
    if (!ota_->prepare(req.metadata) || !ota_->openStream()) {
      session_.reset();
      resp.ok = false;
      resp.message = "PREPARE_FAILED";
      break;
    }
    resp.ok = true;
    resp.message = "OK";
    break;
  }

  case HttpRequestKind::OTA_CHUNK: {
    if (!session_.sessionAuthenticated() || session_.timedOut(now_ms)) {
      ota_->abort();
      session_.reset();
      resp.ok = false;
      resp.message = "SESSION_NOT_ACTIVE";
      break;
    }
    if (!ota_->writeChunk(req.chunk_data, req.chunk_len)) {
      ota_->abort();
      session_.reset();
      resp.ok = false;
      resp.message = "WRITE_FAILED";
      break;
    }
    session_.noteActivity(now_ms);
    resp.ok = true;
    resp.message = "OK";
    break;
  }

  case HttpRequestKind::OTA_FINISH: {
    if (!session_.sessionAuthenticated() || session_.timedOut(now_ms)) {
      ota_->abort();
      session_.reset();
      resp.ok = false;
      resp.message = "SESSION_NOT_ACTIVE";
      break;
    }
    const bool sealed = ota_->finishStream() && ota_->commitBootTarget();
    if (!sealed)
      ota_->abort();
    session_.reset();
    resp.ok = sealed;
    // Deliberately does not call esp_restart() anywhere in this class —
    // see the class comment. A committed boot target waits for a
    // separate, explicit reboot step, exactly like the reviewed manual
    // application-only flash path already does.
    resp.message = sealed ? "COMMITTED_PENDING_REBOOT" : "FINISH_FAILED";
    break;
  }

  case HttpRequestKind::OTA_ABORT: {
    ota_->abort();
    session_.reset();
    resp.ok = true;
    resp.message = "ABORTED";
    break;
  }

  case HttpRequestKind::NONE:
    resp.ok = false;
    resp.message = "UNKNOWN_REQUEST";
    break;
  }
}

// ---------------------------------------------------------------------------
// httpd handlers — run on the httpd task. Each one only ever: parses the
// request, calls dispatch() (which blocks this task, never the Controller
// thread), and formats the response. No ControllerService/OtaSession/
// OtaManager call ever appears below this line.
// ---------------------------------------------------------------------------

esp_err_t HttpTransport::handleStatus(httpd_req_t *req) {
  auto *self = static_cast<HttpTransport *>(req->user_ctx);
  HttpTransportRequest r{};
  r.kind = HttpRequestKind::WEB_STATUS;
  HttpTransportResponse resp{};
  if (!self->dispatch(r, &resp))
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Controller busy");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, resp.text, static_cast<ssize_t>(resp.text_len));
}

esp_err_t HttpTransport::handleOtaChallenge(httpd_req_t *req) {
  auto *self = static_cast<HttpTransport *>(req->user_ctx);
  HttpTransportRequest r{};
  r.kind = HttpRequestKind::OTA_CHALLENGE;
  if (!socketFacts(req, r) || !r.tls || r.ap_socket)
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "TLS STA only");
  HttpTransportResponse resp{};
  if (!self->dispatch(r, &resp))
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Controller busy");
  if (!resp.ok)
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, resp.message);
  char hex[2 * update::kOtaNonceBytes + 1];
  static const char digits[] = "0123456789abcdef";
  for (size_t i = 0; i < update::kOtaNonceBytes; ++i) {
    hex[2 * i] = digits[resp.nonce[i] >> 4];
    hex[2 * i + 1] = digits[resp.nonce[i] & 15];
  }
  hex[2 * update::kOtaNonceBytes] = '\0';
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, hex);
}

esp_err_t HttpTransport::handleOtaUpdate(httpd_req_t *req) {
  if (!update::OtaManager::ingestEnabled()) {
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN,
                               "ingest disabled at compile time");
  }
  auto *self = static_cast<HttpTransport *>(req->user_ctx);

  HttpTransportRequest begin{};
  begin.kind = HttpRequestKind::OTA_BEGIN;
  if (!socketFacts(req, begin) || !begin.tls || begin.ap_socket)
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "TLS STA only");
  if (req->content_len <= 0 || req->content_len > 5242880)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                               "invalid image size");
  if (!readHexHeader(req, "X-Ota-Nonce", begin.nonce, sizeof(begin.nonce)) ||
      !readHexHeader(req, "X-Ota-Signature", begin.signature,
                     sizeof(begin.signature)) ||
      !readHexHeader(req, "X-Ota-Sha256", begin.metadata.sha256,
                     sizeof(begin.metadata.sha256))) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                               "missing/malformed OTA headers");
  }
  char build_id[update::kOtaBuildIdBytes];
  if (httpd_req_get_hdr_value_str(req, "X-Ota-Build-Id", build_id,
                                  sizeof(build_id)) != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                               "missing X-Ota-Build-Id");
  }
  strncpy(begin.metadata.build_id, build_id,
          sizeof(begin.metadata.build_id) - 1);
  begin.metadata.schema_version = update::kOtaMetadataSchema;
  begin.metadata.image_size = static_cast<uint32_t>(req->content_len);

  HttpTransportResponse resp{};
  if (!self->dispatch(begin, &resp)) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Controller busy");
  }
  if (!resp.ok) {
    return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, resp.message);
  }

  uint32_t remaining = begin.metadata.image_size;
  while (remaining > 0) {
    HttpTransportRequest chunk{};
    chunk.kind = HttpRequestKind::OTA_CHUNK;
    const size_t want =
        remaining < kHttpChunkBufferBytes ? remaining : kHttpChunkBufferBytes;
    const int got =
        httpd_req_recv(req, reinterpret_cast<char *>(chunk.chunk_data), want);
    if (got <= 0) {
      HttpTransportRequest abort_req{};
      abort_req.kind = HttpRequestKind::OTA_ABORT;
      HttpTransportResponse abort_resp{};
      self->dispatch(abort_req, &abort_resp);
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                 "body read failed/truncated");
    }
    chunk.chunk_len = static_cast<uint32_t>(got);

    HttpTransportResponse chunk_resp{};
    if (!self->dispatch(chunk, &chunk_resp) || !chunk_resp.ok) {
      HttpTransportRequest abort_req{};
      abort_req.kind = HttpRequestKind::OTA_ABORT;
      HttpTransportResponse abort_resp{};
      self->dispatch(abort_req, &abort_resp);
      return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                 chunk_resp.ok ? "Controller busy"
                                               : chunk_resp.message);
    }
    remaining -= static_cast<uint32_t>(got);
  }

  HttpTransportRequest finish{};
  finish.kind = HttpRequestKind::OTA_FINISH;
  HttpTransportResponse finish_resp{};
  if (!self->dispatch(finish, &finish_resp) || !finish_resp.ok) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               finish_resp.ok ? "Controller busy"
                                              : finish_resp.message);
  }
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, finish_resp.message);
}

void HttpTransport::wifiResponse(HttpTransportResponse &resp) {
  const auto &w = service_->wifiStatus();
  char ip[16], ssid[67], bssid[37], ap[67];
  formatIpv4(w.ipv4, ip, 16);
  jsonString(w.ssid, ssid, sizeof(ssid));
  jsonString(w.bssid, bssid, sizeof(bssid));
  jsonString(w.ap_ssid, ap, sizeof(ap));
  int n = snprintf(
      resp.text, sizeof(resp.text),
      "{\"build_id\":\"%s\",\"ota_ingest\":%u,\"tls\":%s,\"connected\":%s,"
      "\"ssid\":\"%s\","
      "\"bssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%ld,\"channel\":%u,\"ap_"
      "active\":%s,\"ap_ssid\":\"%"
      "s\",\"ap_ip\":\"192.168.4.1\",\"ap_clients\":%u,\"sleep_effective\":%s,"
      "\"sleep_configured\":"
      "%u,\"usb_session_trusted\":false,\"source\":\"%s\",\"active_profile\":%"
      "u,\"disconnect_"
      "reason\":%u,\"config_phase\":\"%s\",\"config_busy\":%s,\"config_error\":"
      "%ld,\"roam_"
      "enabled\":%s,\"roam_threshold\":%d,\"roam_hysteresis\":%u,\"scan_"
      "interval_ms\":%lu,\"roam_"
      "dwell_ms\":%lu,\"bandwidth_mhz\":%u,\"ap_timeout_ms\":%lu,\"ap_always\":"
      "%s,\"worker_max_"
      "us\":%lu,\"worker_stack_free\":%lu,\"heap_free\":%lu,\"heap_min\":%lu,"
      "\"scan_starts\":%lu,"
      "\"scan_inhibited\":%lu,\"roams\":%lu,\"profiles\":[",
      build::kBuildId, update::OtaManager::ingestEnabled() ? 1 : 0,
      tlsStarted() ? "true" : "false", w.connected ? "true" : "false", ssid,
      bssid, ip, (long)w.rssi_dbm, w.channel, w.ap_active ? "true" : "false",
      ap, w.ap_clients, w.sleep_effective ? "true" : "false",
      w.sleep_configured, w.nvs_active ? "NVS" : "COMPILE_FALLBACK",
      w.active_profile, w.disconnect_reason, toString(w.config_phase),
      w.config_busy ? "true" : "false", (long)w.config_error,
      w.roam_enabled ? "true" : "false", w.roam_threshold, w.roam_hysteresis,
      (unsigned long)w.scan_interval_ms, (unsigned long)w.roam_dwell_ms,
      w.bandwidth_mhz, (unsigned long)w.ap_timeout_ms,
      w.ap_always ? "true" : "false", (unsigned long)w.worker_max_us,
      (unsigned long)w.worker_stack_free,
      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
      (unsigned long)w.scan_starts, (unsigned long)w.scan_inhibited,
      (unsigned long)w.roam_count);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(resp.text)) {
    resp.message = "STATUS_OVERFLOW";
    return;
  }
  size_t used = n;
  for (unsigned i = 0; i < 2; ++i) {
    const auto &p = w.profiles[i];
    char ss[67], a[16], m[16], g[16], d[16];
    jsonString(p.ssid, ss, sizeof(ss));
    formatIpv4(p.ip, a, 16);
    formatIpv4(p.mask, m, 16);
    formatIpv4(p.gateway, g, 16);
    formatIpv4(p.dns, d, 16);
    n = snprintf(resp.text + used, sizeof(resp.text) - used,
                 "%s{\"ssid\":\"%s\",\"enabled\":%s,\"dhcp\":%s,\"ip\":\"%s\","
                 "\"mask\":\"%s\","
                 "\"gateway\":\"%s\",\"dns\":\"%s\"}",
                 i ? "," : "", ss, p.enabled ? "true" : "false",
                 p.dhcp ? "true" : "false", a, m, g, d);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(resp.text) - used) {
      resp.message = "STATUS_OVERFLOW";
      return;
    }
    used += n;
  }
  n = snprintf(
      resp.text + used, sizeof(resp.text) - used,
      "],\"ap_reload_pending\":%s,\"sleep_apply_ok\":%s,\"bandwidth_"
      "configured\":%u,"
      "\"bandwidth_apply_ok\":%s,\"snapshot_max_us\":%lu,\"connects\":%lu,"
      "\"connect_timeouts\":%lu,\"link_losses\":%lu,\"scan\":[",
      w.ap_reload_pending ? "true" : "false",
      w.sleep_apply_ok ? "true" : "false", w.bandwidth_configured,
      w.bandwidth_apply_ok ? "true" : "false", (unsigned long)w.max_update_us,
      (unsigned long)w.counters.connects,
      (unsigned long)w.counters.connect_timeouts,
      (unsigned long)w.counters.link_losses);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(resp.text) - used) {
    resp.message = "STATUS_OVERFLOW";
    return;
  }
  used += n;
  for (unsigned i = 0; i < w.scan_count && i < 12; ++i) {
    const auto &e = w.scan[i];
    char ss[67];
    jsonString(e.ssid, ss, sizeof(ss));
    n = snprintf(resp.text + used, sizeof(resp.text) - used,
                 "%s{\"ssid\":\"%s\",\"bssid\":\"%s\",\"rssi\":%ld,\"channel\":"
                 "%u,\"secure\":%s}",
                 i ? "," : "", ss, e.bssid, (long)e.rssi, e.channel,
                 e.secure ? "true" : "false");
    if (n < 0 || static_cast<size_t>(n) >= sizeof(resp.text) - used) {
      resp.message = "STATUS_OVERFLOW";
      return;
    }
    used += n;
  }
  n = snprintf(resp.text + used, sizeof(resp.text) - used, "]}");
  if (n < 0 || static_cast<size_t>(n) >= sizeof(resp.text) - used) {
    resp.message = "STATUS_OVERFLOW";
    return;
  }
  resp.text_len = used + n;
  resp.ok = true;
  resp.message = "OK";
}
esp_err_t HttpTransport::handlePage(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
  httpd_resp_set_hdr(
      req, "Content-Security-Policy",
      "default-src 'self'; script-src 'unsafe-inline'; style-src "
      "'unsafe-inline'; connect-src "
      "'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'");
  return httpd_resp_send(req, kPortalPage, sizeof(kPortalPage) - 1);
}
esp_err_t HttpTransport::handleWifi(httpd_req_t *req) {
  auto *self = static_cast<HttpTransport *>(req->user_ctx);
  HttpTransportRequest r{};
  if (!socketFacts(req, r))
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Socket unavailable");
  if (req->method == HTTP_GET)
    r.kind = HttpRequestKind::WIFI_STATUS;
  else {
    if (!r.local_authorized)
      return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN,
                                 "Protected AP origin required");
    if (req->content_len >= kPortalBodyBytes)
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
    char type[64]{};
    httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type));
    if (strcmp(type, "application/x-www-form-urlencoded") != 0)
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                 "Form content type required");
    char body[kPortalBodyBytes]{};
    size_t read = 0;
    const uint32_t since = millis();
    while (read < req->content_len) {
      if (millis() - since > 3000)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body timeout");
      int got = httpd_req_recv(req, body + read, req->content_len - read);
      if (got <= 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "Truncated body");
      read += got;
    }
    if (strcmp(req->uri, "/wifi/login") == 0) {
      r.kind = HttpRequestKind::WIFI_LOGIN;
      if (read != 70 || strncmp(body, "token=", 6) != 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid login");
      memcpy(r.login, body + 6, 64);
    } else {
      char cookie[128]{};
      httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof(cookie));
      const char *start = strstr(cookie, "md_session=");
      if (start) {
        start += 11;
        if (strlen(start) >= 64 && (start[64] == 0 || start[64] == ';'))
          memcpy(r.cookie, start, 64);
      }
      httpd_req_get_hdr_value_str(req, "X-Csrf-Token", r.csrf, sizeof(r.csrf));
      if (strcmp(req->uri, "/wifi/config") == 0) {
        r.kind = HttpRequestKind::WIFI_CONFIG;
        if (!parseConfigForm(body, read, &r.patch))
          return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                     "Invalid config form");
      } else if (strcmp(req->uri, "/wifi/scan") == 0) {
        r.kind = HttpRequestKind::WIFI_SCAN;
        if (read)
          return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                     "Empty body required");
      } else if (strcmp(req->uri, "/wifi/ap") == 0) {
        r.kind = HttpRequestKind::WIFI_AP;
        if (read != 4 || strncmp(body, "on=", 3) != 0 ||
            (body[3] != '0' && body[3] != '1'))
          return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                     "Invalid AP action");
        r.ap_on = body[3] == '1';
      }
    }
    memset(body, 0, sizeof(body));
  }
  HttpTransportResponse resp{};
  if (!self->dispatch(r, &resp))
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Controller busy");
  if (!resp.ok)
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, resp.message);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (r.kind == HttpRequestKind::WIFI_STATUS) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp.text, resp.text_len);
  }
  if (r.kind == HttpRequestKind::WIFI_LOGIN) {
    char cookie[160];
    snprintf(
        cookie, sizeof(cookie),
        "md_session=%s; HttpOnly; SameSite=Strict; Path=/wifi/; Max-Age=300",
        resp.cookie);
    httpd_resp_set_hdr(req, "Set-Cookie", cookie);
    return httpd_resp_send(req, resp.text, resp.text_len);
  }
  return httpd_resp_sendstr(req, resp.message);
}

} // namespace network
} // namespace matdog
