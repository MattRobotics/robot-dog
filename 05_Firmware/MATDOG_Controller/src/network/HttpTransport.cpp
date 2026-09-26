#include "HttpTransport.h"

#include <esp_random.h>

#include "../update/Sha256.h"

namespace matdog {
namespace network {

namespace {

// Reads exactly two hex characters worth of bytes from a request header
// into a fixed buffer. Returns false on any missing header, wrong length,
// or non-hex character — metadata arriving over a transport is never
// trusted, the same rule Sha256.h's parseHex() already states.
bool readHexHeader(httpd_req_t* req, const char* field, uint8_t* out, size_t out_bytes) {
  char buf[2 * update::kHmac256DigestBytes + 1];  // large enough for every field used here
  const size_t need = out_bytes * 2;
  if (need >= sizeof(buf)) return false;
  if (httpd_req_get_hdr_value_str(req, field, buf, sizeof(buf)) != ESP_OK) return false;
  if (strlen(buf) != need) return false;
  return update::parseHex(buf, need, out);
}

}  // namespace

void HttpTransport::begin(core::ControllerService* service, update::OtaManager* ota,
                          const uint8_t* ota_secret, size_t ota_secret_len) {
  service_ = service;
  ota_ = ota;
  update::OtaSessionConfig config;
  session_.begin(config, ota_secret, ota_secret_len);
}

bool HttpTransport::start() {
  if (server_ != nullptr) return false;

  // Defensive: every partial-failure path below cleans up its own handles
  // via stop(), so this should never find leftovers — but stop() is cheap
  // and idempotent, and calling it first means start() can never CreateBinary()
  // over a still-live handle from some earlier, imperfectly-cleaned attempt.
  stop();

  request_ready_ = xSemaphoreCreateBinary();
  response_ready_ = xSemaphoreCreateBinary();
  slot_free_ = xSemaphoreCreateBinary();
  if (request_ready_ == nullptr || response_ready_ == nullptr || slot_free_ == nullptr) {
    stop();  // releases whichever of the three this attempt did create
    return false;
  }
  // The slot starts free: the first dispatch() must be able to claim it
  // without waiting for a Controller-thread Give() that can never come —
  // nothing has been serviced yet.
  xSemaphoreGive(slot_free_);
  mailbox_ = HttpMailbox{};

  // Explicit default config: exactly one httpd task, requests served one at
  // a time. That is what makes the single-slot mailbox below safe — see
  // the class comment. Do not raise lru_purge_enable / max worker counts
  // without re-reviewing that invariant.
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&server_, &config) != ESP_OK) {
    server_ = nullptr;
    stop();  // releases the three semaphores this attempt created
    return false;
  }

  const httpd_uri_t status_uri = {"/status", HTTP_GET, &HttpTransport::handleStatus, this};
  const httpd_uri_t challenge_uri = {"/ota/challenge", HTTP_GET, &HttpTransport::handleOtaChallenge,
                                     this};
  const httpd_uri_t update_uri = {"/ota/update", HTTP_POST, &HttpTransport::handleOtaUpdate, this};
  httpd_register_uri_handler(server_, &status_uri);
  httpd_register_uri_handler(server_, &challenge_uri);
  httpd_register_uri_handler(server_, &update_uri);
  return true;
}

void HttpTransport::stop() {
  // NOTE (HARDWARE_TO_TEST): httpd_stop() is expected to tear down the
  // httpd task and every in-flight connection before returning, but
  // whether a request genuinely mid-dispatch() at the exact moment
  // @WEB SERVER STOP runs is safely unwound — versus racing the semaphore
  // deletions below — has not been exercised against real concurrent HTTP
  // load. stop() itself is only ever called from the Controller thread
  // (Controller::update() -> CommandRouter -> here), the same thread
  // update()/dispatch()'s Controller-side code runs on, so there is no
  // Controller-thread-vs-itself race; the residual risk is specifically
  // the httpd task, mid-handler, losing its semaphore handles out from
  // under it.
  if (server_ != nullptr) {
    httpd_stop(server_);
    server_ = nullptr;
  }
  // Every semaphore start() may have created, released here — idempotent
  // and individually guarded (vSemaphoreDelete on a null handle is
  // unsafe), so a bounded START -> STOP -> START -> STOP cycle, or a
  // partial-start failure calling this directly, never leaks a handle.
  if (request_ready_ != nullptr) {
    vSemaphoreDelete(request_ready_);
    request_ready_ = nullptr;
  }
  if (response_ready_ != nullptr) {
    vSemaphoreDelete(response_ready_);
    response_ready_ = nullptr;
  }
  if (slot_free_ != nullptr) {
    vSemaphoreDelete(slot_free_);
    slot_free_ = nullptr;
  }
  mailbox_ = HttpMailbox{};
}

bool HttpTransport::dispatch(const HttpTransportRequest& req, HttpTransportResponse* out_resp) {
  // Claim exclusive use of the shared slot before touching it — bounded, so
  // a still-in-flight (even abandoned) previous request can never be
  // overwritten mid-read by the Controller thread. See the class comment
  // and HttpMailbox.h.
  if (xSemaphoreTake(slot_free_, pdMS_TO_TICKS(kDispatchTimeoutMs)) != pdTRUE) {
    return false;  // mailbox still busy with a previous, presumably stuck, request
  }

  pending_request_ = req;
  mailbox_.beginDispatch();
  xSemaphoreGive(request_ready_);
  if (xSemaphoreTake(response_ready_, pdMS_TO_TICKS(kDispatchTimeoutMs)) != pdTRUE) {
    // The Controller thread did not answer in time. Fail the HTTP request
    // closed rather than block the httpd task indefinitely — see the class
    // comment on why this can only mean something upstream already
    // stalled, never a reason to retry the wait. Tell the mailbox nobody
    // is listening any more, so a late Give(response_ready_) for THIS
    // request cannot satisfy a future, unrelated dispatch() (HttpMailbox.h).
    // slot_free_ is deliberately NOT given here: only the Controller thread
    // gives it, once it has genuinely finished with pending_request_/
    // pending_response_, so a new dispatch() still cannot overwrite state
    // that thread might still be reading.
    mailbox_.abandon();
    return false;
  }
  *out_resp = pending_response_;
  return true;
}

void HttpTransport::update(uint32_t now_ms) {
  if (request_ready_ == nullptr) return;  // start() never called
  if (xSemaphoreTake(request_ready_, 0) != pdTRUE) return;  // nothing pending — non-blocking

  serviceRequest(now_ms);

  // Only signal a response if the httpd task is still the one that asked
  // for it — a timed-out dispatch() already called mailbox_.abandon(), and
  // must not be satisfied by a stale Give() meant for it (HttpMailbox.h).
  if (mailbox_.awaitingResponse()) {
    xSemaphoreGive(response_ready_);
  }
  mailbox_.delivered();
  // Safe to reuse the slot only once this thread is fully done with
  // pending_request_/pending_response_ — i.e. now, unconditionally, whether
  // or not anyone was still waiting for the answer.
  xSemaphoreGive(slot_free_);
}

void HttpTransport::serviceRequest(uint32_t now_ms) {
  HttpTransportResponse resp{};
  const HttpTransportRequest& req = pending_request_;

  switch (req.kind) {
    case HttpRequestKind::WEB_STATUS: {
      // Read-only. Every field below is a ControllerService accessor — see
      // ControllerService.h — never a direct hardware read. Field coverage
      // is deliberately the operator's I8 list in full: system/source
      // identity, module health, Wi-Fi state, BMS cached telemetry,
      // IMU/status summary, LED presentation state, actuator readiness/
      // fail-closed state, calibration readiness/state, OTA status/
      // provenance (2026-09-25 correction).
      const power::DalySample& bms = service_->bmsSample();
      const calibration::CalibrationSessionStatus& cal = service_->calibrationStatus();
      const update::OtaManagerStatus& ota = service_->otaStatus();
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
          "calibration_session_state=%s\ncalibration_hardware_motion_authorized=%s\n"
          "calibration_restore_required=%s\n"
          // OTA status / provenance
          "ota_state=%s\nota_running_build_id=%s\nota_ingest_compiled=%s\n"
          "ota_rollback_possible=%s\n",
          build::kBuildId, core::toString(service_->systemHealth()),
          core::toString(service_->powerState()), core::toString(service_->operatingMode()),
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
          core::toString(service_->readiness(core::ServiceCapability::ACTUATOR_TORQUE_ENABLE)),
          core::toString(service_->readiness(core::ServiceCapability::ACTUATOR_POSITION_COMMAND)),
          calibration::toString(cal.state),
          calibration::CalibrationManager::hardwareMotionAuthorized() ? "YES" : "NO",
          cal.restore.required ? "YES" : "NO",
          update::toString(ota.policy.state), ota.running_build_id,
          update::OtaManager::ingestEnabled() ? "YES" : "NO",
          ota.policy.rollback_possible ? "YES" : "NO");
      resp.text_len = (n > 0 && static_cast<size_t>(n) < sizeof(resp.text))
                         ? static_cast<size_t>(n)
                         : sizeof(resp.text) - 1;
      resp.ok = true;
      resp.message = "OK";
      break;
    }

    case HttpRequestKind::OTA_CHALLENGE: {
      uint8_t random_bytes[update::kOtaNonceBytes];
      esp_fill_random(random_bytes, sizeof(random_bytes));
      session_.issueChallenge(now_ms, random_bytes, resp.nonce);
      resp.ok = true;
      resp.message = "OK";
      break;
    }

    case HttpRequestKind::OTA_BEGIN: {
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
      if (!session_.sessionAuthenticated()) {
        resp.ok = false;
        resp.message = "SESSION_NOT_ACTIVE";
        break;
      }
      const bool sealed = ota_->finishStream() && ota_->commitBootTarget();
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

  pending_response_ = resp;
}

// ---------------------------------------------------------------------------
// httpd handlers — run on the httpd task. Each one only ever: parses the
// request, calls dispatch() (which blocks this task, never the Controller
// thread), and formats the response. No ControllerService/OtaSession/
// OtaManager call ever appears below this line.
// ---------------------------------------------------------------------------

esp_err_t HttpTransport::handleStatus(httpd_req_t* req) {
  auto* self = static_cast<HttpTransport*>(req->user_ctx);
  HttpTransportRequest r{};
  r.kind = HttpRequestKind::WEB_STATUS;
  HttpTransportResponse resp{};
  if (!self->dispatch(r, &resp)) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                                            "Controller busy");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, resp.text, static_cast<ssize_t>(resp.text_len));
}

esp_err_t HttpTransport::handleOtaChallenge(httpd_req_t* req) {
  auto* self = static_cast<HttpTransport*>(req->user_ctx);
  HttpTransportRequest r{};
  r.kind = HttpRequestKind::OTA_CHALLENGE;
  HttpTransportResponse resp{};
  if (!self->dispatch(r, &resp)) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                                            "Controller busy");
  char hex[2 * update::kOtaNonceBytes + 1];
  update::toHex(resp.nonce, hex, sizeof(hex));  // digest-sized helper; nonce is smaller, safe to reuse
  hex[2 * update::kOtaNonceBytes] = '\0';
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, hex);
}

esp_err_t HttpTransport::handleOtaUpdate(httpd_req_t* req) {
  if (!update::OtaManager::ingestEnabled()) {
    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "ingest disabled at compile time");
  }
  auto* self = static_cast<HttpTransport*>(req->user_ctx);

  HttpTransportRequest begin{};
  begin.kind = HttpRequestKind::OTA_BEGIN;
  if (!readHexHeader(req, "X-Ota-Nonce", begin.nonce, sizeof(begin.nonce)) ||
      !readHexHeader(req, "X-Ota-Signature", begin.signature, sizeof(begin.signature)) ||
      !readHexHeader(req, "X-Ota-Sha256", begin.metadata.sha256, sizeof(begin.metadata.sha256))) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing/malformed OTA headers");
  }
  char build_id[update::kOtaBuildIdBytes];
  if (httpd_req_get_hdr_value_str(req, "X-Ota-Build-Id", build_id, sizeof(build_id)) != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing X-Ota-Build-Id");
  }
  strncpy(begin.metadata.build_id, build_id, sizeof(begin.metadata.build_id) - 1);
  begin.metadata.schema_version = update::kOtaMetadataSchema;
  begin.metadata.image_size = static_cast<uint32_t>(req->content_len);

  HttpTransportResponse resp{};
  if (!self->dispatch(begin, &resp)) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Controller busy");
  }
  if (!resp.ok) {
    return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, resp.message);
  }

  uint32_t remaining = begin.metadata.image_size;
  while (remaining > 0) {
    HttpTransportRequest chunk{};
    chunk.kind = HttpRequestKind::OTA_CHUNK;
    const size_t want = remaining < kHttpChunkBufferBytes ? remaining : kHttpChunkBufferBytes;
    const int got = httpd_req_recv(req, reinterpret_cast<char*>(chunk.chunk_data), want);
    if (got <= 0) {
      HttpTransportRequest abort_req{};
      abort_req.kind = HttpRequestKind::OTA_ABORT;
      HttpTransportResponse abort_resp{};
      self->dispatch(abort_req, &abort_resp);
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body read failed/truncated");
    }
    chunk.chunk_len = static_cast<uint32_t>(got);

    HttpTransportResponse chunk_resp{};
    if (!self->dispatch(chunk, &chunk_resp) || !chunk_resp.ok) {
      HttpTransportRequest abort_req{};
      abort_req.kind = HttpRequestKind::OTA_ABORT;
      HttpTransportResponse abort_resp{};
      self->dispatch(abort_req, &abort_resp);
      return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                 chunk_resp.ok ? "Controller busy" : chunk_resp.message);
    }
    remaining -= static_cast<uint32_t>(got);
  }

  HttpTransportRequest finish{};
  finish.kind = HttpRequestKind::OTA_FINISH;
  HttpTransportResponse finish_resp{};
  if (!self->dispatch(finish, &finish_resp) || !finish_resp.ok) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               finish_resp.ok ? "Controller busy" : finish_resp.message);
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, finish_resp.message);
}

}  // namespace network
}  // namespace matdog
