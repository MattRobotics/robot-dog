#include "HttpTransport.h"
#include "NetworkPublicView.h"

#include <esp_heap_caps.h>
#include <esp_https_server.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <stdarg.h>
#include <memory>
#include <new>

#include "../config/TlsIdentity.h"
#include "../update/Sha256.h"
#include "PortalPage.h"
#include "TwinHostPolicy.h"
#include "../core/MotionJson.h"
#include "RestoreRequestJson.h"
#include "TwinAssets.h"
#include "WifiManager.h"

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

// HTTPD uses a dual-stack listener with this ESP32 core. lwIP reports IPv4
// connections on it as sockaddr_in6 with an IPv4-mapped address, not sockaddr_in.
// Accept only actual IPv4 endpoints; native IPv6 cannot establish AP authority
// or be reduced to the 32-bit peer identity used by PortalSession.
bool socketIpv4(const sockaddr_storage &address, socklen_t size,
                uint32_t &ip, uint16_t &port) {
  if (address.ss_family == AF_INET && size >= sizeof(sockaddr_in)) {
    sockaddr_in v4{};
    memcpy(&v4, &address, sizeof(v4));
    ip = v4.sin_addr.s_addr;
    port = ntohs(v4.sin_port);
    return true;
  }
  if (address.ss_family == AF_INET6 && size >= sizeof(sockaddr_in6)) {
    sockaddr_in6 v6{};
    memcpy(&v6, &address, sizeof(v6));
    if (!IN6_IS_ADDR_V4MAPPED(&v6.sin6_addr))
      return false;
    memcpy(&ip, &v6.sin6_addr.s6_addr[12], sizeof(ip));
    port = ntohs(v6.sin6_port);
    return true;
  }
  return false;
}
bool socketFacts(httpd_req_t *req, HttpTransportRequest &r,const char** failure=nullptr) {
  auto deny=[&](const char* why){if(failure)*failure=why;return false;};
  sockaddr_storage local{}, peer{};
  socklen_t n = sizeof(local);
  int fd = httpd_req_to_sockfd(req);
  uint32_t local_ip = 0, peer_ip = 0;
  uint16_t local_port = 0, peer_port = 0;
  if(getsockname(fd,reinterpret_cast<sockaddr*>(&local),&n)!=0)return deny("SOCKET_LOCAL_READ_FAILED");
  if(!socketIpv4(local,n,local_ip,local_port))return deny("SOCKET_LOCAL_NOT_IPV4");
  n = sizeof(peer);
  if(getpeername(fd,reinterpret_cast<sockaddr*>(&peer),&n)!=0)return deny("SOCKET_PEER_READ_FAILED");
  if(!socketIpv4(peer,n,peer_ip,peer_port))return deny("SOCKET_PEER_NOT_IPV4");
  r.ap_socket = ntohl(local_ip) == 0xc0a80401u;
  r.peer = peer_ip;
  r.local_ipv4=local_ip;
  r.tls = local_port == 443;
  char host[64]{}, origin[64]{};
  httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host));
  httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin));
  r.local_authorized = portalOrigin(r.ap_socket, host, origin);
  if(r.tls&&!r.ap_socket){
    char address[64]{},expected[72]{};
    const auto* bytes=reinterpret_cast<const uint8_t*>(&local_ip);
    snprintf(address,sizeof(address),"%u.%u.%u.%u",bytes[0],bytes[1],bytes[2],bytes[3]);
    snprintf(expected,sizeof(expected),"https://%s",address);
    r.local_authorized=!strcmp(host,address)&&!strcmp(origin,expected);
    if(!strcmp(host,"matdog-227594.local")&&!strcmp(origin,"https://matdog-227594.local"))r.local_authorized=true;
  }
  return true;
}
// One 64-character cookie value, by exact name. Missing, wrong length or a
// name that is only a suffix of another leaves `out` empty, which every
// consumer treats as "no credential".
void cookieValue(const char *header, const char *name, char out[65]) {
  out[0] = 0;
  const size_t n = strlen(name);
  for (const char *p = header; *p;) {
    while (*p == ' ' || *p == ';')
      ++p;
    size_t len = 0;
    while (p[len] && p[len] != ';')
      ++len;
    if (len == n + 65 && strncmp(p, name, n) == 0 && p[n] == '=') {
      memcpy(out, p + n + 1, 64);
      out[64] = 0;
      return;
    }
    p += len;
  }
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
  service_->networkAdminAccess().raiseFault(FaultSite::WEB_MANUAL_STOP);
  manual_stop_.store(true);
  desired_.store(false);
  tls_desired_.store(false);
}
void HttpTransport::lifecycleEntry(void *self) {
  static_cast<HttpTransport *>(self)->lifecycle();
}
bool HttpTransport::registerHandlers(httpd_handle_t server, bool admin) {
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
      {"/wifi/ap", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/wifi/session", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/wifi/logout", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/api/motion/status", HTTP_GET, &HttpTransport::handleMotion, this},
      {"/api/motion/capabilities", HTTP_GET, &HttpTransport::handleMotion, this},
      {"/api/motion/command", HTTP_POST, &HttpTransport::handleMotion, this},
      {"/api/motion/login", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/api/motion/session", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/api/motion/logout", HTTP_POST, &HttpTransport::handleWifi, this},
      {"/api/calibration/restore", HTTP_POST, &HttpTransport::handleCalibrationRestore, this},
      // Phone Digital Twin (AP, LAN read-only and router TLS alike).
      {"/twin", HTTP_GET, &HttpTransport::handleTwin, this},
      {"/twin/", HTTP_GET, &HttpTransport::handleTwin, this},
      {"/twin/assets/twin.js", HTTP_GET, &HttpTransport::handleTwin, this},
      {"/twin/matdog_collision.mtw", HTTP_GET, &HttpTransport::handleTwin, this},
      {"/twin/matt_robodog_rev00.urdf", HTTP_GET, &HttpTransport::handleTwin, this}};
  for (const auto &uri : uris) {
    // AP-administration only: never on the TLS router listener. RESTORE
    // (Issue #48) also refuses any non-AP socket inside its handler.
    if (!admin && (strncmp(uri.uri, "/wifi/", 6) == 0 || strncmp(uri.uri, "/api/calibration/", 17) == 0)) continue;
    if (httpd_register_uri_handler(server, &uri) != ESP_OK) return false;
  }
  return true;
}
bool HttpTransport::registerReadOnlyHandlers(httpd_handle_t server) {
  const httpd_uri_t routes[]={
    {"/",HTTP_GET,&HttpTransport::handlePage,this},
    {"/status",HTTP_GET,&HttpTransport::handleStatus,this},
    {"/wifi/status",HTTP_GET,&HttpTransport::handleWifi,this},
    {"/api/motion/status",HTTP_GET,&HttpTransport::handleMotion,this},
    {"/api/motion/capabilities",HTTP_GET,&HttpTransport::handleMotion,this},
    {"/twin",HTTP_GET,&HttpTransport::handleTwin,this},
    {"/twin/",HTTP_GET,&HttpTransport::handleTwin,this},
    {"/twin/assets/twin.js",HTTP_GET,&HttpTransport::handleTwin,this},
    {"/twin/matdog_collision.mtw",HTTP_GET,&HttpTransport::handleTwin,this},
    {"/twin/matt_robodog_rev00.urdf",HTTP_GET,&HttpTransport::handleTwin,this}};
  for(const auto& route:routes) if(httpd_register_uri_handler(server,&route)!=ESP_OK)return false;
  return true;
}
bool HttpTransport::readOnlyAdmitted(const HttpTransportRequest& r) const {
  const auto& w=service_->wifiStatus();auto& g=service_->networkAdminAccess();
  wifi_mode_t mode=WIFI_MODE_NULL;wifi_ap_record_t ap{};
  const bool read=r.kind==HttpRequestKind::NONE||r.kind==HttpRequestKind::WEB_STATUS||
    r.kind==HttpRequestKind::WIFI_STATUS||r.kind==HttpRequestKind::MOTION_STATUS||r.kind==HttpRequestKind::MOTION_CAPABILITIES;
  return read&&!r.ap_socket&&!r.tls&&r.local_ipv4&&r.local_ipv4==w.ipv4&&
    r.admin_generation==g.generation()&&!g.closing.load()&&!g.radio_fault.load()&&
    g.phase.load()==NetworkPhase::CLIENT_STA&&!w.run_mode&&w.connected&&
    static_cast<RoleReason>(w.role_reason)==RoleReason::MAINTENANCE_STA_READONLY&&
    esp_wifi_get_mode(&mode)==ESP_OK&&mode==WIFI_MODE_STA&&esp_wifi_sta_get_ap_info(&ap)==ESP_OK;
}
bool HttpTransport::admitReadOnly(HttpTransportRequest& r) {
  r.admin_generation=service_->networkAdminAccess().generation();return readOnlyAdmitted(r);
}
void HttpTransport::lifecycle() {
  for (;;) { lifecycleTick(); vTaskDelay(pdMS_TO_TICKS(50)); }
}
void HttpTransport::lifecycleTick() {
  auto &gate = service_->networkAdminAccess();
  AdminLease drain(gate);
  if (!drain) return;
  const auto phase = gate.phase.load();
  if (phase == NetworkPhase::ADMIN_AP && !WifiManager::verifyAdminRadio()) {
    gate.raiseFault(FaultSite::HTTP_RADIO_LOST);
  }
  desired_.store(!manual_stop_.load() && !gate.closing.load() &&
                 phase == NetworkPhase::ADMIN_AP &&
                 gate.allowed(gate.generation()) && WifiManager::verifyAdminRadio());
  const auto& wifi=service_->wifiStatus();
  const bool readonly=!manual_stop_.load()&&!gate.closing.load()&&!gate.radio_fault.load()&&phase==NetworkPhase::CLIENT_STA&&
    !wifi.run_mode&&wifi.connected&&static_cast<RoleReason>(wifi.role_reason)==RoleReason::MAINTENANCE_STA_READONLY;
  desired_.store(desired_.load()||readonly);
  if(server_&&http_readonly_!=readonly) {
    if(httpd_stop(server_)!=ESP_OK){gate.raiseFault(FaultSite::LISTENER_STOP_FAILED);return;}
    server_=nullptr;listening_.store(false);
  }
  if (phase != NetworkPhase::CLIENT_STA || gate.closing.load() || readonly) tls_desired_.store(false);
  if (desired_.load() || tls_desired_.load()) gate.listeners_closed.store(false);
    if (desired_.load() && !server_) {
      httpd_config_t c = HTTPD_DEFAULT_CONFIG();
      c.stack_size = 24576;
      c.max_uri_handlers = 26;  // 19 admin/motion/restore + 5 twin (+2 spare)
      c.max_open_sockets = 2;
      // A phone browser may open more connections than the two this listener
      // holds; the least recently used one is closed instead of stalling.
      c.lru_purge_enable = true;
      c.recv_wait_timeout = 3;
      c.send_wait_timeout = 3;
      c.task_priority = 2;
      http_readonly_=readonly;
      if (httpd_start(&server_, &c) == ESP_OK && !(readonly?registerReadOnlyHandlers(server_):registerHandlers(server_))) {
        if (httpd_stop(server_) == ESP_OK) server_ = nullptr;
        else { gate.noteFault(FaultSite::LISTENER_STOP_FAILED); gate.revoke(); gate.phase.store(NetworkPhase::FAULT); }
      }
      listening_.store(server_ != nullptr);
    }
    if (tls_desired_.load() && !tls_server_ && config::kTlsIdentityPresent &&
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >= 100000) {
      httpd_ssl_config_t c = HTTPD_SSL_CONFIG_DEFAULT();
      c.httpd.stack_size = 28672;
      c.httpd.max_uri_handlers = 17;  // 10 motion/auth + 5 twin (+2 spare)
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
          !registerHandlers(tls_server_, false)) {
        if (httpd_ssl_stop(tls_server_) == ESP_OK) tls_server_ = nullptr;
        else { gate.noteFault(FaultSite::LISTENER_STOP_FAILED); gate.revoke(); gate.phase.store(NetworkPhase::FAULT); }
      }
      tls_listening_.store(tls_server_ != nullptr);
    }
    if (!desired_.load() && server_) {
      if (httpd_stop(server_) == ESP_OK) { server_ = nullptr; listening_.store(false); }
    }
    if (!tls_desired_.load() && tls_server_) {
      if (httpd_ssl_stop(tls_server_) == ESP_OK) { tls_server_ = nullptr; tls_listening_.store(false); }
    }
  if (gate.closing.load() && !server_ && !tls_server_)
    gate.listeners_closed.store(true); // successful stops, never a timeout
}
bool HttpTransport::admitAdmin(HttpTransportRequest &request,const char** failure) {
  auto &gate = service_->networkAdminAccess();
  request.admin_generation = gate.generation();
  if(!gate.allowed(request.admin_generation,failure))return false;
  return WifiManager::verifyAdminRadio(failure);
}
bool HttpTransport::motionStaAdmitted(bool tls,uint32_t local_ipv4,uint32_t generation,const char** failure) const {
  auto deny=[&](const char* why){if(failure)*failure=why;return false;};
  const auto& w=service_->wifiStatus();auto& gate=service_->networkAdminAccess();
  wifi_mode_t mode=WIFI_MODE_NULL;wifi_ap_record_t ap{};
  if(!tls)return deny("STA_TLS_REQUIRED");
  if(!local_ipv4||local_ipv4!=w.ipv4)return deny("STA_SOCKET_IP_MISMATCH");
  if(generation!=gate.generation())return deny("GATE_GENERATION_CHANGED");
  if(gate.closing.load())return deny("GATE_CLOSING");
  if(gate.radio_fault.load())return deny("GATE_RADIO_FAULT");
  if(gate.phase.load()!=NetworkPhase::CLIENT_STA)return deny("GATE_NOT_CLIENT_STA");
  if(!w.run_mode||!w.connected||static_cast<RoleReason>(w.role_reason)!=RoleReason::RUN_STA)return deny("STA_RUN_LINK_REQUIRED");
  if(esp_wifi_get_mode(&mode)!=ESP_OK||mode!=WIFI_MODE_STA)return deny("STA_NOT_EXCLUSIVE");
  if(esp_wifi_sta_get_ap_info(&ap)!=ESP_OK)return deny("STA_DRIVER_DISCONNECTED");
  return true;
}
bool HttpTransport::motionStaAdmitted(const HttpTransportRequest& r) const {
  return !r.ap_socket&&motionStaAdmitted(r.tls,r.local_ipv4,r.admin_generation);
}
bool HttpTransport::admitMotion(HttpTransportRequest& r,const char** failure){
  if(r.ap_socket)return admitAdmin(r,failure);
  r.admin_generation=service_->networkAdminAccess().generation();
  return motionStaAdmitted(r.tls,r.local_ipv4,r.admin_generation,failure);
}
bool HttpTransport::dispatch(const HttpTransportRequest &request,
                             HttpTransportResponse *response, char* large_text, size_t large_cap,
                             uint32_t budget_ms, const char** failure) {
  const uint32_t entered = millis();
  auto fail = [&](const char* why) { if (failure) *failure = why; return false; };
  if (!budget_ms || budget_ms > kDispatchTimeoutMs) budget_ms = kDispatchTimeoutMs;
  if (xSemaphoreTake(slot_free_, pdMS_TO_TICKS(budget_ms)) != pdTRUE) {
    dispatch_busy_.fetch_add(1);
    return fail("MAILBOX_BUSY");
  }
  xSemaphoreTake(response_ready_,
                 0); // drain prior late wake; IDs remain authoritative
  if (request.kind >= HttpRequestKind::WIFI_STATUS &&
      !((request.kind>=HttpRequestKind::MOTION_STATUS||request.kind==HttpRequestKind::WIFI_STATUS)&&motionStaAdmitted(request)) && !readOnlyAdmitted(request) &&
      (!service_->networkAdminAccess().allowed(request.admin_generation) ||
       !WifiManager::verifyAdminRadio())) {
    xSemaphoreGive(slot_free_); return fail("ADMIN_GATE_CLOSED");
  }
  pending_request_ = request;
  const uint32_t id = sequence_.fetch_add(1) + 1;
  pending_request_.sequence = id;
  mailbox_.beginDispatch();
  xSemaphoreGive(request_ready_);
  // The slot wait already consumed part of the budget.
  const uint32_t since = entered;
  do {
    const uint32_t elapsed = millis() - since;
    if (elapsed >= budget_ms)
      break;
    if (xSemaphoreTake(response_ready_,
                       pdMS_TO_TICKS(budget_ms - elapsed)) != pdTRUE)
      break;
    if (ready_sequence_.load() == id) {
      *response = pending_response_;
      if(request.kind==HttpRequestKind::MOTION_CAPABILITIES && response->ok) {
        // Copy while the mailbox slot is held. The Controller never receives
        // the caller's heap pointer, so abandoned requests cannot write freed memory.
        if(!large_text || response->text_len>=large_cap) {
          response->ok=false;response->message="CAPABILITY_BUFFER_REQUIRED";
        } else memcpy(large_text,motion_capabilities_,response->text_len+1);
      }
      xSemaphoreGive(slot_free_);
      const uint32_t took = millis() - entered;
      dispatch_last_ms_.store(took);
      if (took > dispatch_max_ms_.load()) dispatch_max_ms_.store(took);
      return true;
    }
  } while (millis() - since < budget_ms);
  dispatch_late_.fetch_add(1);
  { const uint32_t took = millis() - entered; dispatch_last_ms_.store(took);
    if (took > dispatch_max_ms_.load()) dispatch_max_ms_.store(took); }
  // If delivery won the atomic race, response bytes are already complete and
  // this reader owns release. Otherwise Controller releases after processing.
  if (!mailbox_.abandon())
    xSemaphoreGive(slot_free_);
  return fail("CONTROLLER_LOOP_LATE");
}
void HttpTransport::appendTransportStats(char* text,size_t cap){
  // Windowed maxima: each report starts a new window.
  const size_t n=strlen(text);if(!n||text[n-1]!='}')return;
  char extra[400];
  const int k=snprintf(extra,sizeof(extra),",\"transport\":{\"loop_last_ms\":%lu,\"loop_max_ms\":%lu,\"dispatch_last_ms\":%lu,\"dispatch_max_ms\":%lu,\"late\":%lu,\"busy\":%lu,\"heap_free\":%lu,\"heap_min_free\":%lu,\"heap_largest\":%lu,\"loop_stack_min_free\":%lu}}",
    (unsigned long)loop_last_ms_.load(),(unsigned long)loop_max_ms_.exchange(0),(unsigned long)dispatch_last_ms_.load(),
    (unsigned long)dispatch_max_ms_.exchange(0),(unsigned long)dispatch_late_.load(),(unsigned long)dispatch_busy_.load(),
    (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),(unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
    (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),(unsigned long)uxTaskGetStackHighWaterMark(nullptr));
  if(k>0&&n-1+static_cast<size_t>(k)<cap)memcpy(text+n-1,extra,static_cast<size_t>(k)+1);
}
void HttpTransport::update(uint32_t now_ms) {
  if (!request_ready_)
    return;
  // Called once per Controller loop: the gap IS the loop period that bounds
  // every mailbox response. Measured, never inferred.
  if (last_update_ms_) {
    const uint32_t gap = now_ms - last_update_ms_;
    loop_last_ms_.store(gap);
    if (gap > loop_max_ms_.load()) loop_max_ms_.store(gap);
  }
  last_update_ms_ = now_ms;
  twin_deferred_.store(service_->motionEngaged());
  if (memcmp(previous_admin_digest_, service_->networkAdminDigest(), 32) != 0) {
    portal_.reset();
    memcpy(previous_admin_digest_, service_->networkAdminDigest(), 32);
  }
  auto &gate = service_->networkAdminAccess();
  if ((!portal_sta_&&!gate.allowed(portal_generation_)) ||
      (portal_sta_&&(gate.closing.load()||gate.phase.load()!=NetworkPhase::CLIENT_STA||!service_->wifiStatus().connected||!service_->wifiStatus().run_mode))) {
    portal_.reset();portal_generation_=0;portal_sta_=false;
  }
  tls_desired_.store(
      !manual_stop_.load() && !gate.closing.load() &&
      gate.phase.load() == NetworkPhase::CLIENT_STA && config::kTlsIdentityPresent &&
      ((service_->wifiStatus().run_mode&&service_->wifiStatus().connected) || service_->remoteUpdateAllowed() ||
       (session_.sessionAuthenticated() && service_->wifiStatus().connected)));
  if (session_.sessionAuthenticated() &&
      (session_.timedOut(now_ms) || !service_->wifiStatus().connected ||
       manual_stop_.load())) {
    ota_->abort();
    session_.reset();
  }
  if (xSemaphoreTake(request_ready_, 0) != pdTRUE)
    return;
  // A deferred request is served on a later pass only while its reader still
  // waits (bounded by its dispatch budget); once abandoned it is released
  // unexecuted, so it never runs late or twice.
  if (deferred_ && !mailbox_.awaitingResponse()) {
    deferred_ = false; xSemaphoreGive(slot_free_); return;
  }
  // STOP precedence: a Motion command never runs while a STOP waits in its
  // lane. The next pass applies the STOP first; the command is then refused
  // SUPERSEDED_BY_STOP by its own epoch.
  deferred_ = pending_request_.kind == HttpRequestKind::MOTION_COMMAND && stop_pending_.load();
  if (!deferred_) serviceRequest(now_ms);   // may defer on fence contention
  if (deferred_) { xSemaphoreGive(request_ready_); return; }
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

  auto &gate = service_->networkAdminAccess();
  const bool readonly=readOnlyAdmitted(req);
  if(req.kind>=HttpRequestKind::OTA_CHALLENGE&&req.kind<=HttpRequestKind::OTA_ABORT&&static_cast<RoleReason>(service_->wifiStatus().role_reason)==RoleReason::MAINTENANCE_STA_READONLY){resp.message="OTA_DENIED_MAINTENANCE_LAN";return;}
  const bool sta_motion=(req.kind>=HttpRequestKind::MOTION_STATUS||req.kind==HttpRequestKind::WIFI_STATUS)&&motionStaAdmitted(req);
  const bool admin_request = req.kind >= HttpRequestKind::WIFI_STATUS&&!sta_motion&&!readonly;
  AdminLease lease(gate);
  if (admin_request && (!gate.allowed(req.admin_generation) ||
                        !WifiManager::verifyAdminRadio())) {
    resp.message = "ADMIN_AP_REQUIRED"; return;
  }
  // Admissible facts but the fence is momentarily held (md_http_life verifies
  // the radio every 50 ms): contention is not an authorization fact. update()
  // retries on the next pass, where everything above is evaluated again.
  if (admin_request && !lease) { deferred_ = true; return; }
  if(!req.ap_socket&&!req.tls&&req.local_ipv4&&!readonly){resp.message="LAN_READ_ONLY_REQUIRED";return;}
  if ((admin_request||sta_motion)&&(portal_generation_!=req.admin_generation||portal_sta_!=sta_motion)) {
    portal_.reset();portal_generation_=req.admin_generation;portal_sta_=sta_motion;
  }
  if(sta_motion&&req.kind!=HttpRequestKind::MOTION_LOGIN&&req.kind!=HttpRequestKind::MOTION_SESSION&&
      req.kind!=HttpRequestKind::MOTION_LOGOUT&&!portal_.current(static_cast<uint64_t>(esp_timer_get_time())/1000,req.peer,req.cookie)){
    resp.message="AUTH_FAILED";return;
  }
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
    // Dashboard extension (2026-10-11). Appended after the original keys so
    // their order is unchanged. Same rule as above: cached state only, no
    // bus transaction, no NVS read. A value that is not known is printed as
    // not known, never as a stale number.
    statusExtension(resp, bms, cal);
    resp.ok = true;
    resp.message = "OK";
    break;
  }

  case HttpRequestKind::MOTION_STATUS:
    // The operational operator heartbeat (deadman) is an AP poll that proves
    // the admin session AND its CSRF token: only the Dashboard page, which
    // holds STOP, has both. The read-only Twin, anonymous and LAN read-only
    // polls never keep a session alive (2026-10-15).
  {
    const bool beat=!readonly&&portal_.authorize(static_cast<uint64_t>(esp_timer_get_time())/1000,req.peer,req.cookie,req.csrf);
    if(beat)service_->operatorHeartbeat(now_ms);
    resp.ok=service_->motionStatus(resp.text,sizeof(resp.text),now_ms);
    if(resp.ok&&readonly){const size_t n=strlen(resp.text);if(n&&n+28<sizeof(resp.text))strcpy(resp.text+n-1,",\"transport_readonly\":true}");else resp.ok=false;}
    // Tells the Dashboard whether THIS poll renewed the operator heartbeat, so
    // a lapsed admin session is shown (and resumed) instead of a silent deadman.
    if(resp.ok){const size_t n=strlen(resp.text);if(n&&n+20<sizeof(resp.text))strcpy(resp.text+n-1,beat?",\"heartbeat\":true}":",\"heartbeat\":false}");else resp.ok=false;}
  }
    if(resp.ok)appendTransportStats(resp.text,sizeof(resp.text));
    resp.message=resp.ok?"OK":"MOTION_STATUS_UNAVAILABLE";
    resp.text_len=resp.ok?strlen(resp.text):0;break;
  case HttpRequestKind::MOTION_CAPABILITIES:
    resp.ok=service_->motionCapabilities(motion_capabilities_,sizeof(motion_capabilities_));
    if(resp.ok&&readonly){const size_t n=strlen(motion_capabilities_);if(n&&n+28<sizeof(motion_capabilities_))strcpy(motion_capabilities_+n-1,",\"transport_readonly\":true}");else resp.ok=false;}
    resp.message=resp.ok?"OK":"MOTION_CAPABILITIES_UNAVAILABLE";
    resp.text_len=resp.ok?strlen(motion_capabilities_):0;break;
  case HttpRequestKind::MOTION_COMMAND: {
    if(!req.local_authorized || !portal_.authorize(static_cast<uint64_t>(esp_timer_get_time())/1000,
                                                  req.peer,req.cookie,req.csrf)) {
      resp.message="AUTH_FAILED";break;
    }
    // Received before a STOP that was applied meanwhile: never executed late.
    if(req.stop_epoch!=stop_epoch_.load()&&req.motion_command.intent!=motion::Intent::STOP){resp.message="SUPERSEDED_BY_STOP";break;}
    // A semantic ack is returned also for Controller refusals. No bus pointer,
    // actuator register, lease request or operational permit exists in Web.
    service_->operatorHeartbeat(now_ms);  // the command itself proves presence
    const auto ack=service_->motionCommand(req.motion_command,now_ms);
    resp.ok=service_->motionAck(ack,resp.text,sizeof(resp.text));
    resp.message=resp.ok?"OK":"ACK_UNAVAILABLE";
    resp.text_len=resp.ok?strlen(resp.text):0;break;
  }
  case HttpRequestKind::CALIBRATION_RESTORE: {
    // Same local-origin + admin session + CSRF proof as a Motion command.
    if(!req.ap_socket||!req.local_authorized||!portal_.authorize(static_cast<uint64_t>(esp_timer_get_time())/1000,
                                                req.peer,req.cookie,req.csrf)) {
      resp.message="AUTH_FAILED";break;
    }
    resp.ok=service_->calibrationRestore(req.restore_generation,req.restore_confirmed,resp.text,sizeof(resp.text));
    resp.message=resp.ok?"OK":"RESTORE_UNAVAILABLE";
    resp.text_len=resp.ok?strlen(resp.text):0;break;
  }
  case HttpRequestKind::WIFI_STATUS: {
    wifiResponse(resp,readonly||sta_motion||!portal_.current(static_cast<uint64_t>(esp_timer_get_time())/1000,req.peer,req.cookie));
    break;
  }
  case HttpRequestKind::MOTION_LOGIN:
  case HttpRequestKind::WIFI_LOGIN: {
    uint8_t random[64 + kPortalDeviceIdBytes];
    esp_fill_random(random, sizeof(random));
    resp.ok = req.local_authorized &&
              portal_.login(static_cast<uint64_t>(esp_timer_get_time()) / 1000,
                            req.peer, req.login,
                            gate.digest(), random);
    resp.message = resp.ok ? "OK" : "AUTH_FAILED";
    if (resp.ok) {
      strcpy(resp.cookie, portal_.cookie());
      strcpy(resp.text, portal_.csrf());
      resp.text_len = 64;
      // "Remember this device" is only ever issued by a token login.
      if (req.remember)
        portalDeviceIssue(gate.digest(), random + 64, resp.device);
    }
    memset(random, 0, sizeof(random));
    break;
  }
  case HttpRequestKind::MOTION_SESSION:
  case HttpRequestKind::WIFI_SESSION: {
    // Page load or expired session. The holder of the live session is told
    // its CSRF token again; otherwise a remembered-device credential opens a
    // fresh session. No token, no NVS access, no wall clock.
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time()) / 1000;
    bool resumed = false;
    if (req.local_authorized && !portal_.current(now, req.peer, req.cookie)) {
      uint8_t random[64];
      esp_fill_random(random, sizeof(random));
      resumed = portal_.resume(now, req.peer, req.device, gate.digest(), random);
      memset(random, 0, sizeof(random));
    }
    resp.ok = req.local_authorized &&
              portal_.current(now, req.peer, resumed ? portal_.cookie() : req.cookie);
    resp.message = resp.ok ? "OK" : "AUTH_FAILED";
    if (resp.ok) {
      strcpy(resp.cookie, portal_.cookie());
      strcpy(resp.text, portal_.csrf());
      resp.text_len = 64;
      // Sent back only after it verified, so the browser renews its lifetime.
      if (resumed)
        memcpy(resp.device, req.device, sizeof(resp.device));
    }
    break;
  }
  case HttpRequestKind::MOTION_LOGOUT:
  case HttpRequestKind::WIFI_LOGOUT: {
    // Ends the session if the caller holds it; the response clears the
    // browser's cookies either way. A remembered device is not individually
    // revocable: a new administrator token revokes all of them.
    if (!req.local_authorized) {
      resp.message = "AUTH_FAILED";
      break;
    }
    if (portal_.current(static_cast<uint64_t>(esp_timer_get_time()) / 1000,
                        req.peer, req.cookie))
      portal_.reset();
    resp.ok = true;
    resp.message = "LOGGED_OUT";
    break;
  }
  case HttpRequestKind::WIFI_CONFIG:
  case HttpRequestKind::WIFI_SCAN:
  case HttpRequestKind::WIFI_AP: {
    if (!req.local_authorized ||
        !portal_.authorize(static_cast<uint64_t>(esp_timer_get_time()) / 1000,
                           req.peer, req.cookie, req.csrf)) {
      resp.message = "AUTH_FAILED";
      break;
    }
    if (session_.sessionAuthenticated() ||
        service_->otaStatus().holds_actuator_inhibit) {
      resp.message = "OTA_BUSY";
      break;
    }
    if (!gate.allowed(req.admin_generation) || !WifiManager::verifyAdminRadio()) {
      resp.message = "ADMIN_REVOKED"; break;
    }
    if (req.kind == HttpRequestKind::WIFI_SCAN) {
      resp.message = "SCAN_DISABLED_ADMIN_AP"; break;
    }
    if (req.kind == HttpRequestKind::WIFI_CONFIG)
      resp.ok = service_->configureNetwork(req.patch, req.admin_generation);
    if (req.kind == HttpRequestKind::WIFI_SCAN)
      resp.ok = service_->scanNetwork();
    if (req.kind == HttpRequestKind::WIFI_AP)
      resp.ok = service_->accessPoint(req.ap_on, req.admin_generation);
    resp.message =
        resp.ok ? (req.kind == HttpRequestKind::WIFI_CONFIG ? "ADMIN_CLOSING_STA_TEST_RELOGIN_REQUIRED"
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

void HttpTransport::statusExtension(
    HttpTransportResponse &resp, const power::DalySample &bms,
    const calibration::CalibrationSessionStatus &cal) {
  size_t used = resp.text_len;
  auto add = [&](const char *format, ...) {
    if (used >= sizeof(resp.text) - 1)
      return;
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(resp.text + used, sizeof(resp.text) - used, format, args);
    va_end(args);
    // A line that does not fit is dropped whole, never sent truncated.
    if (n > 0 && static_cast<size_t>(n) < sizeof(resp.text) - used)
      used += static_cast<size_t>(n);
    else
      resp.text[used] = 0;
  };
  const uint32_t now = millis(); // DALY may have stamped after the tick sample
  const bool valid = service_->bmsHasValidSample();
  const uint32_t sample_age = valid ? now - bms.sampled_at_ms : 0;
  const auto &wifi = service_->wifiStatus();
  const auto &led = service_->ledSnapshot();
  const auto &census = service_->servoCensusResult();
  add("uptime_ms=%lu\nauthority_inhibited=%s\nauthority_inhibit_reason=%s\n",
      (unsigned long)service_->uptimeMillis(now),
      service_->authorityInhibited() ? "YES" : "NO",
      core::toString(service_->authorityInhibitReason()));
  add("imu_detected=%s\nbms_detected=%s\nservo_detected=%s\nled_detected=%s\n",
      core::toString(service_->imuAvailability().detected),
      core::toString(service_->bmsAvailability().detected),
      core::toString(service_->servoAvailability().detected),
      core::toString(service_->ledAvailability().detected));
  add("imu_runtime_resets=%lu\n", (unsigned long)service_->imuRuntimeResetCount());
  // Fresh is the controller's own contract (power/DalyProtocol.h): a valid
  // sample, the latest poll OK, and an age within kDalyTelemetryFreshnessMs.
  add("bms_sample_age_ms=%lu\nbms_fresh=%s\nbms_fresh_limit_ms=%lu\n",
      (unsigned long)sample_age,
      valid && service_->bmsLastCommResult() == power::DalyCommResult::OK &&
              sample_age <= power::kDalyTelemetryFreshnessMs
          ? "YES"
          : "NO",
      (unsigned long)power::kDalyTelemetryFreshnessMs);
  if (valid) {
    add("bms_current_a=%.1f\nbms_state=%s\nbms_charge_mos=%s\n"
        "bms_discharge_mos=%s\nbms_alarms=%04X %04X %04X %04X\n",
        bms.pack_current_a, bms.state_name, bms.charge_mos_on ? "ON" : "OFF",
        bms.discharge_mos_on ? "ON" : "OFF", bms.alarms[0], bms.alarms[1],
        bms.alarms[2], bms.alarms[3]);
    // Capacity (Dashboard 2026-10-17): the DALY 0xD2 remaining-capacity
    // register as reported (a BMS coulomb count, not validated on MATDOG), the
    // configured nominal pack, and SOC x nominal labelled as an ESTIMATE.
    // Display only: nothing here feeds a protection, permit or LED threshold.
    add("bms_remaining_ah=%.1f\nbms_capacity_config_ah=%.1f\n"
        "bms_remaining_estimate_ah=%.2f\n",
        bms.remaining_ah, power::kPackNominalCapacityAh,
        bms.soc_percent >= 0 && bms.soc_percent <= 100
            ? bms.soc_percent * power::kPackNominalCapacityAh / 100.0f
            : 0.0f);
    add("bms_cells=%u\nbms_cell_min_mv=%u\nbms_cell_max_mv=%u\n"
        "bms_cell_delta_mv=%u\n",
        (unsigned)bms.cell_count, (unsigned)bms.cell_min_mv,
        (unsigned)bms.cell_max_mv, (unsigned)bms.cell_delta_mv);
    char cells[8 * 6 + 1];
    size_t w = 0;
    for (unsigned i = 0; i < bms.cell_count && i < 8; ++i)
      w += snprintf(cells + w, sizeof(cells) - w, "%s%u", i ? "," : "",
                    (unsigned)bms.cell_mv[i]);
    cells[w] = 0;
    add("bms_cell_mv=%s\n", cells);
  }
  // Compile-time population facts plus the verdict of the last census, if
  // one ran. NOT_RUN stays NOT_RUN: no population is implied.
  add("servo_canonical=%u\nservo_expected_now=%u\nservo_census=%s\n",
      (unsigned)servo::canonicalAllocatedCount(),
      (unsigned)servo::expectedNowCount(), servo::toString(census.verdict));
  if (census.verdict != servo::CensusVerdict::NOT_RUN)
    add("servo_census_present=%u\nservo_census_missing=%u\n",
        (unsigned)census.present_expected, (unsigned)census.missing_expected);
  add("calibration_holds_authority=%s\ncalibration_stale=%s\n",
      cal.holds_authority ? "YES" : "NO",
      cal.current_calibration_stale ? "YES" : "NO");
  const auto &persist = service_->calibrationPersistence();
  if (persist.published)
    add("calibration_available=%s\ncalibration_persist_verdict=%s\n"
        "calibration_ack_generation=%lu\ncalibration_record_intact=%s\n"
        "calibration_nvs=%s\n",
        persist.available ? "YES" : "NO", persist.verdict,
        (unsigned long)persist.acknowledged_generation,
        persist.record_intact ? "YES" : "NO", persist.nvs);
  add("led_soc_valid=%s\nled_soc_segments=%u\nled_charging=%s\n"
      "led_charging_fault=%s\nled_battery_warning=%s\n"
      "led_battery_critical=%s\nled_diagnostic=%s\n",
      led.soc_valid ? "YES" : "NO", (unsigned)led.soc_segments,
      led.charging ? "YES" : "NO", led.charging_fault ? "YES" : "NO",
      led.battery_warning ? "YES" : "NO", led.battery_critical ? "YES" : "NO",
      status::toString(service_->ledDiagnostic()));
  if (led.soc_valid)
    add("led_soc_percent=%.1f\n", led.soc_percent);
  add("network_phase=%s\nwifi_enabled=%s\nap_active=%s\nap_clients=%u\n"
      "heap_free=%lu\nheap_min_free=%lu\nloop_stack_min_free=%lu\n",
      toString(service_->networkAdminAccess().phase.load()),
      wifi.enabled ? "YES" : "NO", wifi.ap_active ? "YES" : "NO",
      (unsigned)wifi.ap_clients,
      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
      // Lowest free stack ever seen on the Controller task, which formats
      // this response: evidence that the portal stays inside its budget.
      (unsigned long)uxTaskGetStackHighWaterMark(nullptr));
  resp.text_len = used;
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
  if(!socketFacts(req,r)||(!self->admitReadOnly(r)&&!self->admitMotion(r)))
    return httpd_resp_send_err(req,HTTPD_403_FORBIDDEN,"READ_TRANSPORT_REQUIRED");
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

void HttpTransport::wifiResponse(HttpTransportResponse &resp,bool redact_network_names) {
  auto w = service_->wifiStatus();
  const bool readonly=static_cast<RoleReason>(w.role_reason)==RoleReason::MAINTENANCE_STA_READONLY;
  if(redact_network_names)redactNetworkNames(w);

  char ip[16], ssid[67], bssid[37], ap[67], test_ip[16], test_ssid[67];
  formatIpv4(w.sta_test.ipv4, test_ip, 16);
  jsonString(w.sta_test.ssid, test_ssid, sizeof(test_ssid));
  formatIpv4(w.ipv4, ip, 16);
  jsonString(w.ssid, ssid, sizeof(ssid));
  jsonString(w.bssid, bssid, sizeof(bssid));
  jsonString(w.ap_ssid, ap, sizeof(ap));
  int n = snprintf(
      resp.text, sizeof(resp.text),
      "{\"network_phase\":\"%s\",\"admin_generation\":%lu,\"build_id\":\"%s\",\"ota_ingest\":%u,\"tls\":%s,\"connected\":%s,"
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
      toString(service_->networkAdminAccess().phase.load()),
      (unsigned long)service_->networkAdminAccess().generation(),
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
      "\"connect_timeouts\":%lu,\"link_losses\":%lu,"
      "\"sta_enabled\":%s,\"sta_credentials\":%s,\"sta_state\":\"%s\","
      "\"sta_fault\":\"%s\",\"admin_provisioned\":%s,"
      "\"sta_test\":{\"seq\":%lu,\"outcome\":\"%s\",\"ssid\":\"%s\","
      "\"profile\":%u,\"associated\":%s,\"reason\":%u,\"attempts\":%u,"
      "\"ip\":\"%s\",\"rssi\":%ld,\"channel\":%u,\"elapsed_ms\":%lu,"
      "\"rolled_back\":%s},"
      "\"policy\":{\"run\":%s,\"role\":\"%s\",\"recovery_level\":%u,"
      "\"retry_in_ms\":%lu},\"scan\":[",
      w.ap_reload_pending ? "true" : "false",
      w.sleep_apply_ok ? "true" : "false", w.bandwidth_configured,
      w.bandwidth_apply_ok ? "true" : "false", (unsigned long)w.max_update_us,
      (unsigned long)w.counters.connects,
      (unsigned long)w.counters.connect_timeouts,
      (unsigned long)w.counters.link_losses, w.enabled ? "true" : "false",
      w.credentials_present ? "true" : "false", toString(w.state),
      toString(w.fault), w.admin_provisioned ? "true" : "false",
      (unsigned long)w.sta_test.seq,
      toString(static_cast<StaTestOutcome>(w.sta_test.outcome)), test_ssid,
      w.sta_test.profile, w.sta_test.associated ? "true" : "false",
      w.sta_test.reason, w.sta_test.attempts, test_ip,
      (long)w.sta_test.rssi_dbm, w.sta_test.channel,
      (unsigned long)w.sta_test.elapsed_ms,
      w.sta_test.rolled_back ? "true" : "false", w.run_mode ? "true" : "false",
      toString(static_cast<RoleReason>(w.role_reason)), w.recovery_level,
      (unsigned long)w.recovery_retry_ms);
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
  n = snprintf(resp.text + used, sizeof(resp.text) - used, "],\"read_only\":%s}",readonly?"true":"false");
  if (n < 0 || static_cast<size_t>(n) >= sizeof(resp.text) - used) {
    resp.message = "STATUS_OVERFLOW";
    return;
  }
  resp.text_len = used + n;
  resp.ok = true;
  resp.message = "OK";
}
esp_err_t HttpTransport::handlePage(httpd_req_t *req) {
  auto *self = static_cast<HttpTransport *>(req->user_ctx);
  HttpTransportRequest r{};
  if(!socketFacts(req,r)||(!self->admitReadOnly(r)&&!self->admitMotion(r)))
    return httpd_resp_send_err(req,HTTPD_403_FORBIDDEN,"MOTION_TRANSPORT_REQUIRED");
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
  httpd_resp_set_hdr(
      req, "Content-Security-Policy",
      kDashboardContentSecurityPolicy);
  return httpd_resp_send(req, kPortalPage, sizeof(kPortalPage) - 1);
}
esp_err_t HttpTransport::handleMotion(httpd_req_t* req) {
  auto* self=static_cast<HttpTransport*>(req->user_ctx);
  const uint32_t t0=millis();
  HttpTransportRequest r{};
  if(req->method==HTTP_GET)r.kind=!strcmp(req->uri,"/api/motion/status")?HttpRequestKind::MOTION_STATUS:HttpRequestKind::MOTION_CAPABILITIES;
  const char* admission_failure="SOCKET_FACTS";
  if(!socketFacts(req,r,&admission_failure) || (!(req->method==HTTP_GET&&self->admitReadOnly(r))&& !self->admitMotion(r,&admission_failure))){
    httpd_resp_set_hdr(req,"X-Matdog-Admission",admission_failure);
    return sendJsonError(req,"403 Forbidden",admission_failure);
  }
  char session_cookie[256]{};httpd_req_get_hdr_value_str(req,"Cookie",session_cookie,sizeof(session_cookie));
  cookieValue(session_cookie,"md_session",r.cookie);
  if(req->method==HTTP_GET) {
    r.kind=strcmp(req->uri,"/api/motion/status")==0?HttpRequestKind::MOTION_STATUS:HttpRequestKind::MOTION_CAPABILITIES;
    httpd_req_get_hdr_value_str(req,"X-Csrf-Token",r.csrf,sizeof(r.csrf));  // heartbeat proof only, never a write
  } else {
    if(!r.local_authorized || req->content_len<=0 || req->content_len>1024)
      return sendJsonError(req,"400 Bad Request","ORIGIN_OR_BOUNDED_BODY_REQUIRED");
    char type[64]{};httpd_req_get_hdr_value_str(req,"Content-Type",type,sizeof(type));
    if(strcmp(type,"application/json"))return sendJsonError(req,"400 Bad Request","JSON_REQUIRED");
    char body[1025]{};size_t used=0;
    if(!readBoundedBody(req,body,sizeof(body),&used))return sendJsonError(req,"400 Bad Request","BODY_TIMEOUT_OR_TRUNCATED");
    const char* reason=nullptr;
    if(!core::parseMotionJson(body,used,&r.motion_command,&reason))
      return sendJsonError(req,"400 Bad Request",reason);
    // The AP transport identifies the client; owner is not a bus authority.
    if(r.motion_command.owner!=motion::ClientOwner::DASHBOARD)
      return sendJsonError(req,"403 Forbidden","DASHBOARD_OWNER_REQUIRED");
    char cookie[256]{};httpd_req_get_hdr_value_str(req,"Cookie",cookie,sizeof(cookie));
    cookieValue(cookie,"md_session",r.cookie);
    httpd_req_get_hdr_value_str(req,"X-Csrf-Token",r.csrf,sizeof(r.csrf));
    r.kind=HttpRequestKind::MOTION_COMMAND;
    r.stop_epoch=self->stop_epoch_.load();
    if(r.motion_command.intent==motion::Intent::STOP){
      // STOP lane: never waits for the ordinary mailbox (a status or command
      // request may hold it for up to its budget). The Controller validates
      // and applies it first thing in its next loop.
      const uint32_t ticket=self->stop_ticket_.fetch_add(1)+1;r.sequence=ticket;
      while(self->stop_lock_.exchange(true,std::memory_order_acquire)){vTaskDelay(1);}
      self->stop_request_=r;self->stop_pending_.store(true);self->stop_lock_.store(false,std::memory_order_release);
      while(self->stop_done_.load()!=ticket&&millis()-t0<kStopLaneBudgetMs)vTaskDelay(pdMS_TO_TICKS(2));
      if(self->stop_done_.load()!=ticket)return sendJsonError(req,"503 Service Unavailable","STOP_LANE_LATE",millis()-t0);
      if(!self->stop_ack_ok_.load())return sendJsonError(req,"403 Forbidden","AUTH_FAILED",millis()-t0);
      httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");
      return httpd_resp_send(req,self->stop_ack_,strlen(self->stop_ack_));
    }
  }
  std::unique_ptr<char[]> catalog;
  if(r.kind==HttpRequestKind::MOTION_CAPABILITIES) {
    catalog.reset(new(std::nothrow) char[kMotionCapabilitiesBytes]);
    if(!catalog)return sendJsonError(req,"503 Service Unavailable","CAPABILITY_BUFFER_UNAVAILABLE",millis()-t0);
  }
  HttpTransportResponse resp{};const char* failure="CONTROLLER_LOOP_LATE";
  // Reads have a shorter, explicit budget; a command keeps the full one.
  const uint32_t budget=r.kind==HttpRequestKind::MOTION_COMMAND?kDispatchTimeoutMs:kMotionReadBudgetMs;
  if(!self->dispatch(r,&resp,catalog.get(),catalog?kMotionCapabilitiesBytes:0,budget,&failure))
    return sendJsonError(req,"503 Service Unavailable",failure,millis()-t0);
  if(!resp.ok)return sendJsonError(req,"403 Forbidden",resp.message,millis()-t0);
  httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");
  return httpd_resp_send(req,catalog?catalog.get():resp.text,resp.text_len);
}
bool HttpTransport::serviceStopLane(uint32_t now_ms) {
  if(!stop_pending_.load())return false;
  // Fence first, before the STOP is consumed: while md_http_life holds it the
  // STOP stays pending (no ACK, not done) for the next service point - loop
  // start, between servo reads, bus preemption - within the lane budget.
  auto &gate = service_->networkAdminAccess();
  AdminLease lease(gate);
  if(!lease)return false;
  // Only the fields the STOP admission needs (the full request is ~2 KB of
  // loop stack). Copied under the lane lock.
  struct { uint32_t sequence,peer,admin_generation,local_ipv4; bool local_authorized,ap_socket,tls; char cookie[65],csrf[65]; motion::MotionCommand motion_command; } req;
  if(stop_lock_.exchange(true,std::memory_order_acquire))return false;   // writer mid-copy: next loop
  req.sequence=stop_request_.sequence;req.peer=stop_request_.peer;req.admin_generation=stop_request_.admin_generation;
  req.local_authorized=stop_request_.local_authorized;req.ap_socket=stop_request_.ap_socket;
  req.tls=stop_request_.tls;req.local_ipv4=stop_request_.local_ipv4;
  memcpy(req.cookie,stop_request_.cookie,sizeof(req.cookie));memcpy(req.csrf,stop_request_.csrf,sizeof(req.csrf));
  req.motion_command=stop_request_.motion_command;
  stop_pending_.store(false);stop_lock_.store(false,std::memory_order_release);
  bool applied=false;stop_ack_ok_.store(false);stop_ack_[0]=0;
  // Recheck the same AP or RUN_STA/TLS admission as serviceRequest under
  // its fence. Socket facts were captured by the handler; generation/link and
  // portal remain live checks. No mailbox wait for either admitted transport.
  const bool sta_motion=!req.ap_socket&&motionStaAdmitted(req.tls,req.local_ipv4,req.admin_generation);
  const bool admitted=lease&&(sta_motion||(req.ap_socket&&gate.allowed(req.admin_generation)&&WifiManager::verifyAdminRadio()));
  if(admitted&&(portal_generation_!=req.admin_generation||portal_sta_!=sta_motion)){
    portal_.reset();portal_generation_=req.admin_generation;portal_sta_=sta_motion;
  }
  if(admitted&&req.local_authorized&&portal_.authorize(static_cast<uint64_t>(esp_timer_get_time())/1000,req.peer,req.cookie,req.csrf)){
    stop_epoch_.fetch_add(1);             // every command received before this STOP is superseded
    service_->operatorHeartbeat(now_ms);
    const auto ack=service_->motionCommand(req.motion_command,now_ms);
    stop_ack_ok_.store(service_->motionAck(ack,stop_ack_,sizeof(stop_ack_)));applied=true;
  }
  stop_done_.store(req.sequence);
  return applied;
}

esp_err_t HttpTransport::handleCalibrationRestore(httpd_req_t* req) {
  auto* self=static_cast<HttpTransport*>(req->user_ctx);
  const uint32_t t0=millis();
  HttpTransportRequest r{};r.kind=HttpRequestKind::CALIBRATION_RESTORE;
  // AP admin portal only: never the LAN, never the TLS router listener.
  if(!socketFacts(req,r)||!r.ap_socket||!self->admitAdmin(r))return sendJsonError(req,"403 Forbidden","ADMIN_AP_REQUIRED");
  if(!r.local_authorized)return sendJsonError(req,"403 Forbidden","LOCAL_ORIGIN_REQUIRED");
  char type[64]{};httpd_req_get_hdr_value_str(req,"Content-Type",type,sizeof(type));
  if(strcmp(type,"application/json"))return sendJsonError(req,"400 Bad Request","JSON_REQUIRED");
  char body[257]{};size_t used=0;
  if(!readBoundedBody(req,body,sizeof(body),&used))return sendJsonError(req,"400 Bad Request","BODY_TIMEOUT_OR_TRUNCATED");
  const char* reason=nullptr;
  if(!parseRestoreRequestJson(body,used,&r.restore_generation,&r.restore_confirmed,&reason))
    return sendJsonError(req,"400 Bad Request",reason);
  char cookie[256]{};httpd_req_get_hdr_value_str(req,"Cookie",cookie,sizeof(cookie));
  cookieValue(cookie,"md_session",r.cookie);
  httpd_req_get_hdr_value_str(req,"X-Csrf-Token",r.csrf,sizeof(r.csrf));
  HttpTransportResponse resp{};const char* failure="CONTROLLER_LOOP_LATE";
  if(!self->dispatch(r,&resp,nullptr,0,kDispatchTimeoutMs,&failure))
    return sendJsonError(req,"503 Service Unavailable",failure,millis()-t0);
  if(!resp.ok)return sendJsonError(req,"403 Forbidden",resp.message,millis()-t0);
  httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");
  return httpd_resp_send(req,resp.text,resp.text_len);
}

// Every Motion/RESTORE refusal is JSON with the real HTTP status, so a client
// never has to guess from a non-JSON body (14 Oct: HTTP_INVALID_JSON hid it).
esp_err_t HttpTransport::sendJsonError(httpd_req_t* req,const char* status,const char* code,uint32_t elapsed_ms){
  char safe[64]{};size_t k=0;
  for(const char* c=code?code:"ERROR";*c&&k+1<sizeof(safe);++c)
    safe[k++]=((*c>='A'&&*c<='Z')||(*c>='a'&&*c<='z')||(*c>='0'&&*c<='9')||*c=='_')?*c:'_';
  char body[128];
  snprintf(body,sizeof(body),"{\"version\":1,\"ok\":false,\"error\":\"%s\",\"elapsed_ms\":%lu}",safe,(unsigned long)elapsed_ms);
  httpd_resp_set_status(req,status);httpd_resp_set_type(req,"application/json");
  httpd_resp_set_hdr(req,"Cache-Control","no-store");
  return httpd_resp_sendstr(req,body);
}
bool HttpTransport::readBoundedBody(httpd_req_t* req,char* body,size_t cap,size_t* used){
  *used=0;const uint32_t since=millis();
  if(req->content_len<=0||static_cast<size_t>(req->content_len)>=cap)return false;
  while(*used<static_cast<size_t>(req->content_len)){
    if(millis()-since>3000)return false;
    const int got=httpd_req_recv(req,body+*used,req->content_len-*used);
    if(got<=0)return false;
    *used+=got;
  }
  body[*used]=0;return true;
}
esp_err_t HttpTransport::handleTwin(httpd_req_t* req) {
  // Public display assets (no secret, no calibration, no command path). They
  // are served from flash in the httpd task: no Controller mailbox involved.
  HttpTransportRequest r{};
  if(!socketFacts(req,r))return sendJsonError(req,"403 Forbidden","SOCKET_FACTS");
  char path[64]{};size_t n=0;while(req->uri[n]&&req->uri[n]!='?'&&n+1<sizeof(path)){path[n]=req->uri[n];++n;}
  if(!strcmp(path,"/twin")){
    httpd_resp_set_status(req,"302 Found");httpd_resp_set_hdr(req,"Location","/twin/");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_send(req,nullptr,0);
  }
  const twin_assets::Asset* asset=nullptr;
  for(size_t i=0;i<twin_assets::kAssetCount;++i)if(!strcmp(twin_assets::kAssets[i].path,path))asset=&twin_assets::kAssets[i];
  if(!asset)return sendJsonError(req,"404 Not Found","TWIN_ASSET_UNKNOWN");
  char tag[48]{};
  if(httpd_req_get_hdr_value_str(req,"If-None-Match",tag,sizeof(tag))==ESP_OK&&!strcmp(tag,asset->etag)){
    httpd_resp_set_status(req,"304 Not Modified");httpd_resp_set_hdr(req,"ETag",asset->etag);return httpd_resp_send(req,nullptr,0);
  }
  auto* self=static_cast<HttpTransport*>(req->user_ctx);
  if(asset->gz_len>kTwinDeferBytes){
    httpd_resp_set_hdr(req,"Retry-After","5");
    return sendJsonError(req,"503 Service Unavailable",self&&self->twin_deferred_.load()?"TWIN_DEFERRED_MOTION_ACTIVE":"TWIN_HOST_REQUIRED");
  }
  httpd_resp_set_type(req,asset->type);
  httpd_resp_set_hdr(req,"Content-Encoding","gzip");
  httpd_resp_set_hdr(req,"ETag",asset->etag);
  httpd_resp_set_hdr(req,"Cache-Control","no-cache");
  httpd_resp_set_hdr(req,"X-Content-Type-Options","nosniff");
  return httpd_resp_send(req,reinterpret_cast<const char*>(asset->gz),static_cast<ssize_t>(asset->gz_len));
}
esp_err_t HttpTransport::handleWifi(httpd_req_t *req) {
  auto *self = static_cast<HttpTransport *>(req->user_ctx);
  HttpTransportRequest r{};
  const char* admission_failure="SOCKET_FACTS";
  if(!socketFacts(req,r,&admission_failure)){
    httpd_resp_set_hdr(req,"X-Matdog-Admission",admission_failure);
    return sendJsonError(req,"403 Forbidden",admission_failure);
  }
  const bool motion_auth=!strcmp(req->uri,"/api/motion/login")||!strcmp(req->uri,"/api/motion/session")||!strcmp(req->uri,"/api/motion/logout");
  const bool ro=req->method==HTTP_GET&&!strcmp(req->uri,"/wifi/status");
  if(ro)r.kind=HttpRequestKind::WIFI_STATUS;
  const bool admitted=ro?(self->admitReadOnly(r)||self->admitMotion(r,&admission_failure)):
    motion_auth?self->admitMotion(r,&admission_failure):self->admitAdmin(r,&admission_failure);
  if(!admitted){
    httpd_resp_set_hdr(req,"X-Matdog-Admission",admission_failure);
    return sendJsonError(req,"403 Forbidden",admission_failure);
  }
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
    if (strcmp(req->uri, "/wifi/login") == 0||strcmp(req->uri,"/api/motion/login")==0) {
      r.kind = motion_auth?HttpRequestKind::MOTION_LOGIN:HttpRequestKind::WIFI_LOGIN;
      // token=<64 hex>, optionally followed by &remember=1 and nothing else.
      r.remember = !motion_auth && read == 81 && memcmp(body + 70, "&remember=1", 11) == 0;
      if ((read != 70 && !r.remember) || strncmp(body, "token=", 6) != 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid login");
      memcpy(r.login, body + 6, 64);
    } else {
      char cookie[256]{};
      httpd_req_get_hdr_value_str(req, "Cookie", cookie, sizeof(cookie));
      cookieValue(cookie, "md_session", r.cookie);
      if (strcmp(req->uri, "/wifi/session") == 0)
        cookieValue(cookie, "md_device", r.device);
      memset(cookie, 0, sizeof(cookie));
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
      } else if (strcmp(req->uri, "/wifi/session") == 0 ||
                 strcmp(req->uri, "/wifi/logout") == 0 || motion_auth) {
        r.kind=motion_auth?(!strcmp(req->uri,"/api/motion/session")?HttpRequestKind::MOTION_SESSION:HttpRequestKind::MOTION_LOGOUT):
          (req->uri[6]=='s'?HttpRequestKind::WIFI_SESSION:HttpRequestKind::WIFI_LOGOUT);
        if (read)
          return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                     "Empty body required");
      } else {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown action");
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
  // The session cookie is short-lived and goes with every /wifi/ request. The
  // remembered-device cookie is long-lived (400 days, renewed on each use) and
  // is scoped to /wifi/session, so it is sent only to open a session.
  char cookie[160], device[176];
  if (r.kind == HttpRequestKind::WIFI_LOGIN ||
      r.kind == HttpRequestKind::WIFI_SESSION||r.kind==HttpRequestKind::MOTION_LOGIN||r.kind==HttpRequestKind::MOTION_SESSION) {
    snprintf(
        cookie, sizeof(cookie),
        "md_session=%s; HttpOnly; SameSite=Strict; Path=/; Max-Age=300%s",
        resp.cookie,r.tls?"; Secure":"");
    httpd_resp_set_hdr(req, "Set-Cookie", cookie);
    if (resp.device[0]) {
      snprintf(device, sizeof(device),
               "md_device=%s; HttpOnly; SameSite=Strict; Path=/wifi/session; Max-Age=34560000",
               resp.device);
      httpd_resp_set_hdr(req, "Set-Cookie", device);
    }
    return httpd_resp_send(req, resp.text, resp.text_len);
  }
  if (r.kind == HttpRequestKind::WIFI_LOGOUT||r.kind==HttpRequestKind::MOTION_LOGOUT) {
    httpd_resp_set_hdr(
        req, "Set-Cookie",
        "md_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0");
    httpd_resp_set_hdr(
        req, "Set-Cookie",
        "md_device=; HttpOnly; SameSite=Strict; Path=/wifi/session; Max-Age=0");
  }
  return httpd_resp_sendstr(req, resp.message);
}

} // namespace network
} // namespace matdog
