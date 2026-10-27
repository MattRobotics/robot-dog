#!/usr/bin/env python3
"""Twin asset responses never hold the single httpd task beyond their budget.

Compiles the production sendTwinAsset() and HttpTransport::handleTwin() text
from src/network/HttpTransport.cpp against a host model of one lwIP TCP send
buffer (TCP_SND_BUF 5744, writable above TCP_SNDLOWAT 2873, as select()
reports it in IDF 5.5.5) drained by a client at a chosen rate, on a simulated
clock. Checks: complete and exact 200 for a fast client; a slow or stalled
client is cut at the budget with a truncated body (handler fails, socket
closed), never a complete response; Motion engaging aborts within one slice;
304 and 503 unchanged; every write fits the free buffer (never waits).
"""
import re, subprocess, sys, tempfile
from pathlib import Path
FW = Path(__file__).resolve().parents[2]
src = (FW / "src/network/HttpTransport.cpp").read_text()
helper = src[src.index("// Twin asset response (handleTwin)."):src.index("} // namespace\n", src.index("esp_err_t sendTwinAsset("))]
handler = src[src.index("esp_err_t HttpTransport::handleTwin("):src.index("esp_err_t HttpTransport::handleWifi(")]

fixture = r'''
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include "TwinAssets.h"
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define HTTPD_SOCK_ERR_FAIL -1
#define HTTPD_SOCK_ERR_TIMEOUT -3
struct httpd_req_t { const char* uri; void* user_ctx; };
// ---- simulated lwIP socket + clock -------------------------------------
static int64_t now_us = 0;
static int64_t esp_timer_get_time() { return now_us; }
static const double kSndBuf = 5744, kSndLowat = 2873;
struct Sim { double queued = 0, rate = 0; /* bytes per ms drained by the peer */
  std::string wire; int select_error = 0; int send_error = 0; size_t engage_at = 0; int64_t engage_us = -1;
  timeval sndtimeo{3, 0}; int writes = 0, waited_writes = 0; int64_t max_wait_us = 0; size_t max_write = 0; };
static Sim sim;
static std::atomic<bool>* engage_flag = nullptr;
static void drain(int64_t dt_us) { sim.queued -= sim.rate * dt_us / 1000.0; if (sim.queued < 0) sim.queued = 0; }
static bool writable() { return kSndBuf - sim.queued > kSndLowat; }
static int fake_select(int, fd_set*, fd_set* w, fd_set*, timeval* tv) {
  if (sim.select_error) return -1;
  const int64_t limit = int64_t(tv->tv_sec) * 1000000 + tv->tv_usec;
  if (limit > sim.max_wait_us) sim.max_wait_us = limit;
  if (writable()) return 1;
  int64_t need = limit + 1;
  if (sim.rate > 0) need = int64_t((sim.queued - (kSndBuf - kSndLowat) + 1) / sim.rate * 1000.0) + 1;
  const int64_t dt = need < limit ? need : limit;
  now_us += dt; drain(dt);
  if (sim.engage_us >= 0 && now_us >= sim.engage_us && engage_flag) engage_flag->store(true);
  if (writable()) return 1;
  FD_ZERO(w); return 0;
}
int httpd_req_to_sockfd(httpd_req_t*) { return 7; }
static int fake_getsockopt(int, int, int name, void* v, socklen_t* n) { if (name == SO_SNDTIMEO) { std::memcpy(v, &sim.sndtimeo, sizeof(timeval)); *n = sizeof(timeval); } return 0; }
static int fake_setsockopt(int, int, int name, const void* v, socklen_t) { if (name == SO_SNDTIMEO) std::memcpy(&sim.sndtimeo, v, sizeof(timeval)); return 0; }
int httpd_send(httpd_req_t*, const char* buf, size_t len) {
  if (sim.send_error) return sim.send_error;
  sim.writes++; if (len > sim.max_write) sim.max_write = len;
  if (kSndBuf - sim.queued < len) sim.waited_writes++;   // would have waited in lwIP
  const double room = kSndBuf - sim.queued; const size_t n = room >= len ? len : size_t(room);
  sim.wire.append(buf, n); sim.queued += n; now_us += 20; drain(20);   // ~20 us copy + tcp_output
  if (sim.engage_at && engage_flag && sim.wire.size() >= sim.engage_at) engage_flag->store(true);
  return int(n);
}
// ---- minimal httpd response API (used by 302/304/404/503 paths) ---------
static std::string status = "200 OK", headers, body; static bool resp_sent = false;
void httpd_resp_set_status(httpd_req_t*, const char* s) { status = s; }
void httpd_resp_set_hdr(httpd_req_t*, const char* k, const char* v) { headers += std::string(k) + ": " + v + "\n"; }
esp_err_t httpd_resp_send(httpd_req_t*, const char* b, ssize_t n) { resp_sent = true; if (b) body.assign(b, size_t(n)); return ESP_OK; }
static std::string if_none_match;
esp_err_t httpd_req_get_hdr_value_str(httpd_req_t*, const char* k, char* out, size_t cap) {
  if (std::strcmp(k, "If-None-Match") || if_none_match.empty() || if_none_match.size() >= cap) return -1;
  std::strcpy(out, if_none_match.c_str()); return ESP_OK; }
namespace matdog { namespace network {
struct HttpTransportRequest {};
static bool socketFacts(httpd_req_t*, HttpTransportRequest&) { return true; }
#define select fake_select
#define getsockopt fake_getsockopt
#define setsockopt fake_setsockopt
namespace {
@HELPER@
}
class HttpTransport { public:
  static esp_err_t handleTwin(httpd_req_t*);
  static esp_err_t sendJsonError(httpd_req_t* req, const char* s, const char* code, uint32_t = 0) {
    status = s; body = std::string("{\"error\":\"") + code + "\"}"; return httpd_resp_send(req, body.c_str(), ssize_t(body.size())); }
  std::atomic<bool> twin_deferred_{false};
  static constexpr size_t kTwinDeferBytes = 8192;
};
@HANDLER@
} }
using namespace matdog::network;
static int checks = 0;
#define CHECK(c, m) do { ++checks; if (!(c)) { std::printf("FAIL %s\n", m); return 1; } } while (0)
static const twin_assets::Asset& asset(const char* p) { for (auto& a : twin_assets::kAssets) if (!std::strcmp(a.path, p)) return a; std::abort(); }
struct Run { esp_err_t rc; int64_t elapsed_us; };
static HttpTransport transport;
static Run run(const char* uri, double rate, bool engaged = false, size_t engage_at = 0) {
  sim = Sim{}; sim.rate = rate; sim.engage_at = engage_at; status = "200 OK"; headers.clear(); body.clear(); resp_sent = false;
  transport.twin_deferred_.store(engaged); engage_flag = &transport.twin_deferred_;
  httpd_req_t req{uri, &transport}; const int64_t t0 = now_us;
  const esp_err_t rc = HttpTransport::handleTwin(&req); return {rc, now_us - t0};
}
static std::string expected(const twin_assets::Asset& a) {
  return std::string("HTTP/1.1 200 OK\r\nContent-Type: ") + a.type + "\r\nContent-Length: " + std::to_string(a.gz_len) +
    "\r\nContent-Encoding: gzip\r\nETag: " + a.etag + "\r\nCache-Control: no-cache\r\nAccept-Ranges: none\r\n"
    "X-Content-Type-Options: nosniff\r\n\r\n" + std::string(reinterpret_cast<const char*>(a.gz), a.gz_len);
}
int main() {
  const auto& js = asset("/twin/assets/twin.js"); const auto& mtw = asset("/twin/matdog_collision.mtw");
  const auto& urdf = asset("/twin/matt_robodog_rev00.urdf"); const auto& html = asset("/twin/");
  const int64_t budget = 1000000, slice = 100000, write_cost = 20;
  // A: fast client (8 Mbit/s) gets the exact, complete response well inside the budget.
  for (const auto* a : {&js, &mtw, &urdf, &html}) {
    Run r = run(a->path, 1000);
    CHECK(r.rc == ESP_OK && sim.wire == expected(*a), "fast client: exact complete 200 with Content-Length/ETag/gzip");
    CHECK(r.elapsed_us < budget, "fast client inside budget");
    CHECK(sim.sndtimeo.tv_sec == 3 && sim.sndtimeo.tv_usec == 0, "keep-alive socket gets httpd's 3 s wait back");
    CHECK(sim.waited_writes == 0 && sim.max_write <= 1436, "every write fits the free send buffer (never waits)");
    std::printf("fast %-30s %7zu B in %6.1f ms, %d writes\n", a->path, a->gz_len, r.elapsed_us / 1000.0, sim.writes);
  }
  // Borderline: 1.3 Mbit/s still completes twin.js (142 KB) inside 1 s.
  { Run r = run(js.path, 160); CHECK(r.rc == ESP_OK && sim.wire == expected(js), "1.3 Mbit/s completes twin.js");
    std::printf("160 B/ms twin.js complete in %.1f ms\n", r.elapsed_us / 1000.0); }
  // B: slow (400 kbit/s) and stalled clients are cut at the budget: truncated body, handler fails.
  for (double rate : {50.0, 5.0, 0.0}) {
    Run r = run(js.path, rate);
    CHECK(r.rc == ESP_FAIL, "slow/stalled client: handler fails -> httpd closes the socket");
    CHECK(sim.wire.size() < expected(js).size() && expected(js).compare(0, sim.wire.size(), sim.wire) == 0, "truncated prefix, never complete");
    CHECK(r.elapsed_us <= budget + write_cost, "occupation <= budget + one write");
    CHECK(sim.max_wait_us <= slice, "each wait is one slice at most");
    CHECK(sim.sndtimeo.tv_sec == 0 && sim.sndtimeo.tv_usec == slice, "closing socket keeps the short send timeout");
    CHECK(sim.waited_writes == 0, "no write waits for the peer");
    std::printf("rate %5.1f B/ms: cut after %6.1f ms, %zu of %zu B sent\n", rate, r.elapsed_us / 1000.0, sim.wire.size(), expected(js).size());
  }
  // C: Motion engaging mid-transfer aborts at the next write (fast) or slice (stalled).
  { Run r = run(js.path, 1000, false, 20000);
    CHECK(r.rc == ESP_FAIL && sim.wire.size() < 20000 + 1436 + 1, "engagement aborts at the next write");
    std::printf("engaged after 20000 B: aborted at %zu B, %.2f ms\n", sim.wire.size(), r.elapsed_us / 1000.0); }
  { sim = Sim{}; sim.engage_us = now_us + 350000;   // Motion engages 350 ms into a stalled transfer
    const int64_t at = sim.engage_us; transport.twin_deferred_.store(false); engage_flag = &transport.twin_deferred_;
    httpd_req_t req{mtw.path, &transport}; const esp_err_t rc = HttpTransport::handleTwin(&req);
    CHECK(rc == ESP_FAIL && now_us >= at && now_us - at <= slice + write_cost, "stalled + engagement: abort within one slice");
    std::printf("stalled, engaged at +350 ms: aborted %.1f ms later\n", (now_us - at) / 1000.0); }
  // D: heavy asset requested during Motion: 503 Retry-After, no body; small assets still served.
  { Run r = run(js.path, 1000, true);
    CHECK(r.rc == ESP_OK && status == "503 Service Unavailable" && headers.find("Retry-After: 5") != std::string::npos &&
          body.find("TWIN_DEFERRED_MOTION_ACTIVE") != std::string::npos && sim.wire.empty(), "heavy during Motion: 503, no transfer"); }
  { Run r = run(urdf.path, 1000, true); CHECK(r.rc == ESP_OK && sim.wire == expected(urdf), "small asset served during Motion"); }
  // E: cached Twin (ETag) revalidates with 304 even during Motion; a stale tag gets the body.
  if_none_match = js.etag;
  { Run r = run(js.path, 1000, true); CHECK(r.rc == ESP_OK && status == "304 Not Modified" && headers.find(js.etag) != std::string::npos && sim.wire.empty(), "304 during Motion"); }
  if_none_match = "\"0000\"";
  { Run r = run(mtw.path, 1000); CHECK(r.rc == ESP_OK && sim.wire == expected(mtw), "stale ETag gets the full body"); }
  if_none_match.clear();
  // Socket errors: fail, never success.
  sim = Sim{};
  { transport.twin_deferred_.store(false); httpd_req_t req{js.path, &transport}; sim.rate = 1000; sim.send_error = HTTPD_SOCK_ERR_FAIL;
    CHECK(HttpTransport::handleTwin(&req) == ESP_FAIL && sim.wire.empty(), "send error fails the handler"); }
  sim = Sim{};
  { httpd_req_t req{js.path, &transport}; sim.rate = 1000; sim.send_error = -0x6880;  /* TLS WANT_WRITE */
    CHECK(HttpTransport::handleTwin(&req) == ESP_FAIL, "TLS want-write fails the handler"); }
  sim = Sim{};
  { httpd_req_t req{js.path, &transport}; sim.rate = 1000; sim.select_error = 1;
    CHECK(HttpTransport::handleTwin(&req) == ESP_FAIL && sim.wire.empty(), "select error fails the handler"); }
  // Routing unchanged.
  { run("/twin", 1000); CHECK(status == "302 Found" && headers.find("Location: /twin/") != std::string::npos, "/twin redirects"); }
  { Run r = run("/twin/?feed=parent", 1000); CHECK(r.rc == ESP_OK && sim.wire == expected(html), "query string ignored for the page"); }
  { run("/twin/nope", 1000); CHECK(status == "404 Not Found" && sim.wire.empty(), "unknown asset 404"); }
  // The dashboard uses this same production helper for headers AND its body.
  { const std::string page(58000, 'x'); const char* h="HTTP/1.1 200 OK\r\nContent-Length: 58000\r\n\r\n";
    httpd_req_t req{"/", &transport};
    sim=Sim{}; sim.rate=1000;
    CHECK(sendBoundedResponse(&req,h,strlen(h),page.data(),page.size(),nullptr)==ESP_OK && sim.wire==std::string(h)+page,"dashboard exact response");
    sim=Sim{}; sim.rate=0; sim.queued=kSndBuf; auto t=now_us;
    CHECK(sendBoundedResponse(&req,h,strlen(h),page.data(),page.size(),nullptr)==ESP_FAIL && now_us-t<=budget+write_cost && sim.wire.empty(),"stalled before headers bounded");
    sim=Sim{}; sim.rate=5; t=now_us;
    CHECK(sendBoundedResponse(&req,h,strlen(h),page.data(),page.size(),nullptr)==ESP_FAIL && now_us-t<=budget+write_cost && sim.wire.size()<strlen(h)+page.size(),"slow dashboard body bounded");
  }
  std::printf("TWIN_BOUNDED_SEND: %d checks PASS\n", checks);
  return 0;
}
'''
fixture = fixture.replace("@HELPER@", helper).replace("@HANDLER@", handler)
with tempfile.TemporaryDirectory() as d:
    cpp, exe = Path(d) / "t.cpp", Path(d) / "t"
    cpp.write_text(fixture)
    subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                    "-I", str(FW / "src/network"), str(cpp), "-o", str(exe)], check=True)
    sys.exit(subprocess.run([str(exe)]).returncode)
