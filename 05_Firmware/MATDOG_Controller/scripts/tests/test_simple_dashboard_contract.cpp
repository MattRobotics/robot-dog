// SIMPLE dashboard -> Controller command contract, offline.
//
// `fixtures` writes real Controller response bodies (MotionService statusJson
// and ackJson on the SimpleMotion rig of test_simple_motion.cpp) for the DOM
// test. `envelopes` takes the request bodies the SHIPPED dashboard JavaScript
// generated, captured verbatim by test_simple_dashboard_dom.cjs, through the
// real parser (MotionJson/MotionContract), the real portal origin and
// session/CSRF checks (PortalSecurity), the real MotionService and the real
// SimpleMotion.
//
// Not executed: esp_http_server, sockets, the mailbox and the STOP lane
// threading. handlerGate()/serve() mirror the POST decision order of
// HttpTransport::handleMotion, serviceRequest and serviceStopLane; that order
// is anchored to the production source text, so a handler change fails here
// until the mirror is reviewed. Scripted observations are software sequencing
// only and qualify no physical STAND, WALK or SAFE_OFF.
// Reuses Rig/ScriptedObservationPort; its main (implicit return) is never called.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
#include "../../src/network/PortalSecurity.h"
#include "../../src/update/Sha256.h"
#include <fstream>
#include <map>
#include <sstream>
using namespace matdog;
namespace {
unsigned checks = 0, failures = 0;
void check(bool ok, const std::string& what) { ++checks; if (!ok) { ++failures; std::printf("FAIL %s\n", what.c_str()); } }

struct Session {
  network::PortalSession portal;
  uint32_t peer = 0x0204a8c0;
  uint64_t now = 10000;
  Session() {
    const std::string token(64, 'a');
    uint8_t digest[32], random[64];
    update::Sha256 sha; sha.update(reinterpret_cast<const uint8_t*>(token.data()), token.size()); sha.finish(digest);
    for (unsigned i = 0; i < 64; ++i) random[i] = static_cast<uint8_t>(i + 1);
    check(portal.login(now, peer, token.c_str(), digest, random), "portal login");
  }
};
struct Http {
  std::string body, host = "192.168.4.1", origin = "http://192.168.4.1", type = "application/json", cookie, csrf;
  uint32_t peer = 0;
};
struct Reply { int status = 0; std::string code, ack; motion::MotionCommand cmd; bool stopLane = false; };

// HttpTransport::handleMotion, POST, up to the mailbox / STOP lane hand-off.
Reply handlerGate(const Http& r) {
  Reply out;
  const bool local = network::portalOrigin(true, r.host.c_str(), r.origin.c_str());
  if (!local || r.body.empty() || r.body.size() > 1024) { out.status = 400; out.code = "ORIGIN_OR_BOUNDED_BODY_REQUIRED"; return out; }
  if (r.type != "application/json") { out.status = 400; out.code = "JSON_REQUIRED"; return out; }
  const char* reason = nullptr;
  if (!core::parseMotionJson(r.body.data(), r.body.size(), &out.cmd, &reason)) { out.status = 400; out.code = reason; return out; }
  if (out.cmd.owner != motion::ClientOwner::DASHBOARD) { out.status = 403; out.code = "DASHBOARD_OWNER_REQUIRED"; return out; }
  out.stopLane = out.cmd.intent == motion::Intent::STOP;
  return out;
}
// serviceRequest MOTION_COMMAND / serviceStopLane: session + CSRF, then MotionService.
Reply serve(const Http& r, Session& session, core::MotionService& service, Rig& rig, uint32_t now_ms) {
  Reply g = handlerGate(r);
  if (g.status) return g;
  if (!session.portal.authorize(session.now, r.peer, r.cookie.c_str(), r.csrf.c_str())) { g.status = 403; g.code = "AUTH_FAILED"; return g; }
  test_clock_us = rig.p.now;
  const auto a = service.command(g.cmd, now_ms);
  char out[512];
  if (!service.ackJson(a, out, sizeof(out))) { g.status = 403; g.code = "ACK_UNAVAILABLE"; return g; }
  g.status = 200; g.code = std::string(motion::toString(a.outcome)) + " " + a.reason; g.ack = out;
  return g;
}
Http request(const std::string& body, Session& s) {
  Http h; h.body = body; h.cookie = s.portal.cookie(); h.csrf = s.portal.csrf(); h.peer = s.peer; return h;
}
std::string replace(std::string s, const std::string& from, const std::string& to) {
  const auto at = s.find(from); check(at != std::string::npos, "mutation anchor " + from);
  return at == std::string::npos ? s : s.replace(at, from.size(), to);
}
std::string statusJson(core::MotionService& service, Rig& rig) {
  char out[core::kMotionStatusJsonBytes]{};
  test_clock_us = rig.p.now;
  check(service.statusJson(out, sizeof(out) - 640, rig.p.now / 1000), "statusJson fits the transport buffer");
  std::string s = out; s.pop_back();
  return s + ",\"admin_session\":true}";  // appended by serviceRequest MOTION_STATUS
}
std::string ackJson(core::MotionService& service, const motion::MotionAck& a) {
  char out[512]{}; check(service.ackJson(a, out, sizeof(out)), "ackJson"); return out;
}
motion::MotionCommand semantic(motion::Intent intent, Rig& rig, double vx = 0) {
  static uint32_t sequence = 0;
  motion::MotionCommand c; c.owner = motion::ClientOwner::DASHBOARD; c.intent = intent; c.sequence = ++sequence;
  c.issued_at_ms = rig.p.now / 1000; c.ttl_ms = 1000; c.vx = vx;
  std::snprintf(c.id, sizeof(c.id), "fixture-%u", static_cast<unsigned>(sequence));
  test_clock_us = rig.p.now;
  return c;
}
std::string quoted(const std::string& s) {
  std::string o = "\"";
  for (char c : s) { if (c == '"' || c == '\\') o += '\\'; o += c; }
  return o + "\"";
}
bool ordered(const std::string& text, std::initializer_list<const char*> parts, const std::string& what) {
  size_t at = 0;
  for (const char* p : parts) {
    const auto found = text.find(p, at);
    if (found == std::string::npos) { check(false, what + ": missing or out of order: " + p); return false; }
    at = found + std::strlen(p);
  }
  check(true, what);
  return true;
}
std::string between(const std::string& text, const std::string& a, const std::string& b) {
  const auto i = text.find(a), j = i == std::string::npos ? i : text.find(b, i);
  check(i != std::string::npos && j != std::string::npos, "source section " + a);
  return i == std::string::npos || j == std::string::npos ? std::string() : text.substr(i, j - i);
}
std::string slurp(const std::string& path) { std::ifstream f(path); std::stringstream s; s << f.rdbuf(); check(f.good() || f.eof(), "read " + path); return s.str(); }

int fixtures(const char* path) {
  std::ofstream out(path);
  out << "{";
  auto put = [&](const char* key, const std::string& value, bool last = false) { out << quoted(key) << ":" << quoted(value) << (last ? "" : ","); };
  {
    Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    for (unsigned i = 0; i < 10; ++i) rig.tick(false);  // idle polling refreshes all 12 feedbacks
    test_clock_us = rig.p.now;
    check(rig.m.fresh(rig.p.now) && !std::strcmp(rig.m.poseRegion(rig.p.now), "PRONE_ENTRY"), "idle PRONE pose classified from fresh feedback");
    put("status_ready", statusJson(service, rig));
    auto prone=semantic(motion::Intent::LIE_DOWN,rig);strcpy(prone.id,"prone-fixture");
    put("ack_prone",ackJson(service,service.command(prone,prone.issued_at_ms)));
    const auto c = semantic(motion::Intent::STAND, rig); auto stand = c; stand.target_height_m = .15;
    const auto a = service.command(stand, c.issued_at_ms);
    check(a.outcome == motion::CommandOutcome::ACCEPTED, "fixture STAND accepted");
    put("ack_stand_accepted", ackJson(service, a));
    rig.tick(); service.update();
    put("status_moving", statusJson(service, rig));
  }
  {
    Rig rig;core::MotionService service;service.begin(&rig.m,nullptr,nullptr);service.setReady(true,"READY");
    check(!rig.m.stand(.05,rig.p.now)&&rig.reachStand(),"PRONE terminal fixture starts in HOLD");
    service.update();
    check(rig.m.state()==simple::State::STANDING&&rig.m.energized(),"fixture STANDING HOLD, torque ON");
    put("status_standing",statusJson(service,rig));  // dev.17: page reload in HOLD
    auto c=semantic(motion::Intent::LIE_DOWN,rig);check(service.command(c,c.issued_at_ms).outcome==motion::CommandOutcome::ACCEPTED,"PRONE accepted from HOLD");
    for(unsigned i=0;i<4100&&rig.m.state()!=simple::State::READY;++i){rig.tick();service.update();}
    for(unsigned i=0;i<15;++i){rig.tick();service.update();}  // the OFF readback pauses FAST reads: one population later
    check(rig.m.proneRest()&&rig.m.state()==simple::State::READY&&rig.m.offVerified()&&!strcmp(rig.m.reason(),"PRONE_REST"),"PRONE ends at rest: torque OFF verified 13/13, no contact claim");
    put("status_prone_rest",statusJson(service,rig));
  }
  {
    Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr);
    rig.in.imuValid = false; rig.in.refusal = "IMU_UNAVAILABLE"; rig.tick(false);
    service.setReady(false, "IMU_UNAVAILABLE");
    check(rig.m.state() == simple::State::STOPPED, "fixture not-ready state");
    put("status_not_ready", statusJson(service, rig));
    auto stand = semantic(motion::Intent::STAND, rig); stand.target_height_m = .15;
    const auto a = service.command(stand, stand.issued_at_ms);
    check(a.outcome == motion::CommandOutcome::REJECTED, "fixture STAND rejected while not ready");
    put("ack_rejected", ackJson(service, a));
  }
  {
    // Pre-flash dynamic joystick: the real status of a joystick walk, of the
    // Controller's own reversal (JOYSTICK_REVERSING: WALK STOP) and of its
    // settling, read by the page's hold through a reversal.
    Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    const double vx = simple::kJoystickStrideM / simple::kJoystickPeriodS;
    check(!rig.m.drive(vx, 0, 0, rig.p.now), "fixture joystick walk accepted");
    for (unsigned i = 0; i < 3000 && rig.m.state() != simple::State::WALKING; ++i) { if (i % 12 == 0) rig.m.drive(vx, 0, 0, rig.p.now); rig.tick(); }
    for (unsigned i = 0; i < 120; ++i) { if (i % 12 == 0) rig.m.drive(vx, 0, 0, rig.p.now); rig.tick(); }
    service.update();
    check(rig.m.state() == simple::State::WALKING && rig.m.joystickActive(), "fixture joystick walking");
    put("status_joystick_walking", statusJson(service, rig));
    auto back = semantic(motion::Intent::BASE_VELOCITY, rig, -vx); std::strcpy(back.gait, "JOYSTICK");
    const auto reversal = service.command(back, back.issued_at_ms);
    check(reversal.outcome == motion::CommandOutcome::ACCEPTED && !std::strcmp(reversal.reason, "JOYSTICK_REVERSING") &&
          !std::strcmp(rig.m.reason(), "JOYSTICK_REVERSING") && !rig.m.joystickActive(), "fixture joystick reversal, ACK JOYSTICK_REVERSING");
    put("ack_joystick_reversing", ackJson(service, reversal));
    rig.tick(); service.update();
    put("status_joystick_reversing", statusJson(service, rig));
    for (unsigned i = 0; i < 4000 && rig.m.state() == simple::State::WALKING; ++i) rig.tick();
    service.update();
    check(rig.m.state() == simple::State::MOVING && !std::strcmp(rig.m.reason(), "WALK_SETTLING") && !rig.m.joystickActive(), "fixture reversal settling");
    put("status_walk_settling", statusJson(service, rig));
  }
  {
    Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    const auto walk = semantic(motion::Intent::BASE_VELOCITY, rig, simple::kStrideM[1] / simple::kWalkPeriodS);
    const auto a = service.command(walk, walk.issued_at_ms);
    check(a.outcome == motion::CommandOutcome::ACCEPTED, "fixture WALK accepted");
    put("ack_walk_accepted", ackJson(service, a));
    for (unsigned i = 0; i < 3000 && rig.m.state() != simple::State::WALKING; ++i) rig.tick();
    service.update();
    check(rig.m.state() == simple::State::WALKING, "fixture WALKING");
    put("status_walking", statusJson(service, rig));
    const auto zero = semantic(motion::Intent::BASE_VELOCITY, rig);
    put("ack_walk_stop", ackJson(service, service.command(zero, zero.issued_at_ms)));
    auto stop = semantic(motion::Intent::STOP, rig);
    const auto s = service.command(stop, stop.issued_at_ms);
    check(s.outcome == motion::CommandOutcome::ACCEPTED && !std::strcmp(s.reason, "STOP_REQUESTED"), "fixture STOP acknowledged");
    put("ack_stop", ackJson(service, s));
    rig.tick(false); service.update();
    check(rig.m.stopping() && !rig.m.offVerified(), "fixture STOP in progress");
    put("status_stopping", statusJson(service, rig));
    for (unsigned i = 0; i < 250 && !rig.m.stopVerified(); ++i) rig.tick();
    service.update();
    check(rig.m.stopVerified() && rig.m.energized() && !rig.m.stopSupportVerified(), std::string("fixture STOP observed, torque ON, support unknown: ")+rig.m.reason());
    put("status_stopped_on", statusJson(service, rig));
    service.safeOff();
    for (unsigned i = 0; i < 20; ++i) rig.tick(false);
    service.update();
    check(rig.m.offVerified(), "fixture explicit SAFE_OFF verified (scripted port)");
    put("status_stopped", statusJson(service, rig), true);
  }
  {
    // dev.17: with STOP latched, a WALK STANDARD preset reaches SimpleMotion
    // (the STAND-after-STOP admission); WALK STOP, joystick, PRONE and any
    // non-preset velocity stay refused with STOP_LATCHED.
    Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    const auto walk = semantic(motion::Intent::BASE_VELOCITY, rig, simple::kStrideM[1] / simple::kWalkPeriodS);
    check(service.command(walk, walk.issued_at_ms).outcome == motion::CommandOutcome::ACCEPTED, "STOP->WALK fixture: WALK75 accepted");
    for (unsigned i = 0; i < 3000 && rig.m.state() != simple::State::WALKING; ++i) rig.tick();
    for (unsigned i = 0; i < 200; ++i) rig.tick();
    auto stop = semantic(motion::Intent::STOP, rig); service.command(stop, stop.issued_at_ms);
    for (unsigned i = 0; i < 300 && !rig.m.stopVerified(); ++i) rig.tick();
    service.update();
    check(rig.m.stopVerified() && rig.m.energized(), "STOP->WALK fixture: STOP verified, torque ON");
    auto refused = [&](motion::MotionCommand c, const char* what) {
      const auto a = service.command(c, c.issued_at_ms);
      check(a.outcome == motion::CommandOutcome::REJECTED && !std::strcmp(a.reason, "STOP_LATCHED"), std::string(what) + " refused while STOP is latched: " + a.reason); };
    refused(semantic(motion::Intent::BASE_VELOCITY, rig), "WALK STOP");
    auto joy = semantic(motion::Intent::BASE_VELOCITY, rig, .003); std::strcpy(joy.gait, "JOYSTICK"); refused(joy, "joystick");
    refused(semantic(motion::Intent::LIE_DOWN, rig), "PRONE");
    refused(semantic(motion::Intent::BASE_VELOCITY, rig, .004), "non-preset velocity");
    const auto again = semantic(motion::Intent::BASE_VELOCITY, rig, simple::kStrideM[2] / simple::kWalkPeriodS);
    const auto a = service.command(again, again.issued_at_ms);
    check(a.outcome == motion::CommandOutcome::ACCEPTED && !std::strcmp(a.reason, "COMMAND_ACCEPTED"), std::string("WALK100 after STOP accepted: ") + a.reason);
    for (unsigned i = 0; i < 4000 && rig.m.state() != simple::State::WALKING; ++i) rig.tick();
    check(rig.m.state() == simple::State::WALKING && std::abs(rig.m.walkVelocityX() - simple::kStrideM[2] / simple::kWalkPeriodS) < 1e-12, "STOP->WALK fixture: WALK100 running");
  }
  {
    // dev.17 pre-flash: dashboard joystick order, through the real JSON parser,
    // the HTTP gate with session/CSRF, MotionService and SimpleMotion. Bodies
    // are shaped like the page's: id "simple-<page session>-<sequence>".
    Session session; Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    check(!rig.m.stand(simple::kWalkHeightM, rig.p.now) && rig.reachStand(), "JOYSTICK order: HOLD at the WALK height");
    const double vx = simple::kJoystickStrideM / simple::kJoystickPeriodS;  // full forward on the pad
    auto now = [&]() { return static_cast<uint32_t>(rig.p.now / 1000); };
    auto joy = [&](const char* page, unsigned seq, uint32_t issued, double v) {
      char b[400];
      std::snprintf(b, sizeof(b), "{\"version\":1,\"id\":\"simple-%s-%u\",\"sequence\":%u,\"issued_at_ms\":%u,\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"BASE_VELOCITY\",\"params\":{\"vx\":%.17g,\"vy\":0,\"yaw_rate\":0,\"gait\":\"JOYSTICK\"}}",
                    page, seq, seq, issued, v);
      return std::string(b); };
    auto send = [&](const std::string& b) { return serve(request(b, session), session, service, rig, now()).code; };
    auto idle = [&](unsigned n) { bool moved = false; for (unsigned k = 0; k < n; ++k) { rig.tick(); moved |= rig.m.state() != simple::State::STANDING; } return moved; };
    auto hold = [&]() { for (unsigned k = 0; k < 3000 && rig.m.state() != simple::State::STANDING; ++k) rig.tick(); return rig.m.state() == simple::State::STANDING; };
    // 1. The move seq 10 is delayed behind the release seq 11 (robot holding):
    //    applied after it, it would start a walk with no finger on the pad.
    const std::string move10 = joy("kq3a1", 10, now(), vx); rig.tick(); rig.tick();
    const std::string neutral11 = joy("kq3a1", 11, now(), 0);
    check(send(neutral11) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: release seq 11 accepted at HOLD");
    rig.tick();
    check(send(move10) == "REJECTED JOYSTICK_SUPERSEDED" && !idle(100), "JOYSTICK order: the late move seq 10 is refused after the release seq 11, the robot keeps holding");
    // 2. Move seq 12 -> release seq 13 -> the move seq 12 delivered again later.
    const std::string move12 = joy("kq3a1", 12, now(), vx);
    check(send(move12) == "ACCEPTED COMMAND_ACCEPTED" && rig.m.joystickActive(), "JOYSTICK order: new move seq 12 after the release accepted");
    rig.tick(); rig.tick();
    const std::string neutral13 = joy("kq3a1", 13, now(), 0);
    check(send(neutral13) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: release seq 13 accepted");
    check(send(move12) == "REJECTED JOYSTICK_SUPERSEDED", "JOYSTICK order: the move seq 12 delivered again after the release is refused");
    // Duplicates and retransmissions: the last command again is its own ack,
    // never executed twice; an older one is refused.
    check(send(neutral13) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: retransmitted release seq 13 answered from the replay slot");
    check(send(joy("kq3a1", 9, now(), vx)) == "REJECTED JOYSTICK_SUPERSEDED", "JOYSTICK order: an older move seq 9 is refused");
    check(hold() && !idle(100), "JOYSTICK order: no walk starts after the release");
    // 3. A new explicit move after the release starts the walk; refreshes keep it.
    unsigned seq = 14; bool walking = false;
    for (unsigned k = 0; k < 3000 && !walking; ++k) {
      if (k % 12 == 0) check(send(joy("kq3a1", seq++, now(), vx)) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: refresh accepted while rising");
      rig.tick(); walking = rig.m.state() == simple::State::WALKING; }
    check(walking && std::abs(rig.m.walkVelocityX() - vx) < 1e-12, "JOYSTICK order: the next press walks (no joystick lock)");
    for (unsigned k = 0; k < 60; ++k) { if (k % 12 == 0) send(joy("kq3a1", seq++, now(), vx)); rig.tick(); }
    // 4. Reload: a new page session restarts at sequence 1 and is accepted; a
    //    late request of the previous page, issued before it, is refused.
    const std::string lateOldPage = joy("kq3a1", seq++, now(), vx);
    rig.tick(); rig.tick(); rig.tick();
    check(send(joy("kr7b2", 1, now(), vx)) == "ACCEPTED COMMAND_ACCEPTED" && rig.m.state() == simple::State::WALKING, "JOYSTICK order: reloaded page, sequence 1, accepted");
    check(send(lateOldPage) == "REJECTED JOYSTICK_SUPERSEDED", "JOYSTICK order: a late request of the previous page is refused after the new page");
    check(send(joy("kr7b2", 1, now(), vx)) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: identical retransmission of the new page answered from the replay slot");
    check(send(joy("kr7b2", 2, now(), vx)) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: the new page goes on");
    // 5. TTL: an expired refresh is refused by the contract and leaves the order unchanged.
    check(send(joy("kr7b2", 3, now() - 1000, vx)) == "REJECTED EXPIRED", "JOYSTICK order: TTL still enforced");
    check(send(joy("kr7b2", 4, now(), vx)) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK order: next fresh refresh accepted after an expired one");
    // 6. Not a joystick update, no order check: a delayed WALK STOP of the
    //    previous page, with a sequence below the joystick ones, still stops.
    check(send(std::string("{\"version\":1,\"id\":\"simple-kq3a1-7\",\"sequence\":7,\"issued_at_ms\":") + std::to_string(now()) +
               ",\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"BASE_VELOCITY\",\"params\":{\"vx\":0,\"vy\":0,\"yaw_rate\":0}}") == "ACCEPTED COMMAND_ACCEPTED" &&
          !std::strcmp(rig.m.reason(), "WALK_STOPPING"), "JOYSTICK order: WALK STOP is never subject to the joystick order");
    // 7. STOP keeps precedence with an old sequence and a long expired timestamp.
    check(send(std::string("{\"version\":1,\"id\":\"simple-kq3a1-1\",\"sequence\":1,\"issued_at_ms\":") + std::to_string(now() - 60000) +
               ",\"ttl_ms\":1,\"owner\":\"dashboard\",\"type\":\"MOTION.STOP\",\"params\":{}}") == "ACCEPTED STOP_REQUESTED", "JOYSTICK order: STOP with an old sequence and timestamp accepted");
    rig.tick();  // the loop services the STOP request
    const auto latched = send(joy("kr7b2", 5, now(), vx));
    check(latched == "REJECTED STOP_IN_PROGRESS" || latched == "REJECTED STOP_LATCHED", "JOYSTICK order: joystick refused while STOP is active: " + latched);
    for (unsigned k = 0; k < 300 && !rig.m.stopVerified(); ++k) rig.tick();
    check(rig.m.stopVerified() && rig.m.energized(), "JOYSTICK order: STOP verified, torque ON");
  }
  {
    // Deadman without the release, and the id format, on a fresh Controller.
    Session session; Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    check(!rig.m.stand(simple::kWalkHeightM, rig.p.now) && rig.reachStand(), "JOYSTICK deadman: HOLD at the WALK height");
    const double vx = simple::kJoystickStrideM / simple::kJoystickPeriodS;
    auto now = [&]() { return static_cast<uint32_t>(rig.p.now / 1000); };
    auto joy = [&](const std::string& id, unsigned seq, uint32_t issued, double v) {
      char b[400];
      std::snprintf(b, sizeof(b), "{\"version\":1,\"id\":\"%s\",\"sequence\":%u,\"issued_at_ms\":%u,\"ttl_ms\":1000,\"owner\":\"dashboard\",\"type\":\"BASE_VELOCITY\",\"params\":{\"vx\":%.17g,\"vy\":0,\"yaw_rate\":0,\"gait\":\"JOYSTICK\"}}",
                    id.c_str(), seq, issued, v);
      return std::string(b); };
    auto send = [&](const std::string& b) { return serve(request(b, session), session, service, rig, now()).code; };
    check(send(joy("joystick", 5, now(), vx)) == "REJECTED JOYSTICK_SEQUENCE_REQUIRED", "JOYSTICK order: id without the page sequence refused");
    check(send(joy("simple-kz9-7", 8, now(), vx)) == "REJECTED JOYSTICK_SEQUENCE_REQUIRED", "JOYSTICK order: id sequence different from the envelope refused");
    std::vector<std::string> sent; bool walking = false; uint64_t lastAcceptedUs = 0;
    for (unsigned k = 0; k < 3000 && (!walking || sent.size() < 4); ++k) {
      if (k % 12 == 0) { const unsigned seq = static_cast<unsigned>(sent.size()) + 1;
        sent.push_back(joy("simple-kz9-" + std::to_string(seq), seq, now(), vx));
        check(send(sent.back()) == "ACCEPTED COMMAND_ACCEPTED", "JOYSTICK deadman: refresh accepted"); lastAcceptedUs = rig.p.now; }
      rig.tick(); walking = rig.m.state() == simple::State::WALKING; }
    check(walking && sent.size() >= 4, "JOYSTICK deadman: walking from refreshes");
    // The page goes silent (release lost). 600 ms later an older refresh arrives
    // inside its TTL: refused, so it cannot extend the 1 s deadman.
    while (rig.p.now < lastAcceptedUs + 600000) rig.tick();
    const auto older = send(sent[sent.size() - 2]);
    check(older == "REJECTED JOYSTICK_SUPERSEDED", "JOYSTICK deadman: an older refresh inside its TTL is refused: " + older + " sent=" + std::to_string(sent.size()));
    for (unsigned k = 0; k < 100 && std::strcmp(rig.m.reason(), "JOYSTICK_TIMEOUT"); ++k) rig.tick();
    const double after = (rig.p.now - lastAcceptedUs) / 1e6;
    check(!std::strcmp(rig.m.reason(), "JOYSTICK_TIMEOUT") && after >= 1.0 && after <= 1.06,
          "JOYSTICK deadman: WALK STOP 1 s after the last accepted refresh, not extended (" + std::to_string(after) + " s)");
    check(rig.reachStand(), "JOYSTICK deadman: the gait law ends in STAND");
  }
  out << "}\n";
  std::printf("SIMPLE_DASHBOARD_FIXTURES = %s (%u checks; real MotionService bodies)\n", failures ? "FAIL" : "PASS", checks);
  return failures ? 1 : 0;
}

void anchors(const std::string& src) {
  const auto http = slurp(src + "/network/HttpTransport.cpp");
  const auto handler = between(http, "esp_err_t HttpTransport::handleMotion(httpd_req_t* req) {", "bool HttpTransport::serviceStopLane");
  ordered(handler, {
    "if(!r.local_authorized || req->content_len<=0 || req->content_len>1024)",
    "return sendJsonError(req,\"400 Bad Request\",\"ORIGIN_OR_BOUNDED_BODY_REQUIRED\");",
    "if(strcmp(type,\"application/json\"))return sendJsonError(req,\"400 Bad Request\",\"JSON_REQUIRED\");",
    "if(!core::parseMotionJson(body,used,&r.motion_command,&reason))",
    "return sendJsonError(req,\"400 Bad Request\",reason);",
    "if(r.motion_command.owner!=motion::ClientOwner::DASHBOARD)",
    "return sendJsonError(req,\"403 Forbidden\",\"DASHBOARD_OWNER_REQUIRED\");",
    "if(r.motion_command.intent==motion::Intent::STOP){",
    "self->stop_request_=r;self->stop_pending_.store(true);",
    "if(!self->stop_applied_.load())return sendJsonError(req,\"403 Forbidden\",\"AUTH_FAILED\",millis()-t0);",
    "if(!self->stop_ack_ok_.load())return sendJsonError(req,\"500 Internal Server Error\",\"ACK_UNAVAILABLE\",millis()-t0);",
    "return httpd_" "resp_send(req,self->stop_ack_,strlen(self->stop_ack_));",
    "if(!self->dispatch(r,&resp,",
    "strcmp(resp.message,\"ACK_UNAVAILABLE\")?\"403 Forbidden\":\"500 Internal Server Error\""}, "handleMotion POST order mirrored by handlerGate; STOP returns before the mailbox dispatch; ACK_UNAVAILABLE is 500, never AUTH_FAILED");
  const auto lane = between(http, "bool HttpTransport::serviceStopLane(", "esp_err_t HttpTransport::handleCalibrationRestore");
  ordered(lane, {"stop_applied_.store(false);", "portal_.authorize(", "service_->motionCommand(req.motion_command,now_ms);", "stop_applied_.store(true);stop_ack_ok_.store("},
          "STOP lane: session+CSRF, then MotionService (owner of the common STOP epoch)");
  ordered(slurp(src + "/core/MotionService.cpp"), {"if(c.intent==motion::Intent::STOP){", "stopEpoch_.fetch_add(1,std::memory_order_release);", "motion_->stop();"},
          "common STOP fence owned by MotionService for HTTP and USB");
  const auto mailbox = between(http, "case HttpRequestKind::MOTION_COMMAND: {", "case HttpRequestKind::CALIBRATION_RESTORE:");
  ordered(mailbox, {"portal_.authorize(", "resp.message=\"AUTH_FAILED\"", "SUPERSEDED_BY_STOP", "if(!mailbox_.awaitingResponse())break;", "service_->motionCommand(req.motion_command,millis());"},
          "mailbox command: session+CSRF, STOP epoch, then MotionService");
  check(http.find("deferred_ = pending_request_.kind == HttpRequestKind::MOTION_COMMAND && stop_pending_.load();") != std::string::npos,
        "a pending STOP is applied before a waiting mailbox command");
  // rc.4: no operator-presence condition anywhere on the Motion path.
  for (const char* file : {"/network/HttpTransport.cpp", "/core/ControllerService.h", "/core/Controller.cpp", "/core/CommandRouter.cpp", "/simple/SimpleMotion.cpp", "/simple/SimpleMotion.h", "/core/MotionService.h"}) {
    const auto text = slurp(src + file);
    check(text.find("operatorHeartbeat") == std::string::npos && text.find("OPERATOR_LINK_LOST") == std::string::npos &&
          text.find("heartbeatUs_") == std::string::npos && text.find("MOTION HEARTBEAT") == std::string::npos &&
          text.find("\"DISCONNECTED\"") == std::string::npos, std::string("no presence deadman in ") + file);
  }
  check(slurp(src + "/core/OperatingMode.h").find("OperatingMode mode_ = OperatingMode::RUN;") != std::string::npos, "boot default is RUN");
  ordered(http, {"resp.motion_session=!readonly&&portal_.authorize(", "\\\"admin_session\\\":true}"},"status poll reports the admin session, no Motion side effect");
  // USB STOP: CommandRouter -> MotionService directly, owner LOCAL; no HTTP, session or mode.
  const auto router = slurp(src + "/core/CommandRouter.cpp");
  ordered(router, {"if(upper==\"@STOP\"||upper==\"@MOTION STOP\"", "c.owner=motion::ClientOwner::LOCAL;", "c.intent=motion::Intent::STOP;",
                   "const auto ack=modules_.service->motionCommand(c,millis());"}, "USB @STOP reaches MotionService without the portal");
  // READY: Motion admission needs fresh servo feedback; the idle polling that
  // keeps it fresh depends on simple_inputs_.ready only (no circular gate).
  const auto integration = slurp(src + "/core/SimpleMotionIntegration.cpp");
  ordered(integration, {"simple_inputs_.ready=build::kServoPowerAvailable&&profile&&mapping&&simple_limits_ready_&&!maintenance&&!authority_.inhibited()&&!wifi_.configBusy();",
                        "const bool feedback=simple_motion_.fresh(now);", "!feedback?\"SERVO_FEEDBACK_STALE\":\"READY\";",
                        "motion_service_.setReady(simple_inputs_.ready&&power&&simple_inputs_.imuValid&&feedback,reason);"}, "READY includes servo freshness, polling does not");
  check(integration.find("simple_inputs_.ready=") != std::string::npos && integration.substr(integration.find("simple_inputs_.ready="), 160).find("feedback") == std::string::npos, "the polling gate never depends on feedback freshness");
  check(slurp(src + "/simple/SimpleMotion.cpp").find("if(!input_.ready&&!engaged())return;") != std::string::npos, "SimpleMotion polling gate unchanged");
  // Digital Twin from this host: heavy assets only while Motion is not engaged.
  ordered(http, {"if(asset->gz_len>kTwinDeferBytes&&(!self||self->twin_deferred_.load())){", "TWIN_DEFERRED_MOTION_ACTIVE"}, "Twin assets deferred only during Motion");
  check(http.find("TWIN_HOST_REQUIRED") == std::string::npos, "no mandatory PC host for the Twin");
  check(slurp(src + "/network/TwinHostPolicy.h").find("frame-src 'self'; img-src 'self';") != std::string::npos, "Twin framed from the same origin only");
  const auto controller = slurp(src + "/core/Controller.cpp");
  check(controller.find("if(c->http_transport_.stopPending())c->http_transport_.serviceStopLane(millis());") != std::string::npos &&
        controller.find("http_transport_.serviceStopLane(now_ms);") != std::string::npos,
        "STOP lane served by the Controller loop and by servo-bus preemption");
  // Pre-flash: the page's joystick link deadline is the Controller's own deadman.
  check(slurp(src + "/network/MotionDashboard.h").find("JOY_DEADLINE_MS=" + std::to_string(simple::kJoystickTimeoutUs / 1000) + ",") != std::string::npos,
        "dashboard JOY_DEADLINE_MS equals kJoystickTimeoutUs");
}

int envelopes(const char* path, const char* src) {
  anchors(src);
  std::ifstream in(path);
  std::string line, standBody, stopBody, walkBody;
  std::map<std::string, std::string> dynamic;  // the page's dynamic joystick press, replayed in order below
  unsigned seen = 0, previousSequence = 0;
  while (std::getline(in, line)) {
    const auto tab = line.find('\t'); if (tab == std::string::npos) continue;
    const std::string name = line.substr(0, tab), body = line.substr(tab + 1);
    ++seen;
    Session session; Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    const auto gate = handlerGate(request(body, session));
    check(gate.status == 0, name + ": accepted by the handler gate (no 400/403)");
    const auto& c = gate.cmd;
    check(c.owner == motion::ClientOwner::DASHBOARD && c.version == 1, name + ": owner dashboard, version 1");
    check(c.sequence > previousSequence, name + ": sequence increases"); previousSequence = c.sequence;
    const char* why = nullptr;
    const uint32_t now = c.issued_at_ms + 40;  // HTTP latency after the dashboard's uptime estimate
    check(motion::validCommand(c, now, &why), name + ": validCommand " + (why ? why : ""));
    if (name.rfind("joystick_dynamic_", 0) == 0) {
      const double T = simple::kJoystickPeriodS, nx = c.vx * T / simple::kJoystickStrideM, ny = c.vy * T / simple::kJoystickLateralM, nw = c.yaw_rate * T / simple::kJoystickYawRad;
      check(c.intent == motion::Intent::BASE_VELOCITY && !std::strcmp(c.gait, "JOYSTICK") && c.ttl_ms == 1000 && !gate.stopLane && nx*nx + ny*ny + nw*nw <= 1 + 1e-9,
            name + ": BASE_VELOCITY gait JOYSTICK inside the Controller ellipsoid, mailbox");
      dynamic[name] = body;
    } else if (name == "stand50" || name == "stand150") {
      const double h = name == "stand50" ? .05 : .15;
      check(c.intent == motion::Intent::STAND && c.target_height_m == h && c.ttl_ms == 5000 && !gate.stopLane, name + ": POSTURE.STAND, exact height, mailbox");
      if (name == "stand150") standBody = body;
      const auto r = serve(request(body, session), session, service, rig, now);
      check(r.status == 200 && r.code == "ACCEPTED COMMAND_ACCEPTED", name + ": HTTP 200 ACCEPTED (" + r.code + ")");
      check(rig.m.state() == simple::State::MOVING && !std::strcmp(rig.m.phase(), "ACQUIRE"), name + ": SimpleMotion MOVING/ACQUIRE");
      check(rig.reachStand() && rig.m.requestedHeightM() == h, name + ": scripted STAND reached at the requested height (software only)");
      Rig late; core::MotionService lateService; lateService.begin(&late.m, nullptr, nullptr); lateService.setReady(true, "READY");
      check(serve(request(body, session), session, lateService, late, c.issued_at_ms + c.ttl_ms).code == "REJECTED EXPIRED", name + ": TTL still enforced");
    } else if (name == "prone") {
      const auto frames=rig.p.frames,off=rig.p.offCalls;
      const auto r=serve(request(body,session),session,service,rig,now);
      check(c.intent==motion::Intent::LIE_DOWN&&!gate.stopLane&&r.code=="REJECTED PRONE_REQUIRES_STAND_HOLD","PRONE explicit refusal through real parser/service");
      check(rig.p.frames==frames&&rig.p.offCalls==off,"PRONE refusal does not move or STOP");
    } else if (name.rfind("walk", 0) == 0 && name != "walk_stop") {
      const unsigned i = name == "walk50" ? 0 : name == "walk75" ? 1 : 2;
      check(c.intent == motion::Intent::BASE_VELOCITY && c.vx == simple::kStrideM[i] / simple::kWalkPeriodS && c.vy == 0 && c.yaw_rate == 0 && c.ttl_ms == 1000,
            name + ": BASE_VELOCITY equals STANDARD preset " + std::to_string(i));
      if (name == "walk75") walkBody = body;
      const auto r = serve(request(body, session), session, service, rig, now);
      check(r.status == 200 && r.code == "ACCEPTED COMMAND_ACCEPTED", name + ": HTTP 200 ACCEPTED (" + r.code + ")");
      for (unsigned k = 0; k < 3000 && rig.m.state() != simple::State::WALKING; ++k) rig.tick();
      check(rig.m.state() == simple::State::WALKING, name + ": scripted WALK STANDARD reached (software only)");
      Rig late; core::MotionService lateService; lateService.begin(&late.m, nullptr, nullptr); lateService.setReady(true, "READY");
      check(serve(request(body, session), session, lateService, late, c.issued_at_ms + 1000).code == "REJECTED EXPIRED", name + ": 1 s TTL enforced");
    } else if (name == "walk_stop") {
      check(c.intent == motion::Intent::BASE_VELOCITY && c.vx == 0 && c.vy == 0 && c.yaw_rate == 0 && !gate.stopLane, name + ": zero BASE_VELOCITY");
      check(!rig.m.walk(1, rig.p.now), name + ": test walk started");
      for (unsigned k = 0; k < 3000 && rig.m.state() != simple::State::WALKING; ++k) rig.tick();
      for (unsigned k = 0; k < 100; ++k) rig.tick();
      const auto r = serve(request(body, session), session, service, rig, now);
      check(r.status == 200 && r.code == "ACCEPTED COMMAND_ACCEPTED" && !std::strcmp(rig.m.reason(), "WALK_STOPPING"), name + ": accepted, coordinated WALK_STOPPING (" + r.code + ")");
      check(rig.reachStand(), name + ": scripted gait ends in STAND (software only)");
    } else if (name == "joystick_forward" || name == "joystick_backward" || name == "joystick_lateral" || name == "joystick_turn") {
      const double T = simple::kJoystickPeriodS, nx = c.vx * T / simple::kJoystickStrideM, ny = c.vy * T / simple::kJoystickLateralM, nw = c.yaw_rate * T / simple::kJoystickYawRad;
      const bool shape = name == "joystick_forward" ? c.vx == simple::kJoystickStrideM / T && c.vy == 0 && c.yaw_rate == 0 :
                         name == "joystick_backward" ? c.vx < 0 && c.vy == 0 && c.yaw_rate == 0 :
                         name == "joystick_lateral" ? c.vx == 0 && c.vy > 0 && c.yaw_rate == 0 : c.vx == 0 && c.vy == 0 && c.yaw_rate > 0;
      check(c.intent == motion::Intent::BASE_VELOCITY && !std::strcmp(c.gait, "JOYSTICK") && c.ttl_ms == 1000 && !gate.stopLane && shape && nx*nx + ny*ny + nw*nw <= 1 + 1e-9,
            name + ": BASE_VELOCITY gait JOYSTICK inside the Controller ellipsoid, mailbox");
      const auto r = serve(request(body, session), session, service, rig, now);
      check(r.status == 200 && r.code == "ACCEPTED COMMAND_ACCEPTED" && rig.m.joystickActive(), name + ": HTTP 200 ACCEPTED, joystick intent held (" + r.code + ")");
      // The page refreshes every 240 ms; the same semantic command stands in for it here.
      for (unsigned k = 0; k < 3000 && rig.m.state() != simple::State::WALKING; ++k) { if (k % 12 == 0) rig.m.drive(c.vx, c.vy, c.yaw_rate, rig.p.now); rig.tick(); }
      check(rig.m.state() == simple::State::WALKING && std::abs(rig.m.walkVelocityX() - c.vx) < 1e-12 && std::abs(rig.m.walkVelocityY() - c.vy) < 1e-12 &&
            std::abs(rig.m.walkYawRate() - c.yaw_rate) < 1e-12, name + ": scripted joystick walk at the commanded velocity (software only)");
      for (unsigned k = 0; k < 100 && std::strcmp(rig.m.reason(), "JOYSTICK_TIMEOUT"); ++k) rig.tick();
      check(!std::strcmp(rig.m.reason(), "JOYSTICK_TIMEOUT") && rig.reachStand(), name + ": silent page -> Controller WALK STOP within 1 s, STAND (software only)");
      Rig late; core::MotionService lateService; lateService.begin(&late.m, nullptr, nullptr); lateService.setReady(true, "READY");
      check(serve(request(body, session), session, lateService, late, c.issued_at_ms + 1000).code == "REJECTED EXPIRED", name + ": 1 s TTL enforced");
    } else if (name == "joystick_release") {
      check(c.intent == motion::Intent::BASE_VELOCITY && c.vx == 0 && c.vy == 0 && c.yaw_rate == 0 && !std::strcmp(c.gait, "JOYSTICK") && !gate.stopLane, name + ": zero BASE_VELOCITY");
      check(!rig.m.drive(simple::kJoystickStrideM / simple::kJoystickPeriodS, 0, 0, rig.p.now), name + ": test joystick walk started");
      for (unsigned k = 0; k < 3000 && rig.m.state() != simple::State::WALKING; ++k) { if (k % 12 == 0) rig.m.drive(simple::kJoystickStrideM / simple::kJoystickPeriodS, 0, 0, rig.p.now); rig.tick(); }
      const auto r = serve(request(body, session), session, service, rig, now);
      check(r.status == 200 && r.code == "ACCEPTED COMMAND_ACCEPTED" && !std::strcmp(rig.m.reason(), "WALK_STOPPING"), name + ": accepted, coordinated WALK_STOPPING (" + r.code + ")");
      check(rig.reachStand(), name + ": scripted gait ends in STAND (software only)");
    } else if (name.rfind("stop", 0) == 0) {
      check(c.intent == motion::Intent::STOP && gate.stopLane, name + ": MOTION.STOP takes the STOP lane");
      stopBody = body;
      check(!rig.m.stand(.15, rig.p.now) && rig.reachStand(), name + ": test STAND reached");
      service.setReady(false, "IMU_UNAVAILABLE");  // STOP never depends on readiness
      const auto off = rig.p.offCalls;
      const auto r = serve(request(body, session), session, service, rig, c.issued_at_ms + 60000);  // stale clock, expired TTL
      check(r.status == 200 && r.code == "ACCEPTED STOP_REQUESTED", name + ": HTTP 200 STOP_REQUESTED with stale TTL and no readiness (" + r.code + ")");
      rig.tick(false);
      check(rig.m.stopInProgress() && rig.p.offCalls == off && rig.m.energized(), name + ": torque retained; ACK is not verification");
      const auto enables=rig.p.enables;
      for (unsigned k = 0; k < 250 && !rig.m.stopVerified(); ++k) rig.tick();
      check(rig.m.stopVerified() && rig.m.state() == simple::State::STOPPED && rig.m.energized() &&
            !rig.m.stopSupportVerified() && rig.p.enables==enables && rig.p.offCalls==off,
            name + ": feedback-verified motion arrest, existing torque ON, support unknown");
    } else check(false, "unknown envelope case " + name);
  }
  check(seen == 21, "twenty-one dashboard envelopes captured (" + std::to_string(seen) + ")");
  // Pre-flash dynamic joystick, end to end (dev.17 HW: a sideways move of the
  // held stick ended the walk). The page's own envelopes of one press, through
  // the handler gate, session, parser, MotionService and SimpleMotion, each
  // refreshed every 240 ms like the page: forward, forward-right, right a hair
  // behind centre, forward-left never stop the walk; a full pull back reverses
  // through one WALK STOP (the page silent meanwhile), the held intent starts
  // the backward walk at STANDING, the release ends it. Software only.
  {
    const char* names[] = {"joystick_dynamic_forward", "joystick_dynamic_diagonal", "joystick_dynamic_lateral", "joystick_dynamic_left",
                           "joystick_dynamic_back", "joystick_dynamic_reverse", "joystick_dynamic_release"};
    for (const char* n : names) check(dynamic.count(n) == 1, std::string(n) + " envelope present");
    Session session; Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    check(!rig.m.stand(.15, rig.p.now) && rig.reachStand(), "dynamic joystick: STAND 150 HOLD");
    unsigned seq = 1000, stops = 0; bool stopping = false;
    auto observe = [&] {  // a WALK STOP began: walking, the joystick walk ended
      const bool now = rig.m.state() == simple::State::WALKING && !rig.m.joystickActive();
      stops += now && !stopping; stopping = now;
    };
    // A later refresh of the same page: next page sequence (id suffix too), Controller time.
    auto post = [&](const std::string& name) {
      const auto& body = dynamic[name]; motion::MotionCommand parsed; const char* why = nullptr;
      check(core::parseMotionJson(body.data(), body.size(), &parsed, &why), name + ": envelope parses");
      const std::string id = parsed.id; ++seq;
      auto fresh = replace(body, "\"id\":\"" + id + "\"", "\"id\":\"" + id.substr(0, id.rfind('-') + 1) + std::to_string(seq) + "\"");
      fresh = replace(fresh, "\"sequence\":" + std::to_string(parsed.sequence), "\"sequence\":" + std::to_string(seq));
      fresh = replace(fresh, "\"issued_at_ms\":" + std::to_string(parsed.issued_at_ms), "\"issued_at_ms\":" + std::to_string(rig.p.now / 1000));
      const auto code = serve(request(fresh, session), session, service, rig, rig.p.now / 1000).code; observe(); return code;
    };
    auto step = [&](const std::string& what) {
      rig.tick(); service.update(); observe();
      check(rig.m.state() != simple::State::FAULT && !rig.m.stopActive(), what + ": no fault, no STOP");
    };
    auto hold = [&](const std::string& name, double cycles) {  // refreshed every 12 releases (240 ms)
      const double end = rig.m.walkCycles() + cycles;
      for (unsigned k = 0; k < 20000 && !(rig.m.state() == simple::State::WALKING && rig.m.walkCycles() >= end); ++k) {
        if (k % 12 == 0) check(post(name) == "ACCEPTED COMMAND_ACCEPTED", name + ": refresh accepted");
        step(name);
      }
      check(rig.m.state() == simple::State::WALKING && rig.m.joystickActive() && !std::strcmp(rig.m.reason(), "WALK_JOYSTICK"), name + ": joystick walk goes on");
    };
    hold("joystick_dynamic_forward", 1.5);
    check(rig.m.walkVelocityX() > 0 && rig.m.walkVelocityY() == 0, "dynamic: forward");
    // Intent -> reference: the next release moves the lateral reference; full
    // scale takes kJoystickRampCycles cycles (rate limit, envelope gate).
    const double vy0 = rig.m.walkVelocityY(), c0 = rig.m.walkCycles();
    check(post("joystick_dynamic_diagonal") == "ACCEPTED COMMAND_ACCEPTED", "dynamic: forward-right accepted");
    step("joystick_dynamic_diagonal");
    const double firstMove = std::abs(rig.m.walkVelocityY() - vy0);
    hold("joystick_dynamic_diagonal", 2.5);
    check(firstMove > 0 && rig.m.walkVelocityX() > 0 && rig.m.walkVelocityY() < 0, "dynamic: forward-right reached, reference moved at the next release");
    hold("joystick_dynamic_lateral", 2.5);
    check(rig.m.walkVelocityX() == 0 && rig.m.walkVelocityY() < 0, "dynamic: right, fore/aft at zero, no reversal");
    hold("joystick_dynamic_left", 3);
    check(rig.m.walkVelocityX() > 0 && rig.m.walkVelocityY() > 0 && stops == 0, "dynamic: forward-left; no WALK STOP in four direction changes");
    std::printf("dynamic joystick: intent->reference at the next release (%.3f mm/s of lateral per release), full scale %.1f s (kJoystickRampCycles %.1f x %.1f s); %.2f cycles without a stop\n",
                firstMove * 1000, simple::kJoystickRampCycles * simple::kJoystickPeriodS, simple::kJoystickRampCycles, simple::kJoystickPeriodS, rig.m.walkCycles() - c0);
    check(post("joystick_dynamic_back") == "ACCEPTED JOYSTICK_REVERSING" && !std::strcmp(rig.m.reason(), "JOYSTICK_REVERSING"), "dynamic: full pull back reverses (WALK STOP), the ACK says so");
    bool settled = false;
    for (unsigned k = 0; k < 6000 && rig.m.state() != simple::State::STANDING; ++k) { step("reversal"); settled |= !std::strcmp(rig.m.reason(), "WALK_SETTLING"); }
    check(rig.m.state() == simple::State::STANDING && settled && stops == 1, "dynamic: one WALK STOP, settling, STANDING (page silent)");
    for (unsigned k = 0; k < 3000 && !(rig.m.state() == simple::State::WALKING && rig.m.walkCycles() > 1); ++k) {
      if (k % 12 == 0) check(post("joystick_dynamic_reverse") == "ACCEPTED COMMAND_ACCEPTED", "dynamic: held backward intent accepted at STANDING");
      step("joystick_dynamic_reverse");
    }
    check(rig.m.state() == simple::State::WALKING && rig.m.joystickActive() && rig.m.walkVelocityX() < 0, "dynamic: the held intent walks backward");
    check(post("joystick_dynamic_release") == "ACCEPTED COMMAND_ACCEPTED" && !std::strcmp(rig.m.reason(), "WALK_STOPPING"), "dynamic: release, WALK STOP");
    check(rig.reachStand(), "dynamic: release ends in STAND (software only)");
  }
  check(!standBody.empty() && !stopBody.empty(), "STAND 150 and STOP envelopes present");
  // Reuse the shipped STOP/STAND envelopes for the new WALK recovery path.
  for(unsigned stride:{1u,2u}){
    Session session;Rig rig;core::MotionService service;service.begin(&rig.m,nullptr,nullptr);service.setReady(true,"READY");
    check(!rig.m.walk(stride,rig.p.now),"HTTP recovery: WALK intent");
    for(unsigned n=0;n<3000&&rig.m.state()!=simple::State::WALKING;++n)rig.tick();
    for(unsigned n=0;n<(stride==1?380u:500u);++n)rig.tick();
    const auto enables=rig.p.enables,off=rig.p.offCalls;
    auto freshBody=[&](const std::string& body,unsigned sequence){
      motion::MotionCommand parsed;const char* why=nullptr;
      check(core::parseMotionJson(body.data(),body.size(),&parsed,&why),"recovery envelope parses");
      auto result=replace(body,"\"sequence\":"+std::to_string(parsed.sequence),"\"sequence\":"+std::to_string(sequence));
      return replace(result,"\"issued_at_ms\":"+std::to_string(parsed.issued_at_ms),"\"issued_at_ms\":"+std::to_string(rig.p.now/1000));
    };
    const auto stop=serve(request(freshBody(stopBody,100),session),session,service,rig,rig.p.now/1000);
    check(stop.status==200&&stop.code=="ACCEPTED STOP_REQUESTED","HTTP WALK STOP accepted");
    for(unsigned n=0;n<300&&!rig.m.stopVerified();++n){rig.tick();service.update();}
    check(rig.m.stopVerified()&&rig.m.energized(),"HTTP WALK STOP verified, torque ON");
    const auto status=statusJson(service,rig);
    check(status.find("\"stand_resume_allowed\":true")!=std::string::npos,"HTTP recovery admission in status");
    const auto stand=serve(request(freshBody(standBody,101),session),session,service,rig,rig.p.now/1000);
    check(stand.status==200&&stand.code=="ACCEPTED COMMAND_ACCEPTED","HTTP explicit recovery STAND accepted");
    for(unsigned n=0;n<3000&&rig.m.state()!=simple::State::STANDING;++n){rig.tick();service.update();}
    core::MotionStatusSnapshot terminal;service.update();service.captureStatus(terminal,rig.p.now/1000);
    check(rig.m.state()==simple::State::STANDING&&!std::strcmp(terminal.activeReason,"STAND_REACHED")&&
      !std::strcmp(terminal.activeOutcome,"COMPLETED"),"HTTP recovery actually reaches STAND_REACHED");
    check(rig.p.enables==enables&&rig.p.offCalls==off,"HTTP recovery has no enable/OFF");
  }
  // dev.17: the shipped WALK75 envelope (preset 1) after a verified STOP in WALK
  // and in HOLD: the STAND-after-STOP admission, the resume route, then WALK STANDARD.
  check(!walkBody.empty(), "WALK75 envelope present");
  for(unsigned origin=0;origin<2;++origin){
    Session session;Rig rig;core::MotionService service;service.begin(&rig.m,nullptr,nullptr);service.setReady(true,"READY");
    if(origin==0){check(!rig.m.walk(2,rig.p.now),"HTTP STOP->WALK: WALK100 intent");
      for(unsigned n=0;n<3000&&rig.m.state()!=simple::State::WALKING;++n)rig.tick();for(unsigned n=0;n<300;++n)rig.tick();}
    else check(!rig.m.stand(.15,rig.p.now)&&rig.reachStand(),"HTTP STOP->WALK: STAND 150 HOLD");
    const auto enables=rig.p.enables,off=rig.p.offCalls;
    auto freshBody=[&](const std::string& body,unsigned sequence){
      motion::MotionCommand parsed;const char* why=nullptr;
      check(core::parseMotionJson(body.data(),body.size(),&parsed,&why),"STOP->WALK envelope parses");
      auto result=replace(body,"\"sequence\":"+std::to_string(parsed.sequence),"\"sequence\":"+std::to_string(sequence));
      return replace(result,"\"issued_at_ms\":"+std::to_string(parsed.issued_at_ms),"\"issued_at_ms\":"+std::to_string(rig.p.now/1000));
    };
    const auto stop=serve(request(freshBody(stopBody,200),session),session,service,rig,rig.p.now/1000);
    check(stop.status==200&&stop.code=="ACCEPTED STOP_REQUESTED","HTTP STOP->WALK: STOP accepted");
    for(unsigned n=0;n<300&&!rig.m.stopVerified();++n){rig.tick();service.update();}
    check(rig.m.stopVerified()&&rig.m.energized()&&statusJson(service,rig).find("\"stand_resume_allowed\":true")!=std::string::npos,"HTTP STOP->WALK: STOP verified, resume admitted");
    const auto walk=serve(request(freshBody(walkBody,201),session),session,service,rig,rig.p.now/1000);
    check(walk.status==200&&walk.code=="ACCEPTED COMMAND_ACCEPTED",std::string("HTTP STOP->WALK: WALK75 accepted (")+walk.code+")");
    for(unsigned n=0;n<4000&&rig.m.state()!=simple::State::WALKING;++n){rig.tick();service.update();}
    check(rig.m.state()==simple::State::WALKING&&std::abs(rig.m.walkVelocityX()-simple::kStrideM[1]/simple::kWalkPeriodS)<1e-12&&
          rig.m.requestedHeightM()==simple::kWalkHeightM,"HTTP STOP->WALK: WALK STANDARD 75 at the WALK height (software only)");
    check(rig.p.enables==enables&&rig.p.offCalls==off,"HTTP STOP->WALK: no enable/OFF, no intermediate STAND command");
  }
  // Rejections keep working on the generated envelopes: none reaches the service.
  {
    Session session; Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    const auto deny = [&](Http h, int status, const std::string& code, const std::string& what) {
      const auto r = serve(h, session, service, rig, 0);
      check(r.status == status && r.code == code, what + " -> " + std::to_string(r.status) + " " + r.code);
    };
    deny(request(replace(standBody, "\"owner\":\"dashboard\"", "\"owner\":\"DASHBOARD\""), session), 403, "DASHBOARD_OWNER_REQUIRED", "owner DASHBOARD");
    deny(request(replace(standBody, "\"owner\":\"dashboard\"", "\"owner\":\"jetson\""), session), 403, "DASHBOARD_OWNER_REQUIRED", "owner jetson");
    deny(request(replace(stopBody, "\"owner\":\"dashboard\"", "\"owner\":\"local\""), session), 403, "DASHBOARD_OWNER_REQUIRED", "STOP owner local");
    deny(request(replace(standBody, "\"POSTURE.STAND\"", "\"STAND\""), session), 400, "INVALID_JSON_SCHEMA", "type STAND");
    deny(request(replace(standBody, "\"POSTURE.STAND\"", "\"posture.stand\""), session), 400, "INVALID_JSON_SCHEMA", "type posture.stand");
    deny(request(replace(stopBody, "\"MOTION.STOP\"", "\"STOP\""), session), 400, "INVALID_JSON_SCHEMA", "type STOP");
    auto h = request(standBody, session); h.origin = "http://evil.invalid"; deny(h, 400, "ORIGIN_OR_BOUNDED_BODY_REQUIRED", "foreign Origin");
    h = request(stopBody, session); h.host = "evil.invalid"; deny(h, 400, "ORIGIN_OR_BOUNDED_BODY_REQUIRED", "STOP foreign Host");
    h = request(standBody, session); h.type = "application/json; charset=utf-8"; deny(h, 400, "JSON_REQUIRED", "content type");
    for (const std::string& body : {standBody, stopBody}) {
      h = request(body, session); h.csrf = std::string(64, '0'); deny(h, 403, "AUTH_FAILED", "wrong CSRF");
      h = request(body, session); h.cookie = std::string(64, '0'); deny(h, 403, "AUTH_FAILED", "wrong session cookie");
      h = request(body, session); h.csrf.clear(); h.cookie.clear(); deny(h, 403, "AUTH_FAILED", "no session");
      h = request(body, session); h.peer = 0x0304a8c0; deny(h, 403, "AUTH_FAILED", "session bound to another peer");
    }
    check(rig.m.state() == simple::State::READY && rig.p.frames == 0 && rig.p.enables == 0 && !rig.m.stopping(), "no rejected envelope reached SimpleMotion");
    // Continuous Dashboard use (a poll every 350 ms for 20 min) keeps the
    // admin session: STOP and commands stay available beyond 300 s.
    Session used; bool alive = true;
    for (uint64_t t = used.now; t <= used.now + 1200000; t += 350) alive = alive && used.portal.authorize(t, used.peer, used.portal.cookie(), used.portal.csrf());
    used.now += 1200000;
    const auto longStop = serve(request(stopBody, used), used, service, rig, 0);
    check(alive && longStop.status == 200 && longStop.code == "ACCEPTED STOP_REQUESTED", "STOP after 20 min of continuous use -> " + longStop.code);
    Session expired; expired.now += 300000;
    const auto late = serve(request(stopBody, expired), expired, service, rig, 0);
    check(late.status == 403 && late.code == "AUTH_FAILED", "STOP with an expired session -> 403 AUTH_FAILED");
  }
  // One Motion core for every source: the same envelope semantics from the
  // USB console (LOCAL) or a future Jetson transport (JETSON) drive the same
  // MotionService and SimpleMotion; only the HTTP transport admits 'dashboard'.
  {
    Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    auto jetson = semantic(motion::Intent::BASE_VELOCITY, rig, simple::kStrideM[2] / simple::kWalkPeriodS); jetson.owner = motion::ClientOwner::JETSON;
    check(service.command(jetson, jetson.issued_at_ms).outcome == motion::CommandOutcome::ACCEPTED, "JETSON-owner WALK accepted by the same core");
    for (unsigned k = 0; k < 3000 && rig.m.state() != simple::State::WALKING; ++k) rig.tick();
    check(rig.m.state() == simple::State::WALKING, "JETSON-owner WALK reaches WALKING (software only)");
    auto local = semantic(motion::Intent::STOP, rig); local.owner = motion::ClientOwner::LOCAL; service.setReady(false, "IMU_UNAVAILABLE");
    const auto a = service.command(local, local.issued_at_ms + 60000);
    rig.tick(false);
    check(a.outcome == motion::CommandOutcome::ACCEPTED && !std::strcmp(a.reason, "STOP_REQUESTED") && rig.m.stopping(), "LOCAL (USB) STOP preempts a JETSON WALK, no readiness or TTL needed");
  }
  // rc.3 (f838f677) payloads, transcribed from its MotionDashboard.h lines
  // 41 and 51-55: the deterministic causes of the observed 400/400/403.
  {
    Session session; Rig rig; core::MotionService service; service.begin(&rig.m, nullptr, nullptr); service.setReady(true, "READY");
    const std::string head = "{\"version\":1,\"id\":\"simple-rc3-1\",\"sequence\":1,\"issued_at_ms\":1000,";
    const auto r1 = serve(request(head + "\"ttl_ms\":5000,\"owner\":\"DASHBOARD\",\"type\":\"STAND\",\"params\":{\"target_height_m\":0.15}}", session), session, service, rig, 1040);
    const auto r2 = serve(request(head + "\"ttl_ms\":5000,\"owner\":\"DASHBOARD\",\"type\":\"STOP\",\"params\":{}}", session), session, service, rig, 1040);
    const auto r3 = serve(request(head + "\"ttl_ms\":1000,\"owner\":\"DASHBOARD\",\"type\":\"BASE_VELOCITY\",\"params\":{\"vx\":0.006923076923076922,\"vy\":0,\"yaw_rate\":0}}", session), session, service, rig, 1040);
    check(r1.status == 400 && r1.code == "INVALID_JSON_SCHEMA", "rc.3 STAND -> 400 INVALID_JSON_SCHEMA");
    check(r2.status == 400 && r2.code == "INVALID_JSON_SCHEMA", "rc.3 STOP -> 400 INVALID_JSON_SCHEMA");
    check(r3.status == 403 && r3.code == "DASHBOARD_OWNER_REQUIRED", "rc.3 BASE_VELOCITY -> 403 DASHBOARD_OWNER_REQUIRED");
  }
  std::printf("SIMPLE_DASHBOARD_CONTRACT = %s (%u checks; %u shipped-JS envelopes through real parser, origin, session/CSRF, MotionService, SimpleMotion; "
              "HTTPD/socket/mailbox/STOP-lane threading mirrored and source-anchored, not executed; NO HARDWARE VALIDATION)\n",
              failures ? "FAIL" : "PASS", checks, seen);
  return failures ? 1 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && !std::strcmp(argv[1], "fixtures")) return fixtures(argv[2]);
  if (argc == 4 && !std::strcmp(argv[1], "envelopes")) return envelopes(argv[2], argv[3]);
  std::fprintf(stderr, "usage: %s fixtures OUT.json | envelopes IN.tsv SRC_DIR\n", argv[0]);
  return 2;
}
