// Offline F02 regression: synthetic credentials, production PortalSession/SHA256.
#include <cstdio>
#include <cstring>
#include "network/PortalSecurity.h"
#include "update/Sha256.h"
using matdog::network::PortalSession;
static unsigned checks=0, failures=0;
static void check(bool result, const char *name) {
  ++checks;
  if (!result) { ++failures; std::printf("FAIL %s\n", name); }
}
struct Fixture {
  PortalSession session;
  char token[65]{}, cookie[65]{}, csrf[65]{};
  uint8_t digest[32]{}, random[64]{};
  static constexpr uint32_t peer=0x0204a8c0;
  Fixture() {
    std::memset(token, 'a', 64); std::memset(random, 0x42, 64);
    matdog::update::Sha256 sha;
    sha.update(reinterpret_cast<const uint8_t *>(token),64); sha.finish(digest);
  }
  bool login(uint64_t now) {
    const bool ok=session.login(now,peer,token,digest,random);
    std::strcpy(cookie,session.cookie()); std::strcpy(csrf,session.csrf());
    return ok;
  }
  bool auth(uint64_t now) { return session.authorize(now,peer,cookie,csrf); }
};
int main() {
  constexpr uint64_t cycle=uint64_t{1}<<32, issued=10000;
  Fixture fresh;
  check(!fresh.auth(0),"initialization denies session");
  check(fresh.login(issued),"new login");
  check(fresh.auth(issued),"fresh session");
  check(fresh.auth(issued+299999),"TTL minus one");
  check(!fresh.auth(issued+300000),"TTL boundary");
  check(!fresh.auth(issued+300001),"expired session");
  check(!fresh.auth(issued+cycle),"no resurrection after full cycle");
  check(!fresh.auth(issued+2*cycle+1),"no resurrection after two cycles");
  check(!fresh.auth(issued),"expired session remains invalid even on clock regression");
  Fixture idle;
  check(idle.login(issued),"idle login");
  check(!idle.auth(issued+cycle),"full-cycle expiry without intervening authorize");
  Fixture crossing;
  check(crossing.login(cycle-1000),"login before 32-bit wrap");
  check(crossing.auth(cycle+1000),"valid session crosses 32-bit wrap");
  check(crossing.auth(cycle-1000+299999),"cross-wrap TTL minus one");
  check(!crossing.auth(cycle-1000+300000),"cross-wrap TTL exact");
  Fixture long_uptime;
  check(long_uptime.login(3*cycle+issued),"login after three uptime cycles");
  check(long_uptime.auth(3*cycle+issued+1),"valid login after three cycles");
  check(!long_uptime.auth(4*cycle+issued),"long-uptime stale credential rejected");
  Fixture backwards;
  check(backwards.login(issued),"clock regression setup");
  check(!backwards.auth(issued-1),"clock regression invalidates");
  check(!backwards.auth(issued+1),"regression cannot restore session");
  Fixture reboot;
  check(reboot.login(issued),"reboot setup");
  PortalSession after_boot;
  check(!after_boot.authorize(0,Fixture::peer,reboot.cookie,reboot.csrf),"new boot denies old credentials");
  reboot.session.reset();
  check(!reboot.auth(issued+1),"explicit reset denies old credentials");
  check(!reboot.auth(issued+cycle),"reset survives long uptime");
  Fixture renewal;
  auto authorize_renewal = [&](uint64_t now, uint32_t peer, const char *cookie, const char *csrf) {
    return renewal.session.authorize(now,peer,cookie,csrf);
  };
  check(renewal.login(issued),"renewal setup");
  char old_cookie[65],old_csrf[65];
  std::strcpy(old_cookie,renewal.cookie);std::strcpy(old_csrf,renewal.csrf);
  check(!renewal.auth(issued+300000),"renewal prior session expired");
  std::memset(renewal.random,0x43,64);
  check(renewal.login(issued+cycle),"new login after expiry");
  check(renewal.auth(issued+cycle),"new credentials valid");
  check(!authorize_renewal(issued+cycle,Fixture::peer,old_cookie,old_csrf),"old cookie/CSRF not reusable after renewal");
  check(!authorize_renewal(issued+cycle,Fixture::peer+1,renewal.cookie,renewal.csrf),"peer binding unchanged");
  check(!authorize_renewal(issued+cycle,Fixture::peer,"bad",renewal.csrf),"cookie gate unchanged");
  check(!authorize_renewal(issued+cycle,Fixture::peer,renewal.cookie,"bad"),"CSRF gate unchanged");
  Fixture throttle;
  const uint64_t start=cycle-2000;
  for (unsigned i=0;i<5;++i)
    check(!throttle.session.login(start+i*1000,Fixture::peer,"wrong",throttle.digest,throttle.random),"bad-token throttle setup across wrap");
  check(!throttle.login(start+4000+59999),"throttle 60s minus one");
  check(throttle.login(start+4000+60000),"throttle boundary releases");
  Fixture throttle_idle;
  check(throttle_idle.login(issued),"throttle long-idle setup");
  check(throttle_idle.login(issued+cycle),"no phantom throttle after full uptime cycle");
  Fixture throttle_backwards;
  check(throttle_backwards.login(issued),"throttle clock rollback setup");
  check(!throttle_backwards.login(0),"backward login clock fails closed");
  check(!throttle_backwards.auth(issued+1),"backward login invalidates old session");
  std::printf("AP_SESSION_CLOCK: %u checks, %u failures\n", checks, failures);
  return failures?1:0;
}
