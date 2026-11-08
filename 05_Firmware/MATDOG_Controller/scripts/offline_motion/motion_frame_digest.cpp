// Motion frame digest (motion-perf-next, 2026-11-07), host only. Runs the
// production SimpleMotion on the scripted rig of test_simple_motion.cpp through
// fixed scenarios (HOLD tilted, WALK45/60 + WALK STOP, steady and seeded random
// JOYSTICK, STOP in WALK) and prints, per scenario, the number of transmitted
// SYNC_WRITE frames, an FNV-1a 64-bit digest of every transmitted RAW frame in
// order, the final state and the T1/T2 reuse counters. Two source trees that
// print the same digests transmit bit-identical RAW references on the rig; a
// differing digest is quantified by --dump <file> (one line per frame) and the
// release-level comparison printed by this tool itself (max RAW step between
// trees is NOT computed here: diff the dumps). No hardware, no timing claim.
// Build: run_motion_frame_digest.sh [--dump file].
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "../tests/test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
namespace {
struct Digest { uint64_t h=1469598103934665603ull; void add(uint16_t v){for(unsigned k=0;k<2;++k){h^=(v>>(8*k))&0xff;h*=1099511628211ull;}} };
FILE* dump=nullptr;
void tilt(Rig& r,double roll,double pitch){
  const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
  r.in.quaternion[0]=cr*cp;r.in.quaternion[1]=sr*cp;r.in.quaternion[2]=cr*sp;r.in.quaternion[3]=-sr*sp;}
void report(const char* name,Rig& r,size_t from){
  Digest d;for(size_t k=from;k<r.p.sent.size();++k){for(unsigned i=0;i<12;++i)d.add(r.p.sent[k][i]);
    if(dump){std::fprintf(dump,"%s %zu",name,k-from);for(unsigned i=0;i<12;++i)std::fprintf(dump," %u",r.p.sent[k][i]);std::fprintf(dump,"\n");}}
  const auto& s=r.m.reuseStats();
  std::printf("%-18s frames=%5zu digest=%016llx state=%s/%s reason=%s T1 light/full/rejected=%u/%u/%u shadow=%.3ftick",name,r.p.sent.size()-from,(unsigned long long)d.h,
    simple::name(r.m.state()),r.m.phase(),r.m.reason(),s.light,s.full,s.rejected,s.maxShadowRad/simple::kTickRad);
#ifdef MATDOG_T2_STATS
  const auto& l=r.m.lookaheadStats();
  std::printf(" T2 solved/lerp/resync=%u/%u/%u lerpVsExactNow=%.3e rad (%.3f tick)",l.exact,l.interp,l.resync,l.maxShadowRad,l.maxShadowRad/simple::kTickRad);
#endif
  // Continuity and safety of every transmitted frame of the scenario: largest
  // RAW step between consecutive frames (WALK drive allows 20 ticks per 20 ms)
  // and every frame inside the walk's certified box and the gen-1 limits.
  {unsigned maxStep=0,outside=0;const auto& box=simple::StopReferenceProbe::box(r.m);
   for(size_t k=from;k<r.p.sent.size();++k){for(unsigned i=0;i<12;++i){
     if(k>from)maxStep=std::max(maxStep,unsigned(std::abs(int(r.p.sent[k][i])-int(r.p.sent[k-1][i]))));
     const double q=(int(r.p.sent[k][i])-2048)*simple::kTickRad;if(q<box.lo[i]||q>box.hi[i]||q<r.in.minRad[i]||q>r.in.maxRad[i])++outside;}}
   std::printf(" maxRawStep=%u framesOutsideBox=%u",maxStep,outside);}
  std::printf("\n");
}
struct Lcg { uint64_t s; explicit Lcg(uint64_t seed):s(seed){} double next(){s=s*6364136223846793005ull+1442695040888963407ull;return double(s>>11)/9007199254740992.0;} };
}
int main(int argc,char** argv){
  for(int i=1;i<argc;++i)if(!std::strcmp(argv[i],"--dump")&&i+1<argc)dump=std::fopen(argv[++i],"w");
  constexpr double degree=3.14159265358979323846/180;const double T=simple::kWalkPeriodS;
  const double lim[3]={simple::kJoystickStrideM/T,simple::kJoystickLateralM/T,simple::kJoystickYawRad/T};
  {Rig r;assert(!r.m.stand(.10,r.p.now));assert(r.reachStand());tilt(r,.8*degree,1.2*degree);const size_t from=r.p.sent.size();
    for(unsigned n=0;n<650;++n)r.tick();report("HOLD100_tilted",r,from);}
  for(unsigned stride:{1u,2u}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(stride,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();tilt(r,.3*degree,.8*degree);const size_t from=r.p.sent.size();
    for(unsigned n=0;n<975;++n)r.tick();assert(r.m.state()==simple::State::WALKING);r.m.walkStop();assert(r.reachStand());
    report(stride==1?"WALK45_tilted_stop":"WALK60_tilted_stop",r,from);}
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());const double v[3]={.5*lim[0],.5*lim[1],.5*lim[2]};
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}
    tilt(r,.3*degree,.8*degree);const size_t from=r.p.sent.size();
    for(unsigned n=0;n<975;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}assert(r.m.state()==simple::State::WALKING);
    r.m.walkStop();assert(r.reachStand());report("JOYSTICK_steady",r,from);}
  for(uint64_t seed:{1ull,2ull,3ull}){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());Lcg g(seed);double v[3]={lim[0]*.6,0,0};
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}
    tilt(r,.2*degree,.5*degree);const size_t from=r.p.sent.size();
    // forward only (no reversal): every ~1.2 s a new point inside the ellipsoid
    for(unsigned n=0;n<1800;++n){if(n%60==0){double x[3];double norm=0;for(unsigned j=0;j<3;++j){x[j]=(2*g.next()-1);norm+=x[j]*x[j];}
        const double s=norm>0?.9/std::sqrt(norm)*std::fmin(1.,std::sqrt(norm)):0;for(unsigned j=0;j<3;++j)v[j]=lim[j]*x[j]*s;if(v[0]<=0)v[0]=lim[0]*.05;}
      if(n%12==0){const char* why=r.m.drive(v[0],v[1],v[2],r.p.now);assert(!why||!std::strcmp(why,"WALK_STOPPING"));}
      r.tick();assert(r.m.state()!=simple::State::FAULT);}
    r.m.walkStop();assert(r.reachStand());char name[32];std::snprintf(name,sizeof name,"JOYSTICK_random_%llu",(unsigned long long)seed);report(name,r,from);}
  {Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());assert(!r.m.walk(1,r.p.now));
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n)r.tick();tilt(r,.3*degree,.8*degree);const size_t from=r.p.sent.size();
    for(unsigned n=0;n<300;++n)r.tick();r.m.stop();for(unsigned n=0;n<200&&!r.m.stopVerified();++n)r.tick();assert(r.m.stopVerified());
    report("WALK45_STOP_at300",r,from);}
  // Review of T2 under retargets (2026-11-08): the joystick command changes on
  // EVERY release (bounded random walk inside the ellipsoid, forward only),
  // releases run late by a random 0..19 ms after their grid point (never a
  // whole period), one release in 97 is skipped; then WALK STOP (seed 11) or
  // STOP in the walk (seed 12). Retargets therefore hit every stance/swing
  // transition. The shadow stats compare the transmitted reference with the
  // exact solve under the law of the release that sends it and with the
  // exact solve at the release clock (v1.0.1 sampling).
  // Seeds 11 (WALK STOP) and 12 (STOP); MATDOG_STRESS_SEEDS="a b c" runs more
  // (even seed: STOP, odd: WALK STOP) for a fault-rate comparison between trees.
  std::vector<uint64_t> seeds{11ull,12ull};
  if(const char* env=std::getenv("MATDOG_STRESS_SEEDS")){seeds.clear();std::string t(env);size_t p=0;while(p<t.size()){size_t q=t.find(' ',p);if(q==std::string::npos)q=t.size();if(q>p)seeds.push_back(std::stoull(t.substr(p,q-p)));p=q+1;}}
  for(uint64_t seed:seeds){Rig r;assert(!r.m.stand(.15,r.p.now));assert(r.reachStand());Lcg g(seed);double v[3]={lim[0]*.5,0,0};
    for(unsigned n=0;n<3000&&r.m.state()!=simple::State::WALKING;++n){if(n%12==0)r.m.drive(v[0],v[1],v[2],r.p.now);r.tick();}
    tilt(r,.2*degree,.5*degree);const size_t from=r.p.sent.size();uint64_t grid=r.p.now;unsigned skipped=0;
    auto release=[&](){grid+=simple::kPeriodUs;const uint64_t now=grid+uint64_t(g.next()*19000);
      r.p.now=now;r.in.imuStampUs=now;r.m.inputs(r.in);r.p.publishCommandedPose();test_clock_us=now;r.m.update(now);r.m.updateTelemetry(now);};
    for(unsigned n=0;n<2000;++n){
      // new command every release: random step of up to 10 % of each axis range, kept inside the ellipsoid, forward only
      for(unsigned j=0;j<3;++j)v[j]+=lim[j]*.1*(2*g.next()-1);if(v[0]<lim[0]*.05)v[0]=lim[0]*.05;
      double norm=0;for(unsigned j=0;j<3;++j)norm+=(v[j]/lim[j])*(v[j]/lim[j]);if(norm>.9)for(unsigned j=0;j<3;++j)v[j]*=std::sqrt(.9/norm);
      const char* why=r.m.drive(v[0],v[1],v[2],r.p.now);assert(!why||!std::strcmp(why,"WALK_STOPPING"));
      if(n%97==96){grid+=simple::kPeriodUs;++skipped;}  // a missed release: the next one is two periods after the previous grid point
      release();
      if(r.m.state()==simple::State::FAULT){std::printf("  FAULT at release %u: %s phase=%s cycles=%.4f v=%.5f,%.5f,%.5f imu=%s\n",n,r.m.reason(),r.m.phase(),r.m.walkCycles(),v[0],v[1],v[2],r.m.imuReason());break;}}
    char name[48];std::snprintf(name,sizeof name,"JOY_retarget_jitter_%s_%llu",seed%2==0?"STOP":"WALKSTOP",(unsigned long long)seed);
    if(r.m.state()==simple::State::FAULT){report(name,r,from);continue;}
    if(seed%2==0){r.m.stop();for(unsigned n=0;n<200&&!r.m.stopVerified();++n)release();assert(r.m.stopVerified());report(name,r,from);}
    else{r.m.walkStop();for(unsigned n=0;n<3000&&r.m.state()!=simple::State::STANDING&&r.m.state()!=simple::State::FAULT;++n)release();
      if(r.m.state()!=simple::State::STANDING)std::printf("  FAULT in WALK STOP settle: %s phase=%s cycles=%.4f\n",r.m.reason(),r.m.phase(),r.m.walkCycles());
      std::printf("  (skipped releases=%u, missed counter=%u)\n",skipped,r.m.missed());report(name,r,from);}}
  if(dump)std::fclose(dump);
  std::puts("digest complete (host replay only, no hardware)");
}
