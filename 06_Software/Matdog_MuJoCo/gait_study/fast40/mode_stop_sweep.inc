// FAST 40 study (2026-11-13): STOP swept through every WALK mode change, on the
// firmware's own servo_plant model (servo_plant.inc, included before this file).
// For each ordered pair of modes, seed and bus voltage: walk the first mode's top
// preset past its start ramp, select the second, request a STOP after `offset`
// seconds (every SWEEP_STEP s up to SWEEP_SPAN s), wait for its verification.
// Per run: fault (reason, refused check), STOP verified, measured box exit, and
// the phase of the change at the request (mode, target, period, height, sway
// amplitudes): a change still in progress or already settled.
namespace plant {
// dev.7 has no body speed modulation: build it with -DSWEEP_NO_SURGE.
#ifdef SWEEP_NO_SURGE
template<class G> double surgeOf(const G&){return 0;}
#else
template<class G> double surgeOf(const G& g){return g.surgeXM;}
#endif
struct SweepRow{unsigned from,to,seed;double volt,offset;unsigned faults;const char* reason;uint8_t refusedBy;bool verified,inChange;double boxOut,cycles,period,height,swayX,surge,advance;int boxJoint;};
static SweepRow sweepOne(unsigned from,unsigned to,unsigned seed,double volt,double offset){
  SweepRow o{from,to,seed,volt,offset,0,"",0,false,false,0,0,0,0,0,0,0,-1};
  std::mt19937 g(seed);Rig r;
  auto fault=[&]{if(r.m.state()==simple::State::FAULT&&!o.faults){o.faults=1;o.reason=r.m.reason();o.refusedBy=r.m.exitSnapshot().refusedBy;o.cycles=r.m.exitSnapshot().cycles;}return o.faults!=0;};
  if(r.m.stand(.15,r.p.now)){o.faults=1;o.reason="STAND_REFUSED";return o;}
  const Param prm=draw(g,volt);Model pl(r,prm);
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::STANDING&&!fault();++n)pl.tick();
  if(o.faults)return o;
  r.m.selectWalkMode(static_cast<simple::WalkMode>(from));tilt(r,.004,.006);
  if(r.m.walk(2,r.p.now)){o.faults=1;o.reason="WALK_REFUSED";return o;}
  for(unsigned n=0;n<4000&&r.m.state()!=simple::State::WALKING&&!fault();++n)pl.tick();
  // past the start ramp and settled on the first mode's own law
  const auto& wf=simple::walkModeSpec(static_cast<simple::WalkMode>(from));
  for(unsigned n=0;n<unsigned(2.5*wf.periodS/.02)&&!fault();++n)pl.tick();
  if(o.faults)return o;
  if(r.m.selectWalkMode(static_cast<simple::WalkMode>(to))){o.faults=1;o.reason="SELECT_REFUSED";return o;}
  for(unsigned n=0;n<unsigned(std::lround(offset/.02))&&!fault();++n)pl.tick();
  if(o.faults)return o;
  const auto& wt=simple::walkModeSpec(static_cast<simple::WalkMode>(to));const auto& G=P::gait(r.m);
  o.period=r.m.period();o.height=r.m.walkHeightM();o.swayX=G.swayXM;o.surge=surgeOf(G);o.cycles=r.m.walkCycles();o.advance=G.advanceXM;
  o.inChange=!(r.m.walkMode()==static_cast<simple::WalkMode>(to)&&r.m.period()==wt.periodS&&r.m.walkHeightM()==wt.heightM&&G.swayXM==wt.swayXM&&surgeOf(G)==surgeOf(wt)&&G.swayYM==wt.swayYM);
  if(getenv("SWEEP_DUMP")){std::fprintf(stderr,"DUMP %u->%u seed %u off %.2f: mode %d target %d T %.4f h %.4f drive %u/%u | h %.5f lift %.4f duty %.3f adv %.4f advY %.4f yaw %.4f swx %.5f swy %.5f com %.4f start %.2f stop %.2f settle %.2f ramp %.3f/%.3f edge %.4f rev %d cond %.0f\n",
    from,to,seed,offset,int(r.m.walkMode()),int(r.m.walkTarget()),r.m.period(),r.m.walkHeightM(),unsigned(r.p.drive.speed),unsigned(r.p.drive.acc),G.heightM,G.liftM,G.duty,G.advanceXM,G.advanceYM,G.yawRad,G.swayXM,G.swayYM,G.comShiftXM,G.startCycles,G.stopCycles,G.settleCycles,G.swingRamp,G.swingRampEnd,G.edgeTiltRad,int(G.reverse),G.maxCondition);}
  r.m.stop();
  for(unsigned k=0;k<600&&!r.m.stopVerified()&&!fault();++k){pl.tick();for(unsigned i=0;i<12;++i){const double q=r.p.observed[i].q,out=std::max(simple::kWalkBox.lo[i]-q,q-simple::kWalkBox.hi[i])/simple::kTickRad;if(out>o.boxOut){o.boxOut=out;o.boxJoint=int(i);}}}
  if(getenv("SWEEP_DUMP")){const auto& pl2=P::stopPlan(r.m);std::fprintf(stderr,"  plan %.2f s; LH_HIP box [%.1f %.1f] ticks, plan end %.1f, measured %.1f (raw %u), sag %.1f, servo pos %.1f; load %.2f accCap %.0f brake %.0f lat %.0f us deadband %.1f sagScale %.2f\n",
    pl2.duration,simple::kWalkBox.lo[9]/simple::kTickRad,simple::kWalkBox.hi[9]/simple::kTickRad,pl2.end[9]/simple::kTickRad,r.p.observed[9].q/simple::kTickRad,unsigned(r.p.observed[9].raw),Model::sag[9]*prm.sagScale,pl.s[9].pos-2048,prm.load,prm.accCap,prm.brakeCap,prm.latencyUs,prm.deadband,prm.sagScale);}
  o.verified=r.m.stopVerified();return o;
}
static void sweep(){
  const double step=getenv("SWEEP_STEP")?atof(getenv("SWEEP_STEP")):.3,span=getenv("SWEEP_SPAN")?atof(getenv("SWEEP_SPAN")):15;
  const unsigned seeds=getenv("SEEDS")?atoi(getenv("SEEDS")):6;
  // SWEEP_STEADY=1: the same mode selected again (no change), a STOP swept through the steady walk.
  const unsigned all[9][2]={{0,2},{2,0},{1,2},{2,1},{0,1},{1,0},{0,0},{1,1},{2,2}};const bool steady=getenv("SWEEP_STEADY")&&atoi(getenv("SWEEP_STEADY"));
  const unsigned (*pairs)[2]=steady?all+6:all;const unsigned npairs=steady?3:6;
  std::printf("from,to,seed,volt,offset_s,faults,reason,refused,verified,in_change,box_out_ticks,cycles,period_s,height_m,sway_x_m,surge_m,advance_m,box_joint\n");
  for(unsigned ip=0;ip<npairs;++ip)for(unsigned seed=1;seed<=seeds;++seed)for(double volt:{11.1,9.6})for(double off=0;off<=span+1e-9;off+=step){
    const auto o=sweepOne(pairs[ip][0],pairs[ip][1],seed*7919+pairs[ip][0]*31+pairs[ip][1]*7,volt,off);
    std::printf("%s,%s,%u,%.1f,%.2f,%u,%s,%s,%d,%d,%.1f,%.3f,%.4f,%.4f,%.5f,%.5f,%.4f,%d\n",simple::walkModeSpec(static_cast<simple::WalkMode>(pairs[ip][0])).name,simple::walkModeSpec(static_cast<simple::WalkMode>(pairs[ip][1])).name,
      seed,volt,off,o.faults,o.reason,o.faults&&o.refusedBy<10?simple::kSafeChecks[o.refusedBy]:"",o.verified,o.inChange,o.boxOut,o.cycles,o.period,o.height,o.swayX,o.surge,o.advance,o.boxJoint);}
}
}
