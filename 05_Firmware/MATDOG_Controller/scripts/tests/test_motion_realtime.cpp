// Production-linked cadence probe. All rates below are OFFLINE models.
#define MATDOG_OPERATIONAL_NO_MAIN
#include "test_operational_motion.cpp"
void cadence(const char* name,uint32_t loop_us,unsigned reads,bool walk,uint32_t jitter_us=0){
 Operational op(2.,-92.070442,1,walk);auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
 StreamModel m;m.name=name;m.loop_us=loop_us;m.reads_per_tick=reads;m.stop_short_ticks=5;m.jitter_us=jitter_us;
 m.speed_rad_s=300*motion::tolerance::kTickRad;m.acc_rad_s2=20*100*motion::tolerance::kTickRad;
 StreamDriver d(op,m);uint64_t start=d.now_us,entry_end=0,last_tx=0;Dist intervals,regular,age;bool last_final=false;bool ok=true;unsigned previous=0;
 while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&d.now_us-start<600000000ull){
  ok=d.tick();if(!entry_end&&!b.coordinator.stand().entryPending()&&d.frames_seen)entry_end=d.now_us;
  if(d.frames_seen!=previous){if(last_tx){intervals.add((d.now_us-last_tx)/1000.);if(!last_final)regular.add((d.now_us-last_tx)/1000.);}last_final=b.coordinator.stand().pendingFinal();last_tx=d.now_us;previous=d.frames_seen;}
  for(unsigned i=0;i<12;++i)age.add((d.now_us-d.sample_us[i])/1000.);
 }
 bool stood=ok&&!std::strcmp(b.coordinator.state(),"STAND");
 std::printf("CADENCE model=%s loop_ms=%.3f reads=%u stood=%d elapsed_s=%.3f entry_s=%.3f frames=%u gap_ms_p50=%.3f p99=%.3f max=%.3f age_ms_max=%.3f reason=%s\n",name,loop_us/1000.,reads,stood,(d.now_us-start)/1e6,entry_end?(entry_end-start)/1e6:-1.,d.frames_seen,intervals.pct(.5),intervals.pct(.99),intervals.max(),age.max(),b.r.executor.status().reason);
 CHECK(stood==(loop_us<10000&&reads>=3));
 std::printf("MICROSTEP_GAP model=%s p50_ms=%.3f p99_ms=%.3f max_ms=%.3f scheduler_late_ms=%.3f\n",name,regular.pct(.5),regular.pct(.99),regular.max(),b.coordinator.schedulerCounters().max_lateness_us/1000.);
 if(loop_us<10000&&reads>=3)CHECK(regular.max()<=20.+loop_us/1000.+jitter_us/1000.+1e-9);
 if(loop_us<10000&&reads==1)REASON(b.r.executor.status().reason,"TRACKING_LAG_LIMIT");
 if(loop_us>=10000)REASON(b.r.executor.status().reason,"STALE_FEEDBACK");
 if(walk&&stood){
  for(unsigned k=0;k<100&&ok;++k)ok=d.tick();
  op.walk_ok=true;op.refresh();
  const auto ack=walkCommand(op,71,motion::operational_walk::profileVelocity(motion::operational_walk::kStandardProfile,1));std::printf("WALK_ADMISSION reason=%s\n",ack.reason);CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);
  d.m.speed_rad_s=1000*motion::tolerance::kTickRad;d.m.acc_rad_s2=50*100*motion::tolerance::kTickRad;
  const uint64_t w0=d.now_us;Dist gaps;last_tx=0;previous=d.frames_seen;
  while(ok&&b.coordinator.walkCycles()<1.&&d.now_us-w0<16000000ull){ok=d.tick();if(previous!=d.frames_seen){if(last_tx)gaps.add((d.now_us-last_tx)/1000.);last_tx=d.now_us;previous=d.frames_seen;}}
  CHECK(ok&&b.coordinator.walkCycles()>=1.);
  std::printf("WALK_CADENCE model=%s ok=%d cycles=%.3f gap_ms_p50=%.3f p99=%.3f max=%.3f reason=%s\n",name,ok,b.coordinator.walkCycles(),gaps.pct(.5),gaps.pct(.99),gaps.max(),b.r.executor.status().reason);
  CHECK(walkCommand(op,72,0).outcome==motion::CommandOutcome::ACCEPTED);const auto stop_at=d.now_us;
  while(ok&&b.coordinator.walking()&&d.now_us-stop_at<30000000ull)ok=d.tick();
  CHECK(ok&&!b.coordinator.walking());std::printf("WALK_STOP ok=%d duration_s=%.3f reason=%s\n",ok,(d.now_us-stop_at)/1e6,b.r.executor.status().reason);
 }
}
void cadenceGapGuard(){
 Rig r;r.envelope.frame_writes=r.envelope.timed_segments=true;r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;
 CHECK(r.arm());auto f=r.frame(61,0);auto v=r.validation(61);v.final=false;
 CHECK(r.executor.submit(f,v,100,20000,.020,r.facts,r.feedback,100));r.executor.update(r.facts,r.feedback,101);
 r.fresh(121);r.executor.update(r.facts,r.feedback,121);CHECK(r.executor.status().completed_sequence==61);
 r.fresh(222);r.executor.update(r.facts,r.feedback,222);REASON(r.executor.status().reason,"FRAME_CADENCE_GAP");CHECK(r.executor.status().safe_off_required);
}
int main(){cadenceGapGuard();cadence("FAST_4MS_RR",4000,3,true);cadence("FAST_LOADED_6MS_RR",6000,3,false);cadence("FAST_JITTER_RR",4000,3,false,1200);cadence("COMBINED_6MS_RR",6000,1,false);cadence("HISTORIC_25MS_RR",25000,1,false);std::printf("CADENCE_PROBE failures=%d hardware=false\n",failures);return failures?1:0;}
