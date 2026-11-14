// Production DalyBms + scheduler/parser. The UART is virtual; no hardware I/O.
#include "../../src/power/DalyBms.h"
#include "../../src/power/MotionPowerPolicy.h"
#include <cassert>
#include <cstdio>
using namespace matdog;
int main(){
 power::DalyBms b;assert(b.begin());auto& u=*HardwareSerial::uart;
 std::vector<uint8_t> response(power::kDalyTelemetryResponseLen);response[0]=0xd2;response[1]=3;response[2]=124;
 auto reg=[&](unsigned i,unsigned v){response[3+2*i]=v>>8;response[4+2*i]=v&255;};
 reg(40,120);reg(49,3);reg(54,1);
 auto crc=[&](){auto c=power::crc16Modbus(response.data(),response.size()-2);response[response.size()-2]=c&255;response.back()=c>>8;};crc();
 for(unsigned cycle=0;cycle<12;++cycle){
  test_ms=2000+cycle*2000;auto t=test_ms;
  // No Motion slack input: every due poll must leave even under 100% Motion load.
  b.update(test_ms,core::OperatingMode::RUN);assert(u.tx.size()==8*(cycle+1)&&test_ms==t&&u.flushes==0);
  test_ms+=140;u.rx.insert(u.rx.end(),response.begin(),response.end());b.update(test_ms,core::OperatingMode::RUN);
  assert(b.sample().valid&&b.sample().sampled_at_ms==test_ms&&b.sample().discharge_mos_on);
 }
 auto s=b.sample();auto stamp=s.sampled_at_ms;
 using P=power::MotionPower;
 assert(power::motionPower(s,stamp+3000,true)==P::OK);
 assert(power::motionPower(s,stamp+3001,true)==P::STALE);
 assert(power::motionPower(s,stamp+5001,true)==P::COMM_LOST);
 assert(power::motionPower(s,stamp,false)==P::STATE_NOT_RUN);
 s.discharge_mos_on=false;assert(power::motionPower(s,stamp,true)==P::MOS_OFF);s.discharge_mos_on=true;
 for(unsigned i=0;i<4;++i){s.alarms[i]=1;assert(power::motionPower(s,stamp+9000,true)==P::ALARM);s.alarms[i]=0;}
 // CRC failure retains the last valid sample (and therefore its alarms).
 reg(58,1);crc();test_ms+=2000;b.update(test_ms,core::OperatingMode::RUN);
 u.rx.insert(u.rx.end(),response.begin(),response.end());test_ms+=140;b.update(test_ms,core::OperatingMode::RUN);
 assert(power::motionPower(b.sample(),test_ms,true)==P::ALARM);
 test_ms+=2000;b.update(test_ms,core::OperatingMode::RUN);response.back()^=1;
 u.rx.insert(u.rx.end(),response.begin(),response.end());test_ms+=140;b.update(test_ms,core::OperatingMode::RUN);
 assert(b.lastCommResult()!=power::DalyCommResult::OK&&power::motionPower(b.sample(),test_ms,true)==P::ALARM);
 // TX backpressure defers without marking a request sent, then resumes.
 auto size=u.tx.size();test_ms+=2000;u.room=7;b.update(test_ms,core::OperatingMode::RUN);assert(u.tx.size()==size);
 u.room=8;b.update(test_ms,core::OperatingMode::RUN);assert(u.tx.size()==size+8&&u.flushes==0);
 s={};assert(power::motionPower(s,test_ms,true)==P::NEVER_SAMPLED);
 s.valid=s.discharge_mos_on=true;s.sampled_at_ms=0xfffffff0u;assert(power::motionPower(s,20,true)==P::OK);
 // D1: transport facts of each completed poll; same requests, same deadline.
 using C=power::DalyCommResult;using D=power::DalyBms;const auto& d=b.telemetryDiag();
 const auto tx0=u.tx.size();uint32_t t=test_ms;unsigned polls=0;crc();
 auto run=[&](uint32_t at){test_ms=at;b.update(test_ms,core::OperatingMode::RUN);};
 auto poll=[&](){t+=2000;run(t);++polls;assert(u.tx.size()==tx0+8*polls);};
 auto reply=[&](uint32_t at,size_t bytes){u.rx.insert(u.rx.end(),response.begin(),response.begin()+bytes);run(at);};
 auto check=[&](C r,unsigned rx,uint32_t first,unsigned stale,uint32_t streak){
  assert(b.lastCommResult()==r&&d.rx_bytes==rx&&d.first_byte_ms==first&&d.stale_bytes==stale&&d.fail_streak==streak);
  assert(b.availability().detected==(r==C::OK?core::DetectedState::ONLINE:core::DetectedState::NO_RESPONSE));};
 reply(t+140,response.size());check(C::OK,129,140,0,0);const auto ok_stamp=b.sample().sampled_at_ms;
 // Silence: nothing is published before the unchanged 750 ms deadline.
 poll();run(t+750);check(C::OK,129,140,0,0);run(t+751);check(C::TIMEOUT,0,D::kNoByte,0,1);
 assert(b.sample().valid&&b.sample().sampled_at_ms==ok_stamp);
 // Partial reply: bytes and first-byte delay, still TIMEOUT.
 poll();reply(t+200,60);check(C::TIMEOUT,0,D::kNoByte,0,1);run(t+751);check(C::TIMEOUT,60,200,0,2);
 // Reply after the deadline: silence for this poll, discarded before the next request.
 poll();run(t+751);check(C::TIMEOUT,0,D::kNoByte,0,3);u.rx.insert(u.rx.end(),response.begin(),response.end());run(t+900);
 // Late but inside the deadline: OK with its delay; the streak resets.
 poll();reply(t+700,response.size());check(C::OK,129,700,129,0);assert(b.sample().sampled_at_ms==t+700);
 // Complete but invalid: all bytes, bad CRC / bad header; the last valid sample stays.
 poll();response.back()^=1;reply(t+140,response.size());check(C::CRC_FAIL,129,140,0,1);response.back()^=1;
 poll();response[0]=0;crc();reply(t+140,response.size());check(C::BAD_HEADER,129,140,0,2);response[0]=0xd2;crc();
 assert(b.sample().sampled_at_ms==ok_stamp+4*2000+560&&power::motionPower(b.sample(),test_ms,true)==P::ALARM);
 std::puts("DALY_ASYNC PASS production scheduler/parser, no flush/wire wait, alarms retained, stale/persistent fail closed, wrap, TX backpressure; D1 silence/partial/late/invalid diagnostics, streak, no extra TX; hardware=false");
}
