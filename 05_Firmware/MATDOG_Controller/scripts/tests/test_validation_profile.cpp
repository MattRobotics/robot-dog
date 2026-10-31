#include "../../src/execution/ValidationProfile.h"
#include <cstdio>
#include "../../src/motion/CertifiedMotionEvidence.h"
#include <cstring>
#include <cstdlib>
using namespace matdog;
int main(){unsigned checks=0;auto check=[&](bool b){++checks;if(!b){printf("PROFILE_FAIL=%u\n",checks);std::exit(1);}};
 execution::ValidationProfile p;check(!p.sealed());check(!p.card("GRANT"));check(!p.card("BEGIN 2 3"));check(!p.card("BEGIN 1 0"));
 check(p.card("BEGIN 1 3"));check(!p.card("JOINT 12 -100000 100000 1000"));check(!p.card("JOINT 0 nan 100000 1000"));
 update::Sha256 hash;auto card=[&](const char* s){check(p.card(s));hash.update(reinterpret_cast<const uint8_t*>(s),strlen(s));const uint8_t lf=10;hash.update(&lf,1);};
 p.clear();card("BEGIN 1 3");card("REVIEW eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");card("RECORD dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");card("ROUTE FIXTURE_LIMIT_COMPATIBLE");char line[220];for(unsigned i=0;i<12;++i){snprintf(line,sizeof(line),"JOINT %u -100000 100000 1000",i);card(line);}
 check(!p.card("JOINT 0 -100000 100000 1000"));check(!p.card("EXEC .05 .2 .0009 40 2000 2500 100 60 50 140 500 0 18"));
 card("EXEC .05 .2 .0009 40 2000 2500 100 60 50 140 500 10 18");card("DRIVE 20 1 .001533980787 .15 cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc");card("CLOCK 9");card("STEP .0015");card("ERROR_BUDGET .000766990394 .0003");card("FIXTURE 0 0 .3 0 0 0 bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
 snprintf(line,sizeof(line),"CORRIDOR 0 251 .004 .001 .001 %s",motion::certified_evidence::kFixtureRouteProof);card(line);
 uint8_t d[32];char sha[65];hash.finish(d);update::toHex(d,sha,65);snprintf(line,sizeof(line),"SEAL %s CONFIRM_INSTALLATION_WORKSPACE_MARGIN_SWEEP_TRACKING_TIMING_OUTPUT",sha);
 check(p.card(line));check(p.sealed());check(!p.card("BEGIN 1 3")||!p.sealed());
 p.clear();check(!p.sealed());check(!p.card("SEAL aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa CONFIRM_INSTALLATION_WORKSPACE_MARGIN_SWEEP_TRACKING_TIMING_OUTPUT"));
 printf("ValidationProfile: %u checks; RAM staging has no grant or NVS operation\n",checks);
}
