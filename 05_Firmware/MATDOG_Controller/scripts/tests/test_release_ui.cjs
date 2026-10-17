const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const src=path.join(__dirname,'../../src');
const policy=fs.readFileSync(path.join(src,'network/TwinHostPolicy.h'),'utf8');
const csp=[...policy.matchAll(/^\s*"([^"]*)"/gm)].map(x=>x[1]).join('');
const motion=fs.readFileSync(path.join(src,'network/MotionDashboard.h'),'utf8');
const origins=JSON.parse(motion.match(/const authorizedTwinOrigins=(\[[^\n]+\]);/)[1].replace(/'/g,'"'));
assert.deepStrictEqual(csp.match(/frame-src ([^;]+);/)[1].split(' '),origins);
assert(csp.includes("connect-src 'self';")&&!csp.includes('*'));
assert.deepStrictEqual(csp.match(/img-src 'self' ([^;]+);/)[1].split(' '),origins,'Twin probe images only from the same allowlist');
const http=fs.readFileSync(path.join(src,'network/HttpTransport.cpp'),'utf8');
assert(http.includes('"Content-Security-Policy",\n      kDashboardContentSecurityPolicy'));
const portal=fs.readFileSync(path.join(src,'network/PortalPage.h'),'utf8');
const formatter=portal.match(/function formatDiagnosticTime\(value\)\{[^\n]+\}/)[0];
const ctx=vm.createContext({Intl,Date});vm.runInContext(formatter,ctx);
for(const [time,offset,zone]of [['2026-10-16T18:29:00Z','+02:00','CEST'],['2026-10-26T18:29:00Z','+01:00','CET'],['2026-03-29T00:59:59Z','+01:00','CET'],['2026-03-29T01:00:00Z','+02:00','CEST'],['2026-10-25T00:59:59Z','+02:00','CEST'],['2026-10-25T01:00:00Z','+01:00','CET']]){
 const out=ctx.formatDiagnosticTime(time);assert(out.includes(offset+' Europe/Rome '+zone));assert(out.includes(time.replace('Z','.000Z')));
}
assert.equal(ctx.formatDiagnosticTime('invalid'),'N/D');
const fallback=vm.createContext({Date,Intl:{DateTimeFormat(){throw Error('unsupported zone formatter')}}});vm.runInContext(formatter,fallback);assert(fallback.formatDiagnosticTime('2026-10-16T18:29:00Z').includes('Europe/Rome non disponibile'));
const controller=fs.readFileSync(path.join(src,'core/Controller.cpp'),'utf8');
const arm=controller.split('const char* Controller::armOperationalSession(')[1].split('bool Controller::RestoreHook')[0];
assert(!arm.includes('Serial.printf')&&arm.includes('writeAdmissionTrace'));
// ADR-006: no compile-time validation fence; qualification is reported.
assert(!fs.readFileSync(path.join(src,'config/BuildConfig.h'),'utf8').includes('kStandOnlyCandidate'));
assert(fs.readFileSync(path.join(src,'core/Commissioning.cpp'),'utf8').includes('if(operational){unsigned joint=12;'));
assert(fs.readFileSync(path.join(src,'core/MotionService.cpp'),'utf8').includes('\\"hardware_qualified\\":{'));
console.log('RELEASE_UI PASS: exact CSP allowlist/header (frame+img), UTC/Rome both DST transitions, no Print formatter, ADR-006 no STAND-only fence');
