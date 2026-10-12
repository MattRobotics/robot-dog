'use strict';
// Fresh headless Chromium against owned localhost offline services only.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {chromium}=require('playwright-core');
(async()=>{
 const out=process.env.MATDOG_BROWSER_OUTPUT;assert(out);fs.mkdirSync(out,{recursive:true});
 const trace=fs.readFileSync(process.env.MATDOG_PAIRED_TRACE,'utf8').trim().split('\n').map(JSON.parse);
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1440,height:1000}}),page=await context.newPage(),errors=[],writes=[];page.on('pageerror',e=>errors.push(e.message));page.on('request',r=>{if(r.method()!=='GET')writes.push(r.url())});
 try{
  await page.goto('http://127.0.0.1:5195',{waitUntil:'networkidle'});await page.locator('nav button[data-page="motion-control"]').click();await page.waitForFunction(()=>document.querySelectorAll('[data-action]').length===48);
  assert.equal(await page.locator('nav button[data-page]').count(),7);assert.equal(await page.locator('[data-action]:disabled').count(),46);assert.equal(await page.locator('#motion-joystick').getAttribute('aria-disabled'),'true');
  for(const id of ['motion-stand','motion-lie','motion-gait','motion-speed','motion-yaw','motion-body'])assert(await page.locator('#'+id).isDisabled());assert(!(await page.locator('#motion-stop-button').isDisabled()));
  await page.screenshot({path:path.join(out,'dashboard_desktop.png'),fullPage:true});await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));await page.screenshot({path:path.join(out,'dashboard_mobile.png'),fullPage:true});assert.equal(writes.length,0);
  const denied=await context.request.post('http://127.0.0.1:5195/api/motion/command',{data:{version:1,id:'offline-denied',sequence:1,type:'POSTURE.STAND',issued_at_ms:0,ttl_ms:250,target_height_m:.15}});assert.equal(denied.status(),409);assert.equal((await denied.json()).reason,'OFFLINE_DISPLAY_ONLY');
  await page.setViewportSize({width:1440,height:1000});await page.goto('http://127.0.0.1:5193/?motion=trace',{waitUntil:'networkidle'});await page.evaluate(()=>window.matdogViewer.ready);await page.waitForFunction(()=>window.matdogViewer.snapshot().motion?.owner==='offline_trace');
  let first=await page.evaluate(()=>({state:window.matdogViewer.snapshot(),render:window.matdogViewer.forceRender()}));assert.equal(first.state.collisionMeshCount,17);assert.equal(Object.keys(first.state.joints).length,12);assert(first.render.triangles>0);assert.match(first.state.motion.reason,/SYNTHETIC_GEOMETRY_ONLY/);
  await page.locator('#btn-motion-pause').click();assert.equal(await page.locator('#twin-fresh').textContent(),'STALE');assert.equal((await page.evaluate(()=>window.matdogViewer.snapshot())).targetVisible,false);
  await page.goto('http://127.0.0.1:5193/',{waitUntil:'networkidle'});await page.evaluate(()=>window.matdogViewer.ready);
  // Consume selected source frames directly to cover the full recording quickly.
  // Values/stamps are unchanged; these are explicit simulation channels.
  const states=[];
  for(const phase of ['PRE_ALIGN','REAR_DEPLOY','FRONT_DEPLOY','CONTACT_LOCKED_RISE','STAND_150','WALK','STOPPING','SIMULATED_REARM_STAND','INTERMEDIATE_RESTART_50MM','RESTART_STAND_150']){
   const raw=trace.find(f=>f.stand_state===phase);assert(raw,phase);const record=structuredClone(raw);record.reason=record.evidence;record.telemetry.feet=Object.entries(record.telemetry.feet).map(([leg,f])=>({...f,leg,contact:f.state==='STANCE'?'support':f.state==='SWING'?'swing':'unknown'}));
   const result=await page.evaluate(record=>{window.matdogViewer.stopMotionFeed();const ok=window.matdogViewer.applyMotionStatus(record);return {ok,state:window.matdogViewer.snapshot(),xyz:window.matdogViewer.robot().rootObject.position.toArray(),render:window.matdogViewer.forceRender()}},record);
   assert(result.ok,phase);assert.deepEqual(result.xyz,record.telemetry.base_position_m);assert.equal(Object.keys(result.state.joints).length,12);assert(result.render.calls>0);states.push({phase,xyz:result.xyz,triangles:result.render.triangles});
   if(['PRE_ALIGN','STAND_150','WALK','INTERMEDIATE_RESTART_50MM'].includes(phase))await page.screenshot({path:path.join(out,'twin_'+phase+'.png'),fullPage:true});
  }
  assert.deepEqual(errors,[]);assert.equal(writes.length,0);
  fs.writeFileSync(path.join(out,'browser_checks.json'),JSON.stringify({result:'PASS',browser:await browser.version(),actions:48,disabled:46,preserved_maintenance_tabs:6,desktop:true,mobile:true,states,display_write_requests:writes,live_hardware_telemetry:false,production_http_enforcement_log:'integrated_regressions.log: 56 production-handler checks; this demo server is a display fixture'},null,2)+'\n');
  console.log('PAIRED_MOTION_BROWSER = PASS: 48 controls, desktop/mobile, 17 STL, XYZ+12 joints, stale, frozen trace and denied fixture command');
 }finally{await context.close();await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
