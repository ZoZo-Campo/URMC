const {chromium}=require('playwright');const assert=require('node:assert/strict');
(async()=>{const browser=await chromium.launch({headless:true,channel:'chrome'});try{
const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
page.on('pageerror',e=>errors.push(e.message));await page.goto('http://127.0.0.1:8775/');
const get=()=>page.evaluate(()=>fetch('/state').then(r=>r.json()));
const wait=async predicate=>{const end=Date.now()+8000;while(Date.now()<end){const d=await get();if(predicate(d))return d;await page.waitForTimeout(100);}throw Error('État non atteint : '+predicate+'\n'+JSON.stringify(await get()))};
await wait(d=>d.status?.fresh&&d.truth?.session?.running===false);
assert.equal((await get()).truth.t,0);
await page.locator('#start').click();await wait(d=>d.status.v>0&&d.truth.t>.5);
await page.locator('#pause').click();const paused=await wait(d=>!d.status.armed&&!d.truth.session.running);
await page.waitForTimeout(800);let d=await get();assert.equal(d.truth.t,paused.truth.t);assert.equal(d.truth.x,paused.truth.x);
await page.locator('#restart').click();await wait(d=>d.truth.t===0&&d.status.state==='WAIT');
await page.locator('#start').click();await wait(d=>d.status.v>0);
for(const key of ['front_blocked','hide_white']){
await page.locator('#'+key).check();await wait(d=>d.truth.perturbations[key]&&d.status.v===0);
await page.locator('#'+key).uncheck();await wait(d=>!d.truth.perturbations[key]&&d.status.v>0);
}
for(const key of ['camera_off','scan_off']){
await page.locator('#'+key).check();await wait(d=>d.truth.perturbations[key]&&!d.status.fresh&&!d.status.armed);
await page.locator('#'+key).uncheck();await wait(d=>!d.truth.perturbations[key]&&d.status.fresh);
assert.equal((await get()).status.armed,false);await page.locator('#start').click();await wait(d=>d.status.v>0);
}
await page.locator('#camera_latency_ms').fill('600');await page.locator('#camera_latency_ms').dispatchEvent('change');
await wait(d=>d.truth.perturbations.camera_latency_ms===600&&!d.status.armed&&!d.status.fresh);
await page.locator('#clearPerturb').click();await wait(d=>d.truth.perturbations.camera_latency_ms===0&&d.status.fresh);
await page.locator('#restart').click();await wait(d=>d.truth.t===0&&d.status.state==='WAIT');
await page.screenshot({path:'/private/tmp/urmc-controls-desktop.png',fullPage:true});
await page.setViewportSize({width:390,height:844});assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
for(const [payload,origin,code] of [[{action:'start'},'http://evil.invalid',403],[{perturbations:{cmd_vel:1}},'http://127.0.0.1:8775',400]]){const r=await fetch('http://127.0.0.1:8775/control',{method:'POST',headers:{'Content-Type':'application/json',Origin:origin},body:JSON.stringify(payload)});assert.equal(r.status,code)}
assert.deepEqual(errors,[]);console.log('OK : attente, démarrage, pause, remise à zéro, obstacle, bordure, coupures caméra/LiDAR, retard, réarmement, mobile, requêtes invalides.');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exit(1)});
