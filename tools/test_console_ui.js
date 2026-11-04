// Exercise the shipped browser script with controlled HTTP response ordering.
// No browser, SSH, ROS or motor connection is made by this test.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),path=require('node:path');
const root=process.argv[2]||path.resolve(__dirname,'..');
function deferred(){let resolve,reject;const promise=new Promise((a,b)=>{resolve=a;reject=b;});return {promise,resolve,reject};}
function response(value){return {ok:true,json:async()=>value};}
function consoleApp(){
 const elements=new Map(),stateRequests=[],starts=[],manual=[],posts=[],timers=[];
 const classes={add(){},remove(){},toggle(){}};
 const context2d=new Proxy({},{get:()=>()=>{},set:()=>true});
 const arrows=[1,2,3,4].map(()=>({disabled:false}));
 function element(id){if(!elements.has(id))elements.set(id,{id,value:'',dataset:{},classList:classes,style:{},parentElement:{},getContext:()=>context2d,querySelector:()=>null,querySelectorAll:()=>[],replaceChildren(){},removeAttribute(){},append(){},focus(){},scrollIntoView(){}});return elements.get(id);}
 const sandbox={console,URLSearchParams,AbortSignal,crypto:require('node:crypto').webcrypto,performance,location:{search:''},URL,Image:class {},setTimeout:(fn,ms)=>timers.push([fn,ms]),window:{addEventListener(){}},document:{getElementById:element,querySelector:()=>({content:'test'}),querySelectorAll:s=>s==='.dpad [data-x]'?arrows:[],addEventListener(){},activeElement:null,createElement:()=>element('generated'+Math.random())},fetch:(url,opts)=>{
  if(url==='/api/state'){const d=deferred();stateRequests.push(d);return d.promise;}
  if(opts?.method==='POST'){const data=JSON.parse(opts.body);posts.push({url,data});if(url==='/api/start'){const d=deferred();starts.push(d);return d.promise;}if(url==='/api/manual')manual.push(data);return Promise.resolve(response({success:true}));}
  return Promise.reject(Error('No sensor connection in this test'));
 }};
 const ctx=vm.createContext(sandbox);vm.runInContext(fs.readFileSync(path.join(root,'tools/console/app.js'),'utf8'),ctx);
 const run=code=>vm.runInContext(code,ctx);
 run("state={connected:true,robot_id:11,mode:'idle',job:{},settings:{},schema:{}};");
 return {run,element,arrows,stateRequests,starts,manual,posts,timers,ctx};
}
const idle={robot_id:11,connected:true,mode:'idle',job:{},robots:{},settings:{},schema:{}};
(async()=>{
 const app=consoleApp();
 // A poll began before activation but completes after a successful start.
 const activation=app.run("start('manual')");app.starts[0].resolve(response({lease:'new-session'}));await activation;
 app.stateRequests[0].resolve(response(idle));await new Promise(setImmediate);
 assert.equal(app.run('lease'),'new-session','an old idle response must not revoke the new manual session');
 app.run('press(1,0)');await new Promise(setImmediate);
 assert.equal(app.manual.at(-1).active,true);assert.equal(app.manual.at(-1).x,1);
 assert.equal(app.manual.at(-1).lease,'new-session');
 app.run('release()');await new Promise(setImmediate);assert.equal(app.manual.at(-1).active,false);
 // An up-to-date server state is still authoritative when a worker has exited.
 const fresh=app.run('poll()');app.stateRequests.at(-1).resolve(response(idle));await fresh;
 assert.equal(app.run('lease'),null);assert(app.arrows.every(b=>b.disabled));
 // Reloading a page does not silently grant motion control.
 app.run("state.mode='manual';renderManual()");assert.match(app.element('manualHint').textContent,/rechargée/);assert(app.arrows.every(b=>b.disabled));
 // STOP revokes even a start response still in flight.
 const again=app.run("start('manual')");await app.run('stop()');app.starts.at(-1).resolve(response({lease:'too-late'}));await again;
 assert.equal(app.run('lease'),null);assert(app.arrows.every(b=>b.disabled));
 // A poll begun while the start was pending must also be discarded afterwards.
 const next=app.run("start('manual')");const during=app.run('poll()');app.starts.at(-1).resolve(response({lease:'third'}));await next;
 app.stateRequests.at(-1).resolve(response(idle));await during;assert.equal(app.run('lease'),'third');
 console.log('PASS: stale polls, delayed activation, STOP, release and manual reload');
 // A new STOP choice must reach the supervisor and must never be silently ignored by an old robot.
 app.element('followAlgorithm').value='follow-robust';app.element('followFinish').value='stop';
 const count=app.starts.length;await app.element('followStart').onclick();assert.equal(app.starts.length,count);
 app.run("state.features={finish_targets:['both','checker','stop']}");
 const withStop=app.element('followStart').onclick();assert.equal(app.posts.at(-1).data.finish,'stop');
 app.starts.at(-1).resolve(response({success:true}));await withStop;
 assert(!app.posts.some(p=>p.url==='/api/settings'),'choosing the finish must not rewrite shared settings');
 app.element('quickAlgorithm').value='follow-robust';
 const quick=app.element('quickStart').onclick();assert.equal(app.posts.at(-1).data.finish,'both');app.starts.at(-1).resolve(response({success:true}));await quick;
 console.log('PASS: finish choice, quick launch and old-version rejection');
 // Indicative dimensions must neither be required nor gate the Run button.
 app.element('mapNavMode').value='local';app.element('mapJourney').value='enclosure';app.element('mapSource').value='simulation';
 vm.runInContext(fs.readFileSync(path.join(root,'tools/console/mapping.js'),'utf8'),app.ctx);
 app.run("mapping={available:true,map_id:'local',mode:'local',fresh:true,confirmed:true,map:{},zone:[-1,-1,3,3],route:{entry:[0,0],inside:[1,0],exit:[2,0]}};renderMapGuide()");
 assert.equal(app.element('mapRun').disabled,false,'no indicative frame is required');
 app.run("mapping.enclosure={width:1,depth:1.25,origin:[100,100,0]};enclosureDirty=true;renderMapGuide()");
 assert.equal(app.element('mapRun').disabled,false,'a misplaced or edited reference must not block navigation');
 app.run('mapping.fresh=false;renderMapGuide()');assert.equal(app.element('mapRun').disabled,true,'fresh LiDAR/position remain mandatory');
 console.log('PASS: optional dimensions and mandatory fresh sensors');
})().catch(e=>{console.error(e);process.exitCode=1;});
