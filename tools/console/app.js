'use strict';
const $=id=>document.getElementById(id),csrf=document.querySelector('meta[name="console-token"]').content;
let activeRobot=11,selectionVersion=0,connectionDirty=false,switching=false;
let state={},tab='drive',lease=null,sequence=0,holding=null,manualBusy=false,formBuilt=false,settingsDirty=false,lastURL=null,simShown=false;
const names={'idle':'À l’arrêt','manual':'Pilotage manuel','follow-robust':'Suivi robuste','follow-classic':'Suivi historique','enclosure':'Enceinte / SLAM','priority':'Priorité à droite','priority-observe':'Observation LiDAR','barrier':'Piste + barrière','observe':'Observation'};
const reasons={STARTING:'Préparation du programme…',STOP:'Arrêt demandé',MANUAL:'Maintenir une flèche pour déplacer le robot.',WAIT_CAMERA:'Aucune image caméra suffisamment récente.',YIELD:'Attente de la priorité à droite.',CROSS:'Traversée tout droit au cap mémorisé.',FOLLOW_AFTER_INTERSECTION:'Deux bordures retrouvées : suivi repris.',WAIT_LIDAR:'LiDAR absent, périmé ou secteur inconnu.',WAIT_ODOM:'Odométrie absente ou périmée.',WAIT_OPEN:'Barrière fermée ou ouverture à confirmer.',WAIT_BARRIER_VIEW:'Bleu détecté, mais panneau mal reconnu.',WAIT_LANE:'Les deux bordures doivent être visibles.',WAIT_LANE_OFFSET:'Piste trop décalée : pause, puis nouvelle confirmation.',CONFIRM_LANE:'Vérification de trois images de piste cohérentes.',FOLLOW:'Suivi de la piste en cours.',FOLLOW_AFTER_BARRIER:'Barrière franchie, poursuite du suivi.',OBSERVATION:'Observation du panneau, sans commande moteur.',FAULT:'Un conflit de commande ou une erreur a interrompu le mode.'};
function notice(message,error=false){$('notice').textContent=message;$('notice').classList.toggle('error',error);}
async function api(path,data={}){const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':csrf},body:JSON.stringify({...data,robot:activeRobot}),signal:AbortSignal.timeout(path==='mapping'?22000:6000)});const value=await r.json();if(!r.ok)throw Error(value.error||'Action impossible');return value;}
function go(next){release();tab=next;document.querySelectorAll('.tab').forEach(x=>x.hidden=x.id!=='tab-'+next);document.querySelectorAll('nav button').forEach(x=>x.classList.toggle('selected',x.dataset.tab===next));$('title').textContent={drive:'Le robot, en direct.',missions:'À chaque mission, son mode.',mapping:'L’enceinte, en vue.',simulation:'Tester avant de rouler.',settings:'Des réglages, pas du code.',help:'Tout pour partir seul.'}[next];}
document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>go(b.dataset.tab));document.querySelectorAll('[data-go]').forEach(b=>b.onclick=()=>go(b.dataset.go));
async function start(algorithm){if(switching)return;release();lease=null;try{const result=await api('start',{algorithm});lease=result.lease||null;sequence=0;notice(algorithm==='manual'?'Manuel activé. Maintenir une flèche pour rouler.':'Programme lancé. Le bouton STOP reste disponible.');go('drive');}catch(e){notice(e.message,true);}}
$('manualMode').onclick=()=>start('manual');$('quickStart').onclick=()=>start($('quickAlgorithm').value);$('followStart').onclick=()=>start($('followAlgorithm').value);document.querySelectorAll('[data-start]').forEach(b=>b.onclick=()=>start(b.dataset.start));
async function stop(){release();lease=null;try{await api('stop');notice('Arrêt confirmé. La caméra reste disponible.');}catch(e){notice('ARRÊT NON CONFIRMÉ : '+e.message,true);}}
$('stop').onclick=stop;$('centerStop').onclick=release;
$('stopAll').onclick=async()=>{release();lease=null;try{const r=await api('stop-all');notice(r.message,!r.success);}catch(e){notice('ARRÊT GLOBAL NON CONFIRMÉ : '+e.message,true);}};
async function selectRobot(id){
 id=Number(id);if(id===activeRobot||switching)return;release();lease=null;switching=true;$('robotSelector').disabled=true;
 try{await api('select',{selected:id});activeRobot=id;selectionVersion++;state={};lastFrame=null;lastNew=0;previousJob='';settingsDirty=false;connectionDirty=false;formBuilt=false;$('settingsFields').replaceChildren();$('camera').removeAttribute('src');$('videoEmpty').hidden=false;$('robotSelector').value=String(id);$('robotNumber').textContent=id;$('robotName').textContent='TurtleBot '+id;$('videoTitle').textContent='Caméra du robot '+id;$('stop').textContent='■ STOP ROBOT '+id;notice('Robot '+id+' sélectionné. Le mode du robot précédent a été arrêté.');}
 catch(e){$('robotSelector').value=String(activeRobot);notice(e.message,true);}
 finally{switching=false;$('robotSelector').disabled=false;}
}
$('robotSelector').onchange=e=>selectRobot(e.target.value);document.querySelectorAll('[data-select]').forEach(b=>b.onclick=()=>selectRobot(b.dataset.select));
$('robotHost').oninput=$('robotDomain').oninput=()=>{connectionDirty=true;};
$('saveConnection').onclick=async()=>{try{await api('robot/configure',{host:$('robotHost').value.trim(),domain:Number($('robotDomain').value)});connectionDirty=false;notice('Connexion enregistrée pour le robot '+activeRobot+'. Cliquer Connecter / préparer.');}catch(e){notice(e.message,true);}};

$('connect').onclick=async()=>{release();lease=null;try{await api('connect');notice('Connexion et préparation en cours. Aucun roulage lancé.');}catch(e){notice(e.message,true);}};
$('ssh').onclick=async()=>{try{await api('ssh');notice('Terminal SSH ouvert sur le Mac. Saisir le mot de passe dans ce Terminal si demandé.');}catch(e){notice(e.message,true);}};
function press(x,y,button){if(!['drive','mapping'].includes(tab)||!lease||state.mode!=='manual'||!state.connected)return;release();holding={x,y,hold:crypto.randomUUID(),button};if(button)button.classList.add('held');pump();}
function release(){const old=holding;holding=null;document.querySelectorAll('.held').forEach(x=>x.classList.remove('held'));if(old&&lease)api('manual',{lease,seq:++sequence,active:false}).catch(()=>{});}
async function pump(){if(!holding||manualBusy)return;const current=holding;manualBusy=true;try{await api('manual',{lease,seq:++sequence,active:true,hold:current.hold,x:current.x,yaw:current.y});}catch(e){release();notice('Pilotage interrompu : '+e.message,true);}finally{manualBusy=false;if(holding)setTimeout(pump,90);}}
document.querySelectorAll('.dpad [data-x]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);press(Number(b.dataset.x),Number(b.dataset.y),b);};b.onpointerup=release;b.onpointercancel=release;b.onlostpointercapture=release;});
const directions={ArrowUp:[1,0],ArrowDown:[-1,0],ArrowLeft:[0,1],ArrowRight:[0,-1]};
window.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(e.code==='Space'){e.preventDefault();stop();return;}if(directions[e.key]&&!e.repeat){e.preventDefault();press(...directions[e.key]);}});window.addEventListener('keyup',e=>{if(directions[e.key])release();});window.addEventListener('blur',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});window.addEventListener('pagehide',release);
function fields(schema,settings){if(formBuilt)return;const groups={};for(const [key,spec] of Object.entries(schema)){const [value,min,max,step,label,group]=spec;if(!groups[group]){const section=document.createElement('section');section.className='card settings-group';const heading=document.createElement('h2');heading.textContent=group;section.append(heading);$('settingsFields').append(section);groups[group]=section;}const item=document.createElement('div');item.className='field';const lab=document.createElement('label');lab.htmlFor='setting-'+key;lab.textContent=label;const input=document.createElement('input');Object.assign(input,{type:'number',id:'setting-'+key,name:key,min,max,step,value:settings[key]??value});input.dataset.default=value;input.oninput=()=>settingsDirty=true;item.append(lab,input);groups[group].append(item);}formBuilt=true;}
$('settingsForm').onsubmit=async e=>{e.preventDefault();const data={};for(const input of $('settingsFields').querySelectorAll('input'))data[input.name]=Number(input.value);try{const result=await api('settings',data);settingsDirty=false;$('settingsResult').textContent='Enregistré. Application sur le robot en moins d’une seconde.';notice('Réglages enregistrés pour la perception et la conduite.');}catch(err){$('settingsResult').textContent=err.message;notice(err.message,true);}};
$('defaults').onclick=()=>{for(const input of $('settingsFields').querySelectorAll('input'))input.value=input.dataset.default;settingsDirty=true;$('settingsResult').textContent='Valeurs d’origine chargées. Cliquer Enregistrer pour les appliquer.';};
$('simStart').onclick=async()=>{simShown=false;$('simFrame').hidden=true;$('simEmpty').hidden=false;try{await api('simulation/start',{mission:Number($('simMission').value)});notice('Préparation de la simulation…');}catch(e){notice(e.message,true);}};
$('simStop').onclick=async()=>{try{await api('simulation/stop');$('simFrame').hidden=true;$('simFrame').src='about:blank';$('simEmpty').hidden=false;simShown=false;}catch(e){notice(e.message,true);}};
$('showLogs').onclick=()=>{$('logs').hidden=!$('logs').hidden;};
let previousJob='';
async function poll(){const version=selectionVersion;try{const r=await fetch('/api/state',{signal:AbortSignal.timeout(2000)});const incoming=await r.json();if(version!==selectionVersion||incoming.robot_id!==activeRobot){setTimeout(poll,200);return;}state=incoming;$('demo').hidden=!state.demo;$('connection').textContent=state.demo?'Démonstration':state.connected?'SSH connecté':'Non connecté';$('connection').classList.toggle('online',state.connected);$('hostLabel').textContent=state.host;$('robotSelector').value=String(activeRobot);$('robotNumber').textContent=activeRobot;$('robotName').textContent='TurtleBot '+activeRobot;$('videoTitle').textContent='Caméra du robot '+activeRobot;$('stop').textContent='■ STOP ROBOT '+activeRobot;$('videoOnly').href='http://127.0.0.1:'+state.video_port+'/';for(const id of [11,9]){$('fleetStatus'+id).textContent=state.robots?.[id]?.connected?'Caméra connectée':'Non connecté';const option=$('robotSelector').querySelector('option[value="'+id+'"]');if(option&&state.robots?.[id]?.host)option.textContent='Robot '+id+' · .'+state.robots[id].host.split('.').pop()+(id===9&&!state.robots[id].connected?' à confirmer':'');}if(!connectionDirty&&document.activeElement!==$('robotHost'))$('robotHost').value=state.host;if(!connectionDirty&&document.activeElement!==$('robotDomain'))$('robotDomain').value=state.domain;$('connectionHint').textContent='Robot '+activeRobot+' · '+(state.domain_detected?'domaine ROS constaté sur le robot':'domaine ROS proposé, vérifié lors de la préparation')+(activeRobot===9?' · IP .209 à confirmer.':'.');$('saveConnection').disabled=state.running||state.job?.busy;$('footHost').textContent=state.host;$('connect').disabled=!!state.job?.busy;$('connect').textContent=state.job?.busy?'Préparation…':state.connected?'Repréparer le robot':'Connecter / préparer';for(const b of [$('manualMode'),$('quickStart'),$('followStart'),...document.querySelectorAll('[data-start]')])b.disabled=!state.connected||state.job?.busy;
 if(!state.connected&&holding)release();if(state.mode!=='manual')lease=null;
 $('mode').textContent=names[state.mode]||'À l’arrêt';const s=state.status||{};$('speed').textContent=((s.v||0)*100).toFixed(1)+' cm/s';$('robotState').textContent=state.connected?(names[state.mode]||'À l’arrêt'):'Robot non connecté';$('robotReason').textContent=s.reason||reasons[s.state]||'La caméra reste visible quand une mission s’arrête.';$('statusDot').classList.toggle('active',state.connected&&state.running);$('logs').textContent=(state.events||[]).slice(-25).map(e=>e.time+'  '+e.message).join('\n')||'Aucun événement.';
 if(!$('documents').dataset.loaded&&state.documents?.length){$('documents').replaceChildren();state.documents.forEach((name,index)=>{const a=document.createElement('a');a.className='secondary';a.href='/api/document?id='+index;a.target='_blank';a.rel='noopener';a.textContent=name+' ↗';$('documents').append(a);});$('documents').dataset.loaded='yes';}
 fields(state.schema||{},state.settings||{});if(!settingsDirty)for(const i of $('settingsFields').querySelectorAll('input'))if(i!==document.activeElement&&state.settings?.[i.name]!==undefined)i.value=state.settings[i.name];
 const job=state.job?.message||'';if(job&&job!==previousJob){notice(job,!!state.job?.error);previousJob=job;}
 if(state.simulation&&!state.job?.busy&&!simShown){$('simFrame').src='http://127.0.0.1:8875/';$('simFrame').hidden=false;$('simEmpty').hidden=true;simShown=true;}
 }catch(e){$('connection').textContent='Console indisponible';release();}setTimeout(poll,650);}
poll();
let lastFrame=null,lastNew=0;
async function frame(){const started=performance.now(),robot=activeRobot,version=selectionVersion;try{
 if(!state.connected)throw Error('Non connecté');
 const r=await fetch('/api/frame?robot='+robot+'&t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(1300)});
 if(!r.ok)throw Error('Caméra indisponible');
 if(Number(r.headers.get('X-Robot-ID'))!==robot)throw Error('Identité vidéo incohérente');
 const blob=await r.blob();if(version!==selectionVersion)throw Error('Robot changé');const url=URL.createObjectURL(blob);$('camera').src=url;await $('camera').decode();if(version!==selectionVersion){URL.revokeObjectURL(url);throw Error('Robot changé');}
 if(lastURL)URL.revokeObjectURL(lastURL);lastURL=url;$('fleetCamera'+robot).src=url;$('videoEmpty').hidden=true;
 const seq=r.headers.get('X-Frame-Sequence');if(seq!==lastFrame||state.demo){lastFrame=seq;lastNew=performance.now();}const fresh=performance.now()-lastNew<1300;
 $('camera').style.opacity=fresh?1:.2;$('fleetCamera'+robot).style.opacity=fresh?1:.2;$('videoBadge').textContent=state.demo?'DÉMONSTRATION':fresh?'EN DIRECT':'IMAGE FIGÉE';$('videoBadge').classList.toggle('on',fresh&&!state.demo);
 $('frameInfo').textContent=state.demo?'Robot '+robot+' · aucune image réelle':'Robot '+robot+' · image '+seq+' · '+Math.round(performance.now()-started)+' ms · âge '+Number(r.headers.get('X-Source-Age')).toFixed(2)+' s';
 }catch(e){if(version===selectionVersion){$('videoBadge').textContent='HORS LIGNE';$('videoBadge').classList.remove('on');$('camera').style.opacity=.2;$('videoEmpty').hidden=false;}}
 setTimeout(frame,Math.max(0,130-(performance.now()-started)));}frame();
const previewURLs={};
async function previews(){
 if(tab==='drive')for(const id of [11,9]){
  if(id===activeRobot)continue;
  if(!state.robots?.[id]?.connected){$('fleetCamera'+id).style.opacity=.2;continue;}
  try{const r=await fetch('/api/frame?robot='+id+'&t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(1200)});if(!r.ok||Number(r.headers.get('X-Robot-ID'))!==id)throw Error();const url=URL.createObjectURL(await r.blob());$('fleetCamera'+id).src=url;$('fleetCamera'+id).style.opacity=1;if(previewURLs[id])URL.revokeObjectURL(previewURLs[id]);previewURLs[id]=url;}catch(e){$('fleetCamera'+id).style.opacity=.2;}
 }
 setTimeout(previews,900);
}previews();

const initial=new URLSearchParams(location.search);if(['drive','missions','mapping','simulation','settings','help'].includes(initial.get('tab')))go(initial.get('tab'));if(/^[1-5]$/.test(initial.get('mission')||''))$('simMission').value=initial.get('mission');

function drawLidar(data){
 const c=$('lidarCanvas'),ctx=c.getContext('2d'),cx=320,cy=300,scale=125;
 ctx.fillStyle='#142c29';ctx.fillRect(0,0,c.width,c.height);
 const point=(x,y)=>[cx-y*scale,cy-x*scale];
 ctx.strokeStyle='#3b5951';ctx.lineWidth=1;ctx.fillStyle='#9ab5ac';ctx.font='14px system-ui';
 for(const r of [.5,1,1.5,2]){ctx.beginPath();ctx.arc(cx,cy,r*scale,0,Math.PI*2);ctx.stroke();ctx.fillText(r+' m',cx+6,cy-r*scale+15);}
 ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,480);ctx.moveTo(0,cy);ctx.lineTo(640,cy);ctx.stroke();
 const radius=data.right_threshold||.8,lo=(data.right_start||-120)*Math.PI/180,hi=(data.right_end||-20)*Math.PI/180;
 ctx.save();ctx.beginPath();ctx.rect(cx+.12*scale,cy-(data.lane_forward||.75)*scale,3*scale,((data.lane_forward||.75)+.15)*scale);ctx.clip();
 ctx.fillStyle='#eab45b27';ctx.beginPath();ctx.moveTo(cx,cy);
 for(let a=lo;a<=hi+.01;a+=.02){const p=point(radius*Math.cos(a),radius*Math.sin(a));ctx.lineTo(...p);}ctx.closePath();ctx.fill();
 ctx.strokeStyle='#eab45b';ctx.stroke();ctx.restore();ctx.globalAlpha=data.fresh?1:.25;
 ctx.fillStyle='#cad8d2';for(const [a,r] of data.points||[]){const p=point(r*Math.cos(a),r*Math.sin(a));ctx.fillRect(p[0]-1.5,p[1]-1.5,3,3);}
 for(const o of data.traffic?.objects||[]){const p=point(o.x,o.y);ctx.strokeStyle=o.blocking?'#ff6456':o.pending?'#eab45b':'#8aaba0';ctx.lineWidth=3;ctx.beginPath();ctx.arc(...p,Math.max(7,o.size*scale/2),0,Math.PI*2);ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.fillText('T'+o.id+' · '+o.distance.toFixed(2)+' m',p[0]+10,p[1]-10);}
 ctx.globalAlpha=1;ctx.fillStyle='#83e6bb';ctx.beginPath();ctx.moveTo(cx,cy-12);ctx.lineTo(cx-9,cy+8);ctx.lineTo(cx+9,cy+8);ctx.closePath();ctx.fill();
 ctx.fillStyle='#e4efe9';ctx.fillText('AVANT',cx-25,20);ctx.fillText('DROITE',565,cy+25);ctx.fillText('ROBOT '+activeRobot,cx-38,cy+30);
}
async function lidar(){const robot=activeRobot,version=selectionVersion;try{
 const r=await fetch('/api/lidar?robot='+robot,{cache:'no-store',signal:AbortSignal.timeout(1200)});if(!r.ok)throw Error('LiDAR indisponible');const data=await r.json();if(version!==selectionVersion||data.robot_id!==activeRobot)throw Error('Robot changé');
 $('lidarRobot').textContent='Robot '+robot;$('lidarBadge').textContent=data.demo?'DÉMONSTRATION':data.fresh?'EN DIRECT':'MESURES PÉRIMÉES';$('lidarBadge').classList.toggle('on',!!data.fresh&&!data.demo);drawLidar(data);
 const distance=v=>v==null?'inconnue':Number(v).toFixed(2)+' m';$('lidarInfo').textContent='Avant : '+distance(data.front)+' · à droite : '+distance(data.right)+' · seuil objet–robot : '+distance(data.right_threshold)+' · âge : '+(data.age??'?')+' s';
 const traffic=data.traffic;$('trafficInfo').textContent=!data.fresh?'Aucune décision fiable sans mesures récentes.':traffic?traffic.reason+(traffic.objects?.length?'\n'+traffic.objects.map(o=>'T'+o.id+' : '+o.label+' · '+o.distance.toFixed(2)+' m · approche '+o.vy.toFixed(2)+' m/s').join('\n'):''):'Mesures brutes. Lancer Observer caméra + LiDAR pour suivre les objets.';
 const s=state.status||{};$('priorityProgress').textContent=s.crossing_distance!==undefined?'Traversée : '+s.crossing_distance.toFixed(2)+' / '+s.crossing_max.toFixed(2)+' m. '+(s.reason||''):'';
 }catch(e){if(version===selectionVersion){$('lidarBadge').textContent='HORS LIGNE';$('lidarBadge').classList.remove('on');$('lidarInfo').textContent='LiDAR indisponible pour le robot '+robot;drawLidar({fresh:false});}}setTimeout(lidar,220);
}drawLidar({fresh:false});lidar();
