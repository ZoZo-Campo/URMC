'use strict';
let speedDraft=false,speedBusy=false,speedRobot=null,speedKey=null;
$('networkConnect').onclick=()=>$('connect').click();$('networkSSH').onclick=()=>$('ssh').click();
$('speedLimit').oninput=()=>{speedDraft=true;$('speedValue').textContent=$('speedLimit').value+' cm/s';$('speedHint').textContent='Modification non envoyée. Cliquer sur Appliquer cette vitesse.';};
$('speedScope').onchange=()=>{speedDraft=false;speedKey=null;};
$('speedForm').onsubmit=async e=>{
 e.preventDefault();if(speedBusy||!state.connected||state.robot_id!==activeRobot||speedRobot!==activeRobot||speedKey!==$('speedScope').value)return;
 const robot=activeRobot,version=selectionVersion,key=$('speedScope').value;speedBusy=true;$('speedApply').disabled=true;
 try{const result=await api('settings',{...state.settings,[key]:Number($('speedLimit').value)/100});if(robot!==activeRobot||version!==selectionVersion)return;state.settings=result.settings;speedDraft=false;$('speedHint').textContent='Vitesse enregistrée sur le robot '+robot+'. Aucun départ demandé.';notice('Limite de vitesse appliquée au robot '+robot+'.');}
 catch(err){if(robot===activeRobot)$('speedHint').textContent=err.message;}
 finally{speedBusy=false;}
};
function updateDashboard(){
 const key=$('speedScope').value;
 if(speedRobot!==activeRobot||speedKey!==key){speedRobot=activeRobot;speedKey=key;speedDraft=false;$('speedHint').textContent='Réglage du robot '+activeRobot+'. Aucun changement envoyé.';}
 const spec=state.schema?.[key];if(spec){$('speedLimit').min=spec[1]*100;$('speedLimit').max=spec[2]*100;$('speedLimit').step=spec[3]*100;$('speedMin').textContent=spec[1]*100+' cm/s';$('speedMax').textContent=spec[2]*100+' cm/s';if(!speedDraft&&!speedBusy)$('speedLimit').value=(state.settings?.[key]??spec[0])*100;}
 $('speedValue').textContent=Number($('speedLimit').value).toFixed(1).replace('.0','')+' cm/s';$('speedApply').disabled=speedBusy||!state.connected||!!state.job?.busy;
 $('healthRobot').textContent='Robot '+activeRobot;$('healthNetwork').textContent=(state.host||'Adresse à configurer')+' · '+(state.connected?'connecté':'hors ligne');
 $('healthCamera').textContent=$('videoBadge').textContent;$('healthLidar').textContent=$('lidarBadge').textContent;$('healthSpeed').textContent=state.connected?((state.status?.v||0)*100).toFixed(1)+' cm/s':'—';$('healthMode').textContent=names[state.mode]||'À l’arrêt';
 $('networkConnect').disabled=!!state.job?.busy;$('networkSSH').disabled=!!state.job?.busy;
 const signature=JSON.stringify([activeRobot,state.robots]);if($('networkFleet').dataset.signature!==signature){$('networkFleet').dataset.signature=signature;$('networkFleet').replaceChildren();for(const id of [11,9]){const data=state.robots?.[id]||{},card=document.createElement('article');card.className='card profile-card'+(id===activeRobot?' chosen':'');const badge=document.createElement('span');badge.className='profile-number';badge.textContent=String(id);const title=document.createElement('h3');title.textContent='TurtleBot '+id;const addr=document.createElement('p');addr.textContent=data.host||'Adresse à configurer';const info=document.createElement('small');info.textContent=(data.connected?'Connecté':'Hors ligne')+' · domaine ROS '+(data.domain??'?');const button=document.createElement('button');button.className=id===activeRobot?'primary':'secondary';button.textContent=id===activeRobot?'Profil sélectionné':'Configurer ce robot';button.disabled=switching;button.onclick=()=>selectRobot(id);card.append(badge,title,addr,info,button);$('networkFleet').append(card);}}
 setTimeout(updateDashboard,500);
}
updateDashboard();
