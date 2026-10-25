"""Full SLAM regression against the isolated Mac bench. Never accepts a robot URL.
Start a fresh bench then set URMC_MAP_TOKEN to its temporary server token.
"""
import sys,json,time,urllib.request,zipfile,io,math,os
URL='http://127.0.0.1:8894';TOKEN=os.environ['URMC_MAP_TOKEN']
def req(path,data=None):
 r=urllib.request.Request(URL+path,data=json.dumps(data).encode() if data is not None else None,headers={'Authorization':'Bearer '+TOKEN,'Content-Type':'application/json'})
 try:
  with urllib.request.urlopen(r,timeout=20) as s:return json.load(s)
 except urllib.error.HTTPError as e:raise RuntimeError(e.read().decode())
def action(kind,**kw):return req('/action',dict(action=kind,**kw))
def snapshot():return req('/snapshot')
seq=0
assert req('/state').get('simulation') is True, 'Ce test est réservé au banc, jamais au robot réel.'
action('new')
for _ in range(40):
 s=snapshot()
 if s.get('fresh') and s.get('map'):break
 time.sleep(.3)
initial=list(s['pose']);assert s['fresh'] and abs(initial[0]+.4)<.1 and abs(initial[1])<.1, 'Replacer le banc au départ.'
print('Carte initiale',initial,flush=True)
def drive(target):
 global seq
 hold=str(time.monotonic());start=time.monotonic()
 while time.monotonic()-start<35:
  s=snapshot();dx=target-s['pose'][0]
  if abs(dx)<.035:break
  seq+=1;action('manual',seq=seq,hold=hold,active=True,x=1 if dx>0 else -1,yaw=0)
  time.sleep(.08)
 else:raise AssertionError(('manuel bloqué',s['pose'],s['reason']))
 seq+=1;action('manual',seq=seq,active=False);print('Manuel arrivé',s['pose'],flush=True)
if '--resume' not in sys.argv:drive(1.95);drive(-.4)
time.sleep(1)
s=snapshot();mid=s['map_id'];action('zone',map_id=mid,zone=[-.6,-.65,2.2,.65])
for key,p in dict(entry=[-.18,0.],inside=[.98,0.],exit=[1.92,0.]).items():action('route',map_id=mid,key=key,point=p)
r=action('save',map_id=mid,name='validation_enceinte');name=r['name'];print('Sauvegarde',name,flush=True)
r=urllib.request.Request(URL+'/export?name='+name,headers={'Authorization':'Bearer '+TOKEN})
with urllib.request.urlopen(r) as response:payload=response.read()
z=zipfile.ZipFile(io.BytesIO(payload));assert any(n.endswith('.posegraph') for n in z.namelist());assert any(n.endswith('map.pgm') for n in z.namelist());print('ZIP validé',len(payload),flush=True)
action('load',name=name)
for _ in range(30):
 s=snapshot()
 if s.get('fresh') and s.get('mode')=='localization':break
 time.sleep(.3)
mid=s['map_id'];action('pose',map_id=mid,pose=[-.4,0.,0.]);time.sleep(2.5);action('confirm',map_id=mid)
action('obstacles',enabled=True);time.sleep(.5);action('run',map_id=mid);start=time.monotonic();phases=[];detour=False
while time.monotonic()-start<65:
 s=snapshot()
 if not phases or phases[-1]!=s['phase']:phases.append(s['phase']);print('Mission',s['phase'],s['pose'],s['reason'],flush=True)
 if s.get('pose') and abs(s['pose'][1])>.15:detour=True
 if s['phase'] in ('DONE','FAULT','TIMEOUT'):break
 time.sleep(.2)
assert s['phase']=='DONE',(phases,s['pose'],s['reason']);assert detour;assert s['remaining']>0
print('OK : vrai SLAM, graph sauvegardé/rechargé, localisation confirmée, obstacle absent de la carte contourné, entrée/intérieur/sortie et arrêt.',flush=True)
action('stop')
