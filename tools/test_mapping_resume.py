"""Stationary SLAM save/resume regression, restricted to the simulated bench."""
import json,os,time,urllib.request
URL=os.environ.get('URMC_BENCH_URL','http://127.0.0.1:8093')
assert URL in ('http://127.0.0.1:8093','http://127.0.0.1:8894')
TOKEN=os.environ['URMC_MAP_TOKEN']
def req(path,data=None):
 r=urllib.request.Request(URL+path,data=json.dumps(data).encode() if data is not None else None,headers={'Authorization':'Bearer '+TOKEN,'Content-Type':'application/json'})
 try:
  with urllib.request.urlopen(r,timeout=20) as f:return json.load(f)
 except urllib.error.HTTPError as e:raise RuntimeError(e.read().decode())
def action(kind,**kw):return req('/action',dict(action=kind,**kw))
def ready():
 for _ in range(100):
  s=req('/snapshot')
  if s.get('map') and s.get('pose') and s.get('fresh'):return s
  time.sleep(.2)
 raise AssertionError(s)
assert req('/state')['simulation'] is True,'Simulation seulement'
action('new');s=ready();pose=s['pose'];before=s['map'];saved=action('save',map_id=s['map_id'],name='test_reprise')
action('resume',name=saved['name'],start_pose=pose);s=ready()
assert s['mode']=='mapping' and not s['active'] and not s['confirmed'] and s['v']==s['w']==0
assert s['map']['width']>=before['width']*.9 and s['map']['height']>=before['height']*.9
assert s['known_percent']>0 and max(abs(a-b) for a,b in zip(pose,s['pose']))<.1
print('OK : graphe sauvegardé et repris en cartographie, carte et pose conservées, aucun déplacement.')
