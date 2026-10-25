"""Single local Mac console. Remote movement is always supervised on the robot."""
import argparse,io,json,os,secrets,shlex,signal,subprocess,sys,tarfile,threading,time,urllib.error,urllib.request
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/urmc_core'))
from urmc_core.operator_settings import DEFAULTS,SCHEMA,validate
import mac_robot
PORT=8090

class Backend:
 def __init__(self,demo=False,robot_id=11,host=None,domain=None):
  self.robot_id=int(robot_id);self.host=host or ('192.168.42.209' if self.robot_id==9 else mac_robot.HOST)
  import ipaddress
  self.host=str(ipaddress.IPv4Address(self.host))
  self.target='ubuntu@'+self.host;self.domain=int(domain) if domain is not None else 30+self.robot_id;self.domain_detected=False
  if not 0<=self.domain<=232:raise ValueError('Domaine ROS invalide.')
  self.video_port=18000+self.robot_id*10;self.agent_port=self.video_port+1;self.tunnels=[]
  self.demo=demo;self.connected=demo;self.robot_token=None;self.job_lock=threading.Lock();self.job={'busy':False,'message':'Prêt. Connecter le robot pour commencer.'};self.last={};self.sim=None;self.sim_cancel=threading.Event();self.sim_name='urmc-console-'+str(os.getpid());self.demo_settings=dict(DEFAULTS);self.demo_mode='idle';self.demo_lease=None
 def remote(self,command,**kwargs):
  return subprocess.run(mac_robot.SSH+['-o','BatchMode=yes',self.target,command],check=True,**kwargs)
 def command(self,script,*args):
  return f'URMC_ROBOT_DOMAIN={self.domain} URMC_ROBOT_ID={self.robot_id} '+mac_robot.command(script,*args)
 def close_tunnels(self):
  for process in self.tunnels:
   if process.poll() is None:
    process.terminate()
    try:process.wait(timeout=2)
    except subprocess.TimeoutExpired:process.kill()
  self.tunnels=[]
 def request(self,path,data=None,timeout=3):
  if not self.robot_token:raise ValueError('Connecter le robot avant cette action.')
  req=urllib.request.Request(f'http://127.0.0.1:{self.agent_port}'+path,data=json.dumps(data).encode() if data is not None else None,headers={'Authorization':'Bearer '+self.robot_token,'Content-Type':'application/json'})
  try:
   with urllib.request.urlopen(req,timeout=timeout) as response:return json.load(response)
  except urllib.error.HTTPError as exc:
   try:message=json.load(exc).get('error','Erreur robot')
   except Exception:message='Erreur robot'
   raise ValueError(message) from None
 def state(self):
  if self.demo:
   state=dict(connected=True,mode=self.demo_mode,running=self.demo_mode!='idle',status={'state':'DEMONSTRATION','v':0,'w':0},settings=self.demo_settings,schema=SCHEMA,events=[])
  else:
   try:
    state=self.request('/state',timeout=.6);self.connected=True;self.last=state
   except Exception:
    self.connected=False;state=dict(self.last);state.update(connected=False,running=False)
  state.update(demo=self.demo,host=self.host,robot_id=self.robot_id,domain=self.domain,domain_detected=self.domain_detected,video_port=self.video_port,job=dict(self.job),simulation=self.sim is not None and self.sim.poll() is None)
  state['documents']=[p.name for p in document_files()]
  state.setdefault('settings',dict(DEFAULTS));state.setdefault('schema',SCHEMA);state.setdefault('events',[])
  return state
 def job_start(self,label,operation):
  if not self.job_lock.acquire(blocking=False):raise ValueError('Une préparation est déjà en cours.')
  self.job={'busy':True,'message':label}
  def run():
   try:operation();self.job={'busy':False,'message':'Prêt. '+label+' : terminé.'}
   except Exception as exc:self.job={'busy':False,'message':str(exc),'error':True}
   finally:self.job_lock.release()
  threading.Thread(target=run,daemon=True).start();return {'success':True}
 def stage(self,message):self.job={'busy':True,'message':message}
 def connect(self):
  if self.demo:return {'success':True}
  return self.job_start('Préparation du robot',self.prepare)
 def prepare(self):
  self.stage('Connexion SSH au robot…')
  try:subprocess.run(mac_robot.SSH+['-o','BatchMode=yes',self.target,'true'],check=True,capture_output=True,text=True,timeout=10)
  except subprocess.SubprocessError:raise ValueError('SSH inaccessible. Vérifier le réseau et se connecter une fois avec ssh ubuntu@'+self.host+' dans un Terminal.') from None
  probe="""import json,os
from pathlib import Path
found=set()
for p in Path('/proc').iterdir():
 try:
  if not p.name.isdigit() or p.stat().st_uid!=os.getuid():continue
  args=p.joinpath('cmdline').read_bytes().decode().split(chr(0))
  if not any(Path(a).name in ('v4l2_camera_node','turtlebot3_ros') for a in args):continue
  values=dict(v.split('=',1) for v in p.joinpath('environ').read_bytes().decode().split(chr(0)) if '=' in v)
  found.add(int(values.get('ROS_DOMAIN_ID','0')))
 except (OSError,ValueError):pass
print(json.dumps(sorted(found)))
"""
  domains=json.loads(self.remote('python3 -c '+shlex.quote(probe),capture_output=True,text=True,timeout=10).stdout)
  if len(domains)>1:raise ValueError('Plusieurs domaines ROS actifs sur ce robot. Vérifier les pilotes en SSH.')
  if domains and domains[0]!=self.domain:raise ValueError('Pilotes actifs dans le domaine ROS '+str(domains[0])+', attendu '+str(self.domain)+'. Vérifier le profil et garder des domaines différents pour les deux robots.')
  if domains:self.domain_detected=True
  self.stage('Arrêt des anciens essais avant mise à jour…')
  # Only our known scripts, belonging to ubuntu, are stopped.
  stop=mac_robot.STOP
  self.remote('python3 -c '+shlex.quote(stop),capture_output=True,timeout=12)
  # Stop the previous console supervisor before replacing its source.
  cleanup='''from pathlib import Path
import os,signal,time
pids=[]
script=str(Path.home()/"urmc_essais_20261020/kit_robot/operator_agent.py")
for p in Path('/proc').iterdir():
 try:
  if p.name.isdigit() and p.stat().st_uid==os.getuid() and script in p.joinpath('cmdline').read_bytes().decode().split(chr(0)):
   os.kill(int(p.name),signal.SIGTERM);pids.append(p)
 except (OSError,ProcessLookupError):pass
for _ in range(30):
 if all(not p.exists() or not p.joinpath('cmdline').read_bytes() for p in pids):break
 time.sleep(.1)
'''
  self.remote('python3 -c '+shlex.quote(cleanup),capture_output=True,timeout=10)
  self.stage('Transfert des programmes et des réglages…')
  with tempfile_archive() as stream:
   self.remote('mkdir -p "$HOME/'+mac_robot.REMOTE+'/archives"; if [ -d "$HOME/'+mac_robot.REMOTE+'/kit_robot" ]; then tar -czf "$HOME/'+mac_robot.REMOTE+'/archives/avant_console.tar.gz" -C "$HOME/'+mac_robot.REMOTE+'" kit_robot src; fi; tar --no-same-owner -xzf - -C "$HOME/'+mac_robot.REMOTE+'"',stdin=stream,capture_output=True,timeout=90)
  build='set -e; source /opt/ros/humble/setup.bash; if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi; cd "$HOME/'+mac_robot.REMOTE+'"; colcon build --symlink-install --packages-select urmc_core urmc_mission_01 urmc_mission_02 urmc_mission_03 urmc_mission_04'
  self.stage('Compilation ROS 2 sur le robot…')
  completed=subprocess.run(mac_robot.SSH+['-o','BatchMode=yes',self.target,'bash -c '+shlex.quote(build)],capture_output=True,text=True,timeout=150)
  if completed.returncode:raise ValueError('Compilation échouée : '+completed.stderr[-1000:])
  # Restart the viewer so the saved settings also affect its annotations.
  viewer_cleanup=cleanup.replace('operator_agent.py','retour_video.py').replace('signal.SIGTERM','signal.SIGINT')
  self.remote('python3 -c '+shlex.quote(viewer_cleanup),capture_output=True,timeout=10)
  self.stage('Démarrage de la caméra et du superviseur…')
  config_path='$HOME/'+mac_robot.REMOTE+'/session_robot11/reglages.json'
  self.remote('URMC_SETTINGS_FILE="'+config_path+'" '+self.command('suivi_piste_camera.py','--prepare-only'),capture_output=True,timeout=40)
  result=self.remote(self.command('operator_agent.py','--ensure'),capture_output=True,text=True,timeout=12)
  token=result.stdout.strip()
  if not token or len(token)>128:raise ValueError('Le superviseur n’a pas démarré correctement.')
  self.robot_token=token
  self.close_tunnels()
  for local,remote in ((self.video_port,8080),(self.agent_port,8091)):
   process=subprocess.Popen(mac_robot.SSH+['-o','BatchMode=yes','-N','-o','ExitOnForwardFailure=yes','-L',f'127.0.0.1:{local}:127.0.0.1:{remote}',self.target],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
   self.tunnels.append(process);time.sleep(.15)
   if process.poll() is not None:raise ValueError('Tunnel du robot '+str(self.robot_id)+' indisponible : '+process.stderr.read()[-300:])
  for _ in range(25):
   try:
    identity=self.request('/state',timeout=.5)
    if identity.get('robot_id')!=self.robot_id:raise ValueError('Identité du superviseur incohérente.')
    self.connected=True;return
   except Exception:time.sleep(.2)
  raise ValueError('Superviseur inaccessible : vérifier le tunnel SSH et le port 8091.')
 def action(self,path,data):
  if self.demo:
   if path=='/settings':self.demo_settings=validate(data)
   elif path=='/start':
    if data.get('algorithm') not in ('manual','follow-robust','follow-classic','barrier','observe','priority','priority-observe','enclosure'):raise ValueError('Algorithme inconnu.')
    self.demo_mode=data['algorithm'];self.demo_lease=secrets.token_hex(16)
   elif path=='/stop':self.demo_mode='idle';self.demo_lease=None
   elif path=='/manual':
    if self.demo_mode!='manual' or data.get('lease')!=self.demo_lease:raise ValueError('Mode manuel inactif.')
   else:raise ValueError('Action inconnue.')
   return {'success':True,'lease':self.demo_lease,'settings':self.demo_settings}
  if path in ('/start','/stop'):data=dict(data,generation=time.monotonic_ns())
  return self.request(path,data,timeout=18 if path=='/mapping' else 3)
 def start_sim(self,mission):
  if isinstance(mission,bool) or mission not in (1,2,3,4,5):raise ValueError('Mission de simulation invalide.')
  if self.job_lock.locked():raise ValueError('Une préparation est déjà en cours.')
  cancel=threading.Event();self.sim_cancel=cancel
  def start():
   self._stop_sim()
   if cancel.is_set():raise ValueError('Simulation annulée.')
   self.stage('Préparation de la simulation…')
   try:subprocess.run(['docker','info'],check=True,capture_output=True,timeout=5)
   except (OSError,subprocess.SubprocessError):raise ValueError('Démarrer Docker ou Colima, puis réessayer.') from None
   image=subprocess.run(['docker','image','inspect','urmc-humble:local'],capture_output=True)
   if image.returncode:subprocess.run(['docker','build','-t','urmc-humble:local',str(ROOT)],check=True,capture_output=True,timeout=240)
   if cancel.is_set():raise ValueError('Simulation annulée.')
   directory=ROOT/'session_console';directory.mkdir(exist_ok=True)
   with (directory/'simulation.log').open('w') as log:
    self.sim=subprocess.Popen(['docker','run','--rm','--name',self.sim_name,'--label','urmc.console=true','-p','127.0.0.1:8875:8765','-v',str(ROOT)+':/project:ro','urmc-humble:local','bash','/project/tools/container_run.sh',f'{mission:02d}'],stdout=log,stderr=subprocess.STDOUT)
   for _ in range(90):
    if cancel.is_set():self._stop_sim();raise ValueError('Simulation annulée.')
    if self.sim.poll() is not None:raise ValueError('Simulation arrêtée au démarrage. Voir session_console/simulation.log.')
    try:
     with urllib.request.urlopen('http://127.0.0.1:8875/state',timeout=.4):return
    except Exception:time.sleep(.5)
   raise ValueError('La simulation tarde à démarrer. Vérifier session_console/simulation.log.')
  return self.job_start('Simulation '+str(mission),start)
 def stop_sim(self):
  self.sim_cancel.set();return self._stop_sim()
 def _stop_sim(self):
  if self.sim is not None and self.sim.poll() is None:
   subprocess.run(['docker','stop','-t','3',self.sim_name],capture_output=True,timeout=8)
   try:self.sim.wait(timeout=3)
   except subprocess.TimeoutExpired:pass
  self.sim=None;return {'success':True}

from contextlib import contextmanager
import tempfile
@contextmanager
def tempfile_archive():
 with tempfile.TemporaryFile() as stream:
  with tarfile.open(fileobj=stream,mode='w:gz',dereference=True) as archive:
   def clean(info):
    if '__pycache__' in info.name or info.name.endswith(('.pyc','.DS_Store')):return None
    info.mtime=0;return info
   for name in ('src','kit_robot'):archive.add(ROOT/name,arcname=name,filter=clean)
  stream.seek(0);yield stream


def document_files():
 directories=[ROOT/'docs/documents']
 return sorted([p for d in directories if d.exists() for p in d.glob('*.pdf')],key=lambda p:p.name)

class Fleet:
 def __init__(self,demo=False,config_path=None):
  self.demo=demo;self.lock=threading.RLock();self.selected=11;self.mapping_sim=None;self.mapping_token=None
  self.config_path=Path(config_path) if config_path else ROOT/'session_console/robots.json'
  config={}
  try:config=json.loads(self.config_path.read_text())
  except (OSError,ValueError):pass
  self.robots={i:Backend(demo,robot_id=i,**config.get(str(i),{})) for i in (11,9)}
 @property
 def connected(self):return any(robot.connected for robot in self.robots.values())
 def get_robot(self,robot_id):
  if isinstance(robot_id,bool):raise ValueError('Robot inconnu.')
  try:return self.robots[int(robot_id)]
  except (ValueError,TypeError,KeyError):raise ValueError('Choisir le robot 9 ou 11.') from None
 def for_request(self,data):
  robot=self.get_robot(data.get('robot'))
  if robot.robot_id!=self.selected:raise ValueError('Le robot sélectionné a changé. Commande ancienne refusée.')
  return robot
 def select(self,robot_id):
  with self.lock:
   target=self.get_robot(robot_id);old=self.robots[self.selected]
   if target is not old:
    if old.robot_token or old.demo:old.action('/stop',{})
    self.selected=target.robot_id
   return {'success':True,'robot':self.selected}
 def configure(self,data):
  import ipaddress
  with self.lock:
   robot=self.for_request(data)
   if robot.job_lock.locked() or robot.state().get('running'):raise ValueError('Arrêter la mission et la préparation avant de modifier la connexion.')
   host=str(ipaddress.IPv4Address(data.get('host','')));domain=data.get('domain')
   if isinstance(domain,bool) or not isinstance(domain,int) or not 0<=domain<=232:raise ValueError('Domaine ROS invalide (0 à 232).')
   if any(other is not robot and other.host==host for other in self.robots.values()):raise ValueError('Les deux robots doivent avoir des adresses différentes.')
   if any(other is not robot and other.domain==domain for other in self.robots.values()):raise ValueError('Les deux robots doivent avoir des domaines ROS différents.')
   robot.close_tunnels();robot.host=host;robot.target='ubuntu@'+host;robot.domain=domain;robot.domain_detected=False;robot.robot_token=None;robot.connected=robot.demo;robot.last={}
   if not self.demo:
    self.config_path.parent.mkdir(exist_ok=True);self.config_path.write_text(json.dumps({str(i):{'host':r.host,'domain':r.domain} for i,r in self.robots.items()},indent=2))
   return {'success':True}
 def state(self):
  from concurrent.futures import ThreadPoolExecutor
  with ThreadPoolExecutor(max_workers=2) as pool:states=dict(zip(self.robots,pool.map(lambda robot:robot.state(),self.robots.values())))
  selected=dict(states[self.selected]);selected['robots']={str(i):{key:s.get(key) for key in ('robot_id','host','domain','domain_detected','video_port','connected','mode','running')} for i,s in states.items()}
  selected['simulation']=self.robots[11].sim is not None and self.robots[11].sim.poll() is None
  if self.robots[11].job.get('busy') and 'Simulation' in self.robots[11].job.get('message',''):selected['job']=dict(self.robots[11].job)
  return selected
 def connect(self,data):
  with self.lock:
   robot=self.for_request(data)
   if any(other is not robot and (other.host==robot.host or other.domain==robot.domain) for other in self.robots.values()):raise ValueError('Les adresses et les domaines ROS des deux robots doivent être différents.')
   return robot.connect()
 def action(self,path,data):
  with self.lock:
   robot=self.for_request(data)
   return robot.action(path,{key:value for key,value in data.items() if key!='robot'})
 def stop_all(self):
  from concurrent.futures import ThreadPoolExecutor
  def stop(robot):
   if not robot.demo and not robot.robot_token:return None
   try:robot.action('/stop',{});return (robot.robot_id,True)
   except Exception:return (robot.robot_id,False)
  with ThreadPoolExecutor(max_workers=2) as pool:results=[r for r in pool.map(stop,self.robots.values()) if r is not None]
  failed=[i for i,ok in results if not ok]
  simulation_stopped=False
  if self.mapping_sim is not None and self.mapping_sim.poll() is None:
   self.mapping_sim_request('/action',{'action':'stop'});simulation_stopped=True
  if failed:raise ValueError('Arrêt non confirmé pour le(s) robot(s) : '+', '.join(map(str,failed)))
  return {'success':bool(results) or simulation_stopped,'message':'Arrêt confirmé sur les robots connectés.' if results else 'Simulation arrêtée ; aucun robot connecté.' if simulation_stopped else 'Aucun robot connecté : aucun arrêt matériel confirmé.'}
 def start_mapping_sim(self):
  if self.mapping_sim is not None and self.mapping_sim.poll() is None:return {'success':True}
  if subprocess.run(['docker','image','inspect','urmc-slam:local'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:raise ValueError('Construire le banc SLAM avec le script tools/preparer_slam_mac.command.')
  self.mapping_token=secrets.token_urlsafe(32)
  self.mapping_sim=subprocess.Popen(['docker','run','--rm','--name','urmc-slam-console-'+str(os.getpid()),'--label','urmc.console=true','-p','127.0.0.1:8894:8093','-e','URMC_MAP_TOKEN='+self.mapping_token,'-v',str(ROOT)+':/project:ro','-v','urmc-cartes-simulation:/ws/cartes','urmc-slam:local','bash','/project/tools/container_slam.sh'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  return {'success':True,'message':'Banc SLAM en préparation. Aucune connexion robot.'}
 def mapping_sim_request(self,path,data=None):
  if not self.mapping_token:raise ValueError('Démarrer le banc SLAM du Mac.')
  req=urllib.request.Request('http://127.0.0.1:8894'+path,data=json.dumps(data).encode() if data is not None else None,headers={'Authorization':'Bearer '+self.mapping_token,'Content-Type':'application/json'})
  try:
   with urllib.request.urlopen(req,timeout=18 if data else 2) as r:return json.load(r)
  except urllib.error.HTTPError as e:
   try:message=json.load(e)['error']
   except Exception:message='Erreur du banc SLAM'
   raise ValueError(message) from None
 def mapping_sim_export(self,name):
  import urllib.parse
  req=urllib.request.Request('http://127.0.0.1:8894/export?name='+urllib.parse.quote(name),headers={'Authorization':'Bearer '+str(self.mapping_token)})
  with urllib.request.urlopen(req,timeout=8) as r:return r.read()
 def stop_mapping_sim(self):
  if self.mapping_sim is not None and self.mapping_sim.poll() is None:
   subprocess.run(['docker','stop','-t','4','urmc-slam-console-'+str(os.getpid())],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=8)
  self.mapping_sim=None;return {'success':True}
 def start_sim(self,mission):return self.robots[11].start_sim(mission)
 def stop_sim(self):return self.robots[11].stop_sim()
 def close(self):
  self.stop_mapping_sim();self.stop_sim()
  for robot in self.robots.values():robot.close_tunnels()

def make_handler(backend,csrf):
 class Handler(BaseHTTPRequestHandler):
  def respond(self,body,mime='application/json',status=200,headers=None):
   if not isinstance(body,bytes):body=json.dumps(body).encode()
   self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)))
   for key,value in (headers or {}).items():self.send_header(key,value)
   self.end_headers()
   try:self.wfile.write(body)
   except (BrokenPipeError,ConnectionResetError):pass
  def do_GET(self):
   path=self.path.split('?')[0]
   if path=='/':return self.respond((ROOT/'tools/console/index.html').read_text().replace('__TOKEN__',csrf).encode(),'text/html; charset=utf-8')
   if path in ('/app.js','/mapping.js','/style.css'):
    return self.respond((ROOT/'tools/console'/path[1:]).read_bytes(),'text/javascript; charset=utf-8' if path.endswith('js') else 'text/css; charset=utf-8')
   if path=='/api/state':return self.respond(backend.state())
   if path=='/api/mapping/frame':
    import urllib.request
    try:
     req=urllib.request.Request('http://127.0.0.1:8894/camera',headers={'Authorization':'Bearer '+str(backend.mapping_token)})
     with urllib.request.urlopen(req,timeout=1) as r:return self.respond(r.read(),'image/jpeg')
    except Exception:return self.respond({'error':'Caméra simulée indisponible'},status=503)
   if path in ('/api/mapping','/api/mapping/export'):
    import urllib.parse
    query=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
    try:
     robot=backend.get_robot(query.get('robot',[backend.selected])[0])
     if query.get('simulation',['0'])[0]=='1':
      if path.endswith('/export'):return self.respond(backend.mapping_sim_export(query.get('name',[''])[0]),'application/zip')
      return self.respond(backend.mapping_sim_request('/snapshot'))
     if path.endswith('/export'):
      req=urllib.request.Request(f'http://127.0.0.1:{robot.agent_port}/mapping/export?name='+urllib.parse.quote(query.get('name',[''])[0]),headers={'Authorization':'Bearer '+str(robot.robot_token)})
      with urllib.request.urlopen(req,timeout=6) as r:return self.respond(r.read(),'application/zip')
     return self.respond(robot.request('/mapping',timeout=5))
    except Exception as e:return self.respond({'error':str(e)},status=400)
   if path=='/api/lidar':
    import urllib.parse,math
    query=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
    try:robot=backend.get_robot(query.get('robot',[backend.selected])[0]) if isinstance(backend,Fleet) else backend
    except ValueError as exc:return self.respond({'error':str(exc)},status=400)
    if robot.demo:return self.respond(dict(robot_id=robot.robot_id,fresh=True,age=0,demo=True,points=[[a/20,.6] for a in range(-35,-25)],right=.6,front=3.5,right_start=-120,right_end=-20,right_threshold=.8,lane_forward=.75,traffic=None))
    if not robot.connected:return self.respond({'error':'Robot non connecté'},status=503)
    try:
     with urllib.request.urlopen(f'http://127.0.0.1:{robot.video_port}/lidar.json',timeout=.8) as r:data=json.load(r)
     data['robot_id']=robot.robot_id;return self.respond(data)
    except Exception:return self.respond({'error':'LiDAR indisponible'},status=503)
   if path=='/api/frame':
    import urllib.parse
    query=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
    try:robot=backend.get_robot(query.get('robot',[backend.selected])[0]) if isinstance(backend,Fleet) else backend
    except ValueError as exc:return self.respond({'error':str(exc)},status=400)
    if robot.demo:
     example=ROOT/'tools/console/examples'/('robot_'+str(robot.robot_id)+'.jpg')
     if example.exists():return self.respond(example.read_bytes(),'image/jpeg',headers={'X-Robot-ID':str(robot.robot_id),'X-Robot-IP':robot.host})
     svg='<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360"><rect width="100%" height="100%" fill="#172e33"/><path d="M140 360 L270 80 M500 360 L370 80" stroke="#e7bb40" stroke-width="8"/><text x="320" y="190" text-anchor="middle" fill="white" font-size="24">DEMONSTRATION - ROBOT '+str(robot.robot_id)+'</text></svg>'
     return self.respond(svg.encode(),'image/svg+xml',headers={'X-Robot-ID':str(robot.robot_id),'X-Robot-IP':robot.host})
    if not robot.connected:return self.respond({'error':'Robot non connecté'},status=503)
    try:
     with urllib.request.urlopen(f'http://127.0.0.1:{robot.video_port}/frame.jpg',timeout=1) as r:
      return self.respond(r.read(),'image/jpeg',headers=dict({key:r.headers.get(key,'') for key in ('X-Frame-Sequence','X-Source-Age','X-Mission')},**{'X-Robot-ID':str(robot.robot_id),'X-Robot-IP':robot.host}))
    except Exception:return self.respond({'error':'Caméra indisponible'},status=503)
   if path=='/api/document':
    import urllib.parse
    key=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('id',[''])[0]
    if key=='guide':return self.respond(b'Utiliser OUVRIR URMC.command, puis Connecter, choisir un algorithme et Demarrer. STOP reste toujours disponible.','text/plain; charset=utf-8')
    files=document_files()
    if key.isdigit() and int(key)<len(files):return self.respond(files[int(key)].read_bytes(),'application/pdf')
    return self.respond({'error':'Document indisponible'},status=404)
   self.respond({'error':'Page inconnue'},status=404)
  def do_POST(self):
   origin=self.headers.get('Origin');allowed={'http://127.0.0.1:'+str(self.server.server_port),'http://localhost:'+str(self.server.server_port)}
   if self.headers.get('Host') not in {x.split('://')[1] for x in allowed} or (origin and origin not in allowed) or not secrets.compare_digest(self.headers.get('X-Console-Token',''),csrf):return self.respond({'error':'Accès refusé'},status=403)
   try:
    length=int(self.headers.get('Content-Length','0'))
    if not 0<length<=8192:raise ValueError('Requête invalide.')
    data=json.loads(self.rfile.read(length))
    if not isinstance(data,dict):raise ValueError('Objet attendu.')
    if self.path=='/api/mapping-sim/start':result=backend.start_mapping_sim()
    elif self.path=='/api/mapping-sim/stop':result=backend.stop_mapping_sim()
    elif self.path=='/api/mapping' and data.pop('simulation',False):result=backend.mapping_sim_request('/action',{k:v for k,v in data.items() if k!='robot'})
    elif self.path=='/api/ssh':
     robot=backend.for_request(data) if isinstance(backend,Fleet) else backend
     command=shlex.join(['ssh','-o','IPQoS=none',robot.target])
     script='tell application "Terminal"\nactivate\ndo script '+json.dumps(command)+'\nend tell'
     subprocess.run(['osascript','-e',script],check=True,timeout=8);result={'success':True}
    elif self.path=='/api/select':result=backend.select(data.get('selected'))
    elif self.path=='/api/robot/configure':result=backend.configure(data)
    elif self.path=='/api/stop-all':result=backend.stop_all()
    elif self.path=='/api/connect':result=backend.connect(data) if isinstance(backend,Fleet) else backend.connect()
    elif self.path in ('/api/start','/api/stop','/api/manual','/api/settings','/api/mapping'):result=backend.action(self.path[4:],data)
    elif self.path=='/api/simulation/start':result=backend.start_sim(data.get('mission'))
    elif self.path=='/api/simulation/stop':result=backend.stop_sim()
    else:return self.respond({'error':'Action inconnue'},status=404)
    self.respond(result)
   except (OSError,ValueError,TypeError,subprocess.SubprocessError) as exc:self.respond({'error':str(exc)},status=400)
  def log_message(self,*_):pass
 return Handler

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true');parser.add_argument('--no-open',action='store_true');parser.add_argument('--tab',choices=['drive','missions','mapping','simulation','settings','help'],default='drive');parser.add_argument('--mission',type=int,choices=range(1,6),default=1);args=parser.parse_args()
 url=f'http://127.0.0.1:{PORT}/?tab={args.tab}&mission={args.mission}'
 backend=Fleet(args.demo);csrf=secrets.token_urlsafe(32)
 try:server=ThreadingHTTPServer(('127.0.0.1',PORT),make_handler(backend,csrf))
 except OSError:
  try:
   with urllib.request.urlopen(f'http://127.0.0.1:{PORT}/api/state',timeout=1) as response:existing=json.load(response)
   if 'schema' not in existing or 'demo' not in existing:raise ValueError()
  except Exception:raise SystemExit('Le port 8090 est occupé par une autre application. Aucun processus n’a été arrêté.')
  print('La console est déjà ouverte : http://127.0.0.1:8090/')
  if not args.no_open:subprocess.run(['open',url])
  return
 if not args.no_open:subprocess.run(['open','-a','Google Chrome',url])
 print(f'Console URMC : http://127.0.0.1:{PORT}/ — aucun mouvement au démarrage.',flush=True)
 def stop(*_):
  try:
   backend.stop_all()
  except Exception:pass
  threading.Thread(target=server.shutdown,daemon=True).start()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
 try:server.serve_forever()
 finally:
  backend.close();server.server_close()
if __name__=='__main__':main()
