"""Localhost-only robot supervisor; launches only explicitly listed programs."""
import argparse,fcntl,json,os,secrets,signal,subprocess,sys,threading,time,urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/urmc_core'))
from urmc_core.operator_settings import DEFAULTS,SCHEMA,validate
from mapping_manager import MappingManager
PORT=8091
ALGORITHMS={
 'follow-robust':('mission_barriere.py','--run','--ignore-barrier'),
 'follow-classic':('suivi_piste_camera.py',),
 'barrier':('mission_barriere.py','--run'),
 'priority':('mission_priorite.py','--run'),
 'priority-observe':('mission_priorite.py',),
 'observe':('mission_barriere.py',),
 'manual':('teleop_robot.py',),
}

class Controller:
 def __init__(self,root=ROOT,popen=subprocess.Popen):
  self.root=Path(root);self.popen=popen;self.lock=threading.RLock();self.process=None;self.mode='idle';self.lease=None;self.sequence=-1;self.generation=-1;self.last={};self.events=deque(maxlen=60)
  self.directory=self.root/'session_robot11';self.directory.mkdir(exist_ok=True)
  self.mapping=MappingManager(self.root)
  self.settings_path=self.directory/'reglages.json'
  if not self.settings_path.exists():self.save_settings(DEFAULTS)
 def settings(self):return validate(json.loads(self.settings_path.read_text()))
 def save_settings(self,values):
  values=validate(values);temp=self.settings_path.with_suffix('.tmp');temp.write_text(json.dumps(values,indent=2));temp.replace(self.settings_path);return values
 def stop(self):
  with self.lock:
   self.lease=None;self.sequence=-1
   self.mapping.stop(required=self.mode=='enclosure')
   p=self.process
   if p is not None and p.poll() is None:
    if self.mode=='manual':
     try:p.stdin.write(json.dumps({'seq':2**53,'active':False})+'\n');p.stdin.flush()
     except (OSError,ValueError):pass
    try:p.send_signal(signal.SIGINT)
    except ProcessLookupError:pass
    try:p.wait(timeout=2)
    except subprocess.TimeoutExpired:
     p.terminate()
     try:p.wait(timeout=1)
     except subprocess.TimeoutExpired:p.kill();p.wait(timeout=1)
   self.process=None;self.mode='idle';self.last={'state':'STOP','v':0,'w':0}
   return {'success':True,'message':'Mission arrêtée. Vidéo disponible.'}
 def start(self,mode,map_id=None):
  if mode not in ALGORITHMS and mode!='enclosure':raise ValueError('Algorithme inconnu.')
  with self.lock:
   self.stop()
   # Respect locks also held by programs launched from an external SSH terminal.
   for name in ('.camera_follow.lock','.barrier.lock'):
    with (self.root/name).open('w') as h:
     try:fcntl.flock(h,fcntl.LOCK_EX|fcntl.LOCK_NB)
     except BlockingIOError:raise ValueError('Une mission externe tourne. L’arrêter avant de changer de mode.')
   if mode=='enclosure':
    self.mapping.ensure();result=self.mapping.request('/action',dict(action='run',map_id=map_id));self.mode=mode;return result
   env=os.environ.copy();env['URMC_SETTINGS_FILE']=str(self.settings_path)
   args=[sys.executable,'-u',str(self.root/'kit_robot'/ALGORITHMS[mode][0]),*ALGORITHMS[mode][1:]]
   self.process=self.popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,env=env)
   self.mode=mode;self.lease=secrets.token_urlsafe(20) if mode=='manual' else None;self.last={'state':'STARTING','v':0,'w':0}
   threading.Thread(target=self.read,args=(self.process,),daemon=True).start()
   return {'success':True,'lease':self.lease,'message':'Mode démarré : '+mode}
 def read(self,process):
  for line in process.stdout:
   line=line.strip()
   with self.lock:
    self.events.append({'time':time.strftime('%H:%M:%S'),'message':line[:1000]})
    if self.process is process:
     try:
      data=json.loads(line)
      if isinstance(data,dict):self.last=data
     except ValueError:
      if line.startswith('ARRET'):self.last={'state':'STOP','reason':line,'v':0,'w':0}
      elif line.startswith(('Attente','PAUSE')):self.last={'state':'WAIT_LANE','reason':line,'v':0,'w':0}
   log=self.directory/'console_evenements.log'
   with self.lock:
    if log.exists() and log.stat().st_size>2_000_000:log.replace(log.with_suffix('.previous.log'))
    with log.open('a') as stream:stream.write(line+'\n')
 def manual(self,data):
  with self.lock:
   if self.mode!='manual' or self.process is None or self.process.poll() is not None or not self.lease or not secrets.compare_digest(str(data.get('lease','')),self.lease):raise ValueError('Pilotage manuel inactif. Activer le mode manuel.')
   seq=data.get('seq')
   if isinstance(seq,bool) or not isinstance(seq,int) or seq<=self.sequence:raise ValueError('Commande ancienne refusée.')
   if data.get('active') is True:
    for key in ('x','yaw'):
     value=data.get(key)
     if isinstance(value,bool) or not isinstance(value,(int,float)) or not -1<=value<=1:raise ValueError('Direction invalide.')
    if not isinstance(data.get('hold'),str) or not data['hold']:raise ValueError('Maintien requis.')
   self.sequence=seq
   self.process.stdin.write(json.dumps({key:data.get(key) for key in ('seq','active','hold','x','yaw')})+'\n');self.process.stdin.flush()
   return {'success':True}
 def state(self):
  with self.lock:
   mapping=self.mapping.summary()
   running=bool(mapping.get('active')) if self.mode=='enclosure' else self.process is not None and self.process.poll() is None
   if self.mode=='enclosure':self.last=dict(mapping,state=mapping.get('phase','IDLE'))
   return dict(robot_id=int(os.environ.get('URMC_ROBOT_ID','11')),domain=int(os.environ.get('URMC_ROBOT_DOMAIN','41')),connected=True,mode=self.mode if running else 'idle',running=running,status=self.last,mapping=mapping,events=list(self.events),settings=self.settings(),schema=SCHEMA)


def handler(controller,token):
 class Handler(BaseHTTPRequestHandler):
  def respond(self,value,status=200):
   body=json.dumps(value).encode();self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers()
   try:self.wfile.write(body)
   except (BrokenPipeError,ConnectionResetError):pass
  def authorized(self):return secrets.compare_digest(self.headers.get('Authorization',''),'Bearer '+token)
  def do_GET(self):
   if not self.authorized():return self.respond({'error':'Accès refusé'},403)
   if self.path=='/state':return self.respond(controller.state())
   if self.path=='/mapping':
    try:controller.mapping.ensure();return self.respond(controller.mapping.request('/snapshot'))
    except (ValueError,OSError) as e:return self.respond({'error':str(e)},400)
   if self.path.startswith('/mapping/export?'):
    import urllib.parse
    try:
     name=urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('name',[''])[0]
     controller.mapping.ensure();token=(controller.directory/'.console_token').read_text().strip();req=urllib.request.Request('http://127.0.0.1:8093/export?name='+urllib.parse.quote(name),headers={'Authorization':'Bearer '+token})
     with urllib.request.urlopen(req,timeout=5) as r:body=r.read()
     self.send_response(200);self.send_header('Content-Type','application/zip');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
    except Exception as e:return self.respond({'error':str(e)},400)
   self.respond({'error':'Page inconnue'},404)
  def do_POST(self):
   if not self.authorized():return self.respond({'error':'Accès refusé'},403)
   try:
    n=int(self.headers.get('Content-Length','0'))
    if not 0<n<=8192:raise ValueError('Requête invalide.')
    data=json.loads(self.rfile.read(n))
    if not isinstance(data,dict):raise ValueError('Objet attendu.')
    if self.path in ('/start','/stop'):
     with controller.lock:
      generation=data.get('generation')
      if isinstance(generation,bool) or not isinstance(generation,int) or generation<=controller.generation:raise ValueError('Action ancienne refusée.')
      controller.generation=generation
      result=controller.start(data.get('algorithm'),data.get('map_id')) if self.path=='/start' else controller.stop()
    elif self.path=='/mapping':
     with controller.lock:
      if data.get('action') not in ('manual',):controller.stop()
      controller.mapping.ensure();result=controller.mapping.request('/action',data,timeout=15)
    elif self.path=='/manual':result=controller.manual(data)
    elif self.path=='/settings':result={'success':True,'settings':controller.save_settings(data)}
    else:return self.respond({'error':'Action inconnue'},404)
    self.respond(result)
   except (ValueError,TypeError,OSError) as exc:self.respond({'error':str(exc)},400)
  def log_message(self,*_):pass
 return Handler

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--ensure',action='store_true');args=parser.parse_args()
 directory=ROOT/'session_robot11';directory.mkdir(exist_ok=True);token_file=directory/'.console_token'
 if not token_file.exists():token_file.write_text(secrets.token_urlsafe(32));token_file.chmod(0o600)
 token=token_file.read_text().strip()
 if args.ensure:
  req=urllib.request.Request(f'http://127.0.0.1:{PORT}/state',headers={'Authorization':'Bearer '+token})
  try:
   with urllib.request.urlopen(req,timeout=1):pass
  except Exception:
   with (directory/'console_serveur.log').open('a') as log:subprocess.Popen([sys.executable,str(Path(__file__).resolve())],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  print(token);return
 controller=Controller();server=ThreadingHTTPServer(('127.0.0.1',PORT),handler(controller,token))
 def shutdown(*_):controller.stop();threading.Thread(target=server.shutdown,daemon=True).start()
 for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,shutdown)
 try:server.serve_forever()
 finally:controller.stop();server.server_close()
if __name__=='__main__':main()
