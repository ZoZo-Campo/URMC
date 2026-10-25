"""Supervised camera-only track following. Stop at checker, lost vision, or 90 seconds."""
# Works both from the mission folder and from the stable kit_robot entry point.
from pathlib import Path as _LayoutPath
import sys as _layout_sys
_WORKSPACE=next((p for p in _LayoutPath(__file__).resolve().parents if (p/'kit_robot').is_dir() and (p/'src').is_dir()),None)
if _WORKSPACE is None:raise RuntimeError('Workspace URMC introuvable autour du script.')
_layout_sys.path.insert(0,str(_WORKSPACE/'kit_robot'))
# Standalone entry point: ROS environment is loaded before any ROS import.
import os,sys,subprocess,fcntl
from pathlib import Path

def load_environment():
 script=Path(__file__).resolve()
 if os.environ.get('URMC_CAMERA_BOOTSTRAPPED')==str(script):return
 workspace=next((p for p in script.parents if (p/'install/setup.bash').is_file()),None)
 if workspace is None:raise SystemExit('Workspace non compile : install/setup.bash introuvable autour du script.')
 env=os.environ.copy();env['URMC_CAMERA_BOOTSTRAPPED']=str(script)
 env['URMC_CAMERA_WORKSPACE']=str(workspace)
 env['ROS_DOMAIN_ID']=env.get('URMC_ROBOT_DOMAIN','41');env['ROS_LOCALHOST_ONLY']='0'
 env['TURTLEBOT3_MODEL']='burger';env['LDS_MODEL']='LDS-01'
 # Positional arguments keep paths out of shell interpolation.
 command='set -e; source /opt/ros/humble/setup.bash; if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi; source "$1"; exec /usr/bin/python3 "$2" "${@:3}"'
 os.execvpe('bash',['bash','-c',command,'urmc',str(workspace/'install/setup.bash'),str(script),*sys.argv[1:]],env)

if __name__=='__main__':load_environment()

def prepare_drivers():
 workspace=Path(os.environ['URMC_CAMERA_WORKSPACE']);logs=workspace/'session_robot11';logs.mkdir(exist_ok=True)
 def active(kind):
  found=[]
  for item in Path('/proc').iterdir():
   if not item.name.isdigit():continue
   try:
    if item.stat().st_uid!=os.getuid():continue
    args=item.joinpath('cmdline').read_bytes().decode().split('\0')
    names=[Path(a).name for a in args if a]
    match=(kind=='base' and ('turtlebot3_ros' in names or ('turtlebot3_bringup' in args and 'robot.launch.py' in args))) or (kind=='camera' and 'v4l2_camera_node' in names) or (kind=='video' and 'retour_video.py' in names)
    if not match:continue
    variables=dict(x.split('=',1) for x in item.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
    if variables.get('ROS_DOMAIN_ID','0')!=os.environ['ROS_DOMAIN_ID']:
     raise RuntimeError(kind+' deja actif dans un autre domaine ROS ; arreter cette ancienne instance avant de relancer.')
    found.append(item.name)
   except (FileNotFoundError,ProcessLookupError):continue
  return found
 commands=[('base',['ros2','launch','turtlebot3_bringup','robot.launch.py']),('camera',['ros2','run','v4l2_camera','v4l2_camera_node','--ros-args','-p','image_size:=[320,240]'])]
 video=workspace/'kit_robot/retour_video.py'
 if not video.exists():video=workspace/'outils/retour_video.py'
 if video.exists():commands.append(('video',['/usr/bin/python3',str(video),'--topic','/image_raw/compressed','--overlay','auto']))
 for name,args in commands:
  if active(name):print(name+' : deja actif',flush=True);continue
  with (logs/(name+'.log')).open('a') as stream:
   proc=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
  (logs/(name+'.pid')).write_text(str(proc.pid))
  print(name+' : demarrage',flush=True)


import time,math,json,signal
from std_srvs.srv import Trigger
from std_msgs.msg import String
from urmc_core.vision import analyze
from urmc_core.operator_settings import current as settings
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2,numpy as np
cv2.setNumThreads(1)
class LaneTracker:
 """Store the geometry of the last accepted pair for diagnostics."""
 def __init__(self):self.width=160.;self.sides={};self.history={};self.last_accepted=None

def lane_details(data,tracker=None):
 """Both borders are required; color does not dictate their left/right position."""
 tracker=tracker if tracker is not None else LaneTracker()
 image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
 if image is None:return dict(error=None,reason='image indecodable')
 cfg=settings()
 hsv=cv2.cvtColor(cv2.resize(image,(320,240)),cv2.COLOR_BGR2HSV)
 attempts=[]
 for top,bottom in ((156,230),(130,200),(110,180)):
  roi=hsv[top:bottom]
  neutral=roi[:,:,2][roi[:,:,1]<=cfg["white_s"]]
  white_v=max(cfg["white_floor"],min(185,int(np.percentile(neutral,99)*.88))) if neutral.size else 185
  previous=tracker.history.get((top,bottom)) or tracker.last_accepted
  recent=previous is not None and 0<=time.monotonic()-previous[0]<.5
  areas={}
  def center(lo,hi,key):
   mask=cv2.inRange(roi,lo,hi)
   count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
   choices=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=40
     and stats[i,cv2.CC_STAT_HEIGHT]>=20 and stats[i,cv2.CC_STAT_WIDTH]>=3
     and stats[i,cv2.CC_STAT_WIDTH]<2.2*stats[i,cv2.CC_STAT_HEIGHT]]
   if recent:
    reference=previous[1][key]
    choices=[i for i in choices if abs(centroids[i,0]-reference[0])<=48]
   if not choices:return None
   # Long thin markings outrank wide textured floor patches.
   i=max(choices,key=lambda i:stats[i,cv2.CC_STAT_HEIGHT]**2/max(3,stats[i,cv2.CC_STAT_WIDTH]))
   areas[key]=int(stats[i,cv2.CC_STAT_AREA])
   return [float(centroids[i,0]),float(centroids[i,1]+top)]
  yellow=center((cfg["yellow_low"],90,65),(cfg["yellow_high"],255,255),"yellow");white=center((0,0,white_v),(179,cfg["white_s"],255),"white")
  result=dict(error=None,reason='aucune bordure',yellow_center=yellow,white_center=white,
      yellow=yellow[0] if yellow else None,white=white[0] if white else None,roi=[top,bottom],white_v=white_v,areas=areas)
  points={key:value for key,value in [('yellow',yellow),('white',white)] if value is not None}
  if len(points)==2 and 35<abs(white[0]-yellow[0])<290:
   measured_width=abs(white[0]-yellow[0])
   if recent and not .65*previous[2]<=measured_width<=1.6*previous[2]:
    result['reason']='saut de largeur rejete';attempts.append(result);continue
   tracker.history[(top,bottom)]=(time.monotonic(),points,measured_width)
   tracker.last_accepted=tracker.history[(top,bottom)]
   tracker.width=measured_width
   tracker.sides={key:('left' if value[0]<sum(p[0] for p in points.values())/2 else 'right') for key,value in points.items()}
   target=[(yellow[0]+white[0])/2,(yellow[1]+white[1])/2]
   result.update(error=(target[0]-160)/160,reason='deux bordures',target_center=target,source='both')
   return result
  if len(points)==1:result['reason']='deux bordures requises : '+('blanc absent' if yellow is not None else 'jaune absente')
  elif len(points)==2:result['reason']='ecartement des bordures invalide'
  attempts.append(result)
 return attempts[0]

def lane(data):
 return lane_details(data)['error']

class Trial(Node):
 def __init__(self):
  super().__init__('urmc_camera_track_follow');self.pub=self.create_publisher(Twist,'/cmd_vel',1)
  self.tracker=LaneTracker();self.error=None;self.received=0;self.valid=0;self.started=None;self.created=time.monotonic();self.done=False
  self.create_subscription(CompressedImage,'/image_raw/compressed',self.image,qos_profile_sensor_data)
  self.last_start_log=0.;self.vision_reason='attente images';self.header_age=None;self.visual_pause=None;self.finish_count=0;self.last_stamp=None;self.last_log=0;self.speed=0.;self.yaw_rate=0.
  self.status=self.create_publisher(String,'/urmc/camera_follow/status',1)
  self.create_service(Trigger,'/urmc/camera_follow/stop',self.request_stop)
  self.create_timer(.05,self.tick)
 def request_stop(self,req,res):
  self.stop('arret demande');res.success=True;res.message='Robot arrete';return res
 def stop(self,why):
  self.pub.publish(Twist());self.done=True;self.status.publish(String(data=json.dumps(dict(state='STOP',reason=why,v=0,w=0))));print('ARRET : '+why,flush=True)
 def image(self,m):
  age=self.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9
  self.header_age=age
  if not 0<=age<.25:self.error=None;self.valid=0;return
  try:
   detection=lane_details(m.data,self.tracker);self.error=detection['error'];self.vision_reason=detection['reason']
   stamp=(m.header.stamp.sec,m.header.stamp.nanosec)
   if stamp!=self.last_stamp:
    image=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
    self.finish_count=self.finish_count+1 if analyze(image)['checker'] else 0
    self.last_stamp=stamp
  except Exception:self.error=None;self.finish_count=0
  self.received=time.monotonic();self.valid=self.valid+1 if self.error is not None else 0
 def tick(self):
  if self.done:return
  now=time.monotonic()
  if now-self.created<1:return
  if self.count_publishers('/cmd_vel')!=1:return self.stop('autre commande moteur detectee')
  fresh=self.error is not None and now-self.received<.25
  if self.finish_count>=3:return self.stop('damier confirme sur trois images')
  if self.started is None:
   self.pub.publish(Twist())
   if now-self.created>30:return self.stop('aucune image recente' if self.header_age is None or now-self.received>.25 else self.vision_reason)
   if now-self.last_start_log>3:
    print('Attente : '+('aucune image recue' if self.header_age is None else 'image perimee' if now-self.received>.25 else self.vision_reason)+' ; images valides consecutives='+str(self.valid),flush=True);self.last_start_log=now
   if not fresh or self.valid<3:return
   self.started=now;print('DEPART : suivi camera, vitesse maximale '+str(settings()['speed'])+' m/s, arret au damier ou 90 secondes; erreur='+str(round(self.error,3)),flush=True)
  if now-self.started>=90:return self.stop('limite de 90 secondes atteinte')
  if now-self.received>=.25 or self.header_age is None or not 0<=self.header_age<.25:
   return self.stop('camera perimee : age reception='+str(round(now-self.received,3))+' age image='+str(self.header_age))
  if self.error is None:
   self.speed=0.;self.yaw_rate=0.;self.pub.publish(Twist())
   if self.visual_pause is None:self.visual_pause=now;print('PAUSE : verification visuelle',flush=True)
   if now-self.visual_pause>.6:return self.stop('bordures perdues plus de 0.6 seconde')
   return
  if self.visual_pause is not None:
   self.pub.publish(Twist())
   if self.valid<3:return
   self.visual_pause=None;print('REPRISE : trois images valides',flush=True)
  if abs(self.error)>.65:return self.stop('piste trop decalee')
  cfg=settings();target=cfg["speed"]/(1+3*abs(self.error));turn=max(-.25,min(.25,-cfg["steering_gain"]*self.error))
  self.speed+=max(-.005,min(.0015,target-self.speed))
  self.yaw_rate+=max(-.025,min(.025,turn-self.yaw_rate))
  cmd=Twist();cmd.linear.x=self.speed;cmd.angular.z=self.yaw_rate;self.pub.publish(cmd)
  state=dict(state='FOLLOW',elapsed=round(now-self.started,1),error=round(self.error,3),v=round(self.speed,3),w=round(self.yaw_rate,3))
  self.status.publish(String(data=json.dumps(state)))
  if now-self.last_log>3:print(json.dumps(state),flush=True);self.last_log=now
def diagnose_camera():
 """Read-only integrated readiness check: never creates a motor publisher."""
 rclpy.init();node=rclpy.create_node('urmc_camera_diagnostic')
 state=dict(images=0,valides=0,fraiches=0,reason='aucune image')
 def receive(msg):
  age=node.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
  result=lane_details(msg.data)
  state['images']+=1;state['fraiches']+=int(0<=age<.25)
  state['valides']+=int(result['error'] is not None)
  state['reason']=result['reason'];state['age']=round(age,3)
 node.create_subscription(CompressedImage,'/image_raw/compressed',receive,qos_profile_sensor_data)
 print('Diagnostic camera seul : aucune commande moteur.',flush=True)
 try:
  start=time.monotonic()
  while time.monotonic()-start<8:rclpy.spin_once(node,timeout_sec=.1)
  print(json.dumps(state),flush=True)
  if not state['images'] or state['valides']/state['images']<.98 or state['fraiches']/state['images']<.98:
   raise RuntimeError('Camera ou detection instable : '+state['reason'])
  print('Camera et bordures disponibles : OK',flush=True)
 finally:node.destroy_node();rclpy.shutdown()

def main():
 if sys.argv[1:]==['--check']:
  print('Environnement ROS charge : domaine '+os.environ['ROS_DOMAIN_ID']+' ; imports OK ; aucun pilote ni moteur lance.');return
 if sys.argv[1:] not in ([],['--prepare-only'],['--diagnostic']):raise SystemExit('Usage : python3 suivi_piste_camera.py [--check|--prepare-only|--diagnostic]')
 lock_path=Path(os.environ['URMC_CAMERA_WORKSPACE'])/'.camera_follow.lock'
 lock=lock_path.open('w')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit('Un suivi est deja en cours. Aucun second lancement.')
 prepare_drivers()
 if sys.argv[1:]==['--prepare-only']:
  print('Pilotes prepares. Aucun controleur moteur lance.');return
 if sys.argv[1:]==['--diagnostic']:diagnose_camera();return
 print('Attente de la camera et des bordures (30 s maximum). Un seul essai sera lance.',flush=True)
 rclpy.init();n=Trial()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda signum,frame:n.stop('interruption'))
 try:
  while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.1)
 except KeyboardInterrupt:pass
 finally:
  for _ in range(6):n.pub.publish(Twist());time.sleep(.05)
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
