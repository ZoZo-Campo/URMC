"""Supervised camera-only track following. Stop at checker, lost vision, or 90 seconds."""
import time,math,json,signal
from std_srvs.srv import Trigger
from std_msgs.msg import String
from urmc_core.vision import analyze
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2,numpy as np
cv2.setNumThreads(1)
def lane(data):
 image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
 if image is None:return None
 image=cv2.resize(image,(320,240));hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
 def center(lo,hi):
  mask=cv2.inRange(hsv,lo,hi)[156:230,:]
  count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
  choices=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=40 and stats[i,cv2.CC_STAT_HEIGHT]>=20]
  if not choices:return None
  return float(centroids[max(choices,key=lambda i:stats[i,cv2.CC_STAT_AREA]),0])
 yellow=center((18,90,90),(38,255,255));white=center((0,0,185),(179,60,255))
 if yellow is None or white is None or not 35<white-yellow<290:return None
 return ((yellow+white)/2-160)/160
class Trial(Node):
 def __init__(self):
  super().__init__('urmc_camera_track_follow');self.pub=self.create_publisher(Twist,'/cmd_vel',1)
  self.error=None;self.received=0;self.valid=0;self.started=None;self.created=time.monotonic();self.done=False
  self.create_subscription(CompressedImage,'/image_raw/compressed',self.image,qos_profile_sensor_data)
  self.header_age=None;self.visual_pause=None;self.finish_count=0;self.last_stamp=None;self.last_log=0;self.speed=0.;self.yaw_rate=0.
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
   self.error=lane(m.data)
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
   if now-self.created>10:return self.stop('ligne/camera non disponible')
   if not fresh or self.valid<3:return
   self.started=now;print('DEPART : suivi camera, maximum 0.05 m/s, arret au damier ou 90 secondes; erreur='+str(round(self.error,3)),flush=True)
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
  target=.05/(1+3*abs(self.error));turn=max(-.25,min(.25,-.8*self.error))
  self.speed+=max(-.005,min(.0015,target-self.speed))
  self.yaw_rate+=max(-.025,min(.025,turn-self.yaw_rate))
  cmd=Twist();cmd.linear.x=self.speed;cmd.angular.z=self.yaw_rate;self.pub.publish(cmd)
  state=dict(state='FOLLOW',elapsed=round(now-self.started,1),error=round(self.error,3),v=round(self.speed,3),w=round(self.yaw_rate,3))
  self.status.publish(String(data=json.dumps(state)))
  if now-self.last_log>3:print(json.dumps(state),flush=True);self.last_log=now
def main():
 rclpy.init();n=Trial()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda signum,frame:n.stop('interruption'))
 try:
  while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.1)
 except KeyboardInterrupt:pass
 finally:
  for _ in range(6):n.pub.publish(Twist());time.sleep(.05)
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
