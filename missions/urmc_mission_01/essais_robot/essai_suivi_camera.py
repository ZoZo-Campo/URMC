"""Supervised five-second camera-only test. No obstacle detection, no search."""
import time,math,json
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
  super().__init__('urmc_camera_short_trial');self.pub=self.create_publisher(Twist,'/cmd_vel',1)
  self.error=None;self.received=0;self.valid=0;self.started=None;self.created=time.monotonic();self.done=False
  self.create_subscription(CompressedImage,'/image_raw/compressed',self.image,qos_profile_sensor_data)
  self.create_timer(.05,self.tick)
 def stop(self,why):
  self.pub.publish(Twist());self.done=True;print('ARRET : '+why,flush=True)
 def image(self,m):
  age=self.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9
  if not 0<=age<.25:self.error=None;self.valid=0;return
  try:self.error=lane(m.data)
  except Exception:self.error=None
  self.received=time.monotonic();self.valid=self.valid+1 if self.error is not None else 0
 def tick(self):
  if self.done:return
  now=time.monotonic()
  if now-self.created<1:return
  if self.count_publishers('/cmd_vel')!=1:return self.stop('autre commande moteur detectee')
  fresh=self.error is not None and now-self.received<.25
  if self.started is None:
   self.pub.publish(Twist())
   if now-self.created>10:return self.stop('ligne/camera non disponible')
   if not fresh or self.valid<3:return
   self.started=now;print('DEPART : suivi camera, maximum 0.03 m/s, 5 secondes; erreur='+str(round(self.error,3)),flush=True)
  if now-self.started>=5:return self.stop('fin des 5 secondes')
  if not fresh:return self.stop('ligne perdue ou camera perimee')
  if abs(self.error)>.65:return self.stop('piste trop decalee')
  cmd=Twist();cmd.linear.x=.03/(1+2*abs(self.error));cmd.angular.z=max(-.12,min(.12,-.6*self.error));self.pub.publish(cmd)
def main():
 rclpy.init();n=Trial()
 try:
  while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.1)
 except KeyboardInterrupt:pass
 finally:
  for _ in range(6):n.pub.publish(Twist());time.sleep(.05)
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
