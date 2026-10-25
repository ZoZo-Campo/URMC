"""Bench only: synthetic enclosure, movable unseen obstacles, odometry and TF."""
import math,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,CompressedImage
from urmc_core.sim import ray_segment
from retour_video import Viewer,make_handler,PAGE
from http.server import ThreadingHTTPServer
import threading
PREFIX='/urmc_sim/m04_slam'
WALLS=[(0,-.65,1.6,-.65),(0,.65,1.6,.65),(0,-.65,0,-.25),(0,.25,0,.65),(1.6,-.65,1.6,-.25),(1.6,.25,1.6,.65),(-.8,-1.2,2.5,-1.2),(-.8,1.2,2.5,1.2),(-.8,-1.2,-.8,1.2),(2.5,-1.2,2.5,1.2)]
class Arena(Node):
 def __init__(self):
  super().__init__('enclosure_bench');self.x=-.4;self.y=0.;self.yaw=0.;self.v=self.w=0.;self.last=time.monotonic();self.command_time=0.
  self.declare_parameter('obstacles',False)
  self.scan=self.create_publisher(LaserScan,PREFIX+'/scan',qos_profile_sensor_data);self.odom=self.create_publisher(Odometry,PREFIX+'/odom',qos_profile_sensor_data);self.image=self.create_publisher(CompressedImage,PREFIX+'/image_raw/compressed',qos_profile_sensor_data)
  self.create_subscription(Twist,PREFIX+'/cmd_vel',self.command,1)
  self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
  t=TransformStamped();t.header.frame_id='base_footprint';t.child_frame_id='base_scan';t.transform.rotation.w=1.;self.static.sendTransform(t)
  self.create_timer(.05,self.tick)
 def command(self,msg):self.v=msg.linear.x;self.w=msg.angular.z;self.command_time=time.monotonic()
 def tick(self):
  now=time.monotonic();dt=min(.1,now-self.last);self.last=now
  if now-self.command_time>.25:self.v=self.w=0.
  self.x+=self.v*math.cos(self.yaw)*dt;self.y+=self.v*math.sin(self.yaw)*dt;self.yaw+=self.w*dt
  stamp=self.get_clock().now().to_msg();t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_footprint';t.transform.translation.x=self.x;t.transform.translation.y=self.y;t.transform.rotation.z=math.sin(self.yaw/2);t.transform.rotation.w=math.cos(self.yaw/2);self.tf.sendTransform(t)
  o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint';o.pose.pose.position.x=self.x;o.pose.pose.position.y=self.y;o.pose.pose.orientation=t.transform.rotation;self.odom.publish(o)
  walls=list(WALLS)
  if self.get_parameter('obstacles').value:walls +=[(.62,-.14,.85,-.14),(.62,.12,.85,.12),(.62,-.14,.62,.12),(.85,-.14,.85,.12)]
  m=LaserScan();m.header.stamp=stamp;m.header.frame_id='base_scan';m.angle_min=-math.pi;m.angle_increment=math.pi/360;m.angle_max=math.pi-m.angle_increment;m.range_min=.05;m.range_max=3.5
  m.ranges=[min(3.5,min(ray_segment(self.x,self.y,self.yaw-math.pi+i*math.pi/360,s) for s in walls)) for i in range(720)];self.scan.publish(m)
  im=np.full((240,320,3),45,np.uint8);cv2.putText(im,'SIMULATION MISSION 4',(22,55),0,.6,(230,230,230),1);cv2.putText(im,'Camera operateur uniquement',(15,90),0,.45,(80,220,150),1);cv2.putText(im,'x %.2f y %.2f'%(self.x,self.y),(30,160),0,.6,(220,200,60),1)
  c=CompressedImage();c.header.stamp=stamp;c.format='jpeg';c.data=cv2.imencode('.jpg',im)[1].tobytes();self.image.publish(c)
def main():
 rclpy.init();arena=Arena();viewer=Viewer(PREFIX+'/image_raw/compressed',False,'/unused','none');server=ThreadingHTTPServer(('127.0.0.1',8080),make_handler(viewer,PAGE));threading.Thread(target=server.serve_forever,daemon=True).start()
 from rclpy.executors import SingleThreadedExecutor
 executor=SingleThreadedExecutor();executor.add_node(arena);executor.add_node(viewer)
 try:executor.spin()
 except KeyboardInterrupt:pass
 finally:server.shutdown();arena.destroy_node();viewer.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
