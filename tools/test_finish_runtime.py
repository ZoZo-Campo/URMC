"""Exercise actual camera pilots with synthetic compressed signs and remapped motors."""
import sys,time,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mission_barriere import BarrierNode
from suivi_piste_camera import Trial
from urmc_core.sim import scene


def main():
 for classic,target in [(False,'stop'),(False,'checker'),(True,'stop')]:
  rclpy.init(args=['--ros-args','-r','/cmd_vel:=/urmc_test/finish/cmd_vel','-r','/image_raw/compressed:=/urmc_test/finish/image','-r','/odom:=/urmc_test/finish/odom'])
  actor=Trial(target) if classic else BarrierNode(True,0.,True,target)
  sensor=Node('finish_test_sensors');executor=SingleThreadedExecutor();executor.add_node(actor);executor.add_node(sensor)
  images=sensor.create_publisher(CompressedImage,'/urmc_test/finish/image',qos_profile_sensor_data);odom=sensor.create_publisher(Odometry,'/urmc_test/finish/odom',qos_profile_sensor_data);commands=[]
  sensor.create_subscription(Twist,'/urmc_test/finish/cmd_vel',lambda m:commands.append(m.linear.x),1)
  def exercise(kind,seconds):
   end=time.monotonic()+seconds;last=0.
   while time.monotonic()<end:
    now=time.monotonic()
    if now-last>.055:
     im=scene(0,0,0,1,0)
     if kind=='red':cv2.circle(im,(260,90),25,(0,0,240),-1)
     elif kind=='stop':
      pts=np.array([(260+26*math.cos(a),90+26*math.sin(a)) for a in np.arange(8)*math.pi/4+math.pi/8],np.int32)
      cv2.fillPoly(im,[pts],(0,0,235));cv2.putText(im,'STOP',(239,95),0,.25,(255,255,255),1)
     elif kind=='checker':
      for yy in range(180,224,12):
       for xx in range(90,234,18):cv2.rectangle(im,(xx,yy),(xx+17,yy+11),(240,)*3 if ((xx-90)//18+(yy-180)//12)%2 else (10,)*3,-1)
     stamp=sensor.get_clock().now().to_msg();msg=CompressedImage();msg.header.stamp=stamp;msg.format='jpeg';msg.data=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,55])[1].tobytes();images.publish(msg)
     o=Odometry();o.header.stamp=stamp;o.pose.pose.orientation.w=1.;odom.publish(o);last=now
    executor.spin_once(timeout_sec=.01)
  try:
   exercise('clear',1.6);assert commands[-1]>0,actor.done
   exercise('red',.5);assert not actor.done # a red blob is not STOP
   exercise('checker' if target=='stop' else 'stop',.6);assert not actor.done
   exercise(target,.7);assert actor.done and actor.finish_event['kind']==target
   assert commands[-1]==0 and ('/cmd_vel' not in dict(sensor.get_topic_names_and_types()))
   print('PASS finish', 'classic' if classic else 'robust',target,flush=True)
  finally:
   actor.destroy_node();sensor.destroy_node();executor.shutdown();rclpy.shutdown()
if __name__=='__main__':main()
