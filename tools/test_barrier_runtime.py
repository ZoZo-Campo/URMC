"""ROS regression: experimental barrier node, all velocities remapped to test-only topic."""
import sys,time,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import cv2,rclpy,numpy as np
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from urmc_core.sim import scene
from mission_barriere import BarrierNode

def main():
    rclpy.init(args=['--ros-args','-r','/cmd_vel:=/urmc_test/barrier/cmd_vel'])
    actor=BarrierNode(True,0);actor.created-=10000;sensor=Node('barrier_test_sensors');executor=SingleThreadedExecutor();executor.add_node(actor);executor.add_node(sensor)
    image_pub=sensor.create_publisher(CompressedImage,'/image_raw/compressed',qos_profile_sensor_data)
    odom_pub=sensor.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
    commands=[];sensor.create_subscription(Twist,'/urmc_test/barrier/cmd_vel',lambda m:commands.append(m.linear.x),1)
    def phase(kind,seconds):
        start=time.monotonic();last=0
        while time.monotonic()-start<seconds:
            now=time.monotonic()
            if now-last>.05:
                image=scene(0,0,0,1,0)
                if kind=='CLOSED':cv2.rectangle(image,(40,115),(280,132),(240,70,20),-1)
                elif kind=='OPEN':cv2.rectangle(image,(35,55),(53,175),(240,70,20),-1)
                elif kind=='AMBIGUOUS':cv2.rectangle(image,(120,50),(170,100),(240,70,20),-1)
                elif kind=='LOST':image[:]=40
                if kind!='STALE':
                    m=CompressedImage();m.header.stamp=sensor.get_clock().now().to_msg();m.format='jpeg'
                    ok,jpeg=cv2.imencode('.jpg',image);m.data=jpeg.tobytes();image_pub.publish(m)
                odom=Odometry();odom.header.stamp=sensor.get_clock().now().to_msg();odom.pose.pose.orientation.w=1.;odom_pub.publish(odom);last=now
            executor.spin_once(timeout_sec=.01)
        assert not actor.done,actor.state
    try:
        phase('AMBIGUOUS',.4);assert commands[-1]==0
        phase('CLEAR',.6);assert commands[-1]>0,actor.state
        phase('CLOSED',1.3);assert actor.gate.closed_seen and commands and commands[-1]==0
        phase('OPEN',.6);assert commands[-1]==0
        phase('OPEN',.8);assert commands[-1]>0,actor.state
        phase('CLEAR',.4);assert commands[-1]>0 # continues after the barrier leaves view
        actor.distance+=.8
        phase('CLEAR',.3);assert commands[-1]>0 # no former 55 cm termination
        phase('AMBIGUOUS',.3);assert commands[-1]==0
        phase('OPEN',.5);assert commands[-1]>0 # confirmed opening survives a partial view, not a new closure
        phase('OPEN',.9);assert commands[-1]>0
        phase('CLOSED',.3);assert commands[-1]==0
        phase('OPEN',1.4);assert commands[-1]>0
        phase('LOST',.3);assert commands[-1]==0
        phase('OPEN',.6);assert commands[-1]>0
        phase('STALE',.4);assert commands[-1]==0
        phase('OPEN',.6);assert commands[-1]>0
        actor.error=.8;actor.lane_valid=0;actor.tick();assert not actor.done and actor.v==0 and actor.state=='WAIT_LANE_OFFSET'
        phase('OPEN',.7);assert commands[-1]>0
        assert all(0<=v<=.05 for v in commands),max(commands)
        assert actor.pub.topic_name=='/urmc_test/barrier/cmd_vel'
        assert '/cmd_vel' not in dict(sensor.get_topic_names_and_types()),'Test emitted a hardware command'
        actor.limit=1.;actor.tick();assert actor.done # explicit optional timeout still works
        print('OK: sans limite par defaut, limite explicite, suivi avant barriere, fermeture, ouverture stable, poursuite apres disparition et au-dela de 55 cm, ambiguite, refermeture, perte piste et camera ; aucune vitesse materielle.')
    finally:
        actor.destroy_node();sensor.destroy_node();executor.shutdown();rclpy.shutdown()
if __name__=='__main__':main()
