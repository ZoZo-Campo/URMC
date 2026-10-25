"""Real ROS 2 publishers/subscribers, simulated camera/scan/odom, no hardware topic."""
import sys,time,math,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mission_priorite import PriorityNode
from urmc_core.sim import scene


def robot_scan(x,y):
    ranges=[]
    for i in range(720):
        a=-math.pi+i*math.pi/360
        if abs(math.sin(a))<1e-6:ranges.append(math.inf);continue
        r=y/math.sin(a)
        ranges.append(r if r>.05 and abs(r*math.cos(a)-x)<.09 else math.inf)
    return ranges


def main():
    rclpy.init(args=['--ros-args','-r','/cmd_vel:=/urmc_test/priority/cmd_vel',
                    '-r','/scan:=/urmc_test/priority/scan','-r','/odom:=/urmc_test/priority/odom',
                    '-r','/image_raw/compressed:=/urmc_test/priority/image/compressed'])
    actor=PriorityNode(True);sensor=Node('priority_test_sensors');executor=SingleThreadedExecutor()
    executor.add_node(actor);executor.add_node(sensor)
    image_pub=sensor.create_publisher(CompressedImage,'/urmc_test/priority/image/compressed',qos_profile_sensor_data)
    scan_pub=sensor.create_publisher(LaserScan,'/urmc_test/priority/scan',qos_profile_sensor_data)
    odom_pub=sensor.create_publisher(Odometry,'/urmc_test/priority/odom',qos_profile_sensor_data)
    commands=[];sensor.create_subscription(Twist,'/urmc_test/priority/cmd_vel',lambda m:commands.append((m.linear.x,m.angular.z)),1)
    x=y=yaw=0.;states=[];yield_time=None;seen_block=False;crossed_without_lines=False;finished=False
    start=last=time.monotonic();last_sensor=0.;waiting_since=None;pause_done=False
    try:
        while time.monotonic()-start<52:
            now=time.monotonic();dt=min(.05,now-last);last=now
            if commands:
                v,w=commands[-1];yaw+=w*dt;x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt
            if now-last_sensor>.05:
                elapsed=now-start;state=actor.brain.state
                if state=='YIELD' and yield_time is None:yield_time=elapsed
                age=elapsed-yield_time if yield_time is not None else -1
                # Target approaches along the right-hand road, pauses, then clears.
                if 0<=age<3.3:target_y=-.66+min(age,2.)*.08;ranges=robot_scan(.2,target_y)
                else:ranges=[math.inf]*720
                stamp=sensor.get_clock().now().to_msg()
                scan=LaserScan();scan.header.stamp=stamp;scan.angle_min=-math.pi;scan.angle_increment=math.pi/360;scan.range_min=.05;scan.range_max=3.5;scan.ranges=ranges;scan_pub.publish(scan)
                # Introduce a short camera outage in CROSS and verify STOP, then recovery.
                if state=='CROSS' and x>.6 and not pause_done:
                    if waiting_since is None:waiting_since=now
                    if now-waiting_since>.65:pause_done=True
                send_image=waiting_since is None or pause_done
                if send_image:
                    image=scene(x,y,yaw,3,elapsed)
                    msg=CompressedImage();msg.header.stamp=stamp;msg.format='jpeg';msg.data=cv2.imencode('.jpg',image)[1].tobytes();image_pub.publish(msg)
                odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2);odom_pub.publish(odom)
                last_sensor=now
            executor.spin_once(timeout_sec=.005)
            if not states or states[-1]!=actor.brain.state:states.append(actor.brain.state)
            if actor.traffic and actor.traffic.get('blocked') and actor.brain.state=='YIELD':
                seen_block=True;assert actor.v==0
            if waiting_since and not pause_done and now-waiting_since>.4:assert actor.v==0,'Camera outage did not stop'
            if actor.brain.state=='CROSS' and .6<x<1.0 and actor.v>0:crossed_without_lines=True
            if actor.brain.state=='FOLLOW_AFTER_INTERSECTION' and actor.v>0:finished=True;break
            assert not actor.done,actor.brain.reason
        assert finished,(states,x,actor.brain.reason)
        assert seen_block and crossed_without_lines and pause_done,(seen_block,crossed_without_lines,pause_done)
        assert abs(y)<.08 and all(0<=v<=.05001 and abs(w)<=.251 for v,w in commands)
        assert '/cmd_vel' not in dict(sensor.get_topic_names_and_types())
        actor.stop('fin test');assert actor.v==0
        print('OK : cible approchant de droite, attente, depart apres disparition confirmee, traversee sans lignes au cap, pause camera et reprise, recuperation des deux bordures ; aucun moteur reel. '+json.dumps(states))
    finally:
        actor.destroy_node();sensor.destroy_node();executor.shutdown();rclpy.shutdown()
if __name__=='__main__':main()
