"""Manual ROS worker: timeout, release and old hold must always publish zero."""
import time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"kit_robot"))
import rclpy
from teleop_robot import ManualNode

def test_deadman_never_restarts_an_expired_hold():
 rclpy.init(args=['--ros-args','-r','/cmd_vel:=/urmc_test/manual/cmd_vel'])
 node=ManualNode()
 try:
  node.command(dict(seq=1,active=True,hold='first',x=1,yaw=0));assert node.v>0
  node.deadline=time.monotonic()-.1;node.tick();assert node.v==0
  node.command(dict(seq=2,active=True,hold='first',x=1,yaw=0));assert node.v==0
  node.command(dict(seq=3,active=False));assert node.v==0
  node.command(dict(seq=4,active=True,hold='second',x=-1,yaw=0));assert node.v<0
  node.command(dict(seq=5,active=False));assert node.v==0
  node.command(dict(seq=4,active=True,hold='old',x=1,yaw=0));assert node.v==0
  assert node.pub.topic_name=='/urmc_test/manual/cmd_vel'
  assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
 finally:node.stop();node.destroy_node();rclpy.shutdown()


def test_manual_motion_release_and_timeout_reach_only_the_test_topic():
 from rclpy.node import Node
 from rclpy.executors import SingleThreadedExecutor
 from geometry_msgs.msg import Twist
 rclpy.init(args=['--ros-args','-r','/cmd_vel:=/urmc_test/manual/cmd_vel'])
 actor=ManualNode();observer=Node('manual_test_observer');executor=SingleThreadedExecutor();executor.add_node(actor);executor.add_node(observer);commands=[]
 observer.create_subscription(Twist,'/urmc_test/manual/cmd_vel',lambda msg:commands.append((msg.linear.x,msg.angular.z)),10)
 sequence=0
 def exercise(seconds,direction=None,hold='held'):
  nonlocal sequence
  end=time.monotonic()+seconds;sent=0.
  while time.monotonic()<end:
   now=time.monotonic()
   if direction is not None and now-sent>.06:
    sequence+=1;actor.command(dict(seq=sequence,active=True,hold=hold,x=direction,yaw=0));sent=now
   executor.spin_once(timeout_sec=.01)
 try:
  exercise(.3);exercise(.4,1)
  assert any(v>0 for v,w in commands[-6:]),'the ROS subscriber must receive forward commands'
  sequence+=1;actor.command(dict(seq=sequence,active=False));exercise(.12)
  assert commands[-1]==(0.,0.)
  exercise(.25,-1,'reverse');assert any(v<0 for v,w in commands[-5:])
  exercise(.4);assert commands[-1]==(0.,0.),'missing heartbeats must stop motion'
  exercise(.2,-1,'reverse');assert all(v==0 for v,w in commands[-3:]),'an expired hold cannot restart motion'
  assert '/cmd_vel' not in dict(observer.get_topic_names_and_types())
 finally:
  actor.stop();executor.remove_node(actor);executor.remove_node(observer);actor.destroy_node();observer.destroy_node();executor.shutdown();rclpy.shutdown()
