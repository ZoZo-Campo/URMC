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
