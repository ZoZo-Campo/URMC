"""Supervisor -> stdin -> ROS velocity -> watchdog, isolated from real motors."""
import os,sys,time,tempfile,json,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import rclpy
from geometry_msgs.msg import Twist
from operator_agent import Controller
from urmc_core.operator_settings import DEFAULTS
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and os.environ.get('ROS_DOMAIN_ID')=='96'
rclpy.init();probe=rclpy.create_node('manual_pipeline_probe');seen=[]
probe.create_subscription(Twist,'/urmc_test/manual/cmd_vel',lambda m:seen.append((m.linear.x,m.angular.z)),1)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=.01)
def launch(args,**kw):return subprocess.Popen([*args,'--ros-args','-r','/cmd_vel:=/urmc_test/manual/cmd_vel'],**kw)
control=Controller(Path('/ws'),launch,power=lambda:dict(enabled=True))
try:
 lease=control.start('manual')['lease']
 for _ in range(100):
  spin(.1)
  if seen:break
 assert seen and all(v==w==0 for v,w in seen)
 def send(seq,hold,active=True):return control.manual(dict(lease=lease,seq=seq,hold=hold,active=active,x=1,yaw=0))
 send(1,'first');spin(.15);assert any(v>0 for v,w in seen[-3:])
 send(2,'first',False);spin(.12);assert seen[-1]==(0.,0.)
 send(3,'expired');spin(.15);assert seen[-1][0]>0
 spin(.35);assert seen[-1]==(0.,0.)
 send(4,'expired');spin(.12);assert seen[-1]==(0.,0.)
 send(5,'fresh');spin(.12);assert seen[-1][0]>0
 try:send(4,'stale');raise AssertionError('old sequence accepted')
 except ValueError:pass
 control.stop();spin(.25);assert seen[-1]==(0.,0.)
 assert not probe.count_publishers('/cmd_vel')
 print('OK : bail manuel, vitesse ROS remappée, relâchement, arrêt sur perte de maintien, refus des appuis expirés et STOP. Aucun moteur matériel.')
finally:control.stop();probe.destroy_node();rclpy.shutdown()
