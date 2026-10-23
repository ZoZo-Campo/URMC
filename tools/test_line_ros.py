"""Real ROS launch regression for M1 obstacle bypass and camera-derived recovery."""
import json,os,signal,subprocess,time
import rclpy
from std_msgs.msg import String

def main():
 rclpy.init();node=rclpy.create_node('line_obstacle_probe');states=[];latest={}
 def receive(msg):
  latest.update(json.loads(msg.data))
  if not states or states[-1]!=latest['state']:states.append(latest['state']);print(latest['state'],flush=True)
 node.create_subscription(String,'/urmc_sim/m01/status',receive,10)
 log=open('/tmp/line_obstacle_ros.log','w')
 proc=subprocess.Popen(['ros2','launch','urmc_mission_01','sim.launch.py','autostart:=true','obstacle:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  deadline=time.monotonic()+110
  while time.monotonic()<deadline and latest.get('state')!='DONE':
   rclpy.spin_once(node,timeout_sec=.1)
   assert proc.poll() is None,'ROS launch exited'
  assert latest.get('state')=='DONE',latest
  assert 'BYPASS' in states and 'REJOIN' in states,states
  assert not node.get_publishers_info_by_topic('/cmd_vel')
  print('OK: ROS 2 obstacle → détour → réalignement → suivi → damier.',flush=True)
 finally:
  if proc.poll() is None:proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
