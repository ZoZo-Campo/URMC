"""Hold-to-drive ROS worker. No command => zero within 250 ms."""
if __name__=='__main__':
 from operator_bootstrap import bootstrap
 bootstrap(__file__)
import fcntl,json,math,queue,signal,sys,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from urmc_core.operator_settings import current

class ManualNode(Node):
 def __init__(self):
  super().__init__('urmc_manual_operator')
  self.pub=self.create_publisher(Twist,'/cmd_vel',1)
  self.status=self.create_publisher(String,'/urmc/operator/status',1)
  self.create_service(Trigger,'/urmc/operator/stop',self.stop_service)
  self.deadline=0.;self.v=0.;self.w=0.;self.hold=None;self.expired=None;self.sequence=-1;self.done=False;self.created=time.monotonic();self.reported=0.
  self.create_timer(.05,self.tick)
 def stop_service(self,request,response):
  self.stop();response.success=True;response.message='Pilotage manuel arrêté';return response
 def stop(self):
  self.v=self.w=0.;self.pub.publish(Twist());self.done=True
 def command(self,data):
  if not isinstance(data,dict) or not isinstance(data.get('seq'),int) or data['seq']<=self.sequence:return
  self.sequence=data['seq']
  if data.get('active') is not True:
   self.v=self.w=0.;self.hold=None;self.pub.publish(Twist());return
  hold=data.get('hold')
  if not isinstance(hold,str) or not hold or hold==self.expired:return
  x,y=data.get('x'),data.get('yaw')
  if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>1 for v in (x,y)):return
  self.hold=hold;self.v=float(x)*current()['speed'];self.w=float(y)*.4;self.deadline=time.monotonic()+.25
 def tick(self):
  now=time.monotonic()
  if self.done:return
  if self.count_publishers(self.pub.topic_name)!=1:
   self.v=self.w=0.;self.pub.publish(Twist())
   if now-self.created>.8:
    print(json.dumps(dict(state='FAULT',reason='Autre commande moteur détectée')),flush=True);self.stop()
   return
  if now>self.deadline:
   if self.hold is not None:self.expired=self.hold
   self.v=self.w=0.
  cmd=Twist();cmd.linear.x=self.v;cmd.angular.z=self.w;self.pub.publish(cmd)
  if now-self.reported>.2:
   data=dict(state='MANUAL',mode='manuel',v=self.v,w=self.w,held=self.hold is not None and now<=self.deadline)
   self.status.publish(String(data=json.dumps(data)));print(json.dumps(data),flush=True);self.reported=now

def main():
 workspace=Path(__file__).resolve().parents[1];locks=[]
 for name in ('.camera_follow.lock','.barrier.lock'):
  handle=(workspace/name).open('w')
  try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
  except BlockingIOError:raise SystemExit('Une mission est déjà active. STOP avant le mode manuel.')
  locks.append(handle)
 rclpy.init();node=ManualNode();commands=queue.Queue(maxsize=20)
 def read():
  for line in sys.stdin:
   try:
    data=json.loads(line)
    if commands.full():commands.get_nowait()
    commands.put_nowait(data)
   except (ValueError,queue.Empty,queue.Full):pass
  commands.put(None)
 threading.Thread(target=read,daemon=True).start()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:node.stop())
 try:
  while rclpy.ok() and not node.done:
   for _ in range(20):
    try:data=commands.get_nowait()
    except queue.Empty:break
    if data is None:node.stop();break
    node.command(data)
   rclpy.spin_once(node,timeout_sec=.02)
 finally:
  for _ in range(4):node.pub.publish(Twist());time.sleep(.05)
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
