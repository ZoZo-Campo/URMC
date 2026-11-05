"""Confirm OpenCR torque before arming a controller. Never publishes a velocity."""
import json,time

def check_power(timeout=5.):
 import rclpy
 from rclpy.qos import qos_profile_sensor_data
 from std_srvs.srv import SetBool
 from turtlebot3_msgs.msg import SensorState
 rclpy.init();node=rclpy.create_node('urmc_motor_power_check');state={}
 def receive(msg):state.update(enabled=bool(msg.torque),received=time.monotonic())
 sub=node.create_subscription(SensorState,'/sensor_state',receive,qos_profile_sensor_data)
 try:
  deadline=time.monotonic()+timeout
  while 'enabled' not in state and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
  if 'enabled' not in state:raise RuntimeError('Aucun état OpenCR reçu. Vérifier la base et sa connexion USB.')
  if node.count_publishers('/cmd_vel'):raise RuntimeError('Un autre contrôleur publie encore des vitesses. STOP avant de reprendre.')
  if state['enabled']:return dict(enabled=True,changed=False)
  client=node.create_client(SetBool,'/motor_power')
  if not client.wait_for_service(timeout_sec=max(.1,deadline-time.monotonic())):raise RuntimeError('Service /motor_power indisponible.')
  request=SetBool.Request();request.data=True;future=client.call_async(request)
  while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
  if not future.done() or not future.result().success:raise RuntimeError('Activation du couple moteur refusée ou non confirmée.')
  while time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.05)
   if state.get('enabled') and time.monotonic()-state.get('received',0)<.3:return dict(enabled=True,changed=True)
  raise RuntimeError('OpenCR ne confirme pas le couple moteur actif.')
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
 from operator_bootstrap import bootstrap
 bootstrap(__file__)
 try:print(json.dumps(check_power()),flush=True)
 except Exception as exc:raise SystemExit('Moteurs : '+str(exc))
