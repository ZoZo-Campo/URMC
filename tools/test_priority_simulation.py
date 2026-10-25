"""Exercise the shipped mission-3 ROS launch, holding and releasing right traffic."""
import json,os,signal,subprocess,time
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from urmc_core.vision import analyze


def main():
    rclpy.init();node=Node('priority_launch_probe');state={};history=[];vision={};moving_without_lines=False
    def status(msg):
        value=json.loads(msg.data);state.update(value)
        if not history or history[-1]!=value['state']:history.append(value['state'])
    def image(msg):vision.update(analyze(cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)))
    node.create_subscription(String,'/urmc_sim/m03/status',status,10)
    node.create_subscription(CompressedImage,'/urmc_sim/m03/image_raw/compressed',image,qos_profile_sensor_data)
    client=node.create_client(SetParameters,'/urmc_sim/m03/simulator/set_parameters')
    with open('/tmp/priority-launch.log','w') as log:
        proc=subprocess.Popen(['ros2','launch','urmc_mission_03','sim.launch.py','autostart:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        def hold(value):
            assert client.wait_for_service(timeout_sec=3)
            param=Parameter(name='right_occupied',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value))
            future=client.call_async(SetParameters.Request(parameters=[param]));rclpy.spin_until_future_complete(node,future,timeout_sec=3)
            assert future.result() and future.result().results[0].successful
        start=time.monotonic();yield_start=None;released=False
        try:
            while time.monotonic()-start<46:
                rclpy.spin_once(node,timeout_sec=.03)
                assert proc.poll() is None,'ROS launch ended prematurely'
                if state.get('state')=='YIELD' and yield_start is None:hold(True);yield_start=time.monotonic()
                if yield_start and not released:
                    assert state.get('v',0)==0,'Right traffic did not hold the robot'
                    if time.monotonic()-yield_start>6:hold(False);released=True
                if state.get('state')=='CROSS' and state.get('v',0)>0 and vision.get('lane_valid') is False:moving_without_lines=True
                if state.get('state')=='DONE':break
            assert state.get('state')=='DONE',(history,state)
            assert released and moving_without_lines and 'YIELD' in history and 'CROSS' in history
            assert not any(name=='/cmd_vel' for name,_ in node.get_topic_names_and_types())
            print('OK : lancement ROS mission 3, cible droite maintenue 6 s, liberation, traversee de la zone sans lignes et fin. '+json.dumps(history))
        finally:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
