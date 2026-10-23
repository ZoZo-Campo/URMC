"""ROS integration: camera watchdog, explicit rearming, and robot launch disarmed."""
import json,os,signal,subprocess,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

def main():
    rclpy.init();probe=Node('fault_probe');state={};proc=None
    probe.create_subscription(String,'/urmc_sim/m01/status',lambda m:state.update(json.loads(m.data)),1)
    def wait(predicate,seconds=8):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(probe,timeout_sec=.03)
            if predicate():return
        raise AssertionError('Timeout, dernier état: '+str(state))
    def stop():
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    try:
        with open('/ws/results/faults.log','w') as log:
            proc=subprocess.Popen(['ros2','launch','urmc_mission_01','sim.launch.py','autostart:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            wait(lambda:state.get('v',0)>.01)
            subprocess.run(['ros2','param','set','/urmc_sim/m01/simulator','drop_camera_after','0.0'],check=True)
            wait(lambda:state.get('fresh') is False and state.get('armed') is False and state.get('v')==0)
            print('OK: perte caméra → vitesse nulle et désarmement',flush=True)
            subprocess.run(['ros2','param','set','/urmc_sim/m01/simulator','drop_camera_after','-1.0'],check=True)
            wait(lambda:state.get('fresh') is True)
            assert not state['armed'] and state['v']==0,'Redémarrage automatique interdit'
            client=probe.create_client(SetBool,'/urmc_sim/m01/enable');assert client.wait_for_service(timeout_sec=3)
            future=client.call_async(SetBool.Request(data=True));wait(lambda:future.done());assert future.result().success
            wait(lambda:state.get('v',0)>.01);print('OK: reprise uniquement après réarmement',flush=True)
            stop();state.clear()
            probe.create_subscription(String,'/urmc/m01/status',lambda m:state.update(json.loads(m.data)),1)
            proc=subprocess.Popen(['ros2','launch','urmc_mission_01','robot.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            wait(lambda:state.get('mission')==1)
            assert state['armed'] is False and state['v']==0
            client=probe.create_client(SetBool,'/urmc/m01/enable');assert client.wait_for_service(timeout_sec=3)
            future=client.call_async(SetBool.Request(data=True));wait(lambda:future.done());assert not future.result().success
            print('OK: lancement robot désarmé, armement refusé sans capteurs',flush=True)
    finally:
        stop();probe.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
