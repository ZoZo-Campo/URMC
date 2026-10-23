"""Five real ROS launches: wait, start, pause, restart, resume without odometry jumps."""
import json,os,signal,subprocess,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pathlib import Path

def main():
    rclpy.init();p=Node('controls_probe');states={};truth={};procs=[];files=[];clients={}
    Path('/ws/results').mkdir(exist_ok=True)
    def wait(predicate,timeout=15):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            rclpy.spin_once(p,timeout_sec=.03)
            if predicate():return
        raise AssertionError(str((states,truth)))
    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(p,timeout_sec=.02)
    def action(name):
        futures=[]
        for i in range(1,6):
            c=clients[i,name];assert c.wait_for_service(timeout_sec=3)
            futures.append(c.call_async(Trigger.Request()))
        wait(lambda:all(f.done() for f in futures))
        assert all(f.result().success for f in futures)
        spin_for(.4)
    try:
        for i in range(1,6):
            ns=f'/urmc_sim/m{i:02d}'
            p.create_subscription(String,ns+'/status',lambda m,i=i:states.update({i:json.loads(m.data)}),1)
            p.create_subscription(String,ns+'/sim_ground_truth',lambda m,i=i:truth.update({i:json.loads(m.data)}),1)
            for a in ('start','pause','restart'):clients[i,a]=p.create_client(Trigger,ns+'/sim/'+a)
            log=open(f'/ws/results/controls-{i}.log','w');files.append(log)
            procs.append(subprocess.Popen(['ros2','launch',f'urmc_mission_{i:02d}','sim.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        wait(lambda:len(truth)==5 and len(states)==5 and all(s['fresh'] for s in states.values()))
        spin_for(1.)
        assert all(t['t']==0 for t in truth.values()) and all(not s['armed'] and s['v']==0 for s in states.values())
        print('OK: cinq simulations attendent Démarrer',flush=True)
        action('start');wait(lambda:all(t['t']>3 for t in truth.values()))
        assert all(s['armed'] for s in states.values())
        action('pause');snap={i:(t['t'],t['x'],t['y']) for i,t in truth.items()}
        spin_for(1.3)
        assert all((t['t'],t['x'],t['y'])==snap[i] for i,t in truth.items())
        assert all(s['v']==0 and not s['armed'] for s in states.values())
        print('OK: cinq pauses figent position, commandes et temps',flush=True)
        action('restart')
        wait(lambda:all(t['t']==0 and t['session']['epoch']==1 for t in truth.values()))
        assert all(s['state']=='WAIT' and not s['armed'] for s in states.values())
        assert all(abs(t['x']-(.2 if i==4 else 0))<1e-8 for i,t in truth.items())
        action('start');wait(lambda:all(t['t']>1 and states[i]['armed'] for i,t in truth.items()))
        print('OK: cinq remises à zéro et nouveaux départs, sans saut odométrique bloquant',flush=True)
        assert not p.get_publishers_info_by_topic('/cmd_vel')
    finally:
        for proc in procs:
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
        for proc in procs:
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for f in files:f.close()
        p.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
