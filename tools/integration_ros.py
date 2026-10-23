"""Exercise actual launch files, processes, DDS, images, scans and Twist feedback."""
import json, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def main():
    rclpy.init();probe=Node('integration_probe');procs=[];files=[];states={};truth={};history={i:[] for i in range(1,6)};violations=[]
    output=Path('/ws/results');output.mkdir(exist_ok=True)
    def status(i,m):
        data=json.loads(m.data);states[i]=data
        if not history[i] or history[i][-1]!=data['state']:
            history[i].append(data['state']);print(f'M{i}: {data["state"]}',flush=True)
    def cmd(i,m):
        if not 0<=m.linear.x<=.10001 or abs(m.angular.z)>.70001:violations.append(f'M{i} vitesse hors limite')
    for i in range(1,6):
        probe.create_subscription(String,f'/urmc_sim/m{i:02d}/status',lambda m,i=i:status(i,m),10)
        probe.create_subscription(String,f'/urmc_sim/m{i:02d}/sim_ground_truth',lambda m,i=i:truth.update({i:json.loads(m.data)}),10)
        probe.create_subscription(Twist,f'/urmc_sim/m{i:02d}/cmd_vel',lambda m,i=i:cmd(i,m),10)
    try:
        for i in range(1,6):
            f=(output/f'm{i:02d}.log').open('w');files.append(f)
            procs.append(subprocess.Popen(['ros2','launch',f'urmc_mission_{i:02d}','sim.launch.py','autostart:=true'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
        start=time.monotonic()
        while time.monotonic()-start<120:
            rclpy.spin_once(probe,timeout_sec=.05)
            if len(states)==5 and all(s['state']=='DONE' for s in states.values()):break
            if any(p.poll() is not None for p in procs):raise AssertionError('Un lancement ROS a quitté prématurément')
        summary=dict(states=states,truth=truth,history=history,violations=violations,elapsed=time.monotonic()-start)
        (output/'integration.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2),flush=True)
        assert len(states)==5 and all(s['state']=='DONE' for s in states.values()),'Certaines missions ne terminent pas le scénario'
        assert not violations
        camera_subs=probe.get_subscriptions_info_by_topic('/urmc_sim/m04/image_raw/compressed')
        assert not any(s.node_name=='mission' for s in camera_subs),'Caméra interdite en mission 4'
        assert not probe.get_publishers_info_by_topic('/cmd_vel'),'Simulation publiée sur le topic matériel'
        assert 'WAIT_BARRIER' in history[2] and 'YIELD' in history[3]
        assert 'INSIDE' in history[4] and 'GRIP' in history[5] and 'RELEASE' in history[5]
        assert all(abs(truth[i]['y'])<.08 for i in (2,3)), 'Sortie de piste nominale'
    finally:
        for p in procs:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in procs:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in files:f.close()
        probe.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
