"""Real ROS loop: no SLAM/map TF, chosen arrival, live obstacle and camera opt-in."""
import sys,time,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import numpy as np,rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sim_enclosure_slam import Arena,PREFIX
from mission_enceinte import EnclosureNode


def main():
    rclpy.init(args=['--ros-args','-r','/tf:=/urmc_test/navigation_modes/tf','-r','/tf_static:=/urmc_test/navigation_modes/tf_static'])
    arena=Arena();actor=EnclosureNode(simulation=True);executor=SingleThreadedExecutor();executor.add_node(arena);executor.add_node(actor)
    commands=[];arena.create_subscription(Twist,PREFIX+'/cmd_vel',lambda m:commands.append((m.linear.x,m.angular.z)),1)
    actor.ros=lambda fn,timeout=4:fn()
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
    try:
        spin(1.);actor.start_local();spin(.8)
        assert actor.mode=='local' and actor.slam_process is None and actor.camera_sub is None
        goal=[1.55,0.];actor.set_route(dict(map_id=actor.map_id,key='exit',point=goal))
        arena.set_parameters([Parameter('obstacles',value=True)]);spin(.5)
        actor.run_navigation(actor.map_id,'goal');start=time.monotonic();detour=False;last=0
        while time.monotonic()-start<58:
            executor.spin_once(timeout_sec=.005)
            if actor.pose and abs(actor.pose[1])>.18:detour=True
            if time.monotonic()-last>6:print(actor.phase,actor.pose,actor.reason,flush=True);last=time.monotonic()
            if actor.phase in ('DONE','FAULT','TIMEOUT'):break
        assert actor.phase=='DONE',(actor.phase,actor.pose,actor.reason)
        assert detour and math.dist(actor.pose[:2],goal)<.08 and actor.route['exit']==goal
        assert actor.slam_process is None and not actor.get_publishers_info_by_topic('/map')
        assert not actor.get_publishers_info_by_topic('/cmd_vel')
        assert commands and all(0<=v<=.1001 and abs(w)<=.451 for v,w in commands)
        before=actor.grid.data.copy();arena.set_parameters([Parameter('obstacles',value=False)]);spin(.7)
        assert np.count_nonzero((before>=50)&(actor.grid.data==0))>2
        actor.set_camera({'enabled':True});spin(.6);assert actor.camera_state()['fresh'] and actor.camera_state()['allowed']
        actor.set_camera({'enabled':False});assert actor.camera_sub is None
        print('OK : sans SLAM ni TF map, arrivée cliquée conservée, obstacle ajouté contourné puis retiré de la vue, caméra uniquement sur activation, aucun moteur matériel.',flush=True)
    finally:
        actor.shutdown();actor.destroy_node();arena.destroy_node();executor.shutdown();rclpy.shutdown()
if __name__=='__main__':main()
