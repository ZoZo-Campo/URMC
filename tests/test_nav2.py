"""Nav2 configuration and cancellation protect the supervised motor gateway."""
import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from nav2_bridge import parameters,Nav2Bridge
from urmc_core.operator_settings import DEFAULTS

def test_nav2_profiles_use_live_lidar_and_metric_clearance():
 p=parameters(DEFAULTS,lambda n:'/test/'+n)
 for name in ('global_costmap','local_costmap'):
  c=p[name][name]['ros__parameters']
  assert c['robot_radius']>=DEFAULTS['m4_radius']+DEFAULTS['m4_margin']-1e-8
  if name=='local_costmap':assert 'static_layer' not in c['plugins']
  if name=='global_costmap':assert c['robot_radius']>=DEFAULTS['m4_radius']+DEFAULTS['m4_margin']+.029
  assert c['static_layer']['map_topic']=='/test/urmc/nav2/map'
  assert c['obstacle_layer']['scan']['clearing'] and c['obstacle_layer']['scan']['marking']
  assert c['obstacle_layer']['scan']['topic']=='/test/scan'
 assert p['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']<p['controller_server']['ros__parameters']['FollowPath']['lookahead_dist']
 assert p['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
 assert p['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']==DEFAULTS['m4_speed']

def test_cancel_blocks_delayed_velocity_and_goal_acceptance():
 bridge=object.__new__(Nav2Bridge);bridge.handle=None;bridge.generation=3;bridge.cmd=(.1,.2)
 bridge.cancel();assert bridge.cmd==(0.,0.) and bridge.target is None
 bridge.velocity(SimpleNamespace(linear=SimpleNamespace(x=.1),angular=SimpleNamespace(z=.2)))
 assert bridge.cmd==(0.,0.) and bridge.received==0
 canceled=[];handle=SimpleNamespace(accepted=True,cancel_goal_async=lambda:canceled.append(True))
 bridge.followed(SimpleNamespace(result=lambda:handle),3)
 assert canceled==[True] and bridge.handle is None

def test_controller_abort_stops_and_retries_only_three_times():
 bridge=object.__new__(Nav2Bridge);bridge.generation=1;bridge.retries=0;bridge.error=None
 for number in (1,2,3):
  bridge.cmd=(.1,.2);bridge.received=123.;bridge.handle=object();bridge.pending=True
  bridge.finished(SimpleNamespace(result=lambda:SimpleNamespace(status=6)),1)
  assert bridge.retries==number and bridge.cmd==(0.,0.) and bridge.received==0.
  assert bridge.handle is None and not bridge.pending
  assert bool(bridge.error)==(number==3)


def test_robot_launch_omits_empty_prefix_argument():
 from nav2_bridge import launch_command
 assert launch_command('launch.py','params.yaml','')==['ros2','launch','launch.py','params_file:=params.yaml']
 assert launch_command('launch.py','params.yaml','/sim')[-1]=='prefix:=/sim'
