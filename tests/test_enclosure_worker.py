import json,sys,threading,time,urllib.request,urllib.error
from pathlib import Path
import pytest,rclpy
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from mission_enceinte import EnclosureNode,handler

@pytest.fixture
def actor(tmp_path):
 rclpy.init();node=EnclosureNode(tmp_path,True)
 try:yield node
 finally:node.shutdown();node.destroy_node();rclpy.shutdown()

def test_no_camera_subscription_or_motor_publisher_at_start(actor):
 assert actor.pub is None
 assert not actor.get_publishers_info_by_topic('/cmd_vel')
 assert not any('image' in name for name,_ in actor.get_topic_names_and_types())

def test_old_map_id_is_refused_before_arming(actor):
 actor.map_id='new-map'
 with pytest.raises(ValueError):actor.run_navigation('old-map')
 assert actor.pub is None and not actor.active

def test_incomplete_sensors_never_arm(actor):
 actor.map_id='map'
 with pytest.raises(ValueError):actor.run_navigation('map')
 assert not actor.active and actor.pub is None

def test_sixty_seconds_timeout_releases_motors(actor):
 actor.acquire_motors();actor.active=True;actor.phase='INSIDE';actor.entered=time.monotonic()-61
 actor.tick()
 assert actor.phase=='TIMEOUT' and actor.pub is None and actor.v==0 and not actor.active
 assert not actor.get_publishers_info_by_topic('/cmd_vel')

def test_stop_releases_the_same_locks_as_camera_and_manual(actor):
 import fcntl
 actor.acquire_motors()
 with (actor.root/'.camera_follow.lock').open('w') as f:
  with pytest.raises(BlockingIOError):fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
 actor.stop_navigation()
 with (actor.root/'.camera_follow.lock').open('w') as f:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
 assert actor.pub is None

def test_worker_http_requires_token(actor):
 server=ThreadingHTTPServer(('127.0.0.1',0),handler(actor,'secret'));threading.Thread(target=server.serve_forever,daemon=True).start()
 try:
  url='http://127.0.0.1:'+str(server.server_port)
  with pytest.raises(urllib.error.HTTPError) as e:urllib.request.urlopen(url+'/state')
  assert e.value.code==403
  req=urllib.request.Request(url+'/state',headers={'Authorization':'Bearer secret'})
  with urllib.request.urlopen(req) as response:assert json.load(response)['active'] is False
 finally:server.shutdown();server.server_close()


def test_timer_freezes_after_completion(actor):
 actor.active=True;actor.entered=time.monotonic()-10
 actor.stop_navigation('DONE','Sortie atteinte');actor.cache_snapshot(False);first=actor.snapshot()['remaining']
 time.sleep(.03);actor.cache_snapshot(False)
 assert 49<first<51 and actor.snapshot()['remaining']==first


def test_without_slam_initializes_from_odometry_and_never_starts_slam(actor,monkeypatch):
 actor.odom_pose=(1.,2.,.5);actor.odom_received=time.monotonic();monkeypatch.setattr(actor,'ros',lambda fn,timeout=4:fn())
 result=actor.start_local()
 assert result['success'] and actor.mode=='local' and actor.confirmed and actor.slam_process is None
 assert actor.local_origin==(1.,2.,.5) and actor.local_view is not None and actor.pub is None
 assert actor.zone==[-3.5,-3.5,3.5,3.5]


def test_direct_goal_requires_only_destination_but_a_full_mission_needs_three_points(actor,monkeypatch):
 import numpy as np
 from urmc_core.enclosure_map import Grid
 actor.grid=Grid(np.zeros((80,100)),.04,(-1.,-1.,0.));actor.zone=[-.5,-.5,2.5,2.];actor.map_id='local';actor.mode='local';actor.confirmed=True;actor.route={'exit':[1.,0.]};actor.pose=(0.,0.,0.)
 monkeypatch.setattr(actor,'sensors',lambda:(True,'test'))
 with pytest.raises(ValueError):actor.run_navigation('local','enclosure')
 actor.run_navigation('local','goal')
 assert actor.active and actor.phase=='GOAL' and actor.task=='goal' and actor.entered is not None


def test_camera_assistance_is_opt_in_and_releasable(actor):
 assert actor.camera_sub is None and not actor.camera_enabled and actor.camera_state()['allowed']
 actor.set_camera({'enabled':True});assert actor.camera_sub is not None and not actor.camera_state()['allowed']
 actor.set_camera({'enabled':False});assert actor.camera_sub is None and actor.camera_state()['allowed']


def test_no_slam_view_export_has_no_fake_pose_graph(actor,monkeypatch):
 import numpy as np
 from urmc_core.enclosure_map import Grid
 actor.grid=Grid(np.zeros((80,80)),.04,(-1.,-1.,0.));actor.mode='local';actor.map_id='view';actor.route={'exit':[1.,0.]};actor.zone=[-.8,-.8,2.,2.]
 result=actor.save_map({'map_id':'view','name':'vue_locale'})
 folder=actor.store.folder(result['name'])
 assert (folder/'map.pgm').exists() and not (folder/'slam.posegraph').exists()
 grid,meta=actor.store.load(result['name']);assert meta['navigation_mode']=='local'
 assert actor.store.list()[0]['has_graph'] is False


@pytest.mark.parametrize('layout',[None,dict(width=1.,depth=1.25,origin=[100.,100.,0.])])
def test_full_enclosure_uses_measured_map_not_indicative_dimensions(actor,monkeypatch,layout):
 import numpy as np
 from urmc_core.enclosure_map import Grid
 actor.grid=Grid(np.zeros((100,120)),.04,(-1.,-2.,0.));actor.zone=[-.7,-1.,2.,1.];actor.map_id='m';actor.mode='localization';actor.confirmed=True
 actor.route=dict(entry=[-.18,0.],inside=[.7,0.],exit=[1.58,0.]);actor.pose=(-.4,0,0);actor.enclosure=layout
 monkeypatch.setattr(actor,'sensors',lambda:(True,'test'))
 from types import SimpleNamespace
 actor.nav2=SimpleNamespace(ready=lambda:True,cancel=lambda:None,publish_map=lambda:None,report=lambda:{'ready':True},process=None)
 actor.run_navigation('m')
 assert actor.active and actor.phase=='ENTRY'


def test_indicative_frame_cannot_make_an_occupied_waypoint_safe(actor,monkeypatch):
 import numpy as np
 from urmc_core.enclosure_map import Grid
 data=np.zeros((100,120));data[45:55,35:50]=100
 actor.grid=Grid(data,.04,(-1.,-2.,0.));actor.zone=[-.7,-1.,2.,1.];actor.map_id='m';actor.mode='localization';actor.confirmed=True
 actor.route=dict(entry=[-.18,0.],inside=[.7,0.],exit=[1.58,0.]);actor.pose=(-.4,0,0);actor.enclosure=dict(width=1.,depth=1.25,origin=[0.,-.5,0.])
 monkeypatch.setattr(actor,'sensors',lambda:(True,'test'))
 with pytest.raises(ValueError):actor.run_navigation('m')
 assert actor.pub is None and not actor.active


def test_location_badge_accounts_for_robot_footprint(actor):
 import numpy as np
 from urmc_core.enclosure_map import Grid
 actor.grid=Grid(np.zeros((100,120)),.04,(-1.,-2.,0.));actor.enclosure=dict(width=1.,depth=1.25,origin=[0.,-.5,0.])
 for point,location in [((-.4,0,0),'outside'),((-.05,0,0),'boundary'),((.5,0,0),'inside')]:
  actor.pose=point;actor.cache_snapshot(True);assert actor.snapshot()['enclosure_location']==location


def test_enclosure_waits_for_measured_jambs_before_committing_to_entry(actor,monkeypatch):
 from sensor_msgs.msg import LaserScan
 from urmc_core.operator_settings import DEFAULTS
 import numpy as np
 from urmc_core.enclosure_map import Grid
 actor.grid=Grid(np.zeros((100,120)),.04,(-1.,-2.,0.));actor.zone=[-.7,-1.,2.,1.];actor.map_id='m';actor.mode='localization';actor.confirmed=True
 actor.route=dict(entry=[-.18,0.],inside=[.7,0.],exit=[1.58,0.]);actor.pose=(-.18,0,0)
 actor.raw_scan=LaserScan();actor.raw_scan.header.stamp.sec=1
 monkeypatch.setattr(actor,'sensors',lambda:(True,'test'))
 from types import SimpleNamespace
 actor.nav2=SimpleNamespace(ready=lambda:True,cancel=lambda:None,publish_map=lambda:None,report=lambda:{'ready':True},process=None)
 actor.run_navigation('m');actor.tick()
 assert actor.phase=='OPENING' and actor.v==0 and actor.entered is None
 # Empty scans or a map alone cannot claim that the two jambs were observed.
 actor.raw_scan.header.stamp.sec=2;actor.tick()
 assert actor.phase=='OPENING' and actor.v==0 and not actor.entrance.result['confirmed']


def test_tunnel_camera_detector_is_opt_in_stationary_and_never_claims_points(actor):
 import cv2
 from sensor_msgs.msg import CompressedImage
 from test_road_signs import tunnel_image
 assert actor.camera_sub is None and not actor.camera_state().get('tunnel_confirmed',False)
 actor.set_camera(dict(enabled=True))
 for _ in range(3):
  m=CompressedImage();m.header.stamp=actor.get_clock().now().to_msg();m.format='jpeg';m.data=cv2.imencode('.jpg',tunnel_image())[1].tobytes();actor.camera(m);time.sleep(.09)
 assert actor.camera_state()['tunnel_confirmed']
 assert actor.pub is None and not actor.active
 actor.sign_confirmation.last_time=time.monotonic()-1
 assert not actor.camera_state()['tunnel_confirmed']
 actor.set_camera(dict(enabled=False));assert actor.camera_sub is None and not actor.camera_enabled
 assert 'tunnel_confirmed' not in actor.camera_state()


def test_partial_scan_updates_mapping_pose_but_cannot_arm_navigation(actor,monkeypatch):
 import math
 from sensor_msgs.msg import LaserScan
 from geometry_msgs.msg import TransformStamped
 now=actor.get_clock().now().to_msg()
 scan=LaserScan();scan.header.frame_id='base_scan';scan.header.stamp=now
 scan.angle_min=0.;scan.angle_increment=2*math.pi/360;scan.range_min=.12;scan.range_max=3.5;scan.ranges=[1.]*320+[0.]*40
 actor.scan(scan);actor.mode='localization';actor.map_id='partial';actor.confirmed=True
 actor.odom_pose=(0.,0.,0.);actor.odom_received=time.monotonic();actor.odom_stamp=now.sec+now.nanosec/1e9
 def transform(*args):
  t=TransformStamped();t.header.stamp=actor.get_clock().now().to_msg();t.transform.rotation.w=1.;return t
 monkeypatch.setattr(actor.tf,'lookup_transform',transform)
 fresh,reason=actor.sensors()
 assert fresh is False and '90' in reason and actor.scan_quality['invalid_count']==40
 assert actor.pose==(0.,0.,0.) and len(actor.world_points)==320
 with pytest.raises(ValueError,match='Navigation autonome suspendue'):actor.run_navigation('partial','goal')
 assert actor.pub is None and not actor.active


def test_invalid_resume_does_not_destroy_current_slam(actor,monkeypatch):
 import mission_enceinte
 previous=object();actor.slam_process=previous;called=[]
 monkeypatch.setattr(actor,'ros',lambda fn,timeout=4:fn())
 monkeypatch.setattr(mission_enceinte,'stop_process',lambda process:called.append(process))
 try:
  with pytest.raises(ValueError,match='Position de reprise'):actor.start_slam('mapping',{'start_pose':[0.,0.]})
  assert actor.slam_process is previous and called==[]
 finally:actor.slam_process=None
