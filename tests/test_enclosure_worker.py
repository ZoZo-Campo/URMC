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
