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
