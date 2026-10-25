"""Console requests, settings bounds, exclusive modes and stale-command rejection."""
import io,json,sys,threading,urllib.request,urllib.error
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from urmc_core.operator_settings import DEFAULTS,validate
from operator_agent import Controller,handler
from operator_console import Backend,make_handler
from http.server import ThreadingHTTPServer

class Process:
 def __init__(self):self.stdin=io.StringIO();self.stdout=io.StringIO();self.returncode=None
 def poll(self):return self.returncode
 def send_signal(self,_):self.returncode=0
 def wait(self,**_):return self.returncode

@pytest.mark.parametrize('values',[{'speed':.3},{'speed':float('nan')},{'blue_low':140,'blue_high':100},{'yellow_low':True},{'shell':'bad'},{'opening_delay':.1}])
def test_refuses_invalid_settings(values):
 with pytest.raises(ValueError):validate(values)

def test_switching_mode_revokes_manual_lease(tmp_path):
 processes=[]
 def popen(*_,**__):p=Process();processes.append(p);return p
 controller=Controller(tmp_path,popen)
 lease=controller.start('manual')['lease']
 controller.manual(dict(lease=lease,seq=1,active=True,hold='a',x=1,yaw=0))
 assert json.loads(processes[0].stdin.getvalue().splitlines()[-1])['x']==1
 controller.start('follow-robust')
 assert processes[0].poll()==0
 with pytest.raises(ValueError):controller.manual(dict(lease=lease,seq=2,active=True,hold='a',x=1,yaw=0))
 controller.stop();assert controller.state()['mode']=='idle'

def test_refuses_old_manual_packet(tmp_path):
 controller=Controller(tmp_path,lambda *a,**kw:Process());lease=controller.start('manual')['lease']
 data=dict(lease=lease,seq=2,active=False);controller.manual(data)
 with pytest.raises(ValueError):controller.manual(dict(data,seq=1,active=True,x=1,yaw=0,hold='old'))
 controller.stop()

def test_http_console_requires_origin_and_token():
 backend=Backend(True);server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(backend,'test-token'));threading.Thread(target=server.serve_forever,daemon=True).start()
 url='http://127.0.0.1:'+str(server.server_port)
 def request(headers):
  return urllib.request.urlopen(urllib.request.Request(url+'/api/start',data=b'{"algorithm":"manual"}',headers=headers))
 try:
  for headers in ({},{'X-Console-Token':'test-token','Origin':'http://evil.invalid'}):
   with pytest.raises(urllib.error.HTTPError) as e:request(headers)
   assert e.value.code==403
  with request({'X-Console-Token':'test-token','Origin':url}) as response:assert json.load(response)['lease']
  assert backend.state()['mode']=='manual'
 finally:server.shutdown();server.server_close()

def test_delayed_start_cannot_override_stop(tmp_path):
 controller=Controller(tmp_path,lambda *a,**kw:Process())
 server=ThreadingHTTPServer(('127.0.0.1',0),handler(controller,'robot-token'));threading.Thread(target=server.serve_forever,daemon=True).start()
 url='http://127.0.0.1:'+str(server.server_port)
 def post(path,data):return urllib.request.urlopen(urllib.request.Request(url+path,data=json.dumps(data).encode(),headers={'Authorization':'Bearer robot-token'}))
 try:
  with post('/stop',{'generation':20}):pass
  with pytest.raises(urllib.error.HTTPError):post('/start',{'generation':19,'algorithm':'manual'})
  assert controller.state()['mode']=='idle'
 finally:server.shutdown();server.server_close()


def test_live_settings_reload_and_invalid_file_commands_zero(tmp_path,monkeypatch):
 from urmc_core import operator_settings as settings
 path=tmp_path/'settings.json';monkeypatch.setenv('URMC_SETTINGS_FILE',str(path))
 path.write_text(json.dumps(dict(DEFAULTS,speed=.02)))
 assert settings.current()['speed']==.02
 path.write_text('broken json');monkeypatch.setattr(settings,'_checked',0.)
 assert settings.current()['speed']==0


def test_color_settings_are_used_by_the_real_detectors(tmp_path,monkeypatch):
 import cv2,numpy as np
 from urmc_core import operator_settings as settings
 from urmc_core.barrier import detect
 from suivi_piste_camera import lane_details
 path=tmp_path/'settings.json';monkeypatch.setenv('URMC_SETTINGS_FILE',str(path))
 image=np.full((240,320,3),65,np.uint8)
 cv2.rectangle(image,(40,40),(200,60),(240,70,20),-1)
 cv2.line(image,(80,100),(80,239),(0,200,220),8);cv2.line(image,(240,100),(240,239),(175,175,175),8)
 jpeg=cv2.imencode('.jpg',image)[1]
 path.write_text(json.dumps(DEFAULTS));monkeypatch.setattr(settings,'_checked',0.)
 assert detect(image)['position']=='CLOSED';assert lane_details(jpeg)['error'] is not None
 path.write_text(json.dumps(dict(DEFAULTS,blue_low=0,blue_high=5,yellow_low=0,yellow_high=5)));monkeypatch.setattr(settings,'_checked',0.)
 assert detect(image)['position']=='UNKNOWN';assert lane_details(jpeg)['error'] is None


def test_simulation_cancel_during_preparation_does_not_launch_container(monkeypatch):
 import operator_console as console
 entered=threading.Event();resume=threading.Event();launched=[]
 class Result:returncode=0
 def run(args,**kwargs):
  if args==['docker','info']:entered.set();assert resume.wait(2)
  return Result()
 monkeypatch.setattr(console.subprocess,'run',run)
 monkeypatch.setattr(console.subprocess,'Popen',lambda *a,**kw:launched.append(a))
 backend=Backend(True);backend.start_sim(1);assert entered.wait(2)
 backend.stop_sim();resume.set()
 assert backend.job_lock.acquire(timeout=2);backend.job_lock.release()
 assert not launched
 assert 'annulée' in backend.job['message']


def test_fleet_keeps_modes_settings_and_tunnels_separate(tmp_path):
 from operator_console import Fleet
 fleet=Fleet(True,tmp_path/'robots.json')
 a,b=fleet.robots[11],fleet.robots[9]
 assert a.video_port!=b.video_port and a.agent_port!=b.agent_port
 assert a.host=='192.168.42.211' and b.host=='192.168.42.209'
 assert 'URMC_ROBOT_DOMAIN=39' in b.command('teleop_robot.py')
 fleet.action('/settings',{'robot':11,'speed':.03})
 fleet.action('/start',{'robot':11,'algorithm':'manual'})
 fleet.select(9)
 assert a.demo_mode=='idle'
 fleet.action('/settings',{'robot':9,'speed':.02})
 fleet.action('/start',{'robot':9,'algorithm':'barrier'})
 assert a.demo_settings['speed']==.03 and b.demo_settings['speed']==.02
 with pytest.raises(ValueError):fleet.action('/start',{'robot':11,'algorithm':'manual'})
 with pytest.raises(ValueError):fleet.action('/manual',{'robot':11,'lease':'old','seq':1,'active':True,'hold':'x','x':1,'yaw':0})
 fleet.stop_all();assert a.demo_mode==b.demo_mode=='idle'


def test_fleet_rejects_duplicate_robot_ip_and_invalid_domain(tmp_path):
 from operator_console import Fleet
 fleet=Fleet(True,tmp_path/'robots.json');fleet.select(9)
 with pytest.raises(ValueError):fleet.configure({'robot':9,'host':'192.168.42.211','domain':39})
 with pytest.raises(ValueError):fleet.configure({'robot':9,'host':'192.168.42.209','domain':999})
 with pytest.raises(ValueError):fleet.configure({'robot':9,'host':'192.168.42.209','domain':41})
 fleet.configure({'robot':9,'host':'192.168.42.210','domain':39})
 assert fleet.robots[11].host=='192.168.42.211' and fleet.robots[9].host=='192.168.42.210'


def test_each_video_response_identifies_its_robot(tmp_path):
 from operator_console import Fleet
 fleet=Fleet(True,tmp_path/'robots.json');server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(fleet,'test-token'));threading.Thread(target=server.serve_forever,daemon=True).start()
 try:
  frames=[]
  for robot in (9,11):
   with urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/api/frame?robot={robot}') as response:
    assert response.headers['X-Robot-ID']==str(robot)
    frames.append(response.read())
    assert response.headers['Content-Type'] in ('image/jpeg','image/svg+xml')
  assert frames[0]!=frames[1]
 finally:server.shutdown();server.server_close()


def test_priority_modes_and_lidar_monitor_keep_robot_identity(tmp_path):
 from operator_console import Fleet
 fleet=Fleet(True,tmp_path/'robots.json');fleet.action('/start',dict(robot=11,algorithm='priority-observe'))
 assert fleet.robots[11].demo_mode=='priority-observe'
 fleet.select(9);fleet.action('/start',dict(robot=9,algorithm='priority'))
 assert fleet.robots[11].demo_mode=='idle'
 server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(fleet,'token'));threading.Thread(target=server.serve_forever,daemon=True).start()
 try:
  for robot in (9,11):
   with urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/api/lidar?robot={robot}') as response:data=json.load(response)
   assert data['robot_id']==robot and data['fresh'] and data['demo']
 finally:server.shutdown();server.server_close();fleet.stop_all()


def test_enclosure_replaces_manual_without_a_second_motor_process(tmp_path):
 controller=Controller(tmp_path,lambda *a,**kw:Process())
 class Maps:
  def __init__(self):self.active=False;self.map_id=None
  def stop(self,required=False):self.active=False
  def ensure(self):pass
  def request(self,path,data):self.active=True;self.map_id=data['map_id'];self.task=data['task'];return {'success':True}
  def summary(self):return dict(active=self.active,phase='ENTRY')
 maps=Maps();controller.mapping=maps
 lease=controller.start('manual')['lease'];previous=controller.process
 controller.start('enclosure','chosen-map')
 assert previous.poll()==0 and controller.lease is None and maps.map_id=='chosen-map'
 assert controller.state()['mode']=='enclosure'
 with pytest.raises(ValueError):controller.manual(dict(lease=lease,seq=3,active=True,hold='x',x=1,yaw=0))
 controller.stop();assert not maps.active
 controller.start('enclosure','chosen-map','goal');assert maps.task=='goal'
 controller.stop()


def test_slam_camera_relay_is_a_read_only_jpeg(monkeypatch,tmp_path):
 import operator_console as console
 class Response:
  def __enter__(self):return self
  def __exit__(self,*_):pass
  def read(self):return b'\xff\xd8test-jpeg'
 fleet=console.Fleet(True,tmp_path/'robots.json');fleet.mapping_token='private-token'
 original=urllib.request.urlopen
 def upstream(req,*args,**kw):
  url=req.full_url if hasattr(req,'full_url') else req
  if url=='http://127.0.0.1:8894/camera':
   assert req.headers.get('Authorization')=='Bearer private-token';return Response()
  return original(req,*args,**kw)
 monkeypatch.setattr(console.urllib.request,'urlopen',upstream)
 server=ThreadingHTTPServer(('127.0.0.1',0),console.make_handler(fleet,'page-token'));threading.Thread(target=server.serve_forever,daemon=True).start()
 try:
  with original(f'http://127.0.0.1:{server.server_port}/api/mapping/frame') as r:
   assert r.headers['Content-Type']=='image/jpeg' and r.read().startswith(b'\xff\xd8')
 finally:server.shutdown();server.server_close()
