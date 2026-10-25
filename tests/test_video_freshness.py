"""Real HTTP checks: stale camera frames must never be presented as live."""
import sys,time,threading,urllib.request,urllib.error
from pathlib import Path
import cv2,numpy as np,pytest,rclpy
from sensor_msgs.msg import CompressedImage
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from retour_video import Viewer,make_handler,PAGE
from http.server import ThreadingHTTPServer


def test_overlay_and_stale_frame_rejection():
 rclpy.init();node=Viewer('/video_test',False,'/unused','lane')
 server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(node,PAGE))
 worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
 url='http://127.0.0.1:'+str(server.server_port)
 try:
  image=np.full((240,320,3),65,np.uint8)
  cv2.line(image,(80,100),(80,239),(0,200,220),8)
  cv2.line(image,(240,100),(240,239),(175,175,175),8)
  msg=CompressedImage();msg.header.stamp=node.get_clock().now().to_msg();msg.format='jpeg'
  msg.data=cv2.imencode('.jpg',image)[1].tobytes();node.compressed(msg)
  with urllib.request.urlopen(url+'/frame.jpg') as response:
   output=cv2.imdecode(np.frombuffer(response.read(),np.uint8),1)
   assert response.headers['X-Frame-Sequence']=='1'
   assert float(response.headers['X-Source-Age'])<.5
   assert output.shape==(360,640,3)
   b,g,r=cv2.split(output.astype(float));assert ((g>150)&(g>1.5*r)&(g>1.5*b)).sum()>30 # actual lane annotation
  time.sleep(.8)
  with pytest.raises(urllib.error.HTTPError) as exc:urllib.request.urlopen(url+'/frame.jpg')
  assert exc.value.code==503
  msg.header.stamp.sec-=30;node.compressed(msg)
  assert node.frames==1 # stale ROS messages rejected
 finally:
  server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()


def test_permanent_camera_follows_mission_and_survives_stop():
 from std_msgs.msg import String
 rclpy.init();node=Viewer('/video_test',False,'/unused','auto')
 try:
  assert node.view_mode()=='lane'
  node.mission_status('barrier',String(data='{"mode":"observation","state":"OBSERVATION"}'))
  assert node.view_mode()=='barrier'
  node.mission_status('follow',String(data='{"state":"FOLLOW"}'))
  assert node.view_mode()=='lane' # rolling controller takes priority over observer
  node.mission_status('follow',String(data='{"state":"STOP"}'))
  assert node.view_mode()=='barrier'
  node.mission_status('barrier',String(data='{"state":"STOP"}'))
  assert node.view_mode()=='lane'
  for key,mode in [('1','lane'),('2','barrier'),('3','lane'),('4','none'),('5','none')]:
   node.mission_status(key,String(data='{"state":"WAIT"}'))
   assert node.view_mode()==mode
   node.mission_status(key,String(data='{"state":"DONE"}'))
  assert node.count_publishers('/cmd_vel')==0
 finally:node.destroy_node();rclpy.shutdown()


def test_lidar_monitor_freshness_and_full_resolution():
 import math,json
 from sensor_msgs.msg import LaserScan
 rclpy.init();node=Viewer('/video_test',False,'/unused','auto')
 server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(node,PAGE));threading.Thread(target=server.serve_forever,daemon=True).start()
 try:
  m=LaserScan();m.header.stamp=node.get_clock().now().to_msg();m.angle_min=-math.pi;m.angle_increment=math.pi/180;m.range_min=.05;m.range_max=3.5;m.ranges=[.5]*360
  node.scan(m)
  with urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/lidar.json') as response:data=json.load(response)
  assert data['fresh'] and len(data['points'])==360
  assert data['right']==pytest.approx(.5) and node.count_publishers('/cmd_vel')==0
  time.sleep(.45);assert not node.lidar_snapshot()['fresh']
  m.header.stamp.sec-=30;node.scan(m);assert not node.lidar_snapshot()['fresh']
 finally:server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()
