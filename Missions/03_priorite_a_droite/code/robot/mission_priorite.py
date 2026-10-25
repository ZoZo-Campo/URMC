"""Mission 3 ROS 2: two borders, yield right, cross the unmarked area straight."""
# Works both from the mission folder and from the stable kit_robot entry point.
from pathlib import Path as _LayoutPath
import sys as _layout_sys
_WORKSPACE=next((p for p in _LayoutPath(__file__).resolve().parents if (p/'kit_robot').is_dir() and (p/'src').is_dir()),None)
if _WORKSPACE is None:raise RuntimeError('Workspace URMC introuvable autour du script.')
_layout_sys.path.insert(0,str(_WORKSPACE/'kit_robot'))
import os,sys
from pathlib import Path
if __name__=='__main__':
    from mission_barriere import load_environment
    load_environment()

import argparse,fcntl,json,math,signal,time
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import CompressedImage,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from urmc_core.priority import PriorityBrain,priority_scan,RightTrafficTracker
from urmc_core.operator_settings import current as settings
from urmc_core.vision import analyze
from suivi_piste_camera import prepare_drivers,lane_details,LaneTracker
LATEST=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
cv2.setNumThreads(1)

class PriorityNode(Node):
    def __init__(self,drive=False):
        super().__init__('urmc_right_of_way')
        self.drive=drive;self.done=False;self.brain=PriorityBrain();self.tracker=LaneTracker()
        self.traffic_tracker=RightTrafficTracker();self.traffic=None;self.pose=None;self.scan={};self.lane={};self.sample_id=None;self.scan_id=None
        self.received={};self.stamps={};self.previous_pose=None;self.finish_count=0
        self.last_image=0.;self.last_log=0.;self.last_state=None;self.v=0.;self.w=0.
        self.pub=self.create_publisher(Twist,'/cmd_vel',1) if drive else None
        self.status=self.create_publisher(String,'/urmc/priority/status',1)
        self.create_subscription(CompressedImage,'/image_raw/compressed',self.image,LATEST)
        self.create_subscription(LaserScan,'/scan',self.on_scan,LATEST)
        self.create_subscription(Odometry,'/odom',self.odom,LATEST)
        self.create_service(Trigger,'/urmc/priority/stop',self.stop_request)
        if drive:self.create_service(Trigger,'/urmc/camera_follow/stop',self.stop_request)
        self.create_timer(.05,self.tick)
    def header(self,msg,key,limit):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        if not math.isfinite(age) or not 0<=age<limit:
            self.received.pop(key,None);return None
        if stamp<=self.stamps.get(key,-math.inf):return None
        return stamp
    def accept(self,key,stamp):
        self.stamps[key]=stamp;self.received[key]=time.monotonic()
    def image(self,msg):
        now=time.monotonic()
        if now-self.last_image<.09:return
        stamp=self.header(msg,'camera',.25)
        if stamp is None:return
        self.last_image=now
        try:
            image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('image indecodable')
            self.lane=lane_details(msg.data,self.tracker);self.sample_id=stamp
            features=analyze(image)
            self.finish_count=self.finish_count+1 if features['checker'] else 0
            self.lane['intersection']=features['intersection']
            self.accept('camera',stamp)
        except Exception:self.received.pop('camera',None);self.finish_count=0
    def on_scan(self,msg):
        stamp=self.header(msg,'lidar',.30)
        if stamp is None:return
        self.scan=priority_scan(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,settings())
        if self.scan['right'] is None or self.scan['front'] is None:self.received.pop('lidar',None);return
        self.scan_id=stamp
        self.traffic=self.traffic_tracker.update(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,self.pose,time.monotonic(),stamp,settings())
        self.accept('lidar',stamp)
    def odom(self,msg):
        stamp=self.header(msg,'odom',.30)
        if stamp is None:return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>.05:
            self.received.pop('odom',None);return
        pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if self.previous_pose is not None and math.hypot(p.x-self.previous_pose[0],p.y-self.previous_pose[1])>.15:
            self.stop('saut odometrique');return
        self.pose=pose;self.previous_pose=pose;self.accept('odom',stamp)
    def freshness(self,now):
        clock=self.get_clock().now().nanoseconds/1e9
        for key,limit in (('camera',.25),('lidar',.30),('odom',.30)):
            if not 0<=now-self.received.get(key,-math.inf)<limit or not 0<=clock-self.stamps.get(key,-math.inf)<limit:
                return False,{'camera':'WAIT_CAMERA','lidar':'WAIT_LIDAR','odom':'WAIT_ODOM'}[key]
        return True,None
    def stop_request(self,request,response):
        self.stop('arret demande');response.success=True;response.message='Mission priorite arretee';return response
    def stop(self,why):
        if self.pub:self.pub.publish(Twist())
        self.done=True;self.v=self.w=0.;self.brain.state='STOP';self.brain.reason=why
        self.report(False,'STOP');print('ARRET : '+why,flush=True)
    def report(self,fresh,state=None):
        cfg=settings()
        data=dict(algorithm='priority' if self.drive else 'priority-observe',mode='roulage' if self.drive else 'observation',
            state=state or self.brain.state,reason=self.brain.reason,fresh=fresh,v=self.v,w=self.w,
            lane_reason=self.lane.get('reason','attente piste'),error=self.lane.get('error'),
            traffic=self.traffic,right=self.scan.get('right'),front=self.scan.get('front'),lidar_points=self.scan.get('points',[]),
            right_start=cfg['priority_right_start'],right_end=cfg['priority_right_end'],right_threshold=cfg['priority_right_distance'],
            crossing_distance=round(self.brain.progress,3),crossing_max=cfg['priority_cross_max'],
            heading=self.brain.heading,clear_frames=self.brain.clear_frames)
        self.status.publish(String(data=json.dumps(data)))
        now=time.monotonic()
        if data['state']!=self.last_state or now-self.last_log>1:
            print(json.dumps(data),flush=True);self.last_log=now;self.last_state=data['state']
    def tick(self):
        if self.done:return
        now=time.monotonic();cfg=settings();fresh,waiting=self.freshness(now)
        if self.pub and self.count_publishers(self.pub.topic_name)!=1:return self.stop('autre commande moteur detectee')
        if fresh and self.finish_count>=3:return self.stop('damier confirme sur trois images')
        error=self.lane.get('error')
        yc,wc=self.lane.get('yellow'),self.lane.get('white')
        obs=dict(fresh=fresh,stale_reason=waiting,pose=self.pose,front=self.scan.get('front'),right=self.scan.get('right'),
                 lane_valid=error is not None,error=error,entry_eligible=yc is None and wc is None,lane_width=abs(wc-yc) if yc is not None and wc is not None else None,
                 traffic=self.traffic,sample_id=self.sample_id,scan_id=self.scan_id,intersection=self.lane.get('intersection',False),route_confirmed=True)
        command=self.brain.step(obs,now,cfg)
        v,w=(command.v,command.w) if self.drive else (0.,0.)
        self.v=0. if v==0 else self.v+max(-.005,min(.0015,v-self.v));self.w=0. if v==0 else w
        if self.pub:
            out=Twist();out.linear.x=float(self.v);out.angular.z=float(self.w);self.pub.publish(out)
        self.report(fresh,waiting if not fresh else None)
        if self.brain.state=='FAULT':self.stop(self.brain.reason)

def main():
    parser=argparse.ArgumentParser(description='Priorite a droite + traversee tout droit sans lignes.')
    parser.add_argument('--run',action='store_true',help='Autoriser les moteurs. Sans cette option : observation seule.')
    args,ros_args=parser.parse_known_args()
    workspace=Path(os.environ['URMC_CAMERA_WORKSPACE']);locks=[]
    for name in ('.barrier.lock','.camera_follow.lock'):
        handle=(workspace/name).open('w')
        try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Une mission est deja active. Utiliser STOP avant de changer de mode.')
        locks.append(handle)
    prepare_drivers();rclpy.init(args=ros_args);node=PriorityNode(args.run)
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:node.stop('interruption'))
    print('Mission 3 : '+('MOTEURS ACTIFS' if args.run else 'OBSERVATION SANS MOTEURS')+'. Camera compressee + /scan + /odom. Un passage tout droit, puis reprise du suivi.',flush=True)
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if node.pub:
            for _ in range(6):node.pub.publish(Twist());time.sleep(.05)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
