"""Mission 2 : observation blue rectangle by default; --run enables supervised movement."""
import os,sys,subprocess,fcntl
from pathlib import Path

def load_environment():
 script=Path(__file__).resolve()
 if os.environ.get('URMC_CAMERA_BOOTSTRAPPED')==str(script):return
 workspace=next((p for p in script.parents if (p/'install/setup.bash').is_file()),None)
 if workspace is None:raise SystemExit('Workspace non compile : install/setup.bash introuvable autour du script.')
 env=os.environ.copy();env['URMC_CAMERA_BOOTSTRAPPED']=str(script)
 env['URMC_CAMERA_WORKSPACE']=str(workspace)
 env['ROS_DOMAIN_ID']=env.get('URMC_ROBOT_DOMAIN','41');env['ROS_LOCALHOST_ONLY']='0'
 env['TURTLEBOT3_MODEL']='burger';env['LDS_MODEL']='LDS-01'
 # Positional arguments keep paths out of shell interpolation.
 command='set -e; source /opt/ros/humble/setup.bash; if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi; source "$1"; exec /usr/bin/python3 "$2" "${@:3}"'
 os.execvpe('bash',['bash','-c',command,'urmc',str(workspace/'install/setup.bash'),str(script),*sys.argv[1:]],env)

if __name__=='__main__':load_environment()

import argparse,fcntl,json,math,signal,time
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from urmc_core.barrier import detect,annotate,OpeningGate
from suivi_piste_camera import prepare_drivers,lane_details,LaneTracker
from urmc_core.vision import analyze
cv2.setNumThreads(1)

class BarrierNode(Node):
    def __init__(self,drive=False,seconds=0.):
        super().__init__('urmc_blue_barrier')
        self.drive=drive;self.limit=seconds;self.created=time.monotonic();self.done=False
        self.pub=self.create_publisher(Twist,'/cmd_vel',1) if drive else None
        self.video=self.create_publisher(CompressedImage,'/urmc/barrier/image/compressed',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/urmc/barrier/status',1)
        self.create_subscription(CompressedImage,'/image_raw/compressed',self.image,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_service(Trigger,'/urmc/barrier/stop',self.stop_request)
        if drive:self.create_service(Trigger,'/urmc/camera_follow/stop',self.stop_request)
        self.create_timer(.05,self.tick)
        self.gate=OpeningGate(allow_disappearance=True);self.ready=False;self.received=0.;self.image_age=None;self.previous_stamp=None
        self.result=dict(position='UNKNOWN',reason='attente camera');self.error=None;self.last_processed=0.;self.last_preview=0.
        self.lane_valid=0;self.filtered_error=None;self.last_logged_state=None;self.tracker=LaneTracker();self.barrier_blocked=False;self.opening_validated=False;self.finish_count=0;self.lane_reason="attente piste";self.distance=0.;self.cross_start=None;self.previous_pose=None;self.odom_received=0.
        self.v=0.;self.w=0.;self.last_log=0.;self.state='OBSERVATION' if not drive else 'WAIT_CLOSED'
    def stop_request(self,request,response):
        self.stop('arret demande');response.success=True;response.message='Mission barriere arretee';return response
    def stop(self,why):
        if self.pub:self.pub.publish(Twist())
        self.done=True;self.state='STOP';print('ARRET : '+why,flush=True)
    def odom(self,msg):
        now=time.monotonic();age=self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        p=msg.pose.pose.position
        if not 0<=age<.3 or not all(math.isfinite(v) for v in (p.x,p.y)):return
        if self.previous_pose is not None:
            step=math.hypot(p.x-self.previous_pose[0],p.y-self.previous_pose[1])
            if step>.15:
                self.previous_pose=(p.x,p.y)
                if self.drive:self.stop('saut odometrique')
                return
            self.distance+=step
        self.previous_pose=(p.x,p.y);self.odom_received=now
    def image(self,msg):
        now=time.monotonic()
        if now-self.last_processed<.09:return
        self.last_processed=now
        age=self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
        self.image_age=age
        if not 0<=age<.25:return
        try:
            image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
            if image is None:return
            stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
            if stamp==self.previous_stamp:return
            self.previous_stamp=stamp;self.result=detect(image)
            self.ready=self.gate.update(self.result,now,stamp)
            position=self.result['position']
            if position in ('CLOSED','MOVING'):
                self.barrier_blocked=True;self.opening_validated=False
            if self.ready:self.barrier_blocked=False;self.opening_validated=True
            lane=lane_details(msg.data,self.tracker);self.error=lane['error'];self.lane_reason=lane['reason'];self.received=now
            valid=self.error is not None and abs(self.error)<=.65
            self.lane_valid=self.lane_valid+1 if valid else 0
            self.filtered_error=(self.error if self.filtered_error is None else .35*self.error+.65*self.filtered_error) if valid else None
            self.finish_count=self.finish_count+1 if analyze(image)['checker'] else 0
            if now-self.last_preview>=.2:
                out=CompressedImage();out.header=msg.header;out.format='jpeg'
                ok,jpeg=cv2.imencode('.jpg',annotate(image,self.result,self.ready),[cv2.IMWRITE_JPEG_QUALITY,50])
                if ok:out.data=jpeg.tobytes();self.video.publish(out)
                self.last_preview=now
        except Exception as exc:
            self.ready=False;self.result=dict(position='UNKNOWN',reason=str(exc));self.error=None
    def tick(self):
        if self.done:return
        now=time.monotonic();fresh=now-self.received<.25 and self.image_age is not None and 0<=self.image_age<.25
        v=w=0.
        if self.drive:
            if self.count_publishers(self.pub.topic_name)!=1:return self.stop('autre commande moteur detectee')
            if self.limit>0 and now-self.created>self.limit:return self.stop('limite de temps atteinte')
            if not fresh:self.state='WAIT_CAMERA';self.ready=False;self.gate.since=None;self.gate.frames=0;self.lane_valid=0;self.filtered_error=None
            elif self.finish_count>=3:return self.stop('damier confirme sur trois images')
            elif self.barrier_blocked:self.state='WAIT_OPEN'
            elif self.result.get('position')=='UNKNOWN' and self.result.get('visible',False):self.state='WAIT_BARRIER_VIEW'
            elif now-self.odom_received>=.3:self.state='WAIT_ODOM';self.lane_valid=0
            elif self.error is None:self.state='WAIT_LANE'
            elif abs(self.error)>.65:self.state='WAIT_LANE_OFFSET'
            elif self.lane_valid<3:self.state='CONFIRM_LANE'
            else:
                if self.cross_start is None:self.cross_start=self.distance
                self.state='FOLLOW_AFTER_BARRIER' if self.gate.closed_seen else 'FOLLOW';error=self.filtered_error if self.filtered_error is not None else self.error;v=.05/(1+3*abs(error));w=max(-.2,min(.2,-.8*error))
            self.v=0. if v==0 else self.v+max(-.005,min(.0015,v-self.v))
            self.w=0. if v==0 else self.w+max(-.025,min(.025,w-self.w))
            cmd=Twist();cmd.linear.x=self.v;cmd.angular.z=self.w;self.pub.publish(cmd)
        elif self.limit>0 and now-self.created>self.limit:return self.stop('fin observation')
        data=dict(mode='roulage' if self.drive else 'observation',state=self.state,fresh=fresh,barrier=self.result,
            closed_seen=self.gate.closed_seen,opening_confirmed=self.ready,barrier_blocked=self.barrier_blocked,lane_reason=self.lane_reason,error=self.error,lane_valid_frames=self.lane_valid,v=self.v,w=self.w)
        self.status.publish(String(data=json.dumps(data)))
        if self.state!=self.last_logged_state or now-self.last_log>2:
            print(json.dumps(data),flush=True);self.last_log=now;self.last_logged_state=self.state

def preview(workspace):
    script=Path(__file__).with_name('retour_video.py')
    if not script.exists():return
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:
            args=item.joinpath('cmdline').read_bytes().decode().split('\0')
            if str(script) in args and '8081' in args:return
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    log=workspace/'session_robot11/barriere_video.log'
    with log.open('a') as output:
        subprocess.Popen(['/usr/bin/python3',str(script),'--topic','/urmc/barrier/image/compressed','--port','8081',
            '--stop-service','/urmc/barrier/stop','--fps','6'],stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)

def main():
    parser=argparse.ArgumentParser(description='Rectangle bleu : horizontal ferme, vertical leve.')
    parser.add_argument('--run',action='store_true',help='Autoriser le roulage ; par defaut observation sans commande moteur.')
    parser.add_argument('--seconds',type=float,default=None)
    args=parser.parse_args()
    if args.seconds is not None and not 0<=args.seconds<=180:parser.error('Duree entre 0 et 180 secondes ; 0 = sans limite.')
    workspace=Path(os.environ['URMC_CAMERA_WORKSPACE'])
    locks=[]
    for name in (['.barrier.lock','.camera_follow.lock'] if args.run else ['.barrier.lock']):
        handle=(workspace/name).open('w')
        try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Une mission est deja active. Utiliser STOP avant de changer de mode.')
        locks.append(handle)
    prepare_drivers();preview(workspace)
    rclpy.init();node=BarrierNode(args.run,args.seconds if args.seconds is not None else 0.)
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda signum,frame:node.stop('interruption'))
    print('Video annotee : port 8081. '+('ROULAGE 5 cm/s maximum.' if args.run else 'OBSERVATION : aucune publication moteur.'),flush=True)
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        if node.pub:
            for _ in range(6):node.pub.publish(Twist());time.sleep(.05)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
