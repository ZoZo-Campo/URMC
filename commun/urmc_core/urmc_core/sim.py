"""Small closed-loop 2-D sensor simulator. Invented courses, not official Gazebo."""
import json, math, time
import cv2
cv2.setNumThreads(1)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from .session import Session, DEFAULTS, validate_changes
from collections import deque
from .course import center, boundary, END, obstacle_segments


def scene(x,y,yaw,mission,t,phase='checker',hide_white=False,brightness=1.,barrier_hold=False,hide_lines=False):
    image=np.full((240,320,3),(65,65,65),np.uint8)
    if mission==1:
        # Ground points are projected into the camera; decisions never receive the map.
        for side,color in [(1,(0,220,240)),(-1,(240,240,240))]:
            if hide_lines or (side==-1 and hide_white):continue
            previous=None
            for px in np.linspace(-.5,END+.5,650):
                wx,wy=boundary(px,side);dx,dy=wx-x,wy-y
                forward=dx*math.cos(yaw)+dy*math.sin(yaw)
                left=-dx*math.sin(yaw)+dy*math.cos(yaw)
                if .10<forward<.85:
                    row=int(239-(forward-.12)/.73*159)
                    point=(int(160-left*110/(forward+.05)),row)
                    if previous is not None:cv2.line(image,previous,point,color,6)
                    previous=point
                else:previous=None
    else:
        offset=int(y*250+yaw*100)
        cv2.line(image,(135+offset,80),(85+offset,239),(0,220,240),8)
        if not hide_white:cv2.line(image,(185+offset,80),(235+offset,239),(240,240,240),8)
    if mission==1 and x>END-.16 and abs(y-center(x))<.1 and math.cos(yaw)>.8:
        if phase=='checker':
            for yy in range(180,224,12):
                for xx in range(90,234,18):
                    c=240 if ((xx-90)//18+(yy-180)//12)%2 else 10
                    cv2.rectangle(image,(xx,yy),(xx+17,yy+11),(c,c,c),-1)
        else:
            pts=np.array([(265+int(24*math.cos(a)),110+int(24*math.sin(a))) for a in np.arange(8)*math.pi/4+math.pi/8])
            cv2.fillPoly(image,[pts],(0,0,240));cv2.putText(image,'STOP',(246,114),cv2.FONT_HERSHEY_SIMPLEX,.23,(255,255,255),1)
    if mission==2 and x>.35 and (t<8 or barrier_hold):cv2.rectangle(image,(35,115),(285,133),(0,0,245),-1)
    if mission==3 and .35<x<.6:cv2.rectangle(image,(15,145),(305,157),(240,240,240),-1)
    return image if brightness==1. else np.clip(image.astype(np.float32)*brightness,0,255).astype(np.uint8)


def ray_segment(x,y,a,seg):
    ax,ay,bx,by=seg;dx,dy=math.cos(a),math.sin(a);ex,ey=bx-ax,by-ay
    cross=dx*ey-dy*ex
    if abs(cross)<1e-9:return math.inf
    t=((ax-x)*ey-(ay-y)*ex)/cross
    u=((ax-x)*dy-(ay-y)*dx)/cross
    return t if t>=.03 and 0<=u<=1 else math.inf


def lidar(x,y,yaw,mission,t,obstacle=False,barrier_hold=False,right_occupied=False):
    angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    values=[math.inf]*360
    if mission==1 and obstacle:
        values=[min(ray_segment(x,y,a+yaw,seg) for seg in obstacle_segments()) for a in angles]
    if mission==4:
        segments=[(.6,-.6,2.,-.6),(.6,.6,2.,.6),(.6,-.6,.6,-.25),(.6,.25,.6,.6),
                  (2.,-.6,2.,-.25),(2.,.25,2.,.6)]
        if obstacle:segments += [(1.25,.1,1.4,.1),(1.25,.25,1.4,.25),(1.25,.1,1.25,.25),(1.4,.1,1.4,.25)]
        values=[min(ray_segment(x,y,a+yaw,s) for s in segments) for a in angles]
    if mission==2 and x>.35 and (t<8 or barrier_hold):
        values=[.4 if abs(a)<.3 else r for a,r in zip(angles,values)]
    if mission==3 and .35<x<.65 and (t<9 or right_occupied):
        values=[.5 if abs(a+math.pi/2)<.45 else r for a,r in zip(angles,values)]
    return [r if .05<=r<=3.5 else math.inf for r in values]


class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        for k,v in dict(mission=1,phase='checker',drop_camera_after=-1.,drop_scan_after=-1.,autostart=False,**DEFAULTS).items():self.declare_parameter(k,v)
        if not self.get_namespace().startswith('/urmc_sim/'):
            raise ValueError('Le simulateur ne peut fonctionner que dans /urmc_sim/<mission>')
        self.mission=self.get_parameter('mission').value;self.phase=self.get_parameter('phase').value
        self.session=Session(self.get_parameter('autostart').value);self.camera_queue=deque()
        self.reset_pose()
        self.add_on_set_parameters_callback(self.check_parameters)
        self.session_pub=self.create_publisher(String,'sim_session',1)
        for name in ('start','pause','restart'):
            self.create_service(Trigger,'sim/'+name,lambda req,res,name=name:self.action(name,res))
        self.start=time.monotonic();self.last=self.start;self.last_cmd=-math.inf;self.held=False;self.count=0
        self.image=self.create_publisher(CompressedImage,'image_raw/compressed',qos_profile_sensor_data)
        self.scan=self.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.odom=self.create_publisher(Odometry,'odom',qos_profile_sensor_data)
        self.events=self.create_publisher(String,'storage_observations',10)
        self.truth=self.create_publisher(String,'sim_ground_truth',10)
        self.create_subscription(Twist,'cmd_vel',self.command,1)
        self.create_subscription(Bool,'magnet_request',self.magnet,1)
        self.create_timer(.05,self.tick)

    def reset_pose(self):
        self.x=.2 if self.mission==4 else 0.;self.y=0.;self.yaw=0.;self.v=0.;self.w=0.;self.held=False;self.magnet_requested=False
        self.last_cmd=-math.inf;self.approach_time=None
        self.camera_queue.clear()

    def check_parameters(self,params):
        try:
            changes={p.name:p.value for p in params if p.name in DEFAULTS}
            if changes:validate_changes(changes)
            return SetParametersResult(successful=True)
        except ValueError as exc:return SetParametersResult(successful=False,reason=str(exc))

    def action(self,name,res):
        self.session.action(name)
        if name=='restart':self.reset_pose()
        if name!='start':self.v=self.w=0.
        self.last=time.monotonic()
        self.session_pub.publish(String(data=json.dumps(self.session.snapshot())))
        res.success=True;res.message={'start':'Démarrage demandé','pause':'Simulation en pause','restart':'Replacé au départ ; cliquer Démarrer'}[name]
        return res

    def command(self,m):self.v=m.linear.x;self.w=m.angular.z;self.last_cmd=time.monotonic()
    def magnet(self,m):self.magnet_requested=m.data

    def tick(self):
        now=time.monotonic();dt=min(.1,now-self.last);self.last=now
        self.session.advance(dt);t=self.session.elapsed
        perturb={k:self.get_parameter(k).value for k in DEFAULTS}
        self.session_pub.publish(String(data=json.dumps(self.session.snapshot())))
        self.held=self.magnet_requested and not perturb['grip_failure']
        if now-self.last_cmd>.3 or not self.session.running:self.v=self.w=0.
        self.x+=self.v*math.cos(self.yaw)*dt;self.y+=self.v*math.sin(self.yaw)*dt;self.yaw+=self.w*dt
        if self.mission in (2,3) and self.x>.35 and self.approach_time is None:self.approach_time=t
        scene_t=(5. if self.mission==2 else 6.)+max(0.,t-self.approach_time) if self.approach_time is not None else t
        stamp=self.get_clock().now().to_msg()
        scan_cut=self.get_parameter('drop_scan_after').value
        if not perturb['scan_off'] and (scan_cut<0 or t<scan_cut):
            m=LaserScan();m.header.stamp=stamp;m.header.frame_id='base_scan';m.angle_min=-math.pi;m.angle_max=math.pi-2*math.pi/360;m.angle_increment=2*math.pi/360;m.range_min=.05;m.range_max=3.5
            ranges=lidar(self.x,self.y,self.yaw,self.mission,scene_t,perturb['obstacle'],perturb['barrier_hold'],perturb['right_occupied'])
            if perturb['front_blocked']:
                ranges=[min(r,.13) if abs(-math.pi+i*2*math.pi/360)<.3 else r for i,r in enumerate(ranges)]
            m.ranges=ranges;self.scan.publish(m)
        camera_cut=self.get_parameter('drop_camera_after').value
        if self.mission<=3 and not perturb['camera_off'] and (camera_cut<0 or t<camera_cut):
            m=CompressedImage();m.header.stamp=stamp;m.header.frame_id='camera';m.format='jpeg'
            ok,encoded=cv2.imencode('.jpg',scene(self.x,self.y,self.yaw,self.mission,scene_t,self.phase,perturb['hide_white'],perturb['brightness'],perturb['barrier_hold'],perturb['hide_lines']))
            if ok:
                m.data=encoded.tobytes();self.camera_queue.append((now+perturb['camera_latency_ms']/1000.,m))
        if perturb['camera_off']:self.camera_queue.clear()
        while self.camera_queue and self.camera_queue[0][0]<=now:
            self.image.publish(self.camera_queue.popleft()[1])
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.x=self.x;m.pose.pose.position.y=self.y
        m.pose.pose.orientation.z=math.sin(self.yaw/2);m.pose.pose.orientation.w=math.cos(self.yaw/2);self.odom.publish(m)
        if self.mission==5:
            near=lambda x:abs(self.x-x)<.04 and abs(self.y)<.04
            data=dict(stamp=stamp.sec+stamp.nanosec/1e9,source_id=1,target_id=2,pallet=True,
                pickup_goal=[.3,0.,0.],drop_goal=[.9,0.,0.],exit_goal=[1.15,0.,0.],
                at_pickup=near(.3),grip_confirmed=self.held,at_drop=near(.9),
                orientation_ok=abs(self.yaw)<.08,path_allowed=True,released_confirmed=not self.held,
                zone_exited=near(1.15),forbidden_zone=False)
            self.events.publish(String(data=json.dumps(data)))
        self.truth.publish(String(data=json.dumps(dict(t=t,x=self.x,y=self.y,yaw=self.yaw,magnet=self.held,session=self.session.snapshot(),perturbations=perturb,course=[list(boundary(float(x),side)) for side in (1,-1) for x in np.linspace(0,END,100)] if self.mission==1 else [],obstacles=obstacle_segments() if self.mission==1 and perturb['obstacle'] else []))))


def main():
    rclpy.init();node=Simulator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
