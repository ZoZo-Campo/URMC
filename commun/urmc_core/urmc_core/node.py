"""ROS 2 adapter: real subscriptions, watchdog, arming service and bounded Twist."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from .control import Brain, guard, scan_features


class MissionNode(Node):
    def __init__(self, mission):
        super().__init__('mission')
        self.mission=mission
        for k,v in dict(phase='checker',speed=.07,kp=.8,timeout=.30,
            auto_arm=False,simulation=False,route='unconfirmed',storage_calibrated=False,
            white_s=60,white_v=180,yellow_low=18,yellow_high=38,stop_shape_enabled=False,
            image_transport='compressed',scan_yaw_offset=0.0).items(): self.declare_parameter(k,v)
        self.param=lambda k:self.get_parameter(k).value
        if self.param('auto_arm') and not self.param('simulation'):
            raise ValueError('auto_arm réservé au simulateur')
        if self.param('simulation') and not self.get_namespace().startswith('/urmc_sim/'):
            raise ValueError('Simulation obligatoire sous /urmc_sim/<mission>')
        if not 0<self.param('speed')<=.1 or not 0<self.param('timeout')<=.5:
            raise ValueError('Vitesse ou timeout hors des limites du prototype')
        self.brain=self.new_brain();self.armed=False
        self.sim_session=None;self.sim_pending=False
        if self.param('simulation'):self.create_subscription(String,'sim_session',self.on_session,1)
        self.scan=None;self.vision={};self.pose=None;self.events={};self.received={}
        self.have_fresh=False;self.distance=0.;self.previous_pose=None;self.lidar_barrier_seen=False
        self.pub=self.create_publisher(Twist,'cmd_vel',1)
        self.status=self.create_publisher(String,'status',10)
        self.magnet=self.create_publisher(Bool,'magnet_request',1)
        self.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        if mission!=4:
            if mission<=3:
                if self.param('image_transport')=='compressed':
                    self.create_subscription(CompressedImage,'image_raw/compressed',self.on_image,qos_profile_sensor_data)
                else:self.create_subscription(Image,'image_raw',self.on_raw,qos_profile_sensor_data)
            if mission in (1,2,3,5):self.create_subscription(Odometry,'odom',self.on_odom,qos_profile_sensor_data)
        if mission==5:self.create_subscription(String,'storage_observations',self.on_events,10)
        self.create_service(SetBool,'enable',self.enable)
        self.create_service(Trigger,'reset',self.reset)
        self.create_timer(.05,self.tick)
        self.get_logger().info(f'Mission {mission}: {"simulation armée" if self.armed else "désarmée — service enable requis"}')

    def on_session(self,msg):
        try:
            data=json.loads(msg.data)
            if type(data['running']) is not bool or type(data['epoch']) is not int or type(data['revision']) is not int or not math.isfinite(data['t']):return
        except (KeyError,ValueError,TypeError):return
        previous=self.sim_session
        if previous is None or data['epoch']!=previous['epoch']:
            self.armed=False;self.brain=self.new_brain();self.distance=0.;self.previous_pose=None
            self.received={};self.scan=None;self.vision={};self.events={};self.pose=None
            self.have_fresh=False;self.lidar_barrier_seen=False
        if previous is None or data['revision']!=previous['revision']:
            self.sim_pending=data['running']
        self.sim_session=data;self.received['session']=time.monotonic()
        if not data['running']:
            self.armed=False;self.sim_pending=False;self.pub.publish(Twist())

    def new_brain(self):return Brain(self.mission,self.param('phase'),self.param('speed'),self.param('kp'))

    def valid_header(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        return 0<=age<=self.param('timeout')

    def on_scan(self,msg):
        try:
            if not self.valid_header(msg) or msg.angle_increment<=0 or not msg.ranges:
                self.received.pop('scan',None);return
            self.scan=scan_features(msg.ranges,msg.angle_min+self.param('scan_yaw_offset'),msg.angle_increment,msg.range_min,msg.range_max,compute_heading=self.mission==4)
            self.received['scan']=time.monotonic()
        except (ValueError,TypeError):self.received.pop('scan',None)

    def decode(self,image,msg):
        from .vision import analyze
        if not self.valid_header(msg):self.received.pop('vision',None);return
        self.vision=analyze(image,self.param('white_s'),self.param('white_v'),self.param('yellow_low'),self.param('yellow_high'))
        # No semantic STOP detector is claimed: shape mode only allowed in simulation.
        if not (self.param('simulation') and self.param('stop_shape_enabled')):self.vision['stop']=False
        self.vision['sample_id']=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        self.received['vision']=time.monotonic()

    def on_image(self,msg):
        try:
            import cv2,numpy as np
            if not self.valid_header(msg):self.received.pop('vision',None);return
            self.decode(cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR),msg)
        except Exception as exc:
            self.received.pop('vision',None);self.get_logger().warning('Image refusée: '+str(exc),throttle_duration_sec=5.)

    def on_raw(self,msg):
        try:
            from cv_bridge import CvBridge
            self.decode(CvBridge().imgmsg_to_cv2(msg,'bgr8'),msg)
        except Exception:self.received.pop('vision',None)

    def on_odom(self,msg):
        if not self.valid_header(msg):self.received.pop('odom',None);return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not all(math.isfinite(x) for x in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.05:
            self.received.pop('odom',None);return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose=(p.x,p.y,yaw)
        if self.previous_pose:
            step=math.hypot(p.x-self.previous_pose[0],p.y-self.previous_pose[1])
            if step>.15:
                self.armed=False;self.previous_pose=pose;self.received.pop('odom',None);return
            self.distance+=step
        self.pose=pose;self.previous_pose=pose;self.received['odom']=time.monotonic()

    def on_events(self,msg):
        try:
            data=json.loads(msg.data);stamp=data['stamp']
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if not isinstance(data,dict) or not math.isfinite(age) or not 0<=age<=self.param('timeout'):raise ValueError()
            self.events=data;self.received['events']=time.monotonic()
        except (ValueError,KeyError,TypeError):self.received.pop('events',None)

    def requirements(self):
        return (['session'] if self.param('simulation') else [])+['scan']+(['vision'] if self.mission<=3 else [])+(['odom'] if self.mission in (1,2,3,5) else [])+(['events'] if self.mission==5 else [])

    def fresh(self,now):
        return (self.scan is not None and all(self.scan.get(k) is not None for k in ('front','left','right','rear','fl','fr'))
                and all(0<=now-self.received.get(k,-math.inf)<=self.param('timeout') for k in self.requirements()))

    def enable(self,req,res):
        if not req.data:
            self.sim_pending=False
            self.armed=False;self.pub.publish(Twist());res.success=True;res.message='Désarmé';return res
        if self.param('simulation') and (not self.sim_session or not self.sim_session['running']):
            res.success=False;res.message='Simulation en pause ; utiliser Démarrer';return res
        if self.mission==1 and self.param('phase')=='stop' and not self.param('simulation'):
            res.success=False;res.message='Détecteur STOP réel non intégré ; phase checker disponible';return res
        if self.mission==3 and self.param('route')!='straight':
            res.success=False;res.message='Seule la route straight expérimentale est implémentée';return res
        if self.mission==5 and not self.param('storage_calibrated'):
            res.success=False;res.message='Localisation des zones et pilote aimant à intégrer';return res
        res.success=self.fresh(time.monotonic()) and self.brain.state not in ('DONE','FAULT','TIMEOUT')
        self.armed=res.success;res.message='Armé' if res.success else 'Capteurs absents/périmés ou état terminal';return res

    def reset(self,req,res):
        if self.armed or self.brain.magnet:
            res.success=False;res.message='Désarmer ; ne pas réinitialiser avec une charge tenue';return res
        self.brain=self.new_brain();self.lidar_barrier_seen=False;self.have_fresh=False
        res.success=True;res.message='Mission réinitialisée, toujours désarmée';return res

    def tick(self):
        now=time.monotonic();fresh=self.fresh(now)
        if self.armed and self.have_fresh and not fresh:
            self.armed=False;self.sim_pending=False
            self.get_logger().warning("Capteurs périmés : désarmé, réarmement manuel requis")
        self.have_fresh=self.have_fresh or fresh
        if self.param('simulation') and self.sim_pending and fresh:
            self.sim_pending=False
            self.armed=self.brain.state not in ('DONE','FAULT','TIMEOUT')
        if self.param('simulation') and (not self.sim_session or not self.sim_session['running']):self.armed=False
        o=dict(self.events if self.mission==5 else self.vision)
        o.update(self.scan or {});o.update(fresh=fresh,x=self.distance,pose=self.pose,
            route_confirmed=self.param('route')=='straight',storage_calibrated=self.param('storage_calibrated'))
        if self.mission==2:
            if fresh and o.get('barrier') and (o.get('front') or 0)<.6:self.lidar_barrier_seen=True
            o['barrier_clear']=bool(self.lidar_barrier_seen and o.get('barrier_clear') and o.get('lane_valid'))
        if self.armed:command=self.brain.step(o,self.sim_session['t'] if self.param('simulation') else now)
        else:command=self.brain.result()
        command=guard(command,self.scan,self.armed,fresh)
        msg=Twist();msg.linear.x=float(command.v);msg.angular.z=float(command.w);self.pub.publish(msg)
        self.magnet.publish(Bool(data=bool(self.brain.magnet)))
        self.status.publish(String(data=json.dumps(dict(mission=self.mission,state=self.brain.state,
            armed=self.armed,fresh=fresh,v=command.v,w=command.w,magnet=self.brain.magnet))))


def run(mission):
    rclpy.init();node=None
    try:
        node=MissionNode(mission);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:
            if rclpy.ok():node.pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
