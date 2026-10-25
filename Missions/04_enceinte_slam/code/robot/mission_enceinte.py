"""Mission 4: SLAM or local LiDAR/odometry, with optional camera test assistance."""
# Works both from the mission folder and from the stable kit_robot entry point.
from pathlib import Path as _LayoutPath
import sys as _layout_sys
_WORKSPACE=next((p for p in _LayoutPath(__file__).resolve().parents if (p/'kit_robot').is_dir() and (p/'src').is_dir()),None)
if _WORKSPACE is None:raise RuntimeError('Workspace URMC introuvable autour du script.')
_layout_sys.path.insert(0,str(_WORKSPACE/'kit_robot'))
if __name__=='__main__':
    from operator_bootstrap import bootstrap
    bootstrap(__file__)
import base64,concurrent.futures,fcntl,json,math,os,queue,secrets,signal,subprocess,threading,time,uuid
from datetime import datetime,timezone
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np,rclpy,yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy,qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan,BatteryState,CompressedImage
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from ament_index_python.packages import get_package_share_directory
from urmc_core.enclosure_map import Grid,astar,plan_goal,path_command,swept_clear,validate_route,bounded_grid
from urmc_core.live_navigation import LocalView,LiveGridLayer,CameraWatch,validate_destination
from urmc_core.operator_settings import current
from map_store import MapStore,slug
ROOT=_WORKSPACE

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

def stop_process(p):
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=2)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

class EnclosureNode(Node):
    def __init__(self,root=ROOT,simulation=False):
        super().__init__('urmc_enclosure_worker');self.root=Path(root);self.simulation=simulation
        self.prefix='/urmc_sim/m04_slam' if simulation else '';self.topic=lambda name:self.prefix+'/'+name
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.grid=None;self.map_id=None;self.mode='off';self.selected=None;self.route={};self.zone=None;self.navigation_grid=None;self.revision=0;self.active=False;self.confirmed=False
        self.task='enclosure';self.local_view=None;self.local_origin=None;self.local_fault=False;self.odom_pose=None;self.odom_motion=None;self.measured_speed=0.;self.measured_rotation=0.;self.live_layer=LiveGridLayer();self.camera_enabled=False;self.camera_watch=CameraWatch();self.camera_sub=None
        self.phase='IDLE';self.reason='Créer une carte ou charger une carte existante.';self.pub=None;self.locks=[]
        self.raw_scan=None;self.scan_received=0.;self.odom_received=0.;self.odom_stamp=0.;self.pose=None;self.pose_stamp=0.;self.local_points=[];self.world_points=[]
        self.scan_good=False;self.sensor_reason='Attente LiDAR / TF';self.map_received=0.;self.previous_pose=None;self.path=[];self.plan=None;self.plan_time=0.;self.plan_target=None;self.last_replan=0.;self.epoch=0
        self.pool=concurrent.futures.ThreadPoolExecutor(max_workers=1);self.commands=queue.Queue();self.action_lock=threading.Lock();self.snapshot_lock=threading.Lock();self.cached={};self.png_revision=-1;self.png=None
        self.started=None;self.entered=None;self.finished_at=None;self.v=self.w=0.;self.last_status=0.;self.last_snapshot=0.;self.job='';self.battery=None;self.seed_time=None
        self.manual_deadline=0.;self.manual_seq=-1;self.manual_expired=None;self.manual_hold=None
        self.slam_process=None;self.store=MapStore(self.root/'cartes');self.sessions=self.root/'session_robot11';self.sessions.mkdir(exist_ok=True)
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);self.sensor_qos=latest
        map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan,self.topic('scan'),self.scan,latest)
        self.create_subscription(Odometry,self.topic('odom'),self.odom,latest)
        self.create_subscription(OccupancyGrid,self.topic('map'),self.map,map_qos)
        self.create_subscription(BatteryState,self.topic('battery_state'),lambda m:setattr(self,'battery',m.voltage if math.isfinite(m.voltage) else None),latest)
        self.status=self.create_publisher(String,self.topic('urmc/enclosure/status'),1)
        self.initial_pose=self.create_publisher(PoseWithCovarianceStamped,self.topic('initialpose'),1)
        self.create_timer(.05,self.tick)
        try:self.slam_share=Path(get_package_share_directory('slam_toolbox'))
        except Exception:self.slam_share=None
    def ros(self,fn,timeout=4):
        event=threading.Event();box={};self.commands.put((fn,event,box))
        if not event.wait(timeout):raise ValueError('Commande ROS non confirmée dans le délai.')
        if 'error' in box:raise ValueError(box['error'])
        return box.get('result',{'success':True})
    def scan(self,msg):
        age=self.get_clock().now().nanoseconds/1e9-stamp(msg)
        if not 0<=age<.3 or msg.angle_increment<=0 or len(msg.ranges)<90 or not math.isfinite(msg.range_max) or not 0<msg.range_min<msg.range_max:self.scan_good=False;return
        if self.raw_scan is not None and stamp(msg)<=stamp(self.raw_scan):return
        valid=sum(r==math.inf or (math.isfinite(r) and msg.range_min<=r<=msg.range_max) for r in msg.ranges)
        coverage=msg.angle_increment*len(msg.ranges)
        self.scan_good=valid/len(msg.ranges)>=.90 and coverage>=math.radians(350)
        self.raw_scan=msg;self.scan_received=time.monotonic()
    def odom(self,msg):
        age=self.get_clock().now().nanoseconds/1e9-stamp(msg);p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not 0<=age<.3 or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>.05:self.odom_received=0.;return
        value=stamp(msg)
        if value<=self.odom_stamp:return
        pose=(p.x,p.y,yaw(q))
        if self.odom_motion:
            old,t=self.odom_motion;dt=value-t
            if dt>0:
                self.measured_speed=math.dist(pose[:2],old[:2])/dt;self.measured_rotation=abs(math.atan2(math.sin(pose[2]-old[2]),math.cos(pose[2]-old[2])))/dt
        self.odom_motion=(pose,value);self.odom_pose=pose;self.odom_received=time.monotonic();self.odom_stamp=value
    def map(self,msg):
        if self.mode=='local':return
        try:
            if msg.header.frame_id!='map':raise ValueError('Repère de carte inattendu.')
            p=msg.info.origin.position;q=msg.info.origin.orientation
            grid=Grid(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),msg.info.resolution,(p.x,p.y,yaw(q)))
            self.grid=grid;self.revision+=1;self.map_received=time.monotonic()
        except Exception as e:self.reason='Carte refusée : '+str(e)
    def working_grid(self):
        if self.grid is None:return None
        return self.live_layer.apply(self.grid) if self.mode=='localization' else self.grid
    def camera_state(self):
        return self.camera_watch.report(time.monotonic(),current()['m4_camera_light']) if self.camera_enabled else dict(enabled=False,allowed=True,state='DESACTIVEE')
    def camera(self,msg):
        clock=self.get_clock().now().nanoseconds/1e9
        if not self.camera_enabled or not 0<=clock-stamp(msg)<.35:return
        try:
            import cv2
            image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
            if image is None:return
            moving=abs(self.v)>.005 or abs(self.w)>.03 or self.measured_speed>.01 or self.measured_rotation>.03
            self.camera_watch.image(image,time.monotonic(),stamp(msg),moving=moving,change_threshold=current()['m4_camera_change'])
        except Exception:pass
    def set_camera(self,data):
        if type(data.get('enabled')) is not bool:raise ValueError('Booléen attendu pour l’assistance caméra.')
        if self.pub or self.active:raise ValueError('Arrêter avant de changer l’assistance caméra.')
        self.camera_enabled=data['enabled'];self.camera_watch=CameraWatch()
        if self.camera_sub is not None:self.destroy_subscription(self.camera_sub);self.camera_sub=None
        if self.camera_enabled:self.camera_sub=self.create_subscription(CompressedImage,self.topic('image_raw/compressed'),self.camera,self.sensor_qos)
        return {'success':True,'message':'Assistance caméra d’essai activée ; attendre une image récente.' if self.camera_enabled else 'Assistance caméra désactivée : navigation LiDAR uniquement.'}
    def sensors(self):
        now=time.monotonic();clock=self.get_clock().now().nanoseconds/1e9
        if not self.scan_good or self.raw_scan is None or not 0<=now-self.scan_received<.3 or not 0<=clock-stamp(self.raw_scan)<.3:return False,'LiDAR absent, périmé ou couverture incomplète'
        if self.odom_pose is None or not 0<=now-self.odom_received<.3 or not 0<=clock-self.odom_stamp<.3:return False,'Odométrie absente ou périmée'
        try:
            if self.mode=='local':
                if self.local_origin is None or self.local_fault:return False,'Vue locale à réinitialiser après changement d’odométrie'
                ox,oy,oa=self.local_origin;dx,dy=self.odom_pose[0]-ox,self.odom_pose[1]-oy;c,s=math.cos(oa),math.sin(oa)
                pose=(dx*c+dy*s,-dx*s+dy*c,math.atan2(math.sin(self.odom_pose[2]-oa),math.cos(self.odom_pose[2]-oa)));pose_stamp=self.odom_stamp
                if math.hypot(pose[0],pose[1])>3.:
                    self.stop_navigation('FAULT','Limite de 3 m de la vue locale atteinte');self.local_fault=True;return False,'Recommencer une vue locale ou utiliser le SLAM'
            else:
                trans=self.tf.lookup_transform('map','base_footprint',Time());age=clock-stamp(trans)
                if not -.1<=age<.3:return False,'Position TF périmée'
                p=trans.transform.translation;q=trans.transform.rotation;pose=(p.x,p.y,yaw(q));pose_stamp=stamp(trans)
            if not all(math.isfinite(v) for v in pose):return False,'Pose non finie'
            laser=self.tf.lookup_transform('base_footprint',self.raw_scan.header.frame_id,Time()).transform
            lyaw=yaw(laser.rotation);lx,ly=laser.translation.x,laser.translation.y
            jumped=self.previous_pose and (math.dist(pose[:2],self.previous_pose[:2])>.12 or abs(math.atan2(math.sin(pose[2]-self.previous_pose[2]),math.cos(pose[2]-self.previous_pose[2])))>.35)
            if jumped and (self.active or self.mode=='local'):
                self.stop_navigation('FAULT','Saut de localisation : vérifier la position avant de relancer.');self.confirmed=False
                if self.mode=='local':self.local_fault=True;return False,'Vue locale invalidée par un saut d’odométrie'
            self.previous_pose=pose;self.pose=pose;self.pose_stamp=pose_stamp
            c,s=math.cos(pose[2]),math.sin(pose[2]);origin=(pose[0]+lx*c-ly*s,pose[1]+lx*s+ly*c)
            points=[];rays=[]
            for i,value in enumerate(self.raw_scan.ranges):
                if value!=math.inf and not (math.isfinite(value) and self.raw_scan.range_min<=value<=self.raw_scan.range_max):continue
                r=min(value,self.raw_scan.range_max);a=self.raw_scan.angle_min+i*self.raw_scan.angle_increment+lyaw
                x,y=lx+r*math.cos(a),ly+r*math.sin(a);wx,wy=pose[0]+x*c-y*s,pose[1]+x*s+y*c
                occupied=math.isfinite(value) and value<self.raw_scan.range_max-.02
                rays.append((wx,wy,occupied))
                if occupied:points.append((x,y))
            self.local_points=points;self.world_points=[(pose[0]+x*c-y*s,pose[1]+x*s+y*c) for x,y in points]
            if self.mode=='local' and self.local_view is not None:
                if self.local_view.observe(origin,rays,stamp(self.raw_scan)):self.grid=self.local_view.grid;self.revision+=1
            elif self.mode=='localization' and self.grid is not None:self.live_layer.observe(self.grid,origin,rays,stamp(self.raw_scan))
            return True,'Capteurs et position récents'
        except Exception:return False,'TF map → base_footprint ou base → LiDAR indisponible' if self.mode!='local' else 'Transformation base → LiDAR indisponible'
    def acquire_motors(self):
        for name in ('.camera_follow.lock','.barrier.lock'):
            f=(self.root/name).open('w')
            try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:
                f.close()
                for h in self.locks:h.close()
                self.locks=[];raise ValueError('Un autre pilote commande le robot.')
            self.locks.append(f)
        if self.count_publishers(self.topic('cmd_vel')):
            for h in self.locks:h.close()
            self.locks=[];raise ValueError('Un autre émetteur de vitesse est présent.')
        self.pub=self.create_publisher(Twist,self.topic('cmd_vel'),1)
    def stop_navigation(self,phase='STOP',reason='Arrêt demandé'):
        if self.active and self.entered is not None:self.finished_at=time.monotonic()
        if self.pub:
            for _ in range(3):self.pub.publish(Twist())
            self.destroy_publisher(self.pub);self.pub=None
        for h in self.locks:h.close()
        self.locks=[];self.active=False;self.manual_deadline=0.;self.epoch+=1;self.path=[];self.v=self.w=0.;self.phase=phase;self.reason=reason
        return {'success':True,'message':reason}
    def run_navigation(self,map_id,task='enclosure'):
        self.check_id(map_id);fresh,why=self.sensors()
        if not fresh:raise ValueError(why)
        if task not in ('enclosure','goal'):raise ValueError('Choisir parcours complet ou point d’arrivée.')
        if self.mode not in ('localization','local') or not self.confirmed:raise ValueError('Charger une carte et confirmer la position, ou activer la vue sans SLAM.')
        if not self.camera_state()['allowed']:raise ValueError('Assistance caméra : attendre une image claire, récente et stable.')
        if self.grid is None:raise ValueError('Aucune carte utilisable.')
        self.navigation_grid=bounded_grid(self.working_grid(),self.zone)
        if task=='enclosure':self.route=validate_route(self.route,self.navigation_grid,self.radius())
        else:validate_destination(self.route,self.navigation_grid,self.radius())
        if self.mode=='local' and any(math.hypot(*point)>3. for point in (self.route.values() if task=='enclosure' else [self.route['exit']])):raise ValueError('Sans SLAM, garder les objectifs à moins de 3 m de l’origine.')
        self.task=task;self.plan_target=None
        self.stop_navigation();self.acquire_motors();self.active=True;self.phase='GOAL' if task=='goal' else 'ENTRY';self.started=time.monotonic();self.entered=self.started if task=='goal' else None;self.finished_at=None;self.last_replan=0.;self.plan=None;self.epoch+=1;self.reason='Calcul du trajet vers l’arrivée' if task=='goal' else 'Calcul du trajet vers l’entrée'
        return {'success':True,'message':'Navigation vers le point d’arrivée démarrée.' if task=='goal' else 'Mission 4 démarrée : entrée → intérieur → sortie.'}
    def radius(self):cfg=current();return cfg['m4_radius']+cfg['m4_margin']
    def check_id(self,map_id):
        if not self.map_id or map_id!=self.map_id:raise ValueError('La carte a changé. Rafraîchir avant de commander.')
    def set_route(self,data):
        self.check_id(data.get('map_id'))
        if self.active:raise ValueError('Arrêter les moteurs avant de modifier les repères.')
        key=data.get('key');point=data.get('point')
        if key not in ('entry','inside','exit') or not isinstance(point,list) or len(point)!=2 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in point):raise ValueError('Repère invalide.')
        if self.grid is None:raise ValueError('Créer ou charger une carte.')
        grid=self.working_grid();p=grid.cell(*point)
        if not grid.inside(p) or grid.blocked(self.radius()+(.045 if key=='exit' else 0.))[p[1],p[0]]:raise ValueError('Point inconnu, occupé ou trop proche d’un mur.')
        if self.mode=='local' and math.hypot(*point)>3.:raise ValueError('Choisir une arrivée à moins de 3 m de l’origine locale.')
        self.route[key]=point
        if self.selected:
            folder=self.store.folder(self.selected);meta=json.loads((folder/'metadata.json').read_text());meta['route']=self.route;self.store.metadata(folder,meta)
        return {'success':True}
    def seed(self,data):
        self.check_id(data.get('map_id'))
        if self.mode=='local':raise ValueError('Sans SLAM, l’origine est l’odométrie de départ ; recommencer la vue pour la réinitialiser.')
        if self.active:raise ValueError('Arrêter avant de relocaliser.')
        point=data.get('pose')
        if not isinstance(point,list) or len(point)!=3 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in point):raise ValueError('Pose invalide.')
        p=self.grid.cell(*point[:2]) if self.grid else None
        if p is None or not self.grid.inside(p):raise ValueError('Pose hors carte.')
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg();msg.pose.pose.position.x=float(point[0]);msg.pose.pose.position.y=float(point[1]);msg.pose.pose.orientation.z=math.sin(point[2]/2);msg.pose.pose.orientation.w=math.cos(point[2]/2);msg.pose.covariance[0]=msg.pose.covariance[7]=.04;msg.pose.covariance[35]=.07
        self.initial_pose.publish(msg);self.confirmed=False;self.seed_time=time.monotonic();self.previous_pose=None
        return {'success':True,'message':'Position proposée. Vérifier le repère et les scans, puis confirmer.'}
    def confirm(self,map_id):
        self.check_id(map_id);fresh,why=self.sensors()
        if not fresh:raise ValueError(why)
        if self.seed_time is None or time.monotonic()-self.seed_time<2:raise ValueError('Définir la pose et attendre deux secondes avant de confirmer.')
        self.confirmed=True;return {'success':True,'message':'Position confirmée par l’opérateur.'}
    def sim_manual(self,data):
        if not self.simulation:raise ValueError('Utiliser le mode manuel supervisé du robot.')
        seq=data.get('seq')
        if not isinstance(seq,int) or isinstance(seq,bool) or seq<=self.manual_seq:raise ValueError('Commande ancienne.')
        self.manual_seq=seq
        if data.get('active') is not True:return self.stop_navigation()
        if self.active:raise ValueError('Arrêter la mission avant le manuel.')
        fresh,why=self.sensors()
        if not fresh:raise ValueError(why)
        hold=data.get('hold');x=data.get('x');w=data.get('yaw')
        if not isinstance(hold,str) or not hold or hold==self.manual_expired:raise ValueError('Nouvelle pression nécessaire.')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>1 for v in (x,w)):raise ValueError('Direction invalide.')
        if self.pub is None:self.acquire_motors()
        self.phase='MANUAL';self.manual_hold=hold;self.manual_deadline=time.monotonic()+.25;self.v=x*.10;self.w=w*.4
        return {'success':True}
    def tick(self):
        for _ in range(10):
            try:fn,event,box=self.commands.get_nowait()
            except queue.Empty:break
            try:box['result']=fn()
            except Exception as e:box['error']=str(e)
            event.set()
        now=time.monotonic();fresh,self.sensor_reason=self.sensors();v=w=0.;cfg=current()
        if self.pub and self.count_publishers(self.topic('cmd_vel'))!=1:self.stop_navigation('FAULT','Conflit de commande moteur')
        if self.slam_process is not None and self.slam_process.poll() is not None and self.mode!='off':
            self.stop_navigation('FAULT','SLAM arrêté : consulter cartographie_slam.log');self.mode='off';self.confirmed=False
        if self.active:
            if self.entered is not None and now-self.entered>=60:self.stop_navigation('TIMEOUT','Délai de 60 s écoulé depuis le départ' if self.task=='goal' else 'Délai de 60 s écoulé depuis l’entrée')
            elif not fresh:self.reason=self.sensor_reason;self.path=[]
            elif self.grid is None:self.reason='Carte absente'
            else:
                target=self.plan_target or self.route[{'ENTRY':'entry','INSIDE':'inside','EXIT':'exit','GOAL':'exit'}[self.phase]]
                if math.dist(self.pose[:2],target)<cfg['m4_goal_tolerance']:
                    if self.phase in ('EXIT','GOAL'):self.stop_navigation('DONE','Arrivée atteinte ; robot arrêté')
                    else:
                        if self.phase=='ENTRY':self.entered=now
                        self.phase='INSIDE' if self.phase=='ENTRY' else 'EXIT';self.plan_target=None;self.epoch+=1;self.path=[];self.plan=None;self.last_replan=0.
                if self.active:
                    if self.plan is not None and self.plan.done():
                        try:
                            epoch,planned_at,(path,actual_goal)=self.plan.result()
                            if epoch==self.epoch:self.path=path;self.plan_time=planned_at;self.plan_target=actual_goal
                        except Exception:self.path=[]
                        self.plan=None
                    if self.plan is None and now-self.last_replan>.45:
                        epoch=self.epoch;grid=bounded_grid(self.working_grid(),self.zone);self.navigation_grid=grid;pose=self.pose;points=list(self.world_points);radius=self.radius();target=self.plan_target or self.route[{'ENTRY':'entry','INSIDE':'inside','EXIT':'exit','GOAL':'exit'}[self.phase]]
                        self.plan=self.pool.submit(lambda e,g,p,t,r,o,n,near,local:(e,n,plan_goal(g,p[:2],t,r,o,near,allow_start=local)),epoch,grid,pose,target,radius,points,now,self.phase=='INSIDE',self.mode=='local');self.last_replan=now
                    if self.path and now-self.plan_time<1.2:
                        v,w=path_command(self.pose,self.path,cfg['m4_speed']);self.reason='Trajet recalculé avec les obstacles LiDAR'
                    else:self.reason='Recherche d’un passage connu et libre ; arrêt si aucun trajet'
        elif self.phase=='MANUAL' and self.pub:
            if now>self.manual_deadline:self.manual_expired=self.manual_hold;self.stop_navigation('STOP','Maintien manuel expiré')
            elif fresh:v,w=self.v,self.w
        if self.pub:
            if self.camera_enabled and not self.camera_state()['allowed']:v=w=0.;self.reason='Pause assistance caméra : '+self.camera_state()['state']
            grid=self.navigation_grid if self.active else self.grid
            if self.mode=='local' and self.grid is not None:
                # The planner inflates raster hits; final local collision checking
                # uses exact CURRENT scan points, while preserving unknown space
                # and the authorized boundary. Avoid counting hit cells twice.
                cells=self.grid.data.copy()
                if self.local_view is not None:cells[self.local_view.current_hits]=0
                grid=Grid(cells,self.grid.res,self.grid.origin)
                if self.active:grid=bounded_grid(grid,self.zone)
            if not fresh:v=w=0.;self.reason='Arrêt : données périmées'
            elif not swept_clear(self.pose,v,w,grid,self.radius(),self.local_points,horizon=.35):
                safe=False
                for ratio in (.5,.25,.1,0.):
                    if swept_clear(self.pose,v*ratio,w,grid,self.radius(),self.local_points,horizon=.35):v*=ratio;safe=True;break
                if not safe:v=w=0.
                self.reason='Ralentissement / arrêt : marge autour de l’obstacle'
            self.v,self.w=v,w;msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.pub.publish(msg)
        if now-self.last_snapshot>.25:self.cache_snapshot(fresh);self.last_snapshot=now
        if now-self.last_status>.3:
            self.status.publish(String(data=json.dumps(self.summary())));self.last_status=now
    def summary(self):
        return dict(available=True,simulation=self.simulation,slam_available=self.slam_share is not None,active=self.active,phase=self.phase,reason=self.reason,mode=self.mode,navigation_mode='local' if self.mode=='local' else 'slam',task=self.task,camera=self.camera_state(),map_id=self.map_id,selected=self.selected,confirmed=self.confirmed,v=self.v,w=self.w,job=self.job,robot_id=int(os.environ.get('URMC_ROBOT_ID','11')))
    def cache_snapshot(self,fresh):
        data=self.summary();data.update(pose=self.pose,pose_age=max(0.,self.get_clock().now().nanoseconds/1e9-self.pose_stamp) if self.pose_stamp else None,
            scan_age=max(0.,time.monotonic()-self.scan_received) if self.scan_received else None,odom_age=max(0.,time.monotonic()-self.odom_received) if self.odom_received else None,
            fresh=fresh,environment_revision=self.live_layer.revision if self.mode=='localization' else self.revision,cleared_obstacle_cells=self.live_layer.cleared if self.mode=='localization' else 0,destination=self.route.get('exit'),local_limit=3. if self.mode=='local' else None,nearest_obstacle=min((math.hypot(x,y) for x,y in self.local_points),default=None),known_percent=round(float(np.count_nonzero(self.grid.data>=0)/self.grid.data.size*100),1) if self.grid else None,zone=self.zone,route=self.route,path=self.path[::max(1,len(self.path)//250)],points=self.world_points[::max(1,len(self.world_points)//240)],radius=self.radius(),battery_voltage=self.battery,
            remaining=max(0.,60-((self.finished_at or time.monotonic())-self.entered)) if self.entered else 60.,maps=self.store.list())
        if self.grid:
            revision=(self.revision,self.live_layer.revision if self.mode=='localization' else 0)
            if self.png_revision!=revision:self.png=base64.b64encode(self.working_grid().png()).decode();self.png_revision=revision
            data['map']=dict(width=self.grid.w,height=self.grid.h,resolution=self.grid.res,origin=self.grid.origin,png=self.png,revision=list(revision),frame='local_odom' if self.mode=='local' else 'map')
        with self.snapshot_lock:self.cached=data
    def snapshot(self):
        with self.snapshot_lock:return dict(self.cached)
    def slam_config(self,mode,graph=None):
        if self.slam_share is None:raise ValueError('SLAM Toolbox absent. Installer ros-humble-slam-toolbox avec le script fourni.')
        config=yaml.safe_load((self.slam_share/'config/mapper_params_online_async.yaml').read_text())
        params=config['slam_toolbox']['ros__parameters'];params.update(mode='localization' if mode=='localization' else 'mapping',use_sim_time=False,base_frame='base_footprint',map_frame='map',odom_frame='odom',scan_topic=self.topic('scan'),resolution=.03,map_update_interval=1.,minimum_travel_distance=.04,minimum_travel_heading=.07,minimum_time_interval=.1,max_laser_range=3.5,enable_interactive_mode=False,transform_publish_period=.05)
        if graph:params.update(map_file_name=str(graph),map_start_pose=[0.,0.,0.],scan_buffer_size=5)
        path=self.sessions/'slam_active.yaml';path.write_text(yaml.safe_dump(config));return path
    def start_slam(self,mode,data):
        self.ros(lambda:self.stop_navigation())
        stop_process(self.slam_process);self.slam_process=None
        for _ in range(20):
            if not self.count_publishers(self.topic('map')):break
            time.sleep(.1)
        if self.count_publishers(self.topic('map')):raise ValueError('Un autre serveur de carte est actif. L’arrêter avant de créer/recharger une carte.')
        graph=None;grid=meta=None
        if mode=='localization':
            name=slug(data.get('name'));grid,meta=self.store.load(name);graph=self.store.folder(name)/'slam'
            if not graph.with_suffix('.posegraph').exists() or not graph.with_suffix('.data').exists():raise ValueError('Graphe SLAM manquant : refaire puis enregistrer la cartographie.')
        config=self.slam_config(mode,graph)
        def reset():
            self.map_id=uuid.uuid4().hex;self.mode=mode;self.selected=data.get('name') if mode=='localization' else None;self.route=meta.get('route',{}) if meta else {};self.zone=meta.get('zone') if meta else None;self.grid=grid;self.live_layer=LiveGridLayer();self.local_view=None;self.local_origin=None;self.local_fault=False;self.revision+=1;self.confirmed=False;self.seed_time=None;self.previous_pose=None;self.pose=None;self.phase='IDLE';self.reason='Cartographie : parcourir l’enceinte en manuel' if mode=='mapping' else 'Carte chargée : définir puis confirmer la pose'
        self.ros(reset)
        executable='localization_slam_toolbox_node' if mode=='localization' else 'async_slam_toolbox_node'
        args=['ros2','run','slam_toolbox',executable,'--ros-args','--params-file',str(config)]
        if self.simulation:args+=['-r','/map:='+self.topic('map'),'-r','/initialpose:='+self.topic('initialpose'),'-r','/tf:='+self.topic('tf'),'-r','/tf_static:='+self.topic('tf_static'),'-r','/slam_toolbox/serialize_map:='+self.topic('slam_toolbox/serialize_map')]
        with (self.sessions/'cartographie_slam.log').open('a') as log:self.slam_process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(.2)
        if self.slam_process.poll() is not None:raise ValueError('Démarrage SLAM échoué : consulter cartographie_slam.log')
        return {'success':True,'message':'SLAM démarré, attendre la carte et la localisation.'}
    def start_local(self):
        self.ros(lambda:self.stop_navigation());stop_process(self.slam_process);self.slam_process=None
        def reset():
            now=time.monotonic()
            if self.odom_pose is None or not 0<=now-self.odom_received<.3:raise ValueError('Attendre une odométrie récente avant d’activer la vue sans SLAM.')
            self.local_origin=self.odom_pose;self.local_view=LocalView();self.grid=self.local_view.grid;self.live_layer=LiveGridLayer();self.mode='local';self.map_id=uuid.uuid4().hex;self.selected=None;self.route={};self.zone=[-3.5,-3.5,3.5,3.5];self.navigation_grid=None;self.revision+=1;self.previous_pose=None;self.local_fault=False;self.confirmed=True;self.phase='IDLE';self.reason='Vue locale sans SLAM : origine au démarrage, dérive odométrique possible';self.seed_time=None
            return {'success':True,'message':'Vue locale active, sans SLAM. Choisir l’arrivée dans la zone observée, à moins de 3 m.'}
        return self.ros(reset)
    def save_map(self,data):
        self.check_id(data.get('map_id'))
        if self.pub or self.active:raise ValueError('Arrêter les moteurs avant d’enregistrer.')
        if self.mode not in ('mapping','local') or self.grid is None:raise ValueError('Créer une carte ou une vue locale avant d’enregistrer.')
        label=slug(data.get('name'));name=label+'_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        final=self.store.folder(name)
        if final.exists():raise ValueError('Cette version existe déjà. Choisir un autre nom.')
        temp=self.store.root/('.'+name+'_'+secrets.token_hex(3));temp.mkdir()
        try:
            if self.mode=='local':
                meta=dict(created=datetime.now(timezone.utc).isoformat(),robot_id=int(os.environ.get('URMC_ROBOT_ID','11')),route=self.route,zone=self.zone,map_id=self.map_id,format_version=1,navigation_mode='local')
                self.store.write_grid(temp,self.grid,meta);temp.rename(final)
                return {'success':True,'name':name,'message':'Vue locale enregistrée pour consultation ; ce fichier n’est pas un graphe SLAM et ne permet pas de se relocaliser.'}
            from slam_toolbox.srv import SerializePoseGraph
            client=self.create_client(SerializePoseGraph,self.topic('slam_toolbox/serialize_map'))
            try:
                if not client.wait_for_service(timeout_sec=2):raise ValueError('Service de sauvegarde SLAM absent.')
                event=threading.Event();req=SerializePoseGraph.Request();req.filename=str(temp/'slam');future=client.call_async(req);future.add_done_callback(lambda _:event.set())
                if not event.wait(8) or future.result() is None or future.result().result!=0:raise ValueError('Sauvegarde du graphe SLAM échouée.')
            finally:self.destroy_client(client)
            if not (temp/'slam.posegraph').exists() or not (temp/'slam.data').exists():raise ValueError('Graphe SLAM incomplet.')
            meta=dict(created=datetime.now(timezone.utc).isoformat(),robot_id=int(os.environ.get('URMC_ROBOT_ID','11')),route=self.route,zone=self.zone,map_id=self.map_id,format_version=1)
            self.store.write_grid(temp,self.grid,meta);temp.rename(final)
        except Exception:
            import shutil
            shutil.rmtree(temp,ignore_errors=True);raise
        return {'success':True,'name':name,'message':'Carte, image, graphe SLAM et repères enregistrés.'}
    def action(self,data):
        with self.action_lock:
            action=data.get('action')
            if action=='stop':return self.ros(lambda:self.stop_navigation())
            if action=='new':return self.start_slam('mapping',data)
            if action=='local':return self.start_local()
            if action=='camera':return self.ros(lambda:self.set_camera(data))
            if action=='load':return self.start_slam('localization',data)
            if action=='save':return self.save_map(data)
            if action=='route':return self.ros(lambda:self.set_route(data))
            if action=='zone':
                def zone():
                    self.check_id(data.get('map_id'))
                    if self.pub or self.active:raise ValueError('Arrêter les moteurs avant de modifier la zone.')
                    bounded_grid(self.grid,data.get('zone'));self.zone=data['zone']
                    if self.selected:
                        folder=self.store.folder(self.selected);meta=json.loads((folder/'metadata.json').read_text());meta['zone']=self.zone;self.store.metadata(folder,meta)
                    return {'success':True}
                return self.ros(zone)
            if action=='pose':return self.ros(lambda:self.seed(data))
            if action=='confirm':return self.ros(lambda:self.confirm(data.get('map_id')))
            if action=='run':return self.ros(lambda:self.run_navigation(data.get('map_id'),data.get('task','enclosure')))
            if action=='manual':return self.ros(lambda:self.sim_manual(data))
            if action=='obstacles' and self.simulation:
                from rcl_interfaces.srv import SetParameters
                from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
                if type(data.get('enabled')) is not bool:raise ValueError('Booléen attendu.')
                client=self.create_client(SetParameters,'/enclosure_bench/set_parameters');event=threading.Event()
                if not client.wait_for_service(timeout_sec=2):raise ValueError('Banc indisponible.')
                p=Parameter(name='obstacles',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=data['enabled']));future=client.call_async(SetParameters.Request(parameters=[p]));future.add_done_callback(lambda _:event.set())
                if not event.wait(3) or not future.result() or not future.result().results[0].successful:raise ValueError('Perturbation non confirmée.')
                self.destroy_client(client);return {'success':True}
            if action=='close':
                self.ros(lambda:self.stop_navigation());stop_process(self.slam_process);self.slam_process=None;self.mode='off';self.confirmed=False;return {'success':True}
            raise ValueError('Action cartographie inconnue.')
    def shutdown(self):
        self.stop_navigation();stop_process(self.slam_process);self.pool.shutdown(wait=False,cancel_futures=True)

def handler(node,token):
    class Handler(BaseHTTPRequestHandler):
        def respond(self,data,status=200,mime='application/json'):
            body=data if isinstance(data,bytes) else json.dumps(data).encode();self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def authorized(self):return secrets.compare_digest(self.headers.get('Authorization',''),'Bearer '+token)
        def do_GET(self):
            if not self.authorized():return self.respond({'error':'Accès refusé'},403)
            if self.path=='/camera' and node.simulation:
                import urllib.request
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8080/frame.jpg',timeout=.8) as r:return self.respond(r.read(),mime='image/jpeg')
                except Exception:return self.respond({'error':'Image indisponible'},503)
            if self.path=='/state':return self.respond(node.summary())
            if self.path=='/snapshot':return self.respond(node.snapshot())
            if self.path.startswith('/export?'):
                import urllib.parse
                try:return self.respond(node.store.export(urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('name',[''])[0]),mime='application/zip')
                except (ValueError,OSError) as e:return self.respond({'error':str(e)},400)
            return self.respond({'error':'Page inconnue'},404)
        def do_POST(self):
            if not self.authorized():return self.respond({'error':'Accès refusé'},403)
            try:
                n=int(self.headers.get('Content-Length',0))
                if self.path!='/action' or not 0<n<=8192:raise ValueError('Requête invalide.')
                data=json.loads(self.rfile.read(n))
                if not isinstance(data,dict):raise ValueError('Objet attendu.')
                self.respond(node.action(data))
            except (OSError,ValueError,TypeError) as e:self.respond({'error':str(e)},400)
        def log_message(self,*_):pass
    return Handler

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--simulation',action='store_true');args,rosargs=parser.parse_known_args();rclpy.init(args=rosargs)
    node=EnclosureNode(simulation=args.simulation);port=int(os.environ.get('URMC_MAPPING_PORT','8093'))
    token=os.environ.get('URMC_MAP_TOKEN') or (ROOT/'session_robot11/.console_token').read_text().strip()
    server=ThreadingHTTPServer(('0.0.0.0' if args.simulation else '127.0.0.1',port),handler(node,token));threading.Thread(target=server.serve_forever,daemon=True).start();done=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:done.set())
    try:
        while rclpy.ok() and not done.is_set():rclpy.spin_once(node,timeout_sec=.03)
    finally:node.shutdown();server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
