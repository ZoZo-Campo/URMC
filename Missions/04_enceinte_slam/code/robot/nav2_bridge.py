"""Nav2 planner/controller adapter; only the mission supervisor publishes motor commands."""
import math,time,subprocess
from pathlib import Path

def parameters(cfg,topic):
    radius=cfg['m4_radius']+cfg['m4_margin']+.03;speed=cfg['m4_speed']
    def costmap(local):
        return {'ros__parameters':dict(use_sim_time=False,global_frame='odom' if local else 'map',robot_base_frame='base_footprint',transform_tolerance=.2,update_frequency=5.,publish_frequency=2.,resolution=.03,robot_radius=radius-.03 if local else radius,footprint_padding=0.,rolling_window=local,**({'width':2,'height':2} if local else {}),track_unknown_space=not local,plugins=['obstacle_layer','inflation_layer'] if local else ['static_layer','obstacle_layer','inflation_layer'],static_layer=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic=topic('urmc/nav2/map'),map_subscribe_transient_local=True),obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer',observation_sources='scan',scan=dict(topic=topic('scan'),data_type='LaserScan',marking=True,clearing=True,inf_is_valid=True,obstacle_max_range=3.45,raytrace_max_range=3.5,obstacle_min_range=.1,raytrace_min_range=.1)),inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=radius+.08,cost_scaling_factor=10.),always_send_full_costmap=True)}
    result=dict(planner_server={'ros__parameters':dict(use_sim_time=False,expected_planner_frequency=1.,planner_plugins=['GridBased'],GridBased=dict(plugin='nav2_navfn_planner/NavfnPlanner',tolerance=.03,use_astar=True,allow_unknown=False))},controller_server={'ros__parameters':dict(use_sim_time=False,controller_frequency=10.,odom_topic=topic('odom'),min_x_velocity_threshold=.001,min_y_velocity_threshold=.001,min_theta_velocity_threshold=.001,progress_checker_plugin='progress_checker',goal_checker_plugins=['goal_checker'],controller_plugins=['FollowPath'],progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.03,movement_time_allowance=15.),goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=min(.03,cfg['m4_goal_tolerance']/2),yaw_goal_tolerance=3.14,stateful=True),FollowPath=dict())},global_costmap={'global_costmap':costmap(False)},local_costmap={'local_costmap':costmap(True)})

    result['controller_server']['ros__parameters']['FollowPath']=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',desired_linear_vel=speed,lookahead_dist=.045,min_lookahead_dist=.04,max_lookahead_dist=.15,lookahead_time=1.,rotate_to_heading_angular_vel=.5,transform_tolerance=.2,use_velocity_scaled_lookahead_dist=False,min_approach_linear_velocity=min(.025,speed),approach_velocity_scaling_dist=.15,use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=.6,use_regulated_linear_velocity_scaling=True,use_cost_regulated_linear_velocity_scaling=True,cost_scaling_dist=.20,cost_scaling_gain=1.,inflation_cost_scaling_factor=10.,regulated_linear_scaling_min_radius=.2,regulated_linear_scaling_min_speed=min(.04,speed),use_rotate_to_heading=True,rotate_to_heading_min_angle=.15,max_angular_accel=1.,use_interpolation=True)
    return result

def launch_command(launch,config,prefix):
    args=['ros2','launch',str(launch),'params_file:='+str(config)]
    if prefix:args.append('prefix:='+prefix)
    return args

class Nav2Bridge:
    def __init__(self,node):
        from lifecycle_msgs.srv import GetState
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose,FollowPath
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.states={};self.state_time=0.;self.State=GetState
        self.state_clients={name:node.create_client(GetState,node.topic('urmc_nav2/'+name+'/get_state')) for name in ('planner_server','controller_server')}
        self.node=node;self.Compute=ComputePathToPose;self.Follow=FollowPath
        self.planner=ActionClient(node,ComputePathToPose,node.topic('urmc_nav2/compute_path_to_pose'))
        self.controller=ActionClient(node,FollowPath,node.topic('urmc_nav2/follow_path'))
        self.map_pub=node.create_publisher(OccupancyGrid,node.topic('urmc/nav2/map'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sub=node.create_subscription(Twist,node.topic('urmc/nav2/cmd_vel'),self.velocity,1)
        self.process=None;self.generation=0;self.handle=None;self.pending=False;self.target=None;self.path=[];self.error=None;self.cmd=(0.,0.);self.received=0.;self.last_plan=0.;self.map_time=0.;self.config=None;self.last_gate={};self.proposed=(0.,0.);self.retries=0;self.last_failure=None
    def velocity(self,msg):
        if self.handle is not None and self.target is not None:
            self.cmd=(msg.linear.x,msg.angular.z);self.received=time.monotonic()
    def ready(self):
        alive=self.process is not None and self.process.poll() is None
        if alive and time.monotonic()-self.state_time>.5:
            self.state_time=time.monotonic()
            for name,client in self.state_clients.items():
                if client.service_is_ready():client.call_async(self.State.Request()).add_done_callback(lambda f,n=name:self.lifecycle(f,n))
        return alive and all(self.states.get(n)==3 for n in self.state_clients) and self.planner.server_is_ready() and self.controller.server_is_ready()
    def lifecycle(self,f,name):
        try:self.states[name]=f.result().current_state.id
        except Exception:self.states[name]=0
    def start(self,cfg):
        import yaml
        from ament_index_python.packages import get_package_share_directory
        try:
            for package in ('nav2_planner','nav2_controller','nav2_lifecycle_manager','nav2_navfn_planner','nav2_regulated_pure_pursuit_controller'):get_package_share_directory(package)
        except Exception as exc:raise ValueError('Nav2 incomplet : bash ~/urmc/kit_robot/installer_mission4.sh') from exc
        if self.process is not None and self.process.poll() is None:
            if self.config!=cfg:raise ValueError('Fermer Nav2 puis le préparer à nouveau pour appliquer les paramètres.')
            return
        self.config=dict(cfg);self.error=None;self.states={}
        path=self.node.sessions/'nav2_active.yaml';path.write_text(yaml.safe_dump({self.node.prefix+"/urmc_nav2":parameters(cfg,self.node.topic)}))
        launch=Path(__file__).with_name('nav2_mission.launch.py')
        args=launch_command(launch,path,self.node.prefix)
        with (self.node.sessions/'nav2.log').open('a') as log:self.process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def publish_map(self):
        if self.process is None or self.process.poll() is not None or self.node.grid is None:return
        now=time.monotonic()
        if now-self.map_time<.5:return
        from nav_msgs.msg import OccupancyGrid
        from urmc_core.enclosure_map import bounded_grid
        grid=bounded_grid(self.node.working_grid(),self.node.zone) if self.node.zone else self.node.working_grid()
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.node.get_clock().now().to_msg();m.info.resolution=float(grid.res);m.info.width=grid.w;m.info.height=grid.h
        m.info.origin.position.x=float(grid.origin[0]);m.info.origin.position.y=float(grid.origin[1]);m.info.origin.orientation.z=math.sin(grid.origin[2]/2);m.info.origin.orientation.w=math.cos(grid.origin[2]/2);m.data=grid.data.astype('int8').ravel().tolist();self.map_pub.publish(m);self.map_time=now
    def cancel(self):
        self.generation+=1
        if self.handle is not None:self.handle.cancel_goal_async()
        self.handle=None;self.target=None;self.pending=False;self.path=[];self.cmd=(0.,0.);self.received=0.;self.error=None;self.retries=0;self.last_failure=None
    def command(self,target):
        if self.target!=tuple(target):self.cancel();self.target=tuple(target);self.last_plan=0.
        if not self.ready():self.error='Nav2 indisponible';return 0.,0.
        now=time.monotonic()
        if not self.pending and not self.error and now-self.last_plan>1.:
            from geometry_msgs.msg import PoseStamped
            self.pending=True;self.last_plan=now;epoch=self.generation
            goal=self.Compute.Goal();goal.goal=PoseStamped();goal.goal.header.frame_id='map';goal.goal.header.stamp=self.node.get_clock().now().to_msg();goal.goal.pose.position.x=float(target[0]);goal.goal.pose.position.y=float(target[1]);goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased';goal.use_start=False
            self.planner.send_goal_async(goal).add_done_callback(lambda f:self.planned_goal(f,epoch))
        if self.error or now-self.received>.25 or not all(math.isfinite(x) for x in self.cmd):return 0.,0.
        self.proposed=self.cmd
        return self.cmd
    def planned_goal(self,f,epoch):
        if epoch!=self.generation:return
        try:
            h=f.result()
            if not h.accepted:raise ValueError('Plan refusé par Nav2')
            h.get_result_async().add_done_callback(lambda r:self.planned(r,epoch))
        except Exception as e:self.pending=False;self.error=str(e)
    def planned(self,f,epoch):
        if epoch!=self.generation:return
        try:
            result=f.result()
            if result.status!=4 or not result.result.path.poses:raise ValueError('Nav2 : aucun trajet libre')
            path=result.result.path;self.path=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
            # New FollowPath goals preempt the previous path on the same controller.
            self.generation+=1;tag=self.generation;goal=self.Follow.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
            self.controller.send_goal_async(goal).add_done_callback(lambda r:self.followed(r,tag))
        except Exception as e:self.pending=False;self.error=str(e);self.cmd=(0.,0.)
    def followed(self,f,epoch):
        try:
            h=f.result()
            if epoch!=self.generation:
                if h.accepted:h.cancel_goal_async()
                return
            self.pending=False
            if not h.accepted:raise ValueError('Trajet refusé par le contrôleur Nav2')
            self.handle=h;self.last_failure=None;h.get_result_async().add_done_callback(lambda r:self.finished(r,epoch))
        except Exception as e:self.pending=False;self.error=str(e)
    def finished(self,f,epoch):
        if epoch!=self.generation:return
        try:
            status=f.result().status
            if status not in (4,5):
                self.retries+=1;self.last_failure='Nav2 : arrêt du suivi, nouveau calcul '+str(self.retries)+'/3'
                if self.retries>=3:self.error='Nav2 : trajet interrompu après trois tentatives'
                self.last_plan=time.monotonic();self.pending=False;self.handle=None
        except Exception as e:self.error=str(e)
        self.cmd=(0.,0.);self.received=0.
    def report(self):return dict(process_error='Nav2 arrêté : consulter session_robot11/nav2.log' if self.process is not None and self.process.poll() is not None else None,retries=self.retries,last_failure=self.last_failure,proposed=self.proposed,gate=self.last_gate,available=True,ready=self.ready(),running=self.process is not None and self.process.poll() is None,error=self.error,backend='Nav2',radius=self.config.get('m4_radius') if self.config else None,margin=self.config.get('m4_margin') if self.config else None)
