"""Humble Nav2 servers isolated from the physical cmd_vel topic."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def nodes(context):
    prefix=LaunchConfiguration('prefix').perform(context);params=LaunchConfiguration('params_file').perform(context)
    ns=prefix+'/urmc_nav2';remaps=[('/tf',prefix+'/tf'),('/tf_static',prefix+'/tf_static'),('cmd_vel',prefix+'/urmc/nav2/cmd_vel')]
    return [Node(package=p,executable=e,name=n,namespace=ns,parameters=[params],remappings=remaps,output='screen') for p,e,n in [('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server')]]+[Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_nav2',namespace=ns,parameters=[{'autostart':True,'node_names':['planner_server','controller_server'],'bond_timeout':5.}],output='screen')]
def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('prefix',default_value=''),DeclareLaunchArgument('params_file'),OpaqueFunction(function=nodes)])
