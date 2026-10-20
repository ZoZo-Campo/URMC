from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def make_launch(mission, simulation):
    pkg=f'urmc_mission_{mission:02d}';ns=f'urmc_sim/m{mission:02d}' if simulation else f'urmc/m{mission:02d}'
    cfg=str(Path(get_package_share_directory(pkg))/'config'/'mission.yaml')
    actions=[DeclareLaunchArgument('autostart',default_value='false'),
             DeclareLaunchArgument('phase',default_value='checker'),
             DeclareLaunchArgument('route',default_value='straight' if simulation else 'unconfirmed'),
             DeclareLaunchArgument('image_transport',default_value='compressed'),
             DeclareLaunchArgument('speed',default_value='0.07')]
    params=[cfg,dict(simulation=simulation,auto_arm=False,storage_calibrated=simulation,
        stop_shape_enabled=simulation,phase=LaunchConfiguration('phase'),route=LaunchConfiguration('route'),
        image_transport=LaunchConfiguration('image_transport'),speed=ParameterValue(LaunchConfiguration('speed'),value_type=float))]
    remaps=[] if simulation else [('cmd_vel','/cmd_vel'),('scan','/scan'),('odom','/odom'),
        ('image_raw/compressed','/image_raw/compressed'),('image_raw','/image_raw'),
        ('storage_observations','/urmc/storage_observations'),('magnet_request','/urmc/magnet_request')]
    actions.append(Node(package=pkg,executable='mission',namespace=ns,parameters=params,remappings=remaps,output='screen'))
    if simulation:
        actions.extend([DeclareLaunchArgument('dashboard',default_value='false'),
            Node(package='urmc_core',executable='dashboard',namespace=ns,condition=IfCondition(LaunchConfiguration('dashboard')),output='screen'),
            DeclareLaunchArgument('obstacle',default_value='false'),
            Node(package='urmc_core',executable='simulator',namespace=ns,parameters=[dict(mission=mission,autostart=ParameterValue(LaunchConfiguration('autostart'),value_type=bool),
                phase=LaunchConfiguration('phase'),obstacle=ParameterValue(LaunchConfiguration('obstacle'),value_type=bool))],output='screen')])
    return LaunchDescription(actions)
