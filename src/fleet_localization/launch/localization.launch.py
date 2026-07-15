"""Launch exactly one selected robot's readiness-gated local EKF."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fleet_localization.interfaces import resolve_robot

def setup(context):
    config=LaunchConfiguration('fleet_config').perform(context)
    robot=LaunchConfiguration('robot').perform(context)
    mode=LaunchConfiguration('mode').perform(context)
    timeout=LaunchConfiguration('timeout').perform(context)
    if mode != 'localization':
        return [LogInfo(msg=f'ERROR: mode {mode!r} is not implemented in slice 02'),EmitEvent(event=Shutdown(reason='unsupported mode'))]
    interface=resolve_robot(config,robot)
    profile=Path(get_package_share_directory('fleet_localization'))/'config'/'burger_sim.yaml'
    parameters=yaml.safe_load(profile.read_text())['ekf_filter_node']['ros__parameters']
    parameters.update({
        'use_sim_time':True,'odom_frame':interface.frame('odom'),'world_frame':interface.frame('odom'),
        'base_link_frame':interface.frame('base_footprint'),'odom0':'wheel/odometry','imu0':'imu',
    })
    ready=Node(package='fleet_localization',executable='readiness',name='readiness',namespace=interface.namespace,
        parameters=[{'fleet_config':config,'robot':robot,'timeout':float(timeout),'use_sim_time':True}],output='screen')
    ekf=Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',namespace=interface.namespace,
        parameters=[parameters],remappings=[('odometry/filtered','odometry/filtered')],output='screen')
    health=Node(package='fleet_localization',executable='health',name='localization_health',namespace=interface.namespace,
        parameters=[{'fleet_config':config,'robot':robot,'use_sim_time':True}],output='screen')
    def exited(event, _context):
        if event.returncode == 0: return [ekf,health]
        return [LogInfo(msg=f'ERROR: readiness failed with exit code {event.returncode}'),EmitEvent(event=Shutdown(reason='readiness failed'))]
    return [ready,RegisterEventHandler(OnProcessExit(target_action=ready,on_exit=exited))]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet_config'),DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('mode',default_value='localization'),DeclareLaunchArgument('map_id',default_value=''),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('timeout',default_value='60.0'),
        OpaqueFunction(function=setup),
    ])
