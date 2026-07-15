"""Launch one selected robot in localization or exclusive fresh-mapping mode."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleet_localization.interfaces import resolve_robot
from fleet_localization.map_catalog import resolve_map
from fleet_localization.map_store_lock import MapStoreLock
from fleet_localization.operator_config import render_rviz
from turtlebot_fleet_sim.fleet_config import load_fleet_config


def _boolean(value, name):
    lowered=value.lower()
    if lowered not in ('true','false'): raise ValueError(f'{name} must be true or false, got {value!r}')
    return lowered == 'true'


def activation_after_readiness(returncode, selected_map, nodes):
    """Return the gated localization graph only after rechecking map immutability."""
    if returncode != 0:
        return None
    selected_map.assert_unchanged()
    return nodes


def setup(context):
    config=LaunchConfiguration('fleet_config').perform(context)
    robot=LaunchConfiguration('robot').perform(context)
    mode=LaunchConfiguration('mode').perform(context)
    map_override=LaunchConfiguration('map_id').perform(context)
    timeout=float(LaunchConfiguration('timeout').perform(context))
    use_rviz=_boolean(LaunchConfiguration('rviz').perform(context),'rviz')
    if mode not in ('localization','mapping'):
        return [LogInfo(msg=f'ERROR: mode must be localization or mapping, got {mode!r}'),EmitEvent(event=Shutdown(reason='unsupported mode'))]
    interface=resolve_robot(config,robot)
    manifest=load_fleet_config(config)
    selected_map=resolve_map(config,map_override) if mode == 'localization' else None
    lock=(MapStoreLock.acquire_shared(manifest.map_store) if mode == 'localization'
          else MapStoreLock.acquire_exclusive(manifest.map_store))
    share=Path(get_package_share_directory('fleet_localization'))
    profile=yaml.safe_load((share/'config'/'burger_sim.yaml').read_text())
    ekf_parameters=dict(profile['ekf_filter_node']['ros__parameters'])
    ekf_parameters.update({
        'use_sim_time':True,'odom_frame':interface.frame('odom'),'world_frame':interface.frame('odom'),
        'base_link_frame':interface.frame('base_footprint'),'odom0':'wheel/odometry','imu0':'imu'})
    amcl_parameters=dict(profile['amcl']['ros__parameters'])
    amcl_parameters.update({
        'use_sim_time':True,'global_frame_id':interface.frame('map'),'odom_frame_id':interface.frame('odom'),
        'base_frame_id':interface.frame('base_footprint'),'scan_topic':'scan','tf_broadcast':True})
    rviz_template='robot_localization.rviz' if mode == 'localization' else 'robot_mapping.rviz'
    rviz_path=render_rviz(share/'rviz'/rviz_template,robot,interface.namespace,interface.prefix) if use_rviz else None

    ready=Node(package='fleet_localization',executable='readiness',name='readiness',namespace=interface.namespace,
        parameters=[{'fleet_config':config,'robot':robot,'timeout':timeout,'use_sim_time':True}],output='screen')
    ekf=Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',namespace=interface.namespace,
        parameters=[ekf_parameters],remappings=[('odometry/filtered','odometry/filtered')],output='screen')
    map_server=Node(package='nav2_map_server',executable='map_server',name='map_server',namespace=interface.namespace,
        parameters=[{'use_sim_time':True,'yaml_filename':str(selected_map.yaml_path) if selected_map else '',
            'topic_name':'map','frame_id':interface.frame('map')}],output='screen')
    amcl=Node(package='nav2_amcl',executable='amcl',name='amcl',namespace=interface.namespace,
        parameters=[amcl_parameters],output='screen')
    lifecycle=Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',namespace=interface.namespace,
        parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl'],'bond_timeout':4.0}],output='screen')
    health=Node(package='fleet_localization',executable='health',name='localization_health',namespace=interface.namespace,
        parameters=[{'fleet_config':config,'robot':robot,'mode':mode,'use_sim_time':True}],output='screen')
    rviz=Node(package='rviz2',executable='rviz2',name=f'{robot}_{mode}_rviz',namespace=interface.namespace,
        arguments=['-d',str(rviz_path)],parameters=[{'use_sim_time':True}],output='screen') if rviz_path else None

    slam_profile=yaml.safe_load((share/'config'/'burger_slam_async.yaml').read_text())
    slam_parameters=dict(slam_profile['slam_toolbox']['ros__parameters'])
    slam_parameters.update({'use_sim_time':True,'map_frame':interface.frame('map'),
        'odom_frame':interface.frame('odom'),'base_frame':interface.frame('base_footprint'),
        'scan_topic':'scan','use_lifecycle_manager':True})
    slam=Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
        namespace=interface.namespace,parameters=[slam_parameters],
        remappings=[('/map','map'),('/map_metadata','map_metadata')],output='screen')
    slam_lifecycle=Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_mapping',namespace=interface.namespace,
        parameters=[{'use_sim_time':True,'autostart':True,'node_names':['slam_toolbox'],'bond_timeout':4.0}],output='screen')

    def readiness_exited(event, _context):
        nodes=([ekf,map_server,amcl,lifecycle,health] if mode == 'localization'
               else [ekf,slam,slam_lifecycle,health])
        if rviz is not None: nodes.append(rviz)
        activated=(activation_after_readiness(event.returncode,selected_map,nodes)
                   if selected_map is not None else (nodes if event.returncode == 0 else None))
        if activated is not None: return activated
        return [LogInfo(msg=f'ERROR: readiness failed with exit code {event.returncode}'),EmitEvent(event=Shutdown(reason='readiness failed'))]
    def localization_exited(event, _context):
        return [LogInfo(msg=f'ERROR: localization process exited with code {event.returncode}'),EmitEvent(event=Shutdown(reason='localization process exited'))]
    def cleanup(_event, _context):
        lock.release()
        if rviz_path is not None: rviz_path.unlink(missing_ok=True)

    monitored=([map_server,amcl,lifecycle] if mode == 'localization' else [slam,slam_lifecycle])
    handlers=[RegisterEventHandler(OnProcessExit(target_action=ready,on_exit=readiness_exited)),
        *[RegisterEventHandler(OnProcessExit(target_action=node,on_exit=localization_exited)) for node in monitored],
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup))]
    return [ready,*handlers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet_config'),DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('mode',default_value='localization'),DeclareLaunchArgument('map_id',default_value=''),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('timeout',default_value='60.0'),
        OpaqueFunction(function=setup)])
