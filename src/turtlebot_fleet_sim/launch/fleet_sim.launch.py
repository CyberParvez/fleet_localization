"""The sole supported project multi-robot simulation entry point."""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from turtlebot_fleet_sim.fleet_config import FleetConfigError, load_fleet_config
from turtlebot_fleet_sim.model import render_burger_sdf


def _boolean(value: str, field: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise FleetConfigError(f'{field} override must be true or false')


def _launch(context):
    fleet_path = LaunchConfiguration('fleet_config').perform(context)
    world_override = LaunchConfiguration('world').perform(context)
    gui_raw = LaunchConfiguration('gui').perform(context)
    config = load_fleet_config(
        fleet_path,
        world=world_override or None,
        gui=None if gui_raw == '' else _boolean(gui_raw, 'gui'),
    )
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    description_share = get_package_share_directory('turtlebot3_description')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    if config.world == 'asymmetric_indoor':
        package_share = get_package_share_directory('turtlebot_fleet_sim')
        world_path = os.path.join(package_share, 'worlds', 'asymmetric_indoor.sdf')
    else:
        world_path = os.path.join(tb3_share, 'worlds', f'{config.world}.world')
    if not os.path.isfile(world_path):
        raise FleetConfigError(f'world asset is missing: {world_path}')

    actions = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(world_path)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_share, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r -s -v2 {world_path}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ]
    if config.gui:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
        ))

    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    bridge_remaps = []
    upstream_sdf = os.path.join(tb3_share, 'models', 'turtlebot3_burger', 'model.sdf')
    upstream_urdf = os.path.join(description_share, 'urdf', 'turtlebot3_burger.urdf')
    for robot in config.robots:
        sdf = render_burger_sdf(upstream_sdf, robot)
        spawn = Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{robot.name}', output='screen',
            arguments=['-name', robot.name, '-string', sdf,
                       '-x', str(robot.spawn.x), '-y', str(robot.spawn.y),
                       '-z', str(robot.spawn.z), '-Y', str(robot.spawn.yaw)],
        )
        actions.append(spawn)
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot.namespace, name='robot_state_publisher', output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command(['xacro ', upstream_urdf, ' namespace:=', robot.frame_prefix, '/']),
            }],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
        ))
        actions.append(Node(
            package='turtlebot_fleet_sim', executable='sensor_contract',
            namespace=robot.namespace, name='sensor_contract', output='screen',
            parameters=[{'use_sim_time': True, 'frame_prefix': robot.frame_prefix}],
        ))
        for suffix, ros_type, gz_type, direction in (
            ('joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', '['),
            ('_sim/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', '['),
            ('_sim/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', '['),
            ('_sim/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', '['),
            ('_sim/cmd_vel', 'geometry_msgs/msg/TwistStamped', 'gz.msgs.Twist', ']'),
        ):
            topic = f'{robot.namespace}/{suffix}'
            bridge_arguments.append(f'{topic}@{ros_type}{direction}{gz_type}')

    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='fleet_bridge',
        arguments=bridge_arguments, output='screen',
    ))
    readiness = Node(
        package='turtlebot_fleet_sim', executable='fleet_readiness', name='fleet_readiness',
        output='screen', parameters=[{
            'use_sim_time': False,
            'timeout': float(LaunchConfiguration('timeout').perform(context)),
            'robots_json': json.dumps([
                {'namespace': robot.namespace, 'frame_prefix': robot.frame_prefix}
                for robot in config.robots
            ]),
        }],
    )
    actions.extend([
        readiness,
        RegisterEventHandler(OnProcessExit(
            target_action=readiness,
            on_exit=[EmitEvent(event=Shutdown(reason='fleet readiness failed'))],
        )),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet_config', description='Absolute or relative fleet YAML path'),
        DeclareLaunchArgument('world', default_value='', description='Optional supported world override'),
        DeclareLaunchArgument('gui', default_value='', description='Optional true/false GUI override'),
        DeclareLaunchArgument('timeout', default_value='60.0', description='Complete-fleet startup timeout'),
        OpaqueFunction(function=_launch),
    ])
