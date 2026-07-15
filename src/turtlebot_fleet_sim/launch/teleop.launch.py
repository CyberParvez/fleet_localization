"""Launch stop-safe teleoperation for exactly one manifest robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from turtlebot_fleet_sim.teleop import select_robot


def _launch(context):
    fleet = LaunchConfiguration('fleet_config').perform(context)
    name = LaunchConfiguration('robot').perform(context)
    robot = select_robot(fleet, name)
    return [Node(
        package='turtlebot_fleet_sim', executable='teleop', name='teleop',
        namespace=robot.namespace, output='screen', emulate_tty=True,
        arguments=['--fleet-config', fleet, '--robot', name],
        parameters=[{'use_sim_time': True}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet_config'), DeclareLaunchArgument('robot'),
        OpaqueFunction(function=_launch),
    ])

