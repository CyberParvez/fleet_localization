import importlib.util
import os
from pathlib import Path

os.environ.setdefault('ROS_LOG_DIR', '/tmp/turtlebot_fleet_sim_test_logs')

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import yaml

from turtlebot_fleet_sim.fleet_config import FleetConfigError


def load_launch_module():
    path = Path(__file__).parents[1] / 'launch' / 'fleet_sim.launch.py'
    spec = importlib.util.spec_from_file_location('fleet_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(path: Path, **overrides):
    result = LaunchContext()
    result.launch_configurations.update({
        'fleet_config': str(path), 'world': '', 'gui': '', 'timeout': '60.0', **overrides,
    })
    return result


def test_launch_declares_required_and_optional_arguments():
    description = load_launch_module().generate_launch_description()
    arguments = {
        action.name: action for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert set(arguments) == {'fleet_config', 'world', 'gui', 'timeout'}
    assert arguments['fleet_config'].default_value is None
    assert arguments['world'].default_value[0].text == ''
    assert arguments['gui'].default_value[0].text == ''
    assert arguments['timeout'].default_value[0].text == '60.0'


def test_invalid_manifest_fails_before_any_asset_lookup(tmp_path, monkeypatch):
    manifest = tmp_path / 'bad.yaml'
    manifest.write_text('fleet: {}\nsimulation: {}\nrobots: []\n', encoding='utf-8')
    module = load_launch_module()
    looked_up = []
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: looked_up.append(name))
    with pytest.raises(FleetConfigError, match='fleet.map_store is required'):
        module._launch(context(manifest))
    assert looked_up == []


def test_bridge_uses_stamped_public_command_and_one_data_driven_robot_path(tmp_path, monkeypatch):
    manifest = tmp_path / 'fleet.yaml'
    manifest.write_text(yaml.safe_dump({
        'fleet': {'map_store': './maps', 'map_id': 'world_v1'},
        'simulation': {'world': 'turtlebot3_world', 'gui': False},
        'robots': [{'name': 'r1', 'namespace': '/r1', 'frame_prefix': 'r1',
                    'model': 'burger', 'spawn': {'x': 0.0, 'y': 0.0}}],
    }), encoding='utf-8')
    shares = tmp_path / 'shares'
    (shares / 'tb3_gazebo/models/turtlebot3_burger').mkdir(parents=True)
    (shares / 'tb3_gazebo/worlds').mkdir(parents=True)
    (shares / 'tb3_description/urdf').mkdir(parents=True)
    (shares / 'ros_gz/launch').mkdir(parents=True)
    upstream = Path('/tmp/tb3pkgs/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf')
    if not upstream.exists():
        pytest.skip('extracted official TurtleBot3 2.3.7 SDF is unavailable')
    (shares / 'tb3_gazebo/models/turtlebot3_burger/model.sdf').write_bytes(upstream.read_bytes())
    (shares / 'tb3_gazebo/worlds/turtlebot3_world.world').write_text('<sdf/>', encoding='utf-8')
    (shares / 'tb3_description/urdf/turtlebot3_burger.urdf').write_text('<robot/>', encoding='utf-8')
    module = load_launch_module()
    lookup = {
        'turtlebot3_gazebo': shares / 'tb3_gazebo',
        'turtlebot3_description': shares / 'tb3_description',
        'ros_gz_sim': shares / 'ros_gz',
    }
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(lookup[name]))
    actions = module._launch(context(manifest))
    bridges = [action for action in actions if isinstance(action, Node) and action.node_executable == 'parameter_bridge']
    assert len(bridges) == 1
    bridge_args = [str(value) for value in bridges[0]._Node__arguments]
    assert '/r1/_sim/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist' in bridge_args
    assert '/r1/_sim/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' in bridge_args
    assert '/r1/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model' in bridge_args
    assert not any('geometry_msgs/msg/Twist]' in value for value in bridge_args)
