from types import SimpleNamespace
import pytest
import yaml

from turtlebot_fleet_sim.fleet_config import FleetConfigError
from turtlebot_fleet_sim.teleop import KEYS, Velocity, select_robot


def manifest(tmp_path):
    path = tmp_path / 'fleet.yaml'
    path.write_text(yaml.safe_dump({
        'fleet': {'map_store': './maps', 'map_id': 'x'},
        'simulation': {'world': 'turtlebot3_world', 'gui': False},
        'robots': [
            {'name': 'robot1', 'namespace': '/robot1', 'frame_prefix': 'robot1', 'model': 'burger', 'spawn': {'x': -1.0, 'y': 0.0}},
            {'name': 'robot2', 'namespace': '/robot2', 'frame_prefix': 'robot2', 'model': 'burger', 'spawn': {'x': 1.0, 'y': 0.0}},
        ]}), encoding='utf-8')
    return path


def test_selection_is_explicit_and_unknown_robot_fails_before_node(tmp_path):
    assert select_robot(str(manifest(tmp_path)), 'robot2').namespace == '/robot2'
    with pytest.raises(FleetConfigError, match='not present'):
        select_robot(str(manifest(tmp_path)), 'missing')


def test_commands_are_bounded_and_every_motion_has_a_zero_definition():
    assert KEYS['w'] == Velocity(0.18, 0.0)
    assert KEYS['a'] == Velocity(0.0, 0.9)
    assert KEYS[' '] == Velocity()
    assert all(abs(v.linear) <= 0.18 and abs(v.angular) <= 0.9 for v in KEYS.values())


def test_publisher_contract_is_twist_stamped_and_namespaced():
    source = (__import__('pathlib').Path(__file__).parents[1] / 'turtlebot_fleet_sim' / 'teleop.py').read_text()
    assert 'TwistStamped' in source
    assert "create_publisher(TwistStamped, 'cmd_vel'" in source
    assert 'Twist)' not in source
    assert 'message.header.stamp = now.to_msg()' in source
    assert 'simulation clock is not available' in source
    assert 'finally:' in source and 'node.publish(Velocity())' in source
