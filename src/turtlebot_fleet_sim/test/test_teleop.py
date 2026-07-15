import pytest
import yaml
from types import SimpleNamespace

from turtlebot_fleet_sim.fleet_config import FleetConfigError
from turtlebot_fleet_sim import teleop
from turtlebot_fleet_sim.teleop import KEYS, Teleop, Velocity, build_message, run_keyboard, select_robot
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time
import rclpy


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


class FakeStream:
    def __init__(self, keys): self.keys = iter(keys)
    def isatty(self): return True
    def fileno(self): return 42
    def read(self, _count): return next(self.keys)


class FakeNode:
    def __init__(self): self.published = []
    def publish(self, velocity): self.published.append(velocity)


def prepare(monkeypatch, keys):
    stream, node, restored = FakeStream(keys), FakeNode(), []
    monkeypatch.setattr(teleop.termios, 'tcgetattr', lambda _fd: ['terminal-state'])
    monkeypatch.setattr(teleop.termios, 'tcsetattr', lambda *args: restored.append(args))
    monkeypatch.setattr(teleop.tty, 'setcbreak', lambda _fd: None)
    monkeypatch.setattr(teleop.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(teleop.rclpy, 'spin_once', lambda *_args, **_kwargs: None)
    return node, stream, restored


def test_motion_pulse_is_released_and_finally_stopped(monkeypatch):
    node, stream, restored = prepare(monkeypatch, ['w', 'q'])
    selections = iter([([stream], [], []), ([], [], []), ([stream], [], [])])
    monkeypatch.setattr(teleop.select, 'select', lambda *_args: next(selections))
    run_keyboard(node, stream, release_timeout=0.0)
    assert node.published == [KEYS['w'], Velocity(), Velocity()]
    assert restored


@pytest.mark.parametrize('key', ['q', '\x04'])
def test_quit_and_eof_key_always_finish_at_zero(monkeypatch, key):
    node, stream, _ = prepare(monkeypatch, [key])
    monkeypatch.setattr(teleop.select, 'select', lambda *_args: ([stream], [], []))
    run_keyboard(node, stream, release_timeout=0.0)
    assert node.published == [Velocity()]


def test_space_publishes_stop_then_quit_finishes_with_stop(monkeypatch):
    node, stream, _ = prepare(monkeypatch, [' ', 'q'])
    monkeypatch.setattr(teleop.select, 'select', lambda *_args: ([stream], [], []))
    run_keyboard(node, stream, release_timeout=0.0)
    assert node.published == [Velocity(), Velocity()]


@pytest.mark.parametrize('error', [KeyboardInterrupt(), EOFError()])
def test_input_interruption_restores_terminal_and_finally_stops(monkeypatch, error):
    node, stream, restored = prepare(monkeypatch, [])
    monkeypatch.setattr(teleop.select, 'select', lambda *_args: (_ for _ in ()).throw(error))
    with pytest.raises(type(error)):
        run_keyboard(node, stream)
    assert node.published == [Velocity()]
    assert restored


def test_invalid_selection_precedes_ros_and_publisher_creation(tmp_path, monkeypatch):
    called = []
    monkeypatch.setattr(teleop.rclpy, 'init', lambda **_kwargs: called.append('init'))
    monkeypatch.setattr(teleop, 'Teleop', lambda _robot: called.append('publisher'))
    with pytest.raises(FleetConfigError, match='not present'):
        teleop.main(['--fleet-config', str(manifest(tmp_path)), '--robot', 'missing'])
    assert called == []


def test_message_builder_preserves_stamp_motion_and_zero():
    stamp = Time(sec=12, nanosec=34)
    motion = build_message(Velocity(0.18, -0.9), stamp)
    stop = build_message(Velocity(), stamp)
    assert isinstance(motion, TwistStamped)
    assert motion.header.stamp is stamp
    assert (motion.twist.linear.x, motion.twist.angular.z) == (0.18, -0.9)
    assert (stop.twist.linear.x, stop.twist.angular.z) == (0.0, 0.0)


def test_teleop_selected_namespace_topic_type_and_clock_guard(tmp_path, monkeypatch):
    created=[]; published=[]
    monkeypatch.setattr(teleop.Node, 'create_publisher',
        lambda _self,msg_type,topic,qos: created.append((msg_type,topic,qos)) or SimpleNamespace(publish=published.append))
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros-log'))
    rclpy.init()
    node=Teleop(select_robot(str(manifest(tmp_path)),'robot2'))
    try:
        assert node.get_namespace() == '/robot2'
        assert (TwistStamped,'cmd_vel') in [item[0:2] for item in created]
        zero_now=SimpleNamespace(nanoseconds=0,to_msg=Time)
        monkeypatch.setattr(node,'get_clock',lambda:SimpleNamespace(now=lambda:zero_now))
        node.publish(Velocity())
        assert isinstance(published[-1],TwistStamped)
        with pytest.raises(RuntimeError,match='simulation clock is not available'):
            node.publish(Velocity(0.18,0.0))
    finally:
        node.destroy_node(); rclpy.shutdown()
