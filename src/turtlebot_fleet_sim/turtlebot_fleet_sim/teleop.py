"""Explicitly selected, simulation-time stamped, stop-safe keyboard teleoperation."""
from dataclasses import dataclass
import argparse
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from .fleet_config import FleetConfigError, load_fleet_config


@dataclass(frozen=True)
class Velocity:
    linear: float = 0.0
    angular: float = 0.0


KEYS = {
    'w': Velocity(0.18, 0.0), 's': Velocity(-0.18, 0.0),
    'a': Velocity(0.0, 0.9), 'd': Velocity(0.0, -0.9),
    ' ': Velocity(),
}


def select_robot(fleet_config: str, robot_name: str):
    config = load_fleet_config(fleet_config)
    match = next((robot for robot in config.robots if robot.name == robot_name), None)
    if match is None:
        raise FleetConfigError(f'robot {robot_name!r} is not present in {config.source}')
    return match


class Teleop(Node):
    def __init__(self, robot):
        super().__init__('teleop', namespace=robot.namespace)
        self.declare_parameter('use_sim_time', True)
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)

    def publish(self, velocity: Velocity) -> None:
        message = TwistStamped()
        now = self.get_clock().now()
        if velocity != Velocity() and now.nanoseconds == 0:
            raise RuntimeError('simulation clock is not available; refusing nonzero command')
        message.header.stamp = now.to_msg()
        message.twist.linear.x = velocity.linear
        message.twist.angular.z = velocity.angular
        self.publisher.publish(message)


def run_keyboard(node: Teleop, stream=sys.stdin, release_timeout: float = 0.12) -> None:
    """Publish a bounded pulse per key; absence of repeat is treated as release."""
    if not stream.isatty():
        raise RuntimeError('teleop requires an interactive terminal')
    descriptor = stream.fileno()
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        while rclpy.ok():
            ready, _, _ = select.select([stream], [], [], 0.1)
            if not ready:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            key = stream.read(1)
            if key in ('q', '\x04'):
                break
            velocity = KEYS.get(key)
            if velocity is None:
                continue
            node.publish(velocity)
            if velocity != Velocity():
                select.select([], [], [], release_timeout)
                node.publish(Velocity())
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)
        node.publish(Velocity())


def main(args=None):
    parser = argparse.ArgumentParser(description='Stop-safe selected-robot teleoperation')
    parser.add_argument('--fleet-config', required=True)
    parser.add_argument('--robot', required=True)
    parsed, ros_args = parser.parse_known_args(args)
    # Validation deliberately precedes ROS/node/publisher creation.
    robot = select_robot(parsed.fleet_config, parsed.robot)
    rclpy.init(args=ros_args)
    node = Teleop(robot)
    try:
        node.get_logger().info('w/a/s/d move, release timeout stops, space stops, q exits')
        run_keyboard(node)
    except (KeyboardInterrupt, EOFError):
        node.publish(Velocity())
    finally:
        node.publish(Velocity())
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
