#!/usr/bin/env python3
"""Publish a namespaced initial pose safely inside the simulation TF cache."""
import argparse
from math import cos, sin
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from turtlebot_fleet_sim.fleet_config import load_fleet_config

parser = argparse.ArgumentParser()
parser.add_argument('--fleet-config', required=True)
parser.add_argument('--robot', required=True)
args = parser.parse_args()
config = load_fleet_config(args.fleet_config)
robot = next((item for item in config.robots if item.name == args.robot), None)
if robot is None:
    raise RuntimeError(f'robot {args.robot!r} is absent from {config.source}')
rclpy.init()
node = rclpy.create_node('localization_smoke_initialpose', namespace=args.robot)
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
publisher = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
deadline = time.monotonic() + 30.0
while node.get_clock().now().nanoseconds < 500_000_000 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
now = node.get_clock().now()
if now.nanoseconds < 500_000_000:
    raise RuntimeError('simulation clock unavailable')
while publisher.get_subscription_count() < 2 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
matched = publisher.get_subscription_count()
if matched < 2:
    raise RuntimeError(f'expected health and AMCL initialpose subscriptions, found {matched}')
message = PoseWithCovarianceStamped()
message.header.frame_id = f'{args.robot}/map'
message.pose.pose.position.x = robot.spawn.x
message.pose.pose.position.y = robot.spawn.y
message.pose.pose.orientation.z = sin(robot.spawn.yaw / 2.0)
message.pose.pose.orientation.w = cos(robot.spawn.yaw / 2.0)
message.pose.covariance[0] = message.pose.covariance[7] = 0.25
message.pose.covariance[35] = 0.068
publish_until = time.monotonic() + 2.0
while time.monotonic() < publish_until:
    message.header.stamp = (node.get_clock().now() - Duration(seconds=0.2)).to_msg()
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.1)
time.sleep(0.3)
node.destroy_node()
rclpy.shutdown()
