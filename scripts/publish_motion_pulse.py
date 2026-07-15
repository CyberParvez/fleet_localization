#!/usr/bin/env python3
"""Publish one bounded simulation-time TwistStamped pulse and a final zero."""
import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True)
    parser.add_argument('--seconds', type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mapping_smoke_motion', namespace=args.robot)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(TwistStamped, 'cmd_vel', 10)
    deadline = time.monotonic() + 30.0
    while node.get_clock().now().nanoseconds == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.get_clock().now().nanoseconds == 0:
        raise RuntimeError('simulation clock unavailable')
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        raise RuntimeError('selected robot command subscription unavailable')
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = f'{args.robot}/base_footprint'
            message.twist.linear.x = 0.08
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.header.frame_id = f'{args.robot}/base_footprint'
        publisher.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
