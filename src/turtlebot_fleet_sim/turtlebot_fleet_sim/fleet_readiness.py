"""Complete-fleet startup readiness with itemized timeout diagnostics."""

import json
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, DurabilityPolicy, HistoryPolicy,
                       QoSProfile, ReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


class ReadinessTracker:
    """Pure readiness aggregation used by the ROS observer and deterministic tests."""

    def __init__(self, robots: list[dict[str, str]]) -> None:
        self.missing = {'/clock'}
        for robot in robots:
            namespace, prefix = robot['namespace'], robot['frame_prefix']
            self.missing.update({
                f'{namespace}/wheel/odometry', f'{namespace}/imu', f'{namespace}/scan',
            })
            self.missing.update({
                f'tf:{prefix}/{frame}' for frame in
                ('base_footprint', 'base_link', 'base_scan', 'imu_link',
                 'wheel_left_link', 'wheel_right_link')
            })

    @property
    def ready(self) -> bool:
        return not self.missing

    def seen(self, key: str) -> None:
        self.missing.discard(key)

    def seen_tf(self, child_frame_id: str) -> None:
        self.seen(f'tf:{child_frame_id}')

    def seen_transform(self, parent_frame_id: str, child_frame_id: str) -> None:
        # A TF-tree root (base_footprint here) is observable only as a parent.
        self.seen_tf(parent_frame_id)
        self.seen_tf(child_frame_id)


class FleetReadiness(Node):
    def __init__(self) -> None:
        super().__init__('fleet_readiness')
        self.declare_parameter('robots_json', '[]')
        self.declare_parameter('timeout', 60.0)
        robots = json.loads(self.get_parameter('robots_json').value)
        self._tracker = ReadinessTracker(robots)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor_handles = []
        for robot in robots:
            namespace, prefix = robot['namespace'], robot['frame_prefix']
            for suffix, msg_type, qos in (
                ('wheel/odometry', Odometry, reliable),
                ('imu', Imu, qos_profile_sensor_data),
                ('scan', LaserScan, qos_profile_sensor_data),
            ):
                topic = f'{namespace}/{suffix}'
                sensor_handles.append(self.create_subscription(
                    msg_type, topic, lambda _, key=topic: self._seen(key), qos))
        self._sensor_handles = sensor_handles
        self.create_subscription(Clock, '/clock', lambda _: self._seen('/clock'), reliable)
        # Keep samples from every transient-local robot_state_publisher, not only the
        # latest writer in a multi-publisher fleet.
        static_qos = QoSProfile(history=HistoryPolicy.KEEP_ALL,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self._tf, static_qos)
        self.create_subscription(TFMessage, '/tf', self._tf, reliable)
        self._deadline = self.get_clock().now().nanoseconds / 1e9 + float(self.get_parameter('timeout').value)
        self._announced_ready = False
        self._readiness_timer = self.create_timer(0.25, self._check)

    def _seen(self, key: str) -> None:
        self._tracker.seen(key)

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._tracker.seen_transform(transform.header.frame_id, transform.child_frame_id)

    def _check(self) -> None:
        if self._tracker.ready:
            if not self._announced_ready:
                self._announced_ready = True
                self.get_logger().info('FLEET_READY: every configured robot source boundary is available')
                # Jazzy exposes Node.timers as a generator. Retain and destroy the
                # actual handle instead, while keeping this process alive as the
                # launch's ongoing fleet-health observer.
                timer = self._readiness_timer
                self._readiness_timer = None
                if timer is not None:
                    timer.cancel()
                    self.destroy_timer(timer)
            return
        if self.get_clock().now().nanoseconds / 1e9 >= self._deadline:
            self.get_logger().fatal('FLEET_NOT_READY; missing: ' + ', '.join(sorted(self._tracker.missing)))
            os._exit(2)



def main() -> None:
    rclpy.init()
    node = FleetReadiness()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
