"""Expose the canonical ROS boundary and explicit estimator covariances."""

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan


class SensorContract(Node):
    def __init__(self) -> None:
        super().__init__('sensor_contract')
        self.declare_parameter('frame_prefix', '')
        prefix = self.get_parameter('frame_prefix').value
        if not prefix or prefix.startswith('/'):
            raise ValueError('frame_prefix must be slash-free and nonempty')
        self._prefix = prefix
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._odom_pub = self.create_publisher(Odometry, 'wheel/odometry', reliable)
        self._imu_pub = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(TwistStamped, '_sim/cmd_vel', reliable)
        self.create_subscription(Odometry, '_sim/odom', self._odom, reliable)
        self.create_subscription(Imu, '_sim/imu', self._imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '_sim/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'cmd_vel', self._cmd, reliable)

    def _cmd(self, message: TwistStamped) -> None:
        self._cmd_pub.publish(message)

    def _odom(self, message: Odometry) -> None:
        message.header.frame_id = f'{self._prefix}/odom'
        message.child_frame_id = f'{self._prefix}/base_footprint'
        # Conservative, finite planar covariance; unused axes remain explicitly uncertain.
        message.pose.covariance = [0.0] * 36
        message.twist.covariance = [0.0] * 36
        for matrix in (message.pose.covariance, message.twist.covariance):
            matrix[0] = 0.02
            matrix[7] = 0.02
            matrix[14] = 1e3
            matrix[21] = 1e3
            matrix[28] = 1e3
            matrix[35] = 0.04
        self._odom_pub.publish(message)

    def _imu(self, message: Imu) -> None:
        message.header.frame_id = f'{self._prefix}/imu_link'
        message.orientation_covariance = [0.0] * 9
        message.angular_velocity_covariance = [0.0] * 9
        message.linear_acceleration_covariance = [0.0] * 9
        for matrix, variance in (
            (message.orientation_covariance, 0.01),
            (message.angular_velocity_covariance, 4e-8),
            (message.linear_acceleration_covariance, 2.89e-4),
        ):
            matrix[0] = matrix[4] = matrix[8] = variance
        self._imu_pub.publish(message)

    def _scan(self, message: LaserScan) -> None:
        message.header.frame_id = f'{self._prefix}/base_scan'
        self._scan_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = SensorContract()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
