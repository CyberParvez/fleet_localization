import os
from unittest.mock import Mock

os.environ.setdefault('ROS_LOG_DIR', '/tmp/turtlebot_fleet_sim_test_logs')

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

from turtlebot_fleet_sim.sensor_contract import SensorContract


def test_public_topics_types_qos_rewriting_and_command_isolation():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot1', '-p', 'frame_prefix:=robot1'])
    node = SensorContract()
    try:
        publishers = {publisher.topic_name: publisher for publisher in node.publishers}
        subscriptions = {subscription.topic_name: subscription for subscription in node.subscriptions}
        assert set(publishers) >= {
            '/robot1/wheel/odometry', '/robot1/imu', '/robot1/scan', '/robot1/_sim/cmd_vel'}
        assert set(subscriptions) >= {
            '/robot1/_sim/odom', '/robot1/_sim/imu', '/robot1/_sim/scan', '/robot1/cmd_vel'}
        assert publishers['/robot1/wheel/odometry'].msg_type is Odometry
        assert publishers['/robot1/imu'].msg_type is Imu
        assert publishers['/robot1/scan'].msg_type is LaserScan
        assert publishers['/robot1/_sim/cmd_vel'].msg_type is TwistStamped
        assert publishers['/robot1/wheel/odometry'].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert publishers['/robot1/imu'].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert publishers['/robot1/scan'].qos_profile.durability == DurabilityPolicy.VOLATILE

        node._odom_pub = Mock()
        odom = Odometry()
        node._odom(odom)
        rewritten = node._odom_pub.publish.call_args.args[0]
        assert rewritten.header.frame_id == 'robot1/odom'
        assert rewritten.child_frame_id == 'robot1/base_footprint'
        assert all(value > 0.0 for value in (rewritten.pose.covariance[0], rewritten.pose.covariance[35],
                                              rewritten.twist.covariance[0], rewritten.twist.covariance[35]))

        node._imu_pub = Mock()
        imu = Imu()
        node._imu(imu)
        rewritten_imu = node._imu_pub.publish.call_args.args[0]
        assert rewritten_imu.header.frame_id == 'robot1/imu_link'
        assert rewritten_imu.orientation_covariance[8] > 0.0
        assert rewritten_imu.angular_velocity_covariance[8] > 0.0

        node._scan_pub = Mock()
        scan = LaserScan()
        node._scan(scan)
        assert node._scan_pub.publish.call_args.args[0].header.frame_id == 'robot1/base_scan'

        node._cmd_pub = Mock()
        command = TwistStamped()
        node._cmd(command)
        node._cmd_pub.publish.assert_called_once_with(command)
        assert all('/robot2/' not in topic for topic in publishers)
    finally:
        node.destroy_node()
        rclpy.shutdown()
