"""Robot-scoped localization diagnostic publisher."""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import TransitionEvent
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from .interfaces import resolve_robot
from .validation import LocalizationHealthState, Observations, Persistence, finite, stamp_seconds


class Health(Node):
    def __init__(self):
        super().__init__('localization_health')
        self.declare_parameter('fleet_config',''); self.declare_parameter('robot','')
        self.interface=resolve_robot(self.get_parameter('fleet_config').value,self.get_parameter('robot').value)
        profile=Path(get_package_share_directory('fleet_localization'))/'config'/'burger_sim.yaml'
        tuning=yaml.safe_load(profile.read_text())['health']
        self.obs=Observations()
        self.state=LocalizationHealthState(
            freshness=float(tuning['freshness_seconds']),
            position_covariance_max=float(tuning['position_covariance_max']),
            yaw_covariance_max=float(tuning['yaw_covariance_max']),
            persistence=Persistence(float(tuning['degrade_after_seconds']),float(tuning['recover_after_seconds'])))
        self.sim_now=0.0; self.map_seen=False; self.lifecycle={'map_server':False,'amcl':False}
        reliable=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        from rosgraph_msgs.msg import Clock
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'sim_now',stamp_seconds(m.clock)),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.interface.wheel_topic,lambda m:self.seen('wheel',m),reliable)
        self.create_subscription(Imu,self.interface.imu_topic,lambda m:self.seen('imu',m),qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.interface.scan_topic,lambda m:self.seen('scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.interface.filtered_topic,self.filtered,reliable)
        self.create_subscription(OccupancyGrid,self.interface.map_topic,lambda _m:setattr(self,'map_seen',True),transient)
        self.create_subscription(PoseWithCovarianceStamped,self.interface.initialpose_topic,self.initialized,reliable)
        self.create_subscription(PoseWithCovarianceStamped,self.interface.amcl_pose_topic,self.amcl_pose,reliable)
        for node in ('map_server','amcl'):
            self.create_subscription(TransitionEvent,f'{self.interface.namespace}/{node}/transition_event',lambda m,n=node:self.transition(n,m),reliable)
        self.pub=self.create_publisher(DiagnosticArray,self.interface.health_topic,reliable)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.create_timer(0.2,self.publish)
    def seen(self,key,msg): self.obs.last[key]=stamp_seconds(msg.header.stamp); self.obs.invalid.pop(key,None)
    def filtered(self,msg):
        values=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z,*msg.pose.covariance,*msg.twist.covariance]
        if finite(values): self.obs.last['ekf']=stamp_seconds(msg.header.stamp); self.obs.invalid.pop('ekf',None)
        else: self.obs.invalid['ekf']='non-finite output'
    def initialized(self,msg):
        stamp=stamp_seconds(msg.header.stamp) or self.sim_now
        if msg.header.frame_id == self.interface.frame('map'): self.state.initialized(stamp)
    def amcl_pose(self,msg):
        values=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w,*msg.pose.covariance]
        if msg.header.frame_id == self.interface.frame('map') and finite(values):
            self.state.estimate(stamp_seconds(msg.header.stamp),msg.pose.covariance)
    def transition(self,node,msg): self.lifecycle[node]=msg.goal_state.label.lower() == 'active'
    def publish(self):
        if self.sim_now <= 0: return
        tf_current=False
        try:
            transform=self.buffer.lookup_transform(self.interface.frame('map'),self.interface.frame('base_scan'),rclpy.time.Time())
            transform_time=stamp_seconds(transform.header.stamp)
            tf_current=0.0 <= self.sim_now-transform_time <= self.state.freshness
        except Exception: pass
        problems=self.obs.problems(self.sim_now,('wheel','imu','scan','ekf'))
        state,problems=self.state.evaluate(self.sim_now,problems,all(self.lifecycle.values()),self.map_seen,tf_current)
        level=DiagnosticStatus.ERROR if state=='degraded' else DiagnosticStatus.OK if state=='localized' else DiagnosticStatus.WARN
        status=DiagnosticStatus(level=level,name=f'{self.interface.name}/localization',message=state,values=[KeyValue(key='problem',value=p) for p in problems])
        msg=DiagnosticArray(); msg.header.stamp=self.get_clock().now().to_msg(); msg.status=[status]; self.pub.publish(msg)

def main():
    rclpy.init(); node=Health()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        try: node.destroy_node()
        except (KeyboardInterrupt,ExternalShutdownException): pass
        if rclpy.ok(): rclpy.shutdown()
