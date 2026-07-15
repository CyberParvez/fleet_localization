"""Robot-scoped localization diagnostic publisher."""
import math
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from .interfaces import resolve_robot
from .validation import (LocalizationHealthState, MappingHealthState, MeasurementValidator,
                         Observations, Persistence, TfAuthorityTracker,
                         covariance_nonnegative, covariance_valid, finite, stamp_seconds)
from .validation import publisher_gid

_CLOCK_EPOCH_ROLLBACK_SECONDS = 1.0


def tf_timestamp_current(sim_now, transform_time, freshness, future_tolerance):
    """Accept AMCL's bounded future-dated TF without relaxing message validation."""
    age = sim_now - transform_time
    return -future_tolerance <= age <= freshness


class Health(Node):
    def __init__(self):
        super().__init__('localization_health')
        self.declare_parameter('fleet_config',''); self.declare_parameter('robot','')
        self.declare_parameter('mode','localization')
        self.mode=self.get_parameter('mode').value
        if self.mode not in ('localization','mapping'): raise ValueError(f'unsupported health mode: {self.mode}')
        self.interface=resolve_robot(self.get_parameter('fleet_config').value,self.get_parameter('robot').value)
        profile=Path(get_package_share_directory('fleet_localization'))/'config'/'burger_sim.yaml'
        tuning=yaml.safe_load(profile.read_text())['health']
        self.tf_future_tolerance=float(tuning['tf_future_tolerance_seconds'])
        self.obs=Observations(float(tuning['freshness_seconds']))
        self.validator=MeasurementValidator(float(tuning['freshness_seconds']),
            float(tuning['future_tolerance_seconds']),float(tuning['ordering_tolerance_seconds']))
        self.state=LocalizationHealthState(
            freshness=float(tuning['freshness_seconds']),
            position_covariance_max=float(tuning['position_covariance_max']),
            yaw_covariance_max=float(tuning['yaw_covariance_max']),
            persistence=Persistence(float(tuning['degrade_after_seconds']),float(tuning['recover_after_seconds'])))
        if self.mode == 'mapping':
            self.state=MappingHealthState(float(tuning['freshness_seconds']),
                Persistence(float(tuning['degrade_after_seconds']),float(tuning['recover_after_seconds'])))
        self.sim_now=0.0; self.map_seen=False; self.map_at=None
        self.tf_tracker=TfAuthorityTracker({
            (self.interface.frame('odom'),self.interface.frame('base_footprint')),
            (self.interface.frame('map'),self.interface.frame('odom'))})
        self.lifecycle=({'map_server':False,'amcl':False} if self.mode == 'localization' else {'slam_toolbox':False})
        reliable=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        from rosgraph_msgs.msg import Clock
        self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        self.create_subscription(Odometry,self.interface.wheel_topic,lambda m:self.seen('wheel',m),reliable)
        self.create_subscription(Imu,self.interface.imu_topic,lambda m:self.seen('imu',m),qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.interface.scan_topic,lambda m:self.seen('scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.interface.filtered_topic,self.filtered,reliable)
        self.create_subscription(OccupancyGrid,self.interface.map_topic,self.map_received,transient)
        self.create_subscription(PoseWithCovarianceStamped,self.interface.initialpose_topic,self.initialized,reliable)
        self.create_subscription(PoseWithCovarianceStamped,self.interface.amcl_pose_topic,self.amcl_pose,reliable)
        self.create_subscription(TFMessage,'/tf',self.tf_message,reliable)
        for node in self.lifecycle:
            self.create_subscription(TransitionEvent,f'{self.interface.namespace}/{node}/transition_event',lambda m,n=node:self.transition(n,m),reliable)
        self.lifecycle_clients={node:self.create_client(GetState,f'{self.interface.namespace}/{node}/get_state') for node in self.lifecycle}
        self.lifecycle_pending={node:False for node in self.lifecycle}
        self.pub=self.create_publisher(DiagnosticArray,self.interface.health_topic,reliable)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.create_timer(0.2,self.publish)
    def clock(self,msg):
        now=stamp_seconds(msg.clock)
        if self.sim_now > 0.0 and now + _CLOCK_EPOCH_ROLLBACK_SECONDS < self.sim_now:
            self.obs.clear(); self.validator.reset(); self.map_seen=False; self.map_at=None
            self.state.reset_epoch()
        self.sim_now=now
    def _observe(self,key,msg,expected_frame,values,covariance=None,required=(),
                 expected_child=None):
        stamp=stamp_seconds(msg.header.stamp)
        reason=self.validator.validate(key,stamp,self.sim_now,frame=msg.header.frame_id,
            expected_frame=expected_frame,child_frame=getattr(msg,'child_frame_id',None),
            expected_child_frame=expected_child,values=values,covariance=covariance,
            required_covariance=required)
        if reason:
            self.obs.last.pop(key,None); self.obs.invalid[key]=reason
            return False
        self.obs.last[key]=stamp; self.obs.invalid.pop(key,None)
        return True
    def seen(self,key,msg):
        if key == 'wheel':
            self._observe(key,msg,self.interface.frame('odom'),
                [msg.twist.twist.linear.x,msg.twist.twist.angular.z],msg.twist.covariance,(0,35),
                self.interface.frame('base_footprint'))
        elif key == 'imu':
            if self._observe(key,msg,self.interface.frame('imu_link'),
                [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w,msg.angular_velocity.z],
                msg.orientation_covariance,(8,)) and not covariance_valid(msg.angular_velocity_covariance,(8,)):
                self.obs.last.pop(key,None); self.obs.invalid[key]='invalid angular velocity covariance'
        else:
            values=[value for value in msg.ranges if not math.isinf(value)]
            self._observe(key,msg,self.interface.frame('base_scan'),values)
    def filtered(self,msg):
        values=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z,*msg.pose.covariance,*msg.twist.covariance]
        self._observe('ekf',msg,self.interface.frame('odom'),values,msg.pose.covariance,(0,7,35),
            self.interface.frame('base_footprint'))
        if 'ekf' in self.obs.last and not covariance_valid(msg.twist.covariance,(0,35)):
            self.obs.last.pop('ekf',None); self.obs.invalid['ekf']='invalid twist covariance'
    def initialized(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        reason=self.validator.validate('initialpose',stamp,self.sim_now,frame=msg.header.frame_id,
            expected_frame=self.interface.frame('map'),values=[msg.pose.pose.position.x,msg.pose.pose.position.y,
                msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w],
            covariance=msg.pose.covariance,required_covariance=(0,7,35))
        if reason: self.obs.invalid['initialpose']=reason
        else: self.obs.invalid.pop('initialpose',None); self.state.initialized(stamp)
    def amcl_pose(self,msg):
        values=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w,*msg.pose.covariance]
        stamp=stamp_seconds(msg.header.stamp)
        reason=self.validator.validate('amcl',stamp,self.sim_now,frame=msg.header.frame_id,
            expected_frame=self.interface.frame('map'),values=values)
        if reason is None and not covariance_nonnegative(msg.pose.covariance,(0,7,35)):
            reason='invalid covariance'
        if reason: self.obs.invalid['amcl']=reason
        else:
            self.obs.invalid.pop('amcl',None); self.state.estimate(stamp,msg.pose.covariance)
    def transition(self,node,msg): self.lifecycle[node]=msg.goal_state.label.lower() == 'active'
    def tf_message(self,msg,info):
        self.tf_tracker.observe(msg.transforms,publisher_gid(info))
    def tf_authority_problems(self):
        conflicts=self.tf_tracker.conflicts()
        if not conflicts: return []
        edges=', '.join(f'{parent} -> {child} ({len(gids)} publishers)'
            for (parent,child),gids in sorted(conflicts.items()))
        return [f'duplicate TF authority: {edges}']
    def poll_lifecycle(self):
        for name,client in self.lifecycle_clients.items():
            if self.lifecycle_pending[name] or not client.service_is_ready(): continue
            self.lifecycle_pending[name]=True
            future=client.call_async(GetState.Request())
            future.add_done_callback(lambda f,n=name:self.lifecycle_result(n,f))
    def lifecycle_result(self,name,future):
        self.lifecycle_pending[name]=False
        try: self.lifecycle[name]=future.result().current_state.label.lower() == 'active'
        except Exception: self.lifecycle[name]=False
    def map_received(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        metadata=msg.info
        values=[metadata.resolution,metadata.origin.position.x,metadata.origin.position.y,
            metadata.origin.position.z,metadata.origin.orientation.x,metadata.origin.orientation.y,
            metadata.origin.orientation.z,metadata.origin.orientation.w]
        reason=self.validator.validate('map',stamp,self.sim_now,frame=msg.header.frame_id,
            expected_frame=self.interface.frame('map'),values=values)
        expected_size=int(metadata.width)*int(metadata.height)
        if reason is None and (not finite([metadata.resolution]) or metadata.resolution <= 0.0):
            reason='invalid resolution'
        elif reason is None and (metadata.width <= 0 or metadata.height <= 0 or
                                 len(msg.data) != expected_size):
            reason='invalid dimensions or data length'
        if reason:
            self.map_seen=False; self.map_at=None; self.obs.invalid['map']=reason
        else:
            self.map_seen=True; self.map_at=stamp; self.obs.invalid.pop('map',None)
    def publish(self):
        if self.sim_now <= 0: return
        self.poll_lifecycle()
        tf_current=False
        try:
            transform=self.buffer.lookup_transform(self.interface.frame('map'),self.interface.frame('base_scan'),rclpy.time.Time())
            transform_time=stamp_seconds(transform.header.stamp)
            tf_current=tf_timestamp_current(self.sim_now,transform_time,self.state.freshness,
                self.tf_future_tolerance)
        except Exception: pass
        problems=self.obs.problems(self.sim_now,('wheel','imu','scan','ekf'))
        problems.extend(f'{key}: {reason}' for key,reason in self.obs.invalid.items()
                        if key in ('initialpose','amcl','map'))
        problems.extend(self.tf_authority_problems())
        if self.mode == 'mapping':
            state,problems=self.state.evaluate(self.sim_now,problems,all(self.lifecycle.values()),self.map_at,tf_current)
        else:
            state,problems=self.state.evaluate(self.sim_now,problems,all(self.lifecycle.values()),self.map_seen,tf_current)
        level=DiagnosticStatus.ERROR if state=='degraded' else DiagnosticStatus.OK if state in ('localized','mapping') else DiagnosticStatus.WARN
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
