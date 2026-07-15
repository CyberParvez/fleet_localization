"""Bounded one-shot readiness gate. EKF is launched only after this exits zero."""
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from .interfaces import resolve_robot
from .validation import (MeasurementValidator, TfAuthorityTracker, covariance_valid,
                         publisher_gid, stamp_seconds)

_CLOCK_EPOCH_ROLLBACK_SECONDS = 1.0
_TF_DISCOVERY_SECONDS = 0.5

class Readiness(Node):
    def __init__(self):
        super().__init__('readiness')
        for name, default in [('fleet_config',''), ('robot',''), ('timeout',60.0),
                              ('freshness',1.0),('future_tolerance',0.1),('ordering_tolerance',0.001)]:
            self.declare_parameter(name, default)
        self.interface = resolve_robot(self.get_parameter('fleet_config').value, self.get_parameter('robot').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.started = time.monotonic(); self.sim_now = 0.0
        self.valid = {}; self.reasons = {}; self.last_stamp = {}
        self.validator=MeasurementValidator(float(self.get_parameter('freshness').value),
            float(self.get_parameter('future_tolerance').value),float(self.get_parameter('ordering_tolerance').value),self.last_stamp)
        self.tf_tracker=TfAuthorityTracker({
            (self.interface.frame('odom'),self.interface.frame('base_footprint')),
            (self.interface.frame('map'),self.interface.frame('odom'))})
        self.tf_discovery_started=None
        reliable=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.interface.wheel_topic, self.wheel, reliable)
        self.create_subscription(Imu, self.interface.imu_topic, self.imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.interface.scan_topic, self.scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.tf_message, reliable)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.timer=self.create_timer(0.1,self.check)
        self.exit_code=None
    def clock(self,msg):
        now=stamp_seconds(msg.clock)
        if self.sim_now > 0.0 and now + _CLOCK_EPOCH_ROLLBACK_SECONDS < self.sim_now:
            # Gazebo may restart its simulation epoch. Measurements accepted in
            # the prior epoch must not poison ordering or freshness in the new one.
            self.valid.clear(); self.reasons.clear()
            if hasattr(self,'validator'): self.validator.reset()
            else: self.last_stamp.clear()
            self.tf_discovery_started=None
        self.sim_now=now
    def _common(self,key,msg,frame,values,covariance=None,required=()):
        if not hasattr(self,'validator'):
            self.validator=MeasurementValidator(1.0,0.1,0.001,self.last_stamp)
        stamp=stamp_seconds(msg.header.stamp)
        reason=self.validator.validate(key,stamp,self.sim_now,frame=msg.header.frame_id,
            expected_frame=frame,values=values,covariance=covariance,required_covariance=required)
        if reason: self.valid.pop(key,None); self.reasons[key]=reason
        else:
            self.last_stamp[key]=stamp
            self.valid[key]=stamp; self.reasons.pop(key,None)
    def wheel(self,m):
        self._common('wheel',m,self.interface.frame('odom'),[m.twist.twist.linear.x,m.twist.twist.angular.z],m.twist.covariance,(0,35))
        if m.child_frame_id != self.interface.frame('base_footprint'):
            self.valid.pop('wheel',None); self.reasons['wheel']=f'child frame {m.child_frame_id!r}, expected {self.interface.frame("base_footprint")!r}'
    def imu(self,m):
        self._common('imu',m,self.interface.frame('imu_link'),[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w,m.angular_velocity.z],m.orientation_covariance,(8,))
        if not covariance_valid(m.angular_velocity_covariance,(8,)):
            self.valid.pop('imu',None); self.reasons['imu']='invalid angular velocity covariance'
    def scan(self,m):
        # Positive infinity is a valid LaserScan "no return" value; NaN is not.
        values=[value for value in m.ranges if not math.isinf(value)]
        self._common('scan',m,self.interface.frame('base_scan'),values)
    def tf_message(self,msg,info=None):
        self.tf_tracker.observe(msg.transforms,publisher_gid(info))
    def check(self):
        if self.sim_now > 0:
            for child in ('base_link','imu_link','base_scan'):
                try: self.buffer.lookup_transform(self.interface.frame('base_footprint'),self.interface.frame(child),rclpy.time.Time())
                except Exception as exc: self.reasons['tf']=f'{child}: {type(exc).__name__}'; self.valid.pop('tf',None); break
            else: self.valid['tf']=self.sim_now; self.reasons.pop('tf',None)
        sources=('wheel','imu','scan','tf')
        if all(key in self.valid for key in sources) and self.tf_discovery_started is None:
            self.tf_discovery_started=time.monotonic()
        observed=self.tf_tracker.observed_edges()
        if observed:
            edges=', '.join(f'{parent} -> {child}' for parent,child in sorted(observed))
            self.valid.pop('tf_authority',None)
            self.reasons['tf_authority']=(f'preexisting TF authority for {edges} observed on /tf; disable the '
                'simulator or external estimator broadcaster before localization')
        elif self.tf_discovery_started is not None and time.monotonic()-self.tf_discovery_started >= _TF_DISCOVERY_SECONDS:
            self.valid['tf_authority']=self.sim_now; self.reasons.pop('tf_authority',None)
        required=(*sources,'tf_authority')
        if self.sim_now > 0 and all(k in self.valid and self.sim_now-self.valid[k] <= 1.0 for k in required):
            self.get_logger().info('readiness complete'); self.exit_code=0; rclpy.shutdown(); return
        if time.monotonic()-self.started >= self.timeout:
            missing=[]
            if self.sim_now <= 0: missing.append('/clock: missing or zero')
            for key in required:
                if key not in self.valid: missing.append(self.reasons.get(key,f'{key}: missing'))
                elif self.sim_now-self.valid[key] > 1.0: missing.append(f'{key}: stale')
            self.get_logger().error('readiness timeout; unmet: '+'; '.join(missing)); self.exit_code=2; rclpy.shutdown()

def main():
    rclpy.init(); node=None
    try:
        node=Readiness(); rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception as exc:
        if rclpy.ok(): rclpy.logging.get_logger('readiness').error(str(exc))
        raise SystemExit(2)
    finally:
        code=2 if node is None or node.exit_code is None else node.exit_code
        if node:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(code)
