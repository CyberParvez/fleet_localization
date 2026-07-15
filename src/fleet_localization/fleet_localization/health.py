"""Robot-scoped estimator diagnostic publisher."""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener
from .interfaces import resolve_robot
from .validation import Observations, Persistence, finite, stamp_seconds

class Health(Node):
    def __init__(self):
        super().__init__('localization_health')
        self.declare_parameter('fleet_config',''); self.declare_parameter('robot','')
        self.interface=resolve_robot(self.get_parameter('fleet_config').value,self.get_parameter('robot').value)
        self.obs=Observations(); self.persistence=Persistence(); self.sim_now=0.0
        reliable=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        from rosgraph_msgs.msg import Clock
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'sim_now',stamp_seconds(m.clock)),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.interface.wheel_topic,lambda m:self.seen('wheel',m),reliable)
        self.create_subscription(Imu,self.interface.imu_topic,lambda m:self.seen('imu',m),qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.interface.scan_topic,lambda m:self.seen('scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.interface.filtered_topic,self.filtered,reliable)
        self.pub=self.create_publisher(DiagnosticArray,self.interface.health_topic,reliable)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.create_timer(0.2,self.publish)
    def seen(self,key,msg): self.obs.last[key]=stamp_seconds(msg.header.stamp); self.obs.invalid.pop(key,None)
    def filtered(self,msg):
        values=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x,msg.twist.twist.angular.z,*msg.pose.covariance,*msg.twist.covariance]
        if finite(values): self.obs.last['ekf']=stamp_seconds(msg.header.stamp); self.obs.invalid.pop('ekf',None)
        else: self.obs.invalid['ekf']='non-finite output'
    def publish(self):
        if self.sim_now <= 0: return
        try: self.buffer.lookup_transform(self.interface.frame('base_footprint'),self.interface.frame('base_scan'),rclpy.time.Time()); self.obs.last['tf']=self.sim_now
        except Exception: self.obs.invalid['tf']='transform unavailable'
        problems=self.obs.problems(self.sim_now,('wheel','imu','scan','tf','ekf'))
        state=self.persistence.update(self.sim_now,problems)
        if state=='ready': state='ekf active'
        status=DiagnosticStatus(level=DiagnosticStatus.ERROR if state=='degraded' else DiagnosticStatus.WARN if problems else DiagnosticStatus.OK,name=f'{self.interface.name}/localization',message=state,values=[KeyValue(key='problem',value=p) for p in problems])
        msg=DiagnosticArray(); msg.header.stamp=self.get_clock().now().to_msg(); msg.status=[status]; self.pub.publish(msg)

def main():
    rclpy.init(); node=Health()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()
