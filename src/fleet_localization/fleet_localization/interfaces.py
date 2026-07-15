"""Resolve one robot's canonical estimator interface from the fleet manifest."""
from dataclasses import dataclass
from turtlebot_fleet_sim.fleet_config import FleetConfigError, load_fleet_config

@dataclass(frozen=True)
class RobotInterface:
    name: str; namespace: str; prefix: str
    @property
    def wheel_topic(self): return f'{self.namespace}/wheel/odometry'
    @property
    def imu_topic(self): return f'{self.namespace}/imu'
    @property
    def scan_topic(self): return f'{self.namespace}/scan'
    @property
    def filtered_topic(self): return f'{self.namespace}/odometry/filtered'
    @property
    def health_topic(self): return f'{self.namespace}/localization/health'
    def frame(self, suffix): return f'{self.prefix}/{suffix}'

def resolve_robot(path: str, name: str) -> RobotInterface:
    config = load_fleet_config(path)
    matches = [robot for robot in config.robots if robot.name == name]
    if not matches:
        raise FleetConfigError(f'robot is not present in fleet manifest: {name!r}')
    robot = matches[0]
    return RobotInterface(robot.name, robot.namespace, robot.frame_prefix)
