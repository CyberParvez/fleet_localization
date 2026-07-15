"""Pure measurement and health state evaluation shared by ROS nodes and tests."""
from dataclasses import dataclass, field
import math

def finite(values): return all(math.isfinite(value) for value in values)

def stamp_seconds(stamp): return stamp.sec + stamp.nanosec / 1e9

def covariance_valid(values, required):
    return len(values) > max(required) and finite(values) and all(values[index] > 0.0 for index in required)

@dataclass
class Persistence:
    degrade_after: float = 2.0
    recover_after: float = 2.0
    degraded: bool = False
    bad_since: float | None = None
    good_since: float | None = None
    def update(self, now: float, problems: list[str]) -> str:
        if problems:
            self.good_since = None
            self.bad_since = now if self.bad_since is None else self.bad_since
            if now - self.bad_since + 1e-9 >= self.degrade_after: self.degraded = True
        else:
            self.bad_since = None
            self.good_since = now if self.good_since is None else self.good_since
            if self.degraded and now - self.good_since + 1e-9 >= self.recover_after: self.degraded = False
        return 'degraded' if self.degraded else ('ready' if not problems else 'waiting for inputs')

@dataclass
class Observations:
    last: dict[str, float] = field(default_factory=dict)
    invalid: dict[str, str] = field(default_factory=dict)
    def problems(self, now: float, required=('wheel', 'imu', 'scan', 'tf')):
        result=[]
        for key in required:
            if key in self.invalid: result.append(f'{key}: {self.invalid[key]}')
            elif key not in self.last: result.append(f'{key}: missing')
            elif now-self.last[key] > 1.0: result.append(f'{key}: stale')
        return result

@dataclass
class LocalizationHealthState:
    freshness: float = 1.0
    position_covariance_max: float = 0.25
    yaw_covariance_max: float = 0.10
    persistence: Persistence = field(default_factory=Persistence)
    initialized_at: float | None = None
    amcl_at: float | None = None
    amcl_covariance: tuple[float, ...] | None = None

    def initialized(self, stamp: float):
        self.initialized_at = stamp
        self.amcl_at = None
        self.amcl_covariance = None

    def estimate(self, stamp: float, covariance):
        if self.initialized_at is not None and stamp >= self.initialized_at:
            self.amcl_at = stamp
            self.amcl_covariance = tuple(covariance)

    def evaluate(self, now: float, base_problems: list[str], lifecycle_active: bool, map_current: bool, tf_current: bool):
        if self.initialized_at is None:
            problems = list(base_problems)
            if not lifecycle_active: problems.append('localization lifecycle inactive')
            if not map_current: problems.append('map unavailable')
            persisted = self.persistence.update(now, problems)
            return ('degraded' if persisted == 'degraded' else 'awaiting initial pose'), problems
        problems = list(base_problems)
        if not lifecycle_active: problems.append('localization lifecycle inactive')
        if not map_current: problems.append('map unavailable')
        if not tf_current: problems.append('map-to-sensor transform unavailable')
        if self.amcl_at is None: problems.append('post-initialization AMCL estimate missing')
        elif now - self.amcl_at > self.freshness: problems.append('AMCL estimate stale')
        covariance = self.amcl_covariance
        if covariance is not None:
            if len(covariance) != 36 or not finite(covariance): problems.append('AMCL covariance invalid')
            else:
                if covariance[0] > self.position_covariance_max or covariance[7] > self.position_covariance_max:
                    problems.append('AMCL planar position covariance too high')
                if covariance[35] > self.yaw_covariance_max: problems.append('AMCL yaw covariance too high')
        persisted = self.persistence.update(now, problems)
        if persisted == 'degraded': return 'degraded', problems
        if problems: return 'localizing', problems
        return 'localized', []


@dataclass
class MappingHealthState:
    freshness: float = 1.0
    persistence: Persistence = field(default_factory=Persistence)

    def evaluate(self, now: float, base_problems: list[str], lifecycle_active: bool,
                 map_at: float | None, tf_current: bool):
        problems = list(base_problems)
        if not lifecycle_active: problems.append('SLAM lifecycle inactive')
        if map_at is None: problems.append('map unavailable')
        elif now - map_at > self.freshness: problems.append('map stale')
        if not tf_current: problems.append('map-to-sensor transform unavailable')
        persisted = self.persistence.update(now, problems)
        if persisted == 'degraded': return 'degraded', problems
        if problems: return 'waiting for mapping', problems
        return 'mapping', []
