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
            if now - self.bad_since >= self.degrade_after: self.degraded = True
        else:
            self.bad_since = None
            self.good_since = now if self.good_since is None else self.good_since
            if self.degraded and now - self.good_since >= self.recover_after: self.degraded = False
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
