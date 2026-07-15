"""Regression tests for recoverable readiness timestamp validation."""
from types import SimpleNamespace

from fleet_localization.readiness import Readiness


def _stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def _message(seconds, frame='robot1/imu_link'):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(seconds), frame_id=frame))


def _readiness(now=10.0):
    node = Readiness.__new__(Readiness)
    node.sim_now = now
    node.valid = {}
    node.reasons = {}
    node.last_stamp = {}
    return node


def _accept(node, seconds):
    node._common('imu', _message(seconds), 'robot1/imu_link', [1.0])


def test_rejected_future_sample_does_not_poison_ordering_state():
    node = _readiness(10.0)
    _accept(node, 9.9)
    _accept(node, 100.0)
    assert node.reasons['imu'] == 'future timestamp'
    assert node.last_stamp['imu'] == 9.9

    node.sim_now = 10.2
    _accept(node, 10.1)
    assert node.valid['imu'] == 10.1
    assert 'imu' not in node.reasons


def test_small_delivery_reordering_recovers_on_next_current_sample():
    node = _readiness(10.2)
    _accept(node, 10.1)
    _accept(node, 9.8)
    assert node.reasons['imu'] == 'out-of-order timestamp'
    assert node.last_stamp['imu'] == 10.1

    _accept(node, 10.2)
    assert node.valid['imu'] == 10.2
    assert 'imu' not in node.reasons


def test_significant_clock_rollback_starts_fresh_timestamp_epoch():
    node = _readiness(50.0)
    node.valid = {'wheel': 49.9, 'imu': 49.9, 'scan': 49.9, 'tf': 50.0}
    node.last_stamp = {'wheel': 49.9, 'imu': 49.9, 'scan': 49.9}
    node.reasons = {'scan': 'out-of-order timestamp'}

    node.clock(SimpleNamespace(clock=_stamp(2.0)))
    assert node.sim_now == 2.0
    assert node.valid == {}
    assert node.last_stamp == {}
    assert node.reasons == {}

    _accept(node, 1.9)
    assert node.valid['imu'] == 1.9


def test_small_clock_reordering_does_not_erase_readiness_state():
    node = _readiness(10.0)
    node.valid = {'imu': 9.9}
    node.last_stamp = {'imu': 9.9}
    node.clock(SimpleNamespace(clock=_stamp(9.95)))
    assert node.valid == {'imu': 9.9}
    assert node.last_stamp == {'imu': 9.9}
