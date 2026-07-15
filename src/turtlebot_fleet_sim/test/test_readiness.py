from turtlebot_fleet_sim.fleet_readiness import ReadinessTracker
from turtlebot_fleet_sim.fleet_readiness import FleetReadiness
from unittest.mock import Mock


ROBOTS = [
    {'namespace': '/robot1', 'frame_prefix': 'robot1'},
    {'namespace': '/robot2', 'frame_prefix': 'robot2'},
]


def test_readiness_aggregates_every_robot_and_reports_all_missing():
    tracker = ReadinessTracker(ROBOTS)
    assert len(tracker.missing) == 19
    assert '/robot1/scan' in tracker.missing
    assert '/robot2/wheel/odometry' in tracker.missing
    assert 'tf:robot2/base_scan' in tracker.missing
    assert not tracker.ready


def test_readiness_succeeds_only_after_every_item_seen():
    tracker = ReadinessTracker(ROBOTS)
    for item in tuple(tracker.missing):
        if item.startswith('tf:'):
            tracker.seen_tf(item.removeprefix('tf:'))
        else:
            tracker.seen(item)
    assert tracker.ready
    assert tracker.missing == set()


def test_tf_root_is_satisfied_when_seen_as_parent():
    tracker = ReadinessTracker([ROBOTS[0]])
    assert 'tf:robot1/base_footprint' in tracker.missing
    tracker.seen_transform('robot1/base_footprint', 'robot1/base_link')
    assert 'tf:robot1/base_footprint' not in tracker.missing
    assert 'tf:robot1/base_link' not in tracker.missing


def test_successful_check_uses_retained_timer_and_is_idempotent():
    class Harness:
        pass

    harness = Harness()
    harness._tracker = ReadinessTracker([])
    harness._tracker.seen('/clock')
    harness._announced_ready = False
    harness._readiness_timer = Mock()
    harness.get_logger = Mock(return_value=Mock())
    harness.destroy_timer = Mock(return_value=True)

    FleetReadiness._check(harness)
    harness._readiness_timer = None
    FleetReadiness._check(harness)

    harness.get_logger.return_value.info.assert_called_once_with(
        'FLEET_READY: every configured robot source boundary is available')
    harness.destroy_timer.assert_called_once()
