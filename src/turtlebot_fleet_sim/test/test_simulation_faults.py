"""Public fleet-startup and runtime fault-boundary regression matrix."""
from pathlib import Path

from turtlebot_fleet_sim.fleet_readiness import ReadinessTracker


ROBOTS = [
    {'namespace': '/robot1', 'frame_prefix': 'robot1'},
    {'namespace': '/robot2', 'frame_prefix': 'robot2'},
]


def test_incomplete_robot_keeps_complete_fleet_unready_atomically():
    tracker = ReadinessTracker(ROBOTS)
    for item in tuple(tracker.missing):
        if 'robot1' in item or item == '/clock':
            tracker.seen_tf(item[3:]) if item.startswith('tf:') else tracker.seen(item)
    assert not tracker.ready
    assert '/robot2/scan' in tracker.missing
    assert all('robot1' not in item for item in tracker.missing)


def test_runtime_sources_are_not_wired_to_shutdown_or_fault_hooks():
    launch = (Path(__file__).parents[1] / 'launch/fleet_sim.launch.py').read_text()
    # Atomic termination owns startup readiness only. Sensor-contract processes
    # remain observable after readiness and do not terminate the shared world.
    assert 'target_action=readiness' in launch
    assert 'target_action=sensor_contract' not in launch
    assert 'drop_sensor' not in launch and 'fault_injection' not in launch


def test_tf_authorities_are_exclusive_at_the_public_launch_boundary():
    launch = (Path(__file__).parents[1] / 'launch/fleet_sim.launch.py').read_text()
    model = (Path(__file__).parents[1] / 'turtlebot_fleet_sim/model.py').read_text()
    assert "('tf_topic', f'{robot.namespace}/_sim/unused_odom_tf')" in model
    assert '_sim/unused_odom_tf' not in launch
    assert launch.count("package='robot_state_publisher'") == 1  # data-driven action, once per robot
    assert "remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]" in launch
