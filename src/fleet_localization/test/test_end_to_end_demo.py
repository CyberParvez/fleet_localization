"""Two separately invoked localization sessions through supported interfaces."""
from pathlib import Path

from fleet_localization.interfaces import resolve_robot
from fleet_localization.map_catalog import resolve_map
from fleet_localization.validation import LocalizationHealthState, Persistence


ROOT = Path(__file__).parents[2]
FLEET = ROOT / 'turtlebot_fleet_sim/config/fleet.example.yaml'


def covariance():
    value = [0.0] * 36
    value[0] = value[7] = value[35] = 0.05
    return value


def test_standard_two_robot_public_contract_and_manual_initialposes():
    first, second = resolve_robot(FLEET, 'robot1'), resolve_robot(FLEET, 'robot2')
    shared = resolve_map(FLEET)
    assert shared.map_id == 'turtlebot3_world_v1'
    assert first.initialpose_topic == '/robot1/initialpose'
    assert second.initialpose_topic == '/robot2/initialpose'
    assert first.map_topic == '/robot1/map' and second.map_topic == '/robot2/map'
    assert first.frame('map') != second.frame('map')
    states = [LocalizationHealthState(persistence=Persistence(2.0, 2.0)) for _ in range(2)]
    for state in states:
        state.initialized(5.0); state.estimate(5.1, covariance())
        assert state.evaluate(5.1, [], True, True, True) == ('localized', [])


def test_custom_pair_uses_same_public_launch_and_independent_instances(tmp_path):
    import yaml
    manifest = yaml.safe_load(FLEET.read_text())
    manifest['fleet']['map_store'] = str(ROOT / 'turtlebot_fleet_sim/config/maps')
    manifest['fleet']['map_id'] = 'asymmetric_indoor_v1'
    manifest['simulation']['world'] = 'asymmetric_indoor'
    custom = tmp_path / 'fleet.yaml'; custom.write_text(yaml.safe_dump(manifest))
    assert resolve_map(custom).world == 'asymmetric_indoor'
    launch = (ROOT / 'fleet_localization/launch/localization.launch.py').read_text()
    assert "DeclareLaunchArgument('robot')" in launch
    assert 'fleet_localization.launch' not in launch
    assert "namespace=interface.namespace" in launch

