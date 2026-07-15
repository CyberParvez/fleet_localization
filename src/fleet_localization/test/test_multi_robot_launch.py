"""Production launch remains a repeatable one-robot invocation, not a fleet wrapper."""
from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_canonical_launch_selects_exactly_one_robot_per_process():
    source = (ROOT / 'launch' / 'localization.launch.py').read_text()
    assert "robot=LaunchConfiguration('robot').perform(context)" in source
    assert 'resolve_robot(config,robot)' in source
    assert "DeclareLaunchArgument('robot')" in source
    assert 'for robot in' not in source
    assert source.count("executable='ekf_node'") == 1
    assert source.count("executable='map_server'") == 1
    assert source.count("executable='amcl'") == 1
    assert source.count("executable='health'") == 1


def test_all_runtime_nodes_are_namespaced_and_lifecycle_targets_are_relative():
    source = (ROOT / 'launch' / 'localization.launch.py').read_text()
    assert source.count('namespace=interface.namespace') == 7
    assert "name='lifecycle_manager_localization'" in source
    assert "'node_names':['map_server','amcl']" in source
    assert "'node_names':['/map_server'" not in source
    assert "name=f'{robot}_localization_rviz'" in source


def test_no_production_multi_robot_launcher_or_per_robot_profiles_exist():
    assert [path.name for path in (ROOT / 'launch').glob('*.launch.py')] == ['localization.launch.py']
    configs = sorted(path.name for path in (ROOT / 'config').glob('*.yaml'))
    assert configs == ['burger_sim.yaml']
