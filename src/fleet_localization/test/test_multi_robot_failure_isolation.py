"""Structural shutdown/restart isolation for separate launch processes."""
from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_launch_has_no_cross_process_or_fleet_shutdown_targets():
    source = (ROOT / 'launch' / 'localization.launch.py').read_text()
    assert 'process A' not in source and 'robot1' not in source and 'robot2' not in source
    assert 'kill(' not in source and 'pkill' not in source
    assert 'fleet_localization.launch' not in source
    assert 'OnShutdown(on_shutdown=cleanup)' in source
    assert 'lock.release()' in source
    assert 'rviz_path.unlink(missing_ok=True)' in source


def test_runtime_resources_are_created_per_invocation():
    launch = (ROOT / 'launch' / 'localization.launch.py').read_text()
    renderer = (ROOT / 'fleet_localization' / 'operator_config.py').read_text()
    assert 'MapStoreLock.acquire_shared(manifest.map_store)' in launch
    assert 'MapStoreLock.acquire_exclusive(manifest.map_store)' in launch
    assert "render_rviz(" in launch
    assert 'NamedTemporaryFile' in renderer and 'delete=False' in renderer
    assert "prefix=f'{robot}_localization_'" in renderer
    assert '/tmp/robot_localization.rviz' not in renderer
