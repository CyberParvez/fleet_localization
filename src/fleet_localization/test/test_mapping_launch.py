from pathlib import Path


ROOT=Path(__file__).parents[1]


def test_mapping_is_exclusive_and_readiness_gated_without_localizers():
    source=(ROOT/'launch'/'localization.launch.py').read_text()
    assert 'MapStoreLock.acquire_exclusive(manifest.map_store)' in source
    assert source.index('MapStoreLock.acquire_exclusive') < source.index("executable='readiness'")
    assert "[ekf,slam,slam_lifecycle,health]" in source
    assert "[ekf,map_server,amcl,lifecycle,health] if mode == 'localization'" in source
    assert "'node_names':['slam_toolbox']" in source
    assert "executable='async_slam_toolbox_node'" in source


def test_save_interface_has_no_path_force_or_shutdown_save():
    source=(ROOT/'fleet_localization'/'map_save.py').read_text()
    assert "add_argument('--fleet-config', required=True)" in source
    assert "add_argument('--robot', required=True)" in source
    assert "add_argument('--map-id', required=True)" in source
    assert '--output' not in source and '--force' not in source
    assert 'localization_health' in source and "['mode','fleet_config','robot']" in source
    launch=(ROOT/'launch'/'localization.launch.py').read_text()
    assert 'map_save' not in launch
